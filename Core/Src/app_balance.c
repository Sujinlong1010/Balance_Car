/* app_balance.c - three-loop balance controller
 *
 * 1:1 port of the JiangKeDa tutorial (project 06, balance car optimised):
 *   User/PID.c    -> PID_Update() below (positional PID, derivative on
 *                    measurement, Ki!=0 guard, integral/output clamps)
 *   User/main.c   -> 10 ms angle loop, 50 ms speed + turn loops, 0.01/0.99
 *                    complementary filter, encoder -> rev/s, cascade
 *
 * Differences forced by this project (everything else is identical):
 *   - FreeRTOS 5 ms task + software-divider instead of a 1 ms TIM ISR
 *   - telemetry / command protocol (app_pid_tune.c)
 * - CAL captures the centre angle automatically and zeroes it (the tutorial
 *   asks the user to hand-tune a centre offset in main.c instead)
 *   - percent-domain PID output x72 -> TB6612 duty (tutorial uses +/-100)
 */
#include "app_balance.h"
#include "app_motor_test.h"
#include "bsp_encoder.h"
#include "bsp_hc05.h"
#include "bsp_delay.h"
#include <math.h>
#include <stdio.h>

static MPU6050_Data_t     s_imu;
static uint8_t            s_imu_ok = 0U;
static AppBalance_State_t s_state = APP_BAL_STATE_IDLE;

/* The three tutorial PID objects. Gains live inside them so SET/SETV/SETT
 * take effect on the very next PID_Update() - no reflash, no mode switch. */
static AppBalance_PID_t s_angle_pid;
static AppBalance_PID_t s_speed_pid;
static AppBalance_PID_t s_turn_pid;

static float   s_angle_raw = 0.0f;   /* fused angle before CAL zeroing, deg */
static float   s_angle = 0.0f;       /* s_angle_raw - s_offset, deg */
static float   s_angle_acc = 0.0f;   /* accelerometer tilt, deg */
static float   s_gyro_y = 0.0f;      /* bias-corrected gyro Y, dps */
static float   s_offset = 0.0f;      /* centre angle captured by CAL, deg */
static int16_t s_duty = 0;           /* applied shared PWM, -7200..7200 */

static int32_t s_p100 = 0;
static int32_t s_i100 = 0;
static int32_t s_d100 = 0;
static int32_t s_vel100 = 0;         /* speed-loop output = target lean, x100 deg */
static int32_t s_turn = 0;           /* turn-loop output in real PWM counts */
static int32_t s_enc_tps = 0;        /* display only */

/* Fixed operating point (not tunable over the air): angle trim = 0, output
 * sign and the 50 deg cut-off come from the constants in app_balance.h. */

/* Remote drive state. The PC sends MOV,fwd,turn in percent; those become the
 * two slow-loop targets, but slew limited so a key press cannot step the
 * angle target onto its clamp. s_move_seen/s_move_ms implement the deadman. */
static int32_t  s_move_fwd = 0;      /* commanded, percent */
static int32_t  s_move_turn = 0;     /* commanded, percent */
static float    s_tgt_speed = 0.0f;  /* commanded speed target, rev/s */
static float    s_tgt_dif = 0.0f;    /* commanded difference target, rev/s */
static float    s_ramp_speed = 0.0f; /* slew-limited, fed to SpeedPID.Target */
static float    s_ramp_dif = 0.0f;   /* slew-limited, fed to TurnPID.Target */
static uint32_t s_move_ms = 0U;
static uint8_t  s_move_seen = 0U;
static int32_t  s_spd100 = 0;        /* measured average speed, x100 rev/s */

/* Held outputs from the 50 ms loops, consumed by the 10 ms loop. */
static float s_speed_out_hold = 0.0f;    /* deg, = SpeedPID.Out */
static int   s_dif_pwm = 0;              /* real counts, = TurnPID.Out x 72 */

/* Encoder pulse accumulator for one 50 ms window (sampled at 5 ms). */
static int32_t s_enc_acc_l = 0L;
static int32_t s_enc_acc_r = 0L;

/* Period dividers: 5 ms task -> 10 ms controller, 50 ms slow loops. */
static uint8_t s_ctrl_div = 0U;
static uint8_t s_slow_div = 0U;

static uint32_t s_last_us = 0U;
static uint32_t s_last_imu_retry_ms = 0U;
static uint8_t  s_filter_seeded = 0U;
static uint16_t s_cal_sum_n = 0U;
static float    s_cal_sum = 0.0f;
static char     s_tx[96];

static void LogLine(const char *text)
{
  (void)HC05_SendString(text);
  (void)HC05_SendString("\n");
}

static float ClampF(float v, float lo, float hi)
{
  if (v < lo) { return lo; }
  if (v > hi) { return hi; }
  return v;
}

/* Move cur toward tgt by at most step. Used for the drive targets so that a
 * full-scale key press becomes a ~0.5 s ramp instead of a step. */
static float SlewF(float cur, float tgt, float step)
{
  if (tgt > (cur + step)) { return cur + step; }
  if (tgt < (cur - step)) { return cur - step; }
  return tgt;
}

/* ==========================================================================
 *  PID_t / PID_Update - EXACT copy of User/PID.c from the tutorial
 * ========================================================================== */

static void PID_Reset(AppBalance_PID_t *p)
{
  /* Keeps the gains; clears only the state, exactly like the tutorial's
   * PID_Init() which is called on every start. */
  p->Target = 0.0f;
  p->Actual = 0.0f;
  p->Actual1 = 0.0f;
  p->Out = 0.0f;
  p->Error0 = 0.0f;
  p->Error1 = 0.0f;
  p->ErrorInt = 0.0f;
}

static void PID_Update(AppBalance_PID_t *p)
{
  p->Error1 = p->Error0;
  p->Error0 = p->Target - p->Actual;

  /* Only integrate when Ki != 0, otherwise the integral could wind up to
   * the clamp while tuning and slam the car the moment Ki is raised. */
  if (p->Ki != 0.0f)
  {
    p->ErrorInt += p->Error0;
    if (p->ErrorInt > p->ErrorIntMax) { p->ErrorInt = p->ErrorIntMax; }
    if (p->ErrorInt < p->ErrorIntMin) { p->ErrorInt = p->ErrorIntMin; }
  }
  else
  {
    p->ErrorInt = 0.0f;
  }

  /* Positional PID with derivative on measurement (not on error). */
  p->Out = (p->Kp * p->Error0) +
           (p->Ki * p->ErrorInt) -
           (p->Kd * (p->Actual - p->Actual1));

  /* OutOffset: push a geared motor past its dead band whenever it is told
   * to move at all. Zero output stays zero. */
  if (p->Out > 0.0f) { p->Out += p->OutOffset; }
  if (p->Out < 0.0f) { p->Out -= p->OutOffset; }

  if (p->Out > p->OutMax) { p->Out = p->OutMax; }
  if (p->Out < p->OutMin) { p->Out = p->OutMin; }

  p->Actual1 = p->Actual;
}

static void ConfigurePIDs(void)
{
  s_angle_pid.Kp = (float)APP_BAL_DEFAULT_KP100 / 100.0f;
  s_angle_pid.Ki = (float)APP_BAL_DEFAULT_KI100 / 100.0f;
  s_angle_pid.Kd = (float)APP_BAL_DEFAULT_KD100 / 100.0f;
  s_angle_pid.OutMax = APP_BAL_ANGLE_OUTMAX;
  s_angle_pid.OutMin = APP_BAL_ANGLE_OUTMIN;
  s_angle_pid.OutOffset = APP_BAL_ANGLE_OFFSET;
  s_angle_pid.ErrorIntMax = APP_BAL_ANGLE_I_MAX;
  s_angle_pid.ErrorIntMin = APP_BAL_ANGLE_I_MIN;

  s_speed_pid.Kp = (float)APP_BAL_SPEED_KP10000 / 10000.0f;
  s_speed_pid.Ki = (float)APP_BAL_SPEED_KI10000 / 10000.0f;
  s_speed_pid.Kd = 0.0f;
  s_speed_pid.OutMax = APP_BAL_SPEED_OUTMAX;
  s_speed_pid.OutMin = APP_BAL_SPEED_OUTMIN;
  s_speed_pid.ErrorIntMax = APP_BAL_SPEED_I_MAX;
  s_speed_pid.ErrorIntMin = APP_BAL_SPEED_I_MIN;

  s_turn_pid.Kp = (float)APP_BAL_TURN_KP100 / 100.0f;
  s_turn_pid.Ki = (float)APP_BAL_TURN_KI100 / 100.0f;
  s_turn_pid.Kd = 0.0f;
  s_turn_pid.OutMax = APP_BAL_TURN_OUTMAX;
  s_turn_pid.OutMin = APP_BAL_TURN_OUTMIN;
  s_turn_pid.ErrorIntMax = APP_BAL_TURN_I_MAX;
  s_turn_pid.ErrorIntMin = APP_BAL_TURN_I_MIN;
}

/* ==========================================================================
 *  Attitude (tutorial 10 ms block, with CAL zeroing instead of "+= 0.5")
 * ========================================================================== */

static void UpdateAttitude(void)
{
  uint32_t now_us;
  float dt;
  float angle_gyro;

  if ((s_imu_ok == 0U) || (MPU6050_ReadData(&s_imu) != 0U))
  {
    s_imu_ok = 0U;
    return;
  }
  s_imu_ok = 1U;

  /* Tutorial: GY -= 16 (a fixed zero offset). Ours is measured at boot and
   * already removed in MPU6050_ReadData(); gy_dps is the corrected value. */
  s_angle_acc = s_imu.pitch;
  s_gyro_y = s_imu.gy_dps;

  now_us = BSP_GetMicros();
  dt = (float)(uint32_t)(now_us - s_last_us) / 1000000.0f;
  s_last_us = now_us;
  if ((dt <= 0.0f) || (dt > 0.05f)) { dt = APP_BAL_DT; }

  if (s_filter_seeded == 0U)
  {
    s_angle_raw = s_angle_acc;
    s_filter_seeded = 1U;
  }
  else
  {
    /* AngleGyro = Angle + GY / 32768.0 * 2000 * 0.01 = Angle + gy_dps*dt */
    angle_gyro = s_angle_raw + (s_gyro_y * dt);
    /* Alpha = 0.01, i.e. 1 % accelerometer / 99 % gyro. */
    s_angle_raw = (APP_BAL_COMP_ALPHA * s_angle_acc) +
                  (APP_BAL_COMP_A * angle_gyro);
  }

  /* CAL makes s_offset equal the tilt when the car is mechanically upright,
   * so the value fed to the angle loop is zero at that instant. */
  s_angle = s_angle_raw - s_offset;
}

/* ==========================================================================
 *  50 ms loops: encoder speed -> SpeedPID -> angle target, DifSpeed -> TurnPID
 * ========================================================================== */

static void RunSlowLoops(uint32_t now_ms)
{
  float scale;
  float left_rps;
  float right_rps;
  float ave_speed;
  float dif_speed;

  /* Encoder_Get(n)/44/0.05/9.27666 = wheel revolutions per second. */
  scale = APP_BAL_ENC_PER_REV * APP_BAL_ENC_WINDOW_S * APP_BAL_GEAR_RATIO;
  left_rps  = (float)s_enc_acc_l / scale;
  right_rps = (float)s_enc_acc_r / scale;
  s_enc_acc_l = 0L;
  s_enc_acc_r = 0L;

  /* Display: pulses per second over the 50 ms window. */
  s_enc_tps = (int32_t)((float)(left_rps + right_rps) *
                        APP_BAL_GEAR_RATIO * (1.0f / APP_BAL_ENC_WINDOW_S));
  s_spd100 = (int32_t)((left_rps + right_rps) * 0.5f * 100.0f);

  /* Deadman: the PC re-sends MOV at 10 Hz. If it stops arriving (Bluetooth
   * dropped, app closed) ramp the targets down instead of driving away. The
   * reported command also goes to 0 so the PC shows what the car is doing. */
  if ((s_move_seen != 0U) &&
      ((uint32_t)(now_ms - s_move_ms) > (uint32_t)APP_BAL_MOVE_TIMEOUT_MS))
  {
    s_tgt_speed = 0.0f;
    s_tgt_dif   = 0.0f;
    s_move_fwd  = 0;
    s_move_turn = 0;
  }
  s_ramp_speed = SlewF(s_ramp_speed, s_tgt_speed, APP_BAL_TARGET_SLEW);
  s_ramp_dif   = SlewF(s_ramp_dif,   s_tgt_dif,   APP_BAL_TARGET_SLEW);

  if (s_state != APP_BAL_STATE_RUN)
  {
    s_speed_out_hold = 0.0f;
    s_dif_pwm = 0;
    return;
  }

  ave_speed = (left_rps + right_rps) * 0.5f;
  dif_speed = left_rps - right_rps;

  /* Speed loop: target is the ramped MOV speed (0 = hold position). Its
   * output is an ANGLE in degrees, cascaded into the angle loop. */
  s_speed_pid.Target = s_ramp_speed;
  s_speed_pid.Actual = ave_speed;
  PID_Update(&s_speed_pid);
  s_speed_out_hold = s_speed_pid.Out;

  /* Turn loop is encoder DIFFERENCE PI; target is the ramped MOV yaw. */
  s_turn_pid.Target = s_ramp_dif;
  s_turn_pid.Actual = dif_speed;
  PID_Update(&s_turn_pid);
  s_dif_pwm = (int)(s_turn_pid.Out * APP_BAL_PWM_SCALE);
}

/* ==========================================================================
 *  10 ms angle loop: PID -> AvePWM -> left/right PWM -> motor
 * ========================================================================== */

static void RunAngleLoop(void)
{
      float ave_pwm;
      float left_pwm;
      float right_pwm;
      float d_prev_actual;   /* Actual1 BEFORE PID_Update overwrites it */

  /* Tutorial: |Angle| > 50 deg -> RunFlag = 0 (motors off). Always on. */
  if (fabsf(s_angle) > APP_BAL_FAULT_DEG)
  {
    s_state = APP_BAL_STATE_FAULT;
    s_duty = 0;
    s_vel100 = 0;
    s_turn = 0;
    App_Balance_ResetIntegrals();
    App_MotorTest_Stop();
    LogLine("LOG,FAULT |angle| > 50 deg - motors stopped. BAL,0 then re-CAL");
    return;
  }

  /* Cascade setpoint from the 50 ms speed loop; operator trim is fixed 0. */
  s_angle_pid.Target = s_speed_out_hold;
  s_angle_pid.Actual = s_angle;
  /* Snapshot the previous sample: PID_Update() sets Actual1 = Actual at its
   * end, so reading Actual1 afterwards always gives zero for the D term. */
  d_prev_actual = s_angle_pid.Actual1;
  PID_Update(&s_angle_pid);

  /* Tutorial: AvePWM = -AnglePID.Out. Here also x72 -> real duty. */
  ave_pwm = -s_angle_pid.Out * APP_BAL_PWM_SCALE;
  left_pwm = ave_pwm + ((float)s_dif_pwm * 0.5f);
  right_pwm = ave_pwm - ((float)s_dif_pwm * 0.5f);

  if (left_pwm > (float)APP_BAL_PWM_LIMIT)  { left_pwm = (float)APP_BAL_PWM_LIMIT; }
  if (left_pwm < (float)-APP_BAL_PWM_LIMIT) { left_pwm = (float)-APP_BAL_PWM_LIMIT; }
  if (right_pwm > (float)APP_BAL_PWM_LIMIT)  { right_pwm = (float)APP_BAL_PWM_LIMIT; }
  if (right_pwm < (float)-APP_BAL_PWM_LIMIT) { right_pwm = (float)-APP_BAL_PWM_LIMIT; }

  /* Telemetry: convert the percent-domain terms to equivalent PWM counts so
   * P + I + D still adds up to the plotted PWM (OutOffset deliberately left
   * out, as before). Reported in the applied direction after SIGN. */
  s_p100 = (int32_t)(s_angle_pid.Kp * s_angle_pid.Error0 *
                     APP_BAL_PWM_SCALE * 100.0f);
  s_i100 = (int32_t)(s_angle_pid.Ki * s_angle_pid.ErrorInt *
                     APP_BAL_PWM_SCALE * 100.0f);
  /* D term must use the derivative-on-measurement delta captured before
   * PID_Update() shifted Actual -> Actual1, otherwise telemetry shows 0. */
  s_d100 = (int32_t)(-s_angle_pid.Kd *
                     (s_angle_pid.Actual - d_prev_actual) *
                     APP_BAL_PWM_SCALE * 100.0f);
  s_vel100 = (int32_t)(s_speed_out_hold * 100.0f);
  s_turn = s_dif_pwm;
  s_duty = (int16_t)(ave_pwm * APP_BAL_OUTPUT_SIGN);

  App_MotorTest_DriveLR((int16_t)(left_pwm * APP_BAL_OUTPUT_SIGN),
                        (int16_t)(right_pwm * APP_BAL_OUTPUT_SIGN));
}

/* ==========================================================================
 *  Application glue
 * ========================================================================== */

void App_Balance_ResetIntegrals(void)
{
  PID_Reset(&s_angle_pid);
  PID_Reset(&s_speed_pid);
  PID_Reset(&s_turn_pid);
  s_enc_acc_l = 0L;
  s_enc_acc_r = 0L;
  s_speed_out_hold = 0.0f;
  s_dif_pwm = 0;
  /* Drop the ramp but keep the commanded MOV targets: the deadman (or the
   * PC's next MOV) is what decides whether a command is still wanted. A
   * stale command has already been zeroed by the 600 ms watchdog. */
  s_ramp_speed = 0.0f;
  s_ramp_dif = 0.0f;
  /* Deliberately NOT resetting s_slow_div: it is a scheduler divider, not
   * controller state. Clearing it every idle tick would starve the 50 ms
   * loop and freeze the encoder-speed readout. */
}

void App_Balance_Init(void)
{
  App_MotorTest_Init();          /* TB6612 init + coast, no auto-start */
  ConfigurePIDs();

  s_imu_ok = (MPU6050_Init(MPU6050_SAMPLE_DIV_TUTORIAL) == 0U) ? 1U : 0U;
  if (s_imu_ok == 0U)
  {
    LogLine("LOG,ERR MPU6050 init failed - retrying every 0.5 s");
  }
  s_last_imu_retry_ms = 0U;
  s_state  = APP_BAL_STATE_IDLE;
  s_angle  = 0.0f;
  s_angle_raw = 0.0f;
  s_offset = 0.0f;
  s_duty   = 0;
  s_p100 = 0;
  s_i100 = 0;
  s_d100 = 0;
  s_vel100 = 0;
  s_turn = 0;
  s_enc_tps = 0;
  s_spd100 = 0;
  s_ctrl_div = 0U;
  App_Balance_ResetIntegrals();
  s_last_us = BSP_GetMicros();
  s_filter_seeded = 0U;
  s_cal_sum_n = 0U;
  s_cal_sum = 0.0f;

  LogLine("LOG,Balance ready (JK 3-loop) - motors coast. Hold upright, send CAL");
}

void App_Balance_Tick(uint32_t now_ms)
{
  /* Sample encoders on the 5 ms tick and accumulate a clean 50 ms window. */
  BSP_Encoder_Update(now_ms);
  s_enc_acc_l += (int32_t)BSP_Encoder_GetDelta(BSP_ENC_LEFT);
  s_enc_acc_r += (int32_t)BSP_Encoder_GetDelta(BSP_ENC_RIGHT);

  /* A single I2C glitch during boot used to leave the IMU dead forever. */
  if ((s_imu_ok == 0U) && (s_state != APP_BAL_STATE_RUN) &&
      ((uint32_t)(now_ms - s_last_imu_retry_ms) >= 500U))
  {
    s_last_imu_retry_ms = now_ms;
    if (MPU6050_Init(MPU6050_SAMPLE_DIV_TUTORIAL) == 0U)
    {
      s_imu_ok = 1U;
      s_filter_seeded = 0U;
      LogLine("LOG,MPU6050 recovered");
    }
  }

  /* 5 ms -> 10 ms: everything the tutorial does in its 10 ms block. */
  s_ctrl_div++;
  if (s_ctrl_div < APP_BAL_CTRL_DIV) { return; }
  s_ctrl_div = 0U;

  UpdateAttitude();

  if (s_imu_ok == 0U)
  {
    if (s_state == APP_BAL_STATE_RUN) { s_state = APP_BAL_STATE_FAULT; }
    s_duty = 0;
    App_Balance_ResetIntegrals();
    App_MotorTest_Stop();
    return;
  }

  /* 10 ms -> 50 ms speed and turn loops. */
  s_slow_div++;
  if (s_slow_div >= APP_BAL_SLOW_DIV)
  {
    s_slow_div = 0U;
    RunSlowLoops(now_ms);
  }

  switch (s_state)
  {
    case APP_BAL_STATE_CAL:
      App_MotorTest_Stop();
      if (fabsf(s_gyro_y) > APP_BAL_CAL_STILL_GYRO)
      {
        s_cal_sum = 0.0f;
        s_cal_sum_n = 0U;
      }
      else
      {
        /* Capture the RAW fused angle; after assigning s_offset the exposed
         * s_angle becomes 0 immediately - this is the tutorial's centre-angle
         * calibration (upright must read zero), done automatically here. */
        s_cal_sum += s_angle_raw;
        s_cal_sum_n++;
        if (s_cal_sum_n >= APP_BAL_CAL_SAMPLES)
        {
          int32_t off100;
          int32_t mag;
          char sign;
          s_offset = s_cal_sum / (float)APP_BAL_CAL_SAMPLES;
          s_angle = s_angle_raw - s_offset;
          s_state = APP_BAL_STATE_IDLE;
          off100 = (int32_t)(s_offset * 100.0f);
          sign = (off100 < 0) ? '-' : '+';
          mag = (off100 < 0) ? -off100 : off100;
          (void)sprintf(s_tx, "LOG,Calibrated centre=%c%ld.%02ld deg (angle zeroed)",
                        sign, (long)(mag / 100), (long)(mag % 100));
          LogLine(s_tx);
        }
      }
      break;

    case APP_BAL_STATE_RUN:
      RunAngleLoop();
      break;

    case APP_BAL_STATE_FAULT:
    case APP_BAL_STATE_IDLE:
    default:
      s_duty = 0;
      s_vel100 = 0;
      s_turn = 0;
      App_Balance_ResetIntegrals();
      App_MotorTest_Stop();
      break;
  }
}

uint8_t App_Balance_Calibrate(void)
{
  if (s_imu_ok == 0U) { return 1U; }
  if (s_state == APP_BAL_STATE_RUN) { return 2U; }
  App_Balance_ResetIntegrals();
  s_cal_sum = 0.0f;
  s_cal_sum_n = 0U;
  s_state = APP_BAL_STATE_CAL;
  s_duty = 0;
  App_MotorTest_Stop();
  LogLine("LOG,CAL started - hold the car upright and still for 1 s");
  return 0U;
}

uint8_t App_Balance_Enable(uint8_t on)
{
  if (on == 0U)
  {
    s_state = APP_BAL_STATE_IDLE;
    s_duty = 0;
    s_vel100 = 0;
    s_turn = 0;
    App_Balance_ResetIntegrals();
    App_MotorTest_Stop();
    LogLine("LOG,Balance disabled - motors coast");
    return 0U;
  }

  if (s_imu_ok == 0U)            { return 1U; }
  if (s_state == APP_BAL_STATE_CAL) { return 2U; }
  if (fabsf(s_angle) > APP_BAL_ARM_MAX_DEG)
  {
    return 3U;
  }

  App_Balance_ResetIntegrals();
  s_state = APP_BAL_STATE_RUN;
  LogLine("LOG,Balance ENABLED (JK 3-loop) - hands off, send BAL,0 to stop");
  return 0U;
}

void App_Balance_SetGains(int32_t kp100, int32_t ki100, int32_t kd100)
{
  s_angle_pid.Kp = (float)ClampF((float)kp100, -1000000.0f, 1000000.0f) / 100.0f;
  s_angle_pid.Ki = (float)ClampF((float)ki100, -1000000.0f, 1000000.0f) / 100.0f;
  s_angle_pid.Kd = (float)ClampF((float)kd100, -1000000.0f, 1000000.0f) / 100.0f;
}

void App_Balance_GetGains(int32_t *kp100, int32_t *ki100, int32_t *kd100)
{
  if (kp100 != 0) { *kp100 = (int32_t)(s_angle_pid.Kp * 100.0f); }
  if (ki100 != 0) { *ki100 = (int32_t)(s_angle_pid.Ki * 100.0f); }
  if (kd100 != 0) { *kd100 = (int32_t)(s_angle_pid.Kd * 100.0f); }
}

void App_Balance_SetSpeedGains(int32_t kp10000, int32_t ki10000)
{
  s_speed_pid.Kp = (float)ClampF((float)kp10000, -1000000.0f, 1000000.0f) / 10000.0f;
  s_speed_pid.Ki = (float)ClampF((float)ki10000, -1000000.0f, 1000000.0f) / 10000.0f;
}

void App_Balance_GetSpeedGains(int32_t *kp10000, int32_t *ki10000)
{
  if (kp10000 != 0) { *kp10000 = (int32_t)(s_speed_pid.Kp * 10000.0f); }
  if (ki10000 != 0) { *ki10000 = (int32_t)(s_speed_pid.Ki * 10000.0f); }
}

void App_Balance_SetTurnGains(int32_t kp100, int32_t ki100)
{
  s_turn_pid.Kp = (float)ClampF((float)kp100, -1000000.0f, 1000000.0f) / 100.0f;
  s_turn_pid.Ki = (float)ClampF((float)ki100, -1000000.0f, 1000000.0f) / 100.0f;
}

void App_Balance_GetTurnGains(int32_t *kp100, int32_t *ki100)
{
  if (kp100 != 0) { *kp100 = (int32_t)(s_turn_pid.Kp * 100.0f); }
  if (ki100 != 0) { *ki100 = (int32_t)(s_turn_pid.Ki * 100.0f); }
}

void App_Balance_SetMove(int32_t fwd_pct, int32_t turn_pct, uint32_t now_ms)
{
  s_move_fwd  = (int32_t)ClampF((float)fwd_pct, -100.0f, 100.0f);
  s_move_turn = (int32_t)ClampF((float)turn_pct, -100.0f, 100.0f);

  s_tgt_speed = (float)s_move_fwd * APP_BAL_SPEED_PER_PCT * APP_BAL_MOVE_FWD_SIGN;
  s_tgt_dif   = (float)s_move_turn * APP_BAL_DIF_PER_PCT * APP_BAL_MOVE_TURN_SIGN;

  /* Refresh the deadman lease. */
  s_move_ms   = now_ms;
  s_move_seen = 1U;
}

void App_Balance_GetMove(int32_t *fwd_pct, int32_t *turn_pct)
{
  if (fwd_pct != 0)  { *fwd_pct = s_move_fwd; }
  if (turn_pct != 0) { *turn_pct = s_move_turn; }
}

void App_Balance_GetStatus(AppBalance_Status_t *out)
{
  if (out == 0) { return; }
  out->angle = s_angle;
  out->angle_acc = s_angle_acc;
  out->gyro_y_dps = s_gyro_y;
  out->offset = s_offset;
  out->duty = s_duty;
  out->p100 = s_p100;
  out->i100 = s_i100;
  out->d100 = s_d100;
  out->vel100 = s_vel100;
  out->turn = s_turn;
  out->enc_tps = s_enc_tps;
  out->gyro_z100 = (int32_t)(s_imu.gz_dps * 100.0f);
  out->spd100 = s_spd100;
  out->move_fwd = s_move_fwd;
  out->move_turn = s_move_turn;
  out->state = (uint8_t)s_state;
  out->imu_ok = s_imu_ok;
  if (s_state == APP_BAL_STATE_CAL)
  {
    out->cal_progress = (uint8_t)((100UL * (uint32_t)s_cal_sum_n) /
                                  (uint32_t)APP_BAL_CAL_SAMPLES);
  }
  else
  {
    out->cal_progress = 100U;
  }
}

void App_Balance_GetImu(MPU6050_Data_t *out)
{
  if (out != 0) { *out = s_imu; }
}

uint8_t App_Balance_IsImuOk(void)
{
  return s_imu_ok;
}

float App_Balance_GetAngle(void)
{
  return s_angle;
}

float App_Balance_GetOffset(void)
{
  return s_offset;
}
