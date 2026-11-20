/* app_balance.h - three-loop balance controller
 *
 * ============================================================================
 *  CONTROL LOGIC: 1:1 PORT OF THE JIANGKEDA TUTORIAL (PROJECT 06)
 * ============================================================================
 *  Reference files:
 *    User/PID.c    : PID_t / PID_Update()  - positional PID, Ki!=0 guard,
 *                    integral clamp, derivative-on-measurement, OutOffset,
 *                    output clamp
 *    User/main.c   : 10 ms angle loop, 50 ms speed/turn loops, complementary
 *                    filter, encoder -> rev/s, cascade structure
 *    Hardware/MPU6050.c : CONFIG=0, SMPLRT_DIV=7, ACCEL_CONFIG=0x18
 *
 *  Only FreeRTOS (instead of a 1 ms TIM ISR), the telemetry protocol and the
 *  CAL auto-zero differ from the tutorial. Gains and ALL PID coefficients are
 *  the tutorial's original values.
 *
 *  --- Important scaling note -----------------------------------------------
 *  The tutorial drives PWM in the range -100..+100 ("percent duty"). This
 *  project drives TB6612 in -7200..+7200. Every PID still runs natively in the
 *  tutorial's percent domain; only at the very end the output is multiplied by
 *  APP_BAL_PWM_SCALE (72) to become real duty. That is why
 *  AnglePID.OutOffset = 3 becomes 216 counts, exactly the deadband this car
 *  already needed - not a coincidence, the same physical idea.
 *
 *  --- Loop structure (identical to the tutorial) ---------------------------
 *    every 10 ms:
 *      read MPU -> GY -= bias -> AngleAcc = -atan2(AX,AZ)*180/pi
 *      Angle = 0.01*AngleAcc + 0.99*(Angle + gy_dps*0.01)
 *      AnglePID.Actual = Angle (already zeroed by CAL)
 *      PID_Update(&AnglePID)
 *      AvePWM = -AnglePID.Out * 72
 *      LeftPWM = AvePWM + DifPWM/2 ; RightPWM = AvePWM - DifPWM/2
 *    every 50 ms:
 *      LeftSpeed  = pulsesL / 44 / 0.05 / 9.27666   (motor shaft rev/s)
 *      RightSpeed = pulsesR / 44 / 0.05 / 9.27666
 *      AveSpeed = (L+R)/2 ; DifSpeed = L-R
 *      SpeedPID.Actual = AveSpeed ; PID_Update
 *      AnglePID.Target = SpeedPID.Out               (cascade: deg)
 *      TurnPID.Actual  = DifSpeed ; PID_Update
 *      DifPWM = TurnPID.Out * 72
 *
 *  --- Commands (over HC-05, LF terminated) ---------------------------------
 *    CAL          hold upright & still; capture the centre angle and ZERO it
 *    BAL,1 / 0    enable / disable closed loop (refuses if tilted too far)
 *    MOV,fwd,turn remote drive in percent (-100..100): fwd + = forward,
 *                 turn + = yaw right. Deadman: re-send within 600 ms
 *                 (the PC does it at 10 Hz) or the car ramps back to a stop.
 *    SET,kp,ki,kd angle gains, x100 (3.30 -> 330, 0.15 -> 15, 3.00 -> 300)
 *    SETV,kp,ki   speed gains, x10000 (0.80 -> 8000, 0.065 -> 650)
 *    SETT,kp,ki   turn gains, x100 (3.00 -> 300, 4.00 -> 400)
 *    GET/GETV/GETT read back the three gain groups
 *    BST / T      status / telemetry  (see app_pid_tune.h)
 *
 *  Everything else is fixed in the constants below, not tunable at runtime:
 *  angle trim = 0, output sign = APP_BAL_OUTPUT_SIGN, 50 deg protection = on,
 *  deadband = APP_BAL_ANGLE_OFFSET (3 percent = 216 counts), motor polarity /
 *  encoder direction / wheel trim = macro defaults.
 */
#ifndef __APP_BALANCE_H
#define __APP_BALANCE_H

#include <stdint.h>
#include "bsp_mpu6050.h"

/* ==========================================================================
 *  Tutorial PID defaults - copied verbatim from the reference main.c
 * ========================================================================== */

/* --- Angle loop: tuned on-car default, out +/-100, offset 3 --------------- *
 * Gains are stored x100 so the existing integer wire protocol is unchanged.
 * These replace the tutorial's 5/0.1/5 with the user's best on-car values. */
#define APP_BAL_DEFAULT_KP100   330L     /* 3.30 */
#define APP_BAL_DEFAULT_KI100   15L      /* 0.15 */
#define APP_BAL_DEFAULT_KD100   300L     /* 3.00 */
#define APP_BAL_ANGLE_OUTMAX    100.0f
#define APP_BAL_ANGLE_OUTMIN    (-100.0f)
#define APP_BAL_ANGLE_OFFSET    3.0f     /* tutorial OutOffset (percent) */
#define APP_BAL_ANGLE_I_MAX     600.0f   /* tutorial ErrorIntMax */
#define APP_BAL_ANGLE_I_MIN     (-600.0f)

/* --- Speed loop: tuned on-car default, Kd=0, out +/-20 deg, int +/-150 ---- */
#define APP_BAL_SPEED_KP10000   8000L    /* 0.80 */
#define APP_BAL_SPEED_KI10000   650L     /* 0.065 */
#define APP_BAL_SPEED_OUTMAX    20.0f    /* deg of lean, tutorial OutMax */
#define APP_BAL_SPEED_OUTMIN    (-20.0f)
#define APP_BAL_SPEED_I_MAX     150.0f
#define APP_BAL_SPEED_I_MIN     (-150.0f)
/* Speed target comes from the PC (MOV) and defaults to 0 (hold position). */

/* --- Turn loop: tuned on-car default (3.00/4.00), Kd=0, out +/-50 --------- *
 * NOTE: the tutorial's turn loop is encoder-difference PI. The old gyro-Z
 * rate damping has been REMOVED; SETT now carries (Kp, Ki), not (Kp, Kd). */
#define APP_BAL_TURN_KP100      300L     /* 3.00 */
#define APP_BAL_TURN_KI100      400L     /* 4.00 */
#define APP_BAL_TURN_OUTMAX     50.0f
#define APP_BAL_TURN_OUTMIN     (-50.0f)
#define APP_BAL_TURN_I_MAX      20.0f
#define APP_BAL_TURN_I_MIN      (-20.0f)

/* --- Motion control: remote forward / backward / spin --------------------- *
 * The PC sends MOV,<fwd>,<turn> in PERCENT (-100..100). Percent maps to the
 * two loop targets like this:
 *     speed target = fwd  % x APP_BAL_SPEED_PER_PCT   [rev/s, 100 % -> 5.0]
 *     dif   target = turn % x APP_BAL_DIF_PER_PCT     [rev/s, 100 % -> 2.0]
 * The speed loop's Actual is the average wheel speed, so the target is a real
 * velocity: the car leans, accelerates and then holds that speed. The turn
 * loop's Actual is the wheel-speed DIFFERENCE, so a positive target spins the
 * car (left wheel faster -> yaw to the right).
 * With a 65 mm wheel 5.0 rev/s is about 1 m/s - a sane indoor top speed.
 *
 * Both targets are slew limited (APP_BAL_TARGET_SLEW per 50 ms tick) so a
 * full-scale key press ramps in over ~0.5 s instead of stepping the angle
 * target straight onto the 20 deg clamp.
 *
 * APP_BAL_MOVE_TIMEOUT_MS is a DEADMAN: the PC repeats MOV at 10 Hz, so if
 * the Bluetooth link dies the targets ramp back to zero by themselves and the
 * car simply stops instead of driving away. */
#define APP_BAL_SPEED_PER_PCT   0.05f    /* 100 % = 5.0 rev/s  (~1 m/s) */
#define APP_BAL_DIF_PER_PCT     0.02f    /* 100 % = 2.0 rev/s difference */
#define APP_BAL_TARGET_SLEW     0.5f     /* rev/s per 50 ms tick */
#define APP_BAL_MOVE_TIMEOUT_MS 600U     /* no MOV for this long -> stop */

/* Flip either sign if the car drives the wrong way / turns the wrong way.
 * Both are one-symbol fixes instead of rewiring the motors. */
#define APP_BAL_MOVE_FWD_SIGN   (1.0f)   /* +1: W drives forward */
#define APP_BAL_MOVE_TURN_SIGN  (1.0f)   /* +1: D turns right */

/* --- Encoder -> wheel speed, tutorial constants --------------------------- *
 * LeftSpeed = Encoder_Get(1) / 44.0 / 0.05 / 9.27666;
 * the /9.27666 turns motor-shaft rev/s into WHEEL rev/s (gearbox). */
#define APP_BAL_ENC_PER_REV     44.0f
#define APP_BAL_ENC_WINDOW_S    0.05f
#define APP_BAL_GEAR_RATIO      9.27666f

/* --- Timing ---------------------------------------------------------------- *
 * FreeRTOS balance task stays at 5 ms. The controller runs every 2nd tick
 * (10 ms) and the two slow loops every 10th tick (50 ms), matching main.c. */
#define APP_BAL_TICK_MS         5U
#define APP_BAL_CTRL_DIV        2U       /* 5 ms x 2  = 10 ms angle loop */
#define APP_BAL_SLOW_DIV        10U      /* 5 ms x 10 = 50 ms speed/turn */
#define APP_BAL_DT              0.01f    /* seconds, used by the filter */

/* --- Output ---------------------------------------------------------------- */
/* percent-domain PID output -> real duty. 7200 / 100 = 72. */
#define APP_BAL_PWM_LIMIT       7200
#define APP_BAL_PWM_SCALE       72.0f

/* Hardware build switch: TB6612 motor driver is fitted. CAL/BAL/BST are
 * only meaningful (and only exposed over the link) when it is present. */
#ifndef APP_ENABLE_TB6612
#define APP_ENABLE_TB6612       1U
#endif

/* Direction: measured on this car: its motor wiring is opposite to the
 * reference, so outputs are inverted by default. Same as SIGN,-1 over BLE.
 * The tutorial's AvePWM = -AnglePID.Out is already included separately. */
#define APP_BAL_OUTPUT_SIGN     (-1.0f)

/* --- Attitude -------------------------------------------------------------- *
 * Tutorial at 10 ms: Alpha = 0.01 -> A = 1 - Alpha = 0.99 gyro weight. */
#define APP_BAL_COMP_A          0.99f
#define APP_BAL_COMP_ALPHA      0.01f
#define APP_BAL_CAL_SAMPLES     100U     /* 100 x 10 ms = 1 s of still data */
#define APP_BAL_CAL_STILL_GYRO  5.0f     /* dps; above this CAL restarts */
#define APP_BAL_ARM_MAX_DEG     12.0f    /* refuse BAL,1 if tilted more */
#define APP_BAL_FAULT_DEG       50.0f    /* tutorial: |Angle| > 50 -> stop */

/* ==========================================================================
 *  Types
 * ========================================================================== */

/* Exact copy of the tutorial's PID_t plus the tuning-only coefficients. */
typedef struct
{
  float Target;
  float Actual;
  float Actual1;
  float Out;
  float Error0;
  float Error1;
  float ErrorInt;
  float Kp;
  float Ki;
  float Kd;
  float OutMax;
  float OutMin;
  float OutOffset;
  float ErrorIntMax;
  float ErrorIntMin;
} AppBalance_PID_t;

typedef enum
{
  APP_BAL_STATE_IDLE = 0U,   /* power-up default: motors coast */
  APP_BAL_STATE_CAL,         /* capturing centre angle, keep car still */
  APP_BAL_STATE_RUN,         /* closed loop active */
  APP_BAL_STATE_FAULT        /* tilt too large / IMU lost: motors coast */
} AppBalance_State_t;

typedef struct
{
  float angle;      /* fused pitch AFTER CAL zeroing, deg (angle loop Actual) */
  float angle_acc;  /* raw accelerometer pitch, deg (before zeroing) */
  float gyro_y_dps; /* gyro Y rate after bias removal, dps */
  float offset;     /* centre angle captured by CAL, deg */
  int16_t duty;     /* applied shared PWM, -7200..7200 */
  int32_t p100;     /* angle-loop P term, x100 (percent domain) */
  int32_t i100;     /* angle-loop I term, x100 */
  int32_t d100;     /* angle-loop D term, x100 */
  int32_t vel100;   /* speed-loop output = angle target, x100 deg */
  int32_t turn;     /* turn-loop output in real PWM counts */
  int32_t enc_tps;  /* left+right encoder rate, pulses/s (display only) */
  int32_t gyro_z100;/* yaw rate, deg/s x100 */
  int32_t spd100;   /* measured average wheel speed, rev/s x100 */
  int32_t move_fwd; /* commanded drive, percent -100..100 (last MOV) */
  int32_t move_turn;/* commanded yaw, percent -100..100 (last MOV) */
  uint8_t state;    /* AppBalance_State_t */
  uint8_t imu_ok;
  uint8_t cal_progress; /* 0..100 percent during CAL */
} AppBalance_Status_t;

/* ==========================================================================
 *  API (unchanged where possible so the protocol layer keeps compiling)
 * ========================================================================== */

void    App_Balance_Init(void);
void    App_Balance_Tick(uint32_t now_ms);   /* called every 5 ms by FreeRTOS */

uint8_t App_Balance_Calibrate(void);
/* 0 = ok, otherwise: 1 = IMU not ready, 2 = CAL still running,
 * 3 = |angle| > APP_BAL_ARM_MAX_DEG (car not level). The protocol layer
 * turns these into distinct LOG lines so the operator knows what to fix. */
uint8_t App_Balance_Enable(uint8_t on);

/* Angle loop gains, x100. */
void    App_Balance_SetGains(int32_t kp100, int32_t ki100, int32_t kd100);
void    App_Balance_GetGains(int32_t *kp100, int32_t *ki100, int32_t *kd100);
/* Speed loop gains, x10000. */
void    App_Balance_SetSpeedGains(int32_t kp10000, int32_t ki10000);
void    App_Balance_GetSpeedGains(int32_t *kp10000, int32_t *ki10000);
/* Turn loop gains, x100. Was (Kp,Kd); now the tutorial's (Kp,Ki). */
void    App_Balance_SetTurnGains(int32_t kp100, int32_t ki100);
void    App_Balance_GetTurnGains(int32_t *kp100, int32_t *ki100);

/* Reset every PID state (Target/Actual/Error/Integral/Out), not the gains. */
void    App_Balance_ResetIntegrals(void);

/* Remote drive: fwd/turn in PERCENT (-100..100), now_ms from the caller's
 * tick. Re-send at least every APP_BAL_MOVE_TIMEOUT_MS or the car stops. */
void    App_Balance_SetMove(int32_t fwd_pct, int32_t turn_pct, uint32_t now_ms);
void    App_Balance_GetMove(int32_t *fwd_pct, int32_t *turn_pct);

void    App_Balance_GetStatus(AppBalance_Status_t *out);
void    App_Balance_GetImu(MPU6050_Data_t *out);
uint8_t App_Balance_IsImuOk(void);
float   App_Balance_GetAngle(void);
float   App_Balance_GetOffset(void);

#endif /* __APP_BALANCE_H */
