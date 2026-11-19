/* app_pid_tune.c - HC-05 wireless PID tuning protocol implementation
 *
 * Design notes:
 *  - Only integer printf formats are used (Keil microlib has no float printf).
 *  - Command bytes come from the HC05 ring buffer one at a time, so a line
 *    that arrives split across several task cycles is still assembled.
 *  - The PID terms are computed in float internally, then scaled to x100
 *    integers for the wire. No motor output is written in this stage.
 *  - All static state lives in .bss, not on the 1 kB FreeRTOS task stack.
 */
#include "app_pid_tune.h"
#include "app_sensor_display.h"
#include "app_balance.h"
#include "app_motor_test.h"
#include "bsp_encoder.h"
#include "bsp_hc05.h"
#include <stdio.h>
#include <string.h>

static PIDTune_Params_t s_params;
static char             s_line[PIDTUNE_LINE_CAP];
static char             s_tx[PIDTUNE_TX_CAP];
static uint16_t         s_line_len = 0U;
static uint8_t          s_started = 0U;
static uint32_t         s_last_tx_ms = 0U;
static int32_t          s_angle100 = 0L;
static int32_t          s_tx_target100 = 0L;
static int32_t          s_p100 = 0L;
static int32_t          s_i100 = 0L;
static int32_t          s_d100 = 0L;
static int32_t          s_pwm100 = 0L;
static int32_t          s_vel100 = 0L;
static int32_t          s_turn = 0L;
static int32_t          s_enc_tps = 0L;
static int32_t          s_gyro_z100 = 0L;

static int32_t ClampInt32(int32_t v, int32_t lo, int32_t hi)
{
  if (v < lo) { return lo; }
  if (v > hi) { return hi; }
  return v;
}

/* Parse a signed decimal integer and advance *pp past it.
 * Deliberately caps the magnitude so val*10+digit cannot overflow int32_t. */
static uint8_t ParseInt32(const char **pp, int32_t *out)
{
  const char *p = *pp;
  int32_t val = 0L;
  int32_t sign = 1L;
  uint8_t digits = 0U;

  if (*p == '-') { sign = -1L; p++; }
  else if (*p == '+') { p++; }

  while ((*p >= '0') && (*p <= '9'))
  {
    if (val > 21474836L) { return 0U; }
    val = (val * 10L) + (int32_t)(*p - '0');
    p++;
    digits++;
  }
  if (digits == 0U) { return 0U; }

  *out = val * sign;
  *pp = p;
  return 1U;
}

static void SendLine(const char *text)
{
  (void)HC05_SendString(text);
  (void)HC05_SendString("\n");
}

static void SendAck(void)
{
  (void)sprintf(s_tx, "ACK,%ld,%ld,%ld\n",
                (long)s_params.kp100, (long)s_params.ki100,
                (long)s_params.kd100);
  (void)HC05_SendString(s_tx);
}

static void SendAckSpeed(void)
{
  (void)sprintf(s_tx, "ACKV,%ld,%ld\n",
                (long)s_params.kv_p10000, (long)s_params.kv_i10000);
  (void)HC05_SendString(s_tx);
}

static void SendAckTurn(void)
{
  (void)sprintf(s_tx, "ACKT,%ld,%ld\n",
                (long)s_params.kt_p100, (long)s_params.kt_i100);
  (void)HC05_SendString(s_tx);
}

/* Deadband compensation acknowledgement: ACKO,<counts> */
static void SendAckOffset(void)
{
  (void)sprintf(s_tx, "ACKO,%ld\n", (long)App_Balance_GetDeadband());
  (void)HC05_SendString(s_tx);
}

static void SendErr(const char *why)
{
  (void)sprintf(s_tx, "LOG,ERR %s\n", why);
  (void)HC05_SendString(s_tx);
}

static void HandleLine(char *line)
{
  const char *p;
  int32_t kp, ki, kd, val;

  if (strcmp(line, "PING") == 0)
  {
    SendLine("PONG");
    return;
  }
  if (strcmp(line, "GET") == 0)
  {
    SendAck();
    return;
  }
  if (strcmp(line, "YAWZERO") == 0)
  {
    MPU6050_ResetYaw();
    SendLine("LOG,YAW reset");
    return;
  }
  if (strcmp(line, "RSTI") == 0)
  {
    App_Balance_ResetIntegrals();
    SendLine("LOG,All integrals reset");
    return;
  }
  if (strcmp(line, "GETV") == 0)
  {
    App_Balance_GetSpeedGains(&s_params.kv_p10000, &s_params.kv_i10000);
    SendAckSpeed();
    return;
  }
  if (strcmp(line, "GETT") == 0)
  {
    App_Balance_GetTurnGains(&s_params.kt_p100, &s_params.kt_i100);
    SendAckTurn();
    return;
  }
  if (strcmp(line, "GETO") == 0)
  {
    SendAckOffset();
    return;
  }
  if (strcmp(line, "PERF") == 0)
  {
    uint32_t avg_us = 0U;
    uint32_t peak_us = 0U;

    App_Balance_GetLoopPerf(&avg_us, &peak_us);
    (void)sprintf(s_tx, "PERF,%lu,%lu\n",
                  (unsigned long)avg_us, (unsigned long)peak_us);
    (void)HC05_SendString(s_tx);
    return;
  }

  if (strncmp(line, "OFFSET,", 7U) == 0)
  {
    p = line + 7U;
    if (ParseInt32(&p, &val) == 0U) { SendErr("OFFSET value"); return; }
    if (*p != '\0')                 { SendErr("OFFSET tail"); return; }
    val = ClampInt32(val, 0L, PIDTUNE_OFFSET_LIMIT);
    App_Balance_SetDeadband(val);
    (void)sprintf(s_tx, "LOG,OFFSET=%ld counts (%ld%% duty)\n",
                  (long)val, (long)((val * 100L) / 7200L));
    (void)HC05_SendString(s_tx);
    SendAckOffset();
    return;
  }

  if (strncmp(line, "SET,", 4U) == 0)
  {
    p = line + 4U;
    if (ParseInt32(&p, &kp) == 0U) { SendErr("SET kp"); return; }
    if (*p != ',')                { SendErr("SET sep1"); return; }
    p++;
    if (ParseInt32(&p, &ki) == 0U) { SendErr("SET ki"); return; }
    if (*p != ',')                 { SendErr("SET sep2"); return; }
    p++;
    if (ParseInt32(&p, &kd) == 0U) { SendErr("SET kd"); return; }
    if (*p != '\0')                { SendErr("SET tail"); return; }

    s_params.kp100 = ClampInt32(kp, -PIDTUNE_PARAM_LIMIT, PIDTUNE_PARAM_LIMIT);
    s_params.ki100 = ClampInt32(ki, -PIDTUNE_PARAM_LIMIT, PIDTUNE_PARAM_LIMIT);
    s_params.kd100 = ClampInt32(kd, -PIDTUNE_PARAM_LIMIT, PIDTUNE_PARAM_LIMIT);
    /* SET drives the REAL balance loop now, not just a demo computation. */
    App_Balance_SetGains(s_params.kp100, s_params.ki100, s_params.kd100);
    SendAck();
    return;
  }

  if (strncmp(line, "SETV,", 5U) == 0)
  {
    int32_t kvp, kvi;

    p = line + 5U;
    if (ParseInt32(&p, &kvp) == 0U) { SendErr("SETV kp"); return; }
    if (*p != ',')                 { SendErr("SETV sep"); return; }
    p++;
    if (ParseInt32(&p, &kvi) == 0U) { SendErr("SETV ki"); return; }
    if (*p != '\0')                { SendErr("SETV tail"); return; }

    s_params.kv_p10000 = ClampInt32(kvp, -PIDTUNE_SPEED_LIMIT,
                                    PIDTUNE_SPEED_LIMIT);
    s_params.kv_i10000 = ClampInt32(kvi, -PIDTUNE_SPEED_LIMIT,
                                    PIDTUNE_SPEED_LIMIT);
    App_Balance_SetSpeedGains(s_params.kv_p10000, s_params.kv_i10000);
    SendAckSpeed();
    return;
  }

  if (strncmp(line, "SETT,", 5U) == 0)
  {
    int32_t ktp, kti;

    p = line + 5U;
    if (ParseInt32(&p, &ktp) == 0U) { SendErr("SETT kp"); return; }
    if (*p != ',')                 { SendErr("SETT sep"); return; }
    p++;
    if (ParseInt32(&p, &kti) == 0U) { SendErr("SETT ki"); return; }
    if (*p != '\0')                { SendErr("SETT tail"); return; }

    s_params.kt_p100 = ClampInt32(ktp, -PIDTUNE_PARAM_LIMIT,
                                  PIDTUNE_PARAM_LIMIT);
    s_params.kt_i100 = ClampInt32(kti, -PIDTUNE_PARAM_LIMIT,
                                  PIDTUNE_PARAM_LIMIT);
    App_Balance_SetTurnGains(s_params.kt_p100, s_params.kt_i100);
    SendAckTurn();
    return;
  }

  if (strncmp(line, "SPD,", 4U) == 0)
  {
    p = line + 4U;
    if (ParseInt32(&p, &val) == 0U) { SendErr("SPD value"); return; }
    if (*p != '\0')                { SendErr("SPD tail"); return; }
    val = ClampInt32(val, -PIDTUNE_SPEED_PULSE_LIMIT,
                     PIDTUNE_SPEED_PULSE_LIMIT);
    App_Balance_SetTargetSpeed(val);
    (void)sprintf(s_tx, "LOG,SPD=%ld.%02ld rev/s\n",
                  (long)(val / 100L), (long)(val % 100L));
    (void)HC05_SendString(s_tx);
    return;
  }

  if (strncmp(line, "YAWSP,", 6U) == 0)
  {
    p = line + 6U;
    if (ParseInt32(&p, &val) == 0U) { SendErr("YAWSP value"); return; }
    if (*p != '\0')                { SendErr("YAWSP tail"); return; }
    val = ClampInt32(val, -PIDTUNE_RC_LIMIT, PIDTUNE_RC_LIMIT);
    App_Balance_SetYawRate(val);
    (void)sprintf(s_tx, "LOG,DIFF SPD=%ld.%02ld rev/s\n",
                  (long)(val / 100L), (long)(val % 100L));
    (void)HC05_SendString(s_tx);
    return;
  }

  if (strncmp(line, "TARGET,", 7U) == 0)
  {
    p = line + 7U;
    if (ParseInt32(&p, &val) == 0U) { SendErr("TARGET value"); return; }
    if (*p != '\0')                 { SendErr("TARGET tail"); return; }
    s_params.target100 = ClampInt32(val, -PIDTUNE_TARGET_LIMIT,
                                    PIDTUNE_TARGET_LIMIT);
    App_Balance_SetTargetOffset(s_params.target100);
    (void)sprintf(s_tx, "LOG,TARGET=%ld (x100 deg)\n",
                  (long)s_params.target100);
    (void)HC05_SendString(s_tx);
    return;
  }

#if (APP_ENABLE_TB6612 != 0U)
  if (strcmp(line, "CAL") == 0)
  {
    uint8_t rc = App_Balance_Calibrate();
    if (rc != 0U) { SendErr("CAL rejected (IMU/balance running)"); }
    return;
  }

  if (strncmp(line, "BAL,", 4U) == 0)
  {
    p = line + 4U;
    if ((p[0] == '1') && (p[1] == '\0'))
    {
      uint8_t rc = App_Balance_Enable(1U);
      if (rc != 0U) { SendErr("BAL,1 rejected - level car then CAL"); }
    }
    else if ((p[0] == '0') && (p[1] == '\0'))
    {
      (void)App_Balance_Enable(0U);
    }
    else
    {
      SendErr("BAL value");
    }
    return;
  }

  if (strcmp(line, "BST") == 0)
  {
    AppBalance_Status_t st;
    App_Balance_GetStatus(&st);
    (void)sprintf(s_tx, "BST,%u,%ld,%ld,%ld,%ld,%ld,%ld,%u,%ld,%ld\n",
                  (unsigned int)st.state,
                  (long)(st.angle * 100.0f),
                  (long)(st.offset * 100.0f),
                  (long)st.duty,
                  (long)st.p100, (long)st.i100, (long)st.d100,
                  (unsigned int)st.cal_progress,
                  (long)st.vel100, (long)st.turn);
    (void)HC05_SendString(s_tx);
    return;
  }

  if (strncmp(line, "POL,", 4U) == 0)
  {
    int32_t inv_l, inv_r;

    p = line + 4U;
    if (ParseInt32(&p, &inv_l) == 0U) { SendErr("POL left"); return; }
    if (*p != ',')                    { SendErr("POL sep"); return; }
    p++;
    if (ParseInt32(&p, &inv_r) == 0U) { SendErr("POL right"); return; }
    if (*p != '\0')                   { SendErr("POL tail"); return; }
    if ((inv_l < 0L) || (inv_l > 1L) || (inv_r < 0L) || (inv_r > 1L))
    {
      SendErr("POL range");
      return;
    }
    App_MotorTest_SetPolarity((uint8_t)inv_l, (uint8_t)inv_r);
    SendLine("LOG,Polarity updated");
    return;
  }

  if (strncmp(line, "TRIM,", 5U) == 0)
  {
    int32_t tl, tr;
    uint8_t l, r;

    p = line + 5U;
    if (ParseInt32(&p, &tl) == 0U) { SendErr("TRIM left"); return; }
    if (*p != ',')                 { SendErr("TRIM sep"); return; }
    p++;
    if (ParseInt32(&p, &tr) == 0U) { SendErr("TRIM right"); return; }
    if (*p != '\0')                { SendErr("TRIM tail"); return; }
    if ((tl < (int32_t)APP_MOTOR_TRIM_MIN) ||
        (tl > (int32_t)APP_MOTOR_TRIM_MAX) ||
        (tr < (int32_t)APP_MOTOR_TRIM_MIN) ||
        (tr > (int32_t)APP_MOTOR_TRIM_MAX))
    {
      SendErr("TRIM range 50..150");
      return;
    }
    l = (uint8_t)tl;
    r = (uint8_t)tr;
    App_MotorTest_SetTrim(l, r);
    (void)sprintf(s_tx, "LOG,TRIM L=%u%% R=%u%%\n",
                  (unsigned int)l, (unsigned int)r);
    (void)HC05_SendString(s_tx);
    return;
  }

  if (strncmp(line, "SIGN,", 5U) == 0)
  {
    p = line + 5U;
    if ((p[0] == '1') && (p[1] == '\0'))
    {
      App_Balance_SetOutputSign(1);
    }
    else if ((p[0] == '-') && (p[1] == '1') && (p[2] == '\0'))
    {
      App_Balance_SetOutputSign(-1);
    }
    else
    {
      SendErr("SIGN value (1 or -1)");
      return;
    }
    (void)sprintf(s_tx, "LOG,SIGN=%d\n", (int)App_Balance_GetOutputSign());
    (void)HC05_SendString(s_tx);
    return;
  }

  if (strncmp(line, "PROT,", 5U) == 0)
  {
    p = line + 5U;
    if ((p[0] == '0') && (p[1] == '\0'))
    {
      App_Balance_SetFaultProtect(0U);
    }
    else if ((p[0] == '1') && (p[1] == '\0'))
    {
      App_Balance_SetFaultProtect(1U);
    }
    else
    {
      SendErr("PROT value (0 or 1)");
      return;
    }
    (void)sprintf(s_tx, "LOG,50deg protection=%u\n",
                  (unsigned int)App_Balance_GetFaultProtect());
    (void)HC05_SendString(s_tx);
    return;
  }

#endif

  if (strcmp(line, "ENC") == 0)
  {
    (void)sprintf(s_tx, "ENC,%ld,%ld,%ld,%ld\n",
                  (long)BSP_Encoder_GetCount(BSP_ENC_LEFT),
                  (long)BSP_Encoder_GetCount(BSP_ENC_RIGHT),
                  (long)BSP_Encoder_GetSpeed(BSP_ENC_LEFT),
                  (long)BSP_Encoder_GetSpeed(BSP_ENC_RIGHT));
    (void)HC05_SendString(s_tx);
    return;
  }
  if (strcmp(line, "ENCZ") == 0)
  {
    BSP_Encoder_Reset(BSP_ENC_LEFT);
    BSP_Encoder_Reset(BSP_ENC_RIGHT);
    SendLine("LOG,Encoders zeroed");
    return;
  }
  if (strncmp(line, "ENCS,", 5U) == 0)
  {
    int32_t il, ir;

    p = line + 5U;
    if (ParseInt32(&p, &il) == 0U) { SendErr("ENCS left"); return; }
    if (*p != ',')                { SendErr("ENCS sep"); return; }
    p++;
    if (ParseInt32(&p, &ir) == 0U) { SendErr("ENCS right"); return; }
    if (*p != '\0')               { SendErr("ENCS tail"); return; }
    if ((il < 0L) || (il > 1L) || (ir < 0L) || (ir > 1L))
    {
      SendErr("ENCS range");
      return;
    }
    BSP_Encoder_SetInvert(BSP_ENC_LEFT, (uint8_t)il);
    BSP_Encoder_SetInvert(BSP_ENC_RIGHT, (uint8_t)ir);
    BSP_Encoder_Reset(BSP_ENC_LEFT);
    BSP_Encoder_Reset(BSP_ENC_RIGHT);
    (void)sprintf(s_tx, "LOG,ENC invert L=%ld R=%ld\n", (long)il, (long)ir);
    (void)HC05_SendString(s_tx);
    return;
  }

  SendErr("unknown cmd");
}

static void PollCommands(void)
{
  int16_t c;

  while ((c = HC05_ReadByte()) >= 0)
  {
    if ((c == '\n') || (c == '\r'))
    {
      if (s_line_len > 0U)
      {
        s_line[s_line_len] = '\0';
        HandleLine(s_line);
        s_line_len = 0U;
      }
    }
    else if (s_line_len < (uint16_t)(PIDTUNE_LINE_CAP - 1U))
    {
      if ((c >= 32) && (c < 127))
      {
        s_line[s_line_len] = (char)c;
        s_line_len++;
      }
    }
    else
    {
      /* Overlong line: drop it entirely and resync on the next LF. */
      s_line_len = 0U;
    }
  }
}

static void ComputeTerms(uint32_t now_ms, const MPU6050_Data_t *imu,
                         uint8_t imu_ok)
{
  AppBalance_Status_t st;
  float target_deg;

  (void)now_ms;
  (void)imu;
  (void)imu_ok;
  /* Stream the REAL balance-loop terms so the PC curves show what the
   * controller actually did, not a separate demo computation. */
  App_Balance_GetStatus(&st);
  s_angle100 = (int32_t)(st.angle * 100.0f);
  s_p100     = st.p100;
  s_i100     = st.i100;
  s_d100     = st.d100;
  s_pwm100   = (int32_t)st.duty;   /* raw PWM -7200..7200, NOT x100 */
  s_vel100   = st.vel100;
  s_turn     = st.turn;
  s_enc_tps  = st.enc_tps;
  s_gyro_z100 = st.gyro_z100;

  /* The angle loop's Actual is already zeroed by CAL (st.angle = raw-centre),
   * so its Target is just the speed-loop tilt plus operator trim. Adding the
   * offset here would plot a target the controller never compares against. */
  target_deg = ((float)s_params.target100 / 100.0f) +
               ((float)st.vel100 / 100.0f);
  s_tx_target100 = (int32_t)(target_deg * 100.0f);
}

static void SendTelemetry(uint32_t now_ms)
{
  (void)sprintf(s_tx, "T,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
                (unsigned long)now_ms,
                (long)s_angle100, (long)s_tx_target100,
                (long)s_p100, (long)s_i100, (long)s_d100,
                (long)s_pwm100, (long)s_vel100, (long)s_turn,
                (long)s_enc_tps, (long)s_gyro_z100);
  (void)HC05_SendString(s_tx);
}

void App_PIDTune_Init(void)
{
  s_params.kp100     = PIDTUNE_DEFAULT_KP100;
  s_params.ki100     = PIDTUNE_DEFAULT_KI100;
  s_params.kd100     = PIDTUNE_DEFAULT_KD100;
  s_params.target100 = PIDTUNE_DEFAULT_TARGET100;
  App_Balance_GetSpeedGains(&s_params.kv_p10000, &s_params.kv_i10000);
  App_Balance_GetTurnGains(&s_params.kt_p100, &s_params.kt_i100);
  s_line_len         = 0U;
  s_started          = 0U;
  s_angle100         = 0L;
  s_tx_target100     = 0L;
  s_p100             = 0L;
  s_i100             = 0L;
  s_d100             = 0L;
  s_pwm100           = 0L;
  s_vel100           = 0L;
  s_turn             = 0L;
  s_enc_tps          = 0L;
  s_gyro_z100        = 0L;
  /* Push the initial gains into the real three-loop controller. */
  App_Balance_SetGains(s_params.kp100, s_params.ki100, s_params.kd100);
  App_Balance_SetSpeedGains(s_params.kv_p10000, s_params.kv_i10000);
  App_Balance_SetTurnGains(s_params.kt_p100, s_params.kt_i100);
  SendLine("LOG,PID tuner ready (3-loop, on-car tuned) - CAL then BAL,1 to move");
}

void App_PIDTune_Poll(uint32_t now_ms, const MPU6050_Data_t *imu,
                      uint8_t imu_ok)
{
  if (s_started == 0U)
  {
    s_started = 1U;
    s_last_tx_ms = now_ms;
  }

  PollCommands();
  ComputeTerms(now_ms, imu, imu_ok);

  /* Unsigned subtraction is wrap-safe for the 32-bit HAL tick counter. */
  if ((uint32_t)(now_ms - s_last_tx_ms) >= (uint32_t)PIDTUNE_TELEMETRY_MS)
  {
    s_last_tx_ms = now_ms;
    SendTelemetry(now_ms);
  }
}

void App_PIDTune_GetParams(PIDTune_Params_t *out)
{
  if (out != 0)
  {
    out->kp100     = s_params.kp100;
    out->ki100     = s_params.ki100;
    out->kd100     = s_params.kd100;
    out->target100 = s_params.target100;
    out->kv_p10000 = s_params.kv_p10000;
    out->kv_i10000 = s_params.kv_i10000;
    out->kt_p100   = s_params.kt_p100;
    out->kt_i100   = s_params.kt_i100;
  }
}

int32_t App_PIDTune_GetPwm100(void)
{
  return s_pwm100;
}
