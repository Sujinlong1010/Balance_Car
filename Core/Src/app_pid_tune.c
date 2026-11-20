/* app_pid_tune.c - HC-05 wireless PID tuning protocol implementation
 *
 * Runtime commands are intentionally limited to the three tuning loops
 * (SET/SETV/SETT + read-back), calibration (CAL), closed-loop enable/stop
 * (BAL,0/1) and status (BST). Everything else - target speed, target turn,
 * angle trim, output sign, fault protection, deadband, motor polarity,
 * encoder direction - is fixed in firmware constants; it is not tunable
 * over the air.
 *
 * Design notes:
 *  - Only integer printf formats are used (Keil microlib has no float printf).
 *  - Command bytes come from the HC05 ring buffer one at a time, so a line
 *    that arrives split across several task cycles is still assembled.
 *  - Telemetry streams the REAL balance-loop terms, scaled to x100 integers.
 *  - All static state lives in .bss, not on the 1 kB FreeRTOS task stack.
 */
#include "app_pid_tune.h"
#include "app_balance.h"
#include "bsp_flash_params.h"
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

static void SendErr(const char *why)
{
  (void)sprintf(s_tx, "LOG,ERR %s\n", why);
  (void)HC05_SendString(s_tx);
}

static void HandleLine(char *line)
{
  const char *p;
  int32_t kp, ki, kd;

  if (strcmp(line, "GET") == 0)
  {
    SendAck();
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
  if (strcmp(line, "SAVE") == 0)
  {
    ParamsStore_Gains_t g;
    AppBalance_Status_t st;

    /* A page erase stalls the CPU for tens of ms - never with motors on. */
    App_Balance_GetStatus(&st);
    if ((st.state == APP_BAL_STATE_RUN) || (st.state == APP_BAL_STATE_CAL))
    {
      SendErr("SAVE rejected - BAL,0 first");
      return;
    }
    /* s_params is the exact integer copy of what was last sent, so a
     * save/restore round trip reproduces the very same gains. */
    g.kp100 = s_params.kp100;       g.ki100 = s_params.ki100;
    g.kd100 = s_params.kd100;       g.kvp10000 = s_params.kv_p10000;
    g.kvi10000 = s_params.kv_i10000;
    g.ktp100 = s_params.kt_p100;    g.kti100 = s_params.kt_i100;
    if (ParamsStore_Save(&g) != 0U)
    {
      SendLine("LOG,Params saved to flash - survives power off");
    }
    else
    {
      SendErr("SAVE flash write failed");
    }
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

#endif

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
   * and the operator trim is fixed at 0 in firmware. Its Target is therefore
   * just the speed-loop tilt, which is the angle the controller compares
   * against. */
  target_deg = (float)st.vel100 / 100.0f;
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
  /* A previously SAVEd record wins over the factory defaults below. */
  {
    ParamsStore_Gains_t g;

    if (ParamsStore_Load(&g) != 0U)
    {
      s_params.kp100 = g.kp100;         s_params.ki100 = g.ki100;
      s_params.kd100 = g.kd100;         s_params.kv_p10000 = g.kvp10000;
      s_params.kv_i10000 = g.kvi10000;
      s_params.kt_p100 = g.ktp100;      s_params.kt_i100 = g.kti100;
      App_Balance_SetGains(s_params.kp100, s_params.ki100, s_params.kd100);
      App_Balance_SetSpeedGains(s_params.kv_p10000, s_params.kv_i10000);
      App_Balance_SetTurnGains(s_params.kt_p100, s_params.kt_i100);
      SendLine("LOG,Params restored from flash (SAVE updates it)");
      return;
    }
  }
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
