/* app_motor_test.c - two-wheel drive helper (polarity handled in one place) */
#include "app_motor_test.h"

static uint8_t s_left_inv  = (uint8_t)APP_MOTOR_LEFT_INVERT;
static uint8_t s_right_inv = (uint8_t)APP_MOTOR_RIGHT_INVERT;
static uint8_t s_left_trim  = 100U;
static uint8_t s_right_trim = 100U;

static uint8_t TrimClamp(int32_t v)
{
  if (v < (int32_t)APP_MOTOR_TRIM_MIN) { return APP_MOTOR_TRIM_MIN; }
  if (v > (int32_t)APP_MOTOR_TRIM_MAX) { return APP_MOTOR_TRIM_MAX; }
  return (uint8_t)v;
}

void App_MotorTest_Init(void)
{
  s_left_inv  = (uint8_t)APP_MOTOR_LEFT_INVERT;
  s_right_inv = (uint8_t)APP_MOTOR_RIGHT_INVERT;
  s_left_trim  = 100U;
  s_right_trim = 100U;
  TB6612_Init();
  TB6612_StopAll();              /* coast on power-up, no auto-start */
}

static int16_t ApplyWheel(int16_t duty, uint8_t trim, uint8_t inv)
{
  int32_t v = (int32_t)duty;
  int32_t out;

  if (v > (int32_t)APP_MOTOR_DUTY_MAX)  { v = (int32_t)APP_MOTOR_DUTY_MAX; }
  if (v < -(int32_t)APP_MOTOR_DUTY_MAX) { v = -(int32_t)APP_MOTOR_DUTY_MAX; }

  /* Apply the hardware trim first, then re-clamp to the PWM full scale. */
  out = (v * (int32_t)trim) / 100L;
  if (out > (int32_t)APP_MOTOR_DUTY_MAX)  { out = (int32_t)APP_MOTOR_DUTY_MAX; }
  if (out < -(int32_t)APP_MOTOR_DUTY_MAX) { out = -(int32_t)APP_MOTOR_DUTY_MAX; }
  if (inv != 0U) { out = -out; }
  return (int16_t)out;
}

void App_MotorTest_Drive(int16_t duty)
{
  App_MotorTest_DriveLR(duty, duty);
}

void App_MotorTest_DriveLR(int16_t left_duty, int16_t right_duty)
{
  TB6612_SetSpeed(APP_MOTOR_LEFT_CHANNEL,
                  ApplyWheel(left_duty, s_left_trim, s_left_inv));
  TB6612_SetSpeed(APP_MOTOR_RIGHT_CHANNEL,
                  ApplyWheel(right_duty, s_right_trim, s_right_inv));
}

void App_MotorTest_Stop(void)
{
  TB6612_StopAll();
}

void App_MotorTest_SetPolarity(uint8_t left_inv, uint8_t right_inv)
{
  s_left_inv  = (left_inv  != 0U) ? 1U : 0U;
  s_right_inv = (right_inv != 0U) ? 1U : 0U;
}

void App_MotorTest_GetPolarity(uint8_t *left_inv, uint8_t *right_inv)
{
  if (left_inv  != 0) { *left_inv  = s_left_inv; }
  if (right_inv != 0) { *right_inv = s_right_inv; }
}

void App_MotorTest_SetTrim(uint8_t left_pct, uint8_t right_pct)
{
  s_left_trim  = TrimClamp((int32_t)left_pct);
  s_right_trim = TrimClamp((int32_t)right_pct);
}

void App_MotorTest_GetTrim(uint8_t *left_pct, uint8_t *right_pct)
{
  if (left_pct  != 0) { *left_pct  = s_left_trim; }
  if (right_pct != 0) { *right_pct = s_right_trim; }
}
