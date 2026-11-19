/* app_sensor_display.c - 3-axis accel + 3 XYZ angles + SR04 -> SSD1306 demo
 *
 * OLED rows: AX / AY / AZ (raw LSB), X / Y / Z* (degrees), D (mm).
 *   X  = angle about X axis = roll  (absolute, from accelerometer)
 *   Y  = angle about Y axis = pitch (absolute, from accelerometer)
 *   Z* = angle about Z axis = yaw   (RELATIVE, integrated from gyro Z, drifts)
 *   D  = SR04 echo distance; "----" on timeout / no echo.
 * Page 7 shows the three-loop controller state. HC-05 + TB6612 are both
 * ACTIVE, but motors stay OFF until CAL then BAL,1 is accepted. Encoders are
 * sampled by the 200 Hz BalanceTask (speed loop), not by this display task.
 */
#include "app_sensor_display.h"
#include "app_pid_tune.h"
#include "app_balance.h"
#include "app_motor_test.h"
#include "bsp_encoder.h"
#include "cmsis_os.h"
#include "bsp_delay.h"
#include "bsp_exti.h"
#include "bsp_oled.h"
#include "bsp_mpu6050.h"
#include "bsp_sr04.h"
#include "bsp_hc05.h"
#include "bsp_tb6612.h"

static uint8_t s_oled_ok = 0U;
/* Keep the IMU struct off the 1 kB task stack. */
static MPU6050_Data_t s_imu;
/* Last valid SR04 result, kept until a new measurement finishes. */
static int32_t s_dist_mm = SR04_NO_ECHO;

/* Print one angle given in 0.1 deg units, e.g. -5 -> "X:-0.5 deg".
 * Splitting the magnitude first avoids the classic "-0.5" -> "+0.5" bug. */
static void App_ShowAngle(uint8_t page, const char *tag, int32_t a10)
{
  char sign;
  int32_t mag;

  sign = (a10 < 0) ? '-' : '+';
  mag  = (a10 < 0) ? -a10 : a10;

  OLED_Printf(0U, page, "%s%c%ld.%ld deg", tag, sign,
              (long)(mag / 10), (long)(mag % 10));
}

static void App_ShowAccel(uint8_t page, const char *tag, int16_t value)
{
  OLED_Printf(0U, page, "%s%+6d", tag, value);
}

static void App_ShowDistance(void)
{
  if (s_dist_mm >= 0)
  {
    OLED_Printf(0U, 6U, "D:%4ld mm", (long)s_dist_mm);
  }
  else
  {
    OLED_ShowString(0U, 6U, "D:  ---- mm");
  }
}

/* Page 7 shows the balance state + wheel duty at a glance. */
static void App_ShowMotor(void)
{
  AppBalance_Status_t st;

  App_Balance_GetStatus(&st);
  switch (st.state)
  {
    case APP_BAL_STATE_RUN:
      OLED_Printf(0U, 7U, "BAL RUN %+4ld", (long)st.duty);
      break;
    case APP_BAL_STATE_CAL:
      OLED_Printf(0U, 7U, "CAL %3u%%", (unsigned long)st.cal_progress);
      break;
    case APP_BAL_STATE_FAULT:
      OLED_ShowString(0U, 7U, "BAL FAULT!");
      break;
    case APP_BAL_STATE_IDLE:
    default:
      OLED_ShowString(0U, 7U, "BAL IDLE");
      break;
  }
}

static void App_ShowInvalid(void)
{
  OLED_ShowString(0U, 0U, "AX:  ----");
  OLED_ShowString(0U, 1U, "AY:  ----");
  OLED_ShowString(0U, 2U, "AZ:  ----");
  OLED_ShowString(0U, 3U, "X:  ---- deg");
  OLED_ShowString(0U, 4U, "Y:  ---- deg");
  OLED_ShowString(0U, 5U, "Z*: ---- deg");
}

void App_SensorDisplay_Init(void)
{
  BSP_Delay_Init();
  BSP_EXTI_Init();

  /* App_Balance_Init() owns MPU6050_Init (gyro Y/Z bias calibration) and
   * TB6612 init (coast). Keep the car still for ~0.5 s at power-up.
   * BalanceTask samples the IMU; this task only reads a snapshot. */
  App_Balance_Init();
  s_oled_ok = (OLED_Init() == 0U) ? 1U : 0U;
  SR04_Init();
  BSP_Encoder_Init();          /* starts TIM2/TIM4 encoder counters */

#if (APP_ENABLE_HC05 != 0U)
  HC05_Init();                 /* starts USART3 RXNE interrupt reception */
  App_PIDTune_Init();          /* CAL/BAL/BST + gains + T,... telemetry */
#endif

  if (s_oled_ok != 0U)
  {
    OLED_Clear();
    (void)OLED_Refresh();      /* blank screen until the first sample */
  }
}

void App_SensorDisplay_Loop(void)
{
  uint8_t imu_ok;
  uint16_t tick = 0U;
  int32_t d;
  int32_t x10;
  int32_t y10;
  int32_t z10;
  MPU6050_Data_t *imu = &s_imu;

  for (;;)
  {
    /* 1. Snapshot of the IMU sampled by BalanceTask @ 200 Hz. No I2C here. */
    App_Balance_GetImu(imu);
    imu_ok = App_Balance_IsImuOk();
    MPU6050_ClearDataReady();

    /* 1b. Bluetooth command parser + real-loop telemetry. */
#if (APP_ENABLE_HC05 != 0U)
    App_PIDTune_Poll(HAL_GetTick(), imu, imu_ok);
#endif
    /* BalanceTask samples the encoders at 200 Hz for the speed loop; this
     * task only reads the snapshot when ENC is requested. */

    /* 2. Consume the result of the PREVIOUS trigger. BUSY and NO_NEW_RESULT
     *    are ignored so the last valid distance stays on screen. */
    d = SR04_GetDistanceMm();
    if ((d != SR04_BUSY) && (d != SR04_NO_NEW_RESULT))
    {
      s_dist_mm = d;           /* >= 0 distance, or SR04_NO_ECHO (-1) */
    }

    /* 3. Start a new measurement every 100 ms (echo is < 38 ms). */
    if ((tick % 5U) == 0U)
    {
      SR04_Trigger();
    }

    /* 4. Repaint every 60 ms (~16 Hz): the I2C framebuffer push is slow. */
    if (((tick % 3U) == 0U) && (s_oled_ok != 0U))
    {
      OLED_Clear();
      if (imu_ok != 0U)
      {
        App_ShowAccel(0U, "AX:", imu->raw.ax);
        App_ShowAccel(1U, "AY:", imu->raw.ay);
        App_ShowAccel(2U, "AZ:", imu->raw.az);

        x10 = (int32_t)(imu->angle_x * 10.0f);
        /* Y row = fused pitch actually used by the balance loop. */
        y10 = (int32_t)(App_Balance_GetAngle() * 10.0f);
        z10 = (int32_t)(imu->angle_z * 10.0f);
        App_ShowAngle(3U, "X:",  x10);
        App_ShowAngle(4U, "Y:",  y10);
        App_ShowAngle(5U, "Z*:", z10);
      }
      else
      {
        App_ShowInvalid();
      }
      App_ShowDistance();      /* page 6, independent of IMU status */
      App_ShowMotor();         /* page 7: duty + ON/OFF */
      (void)OLED_Refresh();
    }

    tick++;
    osDelay(20U);
  }
}
