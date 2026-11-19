/* bsp_mpu6050.c - MPU6050 driver, software I2C backend
 *
 * pitch / roll come from the accelerometer (absolute tilt).
 * yaw is a RELATIVE heading integrated from gyro Z, because the MPU6050 has
 * no magnetometer. A zero-offset calibration is done at init, so keep the
 * board still while powering up.
 */
#include "bsp_mpu6050.h"
#include "bsp_soft_i2c.h"
#include "bsp_delay.h"
#include <math.h>

#define MPU_REG_SMPLRT_DIV   0x19U
#define MPU_REG_CONFIG       0x1AU
#define MPU_REG_GYRO_CONFIG  0x1BU
#define MPU_REG_ACCEL_CONFIG 0x1CU
#define MPU_REG_INT_PIN_CFG  0x37U
#define MPU_REG_INT_ENABLE   0x38U
#define MPU_REG_ACCEL_XOUT_H 0x3BU
#define MPU_REG_PWR_MGMT_1   0x6BU
#define MPU_REG_PWR_MGMT_2   0x6CU
#define MPU_REG_WHO_AM_I     0x75U

/* Register settings below are a 1:1 copy of the JiangKeDa tutorial's
 * Hardware/MPU6050.c (reference project 06):
 *   PWR_MGMT_1   = 0x01  PLL with X gyro
 *   PWR_MGMT_2   = 0x00  all axes enabled
 *   SMPLRT_DIV   = 0x07  sample rate = gyro rate / 8
 *   CONFIG       = 0x00  DLPF OFF (260 Hz gyro / 256 Hz accel)
 *   GYRO_CONFIG  = 0x18  +/-2000 dps
 *   ACCEL_CONFIG = 0x18  +/-16 g
 * The tutorial explicitly prefers bandwidth over noise (low latency matters
 * more), so the DLPF is disabled. Do not "improve" this back to 0x03. */
#define MPU_ACCEL_SCALE      2048.0f     /* LSB per g,   AFS_SEL = 3 (+/-16 g) */
#define MPU_GYRO_SCALE       MPU6050_GYRO_LSB_PER_DPS
#define MPU_RAD_TO_DEG       57.2957795

#define MPU_GYRO_CAL_SAMPLES     100U
#define MPU_GYRO_CAL_DELAY_MS    3U
#define MPU_YAW_MAX_DT           0.2f    /* ignore long gaps, e.g. debug halt */

static volatile uint8_t s_data_ready = 0U;
/* Gyro zero offsets (dps) measured while the board is still at boot.
 * gy drives the balance (pitch) integral, gz drives the relative yaw. */
static float     s_gyro_bias_y = 0.0f;
static float     s_gyro_bias_z = 0.0f;
static float     s_yaw = 0.0f;           /* deg, relative heading */
static uint32_t  s_last_us = 0U;         /* last integration timestamp */

static int16_t MPU_BuildInt16(uint8_t hi, uint8_t lo)
{
  return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

uint8_t MPU6050_WhoAmI(uint8_t *id)
{
  if (id == 0)
  {
    return 1U;
  }
  return BSP_SoftI2C_ReadRegs(MPU6050_I2C_ADDR, MPU_REG_WHO_AM_I, id, 1U);
}

uint8_t MPU6050_Init(uint8_t sample_div)
{
  uint8_t id = 0U;
  uint8_t rc;

  BSP_SoftI2C_Init();

  if (BSP_SoftI2C_CheckDevice(MPU6050_I2C_ADDR) == 0U)
  {
    return 1U;                        /* no ACK on the bus */
  }

  /* Device reset, then select PLL with X-axis gyro as clock source. */
  rc = BSP_SoftI2C_WriteReg(MPU6050_I2C_ADDR, MPU_REG_PWR_MGMT_1, 0x80U);
  if (rc != 0U) { return rc; }
  HAL_Delay(100U);
  /* JiangKeDa: PWR_MGMT_1=0x01 (X gyro PLL), PWR_MGMT_2=0x00 (no standby). */
  rc = BSP_SoftI2C_WriteReg(MPU6050_I2C_ADDR, MPU_REG_PWR_MGMT_1, 0x01U);
  if (rc != 0U) { return rc; }
  rc = BSP_SoftI2C_WriteReg(MPU6050_I2C_ADDR, MPU_REG_PWR_MGMT_2, 0x00U);
  if (rc != 0U) { return rc; }

  /* CONFIG = 0x00 -> DLPF disabled. Lowest latency, highest bandwidth.
   * The caller passes 0x07 (MPU6050_SAMPLE_DIV_TUTORIAL). */
  rc = BSP_SoftI2C_WriteReg(MPU6050_I2C_ADDR, MPU_REG_CONFIG, 0x00U);
  if (rc != 0U) { return rc; }

  rc = BSP_SoftI2C_WriteReg(MPU6050_I2C_ADDR, MPU_REG_SMPLRT_DIV, sample_div);
  if (rc != 0U) { return rc; }

  /* FS_SEL = 3 -> +/-2000 dps, AFS_SEL = 3 -> +/-16 g. */
  rc = BSP_SoftI2C_WriteReg(MPU6050_I2C_ADDR, MPU_REG_GYRO_CONFIG, 0x18U);
  if (rc != 0U) { return rc; }
  rc = BSP_SoftI2C_WriteReg(MPU6050_I2C_ADDR, MPU_REG_ACCEL_CONFIG, 0x18U);
  if (rc != 0U) { return rc; }

  /* INT active LOW, push-pull, 50 us pulse; enable DATA_RDY interrupt. */
  rc = BSP_SoftI2C_WriteReg(MPU6050_I2C_ADDR, MPU_REG_INT_PIN_CFG, 0x80U);
  if (rc != 0U) { return rc; }
  rc = BSP_SoftI2C_WriteReg(MPU6050_I2C_ADDR, MPU_REG_INT_ENABLE, 0x01U);
  if (rc != 0U) { return rc; }

  rc = MPU6050_WhoAmI(&id);
  if ((rc != 0U) || (id != MPU6050_WHO_AM_I_VALUE))
  {
    return 2U;
  }

  /* Gyro Y/Z zero offsets: average raw gy/gz while the board is still.
   * gy is the pitch-rate axis used by the balance loop, so its bias must be
   * removed or the complementary filter's angle drifts continuously.
   * ~100 * (3 ms + I2C read) = about 0.4 s of extra boot time.
   * IMPORTANT: keep the car STILL during power-up calibration. */
  {
    uint16_t i;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
    MPU6050_Raw_t cal;

    for (i = 0U; i < MPU_GYRO_CAL_SAMPLES; i++)
    {
      if (MPU6050_ReadRaw(&cal) != 0U)
      {
        return 3U;
      }
      sum_y += (int32_t)cal.gy;
      sum_z += (int32_t)cal.gz;
      HAL_Delay(MPU_GYRO_CAL_DELAY_MS);
    }
    s_gyro_bias_y = ((float)sum_y / (float)MPU_GYRO_CAL_SAMPLES) / MPU_GYRO_SCALE;
    s_gyro_bias_z = ((float)sum_z / (float)MPU_GYRO_CAL_SAMPLES) / MPU_GYRO_SCALE;
  }

  s_yaw = 0.0f;
  s_last_us = BSP_GetMicros();
  s_data_ready = 0U;
  return 0U;
}

uint8_t MPU6050_ReadRaw(MPU6050_Raw_t *raw)
{
  uint8_t buf[14];
  uint8_t rc;

  if (raw == 0)
  {
    return 1U;
  }

  rc = BSP_SoftI2C_ReadRegs(MPU6050_I2C_ADDR, MPU_REG_ACCEL_XOUT_H, buf, 14U);
  if (rc != 0U)
  {
    return rc;
  }

  raw->ax       = MPU_BuildInt16(buf[0],  buf[1]);
  raw->ay       = MPU_BuildInt16(buf[2],  buf[3]);
  raw->az       = MPU_BuildInt16(buf[4],  buf[5]);
  raw->temp_raw = MPU_BuildInt16(buf[6],  buf[7]);
  raw->gx       = MPU_BuildInt16(buf[8],  buf[9]);
  raw->gy       = MPU_BuildInt16(buf[10], buf[11]);
  raw->gz       = MPU_BuildInt16(buf[12], buf[13]);
  return 0U;
}

uint8_t MPU6050_ReadData(MPU6050_Data_t *data)
{
  uint8_t rc;
  double ay2az2;
  uint32_t now_us;
  float dt;

  if (data == 0)
  {
    return 1U;
  }

  rc = MPU6050_ReadRaw(&data->raw);
  if (rc != 0U)
  {
    return rc;
  }

  data->ax_g = (float)data->raw.ax / MPU_ACCEL_SCALE;
  data->ay_g = (float)data->raw.ay / MPU_ACCEL_SCALE;
  data->az_g = (float)data->raw.az / MPU_ACCEL_SCALE;

  data->gx_dps = (float)data->raw.gx / MPU_GYRO_SCALE;
  /* gy/gz are bias-corrected; gx is exposed raw (not used by balance now). */
  data->gy_dps = ((float)data->raw.gy / MPU_GYRO_SCALE) - s_gyro_bias_y;
  data->gz_dps = ((float)data->raw.gz / MPU_GYRO_SCALE) - s_gyro_bias_z;

  data->temp_c = ((float)data->raw.temp_raw / 340.0f) + 36.53f;

  /* Absolute tilt angles from the gravity vector (chip Z up, flat mount). */
  ay2az2 = (double)data->ay_g * data->ay_g + (double)data->az_g * data->az_g;
  data->pitch = (float)(atan2(-(double)data->ax_g, sqrt(ay2az2)) * MPU_RAD_TO_DEG);
  data->roll  = (float)(atan2((double)data->ay_g, (double)data->az_g) * MPU_RAD_TO_DEG);

  /* Relative yaw: integrate (gyro Z - calibrated bias) with the real dt.
   * BSP_GetMicros() uses HAL_GetTick() + TIM3->CNT, so a slower display
   * refresh does not distort the integral. */
  now_us = BSP_GetMicros();
  if (s_last_us != 0U)
  {
    dt = (float)(uint32_t)(now_us - s_last_us) / 1000000.0f;
    if ((dt > 0.0f) && (dt < MPU_YAW_MAX_DT))
    {
      s_yaw += data->gz_dps * dt;   /* gz_dps is already bias-corrected */
      if (s_yaw > 180.0f)       { s_yaw -= 360.0f; }
      else if (s_yaw < -180.0f) { s_yaw += 360.0f; }
    }
  }
  s_last_us = now_us;
  data->yaw = s_yaw;

  /* Same three angles, named by their rotation axis. */
  data->angle_x = data->roll;    /* about X = roll,  absolute */
  data->angle_y = data->pitch;   /* about Y = pitch, absolute */
  data->angle_z = data->yaw;     /* about Z = yaw,   RELATIVE */

  return 0U;
}

uint8_t MPU6050_DataReady(void)
{
  return s_data_ready;
}

void MPU6050_ClearDataReady(void)
{
  s_data_ready = 0U;
}

void MPU6050_EXTI_Callback(void)
{
  /* Interrupt context: only set a flag, never do I2C here. */
  s_data_ready = 1U;
}

void MPU6050_ResetYaw(void)
{
  s_yaw = 0.0f;
  s_last_us = BSP_GetMicros();
}

float MPU6050_GetGyroBiasY(void)
{
  return s_gyro_bias_y;
}

float MPU6050_GetGyroBiasZ(void)
{
  return s_gyro_bias_z;
}
