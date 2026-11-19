/* bsp_mpu6050.h - MPU6050 (GY-521) over software I2C on PB3/PB4
 *
 * CubeMX: PB3 = SDA, PB4 = SCL, open drain + pull-up, initial HIGH;
 *         PB5 = MPU6050_INT, EXTI9_5 falling edge, priority 5;
 *         HAL_MspInit() disables JTAG so PB3/PB4 are usable GPIO.
 * AD0 tied to GND -> 7-bit address 0x68.
 *
 * Angle note: this is a 6-axis part (3 accel + 3 gyro, NO magnetometer).
 *   pitch / roll = absolute tilt from the gravity vector.
 *   yaw          = relative heading from gyro-Z integration; it drifts and
 *                  has no absolute north reference. Wrap with MPU6050_ResetYaw().
 */
#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H

#include <stdint.h>
#include "main.h"

#define MPU6050_I2C_ADDR          0x68U
/* JiangKeDa tutorial: SMPLRT_DIV = 7, DLPF off. The gyro runs at 8 kHz
 * without the filter, so the sensor registers new data at 8 kHz / 8 = 1 kHz;
 * the control task reads it at 100 Hz (10 ms). Kept as a named constant so
 * the value is identical to MPU6050.c in the reference project. */
#define MPU6050_SAMPLE_DIV_TUTORIAL 7U
#define MPU6050_SAMPLE_DIV_200HZ    MPU6050_SAMPLE_DIV_TUTORIAL  /* alias */
/* FS_SEL = 3 -> +/-2000 dps. Shared with the balance loop: the tutorial
 * subtracts the zero offset from GY once, then uses that corrected GY both
 * for the complementary filter and for the D term. We do the same. */
#define MPU6050_GYRO_LSB_PER_DPS  16.4f
#define MPU6050_WHO_AM_I_VALUE    0x68U

typedef struct
{
  int16_t ax, ay, az;        /* raw accel, +/-16g  -> 2048 LSB/g */
  int16_t temp_raw;
  int16_t gx, gy, gz;        /* raw gyro,  +/-2000dps -> 16.4 LSB/dps */
} MPU6050_Raw_t;

typedef struct
{
  MPU6050_Raw_t raw;
  float ax_g, ay_g, az_g;
  float gx_dps, gy_dps, gz_dps;  /* bias-corrected: gy/gz have their zero offset removed */
  float temp_c;
  float pitch;               /* deg, from accelerometer, about Y axis */
  float roll;                /* deg, from accelerometer, about X axis */
  float yaw;                 /* deg, RELATIVE, integrated from gyro Z */

  /* Same three angles, named by the rotation axis (XYZ convention).
   * angle_x = rotation about X = roll  (absolute, from accel)
   * angle_y = rotation about Y = pitch (absolute, from accel)
   * angle_z = rotation about Z = yaw   (RELATIVE, from gyro-Z integral) */
  float angle_x, angle_y, angle_z;
} MPU6050_Data_t;

/* 0 = OK, non-zero = I2C / device error (return of BSP_SoftI2C_WriteRegs) */
uint8_t MPU6050_Init(uint8_t sample_div);
uint8_t MPU6050_WhoAmI(uint8_t *id);
uint8_t MPU6050_ReadRaw(MPU6050_Raw_t *raw);
uint8_t MPU6050_ReadData(MPU6050_Data_t *data);

/* Data-ready flag set by the PB5 EXTI line; safe to poll from a task. */
uint8_t MPU6050_DataReady(void);
void    MPU6050_ClearDataReady(void);
void    MPU6050_EXTI_Callback(void);

/* Zero the relative heading and restart the integration time base. */
void    MPU6050_ResetYaw(void);

/* Gyro zero offsets measured at boot (dps), for diagnostics/telemetry. */
float   MPU6050_GetGyroBiasY(void);
float   MPU6050_GetGyroBiasZ(void);

#endif /* __BSP_MPU6050_H */
