/* bsp_soft_i2c.h - bit-banged I2C on PB3/PB4 (MPU6050)
 *
 * Hardware I2C1 (PB8/PB9 remap) is reserved for the OLED. PB3/PB4 have no
 * I2C alternate function on STM32F103, so the MPU6050 must use software I2C.
 * CubeMX configured both pins as open-drain + pull-up, initial level HIGH,
 * and HAL_MspInit() calls __HAL_AFIO_REMAP_SWJ_NOJTAG() to free PB3/PB4.
 */
#ifndef __BSP_SOFT_I2C_H
#define __BSP_SOFT_I2C_H

#include <stdint.h>
#include "main.h"

#define SOFT_I2C_SCL_PORT        MPU6050_SCL_GPIO_Port
#define SOFT_I2C_SCL_PIN         MPU6050_SCL_Pin
#define SOFT_I2C_SDA_PORT        MPU6050_SDA_GPIO_Port
#define SOFT_I2C_SDA_PIN         MPU6050_SDA_Pin

/* Half-clock period in us. 5 us gives about 100 kHz, safe for long wires. */
#define SOFT_I2C_HALF_PERIOD_US  5U

void    BSP_SoftI2C_Init(void);
void    BSP_SoftI2C_Start(void);
void    BSP_SoftI2C_Stop(void);
void    BSP_SoftI2C_SendByte(uint8_t byte);
uint8_t BSP_SoftI2C_ReadByte(uint8_t ack);
uint8_t BSP_SoftI2C_WaitAck(void);       /* 0 = ACK, 1 = NACK */
void    BSP_SoftI2C_Ack(void);
void    BSP_SoftI2C_NAck(void);

/* High level helpers. dev_addr is the 7-bit address (e.g. 0x68). */
uint8_t BSP_SoftI2C_CheckDevice(uint8_t dev_addr);   /* 1 = device answered */
uint8_t BSP_SoftI2C_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t value);
uint8_t BSP_SoftI2C_WriteRegs(uint8_t dev_addr, uint8_t reg,
                              const uint8_t *buf, uint16_t len);
uint8_t BSP_SoftI2C_ReadRegs(uint8_t dev_addr, uint8_t reg,
                             uint8_t *buf, uint16_t len);

#endif /* __BSP_SOFT_I2C_H */
