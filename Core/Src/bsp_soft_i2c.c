/* bsp_soft_i2c.c - bit-banged I2C master, open-drain GPIO */
#include "bsp_soft_i2c.h"
#include "bsp_delay.h"

#define SCL_H()  HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET)
#define SCL_L()  HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET)
#define SDA_H()  HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET)
#define SDA_L()  HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_RESET)
#define SDA_IN() HAL_GPIO_ReadPin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN)

#define SIIC_DELAY()  BSP_DelayUs(SOFT_I2C_HALF_PERIOD_US)

void BSP_SoftI2C_Init(void)
{
  /* GPIO mode is already set by MX_GPIO_Init(): open drain, pull-up, HIGH.
   * Just make sure the bus is idle (both lines released). */
  SDA_H();
  SCL_H();
  SIIC_DELAY();
}

void BSP_SoftI2C_Start(void)
{
  SDA_H();
  SCL_H();
  SIIC_DELAY();
  SDA_L();
  SIIC_DELAY();
  SCL_L();
}

void BSP_SoftI2C_Stop(void)
{
  SCL_L();
  SDA_L();
  SIIC_DELAY();
  SCL_H();
  SIIC_DELAY();
  SDA_H();
  SIIC_DELAY();
}

void BSP_SoftI2C_SendByte(uint8_t byte)
{
  uint8_t i;

  SCL_L();
  for (i = 0U; i < 8U; i++)
  {
    if ((byte & 0x80U) != 0U)
    {
      SDA_H();
    }
    else
    {
      SDA_L();
    }
    byte <<= 1;
    SIIC_DELAY();
    SCL_H();
    SIIC_DELAY();
    SCL_L();
    SIIC_DELAY();
  }
}

uint8_t BSP_SoftI2C_ReadByte(uint8_t ack)
{
  uint8_t i;
  uint8_t byte = 0U;

  SDA_H();                 /* release the line so the slave can drive it */
  for (i = 0U; i < 8U; i++)
  {
    byte <<= 1;
    SCL_H();
    SIIC_DELAY();
    if (SDA_IN() != GPIO_PIN_RESET)
    {
      byte |= 0x01U;
    }
    SCL_L();
    SIIC_DELAY();
  }

  if (ack != 0U)
  {
    BSP_SoftI2C_Ack();
  }
  else
  {
    BSP_SoftI2C_NAck();
  }
  return byte;
}

uint8_t BSP_SoftI2C_WaitAck(void)
{
  uint8_t level;

  SDA_H();                 /* release SDA */
  SIIC_DELAY();
  SCL_H();
  SIIC_DELAY();
  level = (SDA_IN() == GPIO_PIN_RESET) ? 0U : 1U;
  SCL_L();
  SIIC_DELAY();
  return level;
}

void BSP_SoftI2C_Ack(void)
{
  SCL_L();
  SDA_L();
  SIIC_DELAY();
  SCL_H();
  SIIC_DELAY();
  SCL_L();
  SDA_H();
}

void BSP_SoftI2C_NAck(void)
{
  SCL_L();
  SDA_H();
  SIIC_DELAY();
  SCL_H();
  SIIC_DELAY();
  SCL_L();
}

uint8_t BSP_SoftI2C_CheckDevice(uint8_t dev_addr)
{
  uint8_t ack;

  BSP_SoftI2C_Start();
  BSP_SoftI2C_SendByte((uint8_t)(dev_addr << 1));  /* write */
  ack = BSP_SoftI2C_WaitAck();
  BSP_SoftI2C_Stop();
  return (ack == 0U) ? 1U : 0U;
}

uint8_t BSP_SoftI2C_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t value)
{
  return BSP_SoftI2C_WriteRegs(dev_addr, reg, &value, 1U);
}

uint8_t BSP_SoftI2C_WriteRegs(uint8_t dev_addr, uint8_t reg,
                              const uint8_t *buf, uint16_t len)
{
  uint16_t i;
  uint8_t status = 0U;

  BSP_SoftI2C_Start();
  BSP_SoftI2C_SendByte((uint8_t)(dev_addr << 1));
  if (BSP_SoftI2C_WaitAck() != 0U) { status = 1U; goto exit; }
  BSP_SoftI2C_SendByte(reg);
  if (BSP_SoftI2C_WaitAck() != 0U) { status = 2U; goto exit; }

  for (i = 0U; i < len; i++)
  {
    BSP_SoftI2C_SendByte(buf[i]);
    if (BSP_SoftI2C_WaitAck() != 0U) { status = 3U; goto exit; }
  }

exit:
  BSP_SoftI2C_Stop();
  return status;
}

uint8_t BSP_SoftI2C_ReadRegs(uint8_t dev_addr, uint8_t reg,
                             uint8_t *buf, uint16_t len)
{
  uint16_t i;
  uint8_t status = 0U;

  if (len == 0U)
  {
    return 0U;
  }

  BSP_SoftI2C_Start();
  BSP_SoftI2C_SendByte((uint8_t)(dev_addr << 1));  /* write: set register pointer */
  if (BSP_SoftI2C_WaitAck() != 0U) { status = 1U; goto exit; }
  BSP_SoftI2C_SendByte(reg);
  if (BSP_SoftI2C_WaitAck() != 0U) { status = 2U; goto exit; }

  BSP_SoftI2C_Start();                              /* repeated start */
  BSP_SoftI2C_SendByte((uint8_t)((dev_addr << 1) | 0x01U));
  if (BSP_SoftI2C_WaitAck() != 0U) { status = 3U; goto exit; }

  for (i = 0U; i < len; i++)
  {
    buf[i] = BSP_SoftI2C_ReadByte((i + 1U < len) ? 1U : 0U); /* NACK on last byte */
  }

exit:
  BSP_SoftI2C_Stop();
  return status;
}
