/* bsp_exti.c - EXTI callback dispatcher (runs in interrupt context) */
#include "bsp_exti.h"
#include "bsp_sr04.h"
#include "bsp_mpu6050.h"

void BSP_EXTI_Init(void)
{
  /* NVIC lines and GPIO modes are configured by MX_GPIO_Init(). Nothing to
   * register at runtime; this hook exists so the app has one init entry. */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SR04_Echo_Pin)
  {
    SR04_EXTI_Callback(GPIO_Pin);          /* PA2, both edges */
  }
  else if (GPIO_Pin == MPU6050_INT_Pin)
  {
    MPU6050_EXTI_Callback();               /* PB5, falling edge */
  }
}
