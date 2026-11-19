/* bsp_delay.c - microsecond timestamp / busy-wait delay */
#include "bsp_delay.h"

void BSP_Delay_Init(void)
{
  /* HAL_InitTick() has already started the timebase TIM in interrupt mode.
   * Nothing else to enable here; kept as an explicit init hook for the app. */
}

uint32_t BSP_GetMillis(void)
{
  return HAL_GetTick();
}

uint32_t BSP_GetMicros(void)
{
  uint32_t ms1, ms2, cnt;

  /* Read tick - counter - tick. Retry if a tick interrupt happened in between,
   * otherwise the counter value would belong to the wrong millisecond. */
  do
  {
    ms1 = HAL_GetTick();
    cnt = BSP_TICK_TIM->CNT;
    ms2 = HAL_GetTick();
  } while (ms1 != ms2);

  return (ms1 * 1000U) + cnt;
}

void BSP_DelayUs(uint32_t us)
{
  uint32_t start = BSP_GetMicros();
  while ((uint32_t)(BSP_GetMicros() - start) < us)
  {
    /* busy wait */
  }
}

void BSP_DelayMs(uint32_t ms)
{
  uint32_t start = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start) < ms)
  {
    /* busy wait, only use before the RTOS scheduler is started */
  }
}
