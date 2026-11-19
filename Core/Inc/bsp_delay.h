/* bsp_delay.h - microsecond time base built on the HAL timebase TIM
 *
 * CubeMX config: SYS -> Timebase Source = TIM3, 1 kHz tick, 1 MHz counter.
 * TIM3->CNT therefore counts microseconds inside the current millisecond.
 * Combining it with HAL_GetTick() gives a continuous 32-bit us timestamp
 * without stealing another hardware timer.
 */
#ifndef __BSP_DELAY_H
#define __BSP_DELAY_H

#include <stdint.h>
#include "main.h"

/* Change this if the CubeMX HAL Timebase Source is ever moved to another TIM */
#define BSP_TICK_TIM              TIM3

void     BSP_Delay_Init(void);
uint32_t BSP_GetMicros(void);
uint32_t BSP_GetMillis(void);
void     BSP_DelayUs(uint32_t us);
void     BSP_DelayMs(uint32_t ms);

#endif /* __BSP_DELAY_H */
