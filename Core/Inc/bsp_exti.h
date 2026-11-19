/* bsp_exti.h - single dispatch point for all GPIO EXTI callbacks
 *
 * stm32f1xx_it.c already routes EXTI2 -> HAL_GPIO_EXTI_IRQHandler() and
 * EXTI9_5 -> HAL_GPIO_EXTI_IRQHandler(). HAL_GPIO_EXTI_Callback() is a weak
 * HAL hook, so it is implemented once here (NOT in each sensor driver) to
 * avoid duplicate symbol definitions.
 */
#ifndef __BSP_EXTI_H
#define __BSP_EXTI_H

#include "main.h"

void BSP_EXTI_Init(void);

#endif /* __BSP_EXTI_H */
