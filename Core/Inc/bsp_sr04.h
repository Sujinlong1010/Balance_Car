/* bsp_sr04.h - HC-SR04 ultrasonic ranger
 *
 * CubeMX: PA3 = Trig (GPIO output), PA2 = Echo (EXTI2, both edges, pull-down).
 * The echo pulse width is measured with the us timestamp from bsp_delay.
 * This module never blocks: call SR04_Trigger(), then poll
 * SR04_GetDistanceMm() from a task. It returns a distance (>=0), or one of
 * the status codes below. SR04_NO_NEW_RESULT means "no fresh result", so the
 * caller can poll continuously without erasing the last valid distance.
 */
#ifndef __BSP_SR04_H
#define __BSP_SR04_H

#include <stdint.h>
#include "main.h"

#define SR04_NO_ECHO        (-1)  /* echo timeout / out of range */
#define SR04_BUSY           (-2)  /* measurement still running */
#define SR04_NO_NEW_RESULT  (-3)  /* idle, previous result already consumed */
#define SR04_TIMEOUT_MS     60U
#define SR04_MAX_PULSE_US  38000U /* ~6.5 m round trip */

void    SR04_Init(void);
void    SR04_Trigger(void);
int32_t SR04_GetDistanceMm(void);
void    SR04_EXTI_Callback(uint16_t pin);

#endif /* __BSP_SR04_H */
