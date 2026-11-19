/* bsp_tb6612.h - TB6612FNG dual motor driver
 *
 * CubeMX: TIM1_CH1 = PA8 (PWMA), TIM1_CH4 = PA11 (PWMB), 10 kHz, ARR = 7199
 *         AIN1 = PB14, AIN2 = PB15, BIN1 = PB13, BIN2 = PB12 (GPIO output)
 *
 * STBY: NOT defined in the current CubeMX project. If the board ties STBY to
 * 3V3 in hardware, leave TB6612_HAS_STBY = 0. If STBY is wired to a free MCU
 * pin (PA9 is unused), set it to 1 below and fill in the port/pin; the pin
 * must also be configured as GPIO_Output in CubeMX with initial level HIGH.
 *
 * POWER (module VCC / STBY are independent of the motor supply VM):
 *   TB6612FNG VCC = 2.7..5.5 V, so powering VCC and tying STBY to either
 *   3.3 V or 5 V is valid and widely used with a 3.3 V STM32 (the LCKFB
 *   Arduino wiring guide explicitly uses 5 V for VCC). Do NOT treat a
 *   5 V STBY as a fault.
 *   VM is a SEPARATE motor supply (battery, < 12 V). VCC present does NOT
 *   mean the H-bridge has power: with VM unconnected the logic works,
 *   encoders and Bluetooth work, but neither motor can ever move.
 *   STM32 GND, module GND and battery - must all be tied together.
 */
#ifndef __BSP_TB6612_H
#define __BSP_TB6612_H

#include <stdint.h>
#include "main.h"

#define TB6612_HAS_STBY  0U

#if (TB6612_HAS_STBY != 0U)
#define TB6612_STBY_GPIO_Port  GPIOA
#define TB6612_STBY_Pin        GPIO_PIN_9
#endif

/* Full scale = TIM1 ARR + 1 = 7200 (PSC 0, 10 kHz), so the speed value maps
 * 1:1 onto CCR exactly like the reference project's PWM_MAX / PWM_MIN = 7200.
 * A speed of 300 therefore means CCR = 300 = 4.17 % duty, same numbers the
 * reference PID gains (Vertical_Kp = 300, Kd = 1.2) were tuned against. */
#define TB6612_SPEED_MAX  7200

typedef enum
{
  TB6612_MOTOR_A = 0,
  TB6612_MOTOR_B
} TB6612_Motor_t;

void TB6612_Init(void);
void TB6612_SetSpeed(TB6612_Motor_t motor, int16_t speed); /* -7200 .. +7200 */
void TB6612_Brake(TB6612_Motor_t motor);
void TB6612_Stop(TB6612_Motor_t motor);
void TB6612_StopAll(void);
void TB6612_Standby(uint8_t standby);   /* 1 = standby (chip off), 0 = active */

#endif /* __BSP_TB6612_H */
