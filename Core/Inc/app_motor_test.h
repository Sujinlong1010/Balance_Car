/* app_motor_test.h - two-wheel drive helper (low-level wrapper over TB6612)
 *
 * Applies the per-wheel polarity inversion and drives both wheels from one
 * signed duty. It owns NO state machine and does NOT auto-start: motor
 * ownership belongs to app_balance. Kept so polarity (mirror mounting) is
 * fixed in exactly one place, shared by the balance loop and wiring checks.
 * App_MotorTest_DriveLR() is the three-loop entry point: the controller
 * passes left = vertical - turn, right = vertical + turn.
 *
 * Numbering (schematic): M1 = LEFT wheel, M2 = RIGHT wheel.
 */
#ifndef __APP_MOTOR_TEST_H
#define __APP_MOTOR_TEST_H

#include <stdint.h>
#include "bsp_tb6612.h"

/* Which TB6612 channel drives which wheel. Swap if wired the other way. */
#define APP_MOTOR_LEFT_CHANNEL    TB6612_MOTOR_A
#define APP_MOTOR_RIGHT_CHANNEL   TB6612_MOTOR_B

/* 0 = as wired, 1 = reverse that wheel (mirror mounting). */
#define APP_MOTOR_LEFT_INVERT     0U
#define APP_MOTOR_RIGHT_INVERT    0U

#define APP_MOTOR_DUTY_MAX        7200    /* = TB6612_SPEED_MAX, 1:1 with CCR */

void App_MotorTest_Init(void);
void App_MotorTest_Drive(int16_t duty);   /* -7200 .. +7200, both wheels */
void App_MotorTest_DriveLR(int16_t left_duty, int16_t right_duty); /* per wheel */
void App_MotorTest_Stop(void);
void App_MotorTest_SetPolarity(uint8_t left_inv, uint8_t right_inv);
void App_MotorTest_GetPolarity(uint8_t *left_inv, uint8_t *right_inv);

/* Per-wheel power trim, in percent (50..150, default 100/100).
 * Compensates for weaker motors / drivetrain friction without changing the
 * balance PID. Example: TRIM,120,100 gives the left wheel 20 % more duty. */
#define APP_MOTOR_TRIM_MIN        50U
#define APP_MOTOR_TRIM_MAX       150U
void App_MotorTest_SetTrim(uint8_t left_pct, uint8_t right_pct);
void App_MotorTest_GetTrim(uint8_t *left_pct, uint8_t *right_pct);

#endif /* __APP_MOTOR_TEST_H */
