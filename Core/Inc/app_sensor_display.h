/* app_sensor_display.h - application layer demo (BSP drivers -> OLED)
 *
 * Scope of this stage: show 3-axis accel (AX/AY/AZ), the 3 XYZ-axis angles
 * (X/Y absolute tilt, Z* relative heading) and SR04 distance on the OLED.
 * Active: HC-05 wireless PID tuning (app_pid_tune) plus a 30 % open-loop
 * forward drive on both wheels (app_motor_test). Both switches below are 1U.
 */
#ifndef __APP_SENSOR_DISPLAY_H
#define __APP_SENSOR_DISPLAY_H

#define APP_ENABLE_HC05      1U   /* HC-05 AT+UART=115200,0,0 must be set once */
#define APP_ENABLE_TB6612    1U   /* STBY assumed tied to 3V3 in hardware */

void App_SensorDisplay_Init(void);   /* call from main.c USER CODE BEGIN 2 */
void App_SensorDisplay_Loop(void);   /* call from a FreeRTOS task, never returns */

#endif /* __APP_SENSOR_DISPLAY_H */
