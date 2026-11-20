/* app_pid_tune.h - HC-05 wireless PID tuning protocol (application layer)
 *
 * Only runtime-tunable items and core balance commands cross the link.
 * Everything else (target speed/turn, angle trim, output sign, fault
 * protection, deadband, motor polarity, encoder direction, wheel trim) is
 * fixed in firmware constants and is NOT tunable over the air.
 *
 * ASCII, LF terminated, x100 fixed point (gains noted otherwise).
 * No float printf on the MCU.
 *   PC  -> MCU : GET / SET,<kp100>,<ki100>,<kd100>   angle-loop gains
 *                GETV / SETV,<kp10000>,<ki10000>     speed-loop gains
 *                GETT / SETT,<kp100>,<ki100>         turn-loop gains (PI)
 *                CAL                 zero-offset capture (hold car still)
 *                BAL,<0|1>           disable / enable the balance loop
 *                BST                 balance status snapshot
 *   MCU -> PC  : ACK,<kp100>,<ki100>,<kd100>
 *                ACKV,<kp10000>,<ki10000>
 *                ACKT,<kp100>,<ki100>
 *                LOG,<text>
 *                BST,<state>,<angle100>,<offset100>,<duty>,
 *                    <p100>,<i100>,<d100>,<cal_progress>,<vel100>,<turn>
 *                T,<ms>,<angle100>,<target100>,<p100>,<i100>,<d100>,
 *                  <pwm100>,<vel100>,<turn>,<enc_tps>,<gyroz100>
 *
 * SET writes the REAL angle-loop gains; SETV/SETT write the speed and turn
 * loops. T/BST stream the real controller terms. No open-loop auto-drive:
 * motors coast on power-up and only move after CAL then BAL,1 is accepted.
 */
#ifndef __APP_PID_TUNE_H
#define __APP_PID_TUNE_H

#include <stdint.h>
#include "bsp_mpu6050.h"

/* Telemetry period. 40 ms = 25 Hz; one frame is about 60 bytes at 115200. */
#define PIDTUNE_TELEMETRY_MS      40U
#define PIDTUNE_LINE_CAP          96U
#define PIDTUNE_TX_CAP            128U

/* Which absolute tilt is used as the controlled angle.
 * 1 = angle_y / pitch (usual for a 2-wheel balance car)
 * 0 = angle_x / roll  (use this if the MPU6050 is mounted rotated 90 deg) */
#define PIDTUNE_ANGLE_FROM_Y      1U

/* On-car tuned defaults (user's best). Speed/turn defaults are read back
 * from app_balance.h; only the angle gains are mirrored here. Keep in sync
 * or App_PIDTune_Init() will overwrite the real gains with stale values.
 *   Angle : Kp=3.30  Ki=0.15  Kd=3.00   (x100)
 *   Speed : Kp=0.80  Ki=0.065           (x10000)
 *   Turn  : Kp=3.00  Ki=4.00            (x100, encoder-difference PI) */
#define PIDTUNE_DEFAULT_KP100     330L     /* 3.30 */
#define PIDTUNE_DEFAULT_KI100     15L      /* 0.15 */
#define PIDTUNE_DEFAULT_KD100     300L     /* 3.00 */

#define PIDTUNE_PARAM_LIMIT       1000000L    /* +/- 10000.00 for KP/KI/KD */
#define PIDTUNE_SPEED_LIMIT       1000000L    /* x10000: +/- 100.0000 */

typedef struct
{
  int32_t kp100;       /* angle loop, x100 */
  int32_t ki100;       /* angle loop, x100 */
  int32_t kd100;       /* angle loop, x100 */
  int32_t kv_p10000;   /* speed loop P, x10000 */
  int32_t kv_i10000;   /* speed loop I, x10000 */
  int32_t kt_p100;     /* turn loop P, x100 (encoder difference PI) */
  int32_t kt_i100;     /* turn loop I, x100 */
} PIDTune_Params_t;

/* Call once after HC05_Init(). Sends a LOG banner over the Bluetooth link. */
void     App_PIDTune_Init(void);

/* Call from the sensor task on every loop (>= 20 ms period).
 * Consumes incoming commands, advances the PID terms and emits telemetry. */
void     App_PIDTune_Poll(uint32_t now_ms, const MPU6050_Data_t *imu,
                          uint8_t imu_ok);

/* Read-only access for the future closed-loop control task. */
void     App_PIDTune_GetParams(PIDTune_Params_t *out);
int32_t  App_PIDTune_GetPwm100(void);

#endif /* __APP_PID_TUNE_H */
