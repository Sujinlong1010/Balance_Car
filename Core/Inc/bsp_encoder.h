/* bsp_encoder.h - quadrature encoders on TIM2 (left) / TIM4 (right)
 *
 * CubeMX configuration (already generated in tim.c):
 *   TIM2_CH1/CH2 = PA0/PA1  = M1A/M1B = LEFT  wheel, TIM_ENCODERMODE_TI12
 *   TIM4_CH1/CH2 = PB6/PB7  = M2A/M2B = RIGHT wheel, TIM_ENCODERMODE_TI12
 *   PSC = 0, ARR = 65535, IC1/IC2 Filter = 4, inputs pulled up.
 *
 * IMPORTANT: CubeMX only generates HAL_TIM_Encoder_Init(). This driver calls
 * HAL_TIM_Encoder_Start() and samples CNT every 20 ms so that the 16-bit
 * hardware counter can be accumulated into a 32-bit signed position.
 *
 * The wheels are mirror-mounted, so "both forward" usually makes the two
 * counters run in OPPOSITE directions. That is expected: compare the signs
 * of the speeds, not their absolute directions. The invert macros below only
 * change the sign reported by this driver, not the hardware.
 */
#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include <stdint.h>

typedef enum
{
  BSP_ENC_LEFT = 0,     /* M1 / TIM2 / PA0-PA1 */
  BSP_ENC_RIGHT         /* M2 / TIM4 / PB6-PB7 */
} BSP_Encoder_Channel_t;

/* 0 = report raw counter direction, 1 = negate that channel.
 * Measured on this car (2026-11-18): pushing both wheels forward gives a
 * POSITIVE count on the left and a NEGATIVE count on the right (mirror
 * mounting), so the right channel is inverted by default. After this the
 * speed loop sees "forward = positive" on both wheels, exactly like the
 * reference project (which inverts the left one). Keep the ENCS command
 * for re-checking without a reflash. */
#define BSP_ENC_LEFT_INVERT    0U
#define BSP_ENC_RIGHT_INVERT   1U

void    BSP_Encoder_Init(void);
void    BSP_Encoder_Update(uint32_t now_ms);   /* call every 20 ms */
int32_t BSP_Encoder_GetCount(BSP_Encoder_Channel_t ch); /* accumulated ticks */
int32_t BSP_Encoder_GetSpeed(BSP_Encoder_Channel_t ch); /* ticks/s, signed */
/* Raw pulse increment since the previous BSP_Encoder_Update() call.
 * This is what the reference project's Read_Speed() returns (it reads the
 * counter and clears it), and it is the unit the speed PI was tuned in:
 * called from the 200 Hz task it is a 5 ms pulse count. */
int32_t BSP_Encoder_GetDelta(BSP_Encoder_Channel_t ch);
void    BSP_Encoder_Reset(BSP_Encoder_Channel_t ch);
/* Runtime direction fix (wheels are mirror-mounted): 1 = negate that wheel.
 * The ENCS command lets you correct a backwards speed-loop input without
 * reflashing. ENCZ clears the accumulated count after changing it. */
void    BSP_Encoder_SetInvert(BSP_Encoder_Channel_t ch, uint8_t inv);
uint8_t BSP_Encoder_GetInvert(BSP_Encoder_Channel_t ch);

#endif /* __BSP_ENCODER_H */
