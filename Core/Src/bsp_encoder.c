/* bsp_encoder.c - quadrature encoder sampling for TIM2 / TIM4
 *
 * The STM32F1 counters are 16-bit. Sampling every 20 ms and casting the
 * unsigned difference to int16_t gives a wrap-safe signed delta as long as
 * the wheel moves less than 32768 ticks between two samples (easily true
 * here: 30 % duty produces a few hundred ticks at most per 20 ms).
 */
#include "bsp_encoder.h"
#include "tim.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

typedef struct
{
  uint16_t last_cnt;
  int32_t  count;
  int32_t  speed_pps;
  int32_t  delta;      /* pulses since the last Update (5 ms @ 200 Hz) */
} EncState_t;

static EncState_t s_left;
static EncState_t s_right;
static uint32_t   s_last_ms = 0U;
static uint8_t    s_started = 0U;
static uint8_t    s_left_inv = (uint8_t)BSP_ENC_LEFT_INVERT;
static uint8_t    s_right_inv = (uint8_t)BSP_ENC_RIGHT_INVERT;

static uint16_t ReadCounter(BSP_Encoder_Channel_t ch)
{
  if (ch == BSP_ENC_LEFT) { return (uint16_t)(TIM2->CNT); }
  return (uint16_t)(TIM4->CNT);
}

static void ZeroCounter(BSP_Encoder_Channel_t ch)
{
  if (ch == BSP_ENC_LEFT) { TIM2->CNT = 0U; }
  else                    { TIM4->CNT = 0U; }
}

static void UpdateOne(EncState_t *st, uint16_t now, uint8_t invert,
                      uint32_t dt_ms)
{
  int16_t delta;
  int32_t d;
  int32_t inst;

  /* Unsigned 16-bit subtraction + int16_t truncation handles 0<->65535
   * wraparound in both directions without any explicit boundary logic. */
  delta = (int16_t)(now - st->last_cnt);
  st->last_cnt = now;

  d = (int32_t)delta;
  if (invert != 0U) { d = -d; }
  st->count += d;
  st->delta  = d;      /* raw 5 ms increment, speed-loop input */

  if (dt_ms > 0U)
  {
    inst = (int32_t)((d * 1000L) / (int32_t)dt_ms);
    /* Light low-pass so a single noisy sample does not spike the readout. */
    st->speed_pps = ((st->speed_pps * 3L) + inst) / 4L;
  }
}

void BSP_Encoder_Init(void)
{
  ZeroCounter(BSP_ENC_LEFT);
  ZeroCounter(BSP_ENC_RIGHT);

  (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  s_left.last_cnt  = ReadCounter(BSP_ENC_LEFT);
  s_right.last_cnt = ReadCounter(BSP_ENC_RIGHT);
  s_left.count     = 0L;
  s_right.count    = 0L;
  s_left.speed_pps = 0L;
  s_right.speed_pps = 0L;
  s_left.delta     = 0L;
  s_right.delta    = 0L;
  s_last_ms        = 0U;
  s_started        = 0U;
}

void BSP_Encoder_Update(uint32_t now_ms)
{
  uint32_t dt;

  if (s_started == 0U)
  {
    /* First call only establishes the time base; no speed yet. */
    s_started = 1U;
    s_last_ms = now_ms;
    s_left.last_cnt  = ReadCounter(BSP_ENC_LEFT);
    s_right.last_cnt = ReadCounter(BSP_ENC_RIGHT);
    s_left.delta     = 0L;
    s_right.delta    = 0L;
    return;
  }

  dt = now_ms - s_last_ms;            /* wrap-safe unsigned subtraction */
  if (dt == 0U)  { return; }
  if (dt > 1000U) { dt = 1000U; }
  s_last_ms = now_ms;

  UpdateOne(&s_left,  ReadCounter(BSP_ENC_LEFT),
            s_left_inv, dt);
  UpdateOne(&s_right, ReadCounter(BSP_ENC_RIGHT),
            s_right_inv, dt);
}

int32_t BSP_Encoder_GetCount(BSP_Encoder_Channel_t ch)
{
  if (ch == BSP_ENC_LEFT) { return s_left.count; }
  return s_right.count;
}

int32_t BSP_Encoder_GetSpeed(BSP_Encoder_Channel_t ch)
{
  if (ch == BSP_ENC_LEFT) { return s_left.speed_pps; }
  return s_right.speed_pps;
}

int32_t BSP_Encoder_GetDelta(BSP_Encoder_Channel_t ch)
{
  if (ch == BSP_ENC_LEFT) { return s_left.delta; }
  return s_right.delta;
}

void BSP_Encoder_Reset(BSP_Encoder_Channel_t ch)
{
  EncState_t *st;

  st = (ch == BSP_ENC_LEFT) ? &s_left : &s_right;
  st->count     = 0L;
  st->speed_pps = 0L;
  st->delta     = 0L;
  st->last_cnt  = ReadCounter(ch);
}

void BSP_Encoder_SetInvert(BSP_Encoder_Channel_t ch, uint8_t inv)
{
  if (ch == BSP_ENC_LEFT) { s_left_inv  = (inv != 0U) ? 1U : 0U; }
  else                    { s_right_inv = (inv != 0U) ? 1U : 0U; }
}

uint8_t BSP_Encoder_GetInvert(BSP_Encoder_Channel_t ch)
{
  if (ch == BSP_ENC_LEFT) { return s_left_inv; }
  return s_right_inv;
}
