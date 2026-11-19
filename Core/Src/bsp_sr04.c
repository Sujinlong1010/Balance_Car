/* bsp_sr04.c - non-blocking echo pulse measurement on EXTI2 */
#include "bsp_sr04.h"
#include "bsp_delay.h"

typedef enum
{
  SR04_IDLE = 0,
  SR04_WAIT_RISE,
  SR04_WAIT_FALL,
  SR04_DONE
} SR04_State_t;

static volatile SR04_State_t s_state = SR04_IDLE;
static volatile uint32_t     s_rise_us = 0U;
static volatile uint32_t     s_pulse_us = 0U;
static volatile uint32_t     s_trigger_ms = 0U;

void SR04_Init(void)
{
  HAL_GPIO_WritePin(SR04_Trig_GPIO_Port, SR04_Trig_Pin, GPIO_PIN_RESET);
  s_state = SR04_IDLE;
  s_rise_us = 0U;
  s_pulse_us = 0U;
}

void SR04_Trigger(void)
{
  /* Ignore a previous unfinished measurement and restart cleanly. */
  s_state = SR04_IDLE;
  s_trigger_ms = HAL_GetTick();

  HAL_GPIO_WritePin(SR04_Trig_GPIO_Port, SR04_Trig_Pin, GPIO_PIN_SET);
  BSP_DelayUs(15U);                 /* >= 10 us TTL trigger pulse */
  HAL_GPIO_WritePin(SR04_Trig_GPIO_Port, SR04_Trig_Pin, GPIO_PIN_RESET);

  s_state = SR04_WAIT_RISE;
}

void SR04_EXTI_Callback(uint16_t pin)
{
  uint32_t now;

  if (pin != SR04_Echo_Pin)
  {
    return;
  }

  now = BSP_GetMicros();

  if (HAL_GPIO_ReadPin(SR04_Echo_GPIO_Port, SR04_Echo_Pin) != GPIO_PIN_RESET)
  {
    /* Rising edge: start of the echo pulse. */
    if (s_state == SR04_WAIT_RISE)
    {
      s_rise_us = now;
      s_state = SR04_WAIT_FALL;
    }
  }
  else
  {
    /* Falling edge: end of the echo pulse. */
    if (s_state == SR04_WAIT_FALL)
    {
      s_pulse_us = now - s_rise_us;
      s_state = SR04_DONE;
    }
  }
}

int32_t SR04_GetDistanceMm(void)
{
  uint32_t pulse;

  if (s_state == SR04_DONE)
  {
    pulse = s_pulse_us;
    s_state = SR04_IDLE;            /* one-shot result, consumed here */
    if (pulse > SR04_MAX_PULSE_US)
    {
      return SR04_NO_ECHO;
    }
    /* 340 m/s -> 0.34 mm/us one way, /2 for the round trip = 0.17 mm/us */
    return (int32_t)((pulse * 17U) / 100U);
  }

  if (s_state != SR04_IDLE)
  {
    if ((uint32_t)(HAL_GetTick() - s_trigger_ms) > SR04_TIMEOUT_MS)
    {
      s_state = SR04_IDLE;
      return SR04_NO_ECHO;
    }
    return SR04_BUSY;
  }

  return SR04_NO_NEW_RESULT;
}
