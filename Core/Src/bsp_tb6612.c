/* bsp_tb6612.c - TB6612FNG direction pins + TIM1 PWM duty */
#include "bsp_tb6612.h"
#include "tim.h"

extern TIM_HandleTypeDef htim1;

static void TB6612_SetPins(TB6612_Motor_t motor, uint8_t in1, uint8_t in2)
{
  GPIO_PinState s1 = (in1 != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState s2 = (in2 != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  if (motor == TB6612_MOTOR_A)
  {
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, s1);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, s2);
  }
  else
  {
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, s1);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, s2);
  }
}

static void TB6612_SetDuty(TB6612_Motor_t motor, uint32_t pulse)
{
  if (motor == TB6612_MOTOR_A)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
  }
}

static uint32_t TB6612_Period(void)
{
  return (uint32_t)(__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U);
}

void TB6612_Init(void)
{
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  /* TIM1 is an advanced timer: its PWM outputs on PA8/PA11 are gated by
   * BDTR.MOE, so the main output must be enabled explicitly. */
  __HAL_TIM_MOE_ENABLE(&htim1);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);

  TB6612_SetPins(TB6612_MOTOR_A, 0U, 0U);
  TB6612_SetPins(TB6612_MOTOR_B, 0U, 0U);

#if (TB6612_HAS_STBY != 0U)
  HAL_GPIO_WritePin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin, GPIO_PIN_SET);
#endif
}

void TB6612_SetSpeed(TB6612_Motor_t motor, int16_t speed)
{
  uint32_t period;
  uint32_t pulse;
  int32_t  mag;

  if (speed > TB6612_SPEED_MAX)  { speed = TB6612_SPEED_MAX; }
  if (speed < -TB6612_SPEED_MAX) { speed = -TB6612_SPEED_MAX; }

  mag = (speed < 0) ? -(int32_t)speed : (int32_t)speed;
  period = TB6612_Period();
  pulse = ((uint32_t)mag * period) / (uint32_t)TB6612_SPEED_MAX;
  if (pulse > period)
  {
    pulse = period;
  }

  if (speed > 0)
  {
    TB6612_SetPins(motor, 1U, 0U);       /* CW  */
  }
  else if (speed < 0)
  {
    TB6612_SetPins(motor, 0U, 1U);       /* CCW */
  }
  else
  {
    TB6612_SetPins(motor, 0U, 0U);       /* coast */
  }
  TB6612_SetDuty(motor, pulse);
}

void TB6612_Brake(TB6612_Motor_t motor)
{
  TB6612_SetPins(motor, 1U, 1U);
  TB6612_SetDuty(motor, TB6612_Period());
}

void TB6612_Stop(TB6612_Motor_t motor)
{
  TB6612_SetDuty(motor, 0U);
  TB6612_SetPins(motor, 0U, 0U);
}

void TB6612_StopAll(void)
{
  TB6612_Stop(TB6612_MOTOR_A);
  TB6612_Stop(TB6612_MOTOR_B);
}

void TB6612_Standby(uint8_t standby)
{
#if (TB6612_HAS_STBY != 0U)
  HAL_GPIO_WritePin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin,
                    (standby != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
  (void)standby;                 /* STBY tied to 3V3 in hardware */
#endif
}
