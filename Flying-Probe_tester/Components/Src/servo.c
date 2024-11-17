/*
 * servo.c
 *
 *  Created on: Nov 16, 2024
 *      Author: yazed
 */
#include "servo.h"

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"


#define SERVO_MIN_DUTY 	2.5f
#define SERVO_MAX_DUTY 12.5f
/**
  * @brief Initialize PWM output
  * @param[in/out] hpwm   : PWM output handler
  * @retval None
  */

void PWM_Init(PWM_Handle_TypeDef* hpwm)
{
  PWM_WriteDuty(hpwm, hpwm->Duty);
  HAL_TIM_PWM_Start(hpwm->Timer, hpwm->Channel);
}

/**
  * @brief Write PWM duty cycle
  * @param[in/out] hpwm   : PWM output handler
  * @param[in]     duty   : PWM duty cycle in percents (0. - 100.)
  * @retval None
  */
void PWM_WriteDuty(PWM_Handle_TypeDef* hpwm, float duty)
{
  // Saturate duty cycle value
  if(duty < 0.0f)
    duty = 0.0;
  else if(duty > 100.0f)
    duty = 100.0f;
  // Write duty to handle field
  hpwm->Duty = duty;
  // Compute Capture/Compare Register value
  int COMPARE = (duty * (__HAL_TIM_GET_AUTORELOAD(hpwm->Timer)+1)) / 100;
  // Write value to register
  __HAL_TIM_SET_COMPARE(hpwm->Timer, hpwm->Channel, COMPARE);
}

/**
  * @brief Set PWM duty cycle
  * @param[in]     hpwm   : PWM output handler
  * @retval PWM duty cycle in percents (0. - 100.)
  */
float PWM_ReadDuty(const PWM_Handle_TypeDef* hpwm)
{
  return hpwm->Duty;
}
void SERVO_Init(SERVO_Handle_TypeDef* hservo)
{
	SERVO_WritePosition(hservo, 90.0f);
	PWM_Init(&(hservo->PwmOut));
}

void SERVO_WritePosition(SERVO_Handle_TypeDef* hservo, float pos)
{
	hservo->Position = __SATURATION(pos, 0.0f, 180.0f);
	float duty = __LINEAR_TRANSFORM(hservo->Position, 0.0f, 180.0f, SERVO_MIN_DUTY, SERVO_MAX_DUTY);
	PWM_WriteDuty(&(hservo->PwmOut), duty);
}

float SERVO_ReadPosition(SERVO_Handle_TypeDef* hservo)
{
	return hservo->Position;
}

