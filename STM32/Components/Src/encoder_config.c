/*
 * encoder_config.c
 *
 *  Created on: Nov 28, 2024
 *      Author: yazed
 */
/* Includes ------------------------------------------------------------------*/
#include "encoder.h"

#ifdef ENC_HARDWARE_COUNTER
#include "stm32f7xx_hal.h"         // Include the main HAL header
#include "stm32f7xx_hal_tim.h"

#else
#include "main.h"
#endif

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
#ifdef ENC_HARDWARE_COUNTER

ENC_Handle_TypeDef henc1 = {
  .Timer = &htim4,
  .Counter  = 0,
  .CounterMax = 400, .CounterMin = 0,
  .CounterInc = 0, .CounterDec = 0,
  .TicksPerStep = 4
};

#else

ENC_Handle_TypeDef henc1 = {
  .CLK_Port = ENC_CLK_GPIO_Port, .CLK_Pin = ENC_CLK_Pin,
  .DT_Port  = ENC_DT_GPIO_Port,  .DT_Pin  = ENC_DT_Pin,
  .Counter  = 0,
  .CounterMax = 400, .CounterMin = 0, .CounterStep = 1,
  .CounterInc = 0, .CounterDec = 0
};

#endif

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/


