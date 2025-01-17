/*
 * extras.h
 *
 *  Created on: Nov 16, 2024
 *      Author: yazed
 */

#ifndef INC_EXTRAS_H_
#define INC_EXTRAS_H_
#include <stdbool.h>
#include "servo.h"
#include "stm32f7xx_hal.h"
#include "main.h"
// Include the main HAL header

// Include TIM functions
bool CheckConnection(SERVO_Handle_TypeDef* hservo,SERVO_Handle_TypeDef* hservo1);
bool IsSensorTriggered(GPIO_TypeDef *sensorPort, uint16_t sensorPin);

#endif /* INC_EXTRAS_H_ */
