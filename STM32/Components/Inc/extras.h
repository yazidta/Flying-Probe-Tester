/*
 * extras.h
 *
 *  Created on: Nov 16, 2024
 *      Author: yazed
 */

#ifndef INC_EXTRAS_H_
#define INC_EXTRAS_H_
#include <stdbool.h>
#include "stm32f7xx_hal.h"         // Include the main HAL header

// Include TIM functions

bool IsSensorTriggered(GPIO_TypeDef *sensorPort, uint16_t sensorPin, uint32_t delayMs);

#endif /* INC_EXTRAS_H_ */
