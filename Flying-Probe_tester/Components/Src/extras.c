/*
 * extras.c
 *
 *  Created on: Nov 16, 2024
 *      Author: yazed
 */


#include "stm32f7xx_hal.h"      // HAL core header
#include "extras.h"
bool IsSensorTriggered(GPIO_TypeDef *sensorPort, uint16_t sensorPin, uint32_t delayMs)
{
    // Read the sensor state
    GPIO_PinState sensor_state = HAL_GPIO_ReadPin(sensorPort, sensorPin);

    // Small delay to avoid button bounce or noise
    HAL_Delay(delayMs);

    // Return true if the sensor is triggered (GPIO_PIN_SET), false otherwise
    return (sensor_state == GPIO_PIN_SET);
}

