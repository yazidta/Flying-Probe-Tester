/*
 * extras.c
 *
 *  Created on: Nov 16, 2024
 *      Author: yazed
 */



#include "extras.h"


bool IsSensorTriggered(GPIO_TypeDef *sensorPort, uint16_t sensorPin)
{
    // Read the sensor state
    GPIO_PinState sensor_state = HAL_GPIO_ReadPin(sensorPort, sensorPin);

    // Small delay to avoid button bounce or noise
    if(sensor_state == GPIO_PIN_SET){
    	return true;

    }
    else{
    	return false;
    }
    // Return true if the sensor is triggered (GPIO_PIN_SET), false otherwise

}

