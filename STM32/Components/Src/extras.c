/*
 * extras.c
 *
 *  Created on: Nov 16, 2024
 *      Author: yazed
 */



#include "extras.h"

bool CheckConnection(SERVO_Handle_TypeDef* hservo,SERVO_Handle_TypeDef* hservo1){

	SERVO_WritePosition(hservo, 50);
	SERVO_WritePosition(hservo1, 52);
	HAL_Delay(2000);
	bool x =0;
	if(HAL_GPIO_ReadPin(Probe_GPIO_Port,Probe_Pin) == GPIO_PIN_SET){
	     x = true;
	}
	else{
		 x=false;
	}
	SERVO_WritePosition(hservo, 90);
	SERVO_WritePosition(hservo1, 92);
	return x;
}
bool IsSensorTriggered(GPIO_TypeDef *sensorPort, uint16_t sensorPin)
{
    // Read the sensor state
    GPIO_PinState sensor_state = HAL_GPIO_ReadPin(sensorPort, sensorPin);

    // Small delay to avoid button bounce or noise
    if(sensor_state == GPIO_PIN_SET){
    	return false;

    }
    else{
    	return true;
    }
    // Return true if the sensor is triggered (GPIO_PIN_SET), false otherwise

}

