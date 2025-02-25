/*
 * extras.c
 *
 *  Created on: Nov 16, 2024
 *      Author: yazed
 */



#include "extras.h"
bool probe = 0;

bool CheckConnection(SERVO_Handle_TypeDef* hservo,SERVO_Handle_TypeDef* hservo2){

	SERVO_WritePosition(hservo, SERVO1_CHECK_POS);
	SERVO_WritePosition(hservo2, SERVO2_CHECK_POS);
	HAL_Delay(100);
	probe = HAL_GPIO_ReadPin(Probe_GPIO_Port,Probe_Pin);
	SERVO_WritePosition(hservo, SERVO1_HOME_POS);
	SERVO_WritePosition(hservo2, SERVO2_HOME_POS);
//	HAL_Delay(1000);
	return probe;
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

