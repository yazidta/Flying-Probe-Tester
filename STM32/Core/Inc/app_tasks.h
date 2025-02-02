/*
 * app_tasks.h
 *
 *  Created on: Feb 2, 2025
 *      Author: Abmed Bouras
 */

#ifndef INC_APP_TASKS_H_
#define INC_APP_TASKS_H_
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "encoder.h"
#include "LCD.h"
// COMMANDS

typedef enum {
	MOTOR_CMD_MOVETO,
	MOTOR_CMD_STOP,
	MOTOR_CMD_START,
	MOTOR_CMD_DIRECTION,
} MotorCommandType;

typedef struct {
	MotorCommandType command;
	uint8_t axisIndex;		// Index of the axis
	uint8_t motorIndex;		// Index of the motor
	float 	targetPositionMM;	// Target position in mm
	uint8_t direction; 			// Motor direction
} MotorCommand;

/* Define a structure to hold pointers to our LCD and encoder handles */
typedef struct {
    LCD_I2C_HandleTypeDef* hlcd;
    ENC_Handle_TypeDef* henc;
} MenuTaskParams_t;

/* Forward declaration of the external function to read button states */
extern bool read_buttons(void);
void vMainMenuTask(void *pvParameters);




// Function prototypes
void motorControlTask(void *arugment);

// MISC

extern QueueHandle_t motorCommandQueue;


#endif /* INC_APP_TASKS_H_ */
