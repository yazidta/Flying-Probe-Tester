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



// Function prototypes
void motorControlTask(void *arugment);

// MISC

extern QueueHandle_t motorCommandQueue;


#endif /* INC_APP_TASKS_H_ */
