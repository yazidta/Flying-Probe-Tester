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
#include "semphr.h"
#include "encoder.h"
#include "LCD.h"
#include "lcd_config.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "encoder_config.h"
#define CALIB_START_BIT    (1 << 0)
#define CALIB_COMPLETE_BIT (1 << 1)

extern EventGroupHandle_t calibEventGroup;
extern SemaphoreHandle_t lcdMutex;      // Protects LCD access

// Global calibration selection (set by UI when calibration is picked)
extern volatile uint8_t g_calibSelection;
// COMMANDS


typedef enum {
    MACHINE_STATE_AUTOCALIB,
    MACHINE_STATE_SEMIAUTO,
    MACHINE_STATE_MANUAL,
	MACHINE_STATE_TESTING

} MachineState;

typedef enum {
    CALIB_STATE_INIT = 0,
    CALIB_STATE_INSTRUCT_PROBE1,
    CALIB_STATE_WAIT_PROBE1_DONE,
    CALIB_STATE_INSTRUCT_PROBE2,
    CALIB_STATE_WAIT_PROBE2_DONE,
    CALIB_STATE_INSTRUCT_PROBE1_Y,
    CALIB_STATE_WAIT_PROBE1_Y_DONE,
    CALIB_STATE_INSTRUCT_PROBE2_X,
    CALIB_STATE_WAIT_PROBE2_X_DONE,
    CALIB_STATE_COMPLETE
} CalibrationSubState;

typedef enum {
    MENU_STATE_MAIN,
    MENU_STATE_SD_TEST,
    MENU_STATE_CALIBRATION,
	MENU_STATE_CALIBRATION2,
	MENU_STATE_CALIBRATION3,
    MENU_STATE_PREPARE_MACHINE,
    MENU_STATE_EXIT
} MenuState;

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
void calibProcessTask(void *pvParameters);




// Function prototypes
void motorControlTask(void *arugment);

// MISC

extern QueueHandle_t motorCommandQueue;


#endif /* INC_APP_TASKS_H_ */
