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
#include "servo.h"
#include "tmc2209.h"


#define CALIB_START_BIT    (1 << 0)
#define CALIB_COMPLETE_BIT (1 << 1)
#define CALIB_STOP_BIT   ( 1 << 2 ) // Test finished
#define TEST_START_BIT    (1 << 0) // Test in progress
#define TEST_STOP_BIT   ( 1 << 1 ) // Test aborted, stall on stall for now.
#define TEST_COMPLETE_BIT   ( 1 << 2 ) // Test finished

#define MAX_CORDS 300


extern EventGroupHandle_t calibEventGroup;
extern EventGroupHandle_t testingEvent;
extern SemaphoreHandle_t lcdMutex;      // Protects LCD access
extern SemaphoreHandle_t xInitSemaphore;
extern SERVO_Handle_TypeDef hservo1;
extern SERVO_Handle_TypeDef hservo2;

// Global calibration selection (set by UI when calibration is picked)
extern volatile uint8_t g_calibSelection;
// COMMANDS
extern float pcbWidth;
extern float pcbHeight;

extern size_t commandsGcode;

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
	MENU_STATE_WELCOME,
    MENU_STATE_MAIN,
    MENU_STATE_SD_TEST,
    MENU_STATE_CALIBRATION,
	MENU_STATE_CALIBRATION2,
	MENU_STATE_CALIBRATION3,
    MENU_STATE_PREPARE_MACHINE,
	MENU_STATE_TESTING,
	MENU_STATE_TEST_ABORTED,
    MENU_STATE_EXIT
} MenuState;

typedef enum {
	MOTOR_CMD_MOVETO,
	MOTOR_CMD_STOP,
	MOTOR_CMD_START,
	MOTOR_CMD_DIRECTION,
	MOTOR_CMD_SETSPEED,
	MOTOR_CMD_CONFIG_MSTEP,
	MOTOR_CMD_CONFIG_SGTHRS,
	MOTOR_CMD_CONFIG_COOLTHRS,
	MOTOR_CMD_CONFIG_CHOPPER,
	MOTOR_CMD_MOVE_ALL_MOTORS,
} MotorCommandType;

typedef struct {
	MotorCommandType command;
	uint8_t axisIndex;		// Index of the axis
	uint8_t motorIndex;		// Index of the motor
	float 	targetPositionMM;	// Target position in mm
	uint8_t direction; 			// Motor direction
	uint16_t speed;				// Motor speed
	uint16_t mstep;				// Motor MSTEP
	uint8_t  sgthrs;			// SGTHRS
	uint16_t coolThrs;			// coolTHRS
	uint8_t  chopper;			// chopper mode (1 = SpreadCycle, 0 = StealthChop)
    float    targetPositionsAxis0[4];
    float    targetPositionsAxis1[2];
} MotorCommand;

typedef struct {
    float x;
    float y;
    uint8_t testResult;
    char netName[20];
} TestPoints;
/* RTOS TASKS */

/* Forward declaration of the external function to read button states */
extern bool read_buttons(void);
void vMainMenuTask(void *pvParameters);
void calibProcessTask(void *pvParameters);
void vTestingTask(void *arugment);
void motorControlTask(void *arugment);
void stallMonitorTask(void *arugment);

// Function prototypes
void ProcessGcode(Axis *axisGroup[], const char *gcodeArray[][MAX_LINE_LENGTH], size_t gcodeCount);
void preformTest();
// MISC

extern QueueHandle_t motorCommandQueue;
extern TestPoints coordinates[MAX_CORDS];


#endif /* INC_APP_TASKS_H_ */
