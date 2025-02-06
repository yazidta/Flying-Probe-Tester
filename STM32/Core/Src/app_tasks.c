/*
 * app_tasks.c
 *
 *  Created on: Feb 2, 2025
 *      Author: Ahmed Bouras
 */

#include "app_tasks.h"



#define STALL_CHECK_INTERVAL_MS 50

QueueHandle_t motorCommandQueue;
MenuState currentState = MENU_STATE_MAIN;



/*-------------------------------------------------------------------
  Global or static variables for the calibration state machine
-------------------------------------------------------------------*/
static CalibrationSubState calibSubState = CALIB_STATE_INIT;
static TickType_t delayStartTime = 0; // For nonblocking delay

EventGroupHandle_t calibEventGroup;
SemaphoreHandle_t lcdMutex;      // Protects LCD access
SemaphoreHandle_t xInitSemaphore;


// Global calibration selection (set by UI when calibration is picked)
volatile uint8_t g_calibSelection = 0;
volatile uint8_t encButton = 0;
/*-------------------------------------------------------------------
  RunCalibrationStateMachine(): Encapsulates the calibration logic.
  Parameters can include pointers to LCD, motors, and any other state
  needed to update the calibration instructions.
-------------------------------------------------------------------*/

void calibProcessTask(void *pvParameters){

	for(;;){
		EventBits_t uxBits = xEventGroupWaitBits(calibEventGroup, CALIB_START_BIT,
		                                                   pdTRUE, pdFALSE, portMAX_DELAY);
	if (uxBits & CALIB_START_BIT) {

     switch(g_calibSelection){
        
        case 1: // AUTO
        AutoCalibration(&axes,&motors); 
        //currentState = MENU_STATE_TestProcess; // TODO: Add Test Process
        break;

        case 2: // SEMI ATUO
        RunSemiAutoCalibrationStateMachine(&hlcd3,&motors);
        currentState = MENU_STATE_CALIBRATION2;
        break;

        case 3: // MANUAL
        RunManualCalibrationStateMachine(&hlcd3, &motors);
        currentState = MENU_STATE_CALIBRATION3;
        break;
        default:
        break;

        //  update the LCD: "Calibration complete"
        if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            LCD_I2C_Clear(&hlcd3);
            LCD_I2C_SetCursor(&hlcd3, 0, 0);
            LCD_I2C_printStr(&hlcd3, "Calibration Done");
            xSemaphoreGive(lcdMutex);
        }

        // Signal that calibration is complete.
        xEventGroupSetBits(calibEventGroup, CALIB_COMPLETE_BIT);
    }
        // Short delay to let other tasks run.

       }
		vTaskDelay(pdMS_TO_TICKS(10));
   }
}


void motorControlTask(void *argument) {
		// Queue for motor cmds
	motorCommandQueue = xQueueCreate(10, sizeof(MotorCommand));
	configASSERT(motorCommandQueue != NULL);

	MotorCommand cmd;

    for(;;) {
    // Wait for a command from the queue
    	if(xQueueReceive(motorCommandQueue, &cmd, portMAX_DELAY) == pdPASS){
    		switch (cmd.command){

    		case	MOTOR_CMD_START: // Start the motor
    				TMC2209_Start(&motors[cmd.motorIndex]);
    				break;

    		case	MOTOR_CMD_MOVETO: // Move the motor to a target position
    				TMC2209_MoveTo(&axes[cmd.axisIndex], cmd.motorIndex, cmd.targetPositionMM);
    				break;

    		case	MOTOR_CMD_STOP:	// Stop the motor
    				TMC2209_Stop(&motors[cmd.motorIndex]);

    				break;

    		case 	MOTOR_CMD_DIRECTION:
    				TMC2209_SetDirection(&motors[cmd.motorIndex], cmd.direction);
    				break;

    		case 	MOTOR_CMD_SETSPEED:
    				TMC2209_SetSpeed(&motors[cmd.motorIndex], cmd.speed);
    				break;

    		case 	MOTOR_CMD_CONFIG_MSTEP:
    				TMC2209_setMicrosteppingResolution(&motors[cmd.motorIndex], cmd.mstep);
    				break;

    		case 	MOTOR_CMD_CONFIG_CHOPPER:
    			    TMC2209_setSpreadCycle(&motors[cmd.motorIndex], cmd.chopper);
    			    break;

    		case	MOTOR_CMD_CONFIG_SGTHRS:
    				TMC2209_enableStallDetection(&motors[cmd.motorIndex], cmd.sgthrs);
    				break;

    		case 	MOTOR_CMD_CONFIG_COOLTHRS:
    				TMC2209_SetTCoolThrs(&motors[cmd.motorIndex], cmd.coolThrs);
    				break;

//    		case 	MOTOR_CMD_CHECK_SPEED:
//
//    		case	MOTOR_CMD_CHECK_MSTEP:
//    		case 	MOTOR_CMD_CHECK_CHOPPER:
//    		case	MOTOR_CMD_CHECK_SGTHRS:
//    		case 	MOTOR_CMD_CHECK_COOLTHRS:
//    				break;
    		default: // unkown command
    				break;
    		}
    	}



    	taskYIELD();
    }
}


/*
 * Stall Monitor Task
 * This task will check the diag pin of each motor and send a stop command for the motor that stalled.
 */
void stallMonitorTask(void *argument) {
	MotorCommand stallCmd;
	stallCmd.command = MOTOR_CMD_STOP;

    for(;;) {
        for(int i = 0; i < MAX_MOTORS; i++) {
        	motors[i].STALL = HAL_GPIO_ReadPin(motors[i].driver.diag_port, motors[i].driver.diag_pin);

            if(motors[i].STALL == GPIO_PIN_SET) {  // Stall detected
                stallCmd.motorIndex = i;
            xQueueSend(motorCommandQueue, &stallCmd, pdMS_TO_TICKS(10));

            }
        }
        vTaskDelay(pdMS_TO_TICKS(STALL_CHECK_INTERVAL_MS));
    }
}



/*
 * FreeRTOS task to handle the main menu.
 * This task will display the menu, wait for a selection,
 * and then call the handler for that selection.
 */
void vMainMenuTask(void *pvParameters)
{
    currentState = MENU_STATE_WELCOME;
    for (;;) {

        switch (currentState) {
        	case MENU_STATE_WELCOME:
        	{
        		LCD_I2C_DisplaySequentialGlossyText(&hlcd3, 2);
        		osDelay(500);
        		LCD_I2C_ClearAllLines(&hlcd3);
        		LCD_I2C_SetCursor(&hlcd3, 0, 1);
        		LCD_I2C_printStr(&hlcd3, "Setting up machine, please wait...");
        		initializeSystem();
                // Wait here until the initialization semaphore is given.
                if (xSemaphoreTake(xInitSemaphore, portMAX_DELAY) == pdTRUE) {
                    currentState = MENU_STATE_MAIN;
                }
        }
        	break;

            case MENU_STATE_MAIN:
                {
                    // Display main menu and get selection.
                	const char* menuItems[] = {"Test from SD", "Prepare Machine"};
                    uint8_t mainSelection = LCD_I2C_menuTemplate(&hlcd3, &henc1, menuItems,2, 0);
                    if (mainSelection == 0) {
                        currentState = MENU_STATE_SD_TEST;
                    } else if (mainSelection == 1) {
                        currentState = MENU_STATE_PREPARE_MACHINE;
                    } 
                }
                break;

            case MENU_STATE_SD_TEST:
                {
                    // Display SD card test menu or process SD card files.

                       LCD_I2C_DisplaySDMenu(&hlcd3, &henc1);
                       currentState = MENU_STATE_CALIBRATION;


                        //currentState = MENU_STATE_MAIN;
                }
                break;

            case MENU_STATE_CALIBRATION:
                {
                    const char* calibMenuItems[] = {"Auto Calibartion", "Semi-Auto Calibration", "Manual Calibration" };
                    uint8_t calibSelection = LCD_I2C_menuTemplate(&hlcd3, &henc1,calibMenuItems,3, 1);

                    if (calibSelection == 0) {  // "Back"
                        currentState = MENU_STATE_MAIN;
                    } else {
                    	g_calibSelection = calibSelection;
                         //Signal the calibration task to start.
                    	xEventGroupSetBits(calibEventGroup, CALIB_START_BIT);
                    	 //Wait for calibration to complete.
                    	 //(You might use a timeout here if desired.)
                    	xEventGroupWaitBits(calibEventGroup, CALIB_COMPLETE_BIT,
                    	                    pdTRUE, pdFALSE, portMAX_DELAY);

                         //Calibration is complete. Return to the main menu or update as needed.
                        currentState = MENU_STATE_MAIN;
                    }
                }
                break;


            case MENU_STATE_PREPARE_MACHINE:
                {
                	LCD_I2C_ClearAllLines(&hlcd3);
                	LCD_I2C_SetCursor(&hlcd3, 0, 1);
                    LCD_I2C_printStr(&hlcd3, "Preparing...");
                    if(MotorsHoming(&motors) == 1){
                       if(calibrationState()){
                           currentState =MENU_STATE_CALIBRATION;
                        }
                       currentState = MENU_STATE_MAIN;
   	                 }

                 }

                break;
            default:
                currentState = MENU_STATE_MAIN;
                break;
        }

        osDelay(1);  // Allow other tasks to run
    }
}



//// FUNCTIONS //////

void ProcessGcode(Axis *axisGroup[], const char *gcodeArray[], size_t gcodeCount) {

	MotorCommand cmd;
    // Variables to hold PCB dimensions
    float pcbWidth = 0.0f, pcbHeight = 0.0f;

    for (size_t i = 0; i < gcodeCount; i++) {
        const char *line = gcodeArray[i];

        if (line[0] == ';') {
            if (strncmp(line, "; G54", 5) == 0) { // G54: actual PCB dimensions. Format G54 X.. Y..

                const char *ptr = strchr(line, 'X');
                if (ptr) {
                    pcbWidth = (float)atof(ptr + 1);
                }

                ptr = strchr(line, 'Y');
                if (ptr) {
                    pcbHeight = (float)atof(ptr + 1);
                }
            }
            continue;
        }

        if (strncmp(line, "G0", 2) == 0) { // G0: move command
            float xTarget = 0.0f, yTarget = 0.0f, zTarget = 0.0f;
            uint8_t probe = (strstr(line, "P1") != NULL) ? 1 : ((strstr(line, "P2") != NULL) ? 2 : 0); // P1 or P2
            // Extract axis coordinates from the line
             const char *ptr = strchr(line, 'X');
             if (ptr) xTarget = (float)atof(ptr + 1);
              	  ptr = strchr(line, 'Y');
             if (ptr) yTarget = (float)atof(ptr + 1);

             int motorIndex = (probe == 2) ? 1 : 0; // Select motor index based on P1 or P2
             // Send MoveTo commands
            if (xTarget >= 0) {
                cmd.motorIndex = motorIndex;
                cmd.axisIndex = 1;
                cmd.targetPositionMM = xTarget;
                cmd.command = MOTOR_CMD_MOVETO;
                xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
            }
            if (yTarget >= 0) {
                cmd.motorIndex = motorIndex;
                cmd.axisIndex = 0;
                cmd.targetPositionMM = yTarget;
                cmd.command = MOTOR_CMD_MOVETO;
                xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
            }

        }

        else if (strncmp(line, "T1", 2) == 0) { // T : perform test
        	CheckConnection(&hservo1, &hservo2);
        }

        else if (line[0] == 'M') {
            if (strncmp(line, "M30", 3) == 0) { // M30: end of program
                break;  // End processing
            }
        }
        // TODO: Some delay?
    }

}


void RunSemiAutoCalibrationStateMachine(LCD_I2C_HandleTypeDef *hlcd, Motor *motors) {
    semiAutoCalibration(&axes,&motors);

	switch (calibSubState)
    {

        case CALIB_STATE_INIT:
            LCD_I2C_Clear(hlcd);
            LCD_I2C_SetCursor(hlcd, 0, 1);
            LCD_I2C_printStr(hlcd, "cheetosckjrcjo to move");
            LCD_I2C_SetCursor(hlcd, 1, 1);
            LCD_I2C_printStr(hlcd, "probe 1");
            delayStartTime = xTaskGetTickCount();
            calibSubState = CALIB_STATE_INSTRUCT_PROBE1;
            break;

        case CALIB_STATE_INSTRUCT_PROBE1:
            // Wait 2000ms nonblocking
            if ((xTaskGetTickCount() - delayStartTime) >= pdMS_TO_TICKS(2000))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 0);
                LCD_I2C_printStr(hlcd, "Place probe 1 facing");
                LCD_I2C_SetCursor(hlcd, 1, 0);
                LCD_I2C_printStr(hlcd, "X-axis edge 1");
                calibSubState = CALIB_STATE_WAIT_PROBE1_DONE;
            }
            break;

        case CALIB_STATE_WAIT_PROBE1_DONE:
            if (Iscalib1Done(&motors[0], &motors[2]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 0);
                LCD_I2C_printStr(hlcd, "Place probe 2 facing");
                LCD_I2C_SetCursor(hlcd, 1, 0);
                LCD_I2C_printStr(hlcd, "Y-axis edge 1");
                calibSubState = CALIB_STATE_INSTRUCT_PROBE2;
            }
            break;

        case CALIB_STATE_INSTRUCT_PROBE2:
            if (Iscalib1Done(&motors[1], &motors[3]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 0);
                LCD_I2C_printStr(hlcd, "Place probe 1 facing");
                LCD_I2C_SetCursor(hlcd, 1, 0);
                LCD_I2C_printStr(hlcd, "Y-axis edge 1");
                calibSubState = CALIB_STATE_WAIT_PROBE1_Y_DONE;
            }
            break;

        case CALIB_STATE_WAIT_PROBE1_Y_DONE:
            if (Iscalib2Done(&motors[0], &motors[2]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 0);
                LCD_I2C_printStr(hlcd, "Place probe 2 facing");
                LCD_I2C_SetCursor(hlcd, 1, 0);
                LCD_I2C_printStr(hlcd, "X-axis edge 2");
                calibSubState = CALIB_STATE_WAIT_PROBE2_X_DONE;
            }
            break;

        case CALIB_STATE_WAIT_PROBE2_X_DONE:
            if (Iscalib2Done(&motors[1], &motors[3]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 0);
                LCD_I2C_printStr(hlcd, "Points saved");
                calibSubState = CALIB_STATE_COMPLETE;
            }
            break;

        case CALIB_STATE_COMPLETE:
            // You may change to the next overall state here.
            // For example: currentState = MENU_STATE_CALIBRATION3;
            // Reset the substate for future calibration sessions.
            calibSubState = CALIB_STATE_INIT;
            break;

        default:
            break;
    }
}


void RunManualCalibrationStateMachine(LCD_I2C_HandleTypeDef *hlcd, Motor *motors) {
    switch (calibSubState)
    {
        ManualCalibration(&axes,&motors);

        case CALIB_STATE_INIT:
            LCD_I2C_Clear(hlcd);
            LCD_I2C_SetCursor(hlcd, 0, 1);
            LCD_I2C_printStr(hlcd, "Use buttons to move");
            LCD_I2C_SetCursor(hlcd, 1, 1);
            LCD_I2C_printStr(hlcd, "probe 1");
            delayStartTime = xTaskGetTickCount();
            calibSubState = CALIB_STATE_INSTRUCT_PROBE1;
            break;

        case CALIB_STATE_INSTRUCT_PROBE1:
            // Wait 2000ms nonblocking
            if ((xTaskGetTickCount() - delayStartTime) >= pdMS_TO_TICKS(2000))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 0);
                LCD_I2C_printStr(hlcd, "Place probe 1 on");
                LCD_I2C_SetCursor(hlcd, 1, 0);
                LCD_I2C_printStr(hlcd, "X-axis edge 1");
                calibSubState = CALIB_STATE_WAIT_PROBE1_DONE;
            }
            break;

        case CALIB_STATE_WAIT_PROBE1_DONE:
            if (Iscalib1Done(&motors[0], &motors[2]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 0);
                LCD_I2C_printStr(hlcd, "Place probe 2 on");
                LCD_I2C_SetCursor(hlcd, 1, 0);
                LCD_I2C_printStr(hlcd, "Y-axis edge 1");
                calibSubState = CALIB_STATE_INSTRUCT_PROBE2;
            }
            break;

        case CALIB_STATE_INSTRUCT_PROBE2:
            if (Iscalib1Done(&motors[1], &motors[3]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 0);
                LCD_I2C_printStr(hlcd, "Place probe 1 on");
                LCD_I2C_SetCursor(hlcd, 1, 0);
                LCD_I2C_printStr(hlcd, "x-axis edge 2");
                calibSubState = CALIB_STATE_WAIT_PROBE1_Y_DONE;
            }
            break;

        case CALIB_STATE_WAIT_PROBE1_Y_DONE:
            if (Iscalib2Done(&motors[0], &motors[2]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 0);
                LCD_I2C_printStr(hlcd, "Place probe 2 on");
                LCD_I2C_SetCursor(hlcd, 1, 0);
                LCD_I2C_printStr(hlcd, "y-axis edge 2");
                calibSubState = CALIB_STATE_WAIT_PROBE2_X_DONE;
            }
            break;

        case CALIB_STATE_WAIT_PROBE2_X_DONE:
            if (Iscalib2Done(&motors[1], &motors[3]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 0);
                LCD_I2C_printStr(hlcd, "Points saved");
                calibSubState = CALIB_STATE_COMPLETE;
            }
            break;

        case CALIB_STATE_COMPLETE:
            // You may change to the next overall state here.
            // For example: currentState = MENU_STATE_CALIBRATION3;
            // Reset the substate for future calibration sessions.
            calibSubState = CALIB_STATE_INIT;
            break;

        default:
            break;
    }
    /* Yield for a short time so other tasks can run */
    vTaskDelay(pdMS_TO_TICKS(10));
}
