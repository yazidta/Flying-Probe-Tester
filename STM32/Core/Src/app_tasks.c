/*
 * app_tasks.c
 *
 *  Created on: Feb 2, 2025
 *      Author: Ahmed Bouras
 */

#include "app_tasks.h"
#include "tmc2209.h"



QueueHandle_t motorCommandQueue;
MenuState currentState = MENU_STATE_MAIN;



/*-------------------------------------------------------------------
  Global or static variables for the calibration state machine
-------------------------------------------------------------------*/
static CalibrationSubState calibSubState = CALIB_STATE_INIT;
static TickType_t delayStartTime = 0; // For nonblocking delay

/*-------------------------------------------------------------------
  RunCalibrationStateMachine(): Encapsulates the calibration logic.
  Parameters can include pointers to LCD, motors, and any other state
  needed to update the calibration instructions.
-------------------------------------------------------------------*/
void RunCalibrationStateMachine(LCD_I2C_HandleTypeDef *hlcd, Motor *motors) {
    switch (calibSubState)
    {
        case CALIB_STATE_INIT:
            LCD_I2C_Clear(hlcd);
            LCD_I2C_SetCursor(hlcd, 0, 0);
            LCD_I2C_printStr(hlcd, "Use buttons to move");
            LCD_I2C_SetCursor(hlcd, 1, 0);
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
        case CALIB_STATE_INIT:
            LCD_I2C_Clear(hlcd);
            LCD_I2C_SetCursor(hlcd, 0, 0);
            LCD_I2C_printStr(hlcd, "Use buttons to move");
            LCD_I2C_SetCursor(hlcd, 1, 0);
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
                LCD_I2C_printStr(hlcd, "Y-axis edge 1");
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
    /* Yield for a short time so other tasks can run */
    vTaskDelay(pdMS_TO_TICKS(10));
}
void calibProcess(uint8_t calibSelection){
     switch(calibSelection){
        
        case 1: // AUTO
        AutoCalibration(&axes,&motors); 
        //currentState = MENU_STATE_TestProcess; // TODO: Add Test Process
        break;

        case 2: // SEMI ATUO
        semiAutoCalibration(&axes,&motors);
        currentState = MENU_STATE_CALIBRATION2;
        break;

        case 3: // MANUAL
        ManualCalibration(&axes,&motors);
        currentState = MENU_STATE_CALIBRATION3;
        break;
        default:
        break;
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
    		default: // unkown command
    				break;
    		}
    	}

    	taskYIELD();
    }
}



/*
 * FreeRTOS task to handle the main menu.
 * This task will display the menu, wait for a selection,
 * and then call the handler for that selection.
 */
void vMainMenuTask(void *pvParameters)
{
    currentState = MENU_STATE_MAIN;
    MenuTaskParams_t *menuParams = (MenuTaskParams_t *)pvParameters;

    for (;;) {

        switch (currentState) {

            case MENU_STATE_MAIN:
                {
                    // Display main menu and get selection.
                    uint8_t mainSelection = LCD_I2C_MainMenu(menuParams->hlcd, menuParams->henc);
                    if (mainSelection == 1) {
                        currentState = MENU_STATE_SD_TEST;
                    } else if (mainSelection == 2) {
                        currentState = MENU_STATE_PREPARE_MACHINE;
                    } 
                }
                break;

            case MENU_STATE_SD_TEST:
                {
                    // Display SD card test menu or process SD card files.

                    	LCD_I2C_DisplaySDMenu(menuParams->hlcd, menuParams->henc);
                        currentState = MENU_STATE_CALIBRATION;


                        //currentState = MENU_STATE_MAIN;
                }
                break;

            case MENU_STATE_CALIBRATION:
                {
                    const char* calibMenuItems[] = {"Auto Calibartion", "Semi-Auto Calibration", "Manual Calibration" };
                    uint8_t calibSelection = LCD_I2C_menuTemplate(menuParams->hlcd, menuParams->henc,calibMenuItems, 1);
                    if (calibSelection == 0) {  // "Back"
                        currentState = MENU_STATE_MAIN;
                    } else {
                        calibProcess(calibSelection); // Will choose the next state..
                    }
                }
                break;
            case MENU_STATE_CALIBRATION2:
            {       // Semi Auto Calibration
                RunCalibrationStateMachine(&hlcd3, &motors);
                currentState = MENU_STATE_CALIBRATION;     // TO CHANGE
            }

                    
                break;
            case MENU_STATE_CALIBRATION3:
            {
                RunManualCalibrationStateMachine(&hlcd3,&motors);
                currentState = MENU_STATE_CALIBRATION;       //TO CHANGE

            }
                break;

            case MENU_STATE_PREPARE_MACHINE:
                {
                    LCD_I2C_SetCursor(menuParams->hlcd, 0, 0);
                    LCD_I2C_printStr(menuParams->hlcd, "Preparing...");
                    HAL_Delay(2000);
                    currentState = MENU_STATE_MAIN;
                }
                break;
            default:
                currentState = MENU_STATE_MAIN;
                break;
        }

        osDelay(1);  // Allow other tasks to run
    }
}
