/*
 * app_tasks.c
 *
 *  Created on: Feb 2, 2025
 *      Author: Ahmed Bouras
 */

#include "app_tasks.h"
#include "tmc2209.h"


QueueHandle_t motorCommandQueue;

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
    /* Retrieve the task parameters which include the LCD and Encoder handles */
    MenuTaskParams_t *menuParams = (MenuTaskParams_t *)pvParameters;

    if(menuParams == NULL)
    {
        /* If parameters are not provided, delete the task to avoid undefined behavior */
        vTaskDelete(NULL);
    }

    /* Main loop: repeatedly display menu and process the selection */
    for (;;)
    {
        /*
         * The LCD_I2C_MainMenu function is assumed to:
         *   - Clear the screen,
         *   - Display the menu items,
         *   - And block until the user selects an option (using the read_buttons()).
         * It returns a 1-based menu option.
         */
        uint8_t selectedOption = LCD_I2C_MainMenu(menuParams->hlcd, read_buttons);

        /*
         * Handle the selected menu option. This function may include its own delays
         * and even loops (for example, if it is handling a sub-menu for file selection).
         */
        LCD_I2C_HandleMenuSelection(selectedOption, menuParams->hlcd, menuParams->henc);

        /* Optionally, add a short delay before re-displaying the menu */
        //vTaskDelay(pdMS_TO_TICKS(100));
    }
}



