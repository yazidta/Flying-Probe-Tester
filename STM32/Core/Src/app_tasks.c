/*
 * app_tasks.c
 *
 *  Created on: Feb 2, 2025
 *      Author: Ahmed Bouras
 */

#include "app_tasks.h"



#define STALL_CHECK_INTERVAL_MS 50

QueueHandle_t motorCommandQueue;
MenuState currentState = MENU_STATE_WELCOME;

extern SERVO_Handle_TypeDef hservo1;
extern SERVO_Handle_TypeDef hservo2;
/*-------------------------------------------------------------------
  Global or static variables for the calibration state machine
-------------------------------------------------------------------*/
static CalibrationSubState calibSubState = CALIB_STATE_INIT;
static TickType_t delayStartTime = 0; // For nonblocking delay

EventGroupHandle_t calibEventGroup;
EventGroupHandle_t testingEvent;
SemaphoreHandle_t lcdMutex;      // Protects LCD access
SemaphoreHandle_t xInitSemaphore;


// Global calibration selection (set by UI when calibration is picked)
volatile uint8_t g_calibSelection = 0;
volatile uint8_t encButton = 0;
float pcbWidth = 0.0f, pcbHeight = 0.0f;
TestPoints coordinates[MAX_CORDS];
size_t commandsGcode = 0;
size_t testResultsCount = 0;

/*-------------------------------------------------------------------
  RunCalibrationStateMachine(): Encapsulates the calibration logic.
  Parameters can include pointers to LCD, motors, and any other state
  needed to update the calibration instructions.
-------------------------------------------------------------------*/

void calibProcessTask(void *pvParameters){

	for(;;){
		EventBits_t uxBits = xEventGroupWaitBits(calibEventGroup, CALIB_START_BIT | CALIB_STOP_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    if(uxBits & CALIB_STOP_BIT) return; // CALIB ABORTED TODO: Display calib aborted on LCD
	if (uxBits & CALIB_START_BIT) {

     switch(g_calibSelection){
        
        case 1: // AUTO
        AutoCalibration(&axes,&motors); 
        xEventGroupSetBits(calibEventGroup, CALIB_COMPLETE_BIT);

        break;

        case 2: // Manual
        	ManualCalibration(&axes,&motors);
            xEventGroupSetBits(calibEventGroup, CALIB_COMPLETE_BIT);
            currentState = MENU_STATE_TESTING; // TODO: Add Test Process
        break;

//        case 3: // Semi-Auto
//
//            //semiAutoCalibration(&axes,&motors);
//                       // xEventGroupSetBits(calibEventGroup, CALIB_COMPLETE_BIT);
//        //RunManualCalibrationStateMachine(&hlcd3, &motors);
//        currentState = MENU_STATE_TESTING; // TODO: Add Test Process
//        break;
        default:
        break;

//        //  update the LCD: "Calibration complete"
//        if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
//            LCD_I2C_Clear(&hlcd3);
//            LCD_I2C_SetCursor(&hlcd3, 0, 0);
//            LCD_I2C_printStr(&hlcd3, "Calibration Done");
//            xSemaphoreGive(lcdMutex);
//        }

    }
        // Short delay to let other tasks run.
     xEventGroupSetBits(calibEventGroup, CALIB_COMPLETE_BIT);

       }
		vTaskDelay(pdMS_TO_TICKS(10));
   }
}


void motorControlTask(void *argument) {
		// Queue for motor cmds
	motorCommandQueue = xQueueCreate(256, sizeof(MotorCommand));
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
       		case 	MOTOR_CMD_MOVE_ALL_MOTORS:
       		{
       			   // Move all motors on axis 0 concurrently.
       			TMC2209_MoveAllMotorsTo(&axes, cmd.targetPositionsAxis0);

       			    break;
       		}

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

    for(;;) {
        for(int i = 0; i < MAX_MOTORS; i++) {
        	motors[i].STALL = HAL_GPIO_ReadPin(motors[i].driver.diag_port, motors[i].driver.diag_pin);

            if(motors[i].STALL == GPIO_PIN_SET) {  // Stall detecte

                xEventGroupSetBits(testingEvent, TEST_STOP_BIT); // Abort Testing task
                TMC2209_Stop(&motors[i]); // Stop stalled motor first
                for(int j = 0; j<MAX_MOTORS; j++){ // Stop the other motors
                   if(j != i) TMC2209_Stop(&motors[j]);

                }
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
                	  LCD_I2C_Clear(&hlcd3);
                	  LCD_I2C_SetCursor(&hlcd3, 0, 1);
                	  LCD_I2C_printStr(&hlcd3, "Performing Tests");
                      LCD_I2C_DisplaySDMenu(&hlcd3, &henc1);

                       size_t numLines = sizeof(lines);
                       ProcessGcode(&axes, &lines, numLines);
            	 currentState = MENU_STATE_CALIBRATION;


                        //currentState = MENU_STATE_MAIN;
                }
                break;

            case MENU_STATE_CALIBRATION:
                {
                    const char* calibMenuItems[] = {"Auto Calibartion", "Manual Calibration" };
                    uint8_t calibSelection = LCD_I2C_menuTemplate(&hlcd3, &henc1,calibMenuItems,2, 1);

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
                        currentState = MENU_STATE_TESTING;
                    }
                }
                break;


            case MENU_STATE_PREPARE_MACHINE:
                {
                	LCD_I2C_ClearAllLines(&hlcd3);
                	LCD_I2C_SetCursor(&hlcd3, 0, 1);
                    LCD_I2C_printStr(&hlcd3, "Preparing...");
                    if(MotorsHoming(&motors) == 1){
                       if(!calibrationState()){
                           currentState =MENU_STATE_CALIBRATION;
                        }
                       currentState = MENU_STATE_MAIN;
   	                 }

                 }
            case MENU_STATE_TESTING:
            {
            	// TDOD: MENU FOR TESTING -- SHOW PROGRESS OF TESTING
            	xEventGroupSetBits(testingEvent, TEST_START_BIT); // Start Testing task
            	xEventGroupWaitBits(testingEvent, TEST_COMPLETE_BIT, pdTRUE, pdFALSE, portMAX_DELAY); // Test Finished.
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

void vTestingTask(void *arugment){
	for(;;){

		EventBits_t testingBits = xEventGroupWaitBits(testingEvent, TEST_START_BIT | TEST_STOP_BIT,
		                                                   pdTRUE, pdFALSE, portMAX_DELAY); // Trigger testing
		if(testingBits & TEST_STOP_BIT){ // Stall detected during test

			continue;	// Abort test
		}
		if (testingBits & TEST_START_BIT) { // Start Test

		preformTest();
		xEventGroupSetBits(testingEvent, TEST_COMPLETE_BIT);
       // xEventGroupClearBits(testingEvent, TEST_STOP_BIT); // clear bit incase we want to restart test

		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}


}



//// FUNCTIONS //////
void preformTest(){

	MotorCommand testingCMD;
	uint16_t j = 0;
		const char reportFilename = {"results.txt"};
	for(int i = 0; i < commandsGcode; i++){
		if(i % 2 == 0){
			testingCMD.targetPositionsAxis0[2] = coordinates[i].x;
			testingCMD.targetPositionsAxis0[0] = coordinates[i].y;
		}
		else{
			testingCMD.targetPositionsAxis0[3] = coordinates[i].x;
			testingCMD.targetPositionsAxis0[1] = coordinates[i].y;
		}
		if(i >= 1 && (i+1)%2 == 0){
		testingCMD.command = MOTOR_CMD_MOVE_ALL_MOTORS;
		xQueueSend(motorCommandQueue, &testingCMD, portMAX_DELAY);
		 coordinates[j].testResult = CheckConnection(&hservo1,&hservo2);
		        j++;
		}
	}
	generate_report(&hlcd3);

	MotorsHoming(&motors);

		//osDelay(2000);

}


void ProcessGcode(Axis *axisGroup[], const char *gcodeArray[][MAX_LINE_LENGTH], size_t gcodeCount) {

    // Variables to hold PCB dimensions
	uint16_t netTestCount = 0;
	    int inNetBlock = 0;
	    size_t currentNetIndex = 0;

	    for(size_t i = 0; i < gcodeCount; i++) {
	            const char *line = gcodeArray[i];
	        // Check for a net definition line.



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
                else{
                	return;
                }
            }
            else{
            	return;
            }
        if (strncmp(line, "; Net:", 6) == 0) {

        	const char *netName = strchr(line, 'Net-(');

        		if(netName != NULL){
              	  //netName = strlen(line);
              	  size_t len = strcspn(netName,"\r\n");
              	  if( len >= 20){
              		  len = 20-1;
              	  }
        			strncpy(coordinates[commandsGcode].netName, netName, len);
        			coordinates[commandsGcode].netName[len] ='\0';
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

             uint8_t motorIndex = (probe == 2) ? 1 : 0; // Select motor index based on P1 or P2
             // Send MoveTo commands
            if (xTarget >= 0) {
                //cmd.motorIndex = motorIndex;
               // cmd.axisIndex = 1;

                if(motorIndex == 0){
                	coordinates[commandsGcode].x = -xTarget;
            }
                else{
                	coordinates[commandsGcode].x = xTarget;
           }
                }
                //cmd.command = MOTOR_CMD_MOVETO;
                //xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

            if (yTarget >= 0) {
               // cmd.motorIndex = motorIndex;
               // cmd.axisIndex = 0;
                if(motorIndex == 0){
                	coordinates[commandsGcode].y = yTarget;
                	commandsGcode++;
                }
                else{
                	coordinates[commandsGcode].y = yTarget;
                	commandsGcode++;
              }
                }
        }
//          if(i == gcodeCount - 1){
//        	   num_elements = i;
//
//          }

        else if (strncmp(line, "T1", 2) == 0) { // T : perform test
        	testResultsCount++;
        	//inNetBlock = 0;
        //CheckConnection(&hservo1, &hservo2);
        }

        else if (line[0] == 'M') {
            if (strncmp(line, "M30", 3) == 0) { // M30: end of program
                break;  // End processing
            }
            //vTaskDelay(100);
        }
        // TODO: Some delay?
    }
	    commandsGcode++;
	    coordinates[commandsGcode].x = 1.5f;
	    coordinates[commandsGcode].y = 1.5f;
	    commandsGcode++;
	    coordinates[commandsGcode].x = 1.5f;
	    coordinates[commandsGcode].y = 1.5f;

}
    //function to generate a report file on the SD card.
    //
    // Parameters:
    //   hlcd         - pointer to your LCD handle (for error messages)
    //   gcodeLines   - a 2D array holding the previously‐read G‑code file lines
    //   lineCount    - the number of lines in gcodeLines
    //   reportFilename - the name of the file to create (for example "report.txt")
    //
    // The report file will have a header and one line per “net” in the following format:
    //
    //   Net           Test Points                    Test result
    //   Net-(R11-Pad2) (37.5995,20.5995); (20.5995,44.5995)  PASS
    //------------------------------------------------------------------------------


void RunSemiAutoCalibrationStateMachine(LCD_I2C_HandleTypeDef *hlcd, Motor *motors) {
    //semiAutoCalibration(&axes,&motors);

	switch (calibSubState)
    {

        case CALIB_STATE_INIT:
            LCD_I2C_Clear(hlcd);
            LCD_I2C_SetCursor(hlcd, 0, 1);
            LCD_I2C_printStr(hlcd, "Use Buttons to move");
            LCD_I2C_printStr(hlcd, "probe 1");
            osDelay(500);
            calibSubState = CALIB_STATE_INSTRUCT_PROBE1;
            break;

        case CALIB_STATE_INSTRUCT_PROBE1:
            // Wait 2000ms nonblocking

                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 1);
                LCD_I2C_printStr(hlcd, "Place probe 1 facing");
                LCD_I2C_printStr(hlcd, "X-axis edge 1");
                calibSubState = CALIB_STATE_WAIT_PROBE1_DONE;

            break;

        case CALIB_STATE_WAIT_PROBE1_DONE:
            if (Iscalib1Done(&motors[0], &motors[2]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 1);
                LCD_I2C_printStr(hlcd, "Place probe 2 facing");
                LCD_I2C_printStr(hlcd, "Y-axis edge 1");
                calibSubState = CALIB_STATE_INSTRUCT_PROBE2;
            }
            break;

        case CALIB_STATE_INSTRUCT_PROBE2:
            if (Iscalib1Done(&motors[1], &motors[3]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 1);
                LCD_I2C_printStr(hlcd, "Place probe 1 facing");
                LCD_I2C_printStr(hlcd, "Y-axis edge 1");
                calibSubState = CALIB_STATE_WAIT_PROBE1_Y_DONE;
            }
            break;

        case CALIB_STATE_WAIT_PROBE1_Y_DONE:
            if (Iscalib2Done(&motors[0], &motors[2]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 1);
                LCD_I2C_printStr(hlcd, "Place probe 2 facing");
                LCD_I2C_printStr(hlcd, "X-axis edge 2");
                calibSubState = CALIB_STATE_WAIT_PROBE2_X_DONE;
            }
            break;

        case CALIB_STATE_WAIT_PROBE2_X_DONE:
            if (Iscalib2Done(&motors[1], &motors[3]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 1);
                LCD_I2C_printStr(hlcd, "Points saved");
                calibSubState = CALIB_STATE_COMPLETE;
            }
            break;

        case CALIB_STATE_COMPLETE:
        {
            currentState = MENU_STATE_TESTING;
            // Reset the substate for future calibration sessions.
            calibSubState = CALIB_STATE_INIT;
        }
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
            LCD_I2C_SetCursor(hlcd, 0, 1);
            LCD_I2C_printStr(hlcd, "Use buttons to move");
            LCD_I2C_printStr(hlcd, "probe 1");
            osDelay(300);
            calibSubState = CALIB_STATE_INSTRUCT_PROBE1;
            break;

        case CALIB_STATE_INSTRUCT_PROBE1:
        {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 1);
                LCD_I2C_printStr(hlcd, "Place probe 1 on");
                LCD_I2C_printStr(hlcd, "X-axis edge 1");
                calibSubState = CALIB_STATE_WAIT_PROBE1_DONE;
        }
            break;

        case CALIB_STATE_WAIT_PROBE1_DONE:
            if (Iscalib1Done(&motors[0], &motors[2]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 1);
                LCD_I2C_printStr(hlcd, "Place probe 2 on");
                LCD_I2C_printStr(hlcd, "Y-axis edge 1");
                calibSubState = CALIB_STATE_INSTRUCT_PROBE2;
            }
            break;

        case CALIB_STATE_INSTRUCT_PROBE2:
            if (Iscalib1Done(&motors[1], &motors[3]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 1);
                LCD_I2C_printStr(hlcd, "Place probe 1 on");
                LCD_I2C_printStr(hlcd, "x-axis edge 2");
                calibSubState = CALIB_STATE_WAIT_PROBE1_Y_DONE;
            }
            break;

        case CALIB_STATE_WAIT_PROBE1_Y_DONE:
            if (Iscalib2Done(&motors[0], &motors[2]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 1);
                LCD_I2C_printStr(hlcd, "Place probe 2 on");
                LCD_I2C_printStr(hlcd, "y-axis edge 2");
                calibSubState = CALIB_STATE_WAIT_PROBE2_X_DONE;
            }
            break;

        case CALIB_STATE_WAIT_PROBE2_X_DONE:
            if (Iscalib2Done(&motors[1], &motors[3]))
            {
                LCD_I2C_Clear(hlcd);
                LCD_I2C_SetCursor(hlcd, 0, 1);
                LCD_I2C_printStr(hlcd, "Points saved");
                calibSubState = CALIB_STATE_COMPLETE;
            }
            break;

        case CALIB_STATE_COMPLETE:
        {
               currentState = MENU_STATE_TESTING;
            // Reset the substate for future calibration sessions.
            calibSubState = CALIB_STATE_INIT;
        }
            break;

        default:
            break;

    }
    vTaskDelay(pdMS_TO_TICKS(10));

}
