#include "Calibration.h" 
uint8_t motorGroup = 0;// 0 for motor[0] and motor[2], 1 for motor[1] and motor[3]
volatile uint8_t Pressed = 0;
int8_t motor1Cali[2];
int8_t motor2Cali[2];
uint32_t StepsFront[4]={0,0,0,0};
int32_t StepsBack[4]={0,0};
uint32_t LastSteps[3] = {0,0,0,0};
uint8_t x = 0;
bool testing = 0;
const uint32_t HOMING_SPEED = 80000;
const uint32_t CALIB_SPEED = 80000;

static void ResetMotorState(Motor *m, float homePosition) {
    m->currentPositionMM = homePosition;
    m->stepsTaken = 0;
    m->StepsBack = 0;
    m->StepsFront = 0;
}

/*------------------------------------------------------------------

  For each motor (assumed here to be in an array of 4 motors):
    • Set servo positions.
    • For each motor, if its corresponding sensor is not triggered,
      configure it (set speed, send a command to set its direction and
      start it) so that it moves toward home.
    • If the sensor is already triggered, mark the motor as homed and
      reset its state.
    • Then poll (with a short delay) until each motor reaches its home.
    • When a sensor is triggered during polling, a STOP command is sent,
      and the motor state is reset.
------------------------------------------------------------------*/
bool MotorsHoming(Motor *motor) {
    bool homed[4] = { false, false, false, false };
    MotorCommand cmd;  // Temporary command structure for queue commands

    /* Set servo positions for homing (adjust positions as needed) */
    //SERVO_WritePosition(&hservo1, 115);
    //SERVO_WritePosition(&hservo2, 115);

    /* --- Start each motor if not already at its home sensor --- */
    /* Motor 0: Uses EndStop2, home position = 0, direction = 1 */
    if (IsSensorTriggered(EndStop2_GPIO_Port, EndStop2_Pin) == 0) {
        TMC2209_SetSpeed(&motor[0], HOMING_SPEED);
        cmd.motorIndex = 0;
        cmd.command = MOTOR_CMD_DIRECTION;
        cmd.direction = 1;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
        cmd.command = MOTOR_CMD_START;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    } else {
        homed[0] = true;
        ResetMotorState(&motor[0], 0);
    }

    /* Motor 1: Uses EndStop4, home position = 450, direction = 0 */
    if (IsSensorTriggered(EndStop4_GPIO_Port, EndStop4_Pin) == 0) {
        TMC2209_SetSpeed(&motor[1], HOMING_SPEED);
        cmd.motorIndex = 1;
        cmd.command = MOTOR_CMD_DIRECTION;
        cmd.direction = 0;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
        cmd.command = MOTOR_CMD_START;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    } else {
        homed[1] = true;
        ResetMotorState(&motor[1], 0);
    }

    /* Motor 2: Uses EndStop1, home position = 0, direction = 0 */
    if (IsSensorTriggered(EndStop1_GPIO_Port, EndStop1_Pin) == 0) {
        TMC2209_SetSpeed(&motor[2], HOMING_SPEED);
        cmd.motorIndex = 2;
        cmd.command = MOTOR_CMD_DIRECTION;
        cmd.direction = 0;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
        cmd.command = MOTOR_CMD_START;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    } else {
        homed[2] = true;
        ResetMotorState(&motor[2], 0);
    }

    /* Motor 3: Uses EndStop3, home position = 0, direction = 1 */
    if (IsSensorTriggered(EndStop3_GPIO_Port, EndStop3_Pin) == 0) {
        TMC2209_SetSpeed(&motor[3], HOMING_SPEED);
        cmd.motorIndex = 3;
        cmd.command = MOTOR_CMD_DIRECTION;
        cmd.direction = 1;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
        cmd.command = MOTOR_CMD_START;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    } else {
        homed[3] = true;
        ResetMotorState(&motor[3], 0);
    }

    /* --- Poll sensors until all motors are homed --- */
    while (!(homed[0] && homed[1] && homed[2] && homed[3])) {
        if (!homed[0] && (IsSensorTriggered(EndStop2_GPIO_Port, EndStop2_Pin) == 1)) {
            cmd.motorIndex = 0;
            cmd.command = MOTOR_CMD_STOP;
            xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
            ResetMotorState(&motor[0], 0);
            homed[0] = true;
        }
        if (!homed[1] && (IsSensorTriggered(EndStop4_GPIO_Port, EndStop4_Pin) == 1)) {
            cmd.motorIndex = 1;
            cmd.command = MOTOR_CMD_STOP;
            xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
            ResetMotorState(&motor[1], 0);
            homed[1] = true;
        }
        if (!homed[2] && (IsSensorTriggered(EndStop1_GPIO_Port, EndStop1_Pin) == 1)) {
            cmd.motorIndex = 2;
            cmd.command = MOTOR_CMD_STOP;
            xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
            ResetMotorState(&motor[2], 0);
            homed[2] = true;
        }
        if (!homed[3] && (IsSensorTriggered(EndStop3_GPIO_Port, EndStop3_Pin) == 1)) {
            cmd.motorIndex = 3;
            cmd.command = MOTOR_CMD_STOP;
            xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
            ResetMotorState(&motor[3], 0);
            homed[3] = true;
        }
        /* Yield for 1 ms to allow other tasks to run */
        //vTaskDelay(pdMS_TO_TICKS(1));
    }

    return true;
}

/*------------------------------------------------------------------
 
  This function performs the auto‑calibration sequence:
    1. Calls MotorsHoming() to home all motors.
    2. Sends a series of MOVETO commands (via the motor command queue)
       to move each motor to its calibration positions.
    3. Uses nonblocking delays with vTaskDelay().

  Adjust axis/motor indexes, positions, and offsets as needed.
------------------------------------------------------------------*/
void AutoCalibration(Axis *axes, Motor *motors) {
    // First, perform homing on all motors.
    MotorCommand cmd;

//    SERVO_WritePosition(&hservo1, SERVO1_HOME_POS);
//    SERVO_WritePosition(&hservo2, SERVO2_HOME_POS);
    // Continue with calibration until the calibration condition is met.
    if(!calibrationState()) {
        MotorsHoming(motors);

        /*
         * Define target positions for each axis.
         *   - Axis 0:
         *       Motor 0 -> 77.9 mm
         *       Motor 1 -> -100.8 mm
         *   - Axis 1:
         *       Motor 0 -> -47.9 mm
         *       Motor 1 -> 50.2 mm
         *
         * Adjust the indices below if your system assigns motors differently.
         */
        TMC2209_SetSpeed(&motors[0], CALIB_SPEED);
        TMC2209_SetSpeed(&motors[1], CALIB_SPEED);
        TMC2209_SetSpeed(&motors[2], CALIB_SPEED);
        TMC2209_SetSpeed(&motors[3], CALIB_SPEED);
        HAL_Delay(3000);

        LCD_I2C_ClearAllLines(&hlcd3);
        LCD_I2C_SetCursor(&hlcd3, 0, 2);
        LCD_I2C_printStr(&hlcd3, "Calibrating!");

        // The Auto calibration based on only hard saving one point was deleted
        // Because of the mechanical design the probes were bending so the next commands
        // are for auto calibration based on 2 hard saved coordinates
        // sending commands to the motor control task
        cmd.targetPositionsAxis0[0] = 38.653677f;   // Y
        cmd.targetPositionsAxis0[1] = -57.26f; // Y
        cmd.targetPositionsAxis0[2] = -80.8456786f;  // X
        cmd.targetPositionsAxis0[3] = 22.0f;   // X

        cmd.command = MOTOR_CMD_MOVE_ALL_MOTORS;

       xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
       CheckConnection(&hservo1,&hservo2);
//       SERVO_WritePosition(&hservo1, SERVO1_HOME_POS);
//       SERVO_WritePosition(&hservo2, SERVO2_HOME_POS);
       axes[0].motors[0]->currentPositionMM = 0.0f;
       axes[0].motors[1]->currentPositionMM = pcbHeight;
       axes[1].motors[0]->currentPositionMM = 0.0f;
       axes[1].motors[1]->currentPositionMM = 0.0f;


       // This is just to leave the function after actually calibrating the numbers don't matter
        motors[0].calib[1] = 1;
        motors[1].calib[1] = 1;
        motors[2].calib[1] = 1;
        motors[3].calib[1] = 1;

        // Clear and update the LCD to indicate calibration is done.
        LCD_I2C_ClearAllLines(&hlcd3);
        LCD_I2C_SetCursor(&hlcd3, 0, 2);
        LCD_I2C_printStr(&hlcd3, "Calibration done!");

        // Perform any connection testing.
       // testing = CheckConnection(&hservo2, &hservo1);

    }
}


bool calibrationState(void) {
    
    if (motors[0].calib[1] != 0 &&
        motors[2].calib[1] != 0 &&
        motors[3].calib[1] != 0 &&
        motors[1].calib[1] != 0)
    {
        return true;
    }
    else {
        return false;
    }
}


void ManualCalibration(Axis *axes, Motor *motors) {
    /* If calibration is complete, exit immediately */
    TMC2209_SetSpeed(&motors[0], 50000);
    TMC2209_SetSpeed(&motors[1], 50000);
    TMC2209_SetSpeed(&motors[2], 50000);
    TMC2209_SetSpeed(&motors[3], 50000);
    MotorCommand cmd;  // Command structure to post to motorControlTask
	LCD_I2C_ClearAllLines(&hlcd3);
	LCD_I2C_SetCursor(&hlcd3, 0, 2);
	LCD_I2C_printStr(&hlcd3, "Calibrate Probe 1!");
	MotorsHoming(motors);
    while(!calibrationState()){
    //RunManualCalibrationStateMachine(&hlcd3, &motors);
        SERVO_WritePosition(&hservo1, SERVO1_CHECK_POS);
        SERVO_WritePosition(&hservo2, SERVO2_CHECK_POS);

    /* Static variables to detect a button press edge */
    	/* Example variables. Put them in a suitable scope (static in file-scope or function-scope). */
    	static uint8_t buttonState = 0;       // 0 = not pressed, 1 = pressed
    	static uint32_t pressStartTime = 0;   // Time at which the button was first pressed

    	// Debounce time in ms:
    	const uint32_t debounceTime = 50;

    /* Set servo positions (this call remains direct) */


    static uint32_t lastPressTime = 0;  // Last valid press timestamp

    /* Process the calibration button (BtnCtr) */

    uint32_t currentTime = xTaskGetTickCount(); // or HAL_GetTick(), whichever you use
    uint8_t currentLevel = HAL_GPIO_ReadPin(EncoderBtn_GPIO_Port, EncoderBtn_Pin);

    // ----------------------------------------------------------
    // Active-low button logic: pressed = (currentLevel == RESET)
    // ----------------------------------------------------------
    if (currentLevel == GPIO_PIN_RESET) {
        // Button is physically pressed
        if (buttonState == 0) {
            // Transition from not-pressed -> pressed
            buttonState = 1;
            pressStartTime = currentTime;
        }
    } else {
        // Button is physically released
        if (buttonState == 1) {
            // Transition from pressed -> released
            buttonState = 0;
            // Check if it was held long enough to count as a valid press
            if ((currentTime - pressStartTime) >= debounceTime) {
                // We register exactly ONE press per cycle
                Pressed++;

                // -----------------------------
                // Handle your calibration steps
                // -----------------------------
                switch (Pressed) {
                    case 1:
                        // Save calibration for first press
                        motors[motorGroup].currentPositionMM =
                            (float)abs(motors[motorGroup].StepsFront - motors[motorGroup].StepsBack)
                              / axes[0].stepPerUnit;
                        motors[motorGroup + 2].currentPositionMM =
                            (float)abs(motors[motorGroup + 2].StepsBack - motors[motorGroup + 2].StepsFront)
                              / axes[1].stepPerUnit;

                        motors[motorGroup].calib[0] = motors[motorGroup].currentPositionMM;
                        motors[motorGroup + 2].calib[0] = motors[motorGroup + 2].currentPositionMM;

                        motorGroup += 1;
                        LCD_I2C_ClearAllLines(&hlcd3);
                        LCD_I2C_SetCursor(&hlcd3, 0, 2);
                        LCD_I2C_printStr(&hlcd3, "Calibrate Probe 2!");
                        break;

                    case 2:
                        // Save calibration for second press

                        motors[motorGroup].currentPositionMM =
                            (float)abs(motors[motorGroup].StepsFront - motors[motorGroup].StepsBack)
                              / axes[0].stepPerUnit;
                        motors[motorGroup + 2].currentPositionMM =
                            (float)abs(motors[motorGroup + 2].StepsBack - motors[motorGroup + 2].StepsFront)
                              / axes[1].stepPerUnit;

                        motors[motorGroup].calib[0] = motors[motorGroup].currentPositionMM;
                        motors[motorGroup + 2].calib[0] = motors[motorGroup + 2].currentPositionMM;
                        MotorsHoming(motors);
                    	SERVO_WritePosition(&hservo1, SERVO1_HOME_POS);
                    	SERVO_WritePosition(&hservo2, SERVO2_HOME_POS);

                        cmd.targetPositionsAxis0[0] = motors[0].calib[0];   // Y
                        cmd.targetPositionsAxis0[1] = -(motors[1].calib[0]); // Y
                        cmd.targetPositionsAxis0[2] = -(motors[2].calib[0]);  // X
                        cmd.targetPositionsAxis0[3] = motors[3].calib[0];   // X
                        cmd.command = MOTOR_CMD_MOVE_ALL_MOTORS;
                        LCD_I2C_ClearAllLines(&hlcd3);
                        LCD_I2C_SetCursor(&hlcd3, 0, 2);
                        LCD_I2C_printStr(&hlcd3, "Calibration Done!");
                       xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
                       axes[0].motors[0]->currentPositionMM = 0.0f;
                       axes[0].motors[1]->currentPositionMM = pcbHeight;
                       axes[1].motors[0]->currentPositionMM = 0.0f;
                       axes[1].motors[1]->currentPositionMM = 0.0f;


                        motorGroup = 0;  // Reset as needed
                        Pressed = 0;     // Reset the press counter
                        //The numbers below don't matter they are just to leave the function
                        motors[0].calib[1] = 1;
                        motors[1].calib[1] = 1;
                        motors[2].calib[1] = 1;
                        motors[3].calib[1] = 1;

                        break;
                        break;

                    default:
                        // If somehow we get more than 2, just reset
                        Pressed = 0;
                        break;
                }
            }
        }
    }




    /* --- Manual motor control via buttons --- */

    /* Example: BtnUp pressed -> move motor (motorGroup) in the forward direction */
    if (HAL_GPIO_ReadPin(BtnUp_GPIO_Port, BtnUp_Pin) == GPIO_PIN_RESET) {
        cmd.motorIndex = motorGroup;
        cmd.command    = MOTOR_CMD_DIRECTION;
        cmd.direction  = GPIO_PIN_SET;  // Set forward direction
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    	//TMC2209_SetDirection(&motors[motorGroup],1);

        cmd.command = MOTOR_CMD_START;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

        /* While the button remains pressed, yield to other tasks */
        while (HAL_GPIO_ReadPin(BtnUp_GPIO_Port, BtnUp_Pin) == GPIO_PIN_RESET) {
            //vTaskDelay(pdMS_TO_TICKS(10));
        }
        cmd.command = MOTOR_CMD_STOP;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    }


    /* Example: BtnDown pressed -> move motor (motorGroup) in the reverse direction */
    if (HAL_GPIO_ReadPin(BtnDown_GPIO_Port, BtnDown_Pin) == GPIO_PIN_RESET) {
        cmd.motorIndex = motorGroup;
        cmd.command    = MOTOR_CMD_DIRECTION;
        cmd.direction  = GPIO_PIN_RESET;  // Set reverse direction
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    	//TMC2209_SetDirection(&motors[motorGroup],1);

        cmd.command = MOTOR_CMD_START;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

        while (HAL_GPIO_ReadPin(BtnDown_GPIO_Port, BtnDown_Pin) == GPIO_PIN_RESET) {
            //vTaskDelay(pdMS_TO_TICKS(10));
        }
        cmd.motorIndex = motorGroup;
        cmd.command    = MOTOR_CMD_STOP;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    }


    /* Example: BtnRight pressed -> move paired motor (motorGroup+2) forward */
    if (HAL_GPIO_ReadPin(BtnRight_GPIO_Port, BtnRight_Pin) == GPIO_PIN_RESET) {
        cmd.motorIndex = motorGroup + 2;
        cmd.command    = MOTOR_CMD_DIRECTION;
        cmd.direction  = GPIO_PIN_SET;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    	//TMC2209_SetDirection(&motors[motorGroup + 2],1);
        cmd.command = MOTOR_CMD_START;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

        while (HAL_GPIO_ReadPin(BtnRight_GPIO_Port, BtnRight_Pin) == GPIO_PIN_RESET) {
            //vTaskDelay(pdMS_TO_TICKS(10));
        }
        cmd.motorIndex = motorGroup + 2;
        cmd.command    = MOTOR_CMD_STOP;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    }



    /* Example: BtnLeft pressed -> move paired motor (motorGroup+2) in reverse */
    if (HAL_GPIO_ReadPin(BtnLeft_GPIO_Port, BtnLeft_Pin) == GPIO_PIN_RESET) {
        cmd.motorIndex = motorGroup + 2;
        cmd.command    = MOTOR_CMD_DIRECTION;
        cmd.direction  = GPIO_PIN_RESET;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    	//TMC2209_SetDirection(&motors[motorGroup + 2],0);

        cmd.command = MOTOR_CMD_START;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);



    while (HAL_GPIO_ReadPin(BtnLeft_GPIO_Port, BtnLeft_Pin) == GPIO_PIN_RESET) {
                //vTaskDelay(pdMS_TO_TICKS(10));
            }
            cmd.motorIndex = motorGroup + 2;
            cmd.command    = MOTOR_CMD_STOP;
            xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
        }
    }
}

/**
 * @brief  Moves a motor at the given speed and direction until a stall is detected.
 *         After the stall, it calculates the current position (in mm) based on the difference
 *         between StepsFront and StepsBack divided by the provided scale factor.
 * @param  motor: pointer to the Motor structure.
 * @param  speed: speed setting for the motor.
 * @param  direction: desired direction (e.g., GPIO_PIN_SET or GPIO_PIN_RESET).
 * @param  scaleFactor: value used to convert steps difference to millimeters.
 */


// Assume that motorCommandQueue is declared globally and created in motorControlTask.
// Also assume that TMC2209_SetSpeed, TMC2209_checkStall, etc., are available.

void moveMotorUntilStallAndCalibrate(Axis *axes, Motor *motors,
                                     uint8_t motorIndex, uint32_t speed,
                                     GPIO_PinState direction, uint8_t calibrationIndex)
{
    MotorCommand cmd;


    // Set the motor speed directly.
    // (If you want to queue speed commands, you could extend the command structure.)
    TMC2209_SetSpeed(&motors[motorIndex], speed);

    // Set the direction via the queue.
    cmd.motorIndex = motorIndex;
    cmd.command = MOTOR_CMD_DIRECTION;
    cmd.direction = direction;
    xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

    // Start the motor via the queue.
    cmd.command = MOTOR_CMD_START;
    xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

    // Loop until a stall is detected.
    for (;;) {
        TMC2209_checkStall(&motors[motorIndex]);
        if (motors[motorIndex].STALL) {
            // When a stall is detected, stop the motor via the queue.
            cmd.command = MOTOR_CMD_STOP;
            xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
            break;
        }
        // Delay 1 ms to prevent a tight loop and yield to other tasks.
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Calculate the calibrated position (in mm) using your step counts.
    if (motorIndex < 1) {  // For motor 0 (and its pair motor 2)
        motors[motorIndex].currentPositionMM = fabs((float)motors[motorIndex].StepsFront -
                                                      (float)motors[motorIndex].StepsBack) / axes[0].stepPerUnit;
        motors[motorIndex].calib[calibrationIndex] = motors[motorIndex].currentPositionMM;

        motors[motorIndex + 2].currentPositionMM = fabs((float)motors[motorIndex + 2].StepsFront -
                                                          (float)motors[motorIndex + 2].StepsBack) / axes[1].stepPerUnit;
        motors[motorIndex + 2].calib[calibrationIndex] = motors[motorIndex + 2].currentPositionMM;
    } else if (motorIndex > 1) {  // For motor 2 or 3 (paired with motorIndex-2)
        motors[motorIndex].currentPositionMM = fabs((float)motors[motorIndex].StepsFront -
                                                      (float)motors[motorIndex].StepsBack) / axes[1].stepPerUnit;
        motors[motorIndex].calib[calibrationIndex] = motors[motorIndex].currentPositionMM;

        motors[motorIndex - 2].currentPositionMM = fabs((float)motors[motorIndex - 2].StepsFront -
                                                          (float)motors[motorIndex - 2].StepsBack) / axes[0].stepPerUnit;
        motors[motorIndex - 2].calib[calibrationIndex] = motors[motorIndex - 2].currentPositionMM;
    }
}

/**
 * @brief  Runs the semi‑auto calibration sequence.
 *         On each valid press of the Ctr button, a specific motor is moved until a stall is detected.
 *         The user should first move the probes to a position facing the PCB to capture points using buttons.
 *         The calibration sequence moves the motors in the following order:
 *            Case 1: Motor 1 (index 0) with scale factor 400.
 *            Case 2: Motor 3 (index 2) with scale factor 160.
 *            Case 3: Motor 2 (index 1) with scale factor 400.
 *            Case 4: Motor 4 (index 3) with scale factor 160.
 * @param  motors: An array of Motor structures.
 */

// Assume calibrationState(), HAL_GPIO_ReadPin(), and the GPIO definitions for BtnCtr, BtnUp, etc., are available.
// Also assume that motorGroup, StepsBack, motors, and axes are declared globally.

void semiAutoCalibration(Axis *axes, Motor *motors)
{
	MotorsHoming(motors);
    // If a calibration is already in progress, exit.
    while(!calibrationState()) {


    // Static variables for button debounce and calibration step tracking.
    static uint8_t ctrPressedFlag = 0;
    static TickType_t pressStartTime = 0;
    static uint8_t calibrationStep = 0;
    const TickType_t debounceTime = pdMS_TO_TICKS(50);
//    LCD_I2C_Clear(&hlcd3);
//    LCD_I2C_SetCursor(&hlcd3, 0, 1);
//    LCD_I2C_printStr(&hlcd3, "Use Buttons to Move");
//    LCD_I2C_SetCursor(hlcd, 1, 1);
//    LCD_I2C_printStr(hlcd, "Probe 1");
//    osDelay(300);
    //delayStartTime = xTaskGetTickCount();
    //calibSubState = CALIB_STATE_INSTRUCT_PROBE1;
    TickType_t currentTime = xTaskGetTickCount();

    // Check the calibration (Ctr) button.
    if (HAL_GPIO_ReadPin(EncoderBtn_GPIO_Port, EncoderBtn_Pin) == GPIO_PIN_SET) {
        if (ctrPressedFlag == 0) {  // First edge detected.
            pressStartTime = currentTime;
            ctrPressedFlag = 1;
        }
    } else {
        // On button release, if debounce time has passed, advance the calibration step.
        if (ctrPressedFlag == 1 && (currentTime - pressStartTime) >= debounceTime) {
            calibrationStep++;
        }
        ctrPressedFlag = 0;
    }
//    LCD_I2C_Clear(hlcd);
//    LCD_I2C_SetCursor(hlcd, 0, 0);
//    LCD_I2C_printStr(hlcd, "Place probe 1 facing");
//    LCD_I2C_SetCursor(hlcd, 1, 0);
//    LCD_I2C_printStr(hlcd, "X-axis edge 1");
    //calibSubState = CALIB_STATE_WAIT_PROBE1_DONE;
    // Execute the calibration step.
    RunSemiAutoCalibrationStateMachine(&hlcd3,&motors);
    switch (calibrationStep) {
        case 1:
            // Case 1: Move Motor 1 (index 0) in reverse until stall.
            moveMotorUntilStallAndCalibrate(axes, motors, 0, 7000, GPIO_PIN_RESET, 0);
//            LCD_I2C_Clear(hlcd);
//            LCD_I2C_SetCursor(hlcd, 0, 0);
//            LCD_I2C_printStr(hlcd, "Place probe 1 facing");
//            LCD_I2C_SetCursor(hlcd, 1, 0);
//            LCD_I2C_printStr(hlcd, "Y-axis edge 1");
            break;

        case 2:
            // Case 2: Move Motor 3 (index 2) in reverse until stall.
            moveMotorUntilStallAndCalibrate(axes, motors, 2, 7000, GPIO_PIN_SET, 1);
//            LCD_I2C_Clear(hlcd);
//            LCD_I2C_SetCursor(hlcd, 0, 0);
//            LCD_I2C_printStr(hlcd, "Place probe 1 facing");
//            LCD_I2C_SetCursor(hlcd, 1, 0);
//            LCD_I2C_printStr(hlcd, "Y-axis edge 1");
            motorGroup++;
            if (motorGroup >= 2) {
                motorGroup = 0;  // Reset as needed.
            }

            break;

        case 3:
            // Case 3: Move Motor 2 (index 1) in reverse until stall.
            moveMotorUntilStallAndCalibrate(axes, motors, 1, 7000, GPIO_PIN_SET, 0);
//            LCD_I2C_Clear(hlcd);
//            LCD_I2C_SetCursor(hlcd, 0, 0);
//            LCD_I2C_printStr(hlcd, "Place probe 2 facing");
//            LCD_I2C_SetCursor(hlcd, 1, 0);
//            LCD_I2C_printStr(hlcd, "X-axis edge 2");
            break;

        case 4:
//            LCD_I2C_Clear(hlcd);
//            LCD_I2C_SetCursor(hlcd, 0, 0);
//            LCD_I2C_printStr(hlcd, "Place probe 2 facing");
//            LCD_I2C_SetCursor(hlcd, 1, 0);
//            LCD_I2C_printStr(hlcd, "X-axis edge 2");

            // Case 4: Move Motor 4 (index 3) in reverse until stall.
            moveMotorUntilStallAndCalibrate(axes, motors, 3, 7000, GPIO_PIN_RESET, 1);
            motorGroup++;
            if (motorGroup >= 2) {
                motorGroup = 0;
            }
            calibrationStep = 0;  // Reset calibration steps.
            break;

        default:
            break;
    }

    // --- Manual Control Using RTOS Commands ---
    // Instead of directly calling TMC2209 functions, we post commands to the queue.
    MotorCommand cmd;

    // Example for the "Up" button:
    if (HAL_GPIO_ReadPin(BtnUp_GPIO_Port, BtnUp_Pin) == GPIO_PIN_RESET) {
        cmd.motorIndex = motorGroup;
        cmd.command = MOTOR_CMD_DIRECTION;
        cmd.direction = GPIO_PIN_SET;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

        cmd.command = MOTOR_CMD_START;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

        // Wait while the button remains pressed.
        while (HAL_GPIO_ReadPin(BtnUp_GPIO_Port, BtnUp_Pin) == GPIO_PIN_RESET) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        cmd.command = MOTOR_CMD_STOP;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    }

    // Example for the "Down" button:
    if (HAL_GPIO_ReadPin(BtnDown_GPIO_Port, BtnDown_Pin) == GPIO_PIN_RESET) {
        cmd.motorIndex = motorGroup;
        cmd.command = MOTOR_CMD_DIRECTION;
        cmd.direction = GPIO_PIN_RESET;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

        cmd.command = MOTOR_CMD_START;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

        while (HAL_GPIO_ReadPin(BtnDown_GPIO_Port, BtnDown_Pin) == GPIO_PIN_RESET) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        cmd.motorIndex = motorGroup;
        cmd.command = MOTOR_CMD_STOP;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    }

    // Example for the "Right" button:
    if (HAL_GPIO_ReadPin(BtnRight_GPIO_Port, BtnRight_Pin) == GPIO_PIN_RESET) {
        cmd.motorIndex = motorGroup + 2;
        cmd.command = MOTOR_CMD_DIRECTION;
        cmd.direction = GPIO_PIN_SET;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

        cmd.motorIndex = motorGroup + 2;
        cmd.command = MOTOR_CMD_START;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

        while (HAL_GPIO_ReadPin(BtnRight_GPIO_Port, BtnRight_Pin) == GPIO_PIN_RESET) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        cmd.motorIndex = motorGroup + 2;
        cmd.command = MOTOR_CMD_STOP;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    }

    // Example for the "Left" button:
    if (HAL_GPIO_ReadPin(BtnLeft_GPIO_Port, BtnLeft_Pin) == GPIO_PIN_RESET) {
        cmd.motorIndex = motorGroup + 2;
        cmd.command = MOTOR_CMD_DIRECTION;
        cmd.direction = GPIO_PIN_RESET;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

        cmd.motorIndex = motorGroup + 2;
        cmd.command = MOTOR_CMD_START;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);

        while (HAL_GPIO_ReadPin(BtnLeft_GPIO_Port, BtnLeft_Pin) == GPIO_PIN_RESET) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        cmd.motorIndex = motorGroup + 2;
        cmd.command = MOTOR_CMD_STOP;
        xQueueSend(motorCommandQueue, &cmd, portMAX_DELAY);
    }
  }
}
bool Iscalib1Done(Motor *motor1,Motor *motor2){
    if(motor1->calib[0] != 0 && motor2->calib[0] != 0){
        return true;
    }
    return false;
}
bool Iscalib2Done(Motor *motor1,Motor *motor2){
    if(motor1->calib[1] != 0 && motor2->calib[1] != 0){
        return true;
    }
    return false;
}

