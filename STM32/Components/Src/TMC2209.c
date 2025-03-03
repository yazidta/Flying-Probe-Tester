/******************************************************************************
 * @file       TMC2209.cpp
 * @author     Ahmed Bouras
 * @date       05/12/2024
 * @version    0.6
 *
 * @copyright  Copyright (c) [2024] Ahmed Bouras
 *
 * @license    Permission is hereby granted, free of charge, to any person
 *             obtaining a copy of this software and associated documentation
 *             files (the "Software"), to deal in the Software without
 *             restriction, including without limitation the rights to use,
 *             copy, modify, merge, publish, distribute, sublicense, and/or
 *             sell copies of the Software, and to permit persons to whom
 *             the Software is furnished to do so, subject to the following
 *             conditions:
 *
 *             The above copyright notice and this permission notice shall
 *             be included in all copies or substantial portions of the
 *             Software.
 *
 *             THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
 *             KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *             WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 *             PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
 *             OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 *             OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 *             OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *             SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @note       This library provides an interface for controlling the TMC2209
 *             stepper motor driver over STEP, DIR, ENN & UART.
 ******************************************************************************
 */
#include "TMC2209.h"
#include "math.h"


uint32_t last_tmc_read_attempt_ms = 0;
uint8_t direction = 0;

////////// HAL FUNCTIONS //////////

// PWM callback for step counting
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  for(int i = 0; i < MAX_MOTORS; i++){
	  if (htim->Instance == motors[i].driver.htim->Instance){ // Check which motor's timer called back
		  motors[i].stepsTaken++;
		  //TMC2209_CountDistance(&motors[i]);
		  if(motors[i].direction == GPIO_PIN_SET){
		  		  motors[i].StepsFront++;

		  }

		  else if(motors[i].direction == GPIO_PIN_RESET){
			  	  motors[i].StepsBack++;

		  }
          if (motors[i].stepsTaken % motors[i].stepsPerRevolution == 0){ // Count Full steps
              motors[i].driver.checkSG_RESULT = 1;
              motors[i].fullSteps++;
          }
      }

    }
  }


// Set the direction of the motor
void TMC2209_SetDirection(Motor *motor, GPIO_PinState state) {
    HAL_GPIO_WritePin(motor->driver.dir_port, motor->driver.dir_pin, state);
    motor->direction = state;
}

// Enable or disable the driver
void TMC2209_EnableDriver(Motor *motor, GPIO_PinState state) {
    HAL_GPIO_WritePin(motor->driver.enn_port, motor->driver.enn_pin, state); // LOW = motor enabled, HIGH = motor disabled
}


// Read DIAG pin status
uint8_t TMC2209_ReadDiag(Motor *motor) {
    return HAL_GPIO_ReadPin(motor->driver.diag_port, motor->driver.diag_pin); // Returns the DIAG pin state (LOW = no errors, HIGH = errors)
}

// Read INDEX position
uint32_t TMC2209_ReadIndexStatus(Motor *motor) {
    return HAL_GPIO_ReadPin(motor->driver.index_port, motor->driver.index_pin); // Returns the INDEX pin state
}


// Start stepping with PWM
void TMC2209_SetSpeed(Motor *motor, uint32_t StepFrequency) {
	uint32_t prescaler = motor->driver.htim->Init.Prescaler;
//	if(StepFrequency >= (1 << 16)){ // divide Period and Prescaler if stepFrequency is greater than 2^16 since the ARR only supports up to 16 bit
//		HAL_TIM_Base_Stop_IT(motor->driver.htim);
//		StepFrequency = StepFrequency / 3;
//		motor->driver.htim->Init.Prescaler = prescaler / 3; // Update prescaler
//		HAL_TIM_Base_Init(motor->driver.htim);
//		HAL_TIM_PWM_Init(motor->driver.htim);
//	};
    uint32_t timerClock = HAL_RCC_GetHCLKFreq() / prescaler ;
    uint32_t ARR = (timerClock / StepFrequency) - 1; // Auto-reload value

    __HAL_TIM_SET_AUTORELOAD(motor->driver.htim, ARR); // Period
    __HAL_TIM_SET_COMPARE(motor->driver.htim, motor->driver.step_channel, ARR / 2); // Duty cycle
    motor->driver.stepFrequency = StepFrequency;
}


// Stop stepping
void TMC2209_Stop(Motor *motor) {
	TIM_HandleTypeDef *htim = motor->driver.htim;
	uint32_t channel = motor->driver.step_channel;
	TMC2209_EnableDriver(motor, GPIO_PIN_SET);
    HAL_TIM_PWM_Stop_IT(htim, channel);
}

void TMC2209_Start(Motor *motor) {
	TIM_HandleTypeDef *htim = motor->driver.htim;
	uint32_t channel = motor->driver.step_channel;

	TMC2209_EnableDriver(motor, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start_IT(htim, channel);
    motor->isStepping = true;
}


static void TMC2209_CountSteps(Motor *motor, uint32_t totalSteps){ // Static for now unless we need to expose it later
	if(totalSteps == 0){
		return;
	}
	motor->nextTotalSteps = totalSteps;
	motor->stepsTaken = 0;

	while (motor->stepsTaken <= motor->nextTotalSteps) {// Wait until we reach required steps and increment position on every step
    // vTaskDelay(10);
	}
	//HAL_Delay(1); // To not fad the cpu --NOTE: CHECK IF THERE SHOULD BE A DELAY

	motor->nextTotalSteps = 0;
}

void TMC2209_CountDistance(Motor *motor){
	if(motor->direction != 0){
		motor->currentPositionMM += getStepPerUnit(motor);
	}
	else {
		motor->currentPositionMM -= getStepPerUnit(motor);
	}
}


void TMC2209_Step(Motor *motor, uint32_t steps){ // This doesn't work anymore since we have MoveTo
	TMC2209_Start(motor);
	TMC2209_CountSteps(motor, steps);
	TMC2209_Stop(motor);

}

void TMC2209_checkStatus(Motor *motor, bool *isStepping, uint32_t *nextTotalSteps){
	 *isStepping = motor->isStepping;
     *nextTotalSteps = motor->nextTotalSteps;
}
void TMC2209_MoveTo(Axis *axis, uint8_t motorIndex, float targetPositionMM) {
    // Validate the motor index

    if (motorIndex >= MAX_MOTORS_PER_AXIS) {
        debug_print("Invalid motor index.\r\n");
        return;
    }

    // Select the motor from the axis
//    if (!motor) {
//        debug_print("Motor not assigned.\r\n");
//        return;
//    }

    // Calculate the distance to move in millimeters
    //motor->currentPositionMM =0;

    float distanceToMoveMM = targetPositionMM - axis->motors[motorIndex]->currentPositionMM;

    // Convert the distance to move into steps
    int32_t stepsToMove = (int32_t)(distanceToMoveMM * axis->stepPerUnit);

    // Update the target position in the motor structure
    axis->motors[motorIndex]->nextPositionMM = targetPositionMM;

    // Decide the direction based on the sign of the steps
    if (stepsToMove > 0) {
        TMC2209_SetDirection(axis->motors[motorIndex], GPIO_PIN_RESET); // Forward direction
    } else {
        TMC2209_SetDirection(axis->motors[motorIndex], GPIO_PIN_SET); // Reverse direction
        stepsToMove = -stepsToMove; // Convert to positive for step count
    }

    // Start the motor

    TMC2209_Start(axis->motors[motorIndex]);

    // Set the total steps to move
    axis->motors[motorIndex]->nextTotalSteps = stepsToMove;
	TMC2209_CountSteps(axis->motors[motorIndex], stepsToMove);

    // Stop the motor
    TMC2209_Stop(axis->motors[motorIndex]);

    // Update the current position in the motor structure
    axis->motors[motorIndex]->currentPositionMM = targetPositionMM;

}
// Function to move all motors concurrently to their target positions.
// The targetPositionsMM array should have one target position (in millimeters)
// for each motor in the axis. We assume that MAX_MOTORS_PER_AXIS is defined.
void TMC2209_MoveAllMotorsTo(Axis axes[2], float targetPositions[4]) {
    uint8_t axisIndex, motorIndex;

    // Initialize each motor on both axes.
    // The mapping is:
    //   targetPositions[ axisIndex * MAX_MOTORS_PER_AXIS + motorIndex ]
    for (axisIndex = 0; axisIndex < 2; axisIndex++) {
        for (motorIndex = 0; motorIndex < MAX_MOTORS_PER_AXIS; motorIndex++) {
            int targetIndex = axisIndex * MAX_MOTORS_PER_AXIS + motorIndex;
            Motor *motor = axes[axisIndex].motors[motorIndex];
            if (motor == NULL) {
                continue;
            }

            // Calculate the distance (in mm) and convert to steps.
            float distanceToMoveMM = targetPositions[targetIndex] - motor->currentPositionMM;
            double stepsError = (distanceToMoveMM * axes[axisIndex].stepPerUnit) + motor->stepError;
            int32_t stepsToMove = (int32_t)(roundf(stepsError));
            motor->stepError = stepsError - stepsToMove;

            // Save the absolute number of steps required.
            motor->nextTotalSteps = (stepsToMove >= 0) ? stepsToMove : -stepsToMove;
            // Reset the steps counter.
            motor->stepsTaken = 0;
            // Store the target position.
            motor->nextPositionMM = targetPositions[targetIndex];

            // Set the motor direction.
            if (stepsToMove >= 0) {
                TMC2209_SetDirection(motor, GPIO_PIN_RESET);  // Forward
            } else {
                TMC2209_SetDirection(motor, GPIO_PIN_SET);      // Reverse
            }

            // Start the motor.
            if(motor->currentPositionMM != motor->nextPositionMM)	TMC2209_Start(motor);

        }
    }

    // Poll all motors concurrently. Each motor will be stopped as soon as it finishes.
    bool motorsStillRunning = true;
    while (motorsStillRunning) {
        motorsStillRunning = false;  // Assume all motors are finished unless one is still moving.

        for (axisIndex = 0; axisIndex < 2; axisIndex++) {
            for (motorIndex = 0; motorIndex < MAX_MOTORS_PER_AXIS; motorIndex++) {
                Motor *motor = axes[axisIndex].motors[motorIndex];
                if (motor == NULL) {
                    continue;
                }
                // If this motor still has steps to take...
                if (motor->nextTotalSteps > 0) {
                    if (motor->stepsTaken >= motor->nextTotalSteps) {
                        // This motor has reached its target: stop it and update its current position.
                        TMC2209_Stop(motor);
                        motor->prevPositionMM = motor->currentPositionMM;
                        motor->currentPositionMM = motor->nextPositionMM;
                        // Mark this motor as finished.
                        motor->nextTotalSteps = 0;
                    } else {
                        // At least one motor is still moving.
                        motorsStillRunning = true;
                    }
                }
            }
        }
        // Delay briefly to avoid hogging the CPU.
        // If you're not using an RTOS, replace vTaskDelay(1) with a suitable delay function (e.g., HAL_Delay(1)).
        //vTaskDelay(1);
    }
}





uint8_t getStepPerUnit(Motor *motor){ // Gets the stepPerUnit of that motor based on the axis it's in
	uint8_t motorID = motor->driver.id;
	if(motorID == 0 || motorID == 2){
		return axes[1].stepPerUnit;
	}
	return axes[2].stepPerUnit;
}



/// TMC2209 UART Function ///

void clear_UART_buffers(UART_HandleTypeDef *huart) {
    debug_print("Clearing UART buffers...\r\n");

    // Clear UART flags
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    // Flush receive buffer
    uint8_t dummy;
    while(__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
        dummy = (uint8_t)(huart->Instance->RDR & 0x00FF);
    }
    (void)dummy;
}


void debug_print(const char* msg) {
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)msg, strlen(msg));
}

 void debug_print_hex(uint8_t* data, uint8_t length) {
    char buffer[256];
    char* ptr = buffer;

    ptr += sprintf(ptr, "[");
    for(uint8_t i = 0; i < length; i++) {
        ptr += sprintf(ptr, "%02X ", data[i]);
    }
    ptr += sprintf(ptr, "]\r\n");

    debug_print(buffer);
}

uint8_t calculate_CRC(uint8_t *datagram, uint8_t length) {
    uint8_t crc = 0;
    for(uint8_t i = 0; i < length; i++) {
        uint8_t currentByte = datagram[i];
        for(uint8_t j = 0; j < 8; j++) {
            if((crc >> 7) ^ (currentByte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            currentByte >>= 1;
        }
    }
    return crc;
}




uint8_t TMC2209_WaitForReply(uint32_t timeout) {
     uint32_t startTime = HAL_GetTick();
     while (!rxBufferReady) {
         if ((HAL_GetTick() - startTime) > timeout) {
             debug_print("Timeout waiting for reply.\r\n");
             return 0; // Timeout
         }
     }
     rxBufferReady = 0; // Clear flag for next use
     return 1; // Success
 }


uint8_t *TMC2209_sendCommand(uint8_t *command, size_t writeLength, size_t readLength, Motor *tmc2209) {
	//clear_UART_buffers(&huart2);
     // Send the command
     if (HAL_UART_Transmit(tmc2209->driver.huart, command, writeLength, HAL_MAX_DELAY) != HAL_OK) {
         if(ENABLE_DEBUG) debug_print("Failed to send command to driver.\r\n");
         tmc2209->driver.STATUS = TMC_SEND_ERROR;
         return	NULL;
     }

     if(readLength && (tmc2209->driver.STATUS == TMC_OK)){

     // Wait for reply
     HAL_UART_Receive_DMA(tmc2209->driver.huart, rxData, readLength + 1);
     if (!TMC2209_WaitForReply(200)) { // Wait 200ms if no reply, timeout
    	 if(ENABLE_DEBUG) debug_print("No reply received from driver.\r\n");
    	 tmc2209->driver.STATUS = TMC_NOREPLY_ERROR;
         return NULL; // command failed
     }

     // Send transmitted data -- Note: this can't be called before receiving or else it will interfere with huart2
     if(ENABLE_DEBUG){
     debug_print("Data Transmitted: ");
     debug_print_hex(command, writeLength);
     // Process received data in rxBuffer
     debug_print("Reply received:\r\n");
     debug_print_hex(rxBuffer, TMC_REPLY_SIZE);
     }
     return rxBuffer; // Success
     }
     return NULL;
 }


void TMC2209_writeInit(Motor *tmc2209, uint8_t regAddress, int32_t value){
 	uint8_t write_request_command[8];
 	write_request_command[0] = SYNC; // SYNC Byte for TMC2209
 	write_request_command[1] = tmc2209->driver.address; // Driver address configure it using MS1(LSB) AND MS2
 	write_request_command[2] = regAddress | 0x80; // Register address to write 0x80 for writing
 	write_request_command[3] = (value >> 24) & 0xFF;
 	write_request_command[4] = (value >> 16) & 0xFF;
 	write_request_command[5] = (value >> 8 ) & 0xFF;
 	write_request_command[6] = (value      ) & 0xFF;
 	write_request_command[7] = calculate_CRC(write_request_command, 7); // checksum
 	TMC2209_sendCommand(&write_request_command[0], TMC_WRITE_DATAGRAM_SIZE, 0, tmc2209); // We don't actually need receive buffer here when we call ReadWrite so we just pass data

 }

int32_t TMC2209_readInit(Motor *tmc2209, uint8_t regAddress){
	if(tmc2209->driver.STATUS != TMC_OK) return tmc2209->driver.STATUS;
 	uint8_t read_request_command[8] = { 0 };
 	read_request_command[0] = SYNC;
 	read_request_command[1] = tmc2209->driver.address;
 	read_request_command[2] = regAddress;
 	read_request_command[3] = calculate_CRC(read_request_command, 3);

 	uint8_t *verifyBuffer = TMC2209_sendCommand(read_request_command, TMC_READ_REQUEST_DATAGRAM_SIZE, TMC_REPLY_SIZE, tmc2209);
 	// Byte 0: Sync nibble correct?
 	if (verifyBuffer[0] != 0x05){
 		// If first byte equals 0 then it means no reply so return
 		if(ENABLE_DEBUG) debug_print("Invalid data received!(SYNC Byte)\r\n");
 		return tmc2209->driver.STATUS = TMC_SYNC_REPLY_ERROR;
 	}
 	// Byte 1: Master address correct?
 	if (verifyBuffer[1] != 0xFF){
 		if(ENABLE_DEBUG) debug_print("Invalid data received!(MCU Address)\r\n");
 		return tmc2209->driver.STATUS = TMC_MCU_REPLY_ERROR;
 	}
 	// Byte 2: Register address correct?
 	if (verifyBuffer[2] != regAddress){
 		if(ENABLE_DEBUG) debug_print("Invalid data received!(Register Address)\r\n");
 		return tmc2209->driver.STATUS = TMC_REG_REPLY_ERROR;
 	}
 	// Byte 7: CRC correct?
 	if (verifyBuffer[7] != calculate_CRC(verifyBuffer, 7)){
 		if(ENABLE_DEBUG) debug_print("Invalid data received!(CRC)\r\n");
 		return tmc2209->driver.STATUS = TMC_CRC_REPLY_ERROR;
 	}
 	return (verifyBuffer[3] << 24) | (verifyBuffer[4] << 16) | (verifyBuffer[5] << 8) | verifyBuffer[6];
 }

bool TMC2209_setPDNuart(Motor *tmc2209, bool enable) {
    // Read current GCONF register value from TMC2209_REG_GCONF
    uint32_t currentGCONF = TMC2209_readInit(tmc2209, TMC2209_REG_GCONF);

    if (ENABLE_DEBUG) {
        char debug_msg[150];
        sprintf(debug_msg, sizeof(debug_msg), "Current GCONF = 0x%08lX\r\n", currentGCONF);
        debug_print(debug_msg);
    }

    // Modify the pdn_disable bit (bit 6). When set to 1, pdn_disable is enabled (UART controls the driver).
    // When cleared (0), PDN function is active.
    if (enable) {
        currentGCONF |= 0x00000040; // Set bit 6 to enable PDN_UART
        if (ENABLE_DEBUG) {
            debug_print("Enabling PDN_UART (pdn_disable=1).\r\n");
        }
    } else {
        currentGCONF &= ~(0x00000040); // Clear bit 6 to disable PDN_UART
        if (ENABLE_DEBUG) {
            debug_print("Disabling PDN_UART (pdn_disable=0).\r\n");
        }
    }

    // Write back the updated GCONF register value
    TMC2209_writeInit(tmc2209, TMC2209_REG_GCONF, currentGCONF);
    HAL_Delay(3); // Allow time for the write to complete

    // Optionally, check if the write was successful by re-reading the register
    uint32_t updatedGCONF = TMC2209_readInit(tmc2209, TMC2209_REG_GCONF);
    HAL_Delay(3);

    if (ENABLE_DEBUG) {
        char verify_msg[150];
        sprintf(verify_msg, sizeof(verify_msg), "Updated GCONF = 0x%08lX\r\n", updatedGCONF);
        debug_print(verify_msg);
    }

    // Verify that the pdn_disable bit matches the intended configuration
    bool bitSet = (updatedGCONF & 0x00000040) ? true : false;
    if (bitSet == enable) {
        return tmc2209->driver.pdn_disable = 1;
    } else {
        if (ENABLE_DEBUG) {
            debug_print("PDN_UART configuration failed to update correctly!\r\n");
        }
        return tmc2209->driver.pdn_disable = 0;
    }
}



void TMC2209_read_ifcnt(Motor *tmc2209) {
     int32_t ifcnt_value = TMC2209_readInit(tmc2209, TMC2209_REG_IFCNT); // IFCNT register address is 0x02
     if (ifcnt_value >= 0) { // This value gets incremented with every sucessful UART write access 0 to 255 then wraps around.
    	 if(ENABLE_DEBUG){
         char debug_msg[100];
         sprintf(debug_msg, "IFCNT Value: %d\r\n",  (int)ifcnt_value);
         debug_print(debug_msg);
    	 }
         tmc2209->driver.IFCNT = ifcnt_value;
     } else {
    	 if (ENABLE_DEBUG)
         debug_print("Failed to read IFCNT register!\r\n");
         tmc2209->driver.IFCNT = TMC_IFCNT_ERROR;
     }

 }

bool configureGCONF(Motor *tmc2209) {
	int ifcntCheck = tmc2209->driver.IFCNT;
	if (ifcntCheck == TMC_IFCNT_ERROR){
        if (ENABLE_DEBUG) debug_print("Failed to configure GCONF! (Failed to read IFCNT register)\r\n");
		return 0;
	}
    uint32_t gconf = 0x000000C0; // pdn_disable = 1, mstep_reg_select = 1
    TMC2209_writeInit(tmc2209, TMC2209_REG_GCONF, gconf);
    TMC2209_read_ifcnt(tmc2209);
    if (tmc2209->driver.IFCNT <= ifcntCheck){
    	if (ENABLE_DEBUG) debug_print("Failed to configure GCONF!(IFCNT DID NOT INCREMENT CHECK TMC_STATUS)\r\n");
    	tmc2209->driver.GCONF = TMC_GCONF_ERROR;
		return 0;
    }
    return (tmc2209->driver.GCONF = true);
}


uint16_t TMC2209_setSpreadCycle(Motor *tmc2209, uint8_t enable) {
	uint32_t gconf;
	uint32_t check_gconf;
	uint8_t driverID = tmc2209->driver.id;
	int32_t IFCNT = tmc2209->driver.IFCNT;

	if (ENABLE_DEBUG){
	char debug_msg[150];
	snprintf(debug_msg, sizeof(debug_msg), "Setting SpreadCycle Mode for Driver: %u\r\n", driverID);
	debug_print(debug_msg);
	}

	gconf = TMC2209_readInit(tmc2209, TMC2209_REG_GCONF);

    if(tmc2209->driver.STATUS != TMC_OK){
    	if (ENABLE_DEBUG) debug_print("Failed to set SpreadCycle Mode!(Invalid Reply 1)\r\n");
    	return gconf;
    }

    check_gconf = gconf;
    if(enable) {
    	gconf |= (1 << TMC2209_EN_SPREADCYCLE_POS);
    } else {
    	gconf &= ~(1 << TMC2209_EN_SPREADCYCLE_POS);
    }

    if(gconf == check_gconf){ //Setpread is already EN/DIS ABLED so skip and return
    	if (ENABLE_DEBUG) debug_print("Failed to set SpreadCycle Mode! (Spread is already on that Mode!)\r\n");
    	return enable;
    }

    TMC2209_writeInit(tmc2209, TMC2209_REG_GCONF, gconf);
    TMC2209_read_ifcnt(tmc2209);
    if(tmc2209->driver.IFCNT <= IFCNT){
    	tmc2209->driver.chopperMode = 0;
    	if (ENABLE_DEBUG) debug_print("Failed to set SpreadCycle Mode!\r\n");
    	return TMC_SET_SPREADCYCLE_ERROR;
    }

    check_gconf = TMC2209_readInit(tmc2209, TMC2209_REG_GCONF);
    if(check_gconf != gconf){
    	if (ENABLE_DEBUG) debug_print("Failed to set SpreadCycle Mode!(invalid Reply 2)\r\n");
    }

    tmc2209->driver.chopperMode = 1;
    return TMC_OK;
}

void checkSpreadCycle(Motor *tmc2209) {
    // Read the GCONF register
    uint32_t gconf = TMC2209_readInit(tmc2209, TMC2209_REG_GCONF);

    // Extract the EN_SPREADCYCLE bit (bit 2 in GCONF)
    uint8_t spreadCycleEnabled = (gconf >> 2) & 0x01;

    // Debug message
    if (spreadCycleEnabled) {
    	if (ENABLE_DEBUG) debug_print("SpreadCycle is ENABLED.\n");
    } else {
    	if (ENABLE_DEBUG) debug_print("SpreadCycle is DISABLED (StealthChop is active).\n");
    }

    tmc2209->driver.chopperMode = spreadCycleEnabled; // Return 1 if SpreadCycle is enabled, 0 otherwise
}


// Function to set the microstepping resolution through UART
uint32_t TMC2209_setMicrosteppingResolution(Motor *tmc2209, uint16_t resolution) {
    uint8_t driverID = tmc2209->driver.id;
    int32_t IFCNT = tmc2209->driver.IFCNT;

    char debug_msg[150];
    if (ENABLE_DEBUG){
    snprintf(debug_msg, sizeof(debug_msg), "Setting Microstepping For Driver ID: %u \r\n", driverID);
    debug_print(debug_msg);
    memset(debug_msg, 0, sizeof(debug_msg)); // clear buffer
    }
    // Ensure GCONF is set to enable UART control for microstepping resolution
    uint32_t gconf = TMC2209_readInit(tmc2209, TMC2209_REG_GCONF);
    gconf |= 0x80;  // / Bit 7 (mstep_reg_select) set to 1. This to change the option to control mstepping using UART instead of MS1 & MS2 pins
    HAL_Delay(3);
    TMC2209_writeInit(tmc2209, TMC2209_REG_GCONF, gconf);

    HAL_Delay(3);
    // Read the current CHOPCONF register value
    uint32_t currentCHOPCONF = TMC2209_readInit(tmc2209, TMC2209_REG_CHOPCONF);

    HAL_Delay(3);

    // Extract the current microstepping resolution (MRES) bits [24:27]
    uint8_t currentMRES = (currentCHOPCONF >> 24) & 0x0F;

    // Determine the MRES value for the new resolution
    uint8_t newMRES;
    switch (resolution) {
        case 256:
            newMRES = 0x00; // %0000 -> 256 microsteps
            break;
        case 128:
            newMRES = 0x01; // %0001 -> 128 microsteps
            break;
        case 64:
            newMRES = 0x02; // %0010 -> 64 microsteps
            break;
        case 32:
            newMRES = 0x03; // %0011 -> 32 microsteps
            break;
        case 16:
            newMRES = 0x04; // %0100 -> 16 microsteps
            break;
        case 8:
            newMRES = 0x05; // %0101 -> 8 microsteps
            break;
        case 4:
            newMRES = 0x06; // %0110 -> 4 microsteps
            break;
        case 2:
            newMRES = 0x07; // %0111 -> 2 microsteps
            break;
        case 1:
            newMRES = 0x08; // %1000 -> Full step
            break;
        default:
            newMRES = currentMRES; // Keep the current resolution if invalid value is provided
            break;
    }

    // If the resolution has not changed, do nothing
    if (newMRES == currentMRES) {
    	if (ENABLE_DEBUG) debug_print("Resolution unchanged, no update needed.\r\n");
        return tmc2209->driver.mstep = resolution;
    }
   // HAL_Delay(2);
    // Update the CHOPCONF register with the new MRES value
    uint32_t updatedCHOPCONF = (currentCHOPCONF & ~(0x0F << 24)) | (newMRES << 24);
    TMC2209_writeInit(tmc2209, TMC2209_REG_CHOPCONF, updatedCHOPCONF);
    HAL_Delay(3);

    TMC2209_read_ifcnt(tmc2209);

    if(tmc2209->driver.IFCNT <= IFCNT){
    	if (ENABLE_DEBUG) debug_print("Failed to set microstepping.\r\n");
    	return tmc2209->driver.mstep = TMC_SET_MSTEP_ERROR;
    }
    else{
        tmc2209->driver.mstep = resolution;
    }

    if (ENABLE_DEBUG) {
    	sprintf(debug_msg, "Updated microstepping resolution to: %d\r\n", resolution);
    	debug_print(debug_msg);
    }
    return TMC_OK;

}


void checkMicrosteppingResolution(Motor *tmc2209) {
	//HAL_Delay(2);
    // Read the CHOPCONF register
    uint32_t chopconf = TMC2209_readInit(tmc2209, TMC2209_REG_CHOPCONF);
    // Extract the MRES bits (bits 24–27 in CHOPCONF)
    uint8_t mres = (chopconf >> 24) & 0x0F;

    // Calculate the current microstepping resolution
    uint16_t resolution;
    switch (mres) {
        case 0x00: resolution = 256; break;
        case 0x01: resolution = 128; break;
        case 0x02: resolution = 64; break;
        case 0x03: resolution = 32; break;
        case 0x04: resolution = 16; break;
        case 0x05: resolution = 8; break;
        case 0x06: resolution = 4; break;
        case 0x07: resolution = 2; break;
        case 0x08: resolution = 1; break;
        default: resolution = 0; // Unknown value
    }

    // Debug
    if (ENABLE_DEBUG){
    char debug_msg[150];
    uint8_t driverID = tmc2209->driver.id;
    sprintf(debug_msg, "Current microstepping resolution for Driver ID: %u, Resolution: %u\n", driverID, resolution);
    debug_print(debug_msg);
    }
    tmc2209->driver.mstep = resolution;
}



uint16_t TMC2209_setIRUN(Motor *tmc2209, uint8_t irun_value) {
	int IFCNT = tmc2209->driver.IFCNT;
    if (irun_value > 31) {
        irun_value = 31; // Limit IRUN value to the maximum allowed (5 bits)
    }

    uint32_t registerValue = TMC2209_readInit(tmc2209, TMC2209_REG_IHOLD_IRUN);

    // Clear the current IRUN bits (Bits 8-12)
    registerValue &= ~(0x1F << 8);
    // Set the new IRUN value
    registerValue |= (irun_value << 8);

    // Write back the updated register value
    TMC2209_writeInit(tmc2209, TMC2209_REG_IHOLD_IRUN, registerValue);
    if(tmc2209->driver.IFCNT < IFCNT){
    	if (ENABLE_DEBUG) debug_print("Failed to set IRUN.\r\n");
    	return tmc2209->driver.IRUN = TMC2209_IRUN_ERROR;
    }

    tmc2209->driver.IRUN = irun_value;
    return TMC_OK;
}
//

void TMC2209_readIRUN(Motor *tmc2209) {
	HAL_Delay(1);
    uint32_t registerValue = TMC2209_readInit(tmc2209, TMC2209_REG_IHOLD_IRUN);

    // Extract IRUN (Bits 8-12)
    uint8_t irunValue = (registerValue >> 8) & 0x1F; // Mask and shift bits
    if (ENABLE_DEBUG){
    char debug_msg[100];
    sprintf(debug_msg, "Current IRUN value: %u\n", irunValue);
    debug_print(debug_msg);
    }

    tmc2209->driver.IRUN = irunValue;
}





void TMC2209_configureCurrent(Motor *tmc2209, uint8_t ihold, uint8_t irun, uint8_t iholddelay) {
	//  The bit assignments in the IHOLD_IRUN register are:
	//  Bits 0-4  : IHOLD (0-31)
	//  Bits 8-12 : IRUN (0-31)
	//  Bits 16-19: IHOLDDELAY (0-15)

    // Clamp values to allowed ranges
    if (ihold > 31) {
        ihold = 31;
    }
    if (irun > 31) {
        irun = 31;
    }
    if (iholddelay > 15) {
        iholddelay = 15;
    }
    uint32_t registerValue;
   // Clear the relevant bits for IHOLD (0-4), IRUN (8-12), and IHOLDDELAY (16-19)
    registerValue &= ~((0x1F) | (0x1F << 8) | (0x0F << 16));

    //   Set bits 0-4 with IHOLD, bits 8-12 with IRUN, and bits 16-19 with IHOLDDELAY.
    registerValue |= ((iholddelay & 0x0F) << 16) | ((irun & 0x1F) << 8) | (ihold & 0x1F);
    uint32_t IFCNT = tmc2209->driver.IFCNT;
    TMC2209_writeInit(tmc2209, TMC2209_REG_IHOLD_IRUN, registerValue);
    if(tmc2209->driver.IFCNT < IFCNT){
    	tmc2209->driver.IHOLD = TMC_IHOLD_ERROR;
    	tmc2209->driver.IRUN = TMC_IRUN_ERROR;
    	tmc2209->driver.IDELAY = TMC_IDELAY_ERROR;
    }

}

// Function to configure the SPREADCYCLE mode parameters in the CHOPCONF register
//void TMC2209_configureSpreadCycle(Motor *tmc2209, uint8_t toff, uint8_t tbl, uint8_t hend, uint8_t hstart) {
//	HAL_Delay(1);
//    // Validate and clamp the input parameters to their allowed ranges
//    if (toff < 2) toff = 2;      // Minimum TOFF value is 2
//    if (toff > 15) toff = 15;    // Maximum TOFF value is 15
//
//    if (tbl > 3) tbl = 3;        // TBL has a range of 0 to 3 (2-bit field)
//
//    if (hend > 15) hend = 15;    // HEND has a range of 0 to 15 (4-bit field)
//    if (hstart < hend) hstart = hend + 1; // HSTART must be greater than HEND
//    if (hstart > 15) hstart = 15; // Maximum HSTART value is 15
//    // Read the current CHOPCONF value
//     uint32_t chopconf = TMC2209_readInit(tmc2209, TMC2209_REG_CHOPCONF);
//     if (chopconf == -1) {
//    	 if (ENABLE_DEBUG) debug_print("Error reading CHOPCONF register.\r\n");
//         return 0;
//     }
//
//     // Preserve MRES (bits 24–27) and other existing settings
//     chopconf &= 0xF0000000; // Clear TOFF, TBL, HEND, and HSTART bits while preserving MRES
//
//     // Set new values for TOFF, TBL, HEND, and HSTART
//     chopconf |= (toff & 0xF) << 0;  // TOFF (bits 0–3)
//     chopconf |= (tbl & 0x3) << 15;  // TBL (bits 15–16)
//     chopconf |= (hend & 0xF) << 7;  // HEND (bits 7–11)
//     chopconf |= (hstart & 0xF) << 4; // HSTART (bits 4–6)
//
//     // Write the modified CHOPCONF value back to the driver
//     TMC2209_writeInit(tmc2209, TMC2209_REG_CHOPCONF, chopconf);
//
//    // Debug: Print the configured CHOPCONF value
//    debug_print("SPREADCYCLE parameters configured: ");
//    debug_print_hex((uint8_t *)&chopconf, 4);
//    debug_print("\r\n");
//}


uint16_t TMC2209_setSendDelay(Motor *tmc2209, uint8_t sendDelay) { // The SENDDELAY field uses 4 bits (bits 11..8).
	// The datasheet recommends SENDDELAY >= 2 in multi-node setups.

	if (sendDelay > 15) sendDelay = 15; // clamp the value to 4 bits max
	if (ENABLE_DEBUG) debug_print("----- Reading Send Delay ----- \r\n");
	uint32_t nodeconf = TMC2209_readInit(tmc2209, TMC_REG_SENDDELAY);	// Read the existing NODECONF register
	nodeconf &= ~(0x0F << 8);	// Clear bits 11..8
	nodeconf |= ((sendDelay & 0x0F) << 8);	// Set bits 11..8 to sendDelay
	int32_t IFCNT = tmc2209->driver.IFCNT;
	TMC2209_writeInit(tmc2209, TMC2209_REG_SLAVECONF, nodeconf);	// Write back the updated value
    TMC2209_read_ifcnt(tmc2209);
    if (tmc2209->driver.IFCNT <= IFCNT){
    	if(ENABLE_DEBUG) debug_print("Failed to set Send Delay! \r\n");
    	return tmc2209->driver.sendDelay = TMC2209_SENDELAY_ERROR;
    }
    tmc2209->driver.sendDelay = sendDelay;
    if(ENABLE_DEBUG) debug_print("Send Delay set successfully! \r\n");
    return TMC_OK;

}

float TMC2209_readTemperature(Motor *tmc2209)
{
    uint32_t drv_status = TMC2209_readInit(tmc2209, TMC2209_REG_DRVSTATUS);
    uint8_t t120 = (drv_status >> 10) & 0x01;
    uint8_t t143 = (drv_status >> 9) & 0x01;
    uint8_t t150 = (drv_status >> 8) & 0x01;
    uint8_t t157 = (drv_status >> 7) & 0x01;

    if(t157) return 157.0f;
    if(t150) return 150.0f;
    if(t143) return 143.0f;
    if(t120) return 120.0f;

    return 25.0f;
}


uint8_t TMC2209_enableStallDetection(Motor *tmc2209, uint8_t sgthrs) {
	int32_t IFCNT = tmc2209->driver.IFCNT;

    TMC2209_writeInit(tmc2209, TMC2209_REG_SGTHRS, sgthrs);    // Set StallGuard threshold (SGTHRS)

    TMC2209_read_ifcnt(tmc2209);
    if (tmc2209->driver.IFCNT <= IFCNT){
    	if(ENABLE_DEBUG) debug_print("Failed to set Send Delay! \r\n");
    	return tmc2209->driver.stallEnabled = TMC_ENABLESTALL_ERROR;
    }


    return tmc2209->driver.stallEnabled = 1;

}

void TMC2209_SetTCoolThrs(Motor *tmc2209, uint32_t stepFrequency) {
    const uint32_t fCLK = 12000000; // TMC2209 Internal clock frequency: 12 MHz
    uint32_t tStep = fCLK / stepFrequency; // The internal clokc trims step frequency that's why we divied it.

    // Ensure tStep doesn't exceed 20 bits (valid for TCOOLTHRS register)
    if (tStep > 0xFFFFF) {
        tStep = 0xFFFFF;
    }

    int32_t IFCNT = tmc2209->driver.IFCNT;

    if (tmc2209->driver.IFCNT <= IFCNT){
    	if(ENABLE_DEBUG) debug_print("Failed to set Send Delay! \r\n");
    	tmc2209->driver.TCoolThrs = TMC2209_TCOOLTHRS_ERROR;
    }
    TMC2209_writeInit(tmc2209, TMC2209_REG_TCOOLTHRS, tStep);
    tmc2209->driver.TCoolThrs = tStep;
}

void TMC2209_readSGResult(Motor *tmc2209) { // IMPORTANT: The SG_RESULT becomes updated with each fullstep, independent of TCOOLTHRS and SGTHRS
    uint32_t sg_result = 0;

    // Read the SG_RESULT register
    sg_result = TMC2209_readInit(tmc2209, TMC2209_REG_SG_RESULT) & 0x3FF; // Mask 10 bits

    if (sg_result == tmc2209->driver.STATUS) {
        tmc2209->driver.SG_RESULT = TMC_STALL_ERROR;
    }

    tmc2209->driver.SG_RESULT = sg_result;
}

bool TMC2209_readStandstillIndicator(Motor *tmc2209) {
    // Read the DRV_STATUS register. Ensure that TMC2209_REG_DRV_STATUS is defined, typically 0x6F.
    uint32_t drvStatus = TMC2209_readInit(tmc2209, TMC2209_REG_DRVSTATUS);

    // Debug: output the DRV_STATUS register value if debug is enabled
    #if ENABLE_DEBUG
        char debug_msg[100];
        sprintf(debug_msg, "DRV_STATUS = 0x%08lX\r\n", drvStatus);
        debug_print(debug_msg);
    #endif

    // Check the standstill bit. stst is typically bit 31.
    if(drvStatus & (1UL << 31)) {
        return tmc2209->driver.standstill;  // Motor is at standstill
    } else {
        return tmc2209->driver.standstill; // Motor is not at standstill
    }
}



void TMC2209_resetMotorsConfiguration(Motor *motors){ // Reset all drivers to Default

    for (uint8_t i = 0; i < MAX_MOTORS; i++) {
    	configureGCONF(&motors[i]);

    	TMC2209_setMicrosteppingResolution(&motors[i], DEFAULT_MSTEP);
        TMC2209_setSpreadCycle(&motors[i], DEFAULT_CHOPPERMODE);

        TMC2209_SetSpeed(&motors[0], DEFAULT_Y_SPEED);
        TMC2209_SetSpeed(&motors[1], DEFAULT_Y_SPEED);
        TMC2209_SetSpeed(&motors[2], DEFAULT_X_SPEED);
        TMC2209_SetSpeed(&motors[3], DEFAULT_X_SPEED);
    }

}

void TMC2209_checkStall(Motor *motor){
	   if(motor->driver.checkSG_RESULT){ // One full step is done so we read sg_Result
		   TMC2209_readSGResult(motor);
	   }
}




