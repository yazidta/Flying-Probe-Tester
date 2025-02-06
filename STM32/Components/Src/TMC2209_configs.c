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
 *             be included in all copies or substantial Portions of the
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
#include "TMC2209_configs.h"
#include "app_tasks.h"

// UART declaration
// UART_HandleTypeDef huart2; in my case it's already generated in main.c


// Motors & axis
extern Motor motors[MAX_MOTORS];
extern Axis axes[MAX_MOTORS_PER_AXIS];


void initializeMotors() {
    // Initialize each motor in the array
    for (int i = 0; i < MAX_MOTORS; i++) {
    	// Setting all for all drivers/motors
    	 // UART handler
    	motors[i].driver.address = 0x00; // Address : 0x00, 0x01 ... Depends on MS1 AND MS2

    	// Motor Parameters
    	motors[i].driver.id = i ;
    	motors[i].driver.STATUS = TMC_OK;
        motors[i].driver.GCONF = 0;
        motors[i].driver.IFCNT = 0;
        motors[i].driver.chopperMode = 0;
        motors[i].driver.SG_RESULT = 0;
        motors[i].driver.checkSG_RESULT = 0;
        motors[i].driver.TCoolThrs = 0;
        motors[i].driver.stepFrequency = 0;

        motors[i].fullSteps = 0;
        motors[i].stepsTaken = 0;
        motors[i].nextTotalSteps = 0;
        motors[i].currentPositionMM = 0;
        motors[i].nextPositionMM = 0;
        motors[i].isStepping = false;
        motors[i].STALL = 0;


        if(i == 0){
         // Configure motor 1 X-axis

        // TIMER configurations
        motors[i].driver.huart = &huart2;
        motors[i].driver.htim = &htim2;				 // TIMER HANDLER
        motors[i].driver.step_channel = TIM_CHANNEL_3; // PWM channel for motor 1
        motors[i].driver.mstep = 16;
        motors[i].driver.IRUN = 16;
        motors[i].driver.IHOLD = 16;
        motors[i].stepsPerRevolution = 200;
        // GPIO PINS
        motors[i].driver.step_port = STEP1_GPIO_Port;
        motors[i].driver.step_pin = STEP1_Pin;
        motors[i].driver.dir_port = DIR1_GPIO_Port;
        motors[i].driver.dir_pin = DIR1_Pin;
        motors[i].driver.enn_port = ENN1_GPIO_Port;
        motors[i].driver.enn_pin = ENN1_Pin;
        motors[i].driver.diag_port = DIAG1_GPIO_Port;
        motors[i].driver.diag_pin = DIAG1_Pin;


        }


        if(i == 1){
        	// Configure motor 2 X-axis
            // TIMER configurations
            motors[i].driver.htim = &htim1;				 // TIMER HANDLER
            motors[i].driver.step_channel = TIM_CHANNEL_4; // PWM channel for motor 1
            motors[i].driver.mstep = 16;
            motors[i].driver.huart = &huart4;
            motors[i].stepsPerRevolution = 200;
            // GPIO PINS
            motors[i].driver.step_port = STEP2_GPIO_Port;
            motors[i].driver.step_pin = STEP2_Pin;
            motors[i].driver.dir_port = DIR2_GPIO_Port;
            motors[i].driver.dir_pin = DIR2_Pin;
            motors[i].driver.enn_port = ENN2_GPIO_Port;
            motors[i].driver.enn_pin = ENN2_Pin;
            motors[i].driver.diag_port = DIAG2_GPIO_Port;
            motors[i].driver.diag_pin = DIAG2_Pin;


        }
        if(i == 2){
        	        	// Configure motor 3 Y-axis
        	            // TIMER configurations
            motors[i].driver.htim = &htim5;				 // TIMER HANDLER
        	motors[i].driver.step_channel = TIM_CHANNEL_1; // PWM channel for motor 1
            motors[i].driver.mstep = 16;
            motors[i].driver.huart = &huart5;
        	motors[i].stepsPerRevolution = 400;
        	            // GPIO PINS
            motors[i].driver.step_port = STEP3_GPIO_Port;
            motors[i].driver.step_pin = STEP3_Pin;
            motors[i].driver.dir_port = DIR3_GPIO_Port;
            motors[i].driver.dir_pin = DIR3_Pin;
            motors[i].driver.enn_port = ENN3_GPIO_Port;
            motors[i].driver.enn_pin = ENN3_Pin;
            motors[i].driver.diag_port = DIAG3_GPIO_Port;
            motors[i].driver.diag_pin = DIAG3_Pin;


        }

        if(i == 3){
        	// Configure motor 4 Y-axis
            // TIMER configurations
            motors[i].driver.htim = &htim3;				 // TIMER HANDLER
            motors[i].driver.step_channel = TIM_CHANNEL_3; // PWM channel for motor 1
            motors[i].driver.mstep = 16;
            motors[i].driver.huart = &huart6;
            motors[i].stepsPerRevolution = 400;
            // GPIO PINS
            motors[i].driver.step_port = STEP4_GPIO_Port;
            motors[i].driver.step_pin = STEP4_Pin;
            motors[i].driver.dir_port = DIR4_GPIO_Port;
            motors[i].driver.dir_pin = DIR4_Pin;
            motors[i].driver.enn_port = ENN4_GPIO_Port;
            motors[i].driver.enn_pin = ENN4_Pin;
            motors[i].driver.diag_port = DIAG4_GPIO_Port;
            motors[i].driver.diag_pin = DIAG4_Pin;
        }



    }


}


void initializeAxis(Axis *axis, Motor *motor1, Motor *motor2, uint8_t circumference, const char *axisName) {
    // Assign motors to the axis
    axis->motors[0] = motor1;
    axis->motors[1] = motor2;
    // The circumference variable is calculated based on the physical setup. For example: GT2 20-tooth pulley with 2mm pitch(Pulley Circumference = Number of Teeth * Belt Pitch)

    // Axis dimensions and step calculations
    axis->motors[0]->currentPositionMM = 0;
    axis->motors[1]->currentPositionMM = 0;
    uint32_t totalStepsPerRevolution = motor1->stepsPerRevolution * motor1->driver.mstep; // Both motors use the same microstepping
    motor1->totalStepsPerRevolution = totalStepsPerRevolution;
    motor2->totalStepsPerRevolution = totalStepsPerRevolution;
    axis->stepPerUnit = totalStepsPerRevolution / circumference;

    // IDs for motors controlling the axis, eg. X1, X2
    snprintf(axis->id[0], sizeof(axis->id[0]), "%s%d", axisName, motor1->driver.id);
    if (motor2 != NULL) {
        snprintf(axis->id[1], sizeof(axis->id[1]), "%s%d", axisName, motor2->driver.id);
    }
}

void initializeSystem(){
	// motors
		initializeMotors();
    // axis
		initializeAxis(&axes[0], &motors[0],&motors[1], 8, "Y");
		initializeAxis(&axes[1], &motors[2],&motors[3], 40, "X");

    // motor configurations
		TMC2209_setMotorsConfiguration(&motors,2, 0);
		 TMC2209_EnableDriver(&motors[0], 1);
		  TMC2209_EnableDriver(&motors[1], 1);
		  TMC2209_EnableDriver(&motors[2], 1);
		  TMC2209_EnableDriver(&motors[3], 1);
		xSemaphoreGive(xInitSemaphore); // signal welcome menu state to proceed with next state

}


