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
#include "TMC2209_configs.h"


// UART declaration
// UART_HandleTypeDef huart2; in my case it's already generated in main.c


// Motors & axis
Motor motors[MAX_MOTORS];
Axis axes[MAX_MOTORS_PER_AXIS - 1];


void initializeMotors() {
    // Initialize each motor in the array
    for (int i = 0; i < MAX_MOTORS; i++) {
    	// Setting all for all drivers/motors
    	motors[i].driver.huart = &huart2; // UART handler
    	motors[i].driver.address = 0x00+i; // Address : 0x00, 0x01 ... Depends on MS1 AND MS2

    	// Motor Parameters
    	motors[i].driver.id = i + 1;

        motors[i].stepsTaken = 0;
        motors[i].nextTotalSteps = 0;
        motors[i].currentPositionMM = 0;
        motors[i].nextPositionMM = 0;
        motors[i].isStepping = false;



        if(i == 0){
         // Configure motor 1 X-axis

        // TIMER configurations
        motors[i].driver.htim = &htim2;				 // TIMER HANDLER
        motors[i].driver.step_channel = TIM_CHANNEL_3; // PWM channel for motor 1
        motors[i].driver.mstep = 8;
        motors[i].stepsPerRevolution = 200;
        // GPIO PINS
        motors[i].driver.step_port = GPIOB;
        motors[i].driver.step_pin = GPIO_PIN_10;
        motors[i].driver.dir_port = GPIOF;
        motors[i].driver.dir_pin = GPIO_PIN_9;
        motors[i].driver.enn_port = GPIOB;
        motors[i].driver.enn_pin = GPIO_PIN_11;
        motors[i].driver.diag_port = GPIOD;
        motors[i].driver.diag_pin = GPIO_PIN_1;
        motors[i].driver.index_port = GPIOA;
        motors[i].driver.index_pin = GPIO_PIN_5;
        }


        if(i == 1){
        	// Configure motor 2 X-axis
            // TIMER configurations
            motors[i].driver.htim = &htim3;				 // TIMER HANDLER
            motors[i].driver.step_channel = TIM_CHANNEL_1; // PWM channel for motor 1
            motors[i].driver.mstep = 16;
            motors[i].stepsPerRevolution = 400;
            // GPIO PINS
            motors[i].driver.step_port = GPIOA;
            motors[i].driver.step_pin = GPIO_PIN_6;
            motors[i].driver.dir_port = GPIOA;
            motors[i].driver.dir_pin = GPIO_PIN_7;
            motors[i].driver.enn_port = GPIOA;
            motors[i].driver.enn_pin = GPIO_PIN_5;
            motors[i].driver.diag_port = GPIOD;
            motors[i].driver.diag_pin = GPIO_PIN_1;
            motors[i].driver.index_port = GPIOA;
            motors[i].driver.index_pin = GPIO_PIN_5;

        }


        if(i == 2){
        	// Configure motor 3 Y-axis
            // TIMER configurations
            //motors[i].driver.htim = &htim3;				 // TIMER HANDLER
           // motors[i].driver.step_channel = TIM_CHANNEL_1; // PWM channel for motor 1
//            motors[i].driver.mstep = 2;
//            motors[i].stepsPerRevolution = 400;
//            // GPIO PINS
//            motors[i].driver.step_port = GPIOB;
//            motors[i].driver.step_pin = GPIO_PIN_10;
//            motors[i].driver.dir_port = GPIOF;
//            motors[i].driver.dir_pin = GPIO_PIN_9;
//            motors[i].driver.enn_port = GPIOB;
//            motors[i].driver.enn_pin = GPIO_PIN_11;
//            motors[i].driver.diag_port = GPIOD;
//            motors[i].driver.diag_pin = GPIO_PIN_1;
//            motors[i].driver.index_port = GPIOA;
//            motors[i].driver.index_pin = GPIO_PIN_5;

        }

        if(i == 3){
        	// Configure motor 4 Y-axis
            // TIMER configurations
            //motors[i].driver.htim = &htim3;				 // TIMER HANDLER
            //motors[i].driver.step_channel = TIM_CHANNEL_1; // PWM channel for motor 1
//            motors[i].driver.mstep = 2;
//            motors[i].stepsPerRevolution = 400;
//            // GPIO PINS
//            motors[i].driver.step_port = GPIOB;
//            motors[i].driver.step_pin = GPIO_PIN_10;
//            motors[i].driver.dir_port = GPIOF;
//            motors[i].driver.dir_pin = GPIO_PIN_9;
//            motors[i].driver.enn_port = GPIOB;
//            motors[i].driver.enn_pin = GPIO_PIN_11;
//            motors[i].driver.diag_port = GPIOD;
//            motors[i].driver.diag_pin = GPIO_PIN_1;
//            motors[i].driver.index_port = GPIOA;
//            motors[i].driver.index_pin = GPIO_PIN_5;
        }



    }


}


void initializeAxis(Axis *axis, Motor *motor1, Motor *motor2, uint8_t circumference, const char *axisName) {
    // Assign motors to the axis
    axis->motors[0] = motor1;
    axis->motors[1] = motor2;
    // The circumference variable is calculated based on the physical setup. For example: GT2 20-tooth pulley with 2mm pitch(Pulley Circumference = Number of Teeth * Belt Pitch)

    // Axis dimensions and step calculations
    uint32_t totalStepsPerRevolution = motor1->stepsPerRevolution * motor1->driver.mstep; // Both motors use the same microstepping
    motor1->totalStepsPerRevolution = totalStepsPerRevolution;
    motor2->totalStepsPerRevolution = totalStepsPerRevolution;
    axis->stepPerUnit = totalStepsPerRevolution / circumference;;

    // IDs for motors controlling the axis, eg. X1, X2
    snprintf(axis->id[0], sizeof(axis->id[0]), "%s%d", axisName, motor1->driver.id);
    if (motor2 != NULL) {
        snprintf(axis->id[1], sizeof(axis->id[1]), "%s%d", axisName, motor2->driver.id);
    }
}

void initializeSystem(){
    // X-axis
    initializeAxis(&axes[0], &motors[0],&motors[0], 400, "X");

    // Y-axis
   // initializeAxis(&axes[0], &motors[2], &motors[3], Y_AXIS_LENGTH, "Y");
    // TODO: ADD Z-AXIS should be a servo
}
