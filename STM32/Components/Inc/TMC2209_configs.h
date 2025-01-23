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
#ifndef TMC2209_CONFIGS_H
#define TMC2209_CONFIGS_H

#include "stm32f7xx_hal.h"
#include "stdbool.h"



// Timer Handler declaration for motors
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

// Motors & axis
#define MAX_MOTORS 4 // Max motors to be added -- You can handle upto 4 TMC2209 drivers on the same UART BUS
#define MAX_MOTORS_PER_AXIS 2 // Num of motors per axis
#define X_AXIS_LENGTH	469.356
#define Y_AXIS_LENGTH 	300



// Driver structure
typedef struct {
	uint8_t id;                      // Motor ID to identify each motor -- this is useless now ince we have address for uart
					//	Microstepping setting of the driver
    UART_HandleTypeDef *huart;
    uint8_t address;                // UART address
    uint32_t STATUS;
    TIM_HandleTypeDef *htim;        // Timer handle for PWM generation
    uint32_t step_channel;          // PWM channel

    // Motor Configurations
    uint16_t mstep;	     // Microstepping: 2,8,16,256, or FULLSTEP
    uint8_t chopperMode; // 1 for StealthChop, 0 for SpreadCycle. Default: 1
    uint8_t sendDelay;
    uint32_t IFCNT; // Sucessful write incrementing counter(This value gets incremented with every sucessful UART write access 0 to 255 then wraps around.)
    uint8_t GCONF;
    uint16_t IRUN;
    uint16_t IHOLD;
    uint16_t IDELAY;

    // GPIO PINS
    GPIO_TypeDef *step_port;
    uint16_t step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef *enn_port;
    uint16_t enn_pin;
    GPIO_TypeDef *diag_port;
    uint16_t diag_pin;
    GPIO_TypeDef *index_port;
    uint16_t index_pin;




} TMC2209_Driver;

// Motor tracking

typedef struct {
    TMC2209_Driver driver;         // Driver settings for the motor
    uint32_t stepsPerRevolution;	// Steps per revolution of the motor
    uint32_t totalStepsPerRevolution; // we will calculate this later based on the mstep option the driver has
    int32_t stepsTaken;           // Count of steps taken
    uint32_t nextTotalSteps;           // Total steps the motor should take
    float currentPositionMM; // Current position on the axis in millimeters
    float nextPositionMM;
    bool isStepping;               // State to track if the motor is currently stepping
    uint32_t StepsFront;           // Tracking Steps Based on direction
    int32_t StepsBack;
    uint16_t calib[2];
} Motor;

// Axis structure
typedef struct {
    Motor *motors[MAX_MOTORS_PER_AXIS];           // Pointing to which motors controlling this axis
    float lengthMM;         // Total length of the axis in millimeters
    uint32_t totalSteps;    // Total steps the motor takes to cover the axis length -- You can get this value using limit switches
    float stepPerUnit;      // How much one step would move on the axis -- Calculate this
    char id[MAX_MOTORS_PER_AXIS][10];          //  Axis name with Motor ID (e.g., "X1", Y1", "Z1") "1" stands for the motor ID which control the axis
} Axis;




extern Motor motors[MAX_MOTORS]; // Global motor array
extern Axis axes[MAX_MOTORS_PER_AXIS];


// Function declarations
void initializeMotors();
void initializeAxis(Axis *axis, Motor *motor1, Motor *motor2, uint8_t circumference, const char *axisName);
void initializeSystem();


#endif // TMC2209_CONFIGS_H
