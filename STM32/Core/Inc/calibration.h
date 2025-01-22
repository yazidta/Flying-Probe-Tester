/*
 * calibration.h
 *
 *  Created on: Jan 22, 2025
 *      Author: ahmed
 */
#include "TMC2209.h"
#include "TMC2209_configs.h"
#include "main.h"
#include "stdbool.h"

bool MotorsHoming(Motor *motor);
void MotorControl_ButtonHandler(Motor *motors);
