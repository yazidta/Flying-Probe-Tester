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
void MotorControl_ButtonHandler(Axis *axes, Motor *motors);
void motorHoming(Motor *motor, GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, int direction, int speed, int homePositionMM);
