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
#include "extras.h"
extern SERVO_Handle_TypeDef hservo1;
extern SERVO_Handle_TypeDef hservo2;
bool MotorsHoming(Motor *motor);
void ManualCalibration(Axis *axes, Motor *motors);
void motorHoming(Motor *motor, GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, int direction, int speed, int homePositionMM);
bool calibrationState();
void AutoCalibration(Axis *axes ,Motor *motors);
void semiAutoCalibration(Axis *axes, Motor *motors);
void moveMotorUntilStallAndCalibrate(Motor *motor, uint8_t motorIndex, uint32_t speed,GPIO_PinState direction, uint8_t calibrationIndex);



