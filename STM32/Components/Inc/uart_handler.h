/*
 * pcb2gcode.h
 *
 *  Created on: Jan 16, 2025
 *      Author: ahmed
 */

#ifndef INC_UART_HANDLER_H_
#define INC_UART_HANDLER_H_

#include "stm32f7xx_hal.h"
#include <stdbool.h>
// Receive Datagrams
#define TMC_REPLY_SIZE	8	// Actual Bytes that TMC2209 sends back
#define PCB2GCODE_REPLY_SIZE 128

extern UART_HandleTypeDef huart2; // Used for TMC2209 drivers(0,1)
extern UART_HandleTypeDef huart6; // Used for TMC2209 drivers(2,3)
extern UART_HandleTypeDef huart3; // Used for debugging/PCB2Gcode Software

extern uint8_t rxData[TMC_REPLY_SIZE + 1]; // +1 for TX
extern uint8_t rxBuffer[TMC_REPLY_SIZE];
extern volatile uint8_t rxBufferReady;	// Flag to indicate data reception

extern uint8_t p2gRxData[PCB2GCODE_REPLY_SIZE];
extern volatile bool uart3_commandReceived;


#endif /* INC_UART_HANDLER_H_ */
