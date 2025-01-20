/*
 * PCB2GCODE.c
 *
 *  Created on: Jan 16, 2025
 *      Author: ahmed
 */

#include "uart_handler.h"
#include "command_parser.h"

uint8_t rxData[TMC_REPLY_SIZE + 1]; // +1 for TX
uint8_t rxBuffer[TMC_REPLY_SIZE];
volatile uint8_t rxBufferReady = 0;	// Flag to indicate data reception

uint8_t uart3_rxData[PCB2GCODE_REPLY_SIZE];
volatile bool uart3_commandReceived = false;

// UART Receive Complete Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// UART callback for read from TMC2209
    if (huart->Instance == USART2 || huart->Instance == USART6 || huart->Instance == UART4 || huart->Instance == UART5) {
        // Prevent buffer overflow by iterating only up to TMC_REPLY_SIZE
        memcpy(rxBuffer, rxData + 1, TMC_REPLY_SIZE+1);
        rxBufferReady = 1;

    }

    // TODO: UART callback for read from PCB2Gcode

}


void handleUARTCommands(void) {
    // Handle UART3 Commands (PCB2Gcode)
    if (uart3_commandReceived) {
        uart3_commandReceived = false;

        static char buffer3[PCB2GCODE_REPLY_SIZE];
        static uint8_t buf3_idx = 0;

        for (uint8_t i = 0; i < PCB2GCODE_REPLY_SIZE; i++) {
            if (uart3_rxData[i] == '\n' || uart3_rxData[i] == '\r' || uart3_rxData[i] == 0) {
                buffer3[buf3_idx] = '\0';
                buf3_idx = 0;

                if (strlen(buffer3) > 0) {
                    parsePCB2GcodeCommand(buffer3);
                }

                // Reset buffer
                memset(buffer3, 0, PCB2GCODE_REPLY_SIZE);
            }
            else {
                if (buf3_idx < PCB2GCODE_REPLY_SIZE - 1) {  // Prevent buffer overflow
                    buffer3[buf3_idx++] = uart3_rxData[i];
                }
            }
        }
    }
}





