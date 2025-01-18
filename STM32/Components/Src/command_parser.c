/*
 * command_parser.c
 *
 *  Created on: Jan 16, 2025
 *      Author: ahmed
 */


//#include "uart_handler.h"
#include "TMC2209.h"
#include "TMC2209_configs.h"


extern Motor motors[];


// Function to parse and execute commands
void parsePCB2GcodeCommand(char *command) {
    char cmd[20];
    char param1[20];
    char param2[20];
    char param3[20];
    uint8_t motorID;
    uint8_t value;
    bool enable;


    // Extract the command parts
    int args = sscanf(command, "%s %s %s %s", cmd, param1, param2, param3);
    // Parse 'SET SENDDELAY 1 2'
    if (args >= 3) {

        if (strcmp(cmd, "SET") == 0) {
            if (strcmp(param1, "SENDDELAY") == 0 && args == 4) {
            	motorID = atoi(param2) - 1;
                value = atoi(param3);
                if (motorID < MAX_MOTORS) {
                    TMC2209_setSendDelay(&motors[motorID], value);
                    HAL_UART_Transmit(&huart2, (uint8_t *)"SENDDELAY set\n", strlen("SENDDELAY set\n"), HAL_MAX_DELAY);
                }
            }

            else if (strcmp(param1, "PDNUART") == 0 && args == 4) {
            	motorID = atoi(param2) - 1;
                enable = (strcmp(param3, "ON") == 0) ? true : false;
                if (motorID < MAX_MOTORS) {
                    if (enable) {
                        TMC2209_enable_PDNuart(&motors[motorID]);
                        HAL_UART_Transmit(&huart2, (uint8_t *)"PDNUART enabled\n", strlen("PDNUART enabled\n"), HAL_MAX_DELAY);
                    } else {
                        // Implement disable function if needed
                        // Example:
                        uint32_t gconf = TMC2209_readInit(&motors[motorID], TMC2209_REG_GCONF);
                        gconf &= ~(1 << 6);  // Clear pdn_disable bit
                        TMC2209_writeInit(&motors[motorID], TMC2209_REG_GCONF, gconf);
                        HAL_UART_Transmit(&huart2, (uint8_t *)"PDNUART disabled\n", strlen("PDNUART disabled\n"), HAL_MAX_DELAY);
                    }
                }
            }

            else if (strcmp(param1, "MICROSTEPPING") == 0 && args == 4) {
            	motorID = atoi(param2) - 1;
                value = atoi(param3);
                if (motorID < MAX_MOTORS) {
                    TMC2209_setMicrosteppingResolution(&motors[motorID], value);
                    HAL_UART_Transmit(&huart2, (uint8_t *)"Microstepping set\n", strlen("Microstepping set\n"), HAL_MAX_DELAY);
                }
            }

            else if (strcmp(param1, "SPREADCYCLE") == 0 && args == 4) {
            	motorID = atoi(param2) - 1;
                enable = (strcmp(param3, "ON") == 0) ? true : false;
                if (motorID < MAX_MOTORS) {
                    TMC2209_setSpreadCycle(&motors[motorID], enable);
                    if(enable){
                        HAL_UART_Transmit(&huart2, (uint8_t *)"SpreadCycle Enabled\n", strlen("SpreadCycle Enabled\n"), HAL_MAX_DELAY);
                    }
                    else{
                        HAL_UART_Transmit(&huart2, (uint8_t *)"SpreadCycle Disabled\n", strlen("SpreadCycle Disabled\n"), HAL_MAX_DELAY);
                    }

                }
            }
        }
    }
}
