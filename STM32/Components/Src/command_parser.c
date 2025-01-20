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
    char cmd[20], param1[20], param2[20], param3[20], param4[20];
    uint8_t motorID;
    uint8_t value;

    // Extract the command parts, expecting up to five
    int args = sscanf(command, "%s %s %s %s %s", cmd, param1, param2, param3, param4);

    // Expected command example: "SET DRIVER 0 CHOPPERMODE StealthChop"
    if (strcmp(cmd, "SET") == 0 && strcmp(param1, "DRIVER") == 0 && args == 5) {
        motorID = atoi(param2);

        if (motorID >= MAX_MOTORS) {
            debug_print("Invalid Driver ID.\r\n");
            return;
        }

        if (strcmp(param3, "CHOPPERMODE") == 0) {
            if (strcmp(param4, "StealthChop") == 0) {
                TMC2209_setSpreadCycle(&motors[motorID], 0);
            }
            else if (strcmp(param4, "SpreadCycle") == 0) {
                TMC2209_setSpreadCycle(&motors[motorID], 1);
            }
            else {
                debug_print("Unknown CHOPPERMODE option.\r\n");
            }
        }
        else if (strcmp(param3, "SENDDELAY") == 0) {
            value = atoi(param4);
            TMC2209_setSendDelay(&motors[motorID], value);
            debug_print("SENDDELAY set\r\n");
        }
        else if (strcmp(param3, "MICROSTEPPING") == 0) {
            value = atoi(param4);
            TMC2209_setMicrosteppingResolution(&motors[motorID], value);
            debug_print("Microstepping set\r\n");
        }
        else {
            debug_print("Unknown parameter for DRIVER.\r\n");
        }
    }
    else {
        debug_print("Invalid command format.\r\n");
    }
}
