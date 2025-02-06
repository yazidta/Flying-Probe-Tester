/*
 * sd_card_reader.h
 *
 *  Created on: Dec 17, 2024
 *      Author: Konrad
 */

#ifndef INC_SD_CARD_READER_H_
#define INC_SD_CARD_READER_H_

#include "main.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h> // for va_list var arg functions
#include <stdbool.h> // for 'bool', 'true', and 'false'

// Maximum G-code line length
#define BUFFER_SIZE 128        // Buffer size for reading file chunks
#define LINE_BUFFER_SIZE 256
#define MAX_LINE_LENGTH 256
#define MAX_COORD_COUNT 500

extern float coordinates_x[MAX_COORD_COUNT];
extern float coordinates_y[MAX_COORD_COUNT];
extern float coordinates_z[MAX_COORD_COUNT];
// Function prototypes
void process_raw_gcode(FIL *fil);
void sd_card_read_gcode();
void uart_transmit_string(const char *str);

//void sd_card_display_files(LCD_I2C_HandleTypeDef* hlcd);

#endif /* INC_SD_CARD_READER_H_ */
