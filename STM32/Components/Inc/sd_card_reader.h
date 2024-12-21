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
#include "LCD.h"
#include "lcd_config.h"
// Maximum G-code line length
#define BUFFER_SIZE 128        // Buffer size for reading file chunks
#define LINE_BUFFER_SIZE 256
#define MAX_LINE_LENGTH 256
#define MAX_COORD_COUNT 500

extern float coordinates_x[MAX_COORD_COUNT];
extern float coordinates_y[MAX_COORD_COUNT];
extern float coordinates_z[MAX_COORD_COUNT];

// Function prototypes
void parse_gcode(const char *char_array);
void process_raw_gcode(FIL *fil);
void uart_transmit_string(const char *str);
void display_files_on_lcd(LCD_I2C_HandleTypeDef* hlcd);

#endif /* INC_SD_CARD_READER_H_ */
