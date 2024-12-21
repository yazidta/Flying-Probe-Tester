#include "sd_card_reader.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h> // for va_list var arg functions
#include <stdbool.h> // for 'bool', 'true', and 'false'
#include "fatfs.h"

#define MAX_LINE_LENGTH 256
#define MAX_COORD_COUNT 500
#define BUFFER_SIZE 128
#define LINE_BUFFER_SIZE 256

float coordinates_x[MAX_COORD_COUNT];
float coordinates_y[MAX_COORD_COUNT];
float coordinates_z[MAX_COORD_COUNT];
int coordinate_index = 0;

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart3; // UART handler for sending debug/output

// Transmit a string via UART
void uart_transmit_string(const char *str) {
    HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

// UART-based custom printf
void myprintf(const char *fmt, ...) {
    static char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void parse_gcode(const char *line) {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    bool x_found = false, y_found = false, z_found = false;

    const char *ptr = line;
    while (*ptr != '\0') {
        if (*ptr == 'X') {
            ptr++;
            x = strtof(ptr, (char **) &ptr);
            x_found = true;
        } else if (*ptr == 'Y') {
            ptr++;
            y = strtof(ptr, (char **) &ptr);
            y_found = true;
        } else if (*ptr == 'Z') {
            ptr++;
            z = strtof(ptr, (char **) &ptr);
            z_found = true;
        } else {
            ptr++;  // Ignore other characters
        }
    }

    // Store the parsed coordinates if valid
    if (coordinate_index < MAX_COORD_COUNT) {
        if (x_found) coordinates_x[coordinate_index] = x;
        if (y_found) coordinates_y[coordinate_index] = y;
        if (z_found) coordinates_z[coordinate_index] = z;
        coordinate_index++;
    } else {
        myprintf("Coordinate buffer full, cannot store more data!\r\n");
    }
}

/* Process the G-code file */
void process_raw_gcode(FIL *fil) {
    char buffer[BUFFER_SIZE];
    UINT bytes_read = 0;
    char line[LINE_BUFFER_SIZE];
    int line_index = 0;

    while (f_read(fil, buffer, sizeof(buffer), &bytes_read) == FR_OK && bytes_read > 0) {
        for (UINT i = 0; i < bytes_read; i++) {
            char c = buffer[i];

            if (c == '\n' || c == '\r') {
                if (line_index > 0) {
                    line[line_index] = '\0';
                    parse_gcode(line);
                    line_index = 0;
                }
            } else {
                if (line_index < LINE_BUFFER_SIZE - 1) {
                    line[line_index++] = c;
                }
            }
        }
    }

    // Handle remaining line
    if (line_index > 0) {
        line[line_index] = '\0';
        parse_gcode(line);
    }
}

/* Open and process the G-code file */
void sd_card_read_gcode(void) {
    // Custom Message
    myprintf("\r\n~ SD card G-code processing ~\r\n");

    // Mounting the SD card
    FATFS FatFs;
    FIL fil;
    FRESULT fres;

    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK) {
        myprintf("Failed to mount SD card (Error: %i)\r\n", fres);
        while (1); // Stop if SD card mount fails
    }

    // Open the G-code file
    fres = f_open(&fil, "test.gco", FA_READ);
    if (fres != FR_OK) {
        myprintf("Failed to open file 'test.gco' (Error: %i)\r\n", fres);
        f_mount(NULL, "", 0); // Unmount before exiting
        while (1); // Stop if file open fails
    }

    // Process the G-code file
    myprintf("Processing 'test.gco'...\r\n");
    process_raw_gcode(&fil);

    // Close and unmount the SD card
    f_close(&fil);
    f_mount(NULL, "", 0);

    myprintf("File processing complete and SD card unmounted.\r\n");
}


// Function to display a list of files on the LCD
void display_files_on_lcd(LCD_I2C_HandleTypeDef* hlcd) {
    DIR dir;       // Directory object
    FILINFO fno;   // File information structure
    FRESULT res;

    uint8_t row = 0;   // LCD row counter
    char file_list[2][17];  // Buffer for two rows of LCD (assuming 16x2 display)

    // Mount the SD card
    FATFS FatFs;
    res = f_mount(&FatFs, "", 1);
    if (res != FR_OK) {
        LCD_I2C_Clear(hlcd);
        LCD_I2C_SetCursor(hlcd, 0, 0);
        LCD_I2C_printStr(hlcd, "SD Card Error");
        LCD_I2C_SetCursor(hlcd, 0, 0);
        LCD_I2C_SetCursor(hlcd, 1, 0);
        LCD_I2C_printDecInt(hlcd, res);


        return;
    }

    // Open the root directory
    res = f_opendir(&dir, "/");
    if (res != FR_OK) {
        LCD_I2C_Clear(hlcd);
        LCD_I2C_SetCursor(hlcd, 0, 0);
        LCD_I2C_printStr(hlcd, "Open Dir Err");
        f_mount(NULL, "", 0);
        return;
    }

    LCD_I2C_Clear(hlcd);

    while (1) {
        // Read a directory item
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break; // Break on error or end of dir

        // Store the file/directory name in the file_list
        strncpy(file_list[row], fno.fname, 16);
        file_list[row][16] = '\0'; // Null-terminate the string

        // Display on LCD
        LCD_I2C_SetCursor(hlcd, row, 0);
        LCD_I2C_printStr(hlcd, file_list[row]);

        row++;

        if (row > 1) { // For 16x2 LCD, display 2 lines at a time
            HAL_Delay(2000); // Wait for 2 seconds
            row = 0;         // Reset row counter
            LCD_I2C_Clear(hlcd);
        }
    }

    f_closedir(&dir);
    f_mount(NULL, "", 0); // Unmount the SD card
}
