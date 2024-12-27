#include "sd_card_reader.h"


#define MAX_LINE_LENGTH 256
#define MAX_COORD_COUNT 500
#define BUFFER_SIZE 128
#define LINE_BUFFER_SIZE 256

float coordinates_x[MAX_COORD_COUNT];
float coordinates_y[MAX_COORD_COUNT];
float coordinates_z[MAX_COORD_COUNT];
int coordinate_index = 0;

extern SPI_HandleTypeDef hspi1;
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

/* List all files in the root directory of the SD card */
//void sd_card_display_files(LCD_I2C_HandleTypeDef* hlcd) {
//    FATFS FatFs;
//    FRESULT fres;
//    DIR dir;
//    FILINFO fno;
//    char fileList[20][20]; // Assuming up to 20 files and max name length of 20 characters
//    uint8_t fileCount = 0;
//    uint8_t startIndex = 0; // Index of the first file to display
//    uint8_t LCD_ROWS = 2;  // Adjust for your LCD size
//    uint8_t LCD_WIDTH = 20; // Adjust for your LCD width
//    int buttonInput = read_buttons();
//
//    fres = f_mount(&FatFs, "", 1);
//    if (fres != FR_OK) {
//        LCD_I2C_Clear(hlcd);
//        LCD_I2C_SetCursor(hlcd, 0, 0);
//        LCD_I2C_printStr(hlcd, "Failed to mount");
//        HAL_Delay(2000);
//        return;
//    }
//
//    // Open the root directory
//    fres = f_opendir(&dir, "/");
//    if (fres != FR_OK) {
//        LCD_I2C_Clear(hlcd);
//        LCD_I2C_SetCursor(hlcd, 0, 0);
//        LCD_I2C_printStr(hlcd, "Open dir failed");
//        f_mount(NULL, "", 0); // Unmount before exiting
//        HAL_Delay(2000);
//        return;
//    }
//
//    // Read all files in the root directory
//    while (1) {
//        fres = f_readdir(&dir, &fno); // Read a directory item
//        if (fres != FR_OK || fno.fname[0] == '\0') break; // Break on error or end of dir
//        if (!(fno.fattrib & AM_DIR)) { // Add only files (ignore directories)
//            strncpy(fileList[fileCount], fno.fname, LCD_WIDTH - 1);
//            fileList[fileCount][LCD_WIDTH - 1] = '\0'; // Null-terminate
//            fileCount++;
//            if (fileCount >= 20) break; // Avoid overflow
//        }
//    }
//
//    f_closedir(&dir);
//    f_mount(NULL, "", 0); // Unmount the SD card
//
//    if (fileCount == 0) {
//        LCD_I2C_Clear(hlcd);
//        LCD_I2C_SetCursor(hlcd, 0, 0);
//        LCD_I2C_printStr(hlcd, "No files found");
//        HAL_Delay(2000);
//        return;
//    }
//
//    // Display files with scrolling logic
//    while (1) {
//        LCD_I2C_Clear(hlcd);
//
//        // Display files within the visible range
//        for (uint8_t i = 0; i < LCD_ROWS; i++) {
//            if (startIndex + i < fileCount) {
//                LCD_I2C_SetCursor(hlcd, i, 0);
//                LCD_I2C_printStr(hlcd, fileList[startIndex + i]);
//            }
//        }
//
//        // Wait for button input for scrolling
//        buttonInput = buttons();
//
//        switch (buttonInput) {
//            case 1: // "Up" button
//                if (startIndex > 0) {
//                    startIndex--;
//                }
//                break;
//
//            case 2: // "Down" button
//                if (startIndex < fileCount - LCD_ROWS) {
//                    startIndex++;
//                }
//                break;
//
//            case 3: // "Back" button or Exit
//                return;
//
//            default:
//                // No valid input, continue looping
//                break;
//        }
//
//        HAL_Delay(200); // Debounce delay
//    }
//}


/* Parse a single G-code line for X, Y, and Z coordinates */
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

    if (coordinate_index < MAX_COORD_COUNT) {
        if (x_found) coordinates_x[coordinate_index] = x;
        if (y_found) coordinates_y[coordinate_index] = y;
        if (z_found) coordinates_z[coordinate_index] = z;
        coordinate_index++;
    } else {
        myprintf("Coordinate buffer full, cannot store more data!\r\n");
    }
}

/* Process the G-code file and display lines */
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
                    myprintf("Line: %s\r\n", line); // Display the line
                    parse_gcode(line);
                    line_index = 0;
                }
            } else {
                if (line_index < LINE_BUFFER_SIZE - 1) {
                    line[line_index++] = c;
                } else {
                    myprintf("Line buffer overflow, skipping line.\r\n");
                    line_index = 0; // Reset for safety
                }
            }
        }
    }

    if (line_index > 0) {
        line[line_index] = '\0';
        myprintf("Line: %s\r\n", line); // Display the line
        parse_gcode(line);
    }
}

/* Open and process the G-code file */
void sd_card_read_gcode(void) {
    myprintf("\r\n~ SD card G-code processing ~\r\n");

    // List all available files
    myprintf("Listing files on SD card:\r\n");
    //sd_card_list_files();

    // Mount the SD card
    FATFS FatFs;
    FIL fil;
    FRESULT fres;

    myprintf("Mounting SD card...\r\n");
    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK) {
        myprintf("Failed to mount SD card (Error: %i)\r\n", fres);
        return;
    }

    // Open a G-code file
    const char *filename = "yazidstink.gcode";
    myprintf("Attempting to open file '%s'\r\n", filename);

    fres = f_open(&fil, filename, FA_READ);
    if (fres != FR_OK) {
        myprintf("Failed to open file '%s' (Error: %i)\r\n", filename, fres);
        f_mount(NULL, "", 0);
        return;
    }

    // Process the G-code file
    myprintf("Processing '%s'...\r\n", filename);
    process_raw_gcode(&fil);

    // Close the file and unmount the SD card
    f_close(&fil);
    f_mount(NULL, "", 0);

    myprintf("File '%s' processing complete and SD card unmounted.\r\n", filename);
}
