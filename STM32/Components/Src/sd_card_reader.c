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
char lines[LINE_BUFFER_SIZE][MAX_LINE_LENGTH];

//// Transmit a string via UART
//void uart_transmit_string(const char *str) {
//    HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
//}
//
//// UART-based custom printf
//void myprintf(const char *fmt, ...) {
//    static char buffer[256];
//    va_list args;
//    va_start(args, fmt);
//    vsnprintf(buffer, sizeof(buffer), fmt, args);
//    va_end(args);
//
//    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//}
//



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


/**
 * @brief  Reads the file (given by its filename) from the SD card and
 *         stores each line into a 2D character array.
 * @param  hlcd: Pointer to the LCD handle (to display status/info).
 * @param  filename: The name of the file to process.
 * @note   This example mounts and unmounts the SD card within the function.
 *         Adjust error handling and processing as needed.
 */

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
    const char *filename = "gg.gcode";
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
