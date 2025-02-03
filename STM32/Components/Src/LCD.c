/**
  ******************************************************************************
  * @file     : lcd.c
  * @author  Konrad Marchewka
  * @author   : AW    Adrian.Wojcik@put.poznan.pl
  * @version V1.0
  * @date    23-Jan-2024
  * @brief    : Simple HD44780 driver library for STM32F7.
  *             NOTE!: This code provides only WRITE features, no READ features.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LCD.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#ifdef LCD_USE_TIMER
#endif
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/


/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/
#define LCD_NUMBER_BUF_SIZE 2
#define LCD_PRINTF_BUF_SIZE 64
#define MAX_MESSAGE_LENGTH 4

/* Macro ---------------------------------------------------------------------*/
#ifdef LCD_USE_TIMER
#define __lcd_delay(__HANDLE__, delay_ms)  __lcd_delay_us((__HANDLE__),(float)delay_ms * 1000.0)
#else
#define __lcd_delay(__HANDLE__, delay_ms)  HAL_Delay((uint32_t)delay_ms);
#endif


/* Private variables ---------------------------------------------------------*/
const uint8_t LCD_ROW_16[] = {0x00, 0x40, 0x10, 0x50};
const uint8_t LCD_ROW_20[] = {0x00, 0x40, 0x12, 0x54};

uint8_t __lcd_i2c_buffer[6] = { 0x00, };
uint8_t fileCount = 0;
char fileList[10][21] = {0}; // Buffer for up to 10 files (20 chars max + null terminator)

/* Public variables ----------------------------------------------------------*/

/* Private function ----------------------------------------------------------*/
#ifdef LCD_USE_TIMER
/**
 * @brief LCD delay function
 * @param[in] htim     : LCD timer handler
 * @param[in] delay_us : Delay period in microseconds
 * @return None
 */
void __lcd_delay_us(LCD_TimerType htim, uint16_t delay_us)
{
  __HAL_TIM_SET_COUNTER(htim, 0);
  HAL_TIM_Base_Start(htim);
  while(__HAL_TIM_GET_COUNTER(htim) < delay_us);
  HAL_TIM_Base_Stop(htim);
}
#endif

/**
 * @brief Write a byte to the data register
 * @param[in] hlcd : LCD handler with I2C interface
 * @param[in] data : Display data byte
 * @return None
 */
void __lcd_i2c_write(LCD_I2C_HandleTypeDef* hlcd, uint8_t rsRwBits, uint8_t data)
{
    /* most significant nibble */
    __lcd_i2c_buffer[0] = rsRwBits | LCD_I2C_BIT_E | LCD_I2C_BIT_BACKIGHT_ON | (data & 0xF0);
    __lcd_i2c_buffer[1] = __lcd_i2c_buffer[0];
    __lcd_i2c_buffer[2] = rsRwBits | LCD_I2C_BIT_BACKIGHT_ON | (data & 0xF0);

    /* least significant nibble */
    __lcd_i2c_buffer[3] = rsRwBits | LCD_I2C_BIT_E | LCD_I2C_BIT_BACKIGHT_ON | ((data << 4) & 0xF0);
    __lcd_i2c_buffer[4] = __lcd_i2c_buffer[3];
    __lcd_i2c_buffer[5] = rsRwBits | LCD_I2C_BIT_BACKIGHT_ON | ((data << 4) & 0xF0);

    HAL_I2C_Master_Transmit(hlcd->I2C, (hlcd->Address << 1), (uint8_t*)__lcd_i2c_buffer, 6, hlcd->Timeout);

    __lcd_delay(hlcd->Timer, 0.05);  // > 41 us
}

/**
 * @brief Write a byte to the command register
 * @param[in] hlcd    : LCD handler with I2C interface
 * @param[in] command : Display command @see lcd.h/Define
 * @return None
 */
void __lcd_i2c_write_command(LCD_I2C_HandleTypeDef* hlcd, uint8_t data)
{
  __lcd_i2c_write(hlcd, LCD_COMMAND_REG, data);
}

/**
 * @brief Write a byte to the data register
 * @param[in] hlcd : LCD handler with I2C interface
 * @param[in] data : Display data byte
 * @return None
 */
void __lcd_i2c_write_data(LCD_I2C_HandleTypeDef* hlcd, uint8_t data)
{
  __lcd_i2c_write(hlcd, LCD_DATA_REG, data);
}


/**
 * @brief LCD initialization procedure.
 * @note Cursor off, Cursor increment on, No blink @see HD44780 technical note.
 * @param[in] hlcd : LCD handler with I2C interface
 * @return None
 */
void LCD_I2C_Init(LCD_I2C_HandleTypeDef* hlcd)
{
    // Step 1: Initial delay after power-on
    __lcd_delay(hlcd->Timer, 50); // Wait at least 40 ms after power-on

    // Step 2: Force LCD into 4-bit mode
    __lcd_i2c_write_command(hlcd, 0x03);  // Function set: 8-bit mode
    __lcd_delay(hlcd->Timer, 5);          // Wait > 4.1 ms

    __lcd_i2c_write_command(hlcd, 0x03);  // Function set: 8-bit mode
    __lcd_delay(hlcd->Timer, 1);          // Wait > 100 µs

    __lcd_i2c_write_command(hlcd, 0x03);  // Function set: 8-bit mode
    __lcd_delay(hlcd->Timer, 1);          // Short delay

    __lcd_i2c_write_command(hlcd, 0x02);  // Function set: 4-bit mode
    __lcd_delay(hlcd->Timer, 1);          // Short delay

    // Step 3: Configure display
    __lcd_i2c_write_command(hlcd, LCD_FUNCTION_SET | LCD_OPT_N);        // Function set: 4-bit, 2-line, 5x8 dots
    __lcd_i2c_write_command(hlcd, LCD_DISPLAY_ON_OFF_CONTROL | LCD_OPT_D); // Display on, cursor off, blink off
    __lcd_i2c_write_command(hlcd, LCD_CLEAR_DISPLAY);                   // Clear display
    __lcd_delay(hlcd->Timer, 2);                                        // Wait > 1.52 ms
    __lcd_i2c_write_command(hlcd, LCD_ENTRY_MODE_SET | LCD_OPT_INC);    // Entry mode: increment cursor, no shift

    // Mark as initialized
    hlcd->IsInitialized = 1;
}
/**
 * @brief Write custom character
 * @param[in] hlcd   : LCD custom character handler with I2C interface
 * @return None
 */


void LCD_I2C_printCustomChar(LCD_I2C_HandleTypeDef* hlcd, uint8_t code) {
  // Assuming __lcd_i2c_write_data allows sending custom character code directly
  __lcd_i2c_write_data(hlcd, code);
}

/**
 * @brief Write a decimal number on the current position.
 * @param[in] hlcd   : LCD handler with I2C interface
 * @param[in] number : Decimal number
 * @return None
 */
void LCD_I2C_printDecInt(LCD_I2C_HandleTypeDef* hlcd, int number)
{
  char buffer[LCD_NUMBER_BUF_SIZE];
  if( number >= 10)
      sprintf(buffer, "%d", number);
  else
	  sprintf(buffer, "0%d", number);

  LCD_I2C_printStr(hlcd, buffer);
}

/**
 * @brief Write a hexadecimal number on the current position.
 * @param[in] hlcd   : LCD handler with I2C interface
 * @param[in] number : Hexadecimal number
 * @return None
 */
void LCD_I2C_printHexInt(LCD_I2C_HandleTypeDef* hlcd, int number)
{
  char buffer[LCD_NUMBER_BUF_SIZE];
  sprintf(buffer, "%x", number);

  LCD_I2C_printStr(hlcd, buffer);
}

/**
 * @brief Write a string on the current position.
 * @param[in] hlcd : LCD handler with I2C interface
 * @param[in] str  : Null-terminated string
 * @return None
 */
void LCD_I2C_printStr(LCD_I2C_HandleTypeDef* hlcd, char* str)
{
  for(uint8_t i = 0; i < strlen(str); i++)
    __lcd_i2c_write_data(hlcd, str[i]);
}

/**
 * @brief Set the cursor position.
 * @param[in] hlcd : LCD handler with I2C interface
 * @param[in] row  : Display row (line): 0 to N
 * @param[in] col  : Display column: 0 to 15 (16 character display) or 19 (20 character display)
 * @return None
 */
void LCD_I2C_Cursor(LCD_I2C_HandleTypeDef* hlcd, uint8_t row, uint8_t col)
{
  #ifdef LCD20xN
  __lcd_i2c_write_command(hlcd, LCD_SET_DDRAM_ADDR + LCD_ROW_20[row] + col);
  #endif

  #ifdef LCD16xN
  __lcd_i2c_write_command(hlcd, LCD_SET_DDRAM_ADDR + LCD_ROW_16[row] + col);
  #endif
}
void LCD_I2C_SetCursor(LCD_I2C_HandleTypeDef* hlcd, uint8_t row, uint8_t col){
	if(row >=2){
		LCD_I2C_Cursor(hlcd,row-2,col+20);
}
	else{
		LCD_I2C_Cursor(hlcd,row,col);
	}
}
/**
 * @brief Clear the screen.
 * @param[in] hlcd : LCD handler with I2C interface
 * @return None
 */
void LCD_I2C_Clear(LCD_I2C_HandleTypeDef * hlcd)
{
  __lcd_i2c_write_command(hlcd, LCD_CLEAR_DISPLAY);
}

/**
 * @brief Write new character to display memory.
 * @param[in] hlcd   : LCD handler with I2C interface
 * @param[in] code   : Defined character code in display memory @see HD44780 technical note.
 * @param[in] bitmap : Defined character array @see HD44780 technical note.
 * @return None
 */
void LCD_I2C_DefineChar(LCD_I2C_HandleTypeDef* hlcd, uint8_t code, uint8_t bitmap[])
{
  __lcd_i2c_write_command(hlcd, LCD_SETCGRAM_ADDR + (code << 3));

  for(uint8_t i=0; i < 8; ++i)
    __lcd_i2c_write_data(hlcd, bitmap[i]);
}


#ifdef LCD_PRINTF_ENABLE
/**
 * @brief Write text in standard format on the current position.
 * @param[in] hlcd   : LCD handler with I2C interface
 * @param[in] format : Text format @see http://www.cplusplus.com/reference/cstdio/printf/
 * @param[in] ...    : Variadic arguments
 * @return None
 */
void LCD_I2C_printf(LCD_I2C_HandleTypeDef* hlcd, const char* format, ...)
{
  char buffer[LCD_PRINTF_BUF_SIZE];
  va_list args;
  va_start(args, format);
  vsprintf(buffer,format, args);
  LCD_I2C_printStr(hlcd, buffer);
  va_end(args);
}




/**
 * @brief Display "BONGO BONG" with a glossy animation.
 * @param[in] hlcd : LCD handler with I2C interface
 * @param[in] row  : Row number to display the text
 * @return None
 */
void LCD_I2C_DisplaySequentialGlossyText(LCD_I2C_HandleTypeDef* hlcd, uint8_t row)
{
    // Ensure row is valid (0 or 1 for a 2x16 LCD)


    const char* text = "BONGO BONG";
    uint8_t len = strlen(text);

    // Create a highlight custom character
    uint8_t highlight_char[8] = {
        0b11111,  // Full block
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111
    };

    // Define the custom character in CGRAM
    LCD_I2C_DefineChar(hlcd, 0, highlight_char);

    // Start from the first character
    for (uint8_t i = 0; i < len; i++) {
        // Highlight the current character
        LCD_I2C_SetCursor(hlcd, row, i+3);
        LCD_I2C_printCustomChar(hlcd, 0);

        // Wait to create the glossy effect
        __lcd_delay(hlcd->Timer, 200); // Adjust delay for visual preference

        // Replace the highlighted character with the original character
        LCD_I2C_SetCursor(hlcd, row, i+3);
        __lcd_i2c_write_data(hlcd, text[i]);
    }
}

void LCD_I2C_HandleMenuSelection(uint8_t selectedOption, LCD_I2C_HandleTypeDef* hlcd, ENC_Handle_TypeDef* henc) {
    // Clear the screen before rendering the option
    LCD_I2C_Clear(hlcd);
    // Sub menus under "Test from SD Card"
    const char* calibMenuItems[] = {"Auto Calibartion", "Semi-Auto Calibration", "Manual Calibration" }; // Calibration Menu


    switch (selectedOption) {
        case 1:
            // Logic for "Test from SD Card"
            while (1) {
                LCD_I2C_DisplaySDMenu(hlcd, henc); // Load Files and Display them...
                // Files is selected so we move with calibration
                LCD_I2C_menuTemplate(hlcd, henc, calibMenuItems, 1); // Load Calibration Menu
            }

                HAL_Delay(100);
                break;

              // **Break to avoid falling through**

        case 2:
            // Logic for "Prepare the Machine"
            LCD_I2C_SetCursor(hlcd, 0, 0);
            LCD_I2C_printStr(hlcd, "                    "); // Clear line (20 spaces)
            LCD_I2C_SetCursor(hlcd, 0, 0);
            LCD_I2C_printStr(hlcd, "Preparing...");
            HAL_Delay(2000); // Simulate machine preparation logic
            break;

        default:
            // Handle invalid cases (should not occur in normal use)
            LCD_I2C_SetCursor(hlcd, 0, 0);
            LCD_I2C_printStr(hlcd, "Invalid Option");
            HAL_Delay(2000);
            break;
    }
}



uint8_t LCD_I2C_menuTemplate(LCD_I2C_HandleTypeDef* hlcd, ENC_Handle_TypeDef* henc, const char* displayItems[][21], bool backOption){
    LCD_I2C_Clear(hlcd);
    uint8_t totalOptions = sizeof(displayItems) / sizeof(displayItems[0]);
    totalOptions = backOption ? totalOptions + 1 : totalOptions;
    const char* menuItems[totalOptions][21];
    if (backOption){
        menuItems[0][1] = "Back";
        for (uint8_t i = 0; i < totalOptions; i++){
            menuItems[i+1][21] = displayItems[i];
        }
    }
    else{
        for (uint8_t i = 0; i< totalOptions; i++){
            menuItems[i][21] = displayItems[i];
        }
    }
    uint8_t selectedOption = 0; // Current selected menu item
    uint8_t previousOption = 255; // Invalid to ensure the first update
    AddBackOption(menuItems, totalOptions);
    
    while (1) {
        // Get the current encoder step count using your ENC_GetCounter function
        uint32_t encoderStep = ENC_GetCounter(henc);

        // Normalize encoder steps to menu options
        selectedOption = encoderStep % totalOptions;

        // Handle wrap-around (if encoderStep can be negative)
        if (selectedOption < 0) {
            selectedOption += totalOptions;
        }

        // Update the menu display only if the selection has changed
        if (selectedOption != previousOption) {
            previousOption = selectedOption;

            for(int i =0 ; i <4 ; i++){
                LCD_I2C_SetCursor(hlcd, i, 0);
                LCD_I2C_printStr(hlcd, "                    "); // Clear line (20 spaces)
            }
            // Update menu display
            for (uint8_t i = 0; i < totalOptions; i++) {
                // Clear the line and set cursor position

                LCD_I2C_SetCursor(hlcd, i, 0);

                // Highlight the selected item
                if (i == selectedOption) {
                    char formattedLine[21];
                    snprintf(formattedLine, 21, ">%-19s", menuItems[i]);
                    LCD_I2C_printStr(hlcd, formattedLine);
                } else {
                    char formattedLine[21];
                    snprintf(formattedLine, 21, " %-19s", menuItems[i]);
                    LCD_I2C_printStr(hlcd, formattedLine);
                }
            }
        }

        // Check for selection button
        bool buttonInput = read_buttons();
        if(buttonInput==0) { 
            osDelay(200);
            if(selectedOption == 0 && backOption == 1){
                return;
            }
            else{
                return selectedOption + 1;
            }
        }

        // Add a delay for smoother updates
        osDelay(100);
    }
}


uint8_t LCD_I2C_MainMenu(LCD_I2C_HandleTypeDef* hlcd, ENC_Handle_TypeDef* henc) {
    const char* menuItems[] = {"Test from SD", "Prepare Machine"}; // Menu options
     return LCD_I2C_menuTemplate(hlcd, henc, menuItems, 0);
}



bool read_buttons(void)
{
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET) return 1; // Down
    else return 0;

}


FRESULT MountSDCard(FATFS* FatFs) {
    return f_mount(FatFs, "", 1);
}

FRESULT OpenRootDir(DIR* dir) {
    return f_opendir(dir, "/");
}

uint8_t ReadFiles(DIR* dir, char fileList[][20 + 1], uint8_t maxFiles) {
    FILINFO fno;
    uint8_t count = 0;
    FRESULT fres;

    while (count < maxFiles) {
        fres = f_readdir(dir, &fno);
        if (fres != FR_OK || fno.fname[0] == '\0') break;
        if (!(fno.fattrib & AM_DIR)) {
            snprintf(fileList[count], 20 + 1, "%-20s", fno.fname);
            count++;
        }
    }

    return count;
}

void AddBackOption(char menuItems[][20 + 1], uint8_t index) {
    snprintf(menuItems[index], 20 + 1, "%-20s", "Back");
}

void DisplayMenu(LCD_I2C_HandleTypeDef* hlcd, char fileList[][20 + 1], uint8_t fileCount, uint8_t selectedIndex) {
    // Clear the LCD
    for (uint8_t i = 0; i < LCD_ROWS; i++) {
        LCD_I2C_SetCursor(hlcd, i, 0);
        LCD_I2C_printStr(hlcd, "                    "); // Clear line
    }

    // Determine the window of files to display
    uint8_t startIdx = 0;
    if (selectedIndex >= LCD_ROWS) {
        startIdx = selectedIndex - (LCD_ROWS - 1);
    }

    for (uint8_t i = 0; i < LCD_ROWS; i++) {
        uint8_t displayIdx = startIdx + i;
        if (displayIdx >= fileCount) {
            // Clear remaining lines if any
            LCD_I2C_SetCursor(hlcd, i, 0);
            LCD_I2C_printStr(hlcd, "                    ");
            continue;
        }

        // Prepare the formatted line with cursor
        char formattedLine[20 + 1] = {0};
        if (displayIdx == selectedIndex) {
            snprintf(formattedLine, sizeof(formattedLine), "> %-19s", fileList[displayIdx]);
        } else {
            snprintf(formattedLine, sizeof(formattedLine), "  %-19s", fileList[displayIdx]);
        }

        // Write to the LCD
        LCD_I2C_SetCursor(hlcd, i, 0);
        LCD_I2C_printStr(hlcd, formattedLine);
    }
}

void LCD_I2C_DisplaySDMenu(LCD_I2C_HandleTypeDef* hlcd, ENC_Handle_TypeDef* henc) {
    FATFS FatFs;
    FRESULT fres;
    DIR dir;
    char fileList[MAX_FILES + 1][20 + 1]; // +1 for "Back"
    uint8_t fileCount = 0;

    fres = MountSDCard(&FatFs);
    if (fres != FR_OK) {
        LCD_I2C_SetCursor(hlcd, 0, 0);
        LCD_I2C_printStr(hlcd, "Failed to mount");
        HAL_Delay(2000);
        return;
    }

    fres = OpenRootDir(&dir);
    if (fres != FR_OK) {
        LCD_I2C_SetCursor(hlcd, 0, 0);
        LCD_I2C_printStr(hlcd, "Open dir failed");
        f_mount(NULL, "", 0);
        HAL_Delay(2000);
        return;
    }

    fileCount = ReadFiles(&dir, fileList, MAX_FILES);
    f_closedir(&dir);
    f_mount(NULL, "", 0);

    // Add "Back" option
    AddBackOption(fileList, fileCount);
    fileCount++;

    if (fileCount == 1) { // Only "Back" is available
        LCD_I2C_SetCursor(hlcd, 0, 0);
        LCD_I2C_printStr(hlcd, "No files found");
        HAL_Delay(2000);
        return;
    }

    uint8_t selectedIndex = 0;
    uint8_t previousIndex = 255;

    while (1) {
        uint32_t encoderStep = ENC_GetCounter(henc);
        selectedIndex = encoderStep % fileCount;

        if (selectedIndex != previousIndex) {
            previousIndex = selectedIndex;
            DisplayMenu(hlcd, fileList, fileCount, selectedIndex);
        }

        // Handle button press for selection
        if (read_buttons() == 0) {
            HAL_Delay(200); // Debounce
            // Clear the LCD
            for (uint8_t i = 0; i < LCD_ROWS; i++) {
                LCD_I2C_SetCursor(hlcd, i, 0);
                LCD_I2C_printStr(hlcd, "                    "); // Clear line
            }

            if (selectedIndex == fileCount - 1) {
                // "Back" selected
                return; // Exit the menu to go back to the previous menu
            } else {
                // "Loading..." selected
                LCD_I2C_SetCursor(hlcd, 0, 0);
                LCD_I2C_printStr(hlcd, "Loading...");
                HAL_Delay(2000);
                // process_file(fileList[selectedIndex]);
                // Add your file processing logic here
                return;
            }
        }

        HAL_Delay(100);
    }
}



#endif
