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
#include <string.h>
#include <stdlib.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"
#include "main.h"
#include "i2c.h"


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
const uint8_t LCD_ROW_20[] = {0x00, 0x40, 0x14, 0x54};

uint8_t __lcd_i2c_buffer[6] = { 0x00, };

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
  __lcd_delay(hlcd->Timer, 15.2);  // >15 ms

  // 4-bit mode
  __lcd_i2c_write_command(hlcd, 0x03);  // 0011
  __lcd_delay(hlcd->Timer, 4.1);        // > 4.1 ms
  __lcd_i2c_write_command(hlcd, 0x03);  // 0011
  __lcd_delay(hlcd->Timer, 4.1);        // > 0.1 ms
  __lcd_i2c_write_command(hlcd, 0x03);  // 0011
  __lcd_i2c_write_command(hlcd, 0x02);  // 0001

  hlcd->IsInitialized = 1;

  __lcd_i2c_write_command(hlcd, LCD_FUNCTION_SET | LCD_OPT_N);

  __lcd_i2c_write_command(hlcd, LCD_CLEAR_DISPLAY);                      // Clear screen
  __lcd_delay(hlcd->Timer, 1.6);                                         // > 1.52 ms
  __lcd_i2c_write_command(hlcd, LCD_DISPLAY_ON_OFF_CONTROL | LCD_OPT_D); // LCD on, Cursor off, On blink
  __lcd_i2c_write_command(hlcd, LCD_ENTRY_MODE_SET | LCD_OPT_INC);       // Cursor increment on
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
void LCD_I2C_SetCursor(LCD_I2C_HandleTypeDef* hlcd, uint8_t row, uint8_t col)
{
  #ifdef LCD20xN
  __lcd_i2c_write_command(hlcd, LCD_SET_DDRAM_ADDR + LCD_ROW_20[row] + col);
  #endif

  #ifdef LCD16xN
  __lcd_i2c_write_command(hlcd, LCD_SET_DDRAM_ADDR + LCD_ROW_16[row] + col);
  #endif
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






#endif
