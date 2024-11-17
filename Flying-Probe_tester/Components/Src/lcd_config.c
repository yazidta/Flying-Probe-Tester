/**
  ******************************************************************************
  * @file    lcd_config.c
  * @author  Konrad Marchewka
  * @author   : AW    Adrian.Wojcik@put.poznan.pl
  * @version V1.0
  * @date    23-Jan-2024
  * @brief   Simple HD44780 driver library for STM32F7 configuration file.
  *
  ******************************************************************************
  */
  /* Private includes ----------------------------------------------------------*/

#include "lcd.h"
#include "lcd_config.h"
#include "main.h"
#include "i2c.h"
#include "stm32f7xx_hal_tim.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#ifdef LCD_USE_TIMER
#endif
extern TIM_HandleTypeDef htim7;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
LCD_I2C_HandleTypeDef hlcd3 = {
    .I2C = &hi2c1,
    .Address = 0x27,  // PCF8574T (for all jumpers OPEN)
    .Timeout = 100,
    .Timer = &htim7
};

/* Private function prototypes -----------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/








