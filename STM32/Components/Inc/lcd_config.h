/**
  ******************************************************************************
  * @file    lcd_config.h
  * @author  Konrad Marchewka
  * @author   : AW    Adrian.Wojcik@put.poznan.pl
  * @version V1.0
  * @date    23-Jan-2024
  * @brief   Simple HD44780 driver library for STM32F7 configuration file.
  *
  ******************************************************************************
  */
#ifndef INC_LCD_CONFIG_H_
#define INC_LCD_CONFIG_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "LCD.h"
#include "main.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"
#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_hal_i2c.h"
#include <string.h>
#include <stdio.h>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

extern LCD_I2C_HandleTypeDef hlcd3;

/* Public function prototypes ------------------------------------------------*/

#endif /* INC_LCD_CONFIG_H_ */
