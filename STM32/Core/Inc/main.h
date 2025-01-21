/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define SD_SPI_HANDLE hspi2


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIR3_Pin GPIO_PIN_2
#define DIR3_GPIO_Port GPIOE
#define Servo2PWM_Pin GPIO_PIN_5
#define Servo2PWM_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define USER_Btn_EXTI_IRQn EXTI15_10_IRQn
#define BtnDown_Pin GPIO_PIN_0
#define BtnDown_GPIO_Port GPIOF
#define BtnUp_Pin GPIO_PIN_1
#define BtnUp_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define STEP3_Pin GPIO_PIN_0
#define STEP3_GPIO_Port GPIOA
#define Servo1PWM_Pin GPIO_PIN_7
#define Servo1PWM_GPIO_Port GPIOA
#define STEP4_Pin GPIO_PIN_0
#define STEP4_GPIO_Port GPIOB
#define EndStop1_Pin GPIO_PIN_13
#define EndStop1_GPIO_Port GPIOF
#define EndStop4_Pin GPIO_PIN_14
#define EndStop4_GPIO_Port GPIOF
#define BtnDownF15_Pin GPIO_PIN_15
#define BtnDownF15_GPIO_Port GPIOF
#define BtnLeft_Pin GPIO_PIN_0
#define BtnLeft_GPIO_Port GPIOG
#define ENN3_Pin GPIO_PIN_7
#define ENN3_GPIO_Port GPIOE
#define ENN4_Pin GPIO_PIN_8
#define ENN4_GPIO_Port GPIOE
#define EndStop2_Pin GPIO_PIN_9
#define EndStop2_GPIO_Port GPIOE
#define ENN2_Pin GPIO_PIN_10
#define ENN2_GPIO_Port GPIOE
#define EndStop3_Pin GPIO_PIN_11
#define EndStop3_GPIO_Port GPIOE
#define DIR2_Pin GPIO_PIN_12
#define DIR2_GPIO_Port GPIOE
#define Probe_Pin GPIO_PIN_13
#define Probe_GPIO_Port GPIOE
#define STEP2_Pin GPIO_PIN_14
#define STEP2_GPIO_Port GPIOE
#define DIR1_Pin GPIO_PIN_15
#define DIR1_GPIO_Port GPIOE
#define STEP1_Pin GPIO_PIN_10
#define STEP1_GPIO_Port GPIOB
#define ENN1_Pin GPIO_PIN_11
#define ENN1_GPIO_Port GPIOB
#define SPI_cs_Pin GPIO_PIN_12
#define SPI_cs_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define EncoderBtn_Pin GPIO_PIN_11
#define EncoderBtn_GPIO_Port GPIOD
#define ENC_DT_Pin GPIO_PIN_12
#define ENC_DT_GPIO_Port GPIOD
#define ENC_CLK_Pin GPIO_PIN_13
#define ENC_CLK_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define BtnCtr_Pin GPIO_PIN_0
#define BtnCtr_GPIO_Port GPIOD
#define BtnRight_Pin GPIO_PIN_1
#define BtnRight_GPIO_Port GPIOD
#define LCD_SCL_Pin GPIO_PIN_8
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_9
#define LCD_SDA_GPIO_Port GPIOB
#define DIR4_Pin GPIO_PIN_0
#define DIR4_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
