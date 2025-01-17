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
#define EndStop1_Pin GPIO_PIN_2
#define EndStop1_GPIO_Port GPIOE
#define enn3_Pin GPIO_PIN_3
#define enn3_GPIO_Port GPIOE
#define EndStop2_Pin GPIO_PIN_4
#define EndStop2_GPIO_Port GPIOE
#define step3_Pin GPIO_PIN_5
#define step3_GPIO_Port GPIOE
#define dir3_Pin GPIO_PIN_6
#define dir3_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define USER_Btn_EXTI_IRQn EXTI15_10_IRQn
#define enn4_Pin GPIO_PIN_0
#define enn4_GPIO_Port GPIOF
#define dir1_Pin GPIO_PIN_7
#define dir1_GPIO_Port GPIOF
#define Servo1PWM_Pin GPIO_PIN_9
#define Servo1PWM_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define servoPWM_Pin GPIO_PIN_0
#define servoPWM_GPIO_Port GPIOA
#define Probe_Pin GPIO_PIN_3
#define Probe_GPIO_Port GPIOA
#define EndStop3_Pin GPIO_PIN_4
#define EndStop3_GPIO_Port GPIOA
#define enn2_Pin GPIO_PIN_5
#define enn2_GPIO_Port GPIOA
#define step2_Pin GPIO_PIN_6
#define step2_GPIO_Port GPIOA
#define dir2_Pin GPIO_PIN_7
#define dir2_GPIO_Port GPIOA
#define EncoderBtn_Pin GPIO_PIN_1
#define EncoderBtn_GPIO_Port GPIOB
#define BtnUp_Pin GPIO_PIN_14
#define BtnUp_GPIO_Port GPIOF
#define BtnDown_Pin GPIO_PIN_15
#define BtnDown_GPIO_Port GPIOF
#define BtnLeft_Pin GPIO_PIN_0
#define BtnLeft_GPIO_Port GPIOG
#define BtnRight_Pin GPIO_PIN_1
#define BtnRight_GPIO_Port GPIOG
#define BtnCtr_Pin GPIO_PIN_11
#define BtnCtr_GPIO_Port GPIOE
#define step1_Pin GPIO_PIN_10
#define step1_GPIO_Port GPIOB
#define enn1_Pin GPIO_PIN_11
#define enn1_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define ENC_DT_Pin GPIO_PIN_12
#define ENC_DT_GPIO_Port GPIOD
#define ENC_CLK_Pin GPIO_PIN_13
#define ENC_CLK_GPIO_Port GPIOD
#define SPI_cs_Pin GPIO_PIN_14
#define SPI_cs_GPIO_Port GPIOD
#define diag_Pin GPIO_PIN_15
#define diag_GPIO_Port GPIOD
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
#define dir4_Pin GPIO_PIN_0
#define dir4_GPIO_Port GPIOD
#define diag1_Pin GPIO_PIN_1
#define diag1_GPIO_Port GPIOD
#define EndStop4_Pin GPIO_PIN_3
#define EndStop4_GPIO_Port GPIOB
#define step4_Pin GPIO_PIN_8
#define step4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
