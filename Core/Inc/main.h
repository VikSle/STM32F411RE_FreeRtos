/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mainHY32D.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define D03_Pin GPIO_PIN_3
#define D03_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define D04_Pin GPIO_PIN_4
#define D04_GPIO_Port GPIOC
#define D00_Pin GPIO_PIN_0
#define D00_GPIO_Port GPIOB
#define D01_Pin GPIO_PIN_1
#define D01_GPIO_Port GPIOB
#define D02_Pin GPIO_PIN_2
#define D02_GPIO_Port GPIOB
#define D10_Pin GPIO_PIN_10
#define D10_GPIO_Port GPIOB
#define D12_Pin GPIO_PIN_12
#define D12_GPIO_Port GPIOB
#define D13_Pin GPIO_PIN_13
#define D13_GPIO_Port GPIOB
#define D14_Pin GPIO_PIN_14
#define D14_GPIO_Port GPIOB
#define D15_Pin GPIO_PIN_15
#define D15_GPIO_Port GPIOB
#define RD_Pin GPIO_PIN_6
#define RD_GPIO_Port GPIOC
#define WR_Pin GPIO_PIN_7
#define WR_GPIO_Port GPIOC
#define RS_Pin GPIO_PIN_8
#define RS_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_9
#define CS_GPIO_Port GPIOA
#define D11_Pin GPIO_PIN_11
#define D11_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define D05_Pin GPIO_PIN_5
#define D05_GPIO_Port GPIOB
#define D06_Pin GPIO_PIN_6
#define D06_GPIO_Port GPIOB
#define D07_Pin GPIO_PIN_7
#define D07_GPIO_Port GPIOB
#define D08_Pin GPIO_PIN_8
#define D08_GPIO_Port GPIOB
#define D09_Pin GPIO_PIN_9
#define D09_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
