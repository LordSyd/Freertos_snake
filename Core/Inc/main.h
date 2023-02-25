/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA
#define Button_A3_Pin GPIO_PIN_0
#define Button_A3_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Button_B_Pin GPIO_PIN_3
#define Button_B_GPIO_Port GPIOB
#define Button_B_EXTI_IRQn EXTI3_IRQn

/* USER CODE BEGIN Private defines */
#define LED1on(); HAL_GPIO_WritePin (GPIOA, GPIO_PIN_5,0);
#define LED1off(); HAL_GPIO_WritePin (GPIOA, GPIO_PIN_5,1);
#define LED1toggle(); HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_5);

#define LED2on(); HAL_GPIO_WritePin (GPIOA, GPIO_PIN_6,0);
#define LED2off(); HAL_GPIO_WritePin (GPIOA, GPIO_PIN_6,1);
#define LED2toggle(); HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_6);

#define LED3on(); HAL_GPIO_WritePin (GPIOA, GPIO_PIN_7,0);
#define LED3off(); HAL_GPIO_WritePin (GPIOA, GPIO_PIN_7,1);
#define LED3toggle(); HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_7);

#define LED4on(); HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6,0);
#define LED4off(); HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6,1);
#define LED4toggle(); HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_6);

#define Pin3Set(); HAL_GPIO_WritePin (GPIOC, GPIO_PIN_2, 1);
#define Pin3Reset(); HAL_GPIO_WritePin (GPIOC, GPIO_PIN_2, 0);

#define Pin2Set(); HAL_GPIO_WritePin (GPIOC, GPIO_PIN_1, 1);
#define Pin2Reset(); HAL_GPIO_WritePin (GPIOC, GPIO_PIN_1, 0);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
