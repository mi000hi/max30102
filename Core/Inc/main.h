/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

#include "hci_tl_interface.h"
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
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOG
#define BLUE_RST_Pin GPIO_PIN_0
#define BLUE_RST_GPIO_Port GPIOH
#define USER_BUTTON_Pin GPIO_PIN_2
#define USER_BUTTON_GPIO_Port GPIOC
#define SPI_DATA_Pin GPIO_PIN_15
#define SPI_DATA_GPIO_Port GPIOB
#define CS_AG_Pin GPIO_PIN_12
#define CS_AG_GPIO_Port GPIOB
#define SPI_SCK_Pin GPIO_PIN_13
#define SPI_SCK_GPIO_Port GPIOB
#define BLUE_MOSI_Pin GPIO_PIN_7
#define BLUE_MOSI_GPIO_Port GPIOA
#define BLUE_MISO_Pin GPIO_PIN_6
#define BLUE_MISO_GPIO_Port GPIOA
#define BLUE_SCK_Pin GPIO_PIN_5
#define BLUE_SCK_GPIO_Port GPIOA
#define CS_P_Pin GPIO_PIN_3
#define CS_P_GPIO_Port GPIOA
#define BLUE_CS_Pin GPIO_PIN_2
#define BLUE_CS_GPIO_Port GPIOB
#define CS_M_Pin GPIO_PIN_1
#define CS_M_GPIO_Port GPIOB
#define BLUE_IRQ_Pin GPIO_PIN_5
#define BLUE_IRQ_GPIO_Port GPIOC
#define BLUE_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define CS_A_Pin GPIO_PIN_4
#define CS_A_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
