/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define OLEDSCL_Pin GPIO_PIN_8
#define OLEDSCL_GPIO_Port GPIOB
#define OLEDSDA_Pin GPIO_PIN_9
#define OLEDSDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/*USART_GPIO*/
#define RS485TX_GPIO_Port           GPIOB
#define RS485TX_PIN                 GPIO_PIN_10
	                                  
#define RS485RX_GPIO_Port           GPIOB
#define RS485RX_PIN                 GPIO_PIN_11
	                                  
#define RS232TX_GPIO_Port           GPIOA
#define RS232TX_PIN                 GPIO_PIN_9
	                                  
#define RS232RX_GPIO_Port           GPIOA
#define RS232RX_PIN                 GPIO_PIN_10

/*Input_GPIO*/
#define GROUND_SWITCH_GPIO_Port     GPIOB
#define GROUND_SWITCH_PIN           GPIO_PIN_3
                                              
#define HAND_BOX_DET_GPIO_Port      GPIOA
#define HAND_BOX_DET_PIN            GPIO_PIN_8

/*Output_GPIO*/
#define LED_GPIO_Port               GPIOC
#define LED_PIN                     GPIO_PIN_13 

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
