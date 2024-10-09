/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"

#define REVERS     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_SET);
#define FORVARD    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET);

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
#define Counter_2_Pin GPIO_PIN_0
#define Counter_2_GPIO_Port GPIOA
#define INP_1_Pin GPIO_PIN_3
#define INP_1_GPIO_Port GPIOA
#define INP_2_Pin GPIO_PIN_4
#define INP_2_GPIO_Port GPIOA
#define SPEED_Pin GPIO_PIN_5
#define SPEED_GPIO_Port GPIOA
#define Pedal_Pin GPIO_PIN_6
#define Pedal_GPIO_Port GPIOA
#define BUZZ_Pin GPIO_PIN_14
#define BUZZ_GPIO_Port GPIOB
#define DIRECTION_2_Pin GPIO_PIN_15
#define DIRECTION_2_GPIO_Port GPIOB
#define STOP_2_Pin GPIO_PIN_11
#define STOP_2_GPIO_Port GPIOA
#define STOP_DRIVER_Pin GPIO_PIN_12
#define STOP_DRIVER_GPIO_Port GPIOA
#define DIRECTION_Pin GPIO_PIN_15
#define DIRECTION_GPIO_Port GPIOA
#define DIRECT_BUTTON_Pin GPIO_PIN_3
#define DIRECT_BUTTON_GPIO_Port GPIOB
#define MODE_BUT_Pin GPIO_PIN_4
#define MODE_BUT_GPIO_Port GPIOB
#define Counter_Pin GPIO_PIN_5
#define Counter_GPIO_Port GPIOB
#define Counter_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
