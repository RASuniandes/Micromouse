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
#define FL_Reciever_Pin GPIO_PIN_0
#define FL_Reciever_GPIO_Port GPIOC
#define FR_Reciever_Pin GPIO_PIN_1
#define FR_Reciever_GPIO_Port GPIOC
#define L_Reciever_Pin GPIO_PIN_2
#define L_Reciever_GPIO_Port GPIOC
#define R_Reciever_Pin GPIO_PIN_3
#define R_Reciever_GPIO_Port GPIOC
#define BOTON1_Pin GPIO_PIN_1
#define BOTON1_GPIO_Port GPIOA
#define R_Emitter_Pin GPIO_PIN_5
#define R_Emitter_GPIO_Port GPIOA
#define L_Emitter_Pin GPIO_PIN_6
#define L_Emitter_GPIO_Port GPIOA
#define FL_Emitter_Pin GPIO_PIN_7
#define FL_Emitter_GPIO_Port GPIOA
#define FR_Emitter_Pin GPIO_PIN_0
#define FR_Emitter_GPIO_Port GPIOB
#define vBat_Pin GPIO_PIN_1
#define vBat_GPIO_Port GPIOB
#define BLUE_Pin GPIO_PIN_9
#define BLUE_GPIO_Port GPIOC
#define BOTON2_Pin GPIO_PIN_11
#define BOTON2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
