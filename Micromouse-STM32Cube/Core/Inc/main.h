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
#define InternalLED_Pin GPIO_PIN_13
#define InternalLED_GPIO_Port GPIOC
#define L_Receiver_Pin GPIO_PIN_0
#define L_Receiver_GPIO_Port GPIOA
#define R_Receiver_Pin GPIO_PIN_1
#define R_Receiver_GPIO_Port GPIOA
#define VBat_Pin GPIO_PIN_4
#define VBat_GPIO_Port GPIOA
#define FR_Receiver_Pin GPIO_PIN_0
#define FR_Receiver_GPIO_Port GPIOB
#define FL_Receiver_Pin GPIO_PIN_1
#define FL_Receiver_GPIO_Port GPIOB
#define MR2_Pin GPIO_PIN_10
#define MR2_GPIO_Port GPIOB
#define Boton_Pin GPIO_PIN_12
#define Boton_GPIO_Port GPIOB
#define ExternalLED_Pin GPIO_PIN_13
#define ExternalLED_GPIO_Port GPIOB
#define ML1_Pin GPIO_PIN_8
#define ML1_GPIO_Port GPIOA
#define ML2_Pin GPIO_PIN_9
#define ML2_GPIO_Port GPIOA
#define MR1_Pin GPIO_PIN_10
#define MR1_GPIO_Port GPIOA
#define FR_Emitter_Pin GPIO_PIN_15
#define FR_Emitter_GPIO_Port GPIOA
#define FL_Emitter_Pin GPIO_PIN_3
#define FL_Emitter_GPIO_Port GPIOB
#define R_Emitter_Pin GPIO_PIN_4
#define R_Emitter_GPIO_Port GPIOB
#define L_Emitter_Pin GPIO_PIN_5
#define L_Emitter_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
