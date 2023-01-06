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
#define OSC_8MHz_IN_Pin GPIO_PIN_0
#define OSC_8MHz_IN_GPIO_Port GPIOH
#define OSC_8MHz_OUT_Pin GPIO_PIN_1
#define OSC_8MHz_OUT_GPIO_Port GPIOH
#define FRSensAnalog_Pin GPIO_PIN_0
#define FRSensAnalog_GPIO_Port GPIOC
#define FLSensAnalog_Pin GPIO_PIN_1
#define FLSensAnalog_GPIO_Port GPIOC
#define RSensAnalog_Pin GPIO_PIN_2
#define RSensAnalog_GPIO_Port GPIOC
#define LSensAnalog_Pin GPIO_PIN_3
#define LSensAnalog_GPIO_Port GPIOC
#define FR_Emitter_Pin GPIO_PIN_2
#define FR_Emitter_GPIO_Port GPIOA
#define FL_Emitter_Pin GPIO_PIN_3
#define FL_Emitter_GPIO_Port GPIOA
#define L_Emitter_Pin GPIO_PIN_5
#define L_Emitter_GPIO_Port GPIOA
#define BatteryAnalog_Pin GPIO_PIN_1
#define BatteryAnalog_GPIO_Port GPIOB
#define STM32_D__Pin GPIO_PIN_14
#define STM32_D__GPIO_Port GPIOB
#define STM32_D_B15_Pin GPIO_PIN_15
#define STM32_D_B15_GPIO_Port GPIOB
#define LED9_Pin GPIO_PIN_9
#define LED9_GPIO_Port GPIOC
#define Boton2_Pin GPIO_PIN_11
#define Boton2_GPIO_Port GPIOA
#define Boton_Pin GPIO_PIN_11
#define Boton_GPIO_Port GPIOC
#define R_Emitter_Pin GPIO_PIN_3
#define R_Emitter_GPIO_Port GPIOB
#define ML1_PWM_Pin GPIO_PIN_6
#define ML1_PWM_GPIO_Port GPIOB
#define ML2_PWM_Pin GPIO_PIN_7
#define ML2_PWM_GPIO_Port GPIOB
#define MR1_PWM_Pin GPIO_PIN_8
#define MR1_PWM_GPIO_Port GPIOB
#define MR2_PWM_Pin GPIO_PIN_9
#define MR2_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
