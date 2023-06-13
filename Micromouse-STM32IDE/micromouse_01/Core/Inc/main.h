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
ADC_HandleTypeDef* Get_HADC1_Ptr(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FR_RECEIVER_Pin GPIO_PIN_0
#define FR_RECEIVER_GPIO_Port GPIOC
#define FL_RECEIVER_Pin GPIO_PIN_1
#define FL_RECEIVER_GPIO_Port GPIOC
#define R_RECEIVER_Pin GPIO_PIN_2
#define R_RECEIVER_GPIO_Port GPIOC
#define F_RECEIVER_Pin GPIO_PIN_3
#define F_RECEIVER_GPIO_Port GPIOC
#define BOTON_Pin GPIO_PIN_1
#define BOTON_GPIO_Port GPIOA
#define A_Pin GPIO_PIN_2
#define A_GPIO_Port GPIOA
#define L_Emitter_Pin GPIO_PIN_5
#define L_Emitter_GPIO_Port GPIOA
#define R_Emitter_Pin GPIO_PIN_6
#define R_Emitter_GPIO_Port GPIOA
#define FR_Emitter_Pin GPIO_PIN_7
#define FR_Emitter_GPIO_Port GPIOA
#define FL_Emitter_Pin GPIO_PIN_0
#define FL_Emitter_GPIO_Port GPIOB
#define BAT_LVL_Pin GPIO_PIN_1
#define BAT_LVL_GPIO_Port GPIOB
#define ML_OUT_A_Pin GPIO_PIN_6
#define ML_OUT_A_GPIO_Port GPIOC
#define ML_OUT_B_Pin GPIO_PIN_7
#define ML_OUT_B_GPIO_Port GPIOC
#define BLUE_LED_Pin GPIO_PIN_9
#define BLUE_LED_GPIO_Port GPIOC
#define MR_OUT_A_Pin GPIO_PIN_8
#define MR_OUT_A_GPIO_Port GPIOA
#define MR_OUT_B_Pin GPIO_PIN_9
#define MR_OUT_B_GPIO_Port GPIOA
#define BOTON2_Pin GPIO_PIN_11
#define BOTON2_GPIO_Port GPIOA
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
