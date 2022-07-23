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
#include "stm32f7xx_hal.h"

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
#define USS_Trigger6_Pin GPIO_PIN_7
#define USS_Trigger6_GPIO_Port GPIOF
#define USS_Data6_Pin GPIO_PIN_12
#define USS_Data6_GPIO_Port GPIOB
#define USS_Data6_EXTI_IRQn EXTI15_10_IRQn
#define REDtest_Pin GPIO_PIN_14
#define REDtest_GPIO_Port GPIOB
#define USS_Trigger2_Pin GPIO_PIN_8
#define USS_Trigger2_GPIO_Port GPIOD
#define USS_Data2_Pin GPIO_PIN_9
#define USS_Data2_GPIO_Port GPIOD
#define USS_Data2_EXTI_IRQn EXTI9_5_IRQn
#define Gsig_Pin GPIO_PIN_12
#define Gsig_GPIO_Port GPIOD
#define Rsig_Pin GPIO_PIN_13
#define Rsig_GPIO_Port GPIOD
#define Bsig_Pin GPIO_PIN_14
#define Bsig_GPIO_Port GPIOD
#define USS_Trigger5_Pin GPIO_PIN_6
#define USS_Trigger5_GPIO_Port GPIOC
#define USS_Data5_Pin GPIO_PIN_7
#define USS_Data5_GPIO_Port GPIOC
#define USS_Data5_EXTI_IRQn EXTI9_5_IRQn
#define USS_Trigger3_Pin GPIO_PIN_10
#define USS_Trigger3_GPIO_Port GPIOC
#define USS_Data3_Pin GPIO_PIN_11
#define USS_Data3_GPIO_Port GPIOC
#define USS_Data3_EXTI_IRQn EXTI15_10_IRQn
#define USS_Trigger4_Pin GPIO_PIN_12
#define USS_Trigger4_GPIO_Port GPIOC
#define USS_Data4_Pin GPIO_PIN_2
#define USS_Data4_GPIO_Port GPIOD
#define USS_Trigger1_Pin GPIO_PIN_5
#define USS_Trigger1_GPIO_Port GPIOD
#define USS_Data1_Pin GPIO_PIN_6
#define USS_Data1_GPIO_Port GPIOD
#define USS_Data1_EXTI_IRQn EXTI9_5_IRQn
#define BLUEtest_Pin GPIO_PIN_8
#define BLUEtest_GPIO_Port GPIOB
#define evt_rxpin_Pin GPIO_PIN_0
#define evt_rxpin_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
