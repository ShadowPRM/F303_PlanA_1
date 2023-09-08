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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f3xx_hal.h"

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
#define MBadr6_Pin GPIO_PIN_13
#define MBadr6_GPIO_Port GPIOC
#define MBadr4_Pin GPIO_PIN_0
#define MBadr4_GPIO_Port GPIOC
#define MBadr3_Pin GPIO_PIN_1
#define MBadr3_GPIO_Port GPIOC
#define MBadr2_Pin GPIO_PIN_2
#define MBadr2_GPIO_Port GPIOC
#define MBadr1_Pin GPIO_PIN_3
#define MBadr1_GPIO_Port GPIOC
#define MBadr7_Pin GPIO_PIN_0
#define MBadr7_GPIO_Port GPIOA
#define InKnB_Pin GPIO_PIN_15
#define InKnB_GPIO_Port GPIOB
#define InKnM_Pin GPIO_PIN_6
#define InKnM_GPIO_Port GPIOC
#define InLatrMax_Pin GPIO_PIN_7
#define InLatrMax_GPIO_Port GPIOC
#define InLatrMin_Pin GPIO_PIN_8
#define InLatrMin_GPIO_Port GPIOC
#define InRezerv2_Pin GPIO_PIN_9
#define InRezerv2_GPIO_Port GPIOC
#define InAuto2_Pin GPIO_PIN_8
#define InAuto2_GPIO_Port GPIOA
#define InAuto1_Pin GPIO_PIN_15
#define InAuto1_GPIO_Port GPIOA
#define StepD3_Pin GPIO_PIN_12
#define StepD3_GPIO_Port GPIOC
#define StepD2_Pin GPIO_PIN_2
#define StepD2_GPIO_Port GPIOD
#define OutRezerv1_Pin GPIO_PIN_4
#define OutRezerv1_GPIO_Port GPIOB
#define PinLEDdebug_Pin GPIO_PIN_5
#define PinLEDdebug_GPIO_Port GPIOB
#define MBadr5_Pin GPIO_PIN_8
#define MBadr5_GPIO_Port GPIOB
#define MBadr0_Pin GPIO_PIN_9
#define MBadr0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
