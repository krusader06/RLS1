/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32wbxx_hal.h"

#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

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
#define BAT_VSENSE_Pin GPIO_PIN_0
#define BAT_VSENSE_GPIO_Port GPIOC
#define LED_COM_Pin GPIO_PIN_0
#define LED_COM_GPIO_Port GPIOA
#define CH4_ARM_Pin GPIO_PIN_1
#define CH4_ARM_GPIO_Port GPIOA
#define CH4_READY_Pin GPIO_PIN_2
#define CH4_READY_GPIO_Port GPIOA
#define CH4_FIRE_Pin GPIO_PIN_3
#define CH4_FIRE_GPIO_Port GPIOA
#define CH3_ARM_Pin GPIO_PIN_4
#define CH3_ARM_GPIO_Port GPIOA
#define CH3_READY_Pin GPIO_PIN_5
#define CH3_READY_GPIO_Port GPIOA
#define CH3_FIRE_Pin GPIO_PIN_6
#define CH3_FIRE_GPIO_Port GPIOA
#define CH2_ARM_Pin GPIO_PIN_7
#define CH2_ARM_GPIO_Port GPIOA
#define CH2_READY_Pin GPIO_PIN_8
#define CH2_READY_GPIO_Port GPIOA
#define CH2_FIRE_Pin GPIO_PIN_9
#define CH2_FIRE_GPIO_Port GPIOA
#define CH1_ARM_Pin GPIO_PIN_4
#define CH1_ARM_GPIO_Port GPIOC
#define CH1_READY_Pin GPIO_PIN_0
#define CH1_READY_GPIO_Port GPIOB
#define CH1_FIRE_Pin GPIO_PIN_1
#define CH1_FIRE_GPIO_Port GPIOB
#define ALARM_Pin GPIO_PIN_13
#define ALARM_GPIO_Port GPIOB
#define BEACON_LED4_Pin GPIO_PIN_14
#define BEACON_LED4_GPIO_Port GPIOB
#define BEACON_LED3_Pin GPIO_PIN_15
#define BEACON_LED3_GPIO_Port GPIOB
#define BEACON_LED2_Pin GPIO_PIN_6
#define BEACON_LED2_GPIO_Port GPIOC
#define BEACON_LED1_Pin GPIO_PIN_10
#define BEACON_LED1_GPIO_Port GPIOA
#define BAT_LED10_Pin GPIO_PIN_15
#define BAT_LED10_GPIO_Port GPIOA
#define BAT_LED9_Pin GPIO_PIN_10
#define BAT_LED9_GPIO_Port GPIOC
#define BAT_LED8_Pin GPIO_PIN_11
#define BAT_LED8_GPIO_Port GPIOC
#define BAT_LED7_Pin GPIO_PIN_12
#define BAT_LED7_GPIO_Port GPIOC
#define BAT_LED6_Pin GPIO_PIN_0
#define BAT_LED6_GPIO_Port GPIOD
#define BAT_LED5_Pin GPIO_PIN_1
#define BAT_LED5_GPIO_Port GPIOD
#define BAT_LED4_Pin GPIO_PIN_4
#define BAT_LED4_GPIO_Port GPIOB
#define BAT_LED3_Pin GPIO_PIN_5
#define BAT_LED3_GPIO_Port GPIOB
#define BAT_LED2_Pin GPIO_PIN_6
#define BAT_LED2_GPIO_Port GPIOB
#define BAT_LED1_Pin GPIO_PIN_7
#define BAT_LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
