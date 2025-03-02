/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define ERROR_LED_Pin GPIO_PIN_13
#define ERROR_LED_GPIO_Port GPIOC
#define DEBUG_LED_Pin GPIO_PIN_14
#define DEBUG_LED_GPIO_Port GPIOC
#define MTR_DECAY_Pin GPIO_PIN_15
#define MTR_DECAY_GPIO_Port GPIOC
#define MTR_ISENA_Pin GPIO_PIN_0
#define MTR_ISENA_GPIO_Port GPIOA
#define MTR_ISENB_Pin GPIO_PIN_1
#define MTR_ISENB_GPIO_Port GPIOA
#define VBAT_SENSE_Pin GPIO_PIN_2
#define VBAT_SENSE_GPIO_Port GPIOA
#define IBAT_SENSE_Pin GPIO_PIN_3
#define IBAT_SENSE_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_0
#define SPI1_CS_GPIO_Port GPIOB
#define MTR_NHOME_Pin GPIO_PIN_1
#define MTR_NHOME_GPIO_Port GPIOB
#define MTR_NFLT_Pin GPIO_PIN_2
#define MTR_NFLT_GPIO_Port GPIOB
#define MTR_NENBL_Pin GPIO_PIN_10
#define MTR_NENBL_GPIO_Port GPIOB
#define MTR_NSLP_Pin GPIO_PIN_11
#define MTR_NSLP_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define MTR_ENCA_Pin GPIO_PIN_8
#define MTR_ENCA_GPIO_Port GPIOA
#define MTR_ENCB_Pin GPIO_PIN_9
#define MTR_ENCB_GPIO_Port GPIOA
#define MTR_NRST_Pin GPIO_PIN_10
#define MTR_NRST_GPIO_Port GPIOA
#define MTR_STEP_Pin GPIO_PIN_4
#define MTR_STEP_GPIO_Port GPIOB
#define MTR_DIR_Pin GPIO_PIN_5
#define MTR_DIR_GPIO_Port GPIOB
#define MTR_M0_Pin GPIO_PIN_6
#define MTR_M0_GPIO_Port GPIOB
#define MTR_M1_Pin GPIO_PIN_8
#define MTR_M1_GPIO_Port GPIOB
#define MTR_M2_Pin GPIO_PIN_9
#define MTR_M2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
