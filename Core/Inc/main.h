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
#include "stm32l0xx_hal.h"

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
#define TX__RF_SW_Pin GPIO_PIN_1
#define TX__RF_SW_GPIO_Port GPIOA
#define RX_RF_SW_Pin GPIO_PIN_2
#define RX_RF_SW_GPIO_Port GPIOA
#define PD4_Pmeas_Pin GPIO_PIN_6
#define PD4_Pmeas_GPIO_Port GPIOA
#define PD3_Pmeas_Pin GPIO_PIN_7
#define PD3_Pmeas_GPIO_Port GPIOA
#define PD2_Pmeas_Pin GPIO_PIN_0
#define PD2_Pmeas_GPIO_Port GPIOB
#define PD1_Pmeas_Pin GPIO_PIN_1
#define PD1_Pmeas_GPIO_Port GPIOB
#define MUX_ENABLE_Pin GPIO_PIN_8
#define MUX_ENABLE_GPIO_Port GPIOA
#define CTRL_4_5V_Pin GPIO_PIN_3
#define CTRL_4_5V_GPIO_Port GPIOB
#define CTRL_5V_Pin GPIO_PIN_4
#define CTRL_5V_GPIO_Port GPIOB
#define PULSE_Pin GPIO_PIN_5
#define PULSE_GPIO_Port GPIOB
#define PULSE_EXTI_IRQn EXTI4_15_IRQn
#define CTRL_12V_Pin GPIO_PIN_8
#define CTRL_12V_GPIO_Port GPIOB
#define CTRL_15V_Pin GPIO_PIN_9
#define CTRL_15V_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
