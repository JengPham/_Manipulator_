/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define LeftLSW_Pin GPIO_PIN_13
#define LeftLSW_GPIO_Port GPIOC
#define RightLSW_Pin GPIO_PIN_14
#define RightLSW_GPIO_Port GPIOC
#define Brake_Pin GPIO_PIN_3
#define Brake_GPIO_Port GPIOA
#define IrefA_Pin GPIO_PIN_4
#define IrefA_GPIO_Port GPIOA
#define IrefB_Pin GPIO_PIN_5
#define IrefB_GPIO_Port GPIOA
#define QEIA_Pin GPIO_PIN_6
#define QEIA_GPIO_Port GPIOA
#define QEIB_Pin GPIO_PIN_7
#define QEIB_GPIO_Port GPIOA
#define W_R_3_Pin GPIO_PIN_2
#define W_R_3_GPIO_Port GPIOB
#define TX3_Pin GPIO_PIN_10
#define TX3_GPIO_Port GPIOB
#define RX3_Pin GPIO_PIN_11
#define RX3_GPIO_Port GPIOB
#define Fault_Pin GPIO_PIN_15
#define Fault_GPIO_Port GPIOB
#define Reset_Pin GPIO_PIN_6
#define Reset_GPIO_Port GPIOC
#define En_Pin GPIO_PIN_7
#define En_GPIO_Port GPIOC
#define Pulse_Pin GPIO_PIN_8
#define Pulse_GPIO_Port GPIOC
#define M2_Pin GPIO_PIN_9
#define M2_GPIO_Port GPIOC
#define M1_Pin GPIO_PIN_8
#define M1_GPIO_Port GPIOA
#define M0_Pin GPIO_PIN_9
#define M0_GPIO_Port GPIOA
#define AngleMonitor_Pin GPIO_PIN_10
#define AngleMonitor_GPIO_Port GPIOA
#define Dir_Pin GPIO_PIN_11
#define Dir_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
#define TX4_Pin GPIO_PIN_10
#define TX4_GPIO_Port GPIOC
#define RX4_Pin GPIO_PIN_11
#define RX4_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
