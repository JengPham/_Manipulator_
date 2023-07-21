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
/* USER CODE BEGIN Private defines */

/***** Công t?c hành trình *****/
#define CT1_PIN    		GPIO_PIN_8
#define CT1_PORT   		GPIOC
#define CT2_PIN    		GPIO_PIN_7
#define CT2_PORT   		GPIOC
#define CT3_PIN    		GPIO_PIN_6
#define CT3_PORT   		GPIOC
#define CT4_PIN    		GPIO_PIN_8
#define CT4_PORT   		GPIOG
#define CT5_PIN    		GPIO_PIN_7
#define CT5_PORT   		GPIOG
#define CT6_PIN    	  GPIO_PIN_6
#define CT6_PORT      GPIOG
#define Dir1_PIN      GPIO_PIN_12
#define Dir1_PORT     GPIOD
#define Pul1_PIN      GPIO_PIN_12
#define Pul1_PORT     GPIOB
#define En1_PIN       GPIO_PIN_13
#define En1_PORT      GPIOB
#define Rst1_PIN      GPIO_PIN_10
#define Rst1_PORT     GPIOB
/****Ð?ng co 2****/
#define Dir2_PIN      GPIO_PIN_14
#define Dir2_PORT     GPIOE
#define Pul2_PIN      GPIO_PIN_12
#define Pul2_PORT     GPIOE
#define En2_PIN       GPIO_PIN_11
#define En2_PORT      GPIOE
#define Rst2_PIN      GPIO_PIN_10
#define Rst2_PORT     GPIOE
/****Ð?ng co 3****/
#define Dir3_PIN      GPIO_PIN_8
#define Dir3_PORT     GPIOE
#define Pul3_PIN      GPIO_PIN_1
#define Pul3_PORT     GPIOG
#define En3_PIN       GPIO_PIN_0
#define En3_PORT      GPIOG
#define Rst3_PIN      GPIO_PIN_15
#define Rst3_PORT     GPIOF
/****Ð?ng co 4****/
#define Dir4_PIN      GPIO_PIN_1
#define Dir4_PORT     GPIOB
#define Pul4_PIN      GPIO_PIN_11
#define Pul4_PORT     GPIOF
#define En4_PIN       GPIO_PIN_12
#define En4_PORT      GPIOF
#define Rst4_PIN      GPIO_PIN_13
#define Rst4_PORT     GPIOF
/****Ð?ng co 5****/
#define Dir5_PIN      GPIO_PIN_0
#define Dir5_PORT     GPIOC
#define Pul5_PIN      GPIO_PIN_3
#define Pul5_PORT     GPIOC
#define En5_PIN       GPIO_PIN_0
#define En5_PORT      GPIOA
#define Rst5_PIN      GPIO_PIN_3
#define Rst5_PORT     GPIOA
/****Ð?ng co 6****/
#define Dir6_PIN      GPIO_PIN_13
#define Dir6_PORT     GPIOC
#define Pul6_PIN      GPIO_PIN_15
#define Pul6_PORT     GPIOC
#define En6_PIN       GPIO_PIN_2
#define En6_PORT      GPIOF
#define Rst6_PIN      GPIO_PIN_5
#define Rst6_PORT     GPIOF


/****Ch?n mode cho d?ng co****/
#define StpMode0_PIN  GPIO_PIN_10
#define StpMode1_PIN  GPIO_PIN_9
#define StpMode2_PIN  GPIO_PIN_8
#define StpMode_PORT  GPIOD
/****SPI*****/
#define LDAC4_PORT    GPIOE
#define LDAC4_PIN     GPIO_PIN_5
#define CS4_PORT      GPIOE
#define CS4_PIN       GPIO_PIN_4
#define LDAC5_PORT    GPIOF
#define LDAC5_PIN     GPIO_PIN_6
#define CS5_PORT      GPIOF
#define CS5_PIN       GPIO_PIN_8

//#define pi  3.141592

typedef struct{
	uint8_t Ready;
	uint8_t Start;
	uint8_t Run;
} Robot_state;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
