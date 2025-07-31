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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"
#include "trcRecorder.h"
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

/* USER CODE BEGIN Private defines */
//#define Travel_1_Pin GPIO_PIN_0
//#define Travel_1_GPIO_Port GPIOC
//#define Travel_2_Pin GPIO_PIN_1
//#define Travel_2_GPIO_Port GPIOC
//#define ENCODER_CH1_Pin GPIO_PIN_0
//#define ENCODER_CH1_GPIO_Port GPIOA
//#define ENCODER_CH2_Pin GPIO_PIN_1
//#define ENCODER_CH2_GPIO_Port GPIOA
//#define Travel_3_Pin GPIO_PIN_6
//#define Travel_3_GPIO_Port GPIOA
//#define Travel_4_Pin GPIO_PIN_7
//#define Travel_4_GPIO_Port GPIOA
//#define REAR_RIGHT_WHEEL_Pin GPIO_PIN_8
//#define REAR_RIGHT_WHEEL_GPIO_Port GPIOA
//#define REAR_LEFT_WHEEL_Pin GPIO_PIN_9
//#define REAR_LEFT_WHEEL_GPIO_Port GPIOA
//#define FRONT_LEFT_WHEEL_Pin GPIO_PIN_10
//#define FRONT_LEFT_WHEEL_GPIO_Port GPIOA
//#define FRONT_RIGHT_WHEEL_Pin GPIO_PIN_11
//#define FRONT_RIGHT_WHEEL_GPIO_Port GPIOA
//#define IMU_SCL_Pin GPIO_PIN_6
//#define IMU_SCL_GPIO_Port GPIOB
//#define IMU_SDA_Pin GPIO_PIN_7
//#define IMU_SDA_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
