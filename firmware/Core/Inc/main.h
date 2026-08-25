/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define ENCODER_LEFT_BACKWARD_A_Pin GPIO_PIN_0
#define ENCODER_LEFT_BACKWARD_A_GPIO_Port GPIOA
#define ENCODER_LEFT_BACKWARD_B_Pin GPIO_PIN_1
#define ENCODER_LEFT_BACKWARD_B_GPIO_Port GPIOA
#define LEFT_BACKWARD_1_Pin GPIO_PIN_2
#define LEFT_BACKWARD_1_GPIO_Port GPIOA
#define LEFT_BACKWARD_2_Pin GPIO_PIN_3
#define LEFT_BACKWARD_2_GPIO_Port GPIOA
#define RIGHT_FORWARD_1_Pin GPIO_PIN_6
#define RIGHT_FORWARD_1_GPIO_Port GPIOA
#define RIGHT_FORWARD_2_Pin GPIO_PIN_7
#define RIGHT_FORWARD_2_GPIO_Port GPIOA
#define RIGHT_BACKWARD_1_Pin GPIO_PIN_0
#define RIGHT_BACKWARD_1_GPIO_Port GPIOB
#define RIGHT_BACKWARD_2_Pin GPIO_PIN_1
#define RIGHT_BACKWARD_2_GPIO_Port GPIOB
#define ENCODER_LEFT_FORWARD_A_Pin GPIO_PIN_9
#define ENCODER_LEFT_FORWARD_A_GPIO_Port GPIOE
#define ENCODER_LEFT_FORWARD_B_Pin GPIO_PIN_11
#define ENCODER_LEFT_FORWARD_B_GPIO_Port GPIOE
#define GYRO_TX_Pin GPIO_PIN_10
#define GYRO_TX_GPIO_Port GPIOB
#define GYRO_RX_Pin GPIO_PIN_11
#define GYRO_RX_GPIO_Port GPIOB
#define ENCODER_RIGHT_FORWARD_A_Pin GPIO_PIN_12
#define ENCODER_RIGHT_FORWARD_A_GPIO_Port GPIOD
#define ENCODER_RIGHT_FORWARD_B_Pin GPIO_PIN_13
#define ENCODER_RIGHT_FORWARD_B_GPIO_Port GPIOD
#define ENCODER_RIGHT_BACKWARD_A_Pin GPIO_PIN_6
#define ENCODER_RIGHT_BACKWARD_A_GPIO_Port GPIOC
#define ENCODER_RIGHT_BACKWARD_B_Pin GPIO_PIN_7
#define ENCODER_RIGHT_BACKWARD_B_GPIO_Port GPIOC
#define OPENMV_TX_Pin GPIO_PIN_9
#define OPENMV_TX_GPIO_Port GPIOA
#define OPENMV_RX_Pin GPIO_PIN_10
#define OPENMV_RX_GPIO_Port GPIOA
#define LEFT_FORWARD_1_Pin GPIO_PIN_15
#define LEFT_FORWARD_1_GPIO_Port GPIOA
#define SERVO_1_TX_Pin GPIO_PIN_10
#define SERVO_1_TX_GPIO_Port GPIOC
#define SERVO_1_RX_Pin GPIO_PIN_11
#define SERVO_1_RX_GPIO_Port GPIOC
#define SERVO_2_TX_Pin GPIO_PIN_12
#define SERVO_2_TX_GPIO_Port GPIOC
#define SERVO_2_RX_Pin GPIO_PIN_2
#define SERVO_2_RX_GPIO_Port GPIOD
#define BLUETOOTH_TX_Pin GPIO_PIN_5
#define BLUETOOTH_TX_GPIO_Port GPIOD
#define BLUETOOTH_RX_Pin GPIO_PIN_6
#define BLUETOOTH_RX_GPIO_Port GPIOD
#define LEFT_FORWARD_2_Pin GPIO_PIN_3
#define LEFT_FORWARD_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
