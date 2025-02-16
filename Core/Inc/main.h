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

#include <math.h>
#include <string.h>

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

void CMD_Handler();
void LED_CMD_Handle(uint8_t cmd_byte, uint32_t data_bytes);
void SERVO_CMD_Handle(uint8_t cmd_byte, uint32_t data_bytes);
void AddrLED_CMD_Handle(uint8_t cmd_byte, uint32_t data_bytes);

void setup_TIM2();
void TIM3_setup();
void setup_TIM4();

void setPWM(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t value);

void setup_CRT();
void set_CRT_Gamma(float gamma);
void setFlashSpeed(float frequency);
void calc_brightness_table();
uint16_t get_CRT(uint16_t val);

void SERVO_setAngle(uint16_t angle);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AddrLED_Pin GPIO_PIN_4
#define AddrLED_GPIO_Port GPIOA
#define SERVO_Pin GPIO_PIN_5
#define SERVO_GPIO_Port GPIOA
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
