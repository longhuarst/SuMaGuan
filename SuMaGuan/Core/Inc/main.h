/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#define STEP_CH1_CLK_Pin GPIO_PIN_0
#define STEP_CH1_CLK_GPIO_Port GPIOA
#define MOTOR_IN1_Pin GPIO_PIN_1
#define MOTOR_IN1_GPIO_Port GPIOA
#define MOTOR_SD_Pin GPIO_PIN_2
#define MOTOR_SD_GPIO_Port GPIOA
#define MOTOR_IN2_Pin GPIO_PIN_3
#define MOTOR_IN2_GPIO_Port GPIOA
#define PPM2_Pin GPIO_PIN_4
#define PPM2_GPIO_Port GPIOA
#define PPM3_Pin GPIO_PIN_5
#define PPM3_GPIO_Port GPIOA
#define OE_A_Pin GPIO_PIN_6
#define OE_A_GPIO_Port GPIOA
#define OE_B_Pin GPIO_PIN_7
#define OE_B_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_1
#define LED0_GPIO_Port GPIOB
#define STEP_CH1_ST_Pin GPIO_PIN_10
#define STEP_CH1_ST_GPIO_Port GPIOB
#define STEP_CH1_M1_Pin GPIO_PIN_11
#define STEP_CH1_M1_GPIO_Port GPIOB
#define STEP_CH1_M2_Pin GPIO_PIN_12
#define STEP_CH1_M2_GPIO_Port GPIOB
#define STEP_CH1_M3_Pin GPIO_PIN_13
#define STEP_CH1_M3_GPIO_Port GPIOB
#define STEP_CH1_EN_Pin GPIO_PIN_14
#define STEP_CH1_EN_GPIO_Port GPIOB
#define STEP_CH1_RESET_Pin GPIO_PIN_15
#define STEP_CH1_RESET_GPIO_Port GPIOB
#define STEP_CH1_DIR_Pin GPIO_PIN_8
#define STEP_CH1_DIR_GPIO_Port GPIOA
#define RE485_DE_Pin GPIO_PIN_11
#define RE485_DE_GPIO_Port GPIOA
#define OLED_CS2_Pin GPIO_PIN_12
#define OLED_CS2_GPIO_Port GPIOA
#define OLED_FCO_Pin GPIO_PIN_15
#define OLED_FCO_GPIO_Port GPIOA
#define OLED_CS1_Pin GPIO_PIN_3
#define OLED_CS1_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_4
#define OLED_DC_GPIO_Port GPIOB
#define OLED_RES_Pin GPIO_PIN_5
#define OLED_RES_GPIO_Port GPIOB
#define PPM0_Pin GPIO_PIN_6
#define PPM0_GPIO_Port GPIOB
#define PPM1_Pin GPIO_PIN_7
#define PPM1_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_8
#define OLED_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_9
#define OLED_SCL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
