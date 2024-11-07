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
#include "stm32l4xx_hal.h"

#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_i2c.h"
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_gpio.h"

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
#define ADC_CNVST_Pin LL_GPIO_PIN_13
#define ADC_CNVST_GPIO_Port GPIOC
#define ADC_BUSY_Pin LL_GPIO_PIN_14
#define ADC_BUSY_GPIO_Port GPIOC
#define FRAM_CS_Pin LL_GPIO_PIN_15
#define FRAM_CS_GPIO_Port GPIOC
#define OSC_HSE_IN_Pin LL_GPIO_PIN_0
#define OSC_HSE_IN_GPIO_Port GPIOH
#define MOTOR_R_PWM_Pin LL_GPIO_PIN_0
#define MOTOR_R_PWM_GPIO_Port GPIOA
#define MOTOR_L_PWM_Pin LL_GPIO_PIN_1
#define MOTOR_L_PWM_GPIO_Port GPIOA
#define SPEAKER_PWM_Pin LL_GPIO_PIN_2
#define SPEAKER_PWM_GPIO_Port GPIOA
#define MOTOR_R_SENS_Pin LL_GPIO_PIN_3
#define MOTOR_R_SENS_GPIO_Port GPIOA
#define MOTOR_L_SENS_Pin LL_GPIO_PIN_4
#define MOTOR_L_SENS_GPIO_Port GPIOA
#define BATTERY_Pin LL_GPIO_PIN_5
#define BATTERY_GPIO_Port GPIOA
#define ENCODER_L1_Pin LL_GPIO_PIN_6
#define ENCODER_L1_GPIO_Port GPIOA
#define ENCODER_L2_Pin LL_GPIO_PIN_7
#define ENCODER_L2_GPIO_Port GPIOA
#define IRLED_P_Pin LL_GPIO_PIN_0
#define IRLED_P_GPIO_Port GPIOB
#define IRLED_N_FL_Pin LL_GPIO_PIN_1
#define IRLED_N_FL_GPIO_Port GPIOB
#define MOTOR_L_EN_Pin LL_GPIO_PIN_2
#define MOTOR_L_EN_GPIO_Port GPIOB
#define MOTOR_R_EN_Pin LL_GPIO_PIN_10
#define MOTOR_R_EN_GPIO_Port GPIOB
#define SUCTION_PWM_Pin LL_GPIO_PIN_11
#define SUCTION_PWM_GPIO_Port GPIOB
#define ADC_CS_Pin LL_GPIO_PIN_12
#define ADC_CS_GPIO_Port GPIOB
#define ADC_SCK_Pin LL_GPIO_PIN_13
#define ADC_SCK_GPIO_Port GPIOB
#define ADC_MISO_Pin LL_GPIO_PIN_14
#define ADC_MISO_GPIO_Port GPIOB
#define ADC_MOSI_Pin LL_GPIO_PIN_15
#define ADC_MOSI_GPIO_Port GPIOB
#define IRLED_N_L_Pin LL_GPIO_PIN_8
#define IRLED_N_L_GPIO_Port GPIOA
#define IRLED_N_R_Pin LL_GPIO_PIN_11
#define IRLED_N_R_GPIO_Port GPIOA
#define IRLED_N_FR_Pin LL_GPIO_PIN_12
#define IRLED_N_FR_GPIO_Port GPIOA
#define IMU_CS_Pin LL_GPIO_PIN_15
#define IMU_CS_GPIO_Port GPIOA
#define IMU_FRAM_SCK_Pin LL_GPIO_PIN_3
#define IMU_FRAM_SCK_GPIO_Port GPIOB
#define IMU_FRAM_MISO_Pin LL_GPIO_PIN_4
#define IMU_FRAM_MISO_GPIO_Port GPIOB
#define IMU_FRAM_MOSI_Pin LL_GPIO_PIN_5
#define IMU_FRAM_MOSI_GPIO_Port GPIOB
#define ENCODER_R1_Pin LL_GPIO_PIN_6
#define ENCODER_R1_GPIO_Port GPIOB
#define ENCODER_R2_Pin LL_GPIO_PIN_7
#define ENCODER_R2_GPIO_Port GPIOB
#define PRESSURE_SCL_Pin LL_GPIO_PIN_8
#define PRESSURE_SCL_GPIO_Port GPIOB
#define PRESSURE_SDA_Pin LL_GPIO_PIN_9
#define PRESSURE_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
