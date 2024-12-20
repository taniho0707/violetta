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

#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

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
#define SWITCH_Pin LL_GPIO_PIN_13
#define SWITCH_GPIO_Port GPIOC
#define LED2_Pin LL_GPIO_PIN_0
#define LED2_GPIO_Port GPIOH
#define LED3_Pin LL_GPIO_PIN_1
#define LED3_GPIO_Port GPIOH
#define SEN_FL_Pin LL_GPIO_PIN_0
#define SEN_FL_GPIO_Port GPIOA
#define SEN_SL_Pin LL_GPIO_PIN_1
#define SEN_SL_GPIO_Port GPIOA
#define SEN_SR_Pin LL_GPIO_PIN_2
#define SEN_SR_GPIO_Port GPIOA
#define SEN_FR_Pin LL_GPIO_PIN_3
#define SEN_FR_GPIO_Port GPIOA
#define LED_A2_Pin LL_GPIO_PIN_4
#define LED_A2_GPIO_Port GPIOA
#define LED_A1_Pin LL_GPIO_PIN_5
#define LED_A1_GPIO_Port GPIOA
#define V_BAT_Pin LL_GPIO_PIN_0
#define V_BAT_GPIO_Port GPIOB
#define IMU_CS2_Pin LL_GPIO_PIN_12
#define IMU_CS2_GPIO_Port GPIOB
#define IMU_SCK2_Pin LL_GPIO_PIN_13
#define IMU_SCK2_GPIO_Port GPIOB
#define IMU_MISO2_Pin LL_GPIO_PIN_14
#define IMU_MISO2_GPIO_Port GPIOB
#define IMU_MOSI2_Pin LL_GPIO_PIN_15
#define IMU_MOSI2_GPIO_Port GPIOB
#define UART_TX_Pin LL_GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin LL_GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA
#define LED1_Pin LL_GPIO_PIN_11
#define LED1_GPIO_Port GPIOA
#define LED0_Pin LL_GPIO_PIN_12
#define LED0_GPIO_Port GPIOA
#define SWDIO_Pin LL_GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin LL_GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define ENC_R_PWM1_Pin LL_GPIO_PIN_15
#define ENC_R_PWM1_GPIO_Port GPIOA
#define ENC_R_PWM2_Pin LL_GPIO_PIN_3
#define ENC_R_PWM2_GPIO_Port GPIOB
#define ENC_L_PWM1_Pin LL_GPIO_PIN_4
#define ENC_L_PWM1_GPIO_Port GPIOB
#define ENC_L_PWM2_Pin LL_GPIO_PIN_5
#define ENC_L_PWM2_GPIO_Port GPIOB
#define MOT_R_PWM1_Pin LL_GPIO_PIN_6
#define MOT_R_PWM1_GPIO_Port GPIOB
#define MOT_R_PWM2_Pin LL_GPIO_PIN_7
#define MOT_R_PWM2_GPIO_Port GPIOB
#define MOT_L_PWM1_Pin LL_GPIO_PIN_8
#define MOT_L_PWM1_GPIO_Port GPIOB
#define MOT_L_PWM2_Pin LL_GPIO_PIN_9
#define MOT_L_PWM2_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
