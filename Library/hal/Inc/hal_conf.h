//******************************************************************************
// @addtogroup HAL
// @file       hal_conf.h
// @brief      ハードウェアに紐付く定義
//******************************************************************************
#pragma once

#ifdef LINUX
#include <cstdint>
#endif

#ifdef STM32
#include "myint.h"
#endif

namespace hal {

/// @brief HalStatus
enum class HalStatus : uint8_t {
    SUCCESS = 0,
    NOIMPLEMENT = 1,
    INVALID_PARAMS = 254,
    ERROR = 255
};

enum class InitializeType : uint8_t {
    Sync = 0,
    Async = 1,
    Dma = 2,
};

#ifdef MOUSE_VIOLETTA
#define LED_NUMS 4
enum class LedNumbers : uint8_t {
    FRONT1 = 0,
    FRONT2,
    TOP1,
    TOP2,
    ALL = 255,
};

const float BATTERY_RATIO = 1.5f;

#define WALLSENSOR_NUMS 5
enum class WallSensorNumbers : uint8_t {
    FRONTLEFT = 0,
    LEFT,
    FRONT,
    RIGHT,
    FRONTRIGHT,
    ALL = 255,
};

struct WallSensorData {
    uint16_t FRONTLEFT;
    uint16_t LEFT;
    uint16_t FRONT;
    uint16_t RIGHT;
    uint16_t FRONTRIGHT;
};
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
#define LED_NUMS 10
enum class LedNumbers : uint8_t {
    LEFT = 0,
    RIGHT,
    FRONTL,
    FRONTR,
    MIDDLE1,
    MIDDLE2,
    MIDDLE3,
    MIDDLE4,
    MIDDLE5,
    FLAG,
    ALL = 255,
};

const float BATTERY_RATIO = 5.7f;

const uint32_t TIMER_COUNT_INTERVAL = 250;  // [us]
const uint32_t TIMER_COUNT_MAX = 12500;     // 1 カウントあたり ? [us] = ? [ns]

const uint16_t IMU_TSEN = 256;    // [LSB/°C]
const uint16_t IMU_TOFF = 25;     // [°C]
const float IMU_GSEN = 0.140;     // [dps/LSB]
const float IMU_ASEN = 0.000244;  // [g/LSB]

#define WALLSENSOR_NUMS 4
enum class WallSensorNumbers : uint8_t {
    FRONTLEFT = 0,
    LEFT,
    RIGHT,
    FRONTRIGHT,
    ALL = 255,
};

struct WallSensorData {
    uint16_t FRONTLEFT;
    uint16_t LEFT;
    uint16_t RIGHT;
    uint16_t FRONTRIGHT;
};

#define ADC_CNVST_Pin           LL_GPIO_PIN_13
#define ADC_CNVST_GPIO_Port     GPIOC
#define ADC_BUSY_Pin            LL_GPIO_PIN_14
#define ADC_BUSY_GPIO_Port      GPIOC
#define FRAM_CS_Pin             LL_GPIO_PIN_15
#define FRAM_CS_GPIO_Port       GPIOC
#define OSC_HSE_IN_Pin          LL_GPIO_PIN_0
#define OSC_HSE_IN_GPIO_Port    GPIOH
#define MOTOR_R_PWM_Pin         LL_GPIO_PIN_0
#define MOTOR_R_PWM_GPIO_Port   GPIOA
#define MOTOR_L_PWM_Pin         LL_GPIO_PIN_1
#define MOTOR_L_PWM_GPIO_Port   GPIOA
#define SPEAKER_PWM_Pin         LL_GPIO_PIN_2
#define SPEAKER_PWM_GPIO_Port   GPIOA
#define MOTOR_R_SENS_Pin        LL_GPIO_PIN_3
#define MOTOR_R_SENS_GPIO_Port  GPIOA
#define MOTOR_L_SENS_Pin        LL_GPIO_PIN_4
#define MOTOR_L_SENS_GPIO_Port  GPIOA
#define BATTERY_Pin             LL_GPIO_PIN_5
#define BATTERY_GPIO_Port       GPIOA
#define ENCODER_L1_Pin          LL_GPIO_PIN_6
#define ENCODER_L1_GPIO_Port    GPIOA
#define ENCODER_L2_Pin          LL_GPIO_PIN_7
#define ENCODER_L2_GPIO_Port    GPIOA
#define IRLED_P_Pin             LL_GPIO_PIN_0
#define IRLED_P_GPIO_Port       GPIOB
#define IRLED_N_FL_Pin          LL_GPIO_PIN_1
#define IRLED_N_FL_GPIO_Port    GPIOB
#define MOTOR_L_EN_Pin          LL_GPIO_PIN_2
#define MOTOR_L_EN_GPIO_Port    GPIOB
#define MOTOR_R_EN_Pin          LL_GPIO_PIN_10
#define MOTOR_R_EN_GPIO_Port    GPIOB
#define SUCTION_PWM_Pin         LL_GPIO_PIN_11
#define SUCTION_PWM_GPIO_Port   GPIOB
#define ADC_CS_Pin              LL_GPIO_PIN_12
#define ADC_CS_GPIO_Port        GPIOB
#define ADC_SCK_Pin             LL_GPIO_PIN_13
#define ADC_SCK_GPIO_Port       GPIOB
#define ADC_MISO_Pin            LL_GPIO_PIN_14
#define ADC_MISO_GPIO_Port      GPIOB
#define ADC_MOSI_Pin            LL_GPIO_PIN_15
#define ADC_MOSI_GPIO_Port      GPIOB
#define IRLED_N_L_Pin           LL_GPIO_PIN_8
#define IRLED_N_L_GPIO_Port     GPIOA
#define IRLED_N_R_Pin           LL_GPIO_PIN_11
#define IRLED_N_R_GPIO_Port     GPIOA
#define IRLED_N_FR_Pin          LL_GPIO_PIN_12
#define IRLED_N_FR_GPIO_Port    GPIOA
#define IMU_CS_Pin              LL_GPIO_PIN_15
#define IMU_CS_GPIO_Port        GPIOA
#define IMU_FRAM_SCK_Pin        LL_GPIO_PIN_3
#define IMU_FRAM_SCK_GPIO_Port  GPIOB
#define IMU_FRAM_MISO_Pin       LL_GPIO_PIN_4
#define IMU_FRAM_MISO_GPIO_Port GPIOB
#define IMU_FRAM_MOSI_Pin       LL_GPIO_PIN_5
#define IMU_FRAM_MOSI_GPIO_Port GPIOB
#define ENCODER_R1_Pin          LL_GPIO_PIN_6
#define ENCODER_R1_GPIO_Port    GPIOB
#define ENCODER_R2_Pin          LL_GPIO_PIN_7
#define ENCODER_R2_GPIO_Port    GPIOB
#define PRESSURE_SCL_Pin        LL_GPIO_PIN_8
#define PRESSURE_SCL_GPIO_Port  GPIOB
#define PRESSURE_SDA_Pin        LL_GPIO_PIN_9
#define PRESSURE_SDA_GPIO_Port  GPIOB
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_ZIRCONIA2KAI
#define LED_NUMS 4
enum class LedNumbers : uint8_t {
    RED = 0,
    YELLOW,
    GREEN,
    BLUE,
    ALL = 255,
};

const float BATTERY_RATIO = 2.0f;

const uint32_t TIMER_COUNT_INTERVAL = 250;  // [us]
const uint32_t TIMER_COUNT_MAX = 12500;     // 1 カウントあたり 0.02 [us] = 20 [ns]

const uint16_t IMU_TSEN = 256;    // [LSB/°C]
const uint16_t IMU_TOFF = 25;     // [°C]
const float IMU_GSEN = 0.140;     // [dps/LSB]
const float IMU_ASEN = 0.000244;  // [g/LSB]

#define WALLSENSOR_NUMS 4
enum class WallSensorNumbers : uint8_t {
    FRONTLEFT = 0,
    LEFT,
    RIGHT,
    FRONTRIGHT,
    ALL = 255,
};

struct WallSensorData {
    uint16_t FRONTLEFT;
    uint16_t LEFT;
    uint16_t RIGHT;
    uint16_t FRONTRIGHT;
};

#define SWITCH_Pin           LL_GPIO_PIN_13
#define SWITCH_GPIO_Port     GPIOC
#define LED2_Pin             LL_GPIO_PIN_0
#define LED2_GPIO_Port       GPIOH
#define LED3_Pin             LL_GPIO_PIN_1
#define LED3_GPIO_Port       GPIOH
#define SEN_FL_Pin           LL_GPIO_PIN_0
#define SEN_FL_GPIO_Port     GPIOA
#define SEN_SL_Pin           LL_GPIO_PIN_1
#define SEN_SL_GPIO_Port     GPIOA
#define SEN_SR_Pin           LL_GPIO_PIN_2
#define SEN_SR_GPIO_Port     GPIOA
#define SEN_FR_Pin           LL_GPIO_PIN_3
#define SEN_FR_GPIO_Port     GPIOA
#define LED_A2_Pin           LL_GPIO_PIN_4
#define LED_A2_GPIO_Port     GPIOA
#define LED_A1_Pin           LL_GPIO_PIN_5
#define LED_A1_GPIO_Port     GPIOA
#define V_BAT_Pin            LL_GPIO_PIN_0
#define V_BAT_GPIO_Port      GPIOB
#define IMU_CS2_Pin          LL_GPIO_PIN_12
#define IMU_CS2_GPIO_Port    GPIOB
#define IMU_SCK2_Pin         LL_GPIO_PIN_13
#define IMU_SCK2_GPIO_Port   GPIOB
#define IMU_MISO2_Pin        LL_GPIO_PIN_14
#define IMU_MISO2_GPIO_Port  GPIOB
#define IMU_MOSI2_Pin        LL_GPIO_PIN_15
#define IMU_MOSI2_GPIO_Port  GPIOB
#define UART_TX_Pin          LL_GPIO_PIN_9
#define UART_TX_GPIO_Port    GPIOA
#define UART_RX_Pin          LL_GPIO_PIN_10
#define UART_RX_GPIO_Port    GPIOA
#define LED1_Pin             LL_GPIO_PIN_11
#define LED1_GPIO_Port       GPIOA
#define LED0_Pin             LL_GPIO_PIN_12
#define LED0_GPIO_Port       GPIOA
#define SWDIO_Pin            LL_GPIO_PIN_13
#define SWDIO_GPIO_Port      GPIOA
#define SWCLK_Pin            LL_GPIO_PIN_14
#define SWCLK_GPIO_Port      GPIOA
#define ENC_R_PWM1_Pin       LL_GPIO_PIN_15
#define ENC_R_PWM1_GPIO_Port GPIOA
#define ENC_R_PWM2_Pin       LL_GPIO_PIN_3
#define ENC_R_PWM2_GPIO_Port GPIOB
#define ENC_L_PWM1_Pin       LL_GPIO_PIN_4
#define ENC_L_PWM1_GPIO_Port GPIOB
#define ENC_L_PWM2_Pin       LL_GPIO_PIN_5
#define ENC_L_PWM2_GPIO_Port GPIOB
#define MOT_R_PWM1_Pin       LL_GPIO_PIN_6
#define MOT_R_PWM1_GPIO_Port GPIOB
#define MOT_R_PWM2_Pin       LL_GPIO_PIN_7
#define MOT_R_PWM2_GPIO_Port GPIOB
#define MOT_L_PWM1_Pin       LL_GPIO_PIN_8
#define MOT_L_PWM1_GPIO_Port GPIOB
#define MOT_L_PWM2_Pin       LL_GPIO_PIN_9
#define MOT_L_PWM2_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0                                     \
    ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority, \
                                4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1                                     \
    ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority, \
                                3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2                                     \
    ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority, \
                                2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3                                     \
    ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority, \
                                1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4                                     \
    ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority, \
                                0 bit  for subpriority */
#endif
#endif  // ifdef MOUSE_ZIRCONIA2KAI

struct ImuData {
    int16_t OUT_TEMP;
    int16_t OUT_X_G;  // Pitch
    int16_t OUT_Y_G;  // Yaw
    int16_t OUT_Z_G;  // Roll
    int16_t OUT_X_A;  // Right
    int16_t OUT_Y_A;  // Front
    int16_t OUT_Z_A;  // Up
};

struct EncoderData {
    int16_t LEFT;
    int16_t RIGHT;
};

}  // namespace hal
