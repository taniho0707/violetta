//******************************************************************************
// @addtogroup HAL
// @file       hal_led.h
// @brief      LED点灯制御
//******************************************************************************
#pragma once

#include "hal_conf.h"

#ifdef STM32L4P5xx
// STM32LL
#include "stm32l4xx_ll_gpio.h"
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#endif  // ifdef STM32F411xE

#ifdef LINUX
#include <cstdint>
#endif

namespace hal {

#ifdef MOUSE_LAZULI
enum class LedDriverCommands : uint8_t {
    CHIP_ID = 0x00,
    MONITOR = 0x01,
    LED_CONFIG = 0x02,
    LED_EN_IDVD = 0x03,
    LED_GBAL_SET = 0x04,
    LED1_SET = 0x05,
    LED2_SET = 0x06,
    LED3_SET = 0x07,
    LED4_SET = 0x08,
    LED5_SET = 0x09,
    LED6_SET = 0x0A,
    LED7_SET = 0x0B,
    LED8_SET = 0x0C,
    LED9_SET = 0x0D,
    FAULT_STAT1 = 0x0E,
    FAULT_STAT2 = 0x0F,
    FAULT_CLEAR = 0x10,
    BUCK_CONFIG = 0x11,
    STATUS = 0x13,
};

const uint8_t LED_DRIVER_ADDR = 0x40;
#else
enum class LedDriverCommands : uint8_t {
    UNDEFINED = 0x00,
}

const uint8_t LED_DRIVER_ADDR = 0x00;
#endif  // ifdef MOUSE_LAZULI

HalStatus initLedPort(LedNumbers num);
HalStatus deinitLedPort(LedNumbers num);

HalStatus transmitLedI2cData(uint8_t device_id, uint8_t* pdata, uint8_t size);
HalStatus receiveLedI2cData(uint8_t device_id, uint8_t* pdata, uint8_t size);
HalStatus transmitLedI2cCommand(LedDriverCommands cmd, uint8_t data);
HalStatus receiveLedI2cCommand(LedDriverCommands cmd, uint8_t* data);

HalStatus onLed(LedNumbers num);
HalStatus offLed(LedNumbers num);

}  // namespace hal
