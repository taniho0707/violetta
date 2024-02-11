//******************************************************************************
// @addtogroup HAL
// @file       hal_led.h
// @brief      LED点灯制御
//******************************************************************************
#pragma once

#include "hal_conf.h"

#ifdef STM32L4P5xx
// STM32HAL/LL
#include "stm32l4xx_hal.h"
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

HalStatus initLedPort(LedNumbers num);
HalStatus deinitLedPort(LedNumbers num);

HalStatus onLed(LedNumbers num);
HalStatus offLed(LedNumbers num);

}  // namespace hal
