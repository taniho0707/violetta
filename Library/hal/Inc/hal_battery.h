//******************************************************************************
// @addtogroup HAL
// @file       hal_battery.h
// @brief      バッテリー電圧取得
//******************************************************************************
#pragma once

#include "hal_conf.h"

#ifdef STM32L4P5xx
#include "stm32l4xx_ll_adc.h"
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#endif  // ifdef STM32F411xE

#ifdef LINUX
#include <cstdint>

#include "observer.h"
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA

#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_ZIRCONIA2KAI

#endif  // ifdef MOUSE_ZIRCONIA2KAI

namespace hal {

HalStatus initBatteryPort();
HalStatus deinitBatteryPort();

HalStatus getBatteryVoltageSync(float& voltage);
// HalStatus getBatteryVoltageAsync();
// HalStatus getBatteryVoltageDma();

}  // namespace hal
