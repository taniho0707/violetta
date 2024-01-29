//******************************************************************************
// @addtogroup HAL
// @file       hal_battery.h
// @brief      バッテリー電圧取得
//******************************************************************************
#pragma once

#include "hal_conf.h"

#ifdef STM32L4P5xx
// STM32HAL/LL
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_adc.h"
#endif  // ifdef STM32L4P5xx

#ifdef LINUX
#include <cstdint>

#include "observer.h"
#endif

namespace hal {

HalStatus initBatteryPort();
HalStatus deinitBatteryPort();

HalStatus getBatteryVoltageSync(float& voltage);
// HalStatus getBatteryVoltageAsync();
// HalStatus getBatteryVoltageDma();

}  // namespace hal
