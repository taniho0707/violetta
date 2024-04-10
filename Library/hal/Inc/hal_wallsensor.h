//******************************************************************************
// @addtogroup HAL
// @file       hal_wallsensor.h
// @brief      壁センサー制御
//******************************************************************************
#pragma once

#include "hal_conf.h"

#ifdef STM32L4P5xx
// STM32HAL/LL
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_adc.h"
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
#include "stm32f411xe.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_utils.h"
#endif  // ifdef STM32F411xE

#ifdef LINUX
#include <cstdint>

#include "observer.h"
#endif

namespace hal {

HalStatus initWallSensorPort();
HalStatus deinitWallSensorPort();

HalStatus setWallSensorLedOn(WallSensorNumbers n);
HalStatus setWallSensorLedOff();

HalStatus getWallSensorSingleSync(uint16_t& data, WallSensorNumbers n);

// HalStatus getWallSensorAllSync(WallSensorData& data);
// HalStatus getBatteryVoltageAsync();
// HalStatus getBatteryVoltageDma();

}  // namespace hal
