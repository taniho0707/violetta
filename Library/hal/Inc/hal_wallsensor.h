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
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#endif  // ifdef STM32F411xE

#ifdef LINUX
#include <cstdint>

#include "observer.h"
#endif

namespace hal {

HalStatus initWallSensorPort();
HalStatus deinitWallSensorPort();

HalStatus getWallSensorAllSync(uint16_t* data);
// HalStatus getBatteryVoltageAsync();
// HalStatus getBatteryVoltageDma();

}  // namespace hal
