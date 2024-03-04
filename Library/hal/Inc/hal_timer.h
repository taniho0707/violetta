//******************************************************************************
// @addtogroup HAL
// @file       hal_timer.h
// @brief      TIMを使いTimerを構成する
//******************************************************************************
#pragma once

#include "hal_conf.h"

#ifdef STM32L4P5xx
// STM32HAL/LL
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_tim.h"
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#endif  // ifdef STM32F411xE

#ifdef LINUX
#include <chrono>
#include <cstdint>
#endif

namespace hal {

HalStatus initTimer();

uint32_t getTimerCount();

#ifdef LINUX
std::chrono::system_clock::time_point getTimeByChrono();
#endif

}  // namespace hal
