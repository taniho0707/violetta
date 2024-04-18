//******************************************************************************
// @addtogroup HAL
// @file       hal_motor.h
// @brief      モータ制御
//******************************************************************************
#pragma once

#include "hal_conf.h"

#ifdef STM32L4P5xx
#include "stm32l4xx_ll_adc.h"
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#endif  // ifdef STM32F411xE

#ifdef LINUX
#include <cstdint>

#include "observer.h"
#endif

namespace hal {

HalStatus initMotorPort();
HalStatus deinitMotorPort();

HalStatus setMotorDutyL(float duty);
HalStatus setMotorDutyR(float duty);

HalStatus setMotorFloat();

}  // namespace hal
