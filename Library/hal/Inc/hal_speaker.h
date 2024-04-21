//******************************************************************************
// @addtogroup HAL
// @file       hal_speaker.h
// @brief      スピーカー制御
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

HalStatus initSpeakerPort();
HalStatus deinitSpeakerPort();

HalStatus setSpeakerFrequency(uint16_t freq);
HalStatus offSpeaker();

}  // namespace hal
