//******************************************************************************
// @addtogroup HAL
// @file       hal_flash.h
// @brief      内蔵FLASH制御
//******************************************************************************
#pragma once

#include "hal_conf.h"

#ifdef STM32L4P5xx
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
#include "stm32f4xx_ll_gpio.h"
#endif  // ifdef STM32F411xE

#ifdef LINUX
#include <cstdint>

#include "observer.h"
#endif

namespace hal {

HalStatus initInternalFlash();
HalStatus deinitInternalFlash();

HalStatus writeInternalFlash(uint32_t address, uint32_t data);
HalStatus writeInternalFlash(uint32_t address, uint32_t* data, uint32_t size);
HalStatus readInternalFlash(uint32_t address, uint32_t& data);
HalStatus readInternalFlash(uint32_t address, uint32_t* data, uint32_t size);

HalStatus eraseInternalFlashSync(InternalFlashSector sector);
HalStatus eraseInternalFlashAsync(InternalFlashSector sector);
HalStatus isDoneEraseInternalFlash();

}  // namespace hal
