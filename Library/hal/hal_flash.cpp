//******************************************************************************
// @addtogroup HAL
// @file       hal_flash.cpp
// @brief      内蔵FLASH制御
//******************************************************************************
#include "hal_flash.h"

#if defined(MOUSE_LAZULI) && defined(STM32L4P5xx)
#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_rcc.h"
#endif  // ifdef MOUSE_LAZULI

hal::HalStatus hal::initInternalFlash() {
#ifdef MOUSE_LAZULI
    return HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_ZIRCONIA2KAI
    return HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_ZIRCONIA2KAI

#ifdef LINUX
    return HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::deinitInternalFlash() {
#ifdef STM32L4P5xx
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus writeInternalFlash(uint32_t address, uint32_t data) {
#ifdef STM32L4P5xx
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus writeInternalFlash(uint32_t address, uint32_t* data, uint32_t size) {
#ifdef STM32L4P5xx
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus readInternalFlash(uint32_t address, uint32_t& data) {
#ifdef STM32L4P5xx
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus readInternalFlash(uint32_t address, uint32_t* data, uint32_t size) {
#ifdef STM32L4P5xx
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus eraseInternalFlashSync(hal::InternalFlashSector sector) {
#ifdef STM32L4P5xx
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus eraseInternalFlashAsync(hal::InternalFlashSector sector) {
#ifdef STM32L4P5xx
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus isDoneEraseInternalFlash() {
#ifdef STM32L4P5xx
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}
