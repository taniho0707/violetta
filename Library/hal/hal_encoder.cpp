//******************************************************************************
// @addtogroup HAL
// @file       hal_encoder.cpp
// @brief      エンコーダ制御
//******************************************************************************

#include "hal_encoder.h"

hal::HalStatus hal::initEncoderPort() {
#ifdef STM32L4P5xx
    return HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::deinitEncoderPort() {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::getEncoderSync(EncoderData& data) {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    if (plt::Observer::getInstance()->getEncoder(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef LINUX
}
