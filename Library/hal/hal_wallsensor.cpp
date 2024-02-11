//******************************************************************************
// @addtogroup HAL
// @file       hal_wallsensor.cpp
// @brief      壁センサー制御
//******************************************************************************

#include "hal_wallsensor.h"

hal::HalStatus hal::initWallSensorPort() {
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

hal::HalStatus hal::deinitWallSensorPort() {
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

hal::HalStatus hal::getWallSensorAllSync(uint16_t* data) {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    if (plt::Observer::getInstance()->getWallSensorData(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef LINUX
}
