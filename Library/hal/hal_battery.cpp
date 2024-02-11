//******************************************************************************
// @addtogroup HAL
// @file       hal_battery.cpp
// @brief      バッテリー電圧制御
//******************************************************************************

#include "hal_battery.h"

hal::HalStatus hal::initBatteryPort() {
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

hal::HalStatus hal::deinitBatteryPort() {
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

hal::HalStatus hal::getBatteryVoltageSync(float& voltage) {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    if (plt::Observer::getInstance()->getBatteryVoltage(voltage)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef LINUX
}
