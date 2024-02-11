//******************************************************************************
// @addtogroup HAL
// @file       hal_imu.cpp
// @brief      IMU制御
//******************************************************************************

#include "hal_imu.h"

hal::HalStatus hal::initImuPort() {
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

hal::HalStatus hal::deinitImuPort() {
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

hal::HalStatus hal::whoamiImu() {
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

hal::HalStatus hal::getImuDataSync(ImuData& data) {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    if (plt::Observer::getInstance()->getImuData(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef LINUX
}
