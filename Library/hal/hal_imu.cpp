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

#ifdef LINUX
    return HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::deinitImuPort() {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::whoamiImu() {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::getImuDataSync(ImuData& data) {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef LINUX
    data = ImuData{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}
