//******************************************************************************
// @addtogroup HAL
// @file       hal_imu.cpp
// @brief      IMU制御
//******************************************************************************

#include "hal_imu.h"

#include <stdint.h>

#include "hal_conf.h"

hal::HalStatus hal::initImuPort() {
    return HalStatus::SUCCESS;
}

hal::HalStatus hal::deinitImuPort() {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::read8bitImuSync(uint8_t address, uint8_t& data) {
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::write8bitImuSync(uint8_t address, uint8_t data) {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::whoamiImu() {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::setImuConfig() {
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::getImuDataSync(ImuData& data) {
    if (plt::Observer::getInstance()->getImuData(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
}
