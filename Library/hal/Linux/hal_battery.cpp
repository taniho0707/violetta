//******************************************************************************
// @addtogroup HAL
// @file       hal_battery.cpp
// @brief      バッテリー電圧制御
//******************************************************************************

#include "hal_battery.h"

hal::HalStatus hal::initBatteryPort() {
    return HalStatus::SUCCESS;
}

hal::HalStatus hal::deinitBatteryPort() {
    return HalStatus::SUCCESS;
}

hal::HalStatus hal::getBatteryVoltageSync(float& voltage) {
    if (plt::Observer::getInstance()->getBatteryVoltage(voltage)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
}
