//******************************************************************************
// @addtogroup HAL
// @file       hal_motor.cpp
// @brief      モータ制御
//******************************************************************************

#include "hal_motor.h"

hal::HalStatus hal::initMotorPort() {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::deinitMotorPort() {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::setMotorDutyR(float duty) {
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::setMotorDutyL(float duty) {
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::setMotorFloat() {
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::setMotorDutySuction(float duty) {
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::getMotorCurrentSync(float& current_l, float& current_r) {
    return hal::HalStatus::NOIMPLEMENT;
}
