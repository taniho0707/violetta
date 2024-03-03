//******************************************************************************
// @addtogroup MPL
// @file       mpl_motor.cpp
// @brief      モータ制御
//******************************************************************************

#include "mpl_motor.h"

#include "hal_conf.h"
#include "hal_motor.h"
#include "mpl_conf.h"

mpl::Motor::Motor() { hal::initMotorPort(); }

void mpl::Motor::initPort() { hal::initMotorPort(); }

void mpl::Motor::deinitPort() { hal::deinitMotorPort(); }

mpl::MplStatus mpl::Motor::setFloat() {
    if (hal::setMotorFloat() == hal::HalStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

mpl::MplStatus mpl::Motor::setDutyL(float duty) {
    if (hal::setMotorDutyL(duty) == hal::HalStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}
mpl::MplStatus mpl::Motor::setDutyR(float duty) {
    if (hal::setMotorDutyR(duty) == hal::HalStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

mpl::MplStatus mpl::Motor::setDuty(float duty_l, float duty_r) {
    auto statusl = setDutyL(duty_l);
    auto statusr = setDutyR(duty_r);
    if (statusl == MplStatus::SUCCESS && statusr == MplStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

void mpl::Motor::interrupt() {}

mpl::Motor* mpl::Motor::getInstance() {
    static mpl::Motor instance;
    return &instance;
}
