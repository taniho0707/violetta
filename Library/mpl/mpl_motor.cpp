//******************************************************************************
// @addtogroup MPL
// @file       mpl_motor.cpp
// @brief      モータ制御
//******************************************************************************

#include "mpl_motor.h"

#include "hal_conf.h"
#include "hal_motor.h"
#include "mpl_conf.h"
#include "msg_format.h"
#include "msg_server.h"

mpl::Motor::Motor() {
    hal::initMotorPort();
}

void mpl::Motor::initPort() {
    hal::initMotorPort();
}

void mpl::Motor::deinitPort() {
    hal::deinitMotorPort();
}

mpl::MplStatus mpl::Motor::setFloat() {
    if (hal::setMotorFloat() == hal::HalStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

mpl::MplStatus mpl::Motor::setDutyL(float duty) {
#pragma message "!!! MOTOR DUTY LIMITTER ENABLED !!!"
    if (duty > 0.3) duty = 0.3;
    if (duty < -0.3) duty = -0.3;

    if (hal::setMotorDutyL(duty) == hal::HalStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}
mpl::MplStatus mpl::Motor::setDutyR(float duty) {
#pragma message "!!! MOTOR DUTY LIMITTER ENABLED !!!"
    if (duty > 0.3) duty = 0.3;
    if (duty < -0.3) duty = -0.3;

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

mpl::MplStatus mpl::Motor::setDutySuction(float duty) {
    if (hal::setMotorDutySuction(duty) == hal::HalStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

mpl::MplStatus mpl::Motor::getCurrentSync(float& current_l, float& current_r) {
    if (hal::getMotorCurrentSync(current_l, current_r) == hal::HalStatus::SUCCESS) {
        return mpl::MplStatus::SUCCESS;
    } else {
        return mpl::MplStatus::ERROR;
    }
}

void mpl::Motor::interruptPeriodic() {
    static auto server = msg::MessageServer::getInstance();
    getCurrentSync(last_current_l, last_current_r);

    msg_motorcurrent.current_l = last_current_l;
    msg_motorcurrent.current_r = last_current_r;
    server->sendMessage(msg::ModuleId::MOTORCURRENT, &msg_motorcurrent);

    server->receiveMessage(msg::ModuleId::MOTOR, &msg_motor);
    setDuty(msg_motor.duty_l, msg_motor.duty_r);
    setDutySuction(msg_motor.duty_suction);
}

mpl::Motor* mpl::Motor::getInstance() {
    static mpl::Motor instance;
    return &instance;
}
