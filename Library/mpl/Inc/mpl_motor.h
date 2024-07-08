//******************************************************************************
// @addtogroup MPL
// @file       mpl_motor.h
// @brief      モータ制御
//******************************************************************************
#pragma once

#include "mpl_conf.h"
#include "msg_format_motor.h"
#include "msg_format_motor_current.h"
#include "msg_server.h"

namespace mpl {

class Motor {
   private:
    Motor();

    float last_current_l;
    float last_current_r;
    msg::MsgFormatMotorCurrent msg_motorcurrent;
    msg::MsgFormatMotor msg_motor;

   public:
    void initPort();
    void deinitPort();

    mpl::MplStatus setFloat();

    mpl::MplStatus setDutyL(float duty);
    mpl::MplStatus setDutyR(float duty);
    mpl::MplStatus setDuty(float duty_l, float duty_r);

    mpl::MplStatus setDutySuction(float duty);

    mpl::MplStatus getCurrentSync(float& current_l, float& current_r);

    void interruptPeriodic();

    static Motor* getInstance();
};

}  // namespace mpl
