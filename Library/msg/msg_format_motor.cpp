//******************************************************************************
// @addtogroup  Message
// @file        msg_format_motor.cpp
// @brief       Message All Format Implements
//******************************************************************************

#include "msg_format_motor.h"

#include "mpl_timer.h"
#include "msg_format.h"

using namespace msg;

msg::MsgFormatMotor::MsgFormatMotor()
    : MsgFormat(ModuleId::BATTERY) {}

void MsgFormatMotor::copy(void* target) {
    auto* t = static_cast<MsgFormatMotor*>(target);
    t->moduleid = moduleid;
    t->count = count;
    t->time = time;
    t->duty_l = duty_l;
    t->duty_r = duty_r;
    t->duty_suction = duty_suction;
}

void MsgFormatMotor::update(void* from) {
    count++;
    time = mpl::Timer::getMicroTime();
    auto* f = static_cast<MsgFormatMotor*>(from);
    moduleid = f->moduleid;
    count = f->count;
    time = f->time;
    duty_l = f->duty_l;
    duty_r = f->duty_r;
    duty_suction = f->duty_suction;
}
