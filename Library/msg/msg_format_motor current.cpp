//******************************************************************************
// @addtogroup  Message
// @file        msg_format_motor_current.cpp
// @brief       Message All Format Implements
//******************************************************************************

#include "mpl_timer.h"
#include "msg_format.h"
#include "msg_format_motor_current.h"

using namespace msg;

msg::MsgFormatMotorCurrent::MsgFormatMotorCurrent()
    : MsgFormat(ModuleId::BATTERY) {}

void MsgFormatMotorCurrent::copy(void* target) {
    auto* t = static_cast<MsgFormatMotorCurrent*>(target);
    t->moduleid = moduleid;
    t->count = count;
    t->time = time;
    t->current_l = current_l;
    t->current_r = current_r;
}

void MsgFormatMotorCurrent::update(void* from) {
    count++;
    time = mpl::Timer::getMicroTime();
    auto* f = static_cast<MsgFormatMotorCurrent*>(from);
    moduleid = f->moduleid;
    count = f->count;
    time = f->time;
    current_l = f->current_l;
    current_r = f->current_r;
}
