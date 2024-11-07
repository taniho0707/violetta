//******************************************************************************
// @addtogroup  Message
// @file        msg_format_motor_controller_internal.cpp
// @brief       Message Format for MotorController Internal Status
//              This file is temporary and to be replaced with msg_format_localizer.h
//******************************************************************************

#include "msg_format_motor_controller_internal.h"

#include "mpl_timer.h"
#include "msg_format.h"

using namespace msg;

msg::MsgFormatMotorControllerInternal::MsgFormatMotorControllerInternal() : MsgFormat(ModuleId::MOTORCONTROLLER_INTERNAL) {}

void MsgFormatMotorControllerInternal::copy(void* target) {
    auto* t = static_cast<MsgFormatMotorControllerInternal*>(target);
    t->moduleid = moduleid;
    t->count = count;
    t->time = time;
    t->integral_translation = integral_translation;
    t->integral_rotation = integral_rotation;
}

void MsgFormatMotorControllerInternal::update(void* from) {
    count++;
    time = mpl::Timer::getMicroTime();
    auto* f = static_cast<MsgFormatMotorControllerInternal*>(from);
    moduleid = f->moduleid;
    count = f->count;
    time = f->time;
    integral_translation = f->integral_translation;
    integral_rotation = f->integral_rotation;
}
