//******************************************************************************
// @addtogroup  Message
// @file        msg_format_motor_controller.cpp
// @brief       Message All Format Implements
//******************************************************************************

#include "msg_format_motor_controller.h"

#include "mpl_timer.h"
#include "msg_format.h"

using namespace msg;

msg::MsgFormatMotorController::MsgFormatMotorController() : MsgFormat(ModuleId::MOTORCONTROLLER) {}

void msg::MsgFormatMotorController::copy(void* target) {
    auto* t = static_cast<MsgFormatMotorController*>(target);
    t->moduleid = moduleid;
    t->count = count;
    t->time = time;
    t->velocity_translation = velocity_translation;
    t->velocity_rotation = velocity_rotation;
    t->position_translation = position_translation;
    t->position_theta = position_theta;
    t->is_controlled = is_controlled;
}

void msg::MsgFormatMotorController::update(void* from) {
    count++;
    time = mpl::Timer::getMicroTime();
    auto* f = static_cast<MsgFormatMotorController*>(from);
    velocity_translation = f->velocity_translation;
    velocity_rotation = f->velocity_rotation;
    position_translation = f->position_translation;
    position_theta = f->position_theta;
    is_controlled = f->is_controlled;
}
