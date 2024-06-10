//******************************************************************************
// @addtogroup  Message
// @file        msg_format.cpp
// @brief       Message All Format Implements
//******************************************************************************

#include "msg_format_localizer.h"

#include "mpl_timer.h"
#include "msg_format.h"

using namespace msg;

msg::MsgFormatLocalizer::MsgFormatLocalizer()
    : MsgFormat(ModuleId::LOCALIZER) {}

void msg::MsgFormatLocalizer::copy(void* target) {
    auto* t = static_cast<MsgFormatLocalizer*>(target);
    t->moduleid = moduleid;
    t->count = count;
    t->time = time;
    t->accel_translation = accel_translation;
    t->accel_rotation = accel_rotation;
    t->velocity_translation = velocity_translation;
    t->velocity_rotation = velocity_rotation;
    t->position_x = position_x;
    t->position_y = position_y;
    t->position_translation = position_translation;
    t->position_theta = position_theta;
}

void msg::MsgFormatLocalizer::update(void* from) {
    count++;
    time = mpl::Timer::getMicroTime();
    auto* f = static_cast<MsgFormatLocalizer*>(from);
    accel_translation = f->accel_translation;
    accel_rotation = f->accel_rotation;
    velocity_translation = f->velocity_translation;
    velocity_rotation = f->velocity_rotation;
    position_x = f->position_x;
    position_y = f->position_y;
    position_translation = f->position_translation;
    position_theta = f->position_theta;
}
