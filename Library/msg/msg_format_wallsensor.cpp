//******************************************************************************
// @addtogroup  Message
// @file        msg_format.cpp
// @brief       Message All Format Implements
//******************************************************************************

#include "msg_format_wallsensor.h"

#include "mpl_timer.h"
#include "msg_format.h"

using namespace msg;

msg::MsgFormatWallsensor::MsgFormatWallsensor()
    : MsgFormat(ModuleId::WALLSENSOR) {}

void MsgFormatWallsensor::copy(void* target) {
    auto* t = static_cast<MsgFormatWallsensor*>(target);
    t->moduleid = moduleid;
    t->count = count;
    t->time = time;
    t->frontleft = frontleft;
    t->left = left;
    t->right = right;
    t->frontright = frontright;
}

void MsgFormatWallsensor::update(void* from) {
    count++;
    time = mpl::Timer::getMicroTime();
    auto* f = static_cast<MsgFormatWallsensor*>(from);
    moduleid = f->moduleid;
    count = f->count;
    time = f->time;
    frontleft = f->frontleft;
    left = f->left;
    right = f->right;
    frontright = f->frontright;
}