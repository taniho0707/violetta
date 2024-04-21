//******************************************************************************
// @addtogroup  Message
// @file        msg_format_battery.cpp
// @brief       Message All Format Implements
//******************************************************************************

#include "msg_format_battery.h"

#include "mpl_timer.h"
#include "msg_format.h"

using namespace msg;

msg::MsgFormatBattery::MsgFormatBattery() : MsgFormat(ModuleId::BATTERY) {}

void MsgFormatBattery::copy(void* target) {
    auto* t = static_cast<MsgFormatBattery*>(target);
    t->moduleid = moduleid;
    t->count = count;
    t->time = time;
    t->battery = battery;
}

void MsgFormatBattery::update(void* from) {
    count++;
    time = mpl::Timer::getMicroTime();
    auto* f = static_cast<MsgFormatBattery*>(from);
    moduleid = f->moduleid;
    count = f->count;
    time = f->time;
    battery = f->battery;
}
