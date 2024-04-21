//******************************************************************************
// @addtogroup  Message
// @file        msg_format.cpp
// @brief       Message All Format Implements
//******************************************************************************

#include "msg_format_encoder.h"

#include "mpl_timer.h"
#include "msg_format.h"

using namespace msg;

msg::MsgFormatEncoder::MsgFormatEncoder() : MsgFormat(ModuleId::ENCODER) {}

void MsgFormatEncoder::copy(void* target) {
    auto* t = static_cast<MsgFormatEncoder*>(target);
    t->moduleid = moduleid;
    t->count = count;
    t->time = time;
    t->left = left;
    t->right = right;
}

void MsgFormatEncoder::update(void* from) {
    count++;
    time = mpl::Timer::getMicroTime();
    auto* f = static_cast<MsgFormatEncoder*>(from);
    moduleid = f->moduleid;
    count = f->count;
    time = f->time;
    left = f->left;
    right = f->right;
}
