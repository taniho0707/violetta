//******************************************************************************
// @addtogroup  Message
// @file        msg_format.cpp
// @brief       Message All Format Implements
//******************************************************************************

#include "msg_format.h"

using namespace msg;

// Base
msg::MsgFormat::MsgFormat(ModuleId id) : moduleid(id), count(0), time(0) {}

// void msg::MsgFormat::update(void* from) {
//     count++;
//     time = mpl::Timer::getMicroTime();
// }
