//******************************************************************************
// @addtogroup  Message
// @file        msg_format_battery.h
// @brief       Message Format for Battery
//******************************************************************************
#pragma once

#include "msg_format.h"

namespace msg {

class MsgFormatBattery : public MsgFormat {
   public:
    MsgFormatBattery();

    void copy(void* target) override;

    void update(void* from) override;

    float battery;
};

}  // namespace msg
