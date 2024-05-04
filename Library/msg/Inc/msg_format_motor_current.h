//******************************************************************************
// @addtogroup  Message
// @file        msg_format_motor_current.h
// @brief       Message Format for Motor Current
//******************************************************************************
#pragma once

#include "msg_format.h"

namespace msg {

class MsgFormatMotorCurrent : public MsgFormat {
   public:
    MsgFormatMotorCurrent();

    void copy(void* target) override;

    void update(void* from) override;

    float current_l;
    float current_r;
};

}  // namespace msg
