//******************************************************************************
// @addtogroup  Message
// @file        msg_format_motor.h
// @brief       Message Format for Motor
//******************************************************************************
#pragma once

#include "msg_format.h"

namespace msg {

class MsgFormatMotor : public MsgFormat {
   public:
    MsgFormatMotor();

    void copy(void* target) override;

    void update(void* from) override;

    float duty_l;
    float duty_r;
    float duty_suction;
};

}  // namespace msg
