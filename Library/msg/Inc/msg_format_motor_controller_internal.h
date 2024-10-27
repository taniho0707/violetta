//******************************************************************************
// @addtogroup  Message
// @file        msg_format_motor_controller_internal.h
// @brief       Message Format for MotorController Internal Status
//              This file is temporary and to be replaced with msg_format_localizer.h
//******************************************************************************
#pragma once

#include "msg_format.h"

namespace msg {

class MsgFormatMotorControllerInternal : public MsgFormat {
   public:
    MsgFormatMotorControllerInternal();

    void copy(void* target) override;

    void update(void* from) override;

    float integral_translation;
    float integral_rotation;
};

}  // namespace msg
