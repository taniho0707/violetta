//******************************************************************************
// @addtogroup  Message
// @file        msg_format_encoder.h
// @brief       Message Format for Encoder
//******************************************************************************
#pragma once

#include "msg_format.h"

namespace msg {

class MsgFormatEncoder : public MsgFormat {
   public:
    MsgFormatEncoder();

    void copy(void* target) override;

    void update(void* from) override;

    float left;   // [mm]
    float right;  // [mm]
};

}  // namespace msg
