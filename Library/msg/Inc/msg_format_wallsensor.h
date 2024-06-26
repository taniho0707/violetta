//******************************************************************************
// @addtogroup  Message
// @file        msg_format_wallsensor.h
// @brief       Message Format for WallSensor
//******************************************************************************
#pragma once

#include "msg_format.h"

namespace msg {

class MsgFormatWallsensor : public MsgFormat {
   public:
    MsgFormatWallsensor();

    void copy(void* target) override;

    void update(void* from) override;

    uint16_t frontleft;
    uint16_t left;
    uint16_t right;
    uint16_t frontright;
};

}  // namespace msg
