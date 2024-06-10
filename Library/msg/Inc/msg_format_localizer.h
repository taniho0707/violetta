//******************************************************************************
// @addtogroup  Message
// @file        msg_format_localizer.h
// @brief       Message Format for mll::Localizer
//******************************************************************************
#pragma once

#include "msg_format.h"

namespace msg {

class MsgFormatLocalizer : public MsgFormat {
   public:
    MsgFormatLocalizer();

    void copy(void* target) override;

    void update(void* from) override;

    float accel_translation;
    float accel_rotation;
    float velocity_translation;
    float velocity_rotation;

    // 左下柱中心が (x, y) = (0, 0) [mm]
    float position_x;
    float position_y;

    // 計算用、直線距離 [mm]
    float position_translation;

    // 時計回りに正の [radian]
    float position_theta;
};

}  // namespace msg
