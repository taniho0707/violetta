//******************************************************************************
// @addtogroup  Message
// @file        msg_format_motor_controller.h
// @brief       Message Format for mll::MotorController
//******************************************************************************
#pragma once

#include "msg_format.h"

namespace msg {

class MsgFormatMotorController : public MsgFormat {
   public:
    MsgFormatMotorController();

    void copy(void* target) override;

    void update(void* from) override;

    float velocity_translation;
    float velocity_rotation;

    // 計算用、直線距離 [mm]
    float position_translation;

    // 時計回りに正の [radian]
    float position_theta;

    // 速度制御の有無
    bool is_controlled;
};

}  // namespace msg
