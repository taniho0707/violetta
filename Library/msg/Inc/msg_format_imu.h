//******************************************************************************
// @addtogroup  Message
// @file        msg_format_imu.h
// @brief       Message Format for IMU
//******************************************************************************
#pragma once

#include "msg_format.h"

namespace msg {

class MsgFormatImu : public MsgFormat {
   public:
    MsgFormatImu();

    void copy(void* target) override;

    void update(void* from) override;

    float gyro_yaw;
    float gyro_roll;
    float gyro_pitch;
    float acc_x;
    float acc_y;
    float acc_z;
    float temperature;
};

}  // namespace msg
