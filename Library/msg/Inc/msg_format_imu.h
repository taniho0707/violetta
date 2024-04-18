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

    int16_t gyro_yaw;
    int16_t gyro_roll;
    int16_t gyro_pitch;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t temperature;
};

}  // namespace msg
