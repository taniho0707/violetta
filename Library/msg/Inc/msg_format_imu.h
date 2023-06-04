//******************************************************************************
// @addtogroup  Message
// @file        msg_format_imu.h
// @brief       Message Format for IMU
//******************************************************************************
#include "msg_format.h"

struct MsgFormatImu : MsgFormat {
    float gyro_yaw;
    float gyro_roll;
    float gyro_pitch;
    float acc_x;
    float acc_y;
    float acc_z;
    float temperature;
};
