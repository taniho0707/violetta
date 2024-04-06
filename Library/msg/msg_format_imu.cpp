//******************************************************************************
// @addtogroup  Message
// @file        msg_format.cpp
// @brief       Message All Format Implements
//******************************************************************************

#include "msg_format_imu.h"

#include "mpl_timer.h"
#include "msg_format.h"

using namespace msg;

msg::MsgFormatImu::MsgFormatImu() : MsgFormat(ModuleId::IMU) {}

void msg::MsgFormatImu::copy(void* target) {
    auto* t = static_cast<MsgFormatImu*>(target);
    t->moduleid = moduleid;
    t->count = count;
    t->time = time;
    t->gyro_yaw = gyro_yaw;
    t->gyro_roll = gyro_roll;
    t->gyro_pitch = gyro_pitch;
    t->acc_x = acc_x;
    t->acc_y = acc_y;
    t->acc_z = acc_z;
    t->temperature = temperature;
}

void msg::MsgFormatImu::update(void* from) {
    count++;
    time = mpl::Timer::getMicroTime();
    auto* f = static_cast<MsgFormatImu*>(from);
    gyro_yaw = f->gyro_yaw;
    gyro_roll = f->gyro_roll;
    gyro_pitch = f->gyro_pitch;
    acc_x = f->acc_x;
    acc_y = f->acc_y;
    acc_z = f->acc_z;
    temperature = f->temperature;
}
