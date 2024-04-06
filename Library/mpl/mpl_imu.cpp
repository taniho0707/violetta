//******************************************************************************
// @addtogroup MPL
// @file       mpl_imu.cpp
// @brief      IMU制御
//******************************************************************************
#include "mpl_imu.h"

#include "mpl_timer.h"
#include "msg_format_imu.h"
#include "msg_server.h"

mpl::Imu::Imu() { init(); }

mpl::MplStatus mpl::Imu::init() {
    auto status = hal::initImuPort();
    if (status == hal::HalStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

void mpl::Imu::deinit() { hal::deinitImuPort(); }

mpl::MplStatus mpl::Imu::whoami() {
    auto status = hal::whoamiImu();
    if (status == hal::HalStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

mpl::MplStatus mpl::Imu::scanAllSync(hal::ImuData& data) {
    auto status = hal::getImuDataSync(data);
    if (status == hal::HalStatus::SUCCESS) {
        last = data;
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

hal::ImuData mpl::Imu::data() { return last; }

void mpl::Imu::interruptPeriodic() {
    static auto server = msg::MessageServer::getInstance();
    // update imu data
    scanAllSync(last);

    msg_format.gyro_yaw = (float)last.OUT_Z_G;
    msg_format.gyro_roll = (float)last.OUT_Y_G;
    msg_format.gyro_pitch = (float)last.OUT_X_G;
    msg_format.acc_x = (float)last.OUT_X_A;
    msg_format.acc_y = (float)last.OUT_Y_A;
    msg_format.acc_z = (float)last.OUT_Z_A;
    msg_format.temperature = (float)last.OUT_TEMP;
    // end update imu data
    server->sendMessage(msg::ModuleId::IMU, &msg_format);
}

mpl::Imu* mpl::Imu::getInstance() {
    static mpl::Imu instance;
    return &instance;
}
