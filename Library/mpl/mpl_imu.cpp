//******************************************************************************
// @addtogroup MPL
// @file       mpl_imu.cpp
// @brief      IMU制御
//******************************************************************************
#include "mpl_imu.h"

#include "mpl_timer.h"
#include "msg_format_imu.h"
#include "msg_server.h"
#include "params.h"

mpl::Imu::Imu() {
    init();
}

mpl::MplStatus mpl::Imu::init() {
    auto status = hal::initImuPort();
    if (status == hal::HalStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

void mpl::Imu::deinit() {
    hal::deinitImuPort();
}

mpl::MplStatus mpl::Imu::whoami() {
    auto status = hal::whoamiImu();
    if (status == hal::HalStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

mpl::MplStatus mpl::Imu::setConfig() {
    auto status = hal::setImuConfig();
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

hal::ImuData mpl::Imu::data() {
    return last;
}

void mpl::Imu::interruptPeriodic() {
    static auto server = msg::MessageServer::getInstance();

    static auto params_cache = misc::Params::getInstance()->getCachePointer();

    // update imu data
    // TODO: DMA対応
    scanAllSync(last);

    msg_format.gyro_yaw = params_cache->imu_sensitivity_gyro * last.OUT_Z_G;
    msg_format.gyro_roll = params_cache->imu_sensitivity_gyro * last.OUT_Y_G;
    msg_format.gyro_pitch = params_cache->imu_sensitivity_gyro * last.OUT_X_G;
    msg_format.acc_x = params_cache->imu_sensitivity_acceleration * last.OUT_X_A;
    msg_format.acc_y = params_cache->imu_sensitivity_acceleration * last.OUT_Y_A;
    msg_format.acc_z = params_cache->imu_sensitivity_acceleration * last.OUT_Z_A;
    msg_format.temperature = last.OUT_TEMP;
    // end update imu data

    server->sendMessage(msg::ModuleId::IMU, &msg_format);
}

mpl::Imu* mpl::Imu::getInstance() {
    static mpl::Imu instance;
    return &instance;
}
