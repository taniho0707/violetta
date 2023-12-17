//******************************************************************************
// @addtogroup MPL
// @file       mpl_imu.cpp
// @brief      IMU制御
//******************************************************************************
#include "mpl_imu.h"

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

void mpl::Imu::interruptPeriodic() {}

mpl::Imu* mpl::Imu::getInstance() {
    static mpl::Imu instance;
    return &instance;
}
