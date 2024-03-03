//******************************************************************************
// @addtogroup MPL
// @file       mpl_battery.cpp
// @brief      バッテリー電圧制御
//******************************************************************************
#include "mpl_battery.h"

mpl::Battery::Battery() {
    // hal::initBatteryPort();
    // FIXME: コンストラクタ内でポートの初期化をするかどうか決定
}

mpl::MplStatus mpl::Battery::initPort() {
    auto status = hal::initBatteryPort();
    if (status == hal::HalStatus::SUCCESS) {
        return mpl::MplStatus::SUCCESS;
    } else {
        return mpl::MplStatus::ERROR;
    }
}

void mpl::Battery::deinitPort() { hal::deinitBatteryPort(); }

mpl::MplStatus mpl::Battery::scanSync(float& voltage) {
    if (hal::getBatteryVoltageSync(voltage) == hal::HalStatus::SUCCESS) {
        return mpl::MplStatus::SUCCESS;
    } else {
        return mpl::MplStatus::ERROR;
    }
}

void mpl::Battery::interrupt() {}

mpl::Battery* mpl::Battery::getInstance() {
    static mpl::Battery instance;
    return &instance;
}
