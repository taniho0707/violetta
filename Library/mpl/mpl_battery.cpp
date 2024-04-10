//******************************************************************************
// @addtogroup MPL
// @file       mpl_battery.cpp
// @brief      バッテリー電圧制御
//******************************************************************************
#include "mpl_battery.h"

#include "msg_format_battery.h"
#include "msg_server.h"

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

void mpl::Battery::interruptPeriodic() {
    static auto server = msg::MessageServer::getInstance();
    scanSync(last);

    msg_format.battery = last;
    server->sendMessage(msg::ModuleId::BATTERY, &msg_format);
}

mpl::Battery* mpl::Battery::getInstance() {
    static mpl::Battery instance;
    return &instance;
}
