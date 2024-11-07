//******************************************************************************
// @addtogroup MPL
// @file       mpl_imu.h
// @brief      IMU 6軸センサ制御
//******************************************************************************
#pragma once

#include "hal_conf.h"
#include "mpl_conf.h"
#include "msg_format_imu.h"

namespace mpl {

class Imu {
   private:
    Imu();

    hal::ImuData last;

    msg::MsgFormatImu msg_format;

   public:
    MplStatus init();
    void deinit();

    MplStatus whoami();
    MplStatus setConfig();

    MplStatus scanAllSync(hal::ImuData& data);
    // MplStatus scanAllAsync();
    // MplStatus doneScanAsync();
    // MplStatus scanAllDma();
    // MplStatus doneScanDma();

    hal::ImuData data();

    // int16_t readGyroX();
    // int16_t readGyroY();
    // int16_t readGyroZ();
    // int16_t readAccelX();
    // int16_t readAccelY();
    // int16_t readAccelZ();

    // void readAccelFront();
    // void readGyroYaw();

    // void resetCalibration();

    // float getAccelFront();

    // // returns [degree/sec] in float
    // float getGyroYaw();

    // float getTotalAngle();
    // void resetTotalAngle();

    void interruptPeriodic();
    // void interruptAsync();
    // void interruptDma();

    static Imu* getInstance();
};

}  // namespace mpl
