//******************************************************************************
// @addtogroup MPL
// @file       mpl_imu.h
// @brief      IMU 6軸センサ制御
//******************************************************************************
#pragma once

// STL
#include <array>

#include "hal_imu.h"

namespace mpl {

struct LedFlickParams {
    uint32_t start_time = 0;
    float freq = 0;
    uint32_t time = 0;
};

class Imu {
   private:
    Imu();

   public:
    void init();
    void deinit();

    bool whoami();

    int16_t readGyroX();
    int16_t readGyroY();
    int16_t readGyroZ();
    int16_t readAccelX();
    int16_t readAccelY();
    int16_t readAccelZ();

    void readAccelFront();
    void readGyroYaw();

    void resetCalibration();

    float getAccelFront();

    // returns [degree/sec] in float
    float getGyroYaw();

    float getTotalAngle();
    void resetTotalAngle();

    void interrupt();

    static Imu* getInstance();
};

}  // namespace mpl
