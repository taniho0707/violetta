//******************************************************************************
// @addtogroup MPL
// @file       mpl_wallsensor.h
// @brief      壁センサー制御
//******************************************************************************
#pragma once

// STL
#include <array>

#include "hal_wallsensor.h"
#include "mpl_conf.h"

namespace mpl {

class WallSensor {
   private:
    WallSensor();

   public:
    void initPort();
    void deinitPort();

    mpl::MplStatus scanAllSync(hal::WallSensorData& data);

    void interrupt();

    static WallSensor* getInstance();
};

}  // namespace mpl
