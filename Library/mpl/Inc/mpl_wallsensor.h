//******************************************************************************
// @addtogroup MPL
// @file       mpl_wallsensor.h
// @brief      壁センサー制御
//******************************************************************************
#pragma once

// STL
// #include <array>

#include "hal_conf.h"
#include "mpl_conf.h"
#include "msg_format_wallsensor.h"
#include "msg_server.h"
#include "params.h"

namespace mpl {

class WallSensor {
   private:
    WallSensor();

    hal::WallSensorData last;

    msg::MessageServer* msg_server;
    msg::MsgFormatWallsensor msg_format;

    misc::MouseParams* params;

   public:
    void initPort();
    void deinitPort();

    mpl::MplStatus scanAllSync(hal::WallSensorData& data);

    void interruptPeriodic();

    static WallSensor* getInstance();
};

}  // namespace mpl
