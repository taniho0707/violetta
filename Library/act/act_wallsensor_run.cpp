//******************************************************************************
// @addtogroup ACT
// @file       act_wallsensor_run.cpp
// @brief      Wallsensor Run Activity for LazuliSensor
//******************************************************************************
#include "act_wallsensor_run.h"

#include "hal_wallsensor.h"
#include "mll_wall_analyser.h"
#include "mpl_wallsensor.h"
#include "msg_format_wall_analyser.h"
#include "msg_server.h"
#include "params.h"

using namespace act;

void WallsensorRunActivity::init(ActivityParameters &params) {}

#ifdef MOUSE_LAZULI_SENSOR
Status WallsensorRunActivity::run() {
    auto message = msg::MessageServer::getInstance();
    msg::MsgFormatWallsensor msg_wallsensor = msg::MsgFormatWallsensor();
    msg::MsgFormatWallAnalyser msg_wallanalyser = msg::MsgFormatWallAnalyser();

    auto wallsensor = mpl::WallSensor::getInstance();
    wallsensor->initPort();

    auto wallanalyser = mll::WallAnalyser::getInstance();
    wallanalyser->init();

    static auto params_cache = misc::Params::getInstance()->getCachePointer();
    misc::Params::getInstance()->load(misc::ParameterDestinationType::HARDCODED);
    // misc::Params::getInstance()->loadSlalom(misc::ParameterDestinationType::HARDCODED);

    // mpl::Timer::init();

    // for Debug
    // uint32_t count = 0;

    // 【壁センサの値を取るための無限ループ】
    while (1) {
        wallsensor->interruptPeriodic();
        wallanalyser->interruptPeriodic();
        // for (int i = 0; i < 1000; i++);

        // for Debug
        // ++count;
    }

    return Status::ERROR;
}
#else
Status WallsensorRunActivity::run() {
    return Status::ERROR;
}
#endif  // MOUSE_LAZULI_SENSOR

void WallsensorRunActivity::finalize(ActivityParameters &params) {}
