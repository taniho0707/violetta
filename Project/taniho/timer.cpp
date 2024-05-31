//******************************************************************************
// @addtogroup TANIHO
// @file       timer.cpp
// @brief      Timer interrupt functions
//******************************************************************************
#include "mpl_timer.h"

// 追加で必要なライブラリ : MPL
#include "mpl_battery.h"
#include "mpl_debug.h"
#include "mpl_encoder.h"
#include "mpl_imu.h"
#include "mpl_led.h"
#include "mpl_speaker.h"
#include "mpl_wallsensor.h"

// 追加で必要なライブラリ : MLL
#include "mll_logger.h"

void mpl::Timer::run1() {
    static auto imu = mpl::Imu::getInstance();
    imu->interruptPeriodic();

    static auto encoder = mpl::Encoder::getInstance();
    encoder->interruptPeriodic();
}

void mpl::Timer::run2() {
    static auto wallsensor = mpl::WallSensor::getInstance();
    wallsensor->interruptPeriodic();
}

void mpl::Timer::run3() {
    // mpl::Led::getInstance()->interrupt();

#ifdef MOUSE_LAZULI
    static auto speaker = mpl::Speaker::getInstance();
    speaker->interruptPeriodic();
#endif  // MOUSE_LAZULI

    static auto battery = mpl::Battery::getInstance();
    battery->interruptPeriodic();
}

void mpl::Timer::run4() {
    static auto logger = mll::Logger::getInstance();
    logger->interruptPeriodic();

    static auto debug = mpl::Debug::getInstance();
    debug->interruptPeriodic();
}
