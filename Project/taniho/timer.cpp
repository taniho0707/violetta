//******************************************************************************
// @addtogroup TANIHO
// @file       timer.cpp
// @brief      Timer interrupt functions
//******************************************************************************
#include "mpl_timer.h"

// 追加で必要なライブラリ
#include "mpl_imu.h"
#include "mpl_led.h"

void mpl::Timer::run1() {
    static auto imu = mpl::Imu::getInstance();
    imu->interruptPeriodic();
}

void mpl::Timer::run2() {}

void mpl::Timer::run3() {
    // mpl::Led::getInstance()->interrupt();
}

void mpl::Timer::run4() {}
