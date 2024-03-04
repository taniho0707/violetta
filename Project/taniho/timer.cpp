//******************************************************************************
// @addtogroup TANIHO
// @file       timer.cpp
// @brief      Timer interrupt functions
//******************************************************************************
#include "mpl_timer.h"

// 追加で必要なライブラリ
#include "mpl_led.h"

void mpl::Timer::run1() {}

void mpl::Timer::run2() {}

void mpl::Timer::run3() {
    // mpl::Led::getInstance()->interrupt();
}

void mpl::Timer::run4() {}
