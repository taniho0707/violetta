//******************************************************************************
// @addtogroup TANIHO
// @file       timer.cpp
// @brief      Timer interrupt functions
//******************************************************************************
#include "mpl_timer.h"

// 追加で必要なライブラリ
#include "mpl_led.h"

void Timer::run1() {}

void Timer::run2() {}

void Timer::run3() { Led::getInstance()->interrupt(); }

void Timer::run4() {}
