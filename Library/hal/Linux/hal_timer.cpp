//******************************************************************************
// @addtogroup HAL
// @file       hal_timer.cpp
// @brief      TIMを使いTimerを構成する
//******************************************************************************

#include "hal_timer.h"

hal::HalStatus hal::initTimer() {
    return hal::HalStatus::SUCCESS;
}

uint32_t hal::getTimerCount() {
    return 0;
}

std::chrono::system_clock::time_point hal::getTimeByChrono() {
    return std::chrono::system_clock::now();
}
