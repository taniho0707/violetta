//******************************************************************************
// @addtogroup MPL
// @file       mpl_timer.cpp
// @brief      Timer
//******************************************************************************
#include "mpl_timer.h"

// #ifdef STM32L4P5xx
// #include "stm32l4xx_ll_bus.h"
// #include "stm32l4xx_ll_tim.h"
// #endif  // ifdef STM32L4P5xx

// #ifdef LINUX
// #include <chrono>
// #include <cstdint>
// #endif

volatile uint32_t mpl::Timer::total = 0;
uint8_t mpl::Timer::tick_count = 0;
uint32_t mpl::Timer::last_reference_us = 0;
std::chrono::system_clock::time_point mpl::Timer::_observe_last_time = {};

void mpl::Timer::init() { hal::initTimer(); }

uint16_t mpl::Timer::getInternalCounter() {
#ifdef STM32L4P5xx
    return LL_TIM_GetCounter(TIM6);
#else
    return 0;
#endif  // ifdef STM32L4P5xx
}

uint32_t mpl::Timer::getMicroTime() { return total; }

uint32_t mpl::Timer::getMilliTime() { return total / 1000; }

uint32_t mpl::Timer::getTimeFromLastReference() {
    uint32_t now;
    now = total + getInternalCounter() / 120;
    uint32_t ret = now - last_reference_us;
    last_reference_us = now;
    return ret;
}

void mpl::Timer::interrupt() {
    total += TIMER_COUNT_INTERVAL;

    switch (tick_count) {
        case 0:
            run1();
            tick_count = 1;
            break;
        case 1:
            run2();
            tick_count = 2;
            break;
        case 2:
            run3();
            tick_count = 3;
            break;
        case 3:
            run4();
            tick_count = 0;
            break;
        default:
            tick_count = 0;
            break;
    }
}
