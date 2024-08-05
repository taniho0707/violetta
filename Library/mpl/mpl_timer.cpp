//******************************************************************************
// @addtogroup MPL
// @file       mpl_timer.cpp
// @brief      Timer
//******************************************************************************
#include "mpl_timer.h"

#include <cstdint>

#include "hal_conf.h"
#include "hal_timer.h"
#include "mpl_conf.h"
#include "util.h"

// #ifdef STM32L4P5xx
// #include "stm32l4xx_ll_bus.h"
// #include "stm32l4xx_ll_tim.h"
// #endif  // ifdef STM32L4P5xx

#ifdef LINUX
#include <chrono>
#include <cstdint>
#endif

volatile uint32_t mpl::Timer::total = 0;
uint8_t mpl::Timer::tick_count = 0;
uint32_t mpl::Timer::last_reference_us = 0;

#ifdef ENABLE_TIMER_STATISTICS
uint16_t mpl::Timer::history_count1[LENGTH_TIMER_STATISTICS] = {0};
uint16_t mpl::Timer::history_count2[LENGTH_TIMER_STATISTICS] = {0};
uint16_t mpl::Timer::history_count3[LENGTH_TIMER_STATISTICS] = {0};
uint16_t mpl::Timer::history_count4[LENGTH_TIMER_STATISTICS] = {0};
uint16_t mpl::Timer::ite_count1 = LENGTH_TIMER_STATISTICS - 1;
uint16_t mpl::Timer::ite_count2 = LENGTH_TIMER_STATISTICS - 1;
uint16_t mpl::Timer::ite_count3 = LENGTH_TIMER_STATISTICS - 1;
uint16_t mpl::Timer::ite_count4 = LENGTH_TIMER_STATISTICS - 1;
mpl::TimerStatistics mpl::Timer::statistics = {
    .count1_max = 0,
    .count2_max = 0,
    .count3_max = 0,
    .count4_max = 0,
    .count1_min = hal::TIMER_COUNT_MAX,
    .count2_min = hal::TIMER_COUNT_MAX,
    .count3_min = hal::TIMER_COUNT_MAX,
    .count4_min = hal::TIMER_COUNT_MAX,
    .count1_avg = 0,
    .count2_avg = 0,
    .count3_avg = 0,
    .count4_avg = 0,
};
#endif  // ENABLE_TIMER_STATISTICS

#ifdef LINUX
std::chrono::system_clock::time_point mpl::Timer::_observe_last_time = {};
#endif

mpl::MplStatus mpl::Timer::init() {
    if (hal::initTimer() != hal::HalStatus::SUCCESS) {
        return MplStatus::ERROR;
    } else {
        return MplStatus::SUCCESS;
    }
}

uint32_t mpl::Timer::getInternalCounter() {
#ifdef LINUX
    return 0;
#else
    return hal::getTimerCount();
#endif
}

uint32_t mpl::Timer::getNanoTimeFromLastInterrupt() {
    return getInternalCounter() * hal::TIMER_COUNT_INTERVAL / hal::TIMER_COUNT_MAX;
}

uint32_t mpl::Timer::getMicroTime() {
    return total;
}

uint32_t mpl::Timer::getMilliTime() {
    return total / 1000;
}

uint32_t mpl::Timer::getTimeFromLastReference() {
    uint32_t now = getMicroTime();
    uint32_t ret = now - last_reference_us;
    last_reference_us = now;
    return ret;
}

void mpl::Timer::sleepNs(uint32_t ns) {
    uint32_t minimum_unit = (1000.f * hal::TIMER_COUNT_INTERVAL / hal::TIMER_COUNT_MAX);  // [ns]
    uint32_t count_wait = misc::min(getInternalCounter() + ns / minimum_unit, hal::TIMER_COUNT_INTERVAL - 2);
    while (getInternalCounter() < count_wait);
    return;
}

void mpl::Timer::sleepMs(uint32_t ms) {
    uint32_t end_time = getMicroTime() + ms * 1000;
    while (getMicroTime() < end_time);
    return;
}

mpl::MplStatus mpl::Timer::getStatistics(TimerStatistics &statistics) {
#ifdef ENABLE_TIMER_STATISTICS
    statistics = Timer::statistics;
    statistics.count1_max = misc::max(history_count1, LENGTH_TIMER_STATISTICS);
    statistics.count2_max = misc::max(history_count2, LENGTH_TIMER_STATISTICS);
    statistics.count3_max = misc::max(history_count3, LENGTH_TIMER_STATISTICS);
    statistics.count4_max = misc::max(history_count4, LENGTH_TIMER_STATISTICS);
    statistics.count1_min = misc::min(history_count1, LENGTH_TIMER_STATISTICS);
    statistics.count2_min = misc::min(history_count2, LENGTH_TIMER_STATISTICS);
    statistics.count3_min = misc::min(history_count3, LENGTH_TIMER_STATISTICS);
    statistics.count4_min = misc::min(history_count4, LENGTH_TIMER_STATISTICS);
    statistics.count1_avg = misc::average(history_count1, LENGTH_TIMER_STATISTICS);
    statistics.count2_avg = misc::average(history_count2, LENGTH_TIMER_STATISTICS);
    statistics.count3_avg = misc::average(history_count3, LENGTH_TIMER_STATISTICS);
    statistics.count4_avg = misc::average(history_count4, LENGTH_TIMER_STATISTICS);
    return MplStatus::SUCCESS;
#else
    return MplStatus::NO_IMPLEMENT;
#endif  // ENABLE_TIMER_STATISTICS
}

void mpl::Timer::interrupt() {
    total += hal::TIMER_COUNT_INTERVAL;

#ifdef ENABLE_TIMER_STATISTICS
    uint16_t next_ite;
#endif  // ENABLE_TIMER_STATISTICS

    switch (tick_count) {
        case 0:
            run1();
            tick_count = 1;
#ifdef ENABLE_TIMER_STATISTICS
            next_ite = ite_count1 + 1;
            if (next_ite >= LENGTH_TIMER_STATISTICS) {
                next_ite = 0;
            }
            // Timer::statistics.count1_avg =
            //     Timer::statistics.count1_avg +
            //     ((history_count1[ite_count1] - history_count1[next_ite]) /
            //      static_cast<float>(LENGTH_TIMER_STATISTICS));
            ite_count1 = next_ite;
            history_count1[ite_count1] = getInternalCounter();
#endif  // ENABLE_TIMER_STATISTICS
            break;
        case 1:
            run2();
            tick_count = 2;
#ifdef ENABLE_TIMER_STATISTICS
            next_ite = ite_count2 + 1;
            if (next_ite >= LENGTH_TIMER_STATISTICS) {
                next_ite = 0;
            }
            // Timer::statistics.count2_avg =
            //     Timer::statistics.count2_avg +
            //     ((history_count2[ite_count2] - history_count2[next_ite]) /
            //      static_cast<float>(LENGTH_TIMER_STATISTICS));
            ite_count2 = next_ite;
            history_count2[ite_count2] = getInternalCounter();
#endif  // ENABLE_TIMER_STATISTICS
            break;
        case 2:
            run3();
            tick_count = 3;
#ifdef ENABLE_TIMER_STATISTICS
            next_ite = ite_count3 + 1;
            if (next_ite >= LENGTH_TIMER_STATISTICS) {
                next_ite = 0;
            }
            // Timer::statistics.count3_avg =
            //     Timer::statistics.count3_avg +
            //     ((history_count3[ite_count3] - history_count3[next_ite]) /
            //      static_cast<float>(LENGTH_TIMER_STATISTICS));
            ite_count3 = next_ite;
            history_count3[ite_count3] = getInternalCounter();
#endif  // ENABLE_TIMER_STATISTICS
            break;
        case 3:
            run4();
            tick_count = 0;
#ifdef ENABLE_TIMER_STATISTICS
            next_ite = ite_count4 + 1;
            if (next_ite >= LENGTH_TIMER_STATISTICS) {
                next_ite = 0;
            }
            // Timer::statistics.count4_avg =
            //     Timer::statistics.count4_avg +
            //     ((history_count4[ite_count4] - history_count4[next_ite]) /
            //      static_cast<float>(LENGTH_TIMER_STATISTICS));
            ite_count4 = next_ite;
            history_count4[ite_count4] = getInternalCounter();
#endif  // ENABLE_TIMER_STATISTICS
            break;
        default:
            tick_count = 0;
            break;
    }
}
