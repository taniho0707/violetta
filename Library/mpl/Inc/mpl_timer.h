//******************************************************************************
// @addtogroup MPL
// @file       mpl_timer.h
// @brief      Timer
//******************************************************************************
#pragma once

#include "hal_timer.h"

// #ifdef STM32L4P5xx
// // STM32HAL/LL
// #include "stm32l4xx_hal.h"
// #include "stm32l4xx_ll_tim.h"
// #endif  // ifdef STM32L4P5xx

// #ifdef LINUX
// #include <chrono>
// #include <cstdint>
// #endif

namespace mpl {

#define TIMER_COUNT_INTERVAL 250

class Timer {
   private:
    // 4294秒程度でオーバーフローする点に注意
    static volatile uint32_t total;

    // 制御周期 1ms のうち何回目の割り込みかを示す変数
    static uint8_t tick_count;

    static uint32_t last_reference_us;

#ifdef LINUX
    static std::chrono::system_clock::time_point _observe_last_time;
#endif

    // TIMER_COUNT_INTERVAL毎に行う処理
    // 各自で定義
    static void run1();
    static void run2();
    static void run3();
    static void run4();

    // 内部カウンタ値を返す
    static uint16_t getInternalCounter();

   public:
    static void init();

    static uint32_t getMilliTime();
    static uint32_t getMicroTime();

    // 前回同じ関数を呼び出してからの時間を [us] で返す
    static uint32_t getTimeFromLastReference();

    /**
     * @brief タイマーの加算を行う。<br>
     * 正確に<TIMER_COUNT_INTERVAL>[us]ごとに呼ばれる必要がある。
     */
    static void interrupt();
};

}  // namespace mpl
