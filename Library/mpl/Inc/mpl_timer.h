//******************************************************************************
// @addtogroup MPL
// @file       mpl_timer.h
// @brief      Timer
//******************************************************************************
#pragma once

#include "hal_timer.h"
#include "mpl_conf.h"

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

class Timer {
   private:
    // 4294秒程度でオーバーフローする点に注意
    // TIMER_COUNT_INTERVAL [us] ごとに加算される
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
    static uint32_t getInternalCounter();

   public:
    static MplStatus init();

    static uint32_t getNanoTimeFromLastInterrupt();
    static uint32_t getMilliTime();
    static uint32_t getMicroTime();

    // 前回同じ関数を呼び出してからの時間を [us] で返す
    // TIMER_COUNT_INTERVAL [us] 単位で記録される
    static uint32_t getTimeFromLastReference();

    // 指定時間待機する
    // ※ Timer::interrupt() 内で使われることを前提としている
    // 最低単位は 20 [ns] = 0.02 [us] のため、
    // 20 [ns]で除した余りは切り捨てられる
    // 精度は + 0 [ns] / - 20 [ns]
    // 新しく<TIMER_COUNT_INTERVAL>割り込みが発生すると即座に返される
    static void sleepNs(uint32_t ns);

    // 指定時間待機する
    // 精度は + 0 [ms] / - 250 [us]
    static void sleepMs(uint32_t ms);

    /**
     * @brief タイマーの加算を行う。<br>
     * 正確に<TIMER_COUNT_INTERVAL>[us]ごとに呼ばれる必要がある。
     */
    static void interrupt();
};

}  // namespace mpl
