//******************************************************************************
// @addtogroup MPL
// @file       mpl_timer.h
// @brief      Timer
//******************************************************************************
#pragma once

#include "mpl_conf.h"
#include "util.h"

#ifndef MOUSE_LAZULI_SENSOR
#define ENABLE_TIMER_STATISTICS       // タイマーの統計情報を取得する
#define LENGTH_TIMER_STATISTICS 1000  // タイマーの Average 取得期間
#endif                                // ifndef MOUSE_LAZULI_SENSOR

// #ifdef STM32L4P5xx
// // STM32HAL/LL
// #include "stm32l4xx_hal.h"
// #include "stm32l4xx_ll_tim.h"
// #endif  // ifdef STM32L4P5xx

#ifdef LINUX
#include <chrono>
#include <cstdint>
#endif

namespace mpl {

struct TimerStatistics {
    uint16_t count1_max;
    uint16_t count2_max;
    uint16_t count3_max;
    uint16_t count4_max;
    uint16_t count1_min;
    uint16_t count2_min;
    uint16_t count3_min;
    uint16_t count4_min;
    float count1_avg;
    float count2_avg;
    float count3_avg;
    float count4_avg;
    uint16_t max() {
        return misc::max(count1_max, count2_max, count3_max, count4_max);
    }
    uint16_t min() {
        return misc::min(count1_min, count2_min, count3_min, count4_min);
    }
    float avg() {
        return (count1_avg + count2_avg + count3_avg + count4_avg) / 4;
    }
};

class Timer {
   private:
    // 4294秒程度でオーバーフローする点に注意
    // TIMER_COUNT_INTERVAL [us] ごとに加算される
    static volatile uint32_t total;

    // 制御周期 1ms のうち何回目の割り込みかを示す変数
    static uint8_t tick_count;

    static uint32_t last_reference_us;

#ifdef ENABLE_TIMER_STATISTICS
    static TimerStatistics statistics;
    static uint16_t ite_count1;  // 最新の記録を指すイテレータ
    static uint16_t ite_count2;
    static uint16_t ite_count3;
    static uint16_t ite_count4;
    static uint16_t history_count1[LENGTH_TIMER_STATISTICS];
    static uint16_t history_count2[LENGTH_TIMER_STATISTICS];
    static uint16_t history_count3[LENGTH_TIMER_STATISTICS];
    static uint16_t history_count4[LENGTH_TIMER_STATISTICS];
#endif  // ENABLE_TIMER_STATISTICS

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

    // タイマーの統計情報を取得する
    static MplStatus getStatistics(TimerStatistics &statistics);

    /**
     * @brief タイマーの加算を行う。<br>
     * 正確に<TIMER_COUNT_INTERVAL>[us]ごとに呼ばれる必要がある。
     */
    static void interrupt();
};

}  // namespace mpl
