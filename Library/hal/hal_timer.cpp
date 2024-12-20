//******************************************************************************
// @addtogroup HAL
// @file       hal_timer.cpp
// @brief      TIMを使いTimerを構成する
//******************************************************************************

#include "hal_timer.h"

#ifdef STM32L4P5xx
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_tim.h"
#endif  // ifdef STM32L4P5xx

#ifdef STM32C011xx
#include "stm32c0xx_ll_bus.h"
#include "stm32c0xx_ll_tim.h"
#endif  // ifdef STM32C011xx

#ifdef STM32F411xE
#include "stm32f4xx_ll_tim.h"
#endif  // ifdef STM32F411xE

hal::HalStatus hal::initTimer() {
#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    TIM_InitStruct.Prescaler = 1 - 1;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 30000 - 1;
    LL_TIM_Init(TIM6, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM6);
    LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM6);

    LL_TIM_EnableIT_UPDATE(TIM6);
    LL_TIM_EnableCounter(TIM6);
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

    // TIM5 の割り込みを有効化する
    // 割り込み周期：250us
    NVIC_SetPriority(TIM5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(TIM5_IRQn);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = TIMER_COUNT_MAX - 1;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM5, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM5);
    LL_TIM_SetClockSource(TIM5, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetTriggerOutput(TIM5, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM5);

    LL_TIM_EnableCounter(TIM5);
    LL_TIM_EnableIT_UPDATE(TIM5);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_LAZULI_SENSOR
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 25000 - 1;  // TODO: params から読み込む
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM3, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM3);
    LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM3);

    LL_TIM_EnableCounter(TIM3);
    // LL_TIM_EnableIT_UPDATE(TIM3);  // NOTE: 割り込み機能を使用しないため、ここで有効化していない

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI_SENSOR

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

    // TIM5 の割り込みを有効化する
    // 割り込み周期：250us
    NVIC_SetPriority(TIM5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(TIM5_IRQn);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = TIMER_COUNT_MAX - 1;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM5, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM5);
    LL_TIM_SetClockSource(TIM5, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetTriggerOutput(TIM5, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM5);

    LL_TIM_EnableCounter(TIM5);
    LL_TIM_EnableIT_UPDATE(TIM5);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

uint32_t hal::getTimerCount() {
#ifdef LINUX
    return 0;
#endif

#ifdef MOUSE_VIOLETTA
    return 0;
#endif  // ifdef MOUSE_VIOLETTA

#if defined(MOUSE_LAZULI)
    return LL_TIM_GetCounter(TIM5);
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_LAZULI_SENSOR
    return LL_TIM_GetCounter(TIM3);
#endif  // ifdef MOUSE_LAZULI_SENSOR

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    return LL_TIM_GetCounter(TIM5);
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

#ifdef LINUX
std::chrono::system_clock::time_point hal::getTimeByChrono() {
    return std::chrono::system_clock::now();
}
#endif
