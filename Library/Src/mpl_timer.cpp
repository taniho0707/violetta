//******************************************************************************
// @addtogroup MPL
// @file       mpl_timer.cpp
// @brief      Timer
//******************************************************************************
#include "mpl_timer.h"

#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_tim.h"

__IO uint32_t Timer::total = 0;
uint8_t Timer::tick_count = 0;
uint32_t Timer::last_reference_us = 0;

Timer::Timer() {
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    NVIC_SetPriority(TIM6_DAC_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
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
}

uint16_t Timer::getInternalCounter() { return LL_TIM_GetCounter(TIM6); }

uint32_t Timer::getMicroTime() { return total; }

uint32_t Timer::getMilliTime() { return total / 1000; }

uint32_t Timer::getTimeFromLastReference() {
    uint32_t now;
    now = total + getInternalCounter() / 120;
    uint32_t ret = now - last_reference_us;
    last_reference_us = now;
    return ret;
}

void Timer::interrupt() {
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
