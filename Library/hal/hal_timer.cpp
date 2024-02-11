//******************************************************************************
// @addtogroup HAL
// @file       hal_timer.cpp
// @brief      TIMを使いTimerを構成する
//******************************************************************************

#include "hal_timer.h"

hal::HalStatus hal::initTimer() {
#ifdef STM32L4P5xx
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
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return HalStatus::SUCCESS;
#endif
}

uint32_t hal::getMilliTimeByTim() {
#ifdef STM32L4P5xx
    return 0;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return 0;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return 0;
#endif
}

uint32_t hal::getMicroTimeByTim() {
#ifdef STM32L4P5xx
    return 0;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return 0;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return 0;
#endif
}

#ifdef LINUX
std::chrono::system_clock::time_point hal::getTimeByChrono() {
    return std::chrono::system_clock::now();
}
#endif
