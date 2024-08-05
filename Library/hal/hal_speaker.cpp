//******************************************************************************
// @addtogroup HAL
// @file       hal_speaker.cpp
// @brief      スピーカー制御
//******************************************************************************

#include "hal_speaker.h"

#ifdef MOUSE_LAZULI
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_tim.h"
#endif  // ifdef MOUSE_LAZULI

#ifdef STM32F411xE
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_rcc.h"
#endif  // ifdef STM32F411xE

#ifdef MOUSE_LAZULI
#define SPEAKER_TIMER_MAX 16
#endif  // ifdef MOUSE_LAZULI

hal::HalStatus hal::initSpeakerPort() {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_LAZULI
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM15);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = SPEAKER_TIMER_MAX - 1;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM15, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM15);
    LL_TIM_OC_EnablePreload(TIM15, LL_TIM_CHANNEL_CH1);
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = SPEAKER_TIMER_MAX / 2 - 1;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_Init(TIM15, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM15, LL_TIM_CHANNEL_CH1);
    LL_TIM_SetTriggerOutput(TIM15, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM15);
    // TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
    // TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
    TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_ENABLE;
    TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_ENABLE;
    TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime = 0;
    TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    // TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE;
    LL_TIM_BDTR_Init(TIM15, &TIM_BDTRInitStruct);

    /**TIM15 GPIO Configuration
    PA2     ------> TIM15_CH1
    */
    GPIO_InitStruct.Pin = SPEAKER_PWM_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_14;
    LL_GPIO_Init(SPEAKER_PWM_GPIO_Port, &GPIO_InitStruct);

    LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM15);
    LL_TIM_EnableAllOutputs(TIM15);
    // LL_TIM_GenerateEvent_UPDATE(TIM15);
#endif  // ifdef MOUSE_LAZULI

    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::deinitSpeakerPort() {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_LAZULI
#endif  // ifdef MOUSE_LAZULI

    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::setSpeakerFrequency(uint16_t freq) {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_LAZULI
    uint32_t system_clock = 100000000;  // FIXME: hal_conf.hで定義した値を使う
    uint32_t value = system_clock / SPEAKER_TIMER_MAX / freq - 1;
    LL_TIM_SetPrescaler(TIM15, value);
    LL_TIM_OC_SetCompareCH1(TIM15, SPEAKER_TIMER_MAX / 2 - 1);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::offSpeaker() {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_LAZULI
    // TODO: 使用しない場合にはクロックを止める
    LL_TIM_OC_SetCompareCH1(TIM15, 0);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

    return hal::HalStatus::NOIMPLEMENT;
}