//******************************************************************************
// @addtogroup HAL
// @file       hal_motor.cpp
// @brief      モータ制御
//******************************************************************************

#include "hal_motor.h"

#include "stm32f4xx_ll_tim.h"

hal::HalStatus hal::initMotorPort() {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload =
        65535;  // TODO: 外部パラメータから設定できるように、暫定 3.2kHz
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM4, &TIM_InitStruct);

    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    LL_TIM_EnableARRPreload(TIM4);
    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);
    LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM4);

    LL_TIM_SetCounter(TIM4, 0);
    LL_TIM_OC_SetCompareCH1(TIM4, 0);
    LL_TIM_OC_SetCompareCH2(TIM4, 0);
    LL_TIM_OC_SetCompareCH3(TIM4, 0);
    LL_TIM_OC_SetCompareCH4(TIM4, 0);
    LL_TIM_EnableCounter(TIM4);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);
    // LL_TIM_EnableAllOutputs(TIM4); // 不要？

    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1 : R_PWM1
    PB7     ------> TIM4_CH2 : R_PWM2
    PB8     ------> TIM4_CH3 : L_PWM1
    PB9     ------> TIM4_CH4 : L_PWM2

    PWM1 | PWM2 | Function
    -----|------|---------
    0    | 0    | Float
    1    | 0    | Reverse
    0    | 1    | Forward
    1    | 1    | Brake
    */
    GPIO_InitStruct.Pin =
        MOT_R_PWM1_Pin | MOT_R_PWM2_Pin | MOT_L_PWM1_Pin | MOT_L_PWM2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::deinitMotorPort() {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::setMotorDutyR(float duty) {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    if (duty >= 0.f && duty <= 1.f) {
        // TODO: 計算式の65535を外部パラメータから読むようにする
        LL_TIM_OC_SetCompareCH4(TIM4, (uint32_t)(65535.f * (1.f - duty)));
        LL_TIM_OC_SetCompareCH3(TIM4, 65535);
    } else if (duty < 0.f && duty >= -1.f) {
        LL_TIM_OC_SetCompareCH4(TIM4, 65535);
        LL_TIM_OC_SetCompareCH3(TIM4, (uint32_t)(65535.f * (1.f + duty)));
    } else {
        return hal::HalStatus::INVALID_PARAMS;
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::setMotorDutyL(float duty) {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    if (duty >= 0.f && duty <= 1.f) {
        // TODO: 計算式の65535を外部パラメータから読むようにする
        LL_TIM_OC_SetCompareCH1(TIM4, (uint32_t)(65535.f * (1.f - duty)));
        LL_TIM_OC_SetCompareCH2(TIM4, 65535);
    } else if (duty < 0.f && duty >= -1.f) {
        LL_TIM_OC_SetCompareCH1(TIM4, 65535);
        LL_TIM_OC_SetCompareCH2(TIM4, (uint32_t)(65535.f * (1.f + duty)));
    } else {
        return hal::HalStatus::INVALID_PARAMS;
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::setMotorFloat() {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    LL_TIM_OC_SetCompareCH1(TIM4, 0);
    LL_TIM_OC_SetCompareCH2(TIM4, 0);
    LL_TIM_OC_SetCompareCH3(TIM4, 0);
    LL_TIM_OC_SetCompareCH4(TIM4, 0);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}
