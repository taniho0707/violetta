//******************************************************************************
// @addtogroup HAL
// @file       hal_motor.cpp
// @brief      モータ制御
//******************************************************************************

#include "hal_motor.h"

#ifdef STM32L4P5xx
#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_tim.h"
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
#include "stm32f4xx_ll_tim.h"
#endif  // ifdef STM32F411xE

hal::HalStatus hal::initMotorPort() {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);

    TIM_InitStruct.Prescaler = 16384 - 1;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    // TIM_InitStruct.Autoreload = 4294967295;
    TIM_InitStruct.Autoreload = MOTOR_TIMER_MAXCOUNT - 1;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM2, &TIM_InitStruct);
    LL_TIM_EnableARRPreload(TIM2);
    LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH4);

    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetOCRefClearInputSource(TIM2, LL_TIM_OCREF_CLR_INT_NC);
    LL_TIM_DisableExternalClock(TIM2);
    LL_TIM_ConfigETR(TIM2, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
    LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM2);

    LL_TIM_SetCounter(TIM2, 0);
    LL_TIM_OC_SetCompareCH1(TIM2, 0);
    LL_TIM_OC_SetCompareCH2(TIM2, 0);
    LL_TIM_OC_SetCompareCH4(TIM2, 0);
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);

    LL_GPIO_SetOutputPin(GPIOB, MOTOR_L_EN_Pin | MOTOR_R_EN_Pin);
    LL_GPIO_ResetOutputPin(GPIOB, SUCTION_PWM_Pin);

    /** TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    PB11     ------> TIM2_CH4

    EN | PH | Function
    ---|----|---------
    0  | x  | Brake
    1  | 0  | Reverse
    1  | 1  | Forward
    */
    GPIO_InitStruct.Pin = MOTOR_R_PWM_Pin | MOTOR_L_PWM_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SUCTION_PWM_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MOTOR_L_EN_Pin | MOTOR_R_EN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1 GPIO Configuration
    PA3   ------> ADC1_IN8 : MOTOR_R_SENS
    PA4   ------> ADC1_IN9 : MOTOR_L_SENS
    */
    GPIO_InitStruct.Pin = MOTOR_R_SENS_Pin | MOTOR_L_SENS_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLLSAI1);
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV2);

    // LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_ADC1);
    // LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    // LL_DMA_SetChannelPriorityLevel(DMA2, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_MEDIUM);
    // LL_DMA_SetMode(DMA2, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
    // LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
    // LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
    // LL_DMA_SetPeriphSize(DMA2, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_HALFWORD);
    // LL_DMA_SetMemorySize(DMA2, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_HALFWORD);

    // NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
    // NVIC_EnableIRQ(ADC1_2_IRQn);

    LL_ADC_Disable(ADC1);

    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);

    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
    // ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
    // ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
    ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV8;
    ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);

    /* Disable ADC deep power down (enabled by default after reset state) */
    LL_ADC_DisableDeepPowerDown(ADC1);
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC1);
    /* Delay for ADC internal voltage regulator stabilization. */
    /* Compute number of CPU cycles to wait for, from delay in us. */
    /* Note: Variable divided by 2 to compensate partially */
    /* CPU processing cycles (depends on compilation optimization). */
    /* Note: If system core clock frequency is below 200kHz, wait time */
    /* is only a few CPU processing cycles. */
    uint32_t wait_loop_index;
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0) {
        wait_loop_index--;
    }

    // ADC のキャリブレーションは省略 (Battery で行っているため)

    LL_ADC_Enable(ADC1);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);

    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_8);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_24CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SINGLE_ENDED);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 65536 / 16 - 1;  // TODO: 外部パラメータから設定できるように、暫定 3.2kHz * 2
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
    GPIO_InitStruct.Pin = MOT_R_PWM1_Pin | MOT_R_PWM2_Pin | MOT_L_PWM1_Pin | MOT_L_PWM2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::deinitMotorPort() {
#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE
}

hal::HalStatus hal::setMotorDutyR(float duty) {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    if (duty >= 0.f && duty <= 1.f) {
        LL_TIM_OC_SetCompareCH1(TIM4, (uint32_t)((1.f - duty) * MOTOR_TIMER_MAXCOUNT));
        LL_GPIO_SetOutputPin(GPIOB, MOTOR_R_EN_Pin);
    } else if (duty < 0.f && duty >= -1.f) {
        LL_TIM_OC_SetCompareCH1(TIM4, (uint32_t)((1.f + duty) * MOTOR_TIMER_MAXCOUNT));
        LL_GPIO_ResetOutputPin(GPIOB, MOTOR_R_EN_Pin);
    } else {
        return hal::HalStatus::INVALID_PARAMS;
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    if (duty >= 0.f && duty <= 1.f) {
        // TODO: 計算式の65535を外部パラメータから読むようにする
        LL_TIM_OC_SetCompareCH4(TIM4, (uint32_t)(4095.f * (1.f - duty)));
        LL_TIM_OC_SetCompareCH3(TIM4, 4095);
    } else if (duty < 0.f && duty >= -1.f) {
        LL_TIM_OC_SetCompareCH4(TIM4, 4095);
        LL_TIM_OC_SetCompareCH3(TIM4, (uint32_t)(4095.f * (1.f + duty)));
    } else {
        return hal::HalStatus::INVALID_PARAMS;
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::setMotorDutyL(float duty) {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    if (duty >= 0.f && duty <= 1.f) {
        LL_TIM_OC_SetCompareCH2(TIM4, (uint32_t)((1.f - duty) * MOTOR_TIMER_MAXCOUNT));
        LL_GPIO_SetOutputPin(GPIOB, MOTOR_L_EN_Pin);
    } else if (duty < 0.f && duty >= -1.f) {
        LL_TIM_OC_SetCompareCH2(TIM4, (uint32_t)((1.f + duty) * MOTOR_TIMER_MAXCOUNT));
        LL_GPIO_ResetOutputPin(GPIOB, MOTOR_L_EN_Pin);
    } else {
        return hal::HalStatus::INVALID_PARAMS;
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    if (duty >= 0.f && duty <= 1.f) {
        // TODO: 計算式の65535を外部パラメータから読むようにする
        LL_TIM_OC_SetCompareCH1(TIM4, (uint32_t)(4095.f * (1.f - duty)));
        LL_TIM_OC_SetCompareCH2(TIM4, 4095);
    } else if (duty < 0.f && duty >= -1.f) {
        LL_TIM_OC_SetCompareCH1(TIM4, 4095);
        LL_TIM_OC_SetCompareCH2(TIM4, (uint32_t)(4095.f * (1.f + duty)));
    } else {
        return hal::HalStatus::INVALID_PARAMS;
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::setMotorFloat() {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    LL_TIM_OC_SetCompareCH1(TIM2, 0);
    LL_TIM_OC_SetCompareCH2(TIM2, 0);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    LL_TIM_OC_SetCompareCH1(TIM4, 0);
    LL_TIM_OC_SetCompareCH2(TIM4, 0);
    LL_TIM_OC_SetCompareCH3(TIM4, 0);
    LL_TIM_OC_SetCompareCH4(TIM4, 0);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::setMotorDutySuction(float duty) {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    if (duty >= 0.f && duty <= 1.f) {
        LL_TIM_OC_SetCompareCH4(TIM2, (uint32_t)((1.f - duty) * MOTOR_TIMER_MAXCOUNT));
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::INVALID_PARAMS;
    }
#endif  // ifdef MOUSE_LAZULI

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE
}

hal::HalStatus hal::getMotorCurrentSync(float& current_l, float& current_r) {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    // Right
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_8);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_24CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SINGLE_ENDED);

    LL_ADC_REG_StartConversion(ADC1);
    while (LL_ADC_IsActiveFlag_EOC(ADC1) == RESET);
    LL_ADC_ClearFlag_EOC(ADC1);
    current_r = LL_ADC_REG_ReadConversionData12(ADC1) / 4095.0f /*[12bit]*/ * 3.0f /*[V]*/ / 1000.0f /*[R]*/;

    // Left
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_9);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_24CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_9, LL_ADC_SINGLE_ENDED);

    LL_ADC_REG_StartConversion(ADC1);
    while (LL_ADC_IsActiveFlag_EOC(ADC1) == RESET);
    LL_ADC_ClearFlag_EOC(ADC1);
    current_l = LL_ADC_REG_ReadConversionData12(ADC1) / 4095.0f /*[12bit]*/ * 3.0f /*[V]*/ / 1000.0f /*[R]*/;

    // FIXME: SamplingTime を調整する
    // TODO: 正確な分圧比をパラメータとして受け取る
    // TODO: 他の ADC での設定変更の際に影響しないようにケア必要
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE
}
