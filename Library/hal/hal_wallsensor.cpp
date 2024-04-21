//******************************************************************************
// @addtogroup HAL
// @file       hal_wallsensor.cpp
// @brief      壁センサー制御
//******************************************************************************

#include "hal_wallsensor.h"

#include "hal_timer.h"

hal::HalStatus hal::initWallSensorPort() {
#ifdef MOUSE_VIOLETTA
    return HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
    // FIXME: Implement WallSensor initialization for Lazuli
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_ZIRCONIA2KAI
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

    // LED_PWM Output Pin
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);

    GPIO_InitStruct.Pin = LED_A2_Pin | LED_A1_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    LL_GPIO_ResetOutputPin(GPIOA, LED_A2_Pin | LED_A1_Pin);

    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

    /* ADC1 GPIO Configuration
    PA0-WKUP   ------> ADC1_IN0
    PA1   ------> ADC1_IN1
    PA2   ------> ADC1_IN2
    PA3   ------> ADC1_IN3
    */
    GPIO_InitStruct.Pin = SEN_FL_Pin | SEN_SL_Pin | SEN_SR_Pin | SEN_FR_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // /* ADC1 DMA Init */
    // LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_0);
    // LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0,
    //                                 LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    // LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0,
    //                               LL_DMA_PRIORITY_VERYHIGH);
    // LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);
    // LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0,
    // LL_DMA_PERIPH_NOINCREMENT); LL_DMA_SetMemoryIncMode(DMA2,
    // LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT); LL_DMA_SetPeriphSize(DMA2,
    // LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD); LL_DMA_SetMemorySize(DMA2,
    // LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD);
    // LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

    // /* ADC1 interrupt Init */
    // NVIC_SetPriority(ADC_IRQn,
    //                  NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    // NVIC_EnableIRQ(ADC_IRQn);

    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
    // ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
    // TODO: Enable DMA on WallSensor ADC
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
    LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
    ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

    /* Configure Regular Channel */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0,
                                  LL_ADC_SAMPLINGTIME_3CYCLES);

    LL_ADC_Enable(ADC1);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI

#ifdef LINUX
    return HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::deinitWallSensorPort() {
#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_ZIRCONIA2KAI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_ZIRCONIA2KAI

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::setWallSensorLedOn(WallSensorNumbers n) {
#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_ZIRCONIA2KAI
    switch (n) {
        case WallSensorNumbers::FRONTLEFT:
            LL_GPIO_ResetOutputPin(GPIOA, LED_A2_Pin | LED_A1_Pin);
            break;
        case WallSensorNumbers::LEFT:
            LL_GPIO_ResetOutputPin(GPIOA, LED_A1_Pin);
            LL_GPIO_SetOutputPin(GPIOA, LED_A2_Pin);
            break;
        case WallSensorNumbers::RIGHT:
            LL_GPIO_ResetOutputPin(GPIOA, LED_A2_Pin);
            LL_GPIO_SetOutputPin(GPIOA, LED_A1_Pin);
            break;
        case WallSensorNumbers::FRONTRIGHT:
            LL_GPIO_SetOutputPin(GPIOA, LED_A2_Pin | LED_A1_Pin);
            break;
        default:
            return hal::HalStatus::ERROR;
    }
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI

#ifdef LINUX
    if (plt::Observer::getInstance()->getWallSensorData(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef LINUX
}

hal::HalStatus hal::setWallSensorLedOff() {
#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_ZIRCONIA2KAI
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI

#ifdef LINUX
    if (plt::Observer::getInstance()->getWallSensorData(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef LINUX
}

hal::HalStatus hal::getWallSensorSingleSync(uint16_t& data,
                                            WallSensorNumbers n) {
#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_ZIRCONIA2KAI
    switch (n) {
        case WallSensorNumbers::FRONTLEFT:
            LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1,
                                         LL_ADC_CHANNEL_0);
            break;
        case WallSensorNumbers::LEFT:
            LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1,
                                         LL_ADC_CHANNEL_1);
            break;
        case WallSensorNumbers::RIGHT:
            LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1,
                                         LL_ADC_CHANNEL_2);
            break;
        case WallSensorNumbers::FRONTRIGHT:
            LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1,
                                         LL_ADC_CHANNEL_3);
            break;
        default:
            return hal::HalStatus::ERROR;
    }
    LL_ADC_REG_StartConversionSWStart(ADC1);
    while (LL_ADC_IsActiveFlag_EOCS(ADC1) == RESET)
        ;
    LL_ADC_ClearFlag_EOCS(ADC1);
    data = LL_ADC_REG_ReadConversionData12(ADC1);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI

#ifdef LINUX
    if (plt::Observer::getInstance()->getWallSensorData(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef LINUX
}

// hal::HalStatus hal::getWallSensorAllSync(hal::WallSensorData& data) {
// #ifdef STM32L4P5xx
//     return hal::HalStatus::SUCCESS;
// #endif  // ifdef STM32L4P5xx

// #ifdef STM32F411xE
//     uint16_t d = 0;
//     hal::setWallSensorLedOn(WallSensorNumbers::FRONTLEFT);
//     // Delay for LED stabilization
//     auto e1 = hal::getWallSensorSingleSync(d, WallSensorNumbers::FRONTLEFT);
//     data.FRONTLEFT = d;
//     hal::setWallSensorLedOn(WallSensorNumbers::LEFT);
//     // Delay for LED stabilization
//     auto e2 = hal::getWallSensorSingleSync(d, WallSensorNumbers::LEFT);
//     data.LEFT = d;
//     hal::setWallSensorLedOn(WallSensorNumbers::RIGHT);
//     // Delay for LED stabilization
//     auto e3 = hal::getWallSensorSingleSync(d, WallSensorNumbers::RIGHT);
//     data.RIGHT = d;
//     hal::setWallSensorLedOn(WallSensorNumbers::FRONTRIGHT);
//     // Delay for LED stabilization
//     auto e4 = hal::getWallSensorSingleSync(d, WallSensorNumbers::FRONTRIGHT);
//     data.FRONTRIGHT = d;
//     hal::setWallSensorLedOff();

//     if (e1 == HalStatus::SUCCESS && e2 == HalStatus::SUCCESS &&
//         e3 == HalStatus::SUCCESS && e4 == HalStatus::SUCCESS) {
//         return hal::HalStatus::SUCCESS;
//     } else {
//         return hal::HalStatus::ERROR;
//     }
// #endif  // ifdef STM32F411xE

// #ifdef LINUX
//     if (plt::Observer::getInstance()->getWallSensorData(data)) {
//         return hal::HalStatus::SUCCESS;
//     } else {
//         return hal::HalStatus::ERROR;
//     }
// #endif  // ifdef LINUX
// }
