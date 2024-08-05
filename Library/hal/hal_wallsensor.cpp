//******************************************************************************
// @addtogroup HAL
// @file       hal_wallsensor.cpp
// @brief      壁センサー制御
//******************************************************************************

#include "hal_wallsensor.h"

#ifdef MOUSE_LAZULI
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_spi.h"
#endif  // ifdef MOUSE_LAZULI

namespace hal {
hal::HalStatus readwriteWallSensorSpiSync(uint16_t tx, uint16_t& rx);
}

hal::HalStatus hal::initWallSensorPort() {
#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_SPI_InitTypeDef SPI_InitStruct = {0};

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

    LL_GPIO_SetOutputPin(GPIOC, ADC_CNVST_Pin);
    LL_GPIO_SetOutputPin(GPIOB, IRLED_P_Pin);
    LL_GPIO_ResetOutputPin(GPIOB, IRLED_N_FL_Pin);
    LL_GPIO_ResetOutputPin(GPIOA, IRLED_N_L_Pin | IRLED_N_R_Pin | IRLED_N_FR_Pin);

    GPIO_InitStruct.Pin = ADC_CNVST_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC_BUSY_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(ADC_BUSY_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = IRLED_P_Pin | IRLED_N_FL_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = IRLED_N_L_Pin | IRLED_N_R_Pin | IRLED_N_FR_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI2 GPIO Configuration
    PB12   ------> SPI2_NSS
    PB13   ------> SPI2_SCK
    PB14   ------> SPI2_MISO
    PB15   ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = ADC_SCK_Pin | ADC_MISO_Pin | ADC_MOSI_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    LL_GPIO_SetOutputPin(GPIOB, ADC_CS_Pin);
    GPIO_InitStruct.Pin = ADC_CS_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMAMUX_REQ_SPI2_RX);
    // LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    // LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_HIGH);
    // LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_NORMAL);
    // LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);
    // LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);
    // LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);
    // LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);

    // LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_6, LL_DMAMUX_REQ_SPI2_TX);
    // LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    // LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PRIORITY_HIGH);
    // LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_NORMAL);
    // LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PERIPH_NOINCREMENT);
    // LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MEMORY_INCREMENT);
    // LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_BYTE);
    // LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_BYTE);

    // NVIC_SetPriority(SPI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    // NVIC_EnableIRQ(SPI2_IRQn);

    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 7;
    LL_SPI_Init(SPI2, &SPI_InitStruct);
    LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_DisableNSSPulseMgt(SPI2);
    LL_SPI_SetRxFIFOThreshold(SPI2, LL_SPI_RX_FIFO_TH_HALF);
    LL_SPI_Enable(SPI2);

    // CONFIGURATION:
    // 0b 0000 0000 1101 1100 (DEBUG)
    // 0b 0000 0000 1101 1000 (疎通確認後)
    uint16_t buf;
    hal::readwriteWallSensorSpiSync(0x00DC | static_cast<uint16_t>(AD7091RCommands::CONFIGURATION) | hal::WALLSENSOR_WRITE_MASK, buf);
    hal::readwriteWallSensorSpiSync(static_cast<uint16_t>(AD7091RCommands::CONFIGURATION), buf);

    hal::readwriteWallSensorSpiSync(0x0001 | static_cast<uint16_t>(AD7091RCommands::CHANNEL) | hal::WALLSENSOR_WRITE_MASK, buf);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
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
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_3CYCLES);

    LL_ADC_Enable(ADC1);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::deinitWallSensorPort() {
#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_ZIRCONIA2KAI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::setWallSensorLedOn(WallSensorNumbers n) {
#ifdef LINUX
    // if (plt::Observer::getInstance()->getWallSensorData(data)) {
    //     return hal::HalStatus::SUCCESS;
    // } else {
    //     return hal::HalStatus::ERROR;
    // }
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    switch (n) {
        case WallSensorNumbers::FRONTLEFT:
            LL_GPIO_SetOutputPin(GPIOB, IRLED_N_FL_Pin);
            break;
        case WallSensorNumbers::LEFT:
            LL_GPIO_SetOutputPin(GPIOA, IRLED_N_L_Pin);
            break;
        case WallSensorNumbers::RIGHT:
            LL_GPIO_SetOutputPin(GPIOA, IRLED_N_R_Pin);
            break;
        case WallSensorNumbers::FRONTRIGHT:
            LL_GPIO_SetOutputPin(GPIOA, IRLED_N_FR_Pin);
            break;
        default:
            return hal::HalStatus::INVALID_PARAMS;
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
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
}

hal::HalStatus hal::setWallSensorLedOff(WallSensorNumbers n) {
#ifdef LINUX
    // if (plt::Observer::getInstance()->getWallSensorData(data)) {
    //     return hal::HalStatus::SUCCESS;
    // } else {
    //     return hal::HalStatus::ERROR;
    // }
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    switch (n) {
        case WallSensorNumbers::FRONTLEFT:
            LL_GPIO_ResetOutputPin(GPIOB, IRLED_N_FL_Pin);
            break;
        case WallSensorNumbers::LEFT:
            LL_GPIO_ResetOutputPin(GPIOA, IRLED_N_L_Pin);
            break;
        case WallSensorNumbers::RIGHT:
            LL_GPIO_ResetOutputPin(GPIOA, IRLED_N_R_Pin);
            break;
        case WallSensorNumbers::FRONTRIGHT:
            LL_GPIO_ResetOutputPin(GPIOA, IRLED_N_FR_Pin);
            break;
        default:
            return hal::HalStatus::INVALID_PARAMS;
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::setWallSensorLedOff() {
#ifdef LINUX
    // if (plt::Observer::getInstance()->getWallSensorData(data)) {
    //     return hal::HalStatus::SUCCESS;
    // } else {
    //     return hal::HalStatus::ERROR;
    // }
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    LL_GPIO_ResetOutputPin(GPIOB, IRLED_N_FL_Pin);
    LL_GPIO_ResetOutputPin(GPIOA, IRLED_N_L_Pin | IRLED_N_R_Pin | IRLED_N_FR_Pin);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::setWallSensorChargeStart() {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_LAZULI
    LL_GPIO_ResetOutputPin(GPIOB, IRLED_P_Pin);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::setWallSensorChargeStop() {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_LAZULI
    LL_GPIO_SetOutputPin(GPIOB, IRLED_P_Pin);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::startWallSensorConversion() {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_LAZULI
    LL_GPIO_ResetOutputPin(GPIOC, ADC_CNVST_Pin);
    // // pipelineの最適化で命令が実行されない可能性に注意
    // __NOP();
    // __NOP();
    LL_GPIO_SetOutputPin(GPIOC, ADC_CNVST_Pin);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::setWallSensorAdcSelect(WallSensorNumbers n) {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_LAZULI
    uint16_t channel_register = 0x0000;
    uint16_t buf;
    switch (n) {
        case WallSensorNumbers::FRONTLEFT:
            channel_register = 0x0001;  // channel 0
            break;
        case WallSensorNumbers::LEFT:
            channel_register = 0x0002;  // channel 1
            break;
        case WallSensorNumbers::RIGHT:
            channel_register = 0x0004;  // channel 2
            break;
        case WallSensorNumbers::FRONTRIGHT:
            channel_register = 0x0008;  // channel 3
            break;
        default:
            return hal::HalStatus::INVALID_PARAMS;
    }
    channel_register = (channel_register | WALLSENSOR_WRITE_MASK | static_cast<uint16_t>(AD7091RCommands::CHANNEL));
    hal::readwriteWallSensorSpiSync(channel_register, buf);
#endif  // ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::getWallSensorSingleSync(uint16_t& data) {
#ifdef MOUSE_LAZULI
    uint16_t data_buf;
    hal::readwriteWallSensorSpiSync(static_cast<uint16_t>(AD7091RCommands::NOP), data_buf);
    data = (data_buf & 0x0FFF);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::getWallSensorSingleSync(uint16_t& data, WallSensorNumbers n) {
#ifdef LINUX
    // if (plt::Observer::getInstance()->getWallSensorData(data)) {
    //     return hal::HalStatus::SUCCESS;
    // } else {
    //     return hal::HalStatus::ERROR;
    // }
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    switch (n) {
        case WallSensorNumbers::FRONTLEFT:
            LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
            break;
        case WallSensorNumbers::LEFT:
            LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
            break;
        case WallSensorNumbers::RIGHT:
            LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);
            break;
        case WallSensorNumbers::FRONTRIGHT:
            LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
            break;
        default:
            return hal::HalStatus::ERROR;
    }
    LL_ADC_REG_StartConversionSWStart(ADC1);
    while (LL_ADC_IsActiveFlag_EOCS(ADC1) == RESET);
    LL_ADC_ClearFlag_EOCS(ADC1);
    data = LL_ADC_REG_ReadConversionData12(ADC1);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
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

hal::HalStatus hal::readwriteWallSensorSpiSync(uint16_t tx, uint16_t& rx) {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_LAZULI
    LL_GPIO_ResetOutputPin(GPIOB, ADC_CS_Pin);
    while (LL_SPI_IsActiveFlag_TXE(SPI2) == RESET);
    LL_SPI_TransmitData16(SPI2, tx);
    while (LL_SPI_IsActiveFlag_RXNE(SPI2) == RESET);
    LL_GPIO_SetOutputPin(GPIOB, ADC_CS_Pin);

    rx = LL_SPI_ReceiveData16(SPI2);
    while (LL_SPI_IsActiveFlag_BSY(SPI2) == SET);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI
}
