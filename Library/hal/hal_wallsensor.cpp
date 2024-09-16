//******************************************************************************
// @addtogroup HAL
// @file       hal_wallsensor.cpp
// @brief      壁センサー制御
//******************************************************************************

#include "hal_wallsensor.h"

#if defined(MOUSE_LAZULI) && defined(STM32L4P5xx)
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_spi.h"
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_LAZULI_SENSOR
#include "stm32c0xx_ll_adc.h"
#include "stm32c0xx_ll_bus.h"
#include "stm32c0xx_ll_dma.h"
#include "stm32c0xx_ll_gpio.h"
#include "stm32c0xx_ll_rcc.h"
#include "stm32c0xx_ll_spi.h"
#endif  // ifdef MOUSE_LAZULI_SENSOR

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

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

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
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 7;
    LL_SPI_Init(SPI2, &SPI_InitStruct);
    LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_DisableNSSPulseMgt(SPI2);
    LL_SPI_SetRxFIFOThreshold(SPI2, LL_SPI_RX_FIFO_TH_HALF);
    LL_SPI_Enable(SPI2);

    // // CONFIGURATION:
    // // 0b 0000 0000 1101 1100 (DEBUG)
    // // 0b 0000 0000 1101 1000 (疎通確認後)
    // uint16_t buf;
    // hal::readwriteWallSensorSpiSync(0x00DC | static_cast<uint16_t>(AD7091RCommands::CONFIGURATION) | hal::WALLSENSOR_WRITE_MASK, buf);
    // hal::readwriteWallSensorSpiSync(static_cast<uint16_t>(AD7091RCommands::CONFIGURATION), buf);

    // hal::readwriteWallSensorSpiSync(0x0001 | static_cast<uint16_t>(AD7091RCommands::CHANNEL) | hal::WALLSENSOR_WRITE_MASK, buf);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_LAZULI_SENSOR
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_SPI_InitTypeDef SPI_InitStruct = {0};

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);
    LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
    LL_RCC_SetI2SClockSource(LL_RCC_I2S1_CLKSOURCE_SYSCLK);

    LL_GPIO_ResetOutputPin(GPIO_4_GPIO_Port, GPIO_4_Pin);
    // LL_GPIO_ResetOutputPin(GPIO_3_GPIO_Port, GPIO_3_Pin);
    LL_GPIO_SetOutputPin(GPIO_2_GPIO_Port, GPIO_2_Pin);
    LL_GPIO_SetOutputPin(GPIO_1_GPIO_Port, GPIO_1_Pin);
    LL_GPIO_SetOutputPin(GPIO_0_GPIO_Port, GPIO_0_Pin);

    // GPIOx Pin Configuration
    GPIO_InitStruct.Pin = GPIO_0_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIO_0_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_4_Pin;
    LL_GPIO_Init(GPIO_4_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_2_Pin;
    LL_GPIO_Init(GPIO_2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_1_Pin;
    LL_GPIO_Init(GPIO_1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_3_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIO_3_GPIO_Port, &GPIO_InitStruct);

    // ADC GPIO Configuration
    // PA0   ------> ADC1_IN0
    // PA1   ------> ADC1_IN1
    // PA2   ------> ADC1_IN2
    // PA3   ------> ADC1_IN3
    // PA4   ------> ADC1_IN4
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // SPI GPIO Configuration
    // PA5   ------> SPI1_SCK
    // PA6   ------> SPI1_MISO
    // PA7   ------> SPI1_MOSI
    // PA8   ------> SPI1_NSS
    GPIO_InitStruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ADC DMA Configuration
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_ADC1);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_HALFWORD);

    // ADC Configuration
    ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);
    LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_FIXED);
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
    LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
    LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
    LL_ADC_SetTriggerFrequencyMode(ADC1, LL_ADC_CLOCK_FREQ_MODE_HIGH);
    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_0 | LL_ADC_CHANNEL_1 | LL_ADC_CHANNEL_2 | LL_ADC_CHANNEL_3 | LL_ADC_CHANNEL_4);

    while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0) {}

    /* Clear flag ADC channel configuration ready */
    LL_ADC_ClearFlag_CCRDY(ADC1);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_39CYCLES_5);
    LL_ADC_DisableIT_EOC(ADC1);
    LL_ADC_DisableIT_EOS(ADC1);

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

    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);

    wait_loop_index = ((LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32) >> 1);
    while (wait_loop_index != 0) {
        wait_loop_index--;
    }

    LL_ADC_Enable(ADC1);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);

    /* DMA1_Channel1_IRQn interrupt configuration */
    // NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
    // NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* DMA1_Channel2_3_IRQn interrupt configuration */
    // NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
    // NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    NVIC_SetPriority(SPI1_IRQn, 0);
    NVIC_EnableIRQ(SPI1_IRQn);

    LL_SPI_EnableIT_RXNE(SPI1);  // こいつを有効にすると送信も同時に開始する
    LL_SPI_EnableIT_ERR(SPI1);
    // LL_SPI_EnableIT_TXE(SPI1);

    // SPI Configuration
    /* SPI1_RX Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_SPI1_RX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

    /* SPI1_TX Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_SPI1_TX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_SLAVE;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_HARD_INPUT;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 7;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
    LL_SPI_Init(SPI1, &SPI_InitStruct);
    LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_DisableNSSPulseMgt(SPI1);

    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_HALF);
    LL_SPI_Enable(SPI1);

    LL_SPI_TransmitData16(SPI1, 0x0000);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI_SENSOR

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

#ifdef MOUSE_LAZULI_SENSOR
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI_SENSOR

#ifdef MOUSE_ZIRCONIA2KAI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::setWallSensorLedOn(WallSensorNumbers n) {
#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_LAZULI_SENSOR
    LL_GPIO_ResetOutputPin(GPIO_4_GPIO_Port, GPIO_4_Pin);  // タイミング同期のため一旦 NJU6080 PWM OFF
    switch (n) {                                           // (S3,S2,S1)=(GPIO2,GPIO1,GPIO0)
        case WallSensorNumbers::FRONTLEFT:                 // D8=Y1=(L,L,H)
            LL_GPIO_ResetOutputPin(GPIO_2_GPIO_Port, GPIO_2_Pin);
            LL_GPIO_ResetOutputPin(GPIO_1_GPIO_Port, GPIO_1_Pin);
            LL_GPIO_SetOutputPin(GPIO_0_GPIO_Port, GPIO_0_Pin);
            break;
        case WallSensorNumbers::LEFT:  // D7=Y2=(L,H,L)
            LL_GPIO_ResetOutputPin(GPIO_2_GPIO_Port, GPIO_2_Pin);
            LL_GPIO_SetOutputPin(GPIO_1_GPIO_Port, GPIO_1_Pin);
            LL_GPIO_ResetOutputPin(GPIO_0_GPIO_Port, GPIO_0_Pin);
            break;
        case WallSensorNumbers::CENTER:  // D10=Y3=(L,H,H)
            LL_GPIO_ResetOutputPin(GPIO_2_GPIO_Port, GPIO_2_Pin);
            LL_GPIO_SetOutputPin(GPIO_1_GPIO_Port, GPIO_1_Pin);
            LL_GPIO_SetOutputPin(GPIO_0_GPIO_Port, GPIO_0_Pin);
            break;
        case WallSensorNumbers::RIGHT:  // D6=Y4=(H.L.L)
            LL_GPIO_SetOutputPin(GPIO_2_GPIO_Port, GPIO_2_Pin);
            LL_GPIO_ResetOutputPin(GPIO_1_GPIO_Port, GPIO_1_Pin);
            LL_GPIO_ResetOutputPin(GPIO_0_GPIO_Port, GPIO_0_Pin);
            break;
        case WallSensorNumbers::FRONTRIGHT:  // D9=Y0=(L,L,L)
            LL_GPIO_ResetOutputPin(GPIO_2_GPIO_Port, GPIO_2_Pin);
            LL_GPIO_ResetOutputPin(GPIO_1_GPIO_Port, GPIO_1_Pin);
            LL_GPIO_ResetOutputPin(GPIO_0_GPIO_Port, GPIO_0_Pin);
            break;
        default:
            LL_GPIO_SetOutputPin(GPIO_2_GPIO_Port, GPIO_2_Pin);
            LL_GPIO_SetOutputPin(GPIO_1_GPIO_Port, GPIO_1_Pin);
            LL_GPIO_SetOutputPin(GPIO_0_GPIO_Port, GPIO_0_Pin);
            return hal::HalStatus::INVALID_PARAMS;
            break;
    }
    LL_GPIO_SetOutputPin(GPIO_4_GPIO_Port, GPIO_4_Pin);  // NJU6080 PWM ON
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI_SENSOR

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
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_LAZULI_SENSOR
    LL_GPIO_ResetOutputPin(GPIO_4_GPIO_Port, GPIO_4_Pin);  // NJU6080 PWM OFF
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI_SENSOR

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
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_LAZULI_SENSOR
    LL_GPIO_ResetOutputPin(GPIO_4_GPIO_Port, GPIO_4_Pin);  // NJU6080 PWM OFF
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI_SENSOR

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
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::setWallSensorChargeStop() {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_LAZULI
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::startWallSensorConversion() {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_LAZULI
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::setWallSensorAdcSelect(WallSensorNumbers n) {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

    // #ifdef MOUSE_LAZULI
    //     uint16_t channel_register = 0x0000;
    //     uint16_t buf;
    //     switch (n) {
    //         case WallSensorNumbers::FRONTLEFT:
    //             channel_register = 0x0001;  // channel 0
    //             break;
    //         case WallSensorNumbers::LEFT:
    //             channel_register = 0x0002;  // channel 1
    //             break;
    //         case WallSensorNumbers::RIGHT:
    //             channel_register = 0x0004;  // channel 2
    //             break;
    //         case WallSensorNumbers::FRONTRIGHT:
    //             channel_register = 0x0008;  // channel 3
    //             break;
    //         default:
    //             return hal::HalStatus::INVALID_PARAMS;
    //     }
    //     channel_register = (channel_register | WALLSENSOR_WRITE_MASK | static_cast<uint16_t>(AD7091RCommands::CHANNEL));
    //     hal::readwriteWallSensorSpiSync(channel_register, buf);
    // #endif  // ifdef MOUSE_LAZULI
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

#ifdef MOUSE_LAZULI_SENSOR
    uint32_t channel;
    switch (n) {
        case WallSensorNumbers::FRONTLEFT:  // D3=ADC_IN2
            channel = LL_ADC_CHANNEL_4;
            break;
        case WallSensorNumbers::LEFT:  // D2=ADC_IN1
            channel = LL_ADC_CHANNEL_3;
            break;
        case WallSensorNumbers::CENTER:  // D5=ADC_IN4
            channel = LL_ADC_CHANNEL_0;
            break;
        case WallSensorNumbers::RIGHT:  // D1=ADC_IN0
            channel = LL_ADC_CHANNEL_1;
            break;
        case WallSensorNumbers::FRONTRIGHT:  // D4=ADC_IN3
            channel = LL_ADC_CHANNEL_2;
            break;
        default:
            return hal::HalStatus::ERROR;
    }
    LL_ADC_REG_SetSequencerChannels(ADC1, channel);
    while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0) {}
    LL_ADC_ClearFlag_CCRDY(ADC1);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_39CYCLES_5);

    LL_ADC_REG_StartConversion(ADC1);
    while (LL_ADC_IsActiveFlag_EOC(ADC1) == RESET);
    data = LL_ADC_REG_ReadConversionData12(ADC1);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI_SENSOR

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

hal::HalStatus hal::getWallSensorAllSync(uint16_t* data) {
#ifdef MOUSE_LAZULI
    uint16_t data_buf;
    for (int i = 0; i < WALLSENSOR_NUMS + 1; i++) {
        for (int i = 0; i < 100; ++i);  // SPI 通信を連続すると LazuliSensor 側が応答しなくなるため適当なウェイトを入れる
        hal::readwriteWallSensorSpiSync(0x0000, data_buf);
        uint8_t channel = (data_buf >> 12);
        if (channel < WALLSENSOR_NUMS) {  // NOTE: WALLSENSOR_NUMS 以上であった場合は不正なデータと判断
            data[channel] = (data_buf & 0x0FFF);
        }
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
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
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::sendWallSensorDataSpiSync(uint16_t& data) {
#ifdef MOUSE_LAZULI_SENSOR
    // for Debug
    // static uint16_t count = 0;
    // ++count;

    // while (LL_SPI_IsActiveFlag_TXE(SPI1) == RESET);
    LL_SPI_TransmitData16(SPI1, data);
    // LL_SPI_TransmitData16(SPI1, 0x55AA);
    // while (LL_SPI_IsActiveFlag_RXNE(SPI1) == RESET);

    // LL_SPI_ReceiveData16(SPI1);  // dummy read
    // while (LL_SPI_IsActiveFlag_BSY(SPI1) == SET);

    return hal::HalStatus::SUCCESS;
#else
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI_SENSOR
}
