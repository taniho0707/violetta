//******************************************************************************
// @addtogroup HAL
// @file       hal_debug.cpp
// @brief      デバッグ用機能(UARTログ等)
//******************************************************************************

#include "hal_debug.h"

#ifdef STM32L4P5xx
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_usart.h"
#endif  // ifdef STM32L4P5xx

hal::HalStatus hal::initUartDebugPort(InitializeType type) {
#ifdef STM32L4P5xx
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    /**USART1 GPIO Configuration
    PA9   ------> USART1_TX
    PA10   ------> USART1_RX
    */

    if (type == InitializeType::Sync) {
        GPIO_InitStruct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
        LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
        USART_InitStruct.BaudRate = 921600;
        USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
        USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
        USART_InitStruct.Parity = LL_USART_PARITY_NONE;
        USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
        USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
        USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
        LL_USART_Init(USART1, &USART_InitStruct);
        LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
        LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
        LL_USART_DisableFIFO(USART1);
        LL_USART_ConfigSyncMode(USART1);
        LL_USART_Enable(USART1);

        return HalStatus::SUCCESS;
    } else if (type == InitializeType::Async) {
        return HalStatus::NOIMPLEMENT;
    } else if (type == InitializeType::Dma) {
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

        LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1,
                                LL_DMAMUX_REQ_USART1_RX);
        LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1,
                                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
        LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1,
                                       LL_DMA_PRIORITY_LOW);
        LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
        LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1,
                                LL_DMA_PERIPH_NOINCREMENT);
        LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1,
                                LL_DMA_MEMORY_INCREMENT);
        LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);
        LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

        LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2,
                                LL_DMAMUX_REQ_USART1_TX);
        LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2,
                                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
        LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2,
                                       LL_DMA_PRIORITY_LOW);
        LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
        LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2,
                                LL_DMA_PERIPH_NOINCREMENT);
        LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2,
                                LL_DMA_MEMORY_INCREMENT);
        LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
        LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

        NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
        NVIC_EnableIRQ(DMA1_Channel1_IRQn);
        NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 8));
        NVIC_EnableIRQ(DMA1_Channel2_IRQn);

        LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
        LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
        LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
        LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);

        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

        NVIC_SetPriority(USART1_IRQn,
                         NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
        NVIC_EnableIRQ(USART1_IRQn);

        GPIO_InitStruct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
        LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
        USART_InitStruct.BaudRate = 921600;
        USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
        USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
        USART_InitStruct.Parity = LL_USART_PARITY_NONE;
        USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
        USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
        USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
        LL_USART_Init(USART1, &USART_InitStruct);
        LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
        LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
        LL_USART_DisableFIFO(USART1);
        LL_USART_ConfigAsyncMode(USART1);
        LL_USART_Enable(USART1);

        // LL_USART_EnableIT_TC(USART1);

        return HalStatus::SUCCESS;
    } else {
        return HalStatus::INVALID_PARAMS;
    }
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    /**USART1 GPIO Configuration
    PA9   ------> USART1_TX
    PA10   ------> USART1_RX
    */
    GPIO_InitStruct.Pin = UART_TX_Pin | UART_RX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // /* USART1_TX Init */
    // LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_7, LL_DMA_CHANNEL_4);
    // LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_7,
    //                                 LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    // LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_7,
    // LL_DMA_PRIORITY_LOW); LL_DMA_SetMode(DMA2, LL_DMA_STREAM_7,
    // LL_DMA_MODE_NORMAL); LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_7,
    // LL_DMA_PERIPH_NOINCREMENT); LL_DMA_SetMemoryIncMode(DMA2,
    // LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT); LL_DMA_SetPeriphSize(DMA2,
    // LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE); LL_DMA_SetMemorySize(DMA2,
    // LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE); LL_DMA_DisableFifoMode(DMA2,
    // LL_DMA_STREAM_7);

    // /* USART1_RX Init */
    // LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_4);
    // LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2,
    //                                 LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    // LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2,
    // LL_DMA_PRIORITY_LOW); LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2,
    // LL_DMA_MODE_NORMAL); LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2,
    // LL_DMA_PERIPH_NOINCREMENT); LL_DMA_SetMemoryIncMode(DMA2,
    // LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT); LL_DMA_SetPeriphSize(DMA2,
    // LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE); LL_DMA_SetMemorySize(DMA2,
    // LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE); LL_DMA_DisableFifoMode(DMA2,
    // LL_DMA_STREAM_2);

    // NVIC_SetPriority(USART1_IRQn,
    //                  NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    // NVIC_EnableIRQ(USART1_IRQn);

    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    auto status = LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_ConfigSyncMode(USART1);
    LL_USART_Enable(USART1);

    if (status == SUCCESS) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::deinitUartDebugPort() {
#ifdef STM32L4P5xx
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::sendUartDebug1Byte(uint8_t data) {
#if defined(STM32L4P5xx) || defined(STM32F411xE)
    LL_USART_TransmitData8(USART1, data);
    while (LL_USART_IsActiveFlag_TC(USART1) == 0) {
    }
    LL_USART_ClearFlag_TC(USART1);
    return hal::HalStatus::SUCCESS;
#endif  // if defined(STM32L4P5xx) || defined(STM32F411xE)

#ifdef LINUX
    if (plt::Observer::getInstance()->getImuData(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef LINUX
}

hal::HalStatus hal::sendUartDebugNByte(const char *data, const int len) {
#if defined(STM32L4P5xx) || defined(STM32F411xE)
    for (int i = 0; i < len; ++i) hal::sendUartDebug1Byte((uint8_t)data[i]);
    return hal::HalStatus::SUCCESS;
#endif  // if defined(STM32L4P5xx) || defined(STM32F411xE)

#ifdef LINUX
    if (plt::Observer::getInstance()->getImuData(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef LINUX
}

hal::HalStatus hal::sendUartDebugDmaNByte(const char *data, const int len) {
#ifdef STM32L4P5xx
    static char debugDmaTxBuffer[DEBUG_DMA_TX_BUFFER_SIZE];

    for (int i = 0; i < len; ++i) debugDmaTxBuffer[i] = data[i];
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_ConfigAddresses(
        DMA1, LL_DMA_CHANNEL_2, (uint32_t)debugDmaTxBuffer,
        LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT),
        LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, len);
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_USART1_TX);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_USART_EnableDMAReq_TX(USART1);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX
}
