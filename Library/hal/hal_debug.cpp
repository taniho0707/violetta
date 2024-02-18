//******************************************************************************
// @addtogroup HAL
// @file       hal_debug.cpp
// @brief      デバッグ用機能(UARTログ等)
//******************************************************************************

#include "hal_debug.h"

hal::HalStatus hal::initUartDebugPort() {
#ifdef STM32L4P5xx
    return HalStatus::SUCCESS;
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
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::sendUartDebug1Byte(uint8_t data) {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    LL_USART_TransmitData8(USART1, data);
    while (LL_USART_IsActiveFlag_TC(USART1) == 0) {
    }
    LL_USART_ClearFlag_TC(USART1);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    if (plt::Observer::getInstance()->getImuData(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef LINUX
}

hal::HalStatus hal::sendUartDebugNByte(uint8_t *data, const int len) {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    for (int i = 0; i < len; ++i) hal::sendUartDebug1Byte(data[i]);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    if (plt::Observer::getInstance()->getImuData(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef LINUX
}

hal::HalStatus hal::sendUartDebugNByte(char *data, const int len) {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    for (int i = 0; i < len; ++i) hal::sendUartDebug1Byte((uint8_t)data[i]);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    if (plt::Observer::getInstance()->getImuData(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef LINUX
}
