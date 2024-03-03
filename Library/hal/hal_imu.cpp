//******************************************************************************
// @addtogroup HAL
// @file       hal_imu.cpp
// @brief      IMU制御
//******************************************************************************

#include "hal_imu.h"

#include <stdint.h>

#include "hal_conf.h"
#include "stm32f411xe.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"

hal::HalStatus hal::initImuPort() {
#ifdef STM32L4P5xx
    return HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE

    LL_SPI_InitTypeDef SPI_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    /** SPI2 GPIO Configuration
    PB12   ------> SPI2_NSS
    PB13   ------> SPI2_SCK
    PB14   ------> SPI2_MISO
    PB15   ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = IMU_SCK2_Pin | IMU_MISO2_Pin | IMU_MOSI2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = IMU_CS2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    LL_GPIO_SetOutputPin(GPIOB, IMU_CS2_Pin);

    // /* SPI2_DMA SPI2_TX Init */
    // LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_4,
    // LL_DMA_CHANNEL_0); LL_DMA_SetDataTransferDirection(DMA1,
    // LL_DMA_STREAM_4,
    //                                 LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    // LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_4,
    // LL_DMA_PRIORITY_HIGH); LL_DMA_SetMode(DMA1, LL_DMA_STREAM_4,
    // LL_DMA_MODE_NORMAL); LL_DMA_SetPeriphIncMode(DMA1,
    // LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT);
    // LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_4,
    // LL_DMA_MEMORY_INCREMENT); LL_DMA_SetPeriphSize(DMA1,
    // LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_HALFWORD);
    // LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_4,
    // LL_DMA_MDATAALIGN_HALFWORD); LL_DMA_DisableFifoMode(DMA1,
    // LL_DMA_STREAM_4);

    // /* SPI2_DMA SPI2_RX Init */
    // LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_3,
    // LL_DMA_CHANNEL_0); LL_DMA_SetDataTransferDirection(DMA1,
    // LL_DMA_STREAM_3,
    //                                 LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    // LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_3,
    // LL_DMA_PRIORITY_HIGH); LL_DMA_SetMode(DMA1, LL_DMA_STREAM_3,
    // LL_DMA_MODE_NORMAL); LL_DMA_SetPeriphIncMode(DMA1,
    // LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);
    // LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_3,
    // LL_DMA_MEMORY_INCREMENT); LL_DMA_SetPeriphSize(DMA1,
    // LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_HALFWORD);
    // LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_3,
    // LL_DMA_MDATAALIGN_HALFWORD); LL_DMA_DisableFifoMode(DMA1,
    // LL_DMA_STREAM_3);

    // /* SPI2 interrupt Init */
    // NVIC_SetPriority(SPI2_IRQn,
    //                  NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
    //                  0, 0));
    // NVIC_EnableIRQ(SPI2_IRQn);

    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
    // SPI_InitStruct.NSS = LL_SPI_NSS_HARD_OUTPUT;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;

    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 10;
    auto status = LL_SPI_Init(SPI2, &SPI_InitStruct);
    LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_Enable(SPI2);

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

hal::HalStatus hal::deinitImuPort() {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::whoamiImu() {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
    if (LL_SPI_IsActiveFlag_RXNE(SPI2)) LL_SPI_ReceiveData16(SPI2);
    if (!LL_SPI_IsEnabled(SPI2)) LL_SPI_Enable(SPI2);

    LL_GPIO_ResetOutputPin(GPIOB, IMU_CS2_Pin);
    uint16_t tx_data = 0x0F00 | 0x8000;  // WHO_AM_I
    LL_SPI_TransmitData16(SPI2, tx_data);
    while (LL_SPI_IsActiveFlag_TXE(SPI2) == RESET)
        ;
    while (LL_SPI_IsActiveFlag_RXNE(SPI2) == RESET)
        ;
    uint16_t rx_data = LL_SPI_ReceiveData16(SPI2);
    LL_GPIO_SetOutputPin(GPIOB, IMU_CS2_Pin);

    if ((rx_data & 0xFF) == 0x6B) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX
}

hal::HalStatus hal::getImuDataSync(ImuData& data) {
#ifdef STM32L4P5xx
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
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
