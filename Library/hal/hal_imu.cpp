//******************************************************************************
// @addtogroup HAL
// @file       hal_imu.cpp
// @brief      IMU制御
//******************************************************************************

#include "hal_imu.h"

#include <stdint.h>

#include "hal_conf.h"

#ifdef STM32L4P5xx
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_spi.h"
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_utils.h"
#endif  // ifdef STM32F411xE

hal::HalStatus hal::initImuPort() {
#ifdef LINUX
    return HalStatus::SUCCESS;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    LL_SPI_InitTypeDef SPI_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    /** SPI3 GPIO Configuration
    PB3 (JTDO/TRACESWO)   ------> SPI3_SCK
    PB4 (NJTRST)   ------> SPI3_MISO
    PB5   ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = IMU_FRAM_SCK_Pin | IMU_FRAM_MISO_Pin | IMU_FRAM_MOSI_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = IMU_CS_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    LL_GPIO_SetOutputPin(GPIOA, IMU_CS_Pin);

    // LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_SPI3_TX);
    // LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3,
    //                                 LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    // LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3,
    //                                LL_DMA_PRIORITY_HIGH);
    // LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);
    // LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3,
    // LL_DMA_PERIPH_NOINCREMENT); LL_DMA_SetMemoryIncMode(DMA1,
    // LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT); LL_DMA_SetPeriphSize(DMA1,
    // LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE); LL_DMA_SetMemorySize(DMA1,
    // LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

    // LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMAMUX_REQ_SPI3_RX);
    // LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4,
    //                                 LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    // LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4,
    //                                LL_DMA_PRIORITY_HIGH);
    // LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);
    // LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4,
    // LL_DMA_PERIPH_NOINCREMENT); LL_DMA_SetMemoryIncMode(DMA1,
    // LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT); LL_DMA_SetPeriphSize(DMA1,
    // LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE); LL_DMA_SetMemorySize(DMA1,
    // LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

    // NVIC_SetPriority(SPI3_IRQn,
    //                  NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    // NVIC_EnableIRQ(SPI3_IRQn);
    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 7;
    auto status = LL_SPI_Init(SPI3, &SPI_InitStruct);
    LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_DisableNSSPulseMgt(SPI3);
    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
    LL_SPI_Enable(SPI3);

    if (status == SUCCESS) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef MOUSE_LAZULI

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
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
    // SPI_InitStruct.NSS = LL_SPI_NSS_HARD_OUTPUT;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;

    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
    // SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;  //
    // 一時的に変更
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
}

hal::HalStatus hal::deinitImuPort() {
#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE
}

hal::HalStatus hal::read8bitImuSync(uint8_t address, uint8_t& data) {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_LAZULI
    LL_GPIO_ResetOutputPin(GPIOA, IMU_CS_Pin);
    uint8_t tx_data = address | 0x80;
    while (LL_SPI_IsActiveFlag_TXE(SPI3) == RESET);
    LL_SPI_TransmitData8(SPI3, tx_data);
    while (LL_SPI_IsActiveFlag_RXNE(SPI3) == RESET);
    LL_SPI_ReceiveData8(SPI3);
    tx_data = 0x00;
    while (LL_SPI_IsActiveFlag_TXE(SPI3) == RESET);
    LL_SPI_TransmitData8(SPI3, tx_data);
    while (LL_SPI_IsActiveFlag_RXNE(SPI3) == RESET);
    data = LL_SPI_ReceiveData8(SPI3);
    while (LL_SPI_IsActiveFlag_BSY(SPI3) == SET);
    LL_GPIO_SetOutputPin(GPIOA, IMU_CS_Pin);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    // if (LL_SPI_IsActiveFlag_RXNE(SPI2)) LL_SPI_ReceiveData16(SPI2);
    // if (!LL_SPI_IsEnabled(SPI2)) LL_SPI_Enable(SPI2);

    LL_GPIO_ResetOutputPin(GPIOB, IMU_CS2_Pin);
    uint8_t tx_data = address | 0x80;
    while (LL_SPI_IsActiveFlag_TXE(SPI2) == RESET);
    LL_SPI_TransmitData8(SPI2, tx_data);
    while (LL_SPI_IsActiveFlag_RXNE(SPI2) == RESET);
    LL_SPI_ReceiveData8(SPI2);
    tx_data = 0x00;
    while (LL_SPI_IsActiveFlag_TXE(SPI2) == RESET);
    LL_SPI_TransmitData8(SPI2, tx_data);
    while (LL_SPI_IsActiveFlag_RXNE(SPI2) == RESET);
    data = LL_SPI_ReceiveData8(SPI2);
    while (LL_SPI_IsActiveFlag_BSY(SPI2) == SET);
    LL_GPIO_SetOutputPin(GPIOB, IMU_CS2_Pin);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::write8bitImuSync(uint8_t address, uint8_t data) {
#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif  // ifdef LINUX

#ifdef MOUSE_LAZULI
    LL_GPIO_ResetOutputPin(GPIOA, IMU_CS_Pin);
    uint8_t tx_data = address;
    while (LL_SPI_IsActiveFlag_TXE(SPI3) == RESET);
    LL_SPI_TransmitData8(SPI3, tx_data);
    while (LL_SPI_IsActiveFlag_RXNE(SPI3) == RESET);
    LL_SPI_ReceiveData8(SPI3);
    tx_data = data;
    while (LL_SPI_IsActiveFlag_TXE(SPI3) == RESET);
    LL_SPI_TransmitData8(SPI3, tx_data);
    while (LL_SPI_IsActiveFlag_RXNE(SPI3) == RESET);
    LL_SPI_ReceiveData8(SPI3);
    while (LL_SPI_IsActiveFlag_BSY(SPI3) == SET);
    LL_GPIO_SetOutputPin(GPIOA, IMU_CS_Pin);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    // if (LL_SPI_IsActiveFlag_RXNE(SPI2)) LL_SPI_ReceiveData16(SPI2);
    // if (!LL_SPI_IsEnabled(SPI2)) LL_SPI_Enable(SPI2);

    LL_GPIO_ResetOutputPin(GPIOB, IMU_CS2_Pin);
    uint8_t tx_data = address;
    while (LL_SPI_IsActiveFlag_TXE(SPI2) == RESET);
    LL_SPI_TransmitData8(SPI2, tx_data);
    while (LL_SPI_IsActiveFlag_RXNE(SPI2) == RESET);
    LL_SPI_ReceiveData8(SPI2);
    tx_data = data;
    while (LL_SPI_IsActiveFlag_TXE(SPI2) == RESET);
    LL_SPI_TransmitData8(SPI2, tx_data);
    while (LL_SPI_IsActiveFlag_RXNE(SPI2) == RESET);
    LL_SPI_ReceiveData8(SPI2);
    while (LL_SPI_IsActiveFlag_BSY(SPI2) == SET);
    LL_GPIO_SetOutputPin(GPIOB, IMU_CS2_Pin);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::whoamiImu() {
#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#if defined(MOUSE_LAZULI) || defined(MOUSE_ZIRCONIA2KAI)
    uint8_t rx_data;
    if (read8bitImuSync(static_cast<uint8_t>(hal::GyroCommands::WHO_AM_I), rx_data) == hal::HalStatus::SUCCESS) {
        if (rx_data == 0x6B) {
            return hal::HalStatus::SUCCESS;
        } else {
            return hal::HalStatus::ERROR;
        }
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef MOUSE_LAZULI || MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::setImuConfig() {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    // write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL3_C),
    //                  0x01);  // Software reset
    // LL_mDelay(100);
    // while (true) {
    //     uint8_t rx_data;
    //     read8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL3_C),
    //     rx_data); if ((rx_data & 0x01) == 0) {
    //         break;
    //     }
    // }

    write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL4_C),
                     0x06);  // I2C_Disable, GyroLPF1_Enable
    write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL9_XL),
                     0xE2);  // I3C Disable

    write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL1_XL),
                     0xAE);  // KOHIRO: ACC 6.66kHz, +-8g
    write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL8_XL),
                     0xC0);  // KOHIRO: ACC LPF 6.66kHz/400
    write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL2_G),
                     0xAD);  // KOHIRO: GYRO 6.66kHz, +-4000dps

    // write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL1_XL),
    //                  0x8C);  // ACC: 1.66kHz, +-8g
    // write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL2_G),
    //                  0x8D);  // GYRO: 1.66kHz, +-4000dps
    // // MEMO: FS_4000 has to be set to 0 when the OIS chain is ON
    // (inCTRL1_OIS)
    // write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL5_C),
    //                  0x00);  // Default
    // write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL6_C),
    //                  0x80);  // USR_OFF_W=0, FTYPE=000(GyroLPF1:274Hz)
    // write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL7_G),
    //                  0x00);  // GyroHPF Disable
    // write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL8_XL),
    //                  0x00);  // AccHPF Disable
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL3_C),
                     0x01);  // Software reset
    LL_mDelay(100);
    // while (true) {
    //     uint8_t rx_data;
    //     read8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL3_C),
    //     rx_data); if ((rx_data & 0x01) == 0) {
    //         break;
    //     }
    // }

    write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL4_C),
                     0x06);  // I2C_Disable, GyroLPF1_Enable
    write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL9_XL),
                     0xE2);  // I3C Disable

    write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL1_XL),
                     0xAE);  // KOHIRO: ACC 6.66kHz, +-8g
    write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL8_XL),
                     0xC0);  // KOHIRO: ACC LPF 6.66kHz/400
    write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL2_G),
                     0xAD);  // KOHIRO: GYRO 6.66kHz, +-4000dps

    // write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL1_XL),
    //                  0x8C);  // ACC: 1.66kHz, +-8g
    // write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL2_G),
    //                  0x8D);  // GYRO: 1.66kHz, +-4000dps
    // // MEMO: FS_4000 has to be set to 0 when the OIS chain is ON
    // (inCTRL1_OIS)
    // write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL5_C),
    //                  0x00);  // Default
    // write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL6_C),
    //                  0x80);  // USR_OFF_W=0, FTYPE=000(GyroLPF1:274Hz)
    // write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL7_G),
    //                  0x00);  // GyroHPF Disable
    // write8bitImuSync(static_cast<uint8_t>(GyroCommands::CTRL8_XL),
    //                  0x00);  // AccHPF Disable
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::getImuDataSync(ImuData& data) {
#ifdef LINUX
    if (plt::Observer::getInstance()->getImuData(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
#endif  // ifdef LINUX

#ifdef MOUSE_VIOLETTA
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    uint8_t rx_data_l, rx_data_h;
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUT_TEMP_L), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUT_TEMP_H), rx_data_h);
    data.OUT_TEMP = static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF));
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTX_L_G), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTX_H_G), rx_data_h);
    data.OUT_X_G = static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF));
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTY_L_G), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTY_H_G), rx_data_h);
    data.OUT_Y_G = static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF));
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTZ_L_G), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTZ_H_G), rx_data_h);
    data.OUT_Z_G = static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF));
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTX_L_A), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTX_H_A), rx_data_h);
    data.OUT_X_A = static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF));
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTY_L_A), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTY_H_A), rx_data_h);
    data.OUT_Y_A = static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF));
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTZ_L_A), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTZ_H_A), rx_data_h);
    data.OUT_Z_A = static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF));
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_ZIRCONIA2KAI
    // FIXME: 取得データを int16_t に直す
    uint8_t rx_data_l, rx_data_h;
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUT_TEMP_L), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUT_TEMP_H), rx_data_h);
    data.OUT_TEMP = (static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF))));
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTX_L_G), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTX_H_G), rx_data_h);
    data.OUT_X_G = (static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF))));
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTY_L_G), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTY_H_G), rx_data_h);
    data.OUT_Y_G = (static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF))));
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTZ_L_G), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTZ_H_G), rx_data_h);
    data.OUT_Z_G = (static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF))));
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTX_L_A), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTX_H_A), rx_data_h);
    data.OUT_X_A = (static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF))));
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTY_L_A), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTY_H_A), rx_data_h);
    data.OUT_Y_A = (static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF))));
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTZ_L_A), rx_data_l);
    read8bitImuSync(static_cast<uint8_t>(GyroCommands::OUTZ_H_A), rx_data_h);
    data.OUT_Z_A = (static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(rx_data_h) << 8) | (static_cast<uint16_t>(rx_data_l) & 0x00FF))));
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}
