//******************************************************************************
// @addtogroup HAL
// @file       hal_led.cpp
// @brief      LED点灯制御
//******************************************************************************

#include "hal_led.h"

#if defined(MOUSE_LAZULI) && defined(STM32L4P5xx)
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_i2c.h"
#include "stm32l4xx_ll_rcc.h"
#endif  // ifdef MOUSE_LAZULI

#ifdef STM32F411xE
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_rcc.h"
#endif  // ifdef STM32F411xE

hal::HalStatus hal::initLedPort(InitializeType type) {
#ifdef LINUX
    return HalStatus::SUCCESS;
#endif

#ifdef MOUSE_VIOLETTA
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_channel;

    gpio_port = GPIOB;
    gpio_channel = GPIO_PIN_8;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    LL_GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_8;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    LL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_3;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    LL_GPIO_Init(GPIOH, &GPIO_InitStructure);

    switch (num) {
        case hal::LedNumbers::FRONT1:
            break;
        case hal::LedNumbers::FRONT2:
            break;
        case hal::LedNumbers::TOP1:
            GPIO_InitStructure.Pin = GPIO_PIN_10;
            GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStructure.Pull = GPIO_NOPULL;
            GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
            LL_GPIO_Init(GPIOA, &GPIO_InitStructure);
            break;
        case hal::LedNumbers::TOP2:
            GPIO_InitStructure.Pin = GPIO_PIN_9;
            GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStructure.Pull = GPIO_NOPULL;
            GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
            LL_GPIO_Init(GPIOA, &GPIO_InitStructure);
            break;
    }
    return HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    LL_I2C_InitTypeDef I2C_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_RCC_SetI2CClockSource(LL_RCC_I2C2_CLKSOURCE_PCLK1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    if (type == InitializeType::Sync) {
        // I2C2 GPIO Configuration
        // PB10   ------> I2C2_SCL
        // PB11   ------> I2C2_SDA
        GPIO_InitStruct.Pin = PRESSURE_SCL_Pin | PRESSURE_SDA_Pin;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
        LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        LL_I2C_EnableAutoEndMode(I2C2);
        LL_I2C_DisableOwnAddress2(I2C2);
        LL_I2C_DisableGeneralCall(I2C2);
        LL_I2C_EnableClockStretching(I2C2);
        I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
        I2C_InitStruct.Timing = 0x009034B6;
        I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
        I2C_InitStruct.DigitalFilter = 0;
        I2C_InitStruct.OwnAddress1 = 0;
        I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
        I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
        LL_I2C_Init(I2C2, &I2C_InitStruct);
        LL_I2C_SetOwnAddress2(I2C2, 0, LL_I2C_OWNADDRESS2_NOMASK);

        LL_I2C_Enable(I2C2);

        // read chipid
        uint8_t chipid;
        receiveLedI2cCommand(LedDriverCommands::CHIP_ID, &chipid);
        if (chipid != 0xA9) {
            return hal::HalStatus::ERROR;
        }

        // configure driver
        transmitLedI2cCommand(LedDriverCommands::LED1_SET, 0x03);  // FLEFT  BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED2_SET, 0x03);  // LEFT   BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED3_SET, 0x03);  // FRONT  BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED4_SET, 0x03);  // RIGHT  BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED5_SET, 0x03);  // FRIGHT BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED6_SET, 0x09);  // WHITE 5mA
        transmitLedI2cCommand(LedDriverCommands::LED7_SET, 0x03);  // 2mA
        transmitLedI2cCommand(LedDriverCommands::LED8_SET, 0x00);  // GREEN 0.5mA
        transmitLedI2cCommand(LedDriverCommands::LED9_SET, 0x03);  // BLUE 2mA

        return hal::HalStatus::SUCCESS;
    } else if (type == InitializeType::Async) {
        // I2C2 GPIO Configuration
        // PB10   ------> I2C2_SCL
        // PB11   ------> I2C2_SDA
        GPIO_InitStruct.Pin = PRESSURE_SCL_Pin | PRESSURE_SDA_Pin;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
        LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        LL_I2C_EnableAutoEndMode(I2C2);
        LL_I2C_DisableOwnAddress2(I2C2);
        LL_I2C_DisableGeneralCall(I2C2);
        LL_I2C_EnableClockStretching(I2C2);
        I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
        I2C_InitStruct.Timing = 0x009034B6;
        I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
        I2C_InitStruct.DigitalFilter = 0;
        I2C_InitStruct.OwnAddress1 = 0;
        I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
        I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
        LL_I2C_Init(I2C2, &I2C_InitStruct);
        LL_I2C_SetOwnAddress2(I2C2, 0, LL_I2C_OWNADDRESS2_NOMASK);

        NVIC_SetPriority(I2C2_EV_IRQn, 6);
        NVIC_EnableIRQ(I2C2_EV_IRQn);
        NVIC_SetPriority(I2C2_ER_IRQn, 6);
        NVIC_EnableIRQ(I2C2_ER_IRQn);

        LL_I2C_EnableIT_RX(I2C2);
        LL_I2C_EnableIT_NACK(I2C2);
        LL_I2C_EnableIT_ERR(I2C2);
        LL_I2C_EnableIT_STOP(I2C2);

        LL_I2C_Enable(I2C2);

        // read chipid
        uint8_t chipid;
        receiveLedI2cCommand(LedDriverCommands::CHIP_ID, &chipid);
        if (chipid != 0xA9) {
            return hal::HalStatus::ERROR;
        }

        // configure driver
        transmitLedI2cCommand(LedDriverCommands::LED1_SET, 0x03);  // FLEFT  BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED2_SET, 0x03);  // LEFT   BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED3_SET, 0x03);  // FRONT  BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED4_SET, 0x03);  // RIGHT  BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED5_SET, 0x03);  // FRIGHT BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED6_SET, 0x09);  // WHITE 5mA
        transmitLedI2cCommand(LedDriverCommands::LED7_SET, 0x03);  // 2mA
        transmitLedI2cCommand(LedDriverCommands::LED8_SET, 0x05);  // 3mA
        transmitLedI2cCommand(LedDriverCommands::LED9_SET, 0x00);  // GREEN 0.5mA

        return hal::HalStatus::SUCCESS;
    } else if (type == InitializeType::Dma) {
        LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_4, LL_DMAMUX_REQ_I2C1_RX);
        LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
        LL_DMA_SetChannelPriorityLevel(DMA2, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);
        LL_DMA_SetMode(DMA2, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);
        LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);
        LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);
        LL_DMA_SetPeriphSize(DMA2, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);
        LL_DMA_SetMemorySize(DMA2, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

        LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_I2C1_TX);
        LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
        LL_DMA_SetChannelPriorityLevel(DMA2, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);
        LL_DMA_SetMode(DMA2, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);
        LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
        LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
        LL_DMA_SetPeriphSize(DMA2, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);
        LL_DMA_SetMemorySize(DMA2, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

        NVIC_SetPriority(DMA2_Channel4_IRQn, 7);
        NVIC_EnableIRQ(DMA2_Channel4_IRQn);
        NVIC_SetPriority(DMA2_Channel3_IRQn, 7);
        NVIC_EnableIRQ(DMA2_Channel3_IRQn);
        LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_4);
        LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_4);
        LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_3);
        LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_3);

        LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_4);
        LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_3);

        NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));
        NVIC_EnableIRQ(I2C2_EV_IRQn);
        NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));
        NVIC_EnableIRQ(I2C2_ER_IRQn);

        // I2C2 GPIO Configuration
        // PB10   ------> I2C2_SCL
        // PB11   ------> I2C2_SDA
        GPIO_InitStruct.Pin = PRESSURE_SCL_Pin | PRESSURE_SDA_Pin;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
        LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        LL_I2C_EnableAutoEndMode(I2C2);
        LL_I2C_DisableOwnAddress2(I2C2);
        LL_I2C_DisableGeneralCall(I2C2);
        LL_I2C_EnableClockStretching(I2C2);
        I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
        I2C_InitStruct.Timing = 0x009034B6;
        I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
        I2C_InitStruct.DigitalFilter = 0;
        I2C_InitStruct.OwnAddress1 = 0;
        I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
        I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
        LL_I2C_Init(I2C2, &I2C_InitStruct);
        LL_I2C_SetOwnAddress2(I2C2, 0, LL_I2C_OWNADDRESS2_NOMASK);

        // read chipid
        uint8_t chipid;
        receiveLedI2cCommand(LedDriverCommands::CHIP_ID, &chipid);
        if (chipid != 0xA9) {
            return hal::HalStatus::ERROR;
        }

        // configure driver
        transmitLedI2cCommand(LedDriverCommands::LED1_SET, 0x03);  // FLEFT  BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED2_SET, 0x03);  // LEFT   BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED3_SET, 0x03);  // FRONT  BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED4_SET, 0x03);  // RIGHT  BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED5_SET, 0x03);  // FRIGHT BLUE 2mA
        transmitLedI2cCommand(LedDriverCommands::LED6_SET, 0x09);  // WHITE 5mA
        transmitLedI2cCommand(LedDriverCommands::LED7_SET, 0x03);  // 2mA
        transmitLedI2cCommand(LedDriverCommands::LED8_SET, 0x05);  // 3mA
        transmitLedI2cCommand(LedDriverCommands::LED9_SET, 0x00);  // GREEN 0.5mA

        LL_I2C_Disable(I2C2);
        LL_I2C_EnableIT_STOP(I2C2);
        LL_I2C_EnableIT_RX(I2C2);
        LL_I2C_Enable(I2C2);

        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::INVALID_PARAMS;
    }

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#ifdef STM32F411xE
    // LED0: BLUE   PA12
    // LED1: GREEN  PA11
    // LED2: YELLOW PH0
    // LED3: RED    PH1

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    ErrorStatus status;

    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    LL_GPIO_ResetOutputPin(GPIOH, LED2_Pin | LED3_Pin);
    LL_GPIO_ResetOutputPin(GPIOA, LED1_Pin | LED0_Pin);

    GPIO_InitStruct.Pin = LED2_Pin | LED3_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    status = LL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    if (status != ErrorStatus::SUCCESS) return hal::HalStatus::ERROR;

    GPIO_InitStruct.Pin = LED1_Pin | LED0_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    status = LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    if (status != ErrorStatus::SUCCESS) return hal::HalStatus::ERROR;

    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE
}

hal::HalStatus hal::deinitLedPort() {
#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif

#ifdef MOUSE_VIOLETTA
    switch (num) {
        case LedNumbers::FRONT1:
            break;
        case LedNumbers::FRONT2:
            break;
        case LedNumbers::TOP1:
            HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10);
            break;
        case LedNumbers::TOP2:
            HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);
            break;
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE
}

hal::HalStatus hal::transmitLedI2cData(uint8_t device_id, uint8_t* pdata, uint8_t size) {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif

#ifdef MOUSE_LAZULI
    while (LL_I2C_IsActiveFlag_BUSY(I2C2) == SET);

    LL_I2C_HandleTransfer(I2C2, device_id, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    for (uint8_t i = 0; i < size; i++) {
        // while (LL_I2C_IsActiveFlag_TXE(I2C1) == RESET);
        while (LL_I2C_IsActiveFlag_TXIS(I2C2) == RESET);
        LL_I2C_TransmitData8(I2C2, pdata[i]);
    }

    while (LL_I2C_IsActiveFlag_STOP(I2C2) == RESET);
    LL_I2C_ClearFlag_STOP(I2C2);

    // (I2C2->CR2 &=
    //  (uint32_t) ~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES
    //  |
    //                          I2C_CR2_RELOAD | I2C_CR2_RD_WRN)));

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::receiveLedI2cData(uint8_t device_id, uint8_t* pdata, uint8_t size) {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif

#ifdef MOUSE_LAZULI
    while (LL_I2C_IsActiveFlag_BUSY(I2C2) == SET);

    LL_I2C_HandleTransfer(I2C2, device_id, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

    for (uint8_t i = 0; i < size; i++) {
        while (LL_I2C_IsActiveFlag_RXNE(I2C2) == RESET);
        pdata[i] = LL_I2C_ReceiveData8(I2C2);
    }

    while (LL_I2C_IsActiveFlag_STOP(I2C2) == RESET);
    LL_I2C_ClearFlag_STOP(I2C2);

    // (I2C1->CR2 &=
    //  (uint32_t) ~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES
    //  |
    //                          I2C_CR2_RELOAD | I2C_CR2_RD_WRN)));
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::transmitLedI2cDataDma(uint8_t device_id, uint8_t* pdata, uint8_t size) {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif

#ifdef MOUSE_LAZULI
    static uint8_t transmit_buffer[10];  // FIXME: バッファーサイズを適切に設定する
    for (uint8_t i = 0; i < size; i++) {
        transmit_buffer[i] = pdata[i];
    }

    while (LL_I2C_IsActiveFlag_BUSY(I2C2) == SET);

    LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_3);
    LL_I2C_HandleTransfer(I2C2, device_id, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_3, size);
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_3, (uint32_t)pdata, (uint32_t)LL_I2C_DMA_GetRegAddr(I2C2, LL_I2C_DMA_REG_DATA_TRANSMIT),
                           LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_CHANNEL_4));
    LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_3, size);
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_3, (uint32_t)transmit_buffer, (uint32_t)LL_I2C_DMA_GetRegAddr(I2C2, LL_I2C_DMA_REG_DATA_TRANSMIT),
                           LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_CHANNEL_3));

    LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_3);
    LL_I2C_EnableDMAReq_TX(I2C2);

    // while (LL_DMA_IsActiveFlag_TC1(DMA2) == RESET);
    // LL_DMA_ClearFlag_TC1(DMA2);

    // while (LL_I2C_IsActiveFlag_STOP(I2C1) == RESET);
    // LL_I2C_ClearFlag_STOP(I2C1);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::transmitLedI2cCommand(hal::LedDriverCommands cmd, uint8_t data) {
    uint8_t tx_buffer[2];
    tx_buffer[0] = static_cast<uint8_t>(cmd);
    tx_buffer[1] = data;
    transmitLedI2cData(LED_DRIVER_ADDR, tx_buffer, 2);
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::receiveLedI2cCommand(hal::LedDriverCommands cmd, uint8_t* data) {
    uint8_t tx_buffer = static_cast<uint8_t>(cmd);
    transmitLedI2cData(LED_DRIVER_ADDR, &tx_buffer, 1);
    receiveLedI2cData(LED_DRIVER_ADDR, data, 1);
    return hal::HalStatus::SUCCESS;
}

// TODO: LedクラスをDMAに対応させる
hal::HalStatus hal::setLedDma(LedNumbers* nums, uint8_t size) {
#ifdef LINUX
    return hal::HalStatus::NOIMPLEMENT;
#endif

#ifdef MOUSE_LAZULI
    uint8_t current_en;
    receiveLedI2cCommand(LedDriverCommands::LED_EN_IDVD, &current_en);

    uint8_t en = 0;
    for (uint8_t i = 0; i < size; i++) {
        switch (nums[i]) {
            case hal::LedNumbers::LEFT:
                en |= 0x80;
                break;
            case hal::LedNumbers::FRONT:
                en |= 0x40;
                break;
            case hal::LedNumbers::RIGHT:
                en |= 0x20;
                break;
            case hal::LedNumbers::FRONTR:
                en |= 0x10;
                break;
            case hal::LedNumbers::MIDDLE1:
                en |= 0x08;
                break;
            case hal::LedNumbers::MIDDLE2:
                en |= 0x04;
                break;
            case hal::LedNumbers::MIDDLE3:
                en |= 0x02;
                break;
            case hal::LedNumbers::MIDDLE4:
                en |= 0x01;
                break;
            case hal::LedNumbers::FRONTL:
                en |= 0x00;
                // TODO: これだけレジスタが違うため修正
                break;
            case hal::LedNumbers::FLAG:
                return hal::HalStatus::NOIMPLEMENT;
                break;
            // case hal::LedNumbers::ALL:
            //     hal::setLedDma(nums, 8);
            //     break;
            default:
                return hal::HalStatus::INVALID_PARAMS;
                break;
        }
    }
    transmitLedI2cCommand(LedDriverCommands::LED_EN_IDVD, current_en | en);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::onLed(hal::LedNumbers num) {
#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif

#ifdef MOUSE_VIOLETTA
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_channel;
    switch (num) {
        case hal::LedNumbers::FRONT1:
            gpio_port = GPIOB;
            gpio_channel = GPIO_PIN_8;
            break;
        case hal::LedNumbers::FRONT2:
            gpio_port = GPIOH;
            gpio_channel = GPIO_PIN_3;
            break;
        case hal::LedNumbers::TOP1:
            gpio_port = GPIOA;
            gpio_channel = GPIO_PIN_10;
            break;
        case hal::LedNumbers::TOP2:
            gpio_port = GPIOA;
            gpio_channel = GPIO_PIN_9;
            break;
    }
    HAL_GPIO_WritePin(gpio_port, gpio_channel, GPIO_PIN_SET);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    uint8_t current_en;
    receiveLedI2cCommand(LedDriverCommands::LED_EN_IDVD, &current_en);
    uint8_t current_config;
    receiveLedI2cCommand(LedDriverCommands::LED_CONFIG, &current_config);

    if ((num & hal::LedNumbers::FRONTL) == hal::LedNumbers::FRONTL) {
        current_config |= 0x01;
        transmitLedI2cCommand(LedDriverCommands::LED_CONFIG, 0x01);
    }
    if ((num & hal::LedNumbers::LEFT) == hal::LedNumbers::LEFT) {
        current_en |= 0x80;
    }
    if ((num & hal::LedNumbers::FRONT) == hal::LedNumbers::FRONT) {
        current_en |= 0x40;
    }
    if ((num & hal::LedNumbers::RIGHT) == hal::LedNumbers::RIGHT) {
        current_en |= 0x20;
    }
    if ((num & hal::LedNumbers::FRONTR) == hal::LedNumbers::FRONTR) {
        current_en |= 0x10;
    }
    if ((num & hal::LedNumbers::MIDDLE1) == hal::LedNumbers::MIDDLE1) {
        current_en |= 0x08;
    }
    if ((num & hal::LedNumbers::MIDDLE2) == hal::LedNumbers::MIDDLE2) {
        current_en |= 0x04;
    }
    if ((num & hal::LedNumbers::MIDDLE3) == hal::LedNumbers::MIDDLE3) {
        current_en |= 0x02;
    }
    if ((num & hal::LedNumbers::MIDDLE4) == hal::LedNumbers::MIDDLE4) {
        current_en |= 0x01;
    }
    transmitLedI2cCommand(LedDriverCommands::LED_EN_IDVD, current_en);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    switch (num) {
        case hal::LedNumbers::RED:
            LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin);
            break;
        case hal::LedNumbers::YELLOW:
            LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
            break;
        case hal::LedNumbers::GREEN:
            LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
            break;
        case hal::LedNumbers::BLUE:
            LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
            break;
        case hal::LedNumbers::ALL:
            hal::onLed(hal::LedNumbers::RED);
            hal::onLed(hal::LedNumbers::YELLOW);
            hal::onLed(hal::LedNumbers::GREEN);
            hal::onLed(hal::LedNumbers::BLUE);
            break;
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}

hal::HalStatus hal::offLed(hal::LedNumbers num) {
#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif

#ifdef MOUSE_VIOLETTA
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_channel;
    switch (num) {
        case hal::LedNumbers::FRONT1:
            gpio_port = GPIOB;
            gpio_channel = GPIO_PIN_8;
            break;
        case hal::LedNumbers::FRONT2:
            gpio_port = GPIOH;
            gpio_channel = GPIO_PIN_3;
            break;
        case hal::LedNumbers::TOP1:
            gpio_port = GPIOA;
            gpio_channel = GPIO_PIN_10;
            break;
        case hal::LedNumbers::TOP2:
            gpio_port = GPIOA;
            gpio_channel = GPIO_PIN_9;
            break;
    }
    HAL_GPIO_WritePin(gpio_port, gpio_channel, GPIO_PIN_RESET);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    uint8_t current_en;
    receiveLedI2cCommand(LedDriverCommands::LED_EN_IDVD, &current_en);
    uint8_t current_config;
    receiveLedI2cCommand(LedDriverCommands::LED_CONFIG, &current_config);

    if ((num & hal::LedNumbers::FRONTL) == hal::LedNumbers::FRONTL) {
        current_config &= 0xFE;
        transmitLedI2cCommand(LedDriverCommands::LED_CONFIG, 0x00);
    }
    if ((num & hal::LedNumbers::LEFT) == hal::LedNumbers::LEFT) {
        current_en &= 0x7F;
    }
    if ((num & hal::LedNumbers::FRONT) == hal::LedNumbers::FRONT) {
        current_en &= 0xBF;
    }
    if ((num & hal::LedNumbers::RIGHT) == hal::LedNumbers::RIGHT) {
        current_en &= 0xDF;
    }
    if ((num & hal::LedNumbers::FRONTR) == hal::LedNumbers::FRONTR) {
        current_en &= 0xEF;
    }
    if ((num & hal::LedNumbers::MIDDLE1) == hal::LedNumbers::MIDDLE1) {
        current_en &= 0xF7;
    }
    if ((num & hal::LedNumbers::MIDDLE2) == hal::LedNumbers::MIDDLE2) {
        current_en &= 0xFB;
    }
    if ((num & hal::LedNumbers::MIDDLE3) == hal::LedNumbers::MIDDLE3) {
        current_en &= 0xFD;
    }
    if ((num & hal::LedNumbers::MIDDLE4) == hal::LedNumbers::MIDDLE4) {
        current_en &= 0xFE;
    }
    transmitLedI2cCommand(LedDriverCommands::LED_EN_IDVD, current_en);

    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_LAZULI

#if defined(MOUSE_ZIRCONIA2KAI) && defined(STM32F411xE)
    switch (num) {
        case hal::LedNumbers::RED:
            LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
            break;
        case hal::LedNumbers::YELLOW:
            LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
            break;
        case hal::LedNumbers::GREEN:
            LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
            break;
        case hal::LedNumbers::BLUE:
            LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
            break;
        case hal::LedNumbers::ALL:
            hal::offLed(hal::LedNumbers::RED);
            hal::offLed(hal::LedNumbers::YELLOW);
            hal::offLed(hal::LedNumbers::GREEN);
            hal::offLed(hal::LedNumbers::BLUE);
            break;
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef MOUSE_ZIRCONIA2KAI
}
