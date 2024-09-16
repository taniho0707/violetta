//******************************************************************************
// @addtogroup HAL
// @file       hal_led.cpp
// @brief      LED点灯制御
//******************************************************************************

#include "hal_led.h"

hal::HalStatus hal::initLedPort(InitializeType type) {
    return HalStatus::SUCCESS;
}

hal::HalStatus hal::deinitLedPort() {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::transmitLedI2cData(uint8_t device_id, uint8_t* pdata, uint8_t size) {
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::receiveLedI2cData(uint8_t device_id, uint8_t* pdata, uint8_t size) {
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::transmitLedI2cDataDma(uint8_t device_id, uint8_t* pdata, uint8_t size) {
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
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::onLed(hal::LedNumbers num) {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::offLed(hal::LedNumbers num) {
    return hal::HalStatus::SUCCESS;
}
