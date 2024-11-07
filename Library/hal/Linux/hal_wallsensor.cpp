//******************************************************************************
// @addtogroup HAL
// @file       hal_wallsensor.cpp
// @brief      壁センサー制御
//******************************************************************************

#include "hal_wallsensor.h"

namespace hal {
hal::HalStatus readwriteWallSensorSpiSync(uint16_t tx, uint16_t& rx);
}

hal::HalStatus hal::initWallSensorPort() {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::deinitWallSensorPort() {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::setWallSensorLedOn(WallSensorNumbers n) {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::setWallSensorLedOff(WallSensorNumbers n) {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::setWallSensorLedOff() {
    // if (plt::Observer::getInstance()->getWallSensorData(data)) {
    //     return hal::HalStatus::SUCCESS;
    // } else {
    //     return hal::HalStatus::ERROR;
    // }
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::setWallSensorChargeStart() {
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::setWallSensorChargeStop() {
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::startWallSensorConversion() {
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::setWallSensorAdcSelect(WallSensorNumbers n) {
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
    // if (plt::Observer::getInstance()->getWallSensorData(data)) {
    //     return hal::HalStatus::SUCCESS;
    // } else {
    //     return hal::HalStatus::ERROR;
    // }
    return hal::HalStatus::NOIMPLEMENT;
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

hal::HalStatus hal::readwriteWallSensorSpiSync(uint16_t tx, uint16_t& rx) {
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::sendWallSensorDataSpiSync(uint16_t& data) {
#ifdef MOUSE_LAZULI_SENSOR
    LL_SPI_TransmitData16(SPI1, data);

    return hal::HalStatus::SUCCESS;
#else
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI_SENSOR
}
