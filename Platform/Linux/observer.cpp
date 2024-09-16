//******************************************************************************
// @addtogroup PLT
// @file       observer.cpp
// @brief      各コンポーネントの内部状況をモニタリングするクラス
//******************************************************************************

#include "observer.h"

plt::Observer::Observer() {
    udp = plt::UdpClient::getInstance();
    udp->connectServer();
}

plt::Observer::~Observer() {}

bool plt::Observer::getImuData(hal::ImuData& data) {
    return udp->getImuData(data);
}

bool plt::Observer::getBatteryVoltage(float& voltage) {
    return udp->getBatteryVoltage(voltage);
}

bool plt::Observer::getWallSensorData(uint16_t* data) {
    return udp->getWallSensorData(data);
}

bool plt::Observer::getEncoder(hal::EncoderData& data) {
    return udp->getEncoder(data);
}

plt::Observer* plt::Observer::getInstance() {
    static plt::Observer instance;
    return &instance;
}