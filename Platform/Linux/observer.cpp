//******************************************************************************
// @addtogroup PLT
// @file       observer.cpp
// @brief      各コンポーネントの内部状況をモニタリングするクラス
//******************************************************************************

#include "observer.h"

plt::Observer::Observer() {
    tcp = plt::TcpClient::getInstance();
    tcp->connectServer();
}

plt::Observer::~Observer() {}

bool plt::Observer::getImuData(hal::ImuData& data) {
    return tcp->getImuData(data);
}

bool plt::Observer::getBatteryVoltage(float& voltage) {
    return tcp->getBatteryVoltage(voltage);
}

plt::Observer* plt::Observer::getInstance() {
    static plt::Observer instance;
    return &instance;
}