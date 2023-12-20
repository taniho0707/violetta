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

plt::Observer* plt::Observer::getInstance() {
    static plt::Observer instance;
    return &instance;
}