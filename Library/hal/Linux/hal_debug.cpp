//******************************************************************************
// @addtogroup HAL
// @file       hal_debug.cpp
// @brief      デバッグ用機能(UARTログ等)
//******************************************************************************

#include "hal_debug.h"

hal::HalStatus hal::initUartDebugPort(InitializeType type) {
    return HalStatus::SUCCESS;
}

hal::HalStatus hal::deinitUartDebugPort() {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::sendUartDebug1Byte(uint8_t data) {
    if (plt::Observer::getInstance()->sendUartDebug(&data, 1)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::sendUartDebugNByte(const char *data, const int len) {
    if (plt::Observer::getInstance()->sendUartDebug(reinterpret_cast<uint8_t *>(const_cast<char *>(data)), len)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
    return hal::HalStatus::NOIMPLEMENT;
}

hal::HalStatus hal::sendUartDebugDmaNByte(const char *data, const int len) {
    return hal::HalStatus::NOIMPLEMENT;
}
