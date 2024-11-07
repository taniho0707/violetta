//******************************************************************************
// @addtogroup HAL
// @file       hal_encoder.cpp
// @brief      エンコーダ制御
//******************************************************************************

#include "hal_encoder.h"

hal::HalStatus hal::initEncoderPort() {
    return HalStatus::SUCCESS;
}

hal::HalStatus hal::deinitEncoderPort() {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus hal::getEncoderSync(EncoderData& data) {
    if (plt::Observer::getInstance()->getEncoder(data)) {
        return hal::HalStatus::SUCCESS;
    } else {
        return hal::HalStatus::ERROR;
    }
}
