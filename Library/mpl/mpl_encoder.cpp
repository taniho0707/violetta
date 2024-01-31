//******************************************************************************
// @addtogroup MPL
// @file       mpl_encoder.cpp
// @brief      エンコーダ制御
//******************************************************************************
#include "mpl_encoder.h"

mpl::Encoder::Encoder() { hal::initEncoderPort(); }

void mpl::Encoder::initPort() { hal::initEncoderPort(); }

void mpl::Encoder::deinitPort() { hal::deinitEncoderPort(); }

mpl::MplStatus mpl::Encoder::scanEncoderSync(hal::EncoderData& data) {
    if (hal::getEncoderSync(data) == hal::HalStatus::SUCCESS) {
        return mpl::MplStatus::SUCCESS;
    } else {
        return mpl::MplStatus::ERROR;
    }
}

void mpl::Encoder::interrupt() {}

mpl::Encoder* mpl::Encoder::getInstance() {
    static mpl::Encoder instance;
    return &instance;
}
