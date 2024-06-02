//******************************************************************************
// @addtogroup MPL
// @file       mpl_encoder.cpp
// @brief      エンコーダ制御
//******************************************************************************
#include "mpl_encoder.h"

#include "msg_format_encoder.h"
#include "msg_server.h"
#include "params.h"
#include "util.h"

mpl::Encoder::Encoder() {}

void mpl::Encoder::initPort() {
    hal::initEncoderPort();
}

void mpl::Encoder::deinitPort() {
    hal::deinitEncoderPort();
}

mpl::MplStatus mpl::Encoder::scanEncoderSync(hal::EncoderData& data) {
    if (hal::getEncoderSync(data) == hal::HalStatus::SUCCESS) {
        last = data;
        return mpl::MplStatus::SUCCESS;
    } else {
        return mpl::MplStatus::ERROR;
    }
}

void mpl::Encoder::interruptPeriodic() {
    static auto server = msg::MessageServer::getInstance();
    scanEncoderSync(last);

    static auto params_cache = misc::Params::getInstance()->getCachePointer();

    msg_format.left = static_cast<float>(last.LEFT) * params_cache->tire_diameter * misc::PI / params_cache->encoder_resolution;
    msg_format.right = static_cast<float>(last.RIGHT) * params_cache->tire_diameter * misc::PI / params_cache->encoder_resolution;
    server->sendMessage(msg::ModuleId::ENCODER, &msg_format);
}

mpl::Encoder* mpl::Encoder::getInstance() {
    static mpl::Encoder instance;
    return &instance;
}
