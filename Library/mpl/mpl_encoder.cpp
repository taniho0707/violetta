//******************************************************************************
// @addtogroup MPL
// @file       mpl_encoder.cpp
// @brief      エンコーダ制御
//******************************************************************************
#include "mpl_encoder.h"

#include "msg_format_encoder.h"
#include "msg_server.h"

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

    msg_format.left = last.LEFT;
    msg_format.right = last.RIGHT;
    server->sendMessage(msg::ModuleId::ENCODER, &msg_format);
}

mpl::Encoder* mpl::Encoder::getInstance() {
    static mpl::Encoder instance;
    return &instance;
}
