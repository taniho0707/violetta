//******************************************************************************
// @addtogroup MPL
// @file       mpl_debug.cpp
// @brief      デバッグ用機能(UARTログ等)
//******************************************************************************
#include "mpl_debug.h"

#include "cmd_server.h"

using namespace mpl;

mpl::Debug::Debug() {
    dmaTxState = DmaState::UNINITIALIZED;
    dmaRxState = DmaState::UNINITIALIZED;
}

mpl::MplStatus mpl::Debug::init(hal::InitializeType type) {
    auto status = hal::initUartDebugPort(type);
    if (status == hal::HalStatus::SUCCESS) {
        dmaTxState = DmaState::IDLE;
        dmaRxState = DmaState::IDLE;
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

void mpl::Debug::deinit() {
    dmaTxState = DmaState::UNINITIALIZED;
    dmaRxState = DmaState::UNINITIALIZED;
    hal::deinitUartDebugPort();
}

mpl::MplStatus mpl::Debug::sendSync(const char *fmt, uint16_t len) {
    if (hal::sendUartDebugNByte(fmt, len) == hal::HalStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

mpl::MplStatus mpl::Debug::sendDma(const char *fmt, uint16_t len) {
    if (hal::sendUartDebugDmaNByte(fmt, len) == hal::HalStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

void mpl::Debug::interruptPeriodic() {
    static auto cmdsrv = cmd::CommandServer::getInstance();

    // DEBUG_TX: Mouse -> PC
    if (cmdsrv->length(cmd::CommandId::DEBUG_TX) > 0) {
        if (dmaTxState != DmaState::IDLE) {
            return;
        }
        dmaTxState = DmaState::RUNNING;
        auto format = cmd::CommandFormatDebugTx{0};
        cmdsrv->pop(cmd::CommandId::DEBUG_TX, &format);
        // sendfSync(format.message);
        sendDma(format.message, format.len);
    }
}

void mpl::Debug::interruptTxComplete() {
    dmaTxState = DmaState::IDLE;
}

void mpl::Debug::interruptTxError() {
    dmaTxState = DmaState::ERROR;
}

void mpl::Debug::interruptRxComplete() {
    dmaRxState = DmaState::IDLE;
}

void mpl::Debug::interruptRxError() {
    dmaRxState = DmaState::ERROR;
}

mpl::Debug *mpl::Debug::getInstance() {
    static mpl::Debug instance;
    return &instance;
}
