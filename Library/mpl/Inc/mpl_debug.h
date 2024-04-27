//******************************************************************************
// @addtogroup MPL
// @file       mpl_debug.h
// @brief      デバッグ用機能(UARTログ等)
//******************************************************************************
#pragma once

// STL
#include <cstdio>

#include "hal_debug.h"
#include "mpl_conf.h"
#include "util.h"

namespace mpl {

constexpr uint16_t SNPRINTF_BUFFER_SIZE = hal::DEBUG_DMA_TX_BUFFER_SIZE;

class Debug {
   private:
    Debug();

   public:
    DmaState dmaTxState;
    DmaState dmaRxState;

    MplStatus init(hal::InitializeType type = hal::InitializeType::Sync);
    void deinit();

    template <typename... Args>
    uint16_t format(char *buffer,
                    const char *fmt, Args const &...args) {
        // FIXME: 入力が0文字の場合は0を返して何もしないようにする
        uint16_t len;
        len = snprintf(buffer, SNPRINTF_BUFFER_SIZE, fmt, args...);
        return misc::min(len, static_cast<uint16_t>(SNPRINTF_BUFFER_SIZE - 1));
    }

    // template <typename... Args>
    // MplStatus sendfSync(const char *fmt, Args const &...args) {
    //     printf(hal::sendUartDebugNByte, fmt, args...);
    //     return MplStatus::SUCCESS;
    // }

    MplStatus sendSync(const char *fmt, uint16_t len);
    MplStatus sendDma(const char *fmt, uint16_t len);

    // TODO: 入力に対応する

    void interruptPeriodic();
    void interruptTxComplete();
    void interruptTxError();
    void interruptRxComplete();
    void interruptRxError();

    static Debug *getInstance();
};

}  // namespace mpl
