//******************************************************************************
// @addtogroup MPL
// @file       mpl_debug.cpp
// @brief      デバッグ用機能(UARTログ等)
//******************************************************************************
#include "mpl_debug.h"

mpl::Debug::Debug() { init(); }

mpl::MplStatus mpl::Debug::init() {
    auto status = hal::initUartDebugPort();
    if (status == hal::HalStatus::SUCCESS) {
        return MplStatus::SUCCESS;
    } else {
        return MplStatus::ERROR;
    }
}

void mpl::Debug::deinit() { hal::deinitUartDebugPort(); }

uint16_t mpl::Debug::printf(const char *fmt, ...) {
    char buffer[300];
    int len;

    va_list ap;
    va_start(ap, fmt);

    len = vsprintf(buffer, fmt, ap);
    hal::sendUartDebugNByte(buffer, len);
    va_end(ap);
    return static_cast<uint16_t>(len);
}

uint16_t mpl::Debug::println(const char *fmt, ...) {
    char buffer[300];
    int len;

    va_list ap;
    va_start(ap, fmt);

    len = vsprintf(buffer, fmt, ap);
    hal::sendUartDebugNByte(buffer, len);
    va_end(ap);

    hal::sendUartDebug1Byte((uint8_t)('\n'));
    return static_cast<uint16_t>(len + 1);
}

mpl::Debug *mpl::Debug::getInstance() {
    static mpl::Debug instance;
    return &instance;
}
