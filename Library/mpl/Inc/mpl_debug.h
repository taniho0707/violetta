//******************************************************************************
// @addtogroup MPL
// @file       mpl_debug.h
// @brief      デバッグ用機能(UARTログ等)
//******************************************************************************
#pragma once

// STL
#include <cstdarg>
#include <cstdio>
#include <string>

#include "hal_debug.h"
#include "mpl_conf.h"

namespace mpl {

class Debug {
   private:
    Debug();

   public:
    MplStatus init();
    void deinit();

    uint16_t printf(const char *fmt, ...);
    uint16_t println(const char *fmt, ...);
    // MplStatus scanAllAsync();
    // MplStatus doneScanAsync();
    // MplStatus scanAllDma();
    // MplStatus doneScanDma();

    // TODO: 入力に対応する

    // void interruptPeriodic();
    // void interruptAsync();
    // void interruptDma();

    static Debug *getInstance();
};

}  // namespace mpl
