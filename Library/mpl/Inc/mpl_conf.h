//******************************************************************************
// @addtogroup MPL
// @file       mpl_conf.h
// @brief      MPL関係の定義
//******************************************************************************
#pragma once

#include "stdint.h"

namespace mpl {

enum class MplStatus : uint8_t {
    SUCCESS = 0,
    DONE = 1,
    PROCESSING = 2,
    NO_IMPLEMENT = 253,
    INVALID_PARAMS = 254,
    ERROR = 255
};

enum class DmaState : uint8_t {
    IDLE = 0,
    RUNNING = 1,
    UNINITIALIZED = 254,
    ERROR = 255,
};

}  // namespace mpl
