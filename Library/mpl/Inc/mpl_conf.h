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

enum class I2cAsyncState : uint8_t {
    IDLE = 0,
    READ_ADDRESS = 1,
    READ_REGISTER = 2,
    READ_DATA = 3,
    WRITE_ADDRESS = 4,
    WRITE_REGISTER = 5,
    WRITE_DATA = 6,
    UNINITIALIZED = 254,
    ERROR = 255,
};

}  // namespace mpl
