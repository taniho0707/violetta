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
    INVALID_PARAMS = 254,
    ERROR = 255
};

}  // namespace mpl
