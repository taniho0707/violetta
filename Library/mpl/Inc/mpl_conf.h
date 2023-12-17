//******************************************************************************
// @addtogroup MPL
// @file       mpl_conf.h
// @brief      MPL関係の定義
//******************************************************************************
#pragma once

namespace mpl {

enum class MplStatus : uint8_t {
    SUCCESS = 0,
    DONE = 1,
    PROCESSING = 2,
    ERROR = 255
};

}  // namespace mpl
