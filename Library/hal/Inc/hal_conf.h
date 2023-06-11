//******************************************************************************
// @addtogroup HAL
// @file       hal_conf.h
// @brief      ハードウェアに紐付く定義
//******************************************************************************
#pragma once

#ifdef LINUX
#include <cstdint>
#endif

namespace hal {

/// @brief HalStatus
enum class HalStatus : uint8_t { SUCCESS = 0, ERROR = 255 };

#ifdef MOUSE_VIOLETTA
#define LED_NUMS 4
enum class LedNumbers : uint8_t {
    FRONT1 = 0,
    FRONT2,
    TOP1,
    TOP2,
    ALL = 255,
};

enum class GyroAxises : uint8_t {
    YAW = 0,
    ROLL,
    PITCH,
};
#endif  // ifdef MOUSE_VIOLETTA

}  // namespace hal
