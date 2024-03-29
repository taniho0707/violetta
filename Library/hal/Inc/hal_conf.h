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

const float BATTERY_RATIO = 1.5f;

#define WALLSENSOR_NUMS 5
enum class WallSensorNumbers : uint8_t {
    FRONTLEFT = 0,
    LEFT,
    FRONT,
    RIGHT,
    FRONTRIGHT,
    ALL = 255,
};

struct WallSensorData {
    uint16_t FRONTLEFT;
    uint16_t LEFT;
    uint16_t FRONT;
    uint16_t RIGHT;
    uint16_t FRONTRIGHT;
};
#endif  // ifdef MOUSE_VIOLETTA

struct ImuData {
    int16_t OUT_TEMP;
    int16_t OUT_X_G;
    int16_t OUT_Y_G;
    int16_t OUT_Z_G;
    int16_t OUT_X_A;
    int16_t OUT_Y_A;
    int16_t OUT_Z_A;
};

struct EncoderData {
    int16_t LEFT;
    int16_t RIGHT;
};

}  // namespace hal
