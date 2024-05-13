//******************************************************************************
// @addtogroup MLL
// @file       mll_conf_motor.h
// @brief      モータ制御に関する定数定義
//******************************************************************************
#pragma once

#include "stdint.h"

namespace mll {

enum class MotorControlResult : uint8_t {
    SUCCESS = 0,
    NO_IMPLEMENT = 254,
    ERROR = 255,
};

// モーターに入力する電流、電圧、Dutyを含める構造体
struct MotorControlDataSingle {
    float current;  // 電流
    float voltage;  // 電圧
    float duty;     // Duty
};

struct MotorControlData {
    MotorControlDataSingle L;
    MotorControlDataSingle R;
};

// PID制御のパラメータを含める構造体
struct MotorControlPID {
    float p;  // 比例ゲイン
    float i;  // 積分ゲイン
    float d;  // 微分ゲイン
};

}  // namespace mll
