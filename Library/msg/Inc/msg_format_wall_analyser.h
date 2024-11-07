//******************************************************************************
// @addtogroup  Message
// @file        msg_format_wall_analyser.h
// @brief       Message Format for WallAnalyser
//******************************************************************************
#pragma once

#include "mll_wall.h"
#include "msg_format.h"

namespace msg {

class MsgFormatWallAnalyser : public MsgFormat {
   public:
    MsgFormatWallAnalyser();

    void copy(void* target) override;

    void update(void* from) override;

    mll::Walldata front_wall;
    float distance_from_center;  // 左右壁センサから計算した中心からの横方向の距離 (左が正)
    float distance_from_front;   // 前壁センサから計算した前壁からの距離
    float angle_from_front;      // 前壁センサから計算した前壁からの角度 (時計方向にずれているときが正)
    bool kabekire_left;          // 左側の壁切れ発生可否
    bool kabekire_right;         // 右側の壁切れ発生可否
};

}  // namespace msg
