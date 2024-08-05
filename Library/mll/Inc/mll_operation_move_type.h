//******************************************************************************
// @addtogroup MLL
// @file       mll_operation_move_type.h
// @brief      マウスの動作を定義
//******************************************************************************
#pragma once

#include "stdint.h"

namespace mll {

enum class OperationMoveType : uint8_t {
    STOP = 0,
    SLALOM90SML_RIGHT,
    SLALOM90SML_LEFT,
    SLALOM90_RIGHT,
    SLALOM90_LEFT,
    SLALOM180_RIGHT,
    SLALOM180_LEFT,
    SLALOM45IN_RIGHT,
    SLALOM45IN_LEFT,
    SLALOM45OUT_RIGHT,
    SLALOM45OUT_LEFT,
    SLALOM135IN_RIGHT,
    SLALOM135IN_LEFT,
    SLALOM135OUT_RIGHT,
    SLALOM135OUT_LEFT,
    SLALOM90OBL_RIGHT,
    SLALOM90OBL_LEFT,
    TRAPACCEL,
    TRAPACCEL_STOP,
    PIVOTTURN,
    // TRAPDIAGO,
    // EXTRALENGTH,
    WAIT,
    LENGTH,
    UNDEFINED
};  // TODO: 考える

struct OperationMoveCombination {
    OperationMoveType type;
    float distance;  // 距離、角度、秒数など
};

enum class SlalomState : uint8_t {
    BEFORE = 0,
    TURN,
    AFTER,
    END,
};

}  // namespace mll
