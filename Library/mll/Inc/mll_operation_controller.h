//******************************************************************************
// @addtogroup MLL
// @file       mll_operation_controller.h
// @brief      マウスの動作を統括するクラス
//******************************************************************************
#pragma once

#include "cmd_format.h"
#include "params.h"

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
    PIVOTTURN,
    TRAPDIAGO,
    LENGTH
};  // TODO: 考える

class OperationController {
   private:
    OperationController();

    cmd::OperationDirectionType current_operation_direction;

    misc::MouseParams* params;

   public:
    void init();

    // 分担
    // 動作状態 (OperationMoveType) で区切って動作させる
    // 動作状態を指定するのは OperationController の役割
    // 動作状態から目標速度等を計算するのも OperationController の役割
    // MotorController は内部で動作状態を持たない
    void interruptPeriodic();

    static OperationController* getInstance();
};

}  // namespace mll
