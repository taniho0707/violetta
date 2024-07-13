//******************************************************************************
// @addtogroup MLL
// @file       mll_operation_controller.h
// @brief      マウスの動作を統括するクラス
//******************************************************************************
#pragma once

#include "cmd_format.h"
#include "mll_trajectory.h"
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
    LENGTH,
    UNDEFINED
};  // TODO: 考える

class OperationController {
   private:
    OperationController();

    cmd::OperationDirectionType current_operation_direction;

    Trajectory trajectory;

    OperationMoveType current_operation_move;
    OperationMoveType next_operation_move;
    bool is_operation_move_changed;  // OperationMoveType が変更されたフレームのみ true

    float continuous_distance_trapaccel;
    float continuous_distance_pivotturn;
    float latest_move_distance_trapaccel;  // 現在のMoveTypeの累計移動距離
    float latest_move_distance_pivotturn;  // 現在のMoveTypeの累計回転距離
    uint32_t latest_start_time;            // 現在のMoveTypeの開始時刻

    float velocity_slowest_translation;  // 超低速移動時の速度;
    float velocity_slowest_rotation;     // 超低速回転時の速度;

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
