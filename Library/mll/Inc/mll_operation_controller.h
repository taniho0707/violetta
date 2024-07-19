//******************************************************************************
// @addtogroup MLL
// @file       mll_operation_controller.h
// @brief      マウスの動作を統括するクラス
//******************************************************************************
#pragma once

#include "cmd_format.h"
#include "mll_maze_solver.h"
#include "mll_operation_move_type.h"
#include "mll_position.h"
#include "mll_trajectory.h"
#include "msg_format_motor_controller.h"
#include "params.h"

namespace mll {

enum class SlalomState : uint8_t {
    BEFORE = 0,
    TURN,
    AFTER,
    END,
};

const float SLALOM_LEFT = 1.f;
const float SLALOM_RIGHT = -1.f;

class OperationController {
   private:
    OperationController();

    cmd::OperationDirectionType current_operation_direction;

    Trajectory trajectory;

    OperationMoveType last_operation_move;  // 直前のMoveType (for EXTRALENGTH)
    OperationMoveType current_operation_move;
    OperationMoveType next_operation_move;
    bool is_operation_move_changed;  // OperationMoveType が変更されたフレームのみ true

    float continuous_distance;          // 累計移動/回転距離
    float latest_move_distance;         // 現在のMoveTypeの累計移動/回転距離
    uint32_t latest_start_time;         // 現在のMoveTypeの開始時刻
    float target_move_distance;         // 現在のMoveTypeの目標移動/回転距離
    float target_velocity_translation;  // 現在のMoveTypeの目標移動速度

    OperationMoveCombination* operation_move_combinations;  // 規定動作の配列、メモリ管理はしないため注意
    uint16_t operation_move_combinations_length;            // 規定動作の配列のサイズ、0なら規定動作無し
    int16_t operation_move_combinations_i;                  // 規定動作の配列の現在インデックス

    // TODO: 値をパラメータから読み出す
    float velocity_slowest_translation;  // 超低速移動時の速度
    float velocity_slowest_rotation;     // 超低速回転時の速度

    SlalomState slalom_state;

    // マウスの論理座標を保持
    // FIXME: 適切なクラスに移管する
    MouseSectionPosition section_position;

    MazeSolver* solver;

    misc::MouseParams* params;
    misc::SlalomParams* slalom_params;

    // スラロームの初期設定
    // current_operation_move に正しい値が入っている前提で動く
    void initSlalom(float rightleft);
    // スラロームの動作
    void runSlalom(msg::MsgFormatMotorController& msg);

   public:
    void init();

    bool isNextOperationBlank();  // 次の動作が未定義なら true を返す

    // TRAPACCEL の場合初速は現在速
    // STOP の場合 distance 後に停止することにする
    // FIXME: 直す…
    void setNextOperation(OperationMoveType next, float distance = 0);  // 次の動作を設定する

    // 分担
    // 動作状態 (OperationMoveType) で区切って動作させる
    // 動作状態を指定するのは OperationController の役割
    // 動作状態から目標速度等を計算するのも OperationController の役割
    // MotorController は内部で動作状態を持たない
    void interruptPeriodic();

    static OperationController* getInstance();
};

}  // namespace mll
