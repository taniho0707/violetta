//******************************************************************************
// @addtogroup MLL
// @file       mll_operation_coordinator.h
// @brief      Activity と マウス動作の取次を行うクラス
//******************************************************************************
#pragma once

#include "act_conf.h"
#include "mll_coordinate_director.h"
#include "mll_maze_solver.h"
#include "mll_position_updater.h"
#include "msg_format_localizer.h"
#include "msg_format_motor_controller.h"
#include "msg_format_motor_controller_internal.h"
#include "msg_server.h"
#include "stdint.h"

namespace mll {

// runSpecific で指定する動作のバッファ
constexpr uint16_t MAX_MOVE_LENGTH = 256;

enum class OperationCoordinatorResult : uint8_t {
    SUCCESS = 0,
    IDLE = 48,
    IDLE_WITH_MOTOR_CONTROL = 49,
    IDLE_WITHOUT_MOTOR_CONTROL = 50,
    RUNNING_SEARCH = 64,
    RUNNING_SHORT = 80,
    RUNNING_SPECIFIC = 96,
    ERROR_MOTOR_FAILSAFE = 200,
    ERROR_LENGTH_TOO_LONG = 253,
    ERROR_ALREADY_RUNNING = 254,
    FATAL_ERROR = 255,
};

// TODO: メンバを考える
struct SearchOptions {
    uint8_t maxSearchTime = 0;  // [ms]
    mll::AlgorithmType algorithm = mll::AlgorithmType::DIJKSTRA;
    bool oneway = false;  // 片道のみの探索

    MultiplePosition goals;  // 現在到達したいゴール座標一覧、全て向かうと完了する

    bool search_completed = false;   // 探索が完了したかどうか
    bool search_found_goal = false;  // ゴールまでの経路を見つけたかどうか

    float velocity_trans = 300.f;  // 速度 [mm/s]
    float velocity_turn = 300.f;   // ターン時の直進速度 [mm/s]
};

// TODO: メンバを考える
struct ShortOptions {
    act::ShortcutMethod shortcut_method;  // 最短経路走行の方法
    bool oneway = false;                  // ゴールで停止するかどうか

    float velocity_trans = 300.f;  // 速度 [mm/s]
    float velocity_turn = 300.f;   // ターン時の直進速度 [mm/s]
};

class OperationCoordinator {
   private:
    OperationCoordinator();

    OperationMoveCombination moves[MAX_MOVE_LENGTH];
    uint16_t moves_current_length;  // 現在の動作の長さ
    uint16_t moves_current_index;   // 現在の動作のインデックス

    // モーター制御の有効無効フラグ
    bool enabled_motor_control;

    SearchOptions search_options;

    // 現在の動作状態
    // Result を流用しているが、一部の状態しか取らないようにする
    OperationCoordinatorResult current_state;

    // 所有する各クラスへのポインタ
    PositionUpdater* position_updater;
    CoordinateDirector* coordinate_director;

    // message
    msg::MessageServer* msg_server;
    msg::MsgFormatMotorController msg_format_motor_controller;
    msg::MsgFormatLocalizer msg_format_localizer;
    msg::MsgFormatMotorControllerInternal msg_format_motor_controller_internal;

   public:
    OperationCoordinatorResult enableMotorControl();
    OperationCoordinatorResult disableMotorControl();

    OperationCoordinatorResult runSearch(SearchOptions opt);

    OperationCoordinatorResult runShort(ShortOptions opt);

    // 指定の走行パターンで走る
    OperationCoordinatorResult runSpecific(OperationMoveCombination* moves, uint16_t length);

    // 現在の状況を返す
    OperationCoordinatorResult state();

    // CoordinateDirector が持っている現在区画情報を返す
    const MouseSectionPosition getCurrentSection() const;

    // 位置をリセットする
    // enableMotorControl の直前に呼び出すことを推奨
    void resetPosition(const MousePhysicalPosition& position);

    void interruptPeriodic();

    static OperationCoordinator* getInstance();
};

}  // namespace mll
