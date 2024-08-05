//******************************************************************************
// @addtogroup MLL
// @file       mll_operation_coordinator.h
// @brief      Activity と マウス動作の取次を行うクラス
//******************************************************************************
#pragma once

#include "mll_maze_solver.h"
#include "mll_position_updater.h"
#include "msg_format_localizer.h"
#include "msg_format_motor_controller.h"
#include "msg_server.h"
#include "stdint.h"

namespace mll {

enum class OperationCoordinatorResult : uint8_t {
    SUCCESS = 0,
    IDLE = 48,
    IDLE_WITH_MOTOR_CONTROL = 49,
    IDLE_WITHOUT_MOTOR_CONTROL = 50,
    RUNNING_SEARCH = 64,
    RUNNING_SHORT = 80,
    RUNNING_SPECIFIC = 96,
    ERROR_LENGTH_TOO_LONG = 253,
    ERROR_ALREADY_RUNNING = 254,
    FATAL_ERROR = 255,
};

struct SearchOptions {
    uint8_t maxSearchTime = 0;  // TODO: メンバを考える
};

struct ShortOptions {
    uint8_t hoge = 0;  // TODO: メンバを考える
};

class OperationCoordinator {
   private:
    OperationCoordinator();

    // runSpecific で指定する動作のバッファ
    static constexpr uint16_t MAX_MOVE_LENGTH = 256;
    OperationMoveCombination moves[MAX_MOVE_LENGTH];
    uint16_t moves_current_length;  // 現在の動作の長さ
    uint16_t moves_current_index;   // 現在の動作のインデックス

    // モーター制御の有効無効フラグ
    bool enabled_motor_control;

    // 現在の動作状態
    // Result を流用しているが、一部の状態しか取らないようにする
    OperationCoordinatorResult current_state;

    // 所有する各クラスへのポインタ
    PositionUpdater* position_updater;

    // message
    msg::MessageServer* msg_server;
    msg::MsgFormatMotorController msg_format_motor_controller;
    msg::MsgFormatLocalizer msg_format_localizer;

   public:
    OperationCoordinatorResult enableMotorControl();
    OperationCoordinatorResult disableMotorControl();

    OperationCoordinatorResult runSearch(AlgorithmType type, SearchOptions opt = SearchOptions{});

    OperationCoordinatorResult runShort(ShortOptions opt = ShortOptions{});

    // 指定の走行パターンで走る
    OperationCoordinatorResult runSpecific(OperationMoveCombination* moves, uint16_t length);

    // 現在の状況を返す
    OperationCoordinatorResult state();

    // 位置をリセットする
    // enableMotorControl の直前に呼び出すことを推奨
    void resetPosition(const MousePhysicalPosition& position);

    void interruptPeriodic();

    static OperationCoordinator* getInstance();
};

}  // namespace mll
