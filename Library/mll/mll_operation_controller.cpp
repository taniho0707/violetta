//******************************************************************************
// @addtogroup MLL
// @file       mll_operation_controller.cpp
// @brief      マウスの動作を統括するクラス
//******************************************************************************
#include "mll_operation_controller.h"

#include "cmd_server.h"
#include "mpl_timer.h"
#include "msg_format_localizer.h"
#include "msg_format_motor_controller.h"
#include "msg_server.h"

mll::OperationController::OperationController() {
    current_operation_direction = cmd::OperationDirectionType::STOP;
    current_operation_move = OperationMoveType::STOP;
    next_operation_move = OperationMoveType::UNDEFINED;
    is_operation_move_changed = false;

    continuous_distance_trapaccel = 0.0f;
    continuous_distance_pivotturn = 0.0f;
    latest_move_distance_trapaccel = 0.0f;
    latest_move_distance_pivotturn = 0.0f;
}

void mll::OperationController::init() {
    params = misc::Params::getInstance()->getCachePointer();
}

void mll::OperationController::interruptPeriodic() {
    static auto msg_server = msg::MessageServer::getInstance();
    static msg::MsgFormatLocalizer msg_format_localizer;
    static msg::MsgFormatMotorController msg_format_motor_controller;

    static auto cmd_server = cmd::CommandServer::getInstance();
    static cmd::CommandFormatOperationDirection cmd_format_operation_direction;

    // 新しい動作コマンドが来ていないか確認
    if (cmd_server->length(cmd::CommandId::OPERATION_DIRECTION) > 0) {
        cmd_server->pop(cmd::CommandId::OPERATION_DIRECTION, &cmd_format_operation_direction);
        switch (current_operation_direction) {
            case cmd::OperationDirectionType::STOP:
                if (cmd_format_operation_direction.type == cmd::OperationDirectionType::SEARCH) {
                    next_operation_move = OperationMoveType::TRAPACCEL;
                } else {
                    // TODO: 実装
                }
                break;
            case cmd::OperationDirectionType::SEARCH:
                break;
            case cmd::OperationDirectionType::RUN:
                break;
            default:
                break;
        }
        current_operation_direction = cmd_format_operation_direction.type;
        is_operation_move_changed = true;
    }

    // MoveTypeの更新が必要か確認
    if (trajectory.isEnd(mpl::Timer::getMilliTime() - latest_start_time)) {
        if (next_operation_move == OperationMoveType::UNDEFINED) {
            current_operation_move = OperationMoveType::STOP;
        } else {
            current_operation_move = next_operation_move;
        }
    }

    // 新しいMoveTypeに変更されたら軌跡の生成
    if (is_operation_move_changed) {
        latest_start_time = mpl::Timer::getMilliTime();
        switch (current_operation_move) {
            case OperationMoveType::STOP:
                break;
            case OperationMoveType::SLALOM90SML_RIGHT:
                break;
            case OperationMoveType::SLALOM90SML_LEFT:
                break;
            case OperationMoveType::SLALOM90_RIGHT:
                break;
            case OperationMoveType::SLALOM90_LEFT:
                break;
            case OperationMoveType::SLALOM180_RIGHT:
                break;
            case OperationMoveType::SLALOM180_LEFT:
                break;
            case OperationMoveType::SLALOM45IN_RIGHT:
                break;
            case OperationMoveType::SLALOM45IN_LEFT:
                break;
            case OperationMoveType::SLALOM45OUT_RIGHT:
                break;
            case OperationMoveType::SLALOM45OUT_LEFT:
                break;
            case OperationMoveType::SLALOM135IN_RIGHT:
                break;
            case OperationMoveType::SLALOM135IN_LEFT:
                break;
            case OperationMoveType::SLALOM135OUT_RIGHT:
                break;
            case OperationMoveType::SLALOM135OUT_LEFT:
                break;
            case OperationMoveType::SLALOM90OBL_RIGHT:
                break;
            case OperationMoveType::SLALOM90OBL_LEFT:
                break;
            case OperationMoveType::TRAPACCEL:
                trajectory.init(TrajectoryCalcType::TIME, TrajectoryFormType::TRAPEZOID, 2000.f, 450.f, 0.0f, 300.f, 0.f);
                break;
            case OperationMoveType::PIVOTTURN:
                break;
            case OperationMoveType::TRAPDIAGO:
                break;
            case OperationMoveType::LENGTH:
            case OperationMoveType::UNDEFINED:
                break;
            default:
                break;
        }
        is_operation_move_changed = false;
    }

    // 動作指令を更新
    switch (current_operation_move) {
        case OperationMoveType::STOP:
            msg_format_motor_controller.velocity_translation = 0.0f;
            msg_format_motor_controller.velocity_rotation = 0.0f;
            msg_format_motor_controller.is_controlled = true;
            break;
        case OperationMoveType::SLALOM90SML_RIGHT:
            break;
        case OperationMoveType::SLALOM90SML_LEFT:
            break;
        case OperationMoveType::SLALOM90_RIGHT:
            break;
        case OperationMoveType::SLALOM90_LEFT:
            break;
        case OperationMoveType::SLALOM180_RIGHT:
            break;
        case OperationMoveType::SLALOM180_LEFT:
            break;
        case OperationMoveType::SLALOM45IN_RIGHT:
            break;
        case OperationMoveType::SLALOM45IN_LEFT:
            break;
        case OperationMoveType::SLALOM45OUT_RIGHT:
            break;
        case OperationMoveType::SLALOM45OUT_LEFT:
            break;
        case OperationMoveType::SLALOM135IN_RIGHT:
            break;
        case OperationMoveType::SLALOM135IN_LEFT:
            break;
        case OperationMoveType::SLALOM135OUT_RIGHT:
            break;
        case OperationMoveType::SLALOM135OUT_LEFT:
            break;
        case OperationMoveType::SLALOM90OBL_RIGHT:
            break;
        case OperationMoveType::SLALOM90OBL_LEFT:
            break;
        case OperationMoveType::TRAPACCEL:
            msg_format_motor_controller.velocity_translation = trajectory.getVelocity(mpl::Timer::getMilliTime() - latest_start_time);
            msg_format_motor_controller.velocity_rotation = 0.0f;
            msg_format_motor_controller.is_controlled = true;
            break;
        case OperationMoveType::PIVOTTURN:
            break;
        case OperationMoveType::TRAPDIAGO:
            break;
        case OperationMoveType::LENGTH:
        case OperationMoveType::UNDEFINED:
            msg_format_motor_controller.is_controlled = false;
            break;
        default:
            break;
    }
    msg_server->sendMessage(msg::ModuleId::MOTORCONTROLLER, &msg_format_motor_controller);
}

mll::OperationController* mll::OperationController::getInstance() {
    static OperationController instance;
    return &instance;
}
