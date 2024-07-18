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

    continuous_distance = 0.0f;
    latest_move_distance = 0.0f;
    latest_start_time = 0;
    target_move_distance = 0.0f;
    target_velocity_translation = 0.0f;

    operation_move_combinations_i = 0;

    velocity_slowest_translation = 50.f;
    velocity_slowest_rotation = 1.f;

    init();
}

void mll::OperationController::init() {
    params = misc::Params::getInstance()->getCachePointer();
    misc::Params::getInstance()->loadSlalom(misc::ParameterDestinationType::HARDCODED);
    slalom_params = misc::Params::getInstance()->getCacheSlalomPointer(300.f);
}

bool mll::OperationController::isNextOperationBlank() {
    return next_operation_move == OperationMoveType::UNDEFINED;
}

void mll::OperationController::setNextOperation(OperationMoveType next, float distance) {
    // TODO: next_operation_move が設定済みの場合どうするか検討する
    next_operation_move = next;
    if (distance != 0) {
        latest_move_distance = 0.0f;
    }
    is_operation_move_changed = true;
}

void mll::OperationController::initSlalom(float rightleft) {
    trajectory.init(TrajectoryCalcType::TIME, TrajectoryFormType::TRAPEZOID,
                    rightleft * slalom_params[static_cast<uint8_t>(current_operation_move)].acc_rad,
                    rightleft * slalom_params[static_cast<uint8_t>(current_operation_move)].deg, 0.f,
                    rightleft * slalom_params[static_cast<uint8_t>(current_operation_move)].max_v_rad, 0.f);
    slalom_state = SlalomState::BEFORE;
    latest_move_distance = 0.f;
}

void mll::OperationController::runSlalom(msg::MsgFormatMotorController& msg) {
    if (slalom_state == SlalomState::BEFORE) {
        msg.velocity_translation = target_velocity_translation;
        msg.velocity_rotation = 0.0f;
        msg.is_controlled = true;
        if (latest_move_distance >= slalom_params[static_cast<uint8_t>(current_operation_move)].d_before) {
            slalom_state = SlalomState::TURN;
        }
        latest_start_time = mpl::Timer::getMilliTime();
    }
    if (slalom_state == SlalomState::TURN) {
        msg.velocity_translation = target_velocity_translation;
        msg.velocity_rotation = trajectory.getVelocity(mpl::Timer::getMilliTime() - latest_start_time);
        msg.is_controlled = true;
        latest_move_distance = 0.f;
        if (trajectory.isEnd(mpl::Timer::getMilliTime() - latest_start_time)) {
            slalom_state = SlalomState::AFTER;
        }
    }
    if (slalom_state == SlalomState::AFTER) {
        msg.velocity_translation = target_velocity_translation;
        msg.velocity_rotation = 0.0f;
        msg.is_controlled = true;
        if (latest_move_distance >= slalom_params[static_cast<uint8_t>(current_operation_move)].d_after) {
            slalom_state = SlalomState::END;
        }
    }
}

void mll::OperationController::interruptPeriodic() {
    static auto msg_server = msg::MessageServer::getInstance();
    static msg::MsgFormatLocalizer msg_format_localizer;
    static msg::MsgFormatMotorController msg_format_motor_controller;
    msg_server->receiveMessage(msg::ModuleId::LOCALIZER, &msg_format_localizer);

    // 現在の MoveType の移動/回転距離更新
    if (current_operation_move == OperationMoveType::PIVOTTURN) {
        latest_move_distance += msg_format_localizer.velocity_rotation * 0.001f;
    } else {
        latest_move_distance += msg_format_localizer.position_translation;
    }

    static auto cmd_server = cmd::CommandServer::getInstance();
    static cmd::CommandFormatOperationDirection cmd_format_operation_direction;
    static cmd::CommandFormatOperationMoveArray cmd_format_operation_move_array;

    // 新しい動作コマンドが来ていないか確認
    if (cmd_server->length(cmd::CommandId::OPERATION_DIRECTION) > 0) {
        cmd_server->pop(cmd::CommandId::OPERATION_DIRECTION, &cmd_format_operation_direction);
        switch (current_operation_direction) {
            case cmd::OperationDirectionType::STOP:
                if (cmd_format_operation_direction.type == cmd::OperationDirectionType::SEARCH) {
                    // next_operation_move = OperationMoveType::TRAPACCEL;
                    next_operation_move = OperationMoveType::PIVOTTURN;
                } else if (cmd_format_operation_direction.type == cmd::OperationDirectionType::SPECIFIC) {
                    cmd_server->pop(cmd::CommandId::OPERATION_MOVE_ARRAY, &cmd_format_operation_move_array);
                    operation_move_combinations_i = -1;
                    operation_move_combinations = (OperationMoveCombination*)cmd_format_operation_move_array.move_array;
                    operation_move_combinations_length = cmd_format_operation_move_array.length;
                } else {
                    // TODO: 実装
                }
                break;
            case cmd::OperationDirectionType::SEARCH:
                // target_move_distance を適切に設定すること
                break;
            case cmd::OperationDirectionType::RUN:
                break;
            case cmd::OperationDirectionType::SPECIFIC:
                break;
            default:
                break;
        }
        current_operation_direction = cmd_format_operation_direction.type;
        is_operation_move_changed = true;
    }

    // MoveTypeの更新が必要か確認
    if (current_operation_move != OperationMoveType::STOP &&
        trajectory.getTimeStartDeceleration() <= mpl::Timer::getMilliTime() - latest_start_time) {
        // 減速開始時刻を過ぎ、終端速度以下になろうとしていれば終端距離まで維持する
        float target_velocity = target_velocity_translation;
        if (current_operation_move == OperationMoveType::PIVOTTURN) {
            target_velocity = misc::max(target_velocity, velocity_slowest_rotation);
        } else {
            target_velocity = misc::max(target_velocity, velocity_slowest_translation);
        }
        if (trajectory.getVelocity(mpl::Timer::getMilliTime() - latest_start_time) <= target_velocity) {
            target_velocity_translation = target_velocity;
            if (current_operation_move != OperationMoveType::EXTRALENGTH) {
                last_operation_move = current_operation_move;
            }
            current_operation_move = OperationMoveType::EXTRALENGTH;
        }
    }

    // 終了条件: 軌跡が終了した or EXTRALENGTH で目標距離に達した or SlalomState::END
    if (((current_operation_move == OperationMoveType::TRAPACCEL || current_operation_move == OperationMoveType::TRAPACCEL_STOP ||
          current_operation_move == OperationMoveType::PIVOTTURN) &&
         trajectory.isEnd(mpl::Timer::getMilliTime() - latest_start_time)) ||
        (current_operation_move == OperationMoveType::EXTRALENGTH && target_move_distance <= latest_move_distance) ||
        slalom_state == SlalomState::END || current_operation_move == OperationMoveType::STOP) {
        // 規定動作の配列がある場合、次の動作を読み込む
        if (current_operation_direction == cmd::OperationDirectionType::SPECIFIC) {
            if (operation_move_combinations_length > 0) {
                ++operation_move_combinations_i;
                if (operation_move_combinations_i < operation_move_combinations_length) {
                    next_operation_move = operation_move_combinations[operation_move_combinations_i].type;
                    target_move_distance = operation_move_combinations[operation_move_combinations_i].distance;
                } else {
                    if (target_velocity_translation == 0.f || target_velocity_translation == velocity_slowest_translation ||
                        target_velocity_translation == velocity_slowest_rotation) {
                        target_move_distance = 0.f;
                        next_operation_move = OperationMoveType::UNDEFINED;
                    } else {
                        target_move_distance = 45.f;
                        next_operation_move = OperationMoveType::TRAPACCEL_STOP;
                    }
                }
            } else {
                next_operation_move = OperationMoveType::UNDEFINED;
            }
            is_operation_move_changed = true;
        }

        if (next_operation_move == OperationMoveType::UNDEFINED) {
            last_operation_move = current_operation_move;
            current_operation_move = OperationMoveType::STOP;
        } else {
            if (current_operation_move != OperationMoveType::EXTRALENGTH) {
                last_operation_move = current_operation_move;
            }
            current_operation_move = next_operation_move;
            next_operation_move = OperationMoveType::UNDEFINED;
        }
        latest_move_distance = 0.f;
    }

    // 新しいMoveTypeに変更されたら軌跡の生成
    if (is_operation_move_changed) {
        latest_start_time = mpl::Timer::getMilliTime();
        switch (current_operation_move) {
            case OperationMoveType::STOP:
                target_velocity_translation = 0.f;
                break;
            case OperationMoveType::SLALOM90SML_RIGHT:
                initSlalom(SLALOM_RIGHT);
                break;
            case OperationMoveType::SLALOM90SML_LEFT:
                initSlalom(SLALOM_LEFT);
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
                trajectory.init(TrajectoryCalcType::TIME, TrajectoryFormType::TRAPEZOID, 2000.f, target_move_distance, target_velocity_translation,
                                300.f, 300.f);
                target_velocity_translation = 300.f;
                break;
            case OperationMoveType::TRAPACCEL_STOP:
                trajectory.init(TrajectoryCalcType::TIME, TrajectoryFormType::TRAPEZOID, 2000.f, target_move_distance, target_velocity_translation,
                                300.f, 0.f);
                target_velocity_translation = 0.f;
                break;
            case OperationMoveType::PIVOTTURN:
                trajectory.init(TrajectoryCalcType::TIME, TrajectoryFormType::TRAPEZOID, 100.f, target_move_distance, 0.0f, 15.f, 0);
                target_velocity_translation = 0.f;
                break;
            case OperationMoveType::TRAPDIAGO:
                break;
            case OperationMoveType::EXTRALENGTH:
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
            runSlalom(msg_format_motor_controller);
            break;
        case OperationMoveType::SLALOM90SML_LEFT:
            runSlalom(msg_format_motor_controller);
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
        case OperationMoveType::TRAPACCEL_STOP:
            msg_format_motor_controller.velocity_translation = trajectory.getVelocity(mpl::Timer::getMilliTime() - latest_start_time);
            msg_format_motor_controller.velocity_rotation = 0.0f;
            msg_format_motor_controller.is_controlled = true;
            break;
        case OperationMoveType::PIVOTTURN:
            msg_format_motor_controller.velocity_translation = 0.0f;
            msg_format_motor_controller.velocity_rotation = trajectory.getVelocity(mpl::Timer::getMilliTime() - latest_start_time);
            msg_format_motor_controller.is_controlled = true;
            break;
        case OperationMoveType::TRAPDIAGO:
            break;
        case OperationMoveType::EXTRALENGTH:
            if (last_operation_move == OperationMoveType::PIVOTTURN) {
                msg_format_motor_controller.velocity_translation = 0.0f;
                msg_format_motor_controller.velocity_rotation = target_velocity_translation;
                msg_format_motor_controller.is_controlled = true;
            } else {
                msg_format_motor_controller.velocity_translation = target_velocity_translation;
                msg_format_motor_controller.velocity_rotation = 0.0f;
                msg_format_motor_controller.is_controlled = true;
            }
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
