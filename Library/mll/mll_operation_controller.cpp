//******************************************************************************
// @addtogroup MLL
// @file       mll_operation_controller.cpp
// @brief      マウスの動作を統括するクラス
//******************************************************************************
#include "mll_operation_controller.h"

#if defined(STM32)
#ifndef STM32C011xx
#include "arm_math.h"
#else
#include "math.h"
#endif
#endif

#ifdef LINUX
#include "arm_math_linux.h"
using namespace plt;
#endif
#include "cmd_server.h"
#include "mpl_timer.h"
#include "msg_format_localizer.h"
#include "msg_format_motor_controller.h"
#include "msg_format_wall_analyser.h"
#include "msg_server.h"
#include "util.h"

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

    section_position.x = 0;
    section_position.y = 1;
    section_position.d = CardinalDirection::NORTH;

    target_physical_position.x = 45;
    target_physical_position.y = 45;
    target_physical_position.angle = 0;

    solver = mll::MazeSolver::getInstance();
    solver->clearFootmap();
    solver->setAlgorithm(AlgorithmType::LEFT_HAND);
    solver->destination.clear();
    solver->map.format();

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

void mll::OperationController::updateTargetPosition(float velocity_translation, float velocity_rotation) {
    target_physical_position.x -= velocity_translation * arm_sin_f32(target_physical_position.angle) * 0.001f;
    target_physical_position.y += velocity_translation * arm_cos_f32(target_physical_position.angle) * 0.001f;
    target_physical_position.angle += velocity_rotation * 0.001f;
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
        updateTargetPosition(target_velocity_translation, 0);
        msg.target_x = target_physical_position.x;
        msg.target_y = target_physical_position.y;
        msg.target_angle = target_physical_position.angle;
        msg.is_controlled = true;
        latest_start_time = mpl::Timer::getMilliTime();
        if (latest_move_distance >= slalom_params[static_cast<uint8_t>(current_operation_move)].d_before) {
            slalom_state = SlalomState::TURN;
        }
    }
    if (slalom_state == SlalomState::TURN) {
        updateTargetPosition(target_velocity_translation, trajectory.getVelocity(mpl::Timer::getMilliTime() - latest_start_time));
        msg.target_x = target_physical_position.x;
        msg.target_y = target_physical_position.y;
        msg.target_angle = target_physical_position.angle;
        msg.is_controlled = true;
        latest_move_distance = 0.f;
        if (trajectory.isEnd(mpl::Timer::getMilliTime() - latest_start_time)) {
            slalom_state = SlalomState::AFTER;
        }
    }
    if (slalom_state == SlalomState::AFTER) {
        updateTargetPosition(target_velocity_translation, 0);
        msg.target_x = target_physical_position.x;
        msg.target_y = target_physical_position.y;
        msg.target_angle = target_physical_position.angle;
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
    static msg::MsgFormatWallAnalyser msg_format_wall_analyser;
    msg_server->receiveMessage(msg::ModuleId::LOCALIZER, &msg_format_localizer);
    msg_server->receiveMessage(msg::ModuleId::WALLANALYSER, &msg_format_wall_analyser);

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
                if (cmd_format_operation_direction.type == cmd::OperationDirectionType::STAY) {
                } else if (cmd_format_operation_direction.type == cmd::OperationDirectionType::SEARCH) {
                    next_operation_move = OperationMoveType::TRAPACCEL;
                    target_move_distance = 45.f;
                    solver->destination.add(1, 0);
                    solver->destination.add(1, 1);
                    solver->destination.add(2, 0);
                    solver->destination.add(2, 1);
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

    // OperationMoveTypeがSTOPならモーターを止める
    // ただし動作開始前の場合はスキップして次の処理に移る
    if (!is_operation_move_changed && current_operation_move == OperationMoveType::STOP) {
        msg_format_motor_controller.target_x = msg_format_localizer.position_x;
        msg_format_motor_controller.target_y = msg_format_localizer.position_y;
        msg_format_motor_controller.target_angle = msg_format_localizer.position_theta;
        msg_format_motor_controller.is_controlled = false;
        msg_server->sendMessage(msg::ModuleId::MOTORCONTROLLER, &msg_format_motor_controller);
        return;
    }

    // 終了条件: 軌跡が終了した or SlalomState::END
    if (((current_operation_move == OperationMoveType::TRAPACCEL || current_operation_move == OperationMoveType::TRAPACCEL_STOP ||
          current_operation_move == OperationMoveType::PIVOTTURN) &&
         trajectory.isEnd(mpl::Timer::getMilliTime() - latest_start_time)) ||
        slalom_state == SlalomState::END || current_operation_move == OperationMoveType::STOP) {
        //
        if (current_operation_direction == cmd::OperationDirectionType::SPECIFIC) {
            // 規定動作の配列がある場合、次の動作を読み込む
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

        } else if (current_operation_direction == cmd::OperationDirectionType::SEARCH) {
            // 探索中の場合、壁センサの値から区画情報の更新をし、次の移動方向を決定
            // 区画走行が完了し、探索時の区画情報の更新

            if (current_operation_move == OperationMoveType::STOP) {
                // まだ動き始めていない場合
                next_operation_move = OperationMoveType::TRAPACCEL;
                target_move_distance = 45;
                target_velocity_translation = 0;
            } else {
                // 動き始めている場合

                // 壁情報の更新
                Walldata walldata = msg_format_wall_analyser.front_wall;
                solver->map.addWall(section_position.x, section_position.y, section_position.d, walldata);

                if (solver->destination.isInclude(misc::Point<uint16_t>{section_position.x, section_position.y})) {
                    // ゴール到達の場合

                    // FIXME: スタート地点に戻ってくる
                    next_operation_move = OperationMoveType::TRAPACCEL_STOP;
                    target_move_distance = 45;
                    target_velocity_translation = 300;
                } else {
                    // ゴールではない場合

                    // 次の移動方向を決定
                    auto next_direction = solver->getNextDirectionInSearch(section_position.x, section_position.y, section_position.d);
                    if (next_direction == FirstPersonDirection::FRONT) {
                        next_operation_move = OperationMoveType::TRAPACCEL;
                        target_move_distance = 90.f;
                        target_velocity_translation = 300.f;
                    } else if (next_direction == FirstPersonDirection::RIGHT) {
                        next_operation_move = OperationMoveType::SLALOM90SML_RIGHT;
                        target_velocity_translation = 300.f;
                    } else if (next_direction == FirstPersonDirection::LEFT) {
                        next_operation_move = OperationMoveType::SLALOM90SML_LEFT;
                        target_velocity_translation = 300.f;
                    } else {
                        next_operation_move = OperationMoveType::PIVOTTURN;
                        target_move_distance = misc::PI;
                        target_velocity_translation = 0.f;
                    }

                    // 論理座標をアップデート
                    section_position.move(next_operation_move);
                }
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
            // 現状維持でいい気がする？
            // msg_format_motor_controller.target_x = msg_format_localizer.position_x;
            // msg_format_motor_controller.target_y = msg_format_localizer.position_y;
            // msg_format_motor_controller.target_angle = msg_format_localizer.position_theta;
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
            updateTargetPosition(trajectory.getVelocity(mpl::Timer::getMilliTime() - latest_start_time), 0);
            msg_format_motor_controller.target_x = target_physical_position.x;
            msg_format_motor_controller.target_y = target_physical_position.y;
            msg_format_motor_controller.target_angle = target_physical_position.angle;
            msg_format_motor_controller.is_controlled = true;
            break;
        case OperationMoveType::TRAPACCEL_STOP:
            updateTargetPosition(trajectory.getVelocity(mpl::Timer::getMilliTime() - latest_start_time), 0);
            msg_format_motor_controller.target_x = target_physical_position.x;
            msg_format_motor_controller.target_y = target_physical_position.y;
            msg_format_motor_controller.target_angle = target_physical_position.angle;
            msg_format_motor_controller.is_controlled = true;
            break;
        case OperationMoveType::PIVOTTURN:
            updateTargetPosition(0, trajectory.getVelocity(mpl::Timer::getMilliTime() - latest_start_time));
            msg_format_motor_controller.target_x = target_physical_position.x;
            msg_format_motor_controller.target_y = target_physical_position.y;
            msg_format_motor_controller.target_angle = target_physical_position.angle;
            msg_format_motor_controller.is_controlled = true;
            break;
        case OperationMoveType::TRAPDIAGO:
            break;
        case mll::OperationMoveType::WAIT:
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
