//******************************************************************************
// @addtogroup MLL
// @file       mll_position_updater.cpp
// @brief      Activity と マウス動作の取次を行うクラス
//******************************************************************************
#include "mll_position_updater.h"

#include "mll_operation_move_type.h"
#include "mpl_timer.h"

using namespace mll;

PositionUpdater::PositionUpdater() {
    params_cache = misc::Params::getInstance()->getCachePointer();
    // FIXME: 複数の速度に対応、パラメータからの読み出し
    slalom_params_cache = misc::Params::getInstance()->getCacheSlalomPointer(300);
    trajectory = Trajectory();
    current_move = OperationMoveType::STOP;
    current_move_distance = 0.f;
    current_velocity = MouseVelocity{0, 0};
    current_velocity_translation_end = 0.f;
    localizer = Localizer::getInstance();
    msg_server = msg::MessageServer::getInstance();
    cmd_server = cmd::CommandServer::getInstance();

    // NOTE: スタート位置に初期値を設定
    // TODO: フェイルセーフ時に速度とcurrent_moveはリセットするが、位置はリセットしないようにしたい
    reset(MousePhysicalPosition{45, 45, 0});
    reset(MouseVelocity{0, 0});
}

void PositionUpdater::updateTargetPosition(const float velocity_translation, const float velocity_rotation) {
#if defined(STM32)
#ifndef STM32C011xx
    target_physical_position.x -= velocity_translation * arm_sin_f32(target_physical_position.angle) * 0.001f;
    target_physical_position.y += velocity_translation * arm_cos_f32(target_physical_position.angle) * 0.001f;
#else
    target_physical_position.x -= velocity_translation * sinf(target_physical_position.angle) * 0.001f;
    target_physical_position.y += velocity_translation * cosf(target_physical_position.angle) * 0.001f;
#endif
#elif defined(LINUX)
    target_physical_position.x -= velocity_translation * sin(target_physical_position.angle) * 0.001f;
    target_physical_position.y += velocity_translation * cos(target_physical_position.angle) * 0.001f;
#endif
    target_physical_position.angle += velocity_rotation * 0.001f;
}

void PositionUpdater::initSlalom(const float rightleft) {
    trajectory.init(TrajectoryCalcType::TIME, TrajectoryFormType::TRAPEZOID,
                    rightleft * slalom_params_cache[static_cast<uint8_t>(current_move)].acc_rad,
                    rightleft * slalom_params_cache[static_cast<uint8_t>(current_move)].deg, 0.f,
                    rightleft * slalom_params_cache[static_cast<uint8_t>(current_move)].max_v_rad, 0.f);
    slalom_state = SlalomState::BEFORE;
    slalom_rightleft = rightleft;
    slalom_move_distance = 0.f;
}

void PositionUpdater::runSlalom() {
    if (slalom_state == SlalomState::BEFORE) {
        current_velocity.rotation = 0;
        updateTargetPosition(current_velocity.translation, 0);
        slalom_move_distance += current_velocity.translation * 0.001f;
        if (slalom_move_distance >= slalom_params_cache[static_cast<uint8_t>(current_move)].d_before) {
            slalom_start_time = mpl::Timer::getMilliTime();
            slalom_state = SlalomState::TURN;
        }
    }
    if (slalom_state == SlalomState::TURN) {
        current_velocity.rotation = trajectory.getVelocity(mpl::Timer::getMilliTime() - slalom_start_time);
        updateTargetPosition(current_velocity.translation, current_velocity.rotation);
        if (trajectory.isEnd(mpl::Timer::getMilliTime() - slalom_start_time)) {
            slalom_move_distance = 0.f;
            slalom_state = SlalomState::AFTER;
        }
    }
    if (slalom_state == SlalomState::AFTER) {
        current_velocity.rotation = 0;
        updateTargetPosition(current_velocity.translation, 0);
        slalom_move_distance += current_velocity.translation * 0.001f;
        if (slalom_move_distance >= slalom_params_cache[static_cast<uint8_t>(current_move)].d_after) {
            slalom_state = SlalomState::END;
        }
    }
}

void PositionUpdater::initMove() {
    switch (current_move) {
        case OperationMoveType::STOP:
            current_velocity.translation = 0.f;
            current_velocity.rotation = 0.f;
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
            // FIXME: 最高速、終端速を適切にする
            trajectory.init(TrajectoryCalcType::TIME, TrajectoryFormType::TRAPEZOID, 2000.f, current_move_distance, current_velocity.translation,
                            300.f, 300.f);
            current_velocity.rotation = 0.f;
            break;
        case OperationMoveType::TRAPACCEL_STOP:
            trajectory.init(TrajectoryCalcType::TIME, TrajectoryFormType::TRAPEZOID, 2000.f, current_move_distance, current_velocity.translation,
                            300.f, 0.f);
            current_velocity.rotation = 0.f;
            break;
        case OperationMoveType::PIVOTTURN_LEFT:
            trajectory.init(TrajectoryCalcType::TIME, TrajectoryFormType::TRAPEZOID, 100.f, current_move_distance, 0.0f, 10.f, 0);
            current_velocity.translation = 0.f;
            break;
        case OperationMoveType::PIVOTTURN_RIGHT:
            trajectory.init(TrajectoryCalcType::TIME, TrajectoryFormType::TRAPEZOID, -100.f, current_move_distance, 0.0f, -10.f, 0);
            current_velocity.translation = 0.f;
            break;
        case OperationMoveType::WAIT:
            start_time = mpl::Timer::getMilliTime();
            current_velocity.translation = 0.f;
            current_velocity.rotation = 0.f;
            break;
        case OperationMoveType::CORRECTION_FRONT:
            start_time = mpl::Timer::getMilliTime();
            current_velocity.translation = 0.f;
            current_velocity.rotation = 0.f;
            break;
        case OperationMoveType::LENGTH:
        case OperationMoveType::UNDEFINED:
        default:
            return;
    };
}

void PositionUpdater::update() {
    last_update_time = mpl::Timer::getMilliTime();

    switch (current_move) {
        case OperationMoveType::STOP:
            // msg_format_motor_controller.is_controlled = true;
            // NOTE: モーター制御のOFFは別モジュールの役割ということにする
            break;
        case OperationMoveType::SLALOM90SML_RIGHT:
            runSlalom();
            break;
        case OperationMoveType::SLALOM90SML_LEFT:
            runSlalom();
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
            // updateTargetPosition(trajectory.getVelocity(mpl::Timer::getMilliTime() - start_time), 0);
            current_velocity.translation = trajectory.getVelocity(mpl::Timer::getMilliTime() - start_time);
            current_velocity.rotation = 0.f;
            break;
        case OperationMoveType::TRAPACCEL_STOP:
            // updateTargetPosition(trajectory.getVelocity(mpl::Timer::getMilliTime() - start_time), 0);
            current_velocity.translation = trajectory.getVelocity(mpl::Timer::getMilliTime() - start_time);
            current_velocity.rotation = 0.f;
            break;
        case OperationMoveType::PIVOTTURN_LEFT:
            // updateTargetPosition(0, trajectory.getVelocity(mpl::Timer::getMilliTime() - start_time));
            current_velocity.translation = 0.f;
            current_velocity.rotation = trajectory.getVelocity(mpl::Timer::getMilliTime() - start_time);
            break;
        case OperationMoveType::PIVOTTURN_RIGHT:
            // updateTargetPosition(0, trajectory.getVelocity(mpl::Timer::getMilliTime() - start_time));
            current_velocity.translation = 0.f;
            current_velocity.rotation = trajectory.getVelocity(mpl::Timer::getMilliTime() - start_time);
            break;
        case OperationMoveType::WAIT:
            // Do nothing
            break;
        case OperationMoveType::CORRECTION_FRONT:
            msg_server->receiveMessage(msg::ModuleId::WALLANALYSER, &msg_wall_analyser);
            current_velocity.translation = params_cache->wall_position_front_translation_kp * msg_wall_analyser.distance_from_center * (-1.f);
            current_velocity.rotation = params_cache->wall_position_front_rotation_kp * msg_wall_analyser.angle_from_front;
            if (current_velocity.translation > 0.f) {
                current_velocity.translation = misc::min(current_velocity.translation, params_cache->wall_position_front_translation_max);
            } else {
                current_velocity.translation = misc::max(current_velocity.translation, -params_cache->wall_position_front_translation_max);
            }
            if (current_velocity.rotation > 0.f) {
                current_velocity.rotation = misc::min(current_velocity.rotation, params_cache->wall_position_front_rotation_max);
            } else {
                current_velocity.rotation = misc::max(current_velocity.rotation, -params_cache->wall_position_front_rotation_max);
            }
            break;
        case OperationMoveType::LENGTH:
        case OperationMoveType::UNDEFINED:
            // msg_format_motor_controller.is_controlled = false;
            break;
        default:
            break;
    }
}

const MousePhysicalPosition PositionUpdater::getTargetPhysicalPosition() const {
    return target_physical_position;
}

const MouseVelocity PositionUpdater::getTargetVelocity() const {
    return MouseVelocity{current_velocity.translation, current_velocity.rotation};
}

bool PositionUpdater::isNextWallReady() {
    // TODO: タイミングをずらすなら実装し直す
    return isMoveComplete();
}

bool PositionUpdater::isMoveComplete() {
    // TODO: trajectory.isEnd の境界条件を確認しておく
    if (current_move == OperationMoveType::STOP) {
        return true;
    } else if (current_move == OperationMoveType::WAIT) {
        if (last_update_time - start_time >= current_move_distance) {
            return true;
        } else {
            return false;
        }
    } else if (current_move == OperationMoveType::CORRECTION_FRONT) {
        if (misc::abs(current_velocity.translation) <= 0.1f && misc::abs(current_velocity.rotation) <= 0.1f) {
            cmd_ui_out.type = mll::UiOutputEffect::DEBUG1;
            cmd_server->push(cmd::CommandId::UI_OUT, &cmd_ui_out);
            return true;
        } else {
            return false;
        }
    } else if (current_move == OperationMoveType::TRAPACCEL || current_move == OperationMoveType::TRAPACCEL_STOP ||
               current_move == OperationMoveType::PIVOTTURN_LEFT || current_move == OperationMoveType::PIVOTTURN_RIGHT ||
               current_move == OperationMoveType::WAIT) {  // スラローム以外で動作中の場合
        return trajectory.isEnd(last_update_time - start_time);
    } else if (slalom_state == SlalomState::END) {  // スラロームで動作中の場合
        return true;
    } else {
        return false;
    }
}

void PositionUpdater::setNextMove(const OperationMoveType move, const float distance) {
    current_move = move;
    current_move_distance = distance;
    start_physical_position = target_physical_position;
    start_time = mpl::Timer::getMilliTime();
    initMove();
}

void PositionUpdater::reset(const MousePhysicalPosition& position) {
    // TODO: trajectory のリセット
    target_physical_position = position;
    last_update_time = 0;
    start_physical_position = position;
    start_time = 0;
    current_move = OperationMoveType::STOP;
    current_move_distance = 0;

    // Localizer の初期化
    localizer->setPosition(position.x, position.y, position.angle);
}

void PositionUpdater::reset(const MouseVelocity& velocity) {
    current_velocity.translation = velocity.translation;
    current_velocity.rotation = velocity.rotation;
}

PositionUpdater* PositionUpdater::getInstance() {
    static PositionUpdater instance;
    return &instance;
}
