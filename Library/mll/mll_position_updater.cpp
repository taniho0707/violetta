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
    // NOTE: スタート位置に初期値を設定
    reset(MousePhysicalPosition{45, 45, 0});
}

void PositionUpdater::updateTargetPosition(const float velocity_translation, const float velocity_rotation) {
    target_physical_position.x -= velocity_translation * arm_sin_f32(target_physical_position.angle) * 0.001f;
    target_physical_position.y += velocity_translation * arm_cos_f32(target_physical_position.angle) * 0.001f;
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
        updateTargetPosition(current_velocity_translation, 0);
        slalom_move_distance += current_velocity_translation * 0.001f;
        if (slalom_move_distance >= slalom_params_cache[static_cast<uint8_t>(current_move)].d_before) {
            slalom_start_time = mpl::Timer::getMilliTime();
            slalom_state = SlalomState::TURN;
        }
    }
    if (slalom_state == SlalomState::TURN) {
        updateTargetPosition(current_velocity_translation, trajectory.getVelocity(mpl::Timer::getMilliTime() - slalom_start_time));
        if (trajectory.isEnd(mpl::Timer::getMilliTime() - slalom_start_time)) {
            slalom_move_distance = 0.f;
            slalom_state = SlalomState::AFTER;
        }
    }
    if (slalom_state == SlalomState::AFTER) {
        updateTargetPosition(current_velocity_translation, 0);
        if (slalom_move_distance >= slalom_params_cache[static_cast<uint8_t>(current_move)].d_after) {
            slalom_state = SlalomState::END;
        }
    }
}

void PositionUpdater::initMove() {
    switch (current_move) {
        case OperationMoveType::STOP:
            current_velocity_translation = 0.f;
            current_velocity_rotation = 0.f;
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
            trajectory.init(TrajectoryCalcType::TIME, TrajectoryFormType::TRAPEZOID, 2000.f, current_move_distance, current_velocity_translation,
                            300.f, current_velocity_translation_end);
            current_velocity_rotation = 0.f;
            break;
        case OperationMoveType::PIVOTTURN:
            trajectory.init(TrajectoryCalcType::TIME, TrajectoryFormType::TRAPEZOID, 100.f, current_move_distance, 0.0f, 15.f, 0);
            current_velocity_translation = 0.f;
            break;
        case OperationMoveType::LENGTH:
        case OperationMoveType::UNDEFINED:
            break;
        default:
            break;
    }
}

void PositionUpdater::update() {
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
            updateTargetPosition(trajectory.getVelocity(mpl::Timer::getMilliTime() - start_time), 0);
            break;
        case OperationMoveType::PIVOTTURN:
            updateTargetPosition(0, trajectory.getVelocity(mpl::Timer::getMilliTime() - start_time));
            break;
        case mll::OperationMoveType::WAIT:
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

bool PositionUpdater::isNextWallReady() {
    // TODO: タイミングをずらすなら実装し直す
    return isMoveComplete();
}

bool PositionUpdater::isMoveComplete() {
    // TODO: trajectory.isEnd の境界条件を確認しておく
    if (current_move == OperationMoveType::STOP) {
        return true;
    } else if (trajectory.isEnd(last_update_time - start_time)) {
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
}

PositionUpdater* PositionUpdater::getInstance() {
    static PositionUpdater instance;
    return &instance;
}
