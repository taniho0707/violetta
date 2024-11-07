//******************************************************************************
// @addtogroup MLL
// @file       mll_operation_coordinator.cpp
// @brief      Activity と マウス動作の取次を行うクラス
//******************************************************************************
#include "mll_operation_coordinator.h"

using namespace mll;

OperationCoordinator::OperationCoordinator() {
    enabled_motor_control = false;
    current_state = OperationCoordinatorResult::IDLE_WITHOUT_MOTOR_CONTROL;

    position_updater = PositionUpdater::getInstance();
    coordinate_director = CoordinateDirector::getInstance();

    msg_server = msg::MessageServer::getInstance();
}

OperationCoordinatorResult OperationCoordinator::enableMotorControl() {
    if (enabled_motor_control) {
        return OperationCoordinatorResult::ERROR_ALREADY_RUNNING;
    }
    enabled_motor_control = true;
    current_state = OperationCoordinatorResult::IDLE_WITH_MOTOR_CONTROL;
    return OperationCoordinatorResult::SUCCESS;
}

OperationCoordinatorResult OperationCoordinator::disableMotorControl() {
    // TODO: 動作中はモーター制御を無効にできないようにする
    enabled_motor_control = false;
    current_state = OperationCoordinatorResult::IDLE_WITHOUT_MOTOR_CONTROL;
    return OperationCoordinatorResult::SUCCESS;
}

OperationCoordinatorResult OperationCoordinator::runSearch(SearchOptions opt) {
    // 他の動作中は命令を受け付けない
    // clang-format off
    if (   current_state != OperationCoordinatorResult::IDLE_WITH_MOTOR_CONTROL
        && current_state != OperationCoordinatorResult::IDLE_WITHOUT_MOTOR_CONTROL
        && current_state != OperationCoordinatorResult::IDLE) {
        return OperationCoordinatorResult::ERROR_ALREADY_RUNNING;
    }
    // clang-format on

    search_options = opt;
    enabled_motor_control = true;
    current_state = OperationCoordinatorResult::RUNNING_SEARCH;

    coordinate_director->setTargetSection(search_options.goals);
    coordinate_director->setAlgorithm(search_options.algorithm);

    moves_current_index = 0;
    moves_current_length = 1;
    moves[0] = OperationMoveCombination{OperationMoveType::TRAPACCEL, 45.f};

    return OperationCoordinatorResult::SUCCESS;
}

OperationCoordinatorResult OperationCoordinator::runShort(ShortOptions opt) {
    return OperationCoordinatorResult::FATAL_ERROR;
}

OperationCoordinatorResult OperationCoordinator::runSpecific(OperationMoveCombination* moves, uint16_t length) {
    // 他の動作中は命令を受け付けない
    // clang-format off
    if (   current_state != OperationCoordinatorResult::IDLE_WITH_MOTOR_CONTROL
        && current_state != OperationCoordinatorResult::IDLE_WITHOUT_MOTOR_CONTROL
        && current_state != OperationCoordinatorResult::IDLE) {
        return OperationCoordinatorResult::ERROR_ALREADY_RUNNING;
    }
    // clang-format on

    if (length > MAX_MOVE_LENGTH) {
        return OperationCoordinatorResult::ERROR_LENGTH_TOO_LONG;
    }
    moves_current_length = length;
    moves_current_index = 0;
    for (uint16_t i = 0; i < length; i++) {
        this->moves[i] = moves[i];
    }

    enabled_motor_control = true;
    current_state = OperationCoordinatorResult::RUNNING_SPECIFIC;

    // TODO: PositionUpdater の初期化をして、離散値が出ないようにする
    resetPosition(MousePhysicalPosition{45.f, 45.f, 0.f});

    return OperationCoordinatorResult::SUCCESS;
}

OperationCoordinatorResult OperationCoordinator::state() {
    return current_state;
}

const MouseSectionPosition OperationCoordinator::getCurrentSection() const {
    return coordinate_director->getCurrentSection();
}

void OperationCoordinator::resetPosition(const MousePhysicalPosition& position) {
    position_updater->reset(position);
}

void OperationCoordinator::interruptPeriodic() {
    switch (current_state) {
        case OperationCoordinatorResult::IDLE_WITH_MOTOR_CONTROL:
            position_updater->update();
            break;
        case OperationCoordinatorResult::RUNNING_SEARCH:
            // モーターのフェイルセーフを検知
            msg_server->receiveMessage(msg::ModuleId::MOTORCONTROLLER_INTERNAL, &msg_format_motor_controller_internal);
            if (misc::abs(msg_format_motor_controller_internal.integral_translation) > 15000 ||
                misc::abs(msg_format_motor_controller_internal.integral_rotation) > 500) {
                current_state = OperationCoordinatorResult::ERROR_MOTOR_FAILSAFE;
                enabled_motor_control = false;
                position_updater->reset(MousePhysicalPosition{45, 45, 0});
                position_updater->reset(MouseVelocity{0, 0});
                break;
            }
            // 1動作 (通常は1区画) が完了していれば次の動作を設定する
            if (position_updater->isMoveComplete()) {
                if (coordinate_director->isEnd()) {
                    current_state = OperationCoordinatorResult::IDLE_WITH_MOTOR_CONTROL;
                    // TODO: 停止する
                } else if (moves_current_index >= moves_current_length) {
                    moves_current_index = 0;
                    coordinate_director->getNextMove(moves, moves_current_length);
                    position_updater->setNextMove(moves[moves_current_index].type, moves[moves_current_index].distance, true);
                    moves_current_index++;
                } else {
                    position_updater->setNextMove(moves[moves_current_index].type, moves[moves_current_index].distance, true);
                    moves_current_index++;
                }
            }
            position_updater->update();
            break;
        case OperationCoordinatorResult::RUNNING_SHORT:
            break;
        case OperationCoordinatorResult::RUNNING_SPECIFIC:
            // PositionUpdater の動作が完了していれば次の動作を設定する
            if (position_updater->isMoveComplete()) {
                if (moves_current_index >= moves_current_length) {
                    current_state = OperationCoordinatorResult::IDLE_WITH_MOTOR_CONTROL;
                } else {
                    position_updater->setNextMove(moves[moves_current_index].type, moves[moves_current_index].distance, false);
                }
                moves_current_index++;
            }
            position_updater->update();
            break;
        default:
            break;
    }

    if (enabled_motor_control) {
        auto target = position_updater->getTargetVelocity();
        msg_format_motor_controller.velocity_translation = target.translation;
        msg_format_motor_controller.velocity_rotation = target.rotation;
        msg_format_motor_controller.is_controlled = true;
        msg_server->sendMessage(msg::ModuleId::MOTORCONTROLLER, &msg_format_motor_controller);
    } else {
        msg_format_motor_controller.is_controlled = false;
        msg_server->sendMessage(msg::ModuleId::MOTORCONTROLLER, &msg_format_motor_controller);
    }
}

OperationCoordinator* OperationCoordinator::getInstance() {
    static OperationCoordinator instance;
    return &instance;
}
