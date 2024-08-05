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

    msg_server = msg::MessageServer::getInstance();
}

OperationCoordinatorResult OperationCoordinator::enableMotorControl() {
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

OperationCoordinatorResult OperationCoordinator::runSearch(AlgorithmType type, SearchOptions opt) {}

OperationCoordinatorResult OperationCoordinator::runShort(ShortOptions opt) {}

OperationCoordinatorResult OperationCoordinator::runSpecific(OperationMoveCombination* moves, uint16_t length) {
    // 他の動作中は命令を受け付けない
    if (current_state != OperationCoordinatorResult::IDLE_WITH_MOTOR_CONTROL &&
        current_state != OperationCoordinatorResult::IDLE_WITHOUT_MOTOR_CONTROL && current_state != OperationCoordinatorResult::IDLE) {
        return OperationCoordinatorResult::ERROR_ALREADY_RUNNING;
    }

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

    return OperationCoordinatorResult::SUCCESS;
}

OperationCoordinatorResult OperationCoordinator::state() {
    return current_state;
}

void OperationCoordinator::resetPosition(const MousePhysicalPosition& position) {
    position_updater->reset(position);
}

void OperationCoordinator::interruptPeriodic() {
    switch (current_state) {
        case OperationCoordinatorResult::RUNNING_SPECIFIC:
            // PositionUpdater の動作が完了していれば次の動作を設定する
            if (position_updater->isMoveComplete()) {
                if (moves_current_index >= moves_current_length) {
                    current_state = OperationCoordinatorResult::IDLE_WITH_MOTOR_CONTROL;
                } else {
                    position_updater->setNextMove(moves[moves_current_index].type, moves[moves_current_index].distance);
                }
                moves_current_index++;
            }

            // PositionUpdater の更新をする
            position_updater->update();

            break;
        default:
            break;
    }

    if (enabled_motor_control) {
        auto target = position_updater->getTargetPhysicalPosition();
        msg_format_motor_controller.target_x = target.x;
        msg_format_motor_controller.target_y = target.y;
        msg_format_motor_controller.target_angle = target.angle;
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
