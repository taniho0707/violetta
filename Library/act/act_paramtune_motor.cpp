//******************************************************************************
// @addtogroup ACT
// @file       act_paramtune_motor.cpp
// @brief      ParamtuneMotor Activity
//******************************************************************************
#include "act_paramtune_motor.h"

#include "cmd_format.h"
#include "cmd_server.h"
#include "mll_logger.h"
#include "mll_operation_coordinator.h"
#include "mpl_debug.h"
#include "mpl_timer.h"
#include "msg_format_encoder.h"
#include "msg_format_imu.h"
#include "msg_format_motor.h"

using namespace act;

void ParamtuneMotorActivity::init(ActivityParameters &params) {
    motor_tune_type = params.motor_tune_type;
    motor_tune_right = params.motor_tune_right;
}

#ifndef MOUSE_LAZULI_SENSOR
Status ParamtuneMotorActivity::run() {
    auto operation_coordinator = mll::OperationCoordinator::getInstance();
    auto cmd_server = cmd::CommandServer::getInstance();
    auto cmd_ui_out = cmd::CommandFormatUiOut{0};
    // auto cmd_ui_in = cmd::CommandFormatUiIn{0};

    // TODO: モーター制御が止まっていることを確認する

    operation_coordinator->resetPosition(mll::MousePhysicalPosition{45.f, 45.f, 0.f});
    operation_coordinator->enableMotorControl();

    // for debug
    auto message = msg::MessageServer::getInstance();
    msg::MsgFormatEncoder msg_encoder = msg::MsgFormatEncoder();
    msg::MsgFormatImu msg_imu = msg::MsgFormatImu();
    msg::MsgFormatMotorController msg_motor_controller = msg::MsgFormatMotorController();
    msg::MsgFormatMotor msg_motor = msg::MsgFormatMotor();
    cmd::CommandFormatDebugTx cmd_debug_tx = cmd::CommandFormatDebugTx();
    auto debug = mpl::Debug::getInstance();

    mpl::Timer::sleepMs(1000);

    mll::OperationMoveCombination moves[48];
    int moves_length = 0;

    switch (motor_tune_type) {
        case act::MotorParameterTuneType::STRAIGHT:
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::TRAPACCEL_STOP,
                .distance = 90.f * 5,
            };
            break;
        case act::MotorParameterTuneType::PIVOTTURN:
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::PIVOTTURN_LEFT,
                .distance = 2 * misc::PI * 5,
            };
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::WAIT,
                .distance = 2000,
            };
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::PIVOTTURN_LEFT,
                .distance = 2 * misc::PI * 1,
            };
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::WAIT,
                .distance = 2000,
            };
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::PIVOTTURN_LEFT,
                .distance = 2 * misc::PI * 1,
            };
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::WAIT,
                .distance = 2000,
            };
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::PIVOTTURN_LEFT,
                .distance = 2 * misc::PI * 1,
            };
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::WAIT,
                .distance = 2000,
            };
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::PIVOTTURN_RIGHT,
                .distance = 2 * misc::PI * 5,
            };
            break;
        case act::MotorParameterTuneType::STAY:
            break;
        case act::MotorParameterTuneType::SMALLTURN_SINGLE:
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::TRAPACCEL,
                .distance = 90.f,
            };
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = (motor_tune_right ? mll::OperationMoveType::SLALOM90SML_RIGHT : mll::OperationMoveType::SLALOM90SML_LEFT),
                .distance = 0.f,
            };
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::TRAPACCEL_STOP,
                .distance = 90.f,
            };
            break;
        case act::MotorParameterTuneType::SMALLTURN_CONTINUOUS:
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::TRAPACCEL,
                .distance = 90.f,
            };
            for (int i = 0; i < 16; ++i) {
                moves[moves_length++] = mll::OperationMoveCombination{
                    .type = mll::OperationMoveType::TRAPACCEL,
                    .distance = 90.f,
                };
                moves[moves_length++] = mll::OperationMoveCombination{
                    .type = (motor_tune_right ? mll::OperationMoveType::SLALOM90SML_RIGHT : mll::OperationMoveType::SLALOM90SML_LEFT),
                    .distance = 0.f,
                };
            }
            moves[moves_length++] = mll::OperationMoveCombination{
                .type = mll::OperationMoveType::TRAPACCEL_STOP,
                .distance = 90.f,
            };
            break;
        case act::MotorParameterTuneType::OVERALLTURN:
            break;
        default:
            break;
    }

    // Logger setting
    auto logger = mll::Logger::getInstance();
    const uint32_t LOG_ADDRESS = 0x20030000;
    constexpr uint16_t ALL_LOG_LENGTH = 0x20000 / sizeof(mll::LogFormatAll);
    auto logconfig = mll::LogConfig{mll::LogType::ALL, mll::LogDestinationType::INTERNAL_RAM, ALL_LOG_LENGTH, LOG_ADDRESS};
    logger->init(logconfig);
    logger->startPeriodic(mll::LogType::ALL, 2);

    operation_coordinator->runSpecific(moves, moves_length);

    while (true) {
        message->receiveMessage(msg::ModuleId::ENCODER, &msg_encoder);
        message->receiveMessage(msg::ModuleId::IMU, &msg_imu);
        message->receiveMessage(msg::ModuleId::MOTORCONTROLLER, &msg_motor_controller);
        message->receiveMessage(msg::ModuleId::MOTOR, &msg_motor);

        // // clang-format off
        // // Time, EncLeft, EncRight, Gyro, AccX, AccY, AccZ, DutyL, DutyR, MotorControl, VelTrans, VelRot
        // cmd_debug_tx.len = debug->format(
        //     cmd_debug_tx.message,
        //     "%10d,% 7.2f,% 7.2f,% 8.2f, % 6.1f, % 6.1f, % 6.1f,"
        //     "% 8.2f,% 8.2f,%s,% 6.2f,% 6.2f\n",
        //     mpl::Timer::getMicroTime(), msg_encoder.left, msg_encoder.right,
        //     msg_imu.gyro_yaw,
        //     msg_imu.acc_x / 1000, msg_imu.acc_y / 1000, msg_imu.acc_z / 1000,
        //     msg_motor.duty_l, msg_motor.duty_r,
        //     (msg_motor_controller.is_controlled ? "ON " : "OFF"),
        //     msg_motor_controller.velocity_translation, msg_motor_controller.velocity_rotation
        //     );
        // cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
        // // clang-format on

        auto state = operation_coordinator->state();
        if (state != mll::OperationCoordinatorResult::RUNNING_SPECIFIC) {
            break;
        }

        mpl::Timer::sleepMs(1);
    }

    logger->stopPeriodic(mll::LogType::ALL);

    mpl::Timer::sleepMs(1000);
    operation_coordinator->disableMotorControl();

    cmd_ui_out.type = mll::UiOutputEffect::SEARCH_COMPLETE;
    cmd_server->push(cmd::CommandId::UI_OUT, &cmd_ui_out);

    return Status::SUCCESS;
}
#endif  // ifndef MOUSE_LAZULI_SENSOR

#ifdef MOUSE_LAZULI_SENSOR
Status ParamtuneMotorActivity::run() {
    return Status::ERROR;
}
#endif  // MOUSE_LAZULI_SENSOR

void ParamtuneMotorActivity::finalize(ActivityParameters &params) {
    params.next_activity = Activities::SELECT_NEXT;
}
