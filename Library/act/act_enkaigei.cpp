//******************************************************************************
// @addtogroup ACT
// @file       act_debug.cpp
// @brief      Debug Activity
//******************************************************************************
#include "act_enkaigei.h"

#include "cmd_format.h"
#include "cmd_server.h"
#include "mll_operation_coordinator.h"
#include "mpl_debug.h"
#include "mpl_timer.h"
#include "msg_format_encoder.h"
#include "msg_format_imu.h"
#include "msg_format_motor.h"

using namespace act;

void EnkaigeiActivity::init(ActivityParameters &params) {}

#ifndef MOUSE_LAZULI_SENSOR
Status EnkaigeiActivity::run() {
    auto operation_coordinator = mll::OperationCoordinator::getInstance();
    auto cmd_server = cmd::CommandServer::getInstance();
    auto cmd_ui_out = cmd::CommandFormatUiOut{0};
    // auto cmd_ui_in = cmd::CommandFormatUiIn{0};

    // TODO: モーター制御が止まっていることを確認する

    operation_coordinator->enableMotorControl();

    // for debug
    auto message = msg::MessageServer::getInstance();
    msg::MsgFormatEncoder msg_encoder = msg::MsgFormatEncoder();
    msg::MsgFormatImu msg_imu = msg::MsgFormatImu();
    msg::MsgFormatMotorController msg_motor_controller = msg::MsgFormatMotorController();
    msg::MsgFormatMotor msg_motor = msg::MsgFormatMotor();
    cmd::CommandFormatDebugTx cmd_debug_tx = cmd::CommandFormatDebugTx();
    auto debug = mpl::Debug::getInstance();

    while (true) {
        message->receiveMessage(msg::ModuleId::ENCODER, &msg_encoder);
        message->receiveMessage(msg::ModuleId::IMU, &msg_imu);
        message->receiveMessage(msg::ModuleId::MOTORCONTROLLER, &msg_motor_controller);
        message->receiveMessage(msg::ModuleId::MOTOR, &msg_motor);
        // clang-format off
        // Time, EncLeft, EncRight, Gyro, AccX, AccY, AccZ, DutyL, DutyR, MotorControl, VelTrans, VelRot
        cmd_debug_tx.len = debug->format(
            cmd_debug_tx.message,
            "%10d,% 7.2f,% 7.2f,% 8.2f, % 6.1f, % 6.1f, % 6.1f,"
            "% 8.2f,% 8.2f,%s,% 6.2f,% 6.2f\n",
            mpl::Timer::getMicroTime(), msg_encoder.left, msg_encoder.right,
            msg_imu.gyro_yaw,
            msg_imu.acc_x / 1000, msg_imu.acc_y / 1000, msg_imu.acc_z / 1000,
            msg_motor.duty_l, msg_motor.duty_r,
            (msg_motor_controller.is_controlled ? "ON " : "OFF"),
            msg_motor_controller.velocity_translation, msg_motor_controller.velocity_rotation
            );
        cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
        // clang-format on

        // TODO: 宴会芸を抜けるパスを用意する
        // ui->stop() が実行されたあとのため、UI IN は来ない
        // if (cmd_server->length(cmd::CommandId::UI_IN) > 0) {
        //     cmd_server->pop(cmd::CommandId::UI_IN, &cmd_ui_in);
        //     if (cmd_ui_in.type == mll::UiInputEffect::WALLSENSOR_LEFT) {
        //         break;
        //     }
        // }
        mpl::Timer::sleepMs(1);
    }

    operation_coordinator->disableMotorControl();
    cmd_ui_out.type = mll::UiOutputEffect::SEARCH_COMPLETE;
    cmd_server->push(cmd::CommandId::UI_OUT, &cmd_ui_out);

    return Status::SUCCESS;
}
#endif  // ifndef MOUSE_LAZULI_SENSOR

#ifdef MOUSE_LAZULI_SENSOR
Status EnkaigeiActivity::run() {
    return Status::ERROR;
}
#endif  // MOUSE_LAZULI_SENSOR

void EnkaigeiActivity::finalize(ActivityParameters &params) {
    params.next_activity = Activities::SELECT_NEXT;
}
