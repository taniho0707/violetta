//******************************************************************************
// @addtogroup ACT
// @file       act_system_identification.cpp
// @brief      SystemIdentification Activity
//******************************************************************************
#include "act_system_identification.h"

#include "cmd_format.h"
#include "cmd_server.h"
#include "mll_logger.h"
#include "mll_motor_controller.h"
#include "mpl_timer.h"
#include "msg_format_battery.h"
#include "msg_format_encoder.h"
#include "msg_format_imu.h"
#include "msg_format_motor.h"
#include "msg_format_motor_controller.h"
#include "msg_server.h"

using namespace act;

void SystemIdentificationActivity::init(ActivityParameters &params) {
    system_identification_type = params.system_identification_type;
}

#ifndef MOUSE_LAZULI_SENSOR
Status SystemIdentificationActivity::run() {
    auto cmd_server = cmd::CommandServer::getInstance();
    auto cmd_ui_out = cmd::CommandFormatUiOut{0};

    // TODO: モーター制御が止まっていることを確認する
    // for debug
    auto msg_server = msg::MessageServer::getInstance();
    msg::MsgFormatEncoder msg_encoder = msg::MsgFormatEncoder();
    msg::MsgFormatImu msg_imu = msg::MsgFormatImu();
    msg::MsgFormatMotorController msg_motor_controller = msg::MsgFormatMotorController();
    msg::MsgFormatMotor msg_motor = msg::MsgFormatMotor();

    mpl::Timer::sleepMs(1000);

    float voltage_l = 0;
    float voltage_r = 0;
    switch (system_identification_type) {
        case SystemIdentificationType::STEP_TRANSLATION:
            voltage_l = 2.f;
            voltage_r = 2.f;
            break;
        case SystemIdentificationType::STEP_ROTATION:
            voltage_l = 2.f;
            voltage_r = -2.f;
            break;
        case SystemIdentificationType::M_SEQUENCE_TRANSLATION:
            break;
        case SystemIdentificationType::M_SEQUENCE_ROTATION:
            break;
        default:
            break;
    }

    uint32_t start_time = mpl::Timer::getMicroTime();
    uint32_t duration_time = 1000 * 1000 * 3;  // 3s

    // Logger setting
    auto logger = mll::Logger::getInstance();
    const uint32_t LOG_ADDRESS = 0x20030000;
    constexpr uint16_t ALL_LOG_LENGTH = 0x20000 / sizeof(mll::LogFormatAll);
    auto logconfig = mll::LogConfig{mll::LogType::ALL, mll::LogDestinationType::INTERNAL_RAM, ALL_LOG_LENGTH, LOG_ADDRESS};
    logger->init(logconfig);
    logger->startPeriodic(mll::LogType::ALL, 10);

    voltage_l = 1.5f;
    voltage_r = -1.5f;

    mll::MotorController::getInstance()->startOverrideControl();

    while (true) {
        // モーターに指令を送る
        static msg::MsgFormatMotor msg_motor;
        static msg::MsgFormatBattery msg_battery;
        msg_server->receiveMessage(msg::ModuleId::BATTERY, &msg_battery);
        msg_motor.duty_l = voltage_l / msg_battery.battery;
        msg_motor.duty_r = voltage_r / msg_battery.battery;
        msg_motor.duty_suction = 0;
        msg_server->sendMessage(msg::ModuleId::MOTOR, &msg_motor);

        mpl::Timer::sleepMs(1);

        if (mpl::Timer::getMicroTime() - start_time > duration_time) {
            break;
        }
    }

    msg_motor.duty_l = 0;
    msg_motor.duty_r = 0;
    msg_motor.duty_suction = 0;
    msg_server->sendMessage(msg::ModuleId::MOTOR, &msg_motor);

    mll::MotorController::getInstance()->stopOverrideControl();

    logger->stopPeriodic(mll::LogType::ALL);

    mpl::Timer::sleepMs(1000);

    cmd_ui_out.type = mll::UiOutputEffect::SEARCH_COMPLETE;
    cmd_server->push(cmd::CommandId::UI_OUT, &cmd_ui_out);

    return Status::SUCCESS;
}
#endif  // ifndef MOUSE_LAZULI_SENSOR

#ifdef MOUSE_LAZULI_SENSOR
Status SystemIdentificationActivity::run() {
    return Status::ERROR;
}
#endif  // MOUSE_LAZULI_SENSOR

void SystemIdentificationActivity::finalize(ActivityParameters &params) {
    params.next_activity = Activities::SELECT_NEXT;
}
