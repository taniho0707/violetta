//******************************************************************************
// @addtogroup ACT
// @file       act_initialize.cpp
// @brief      Initialize Activity
//******************************************************************************
#include "act_initialize.h"

#include "cmd_server.h"
#include "mll_localizer.h"
#include "mll_motor_controller.h"
#include "mll_operation_coordinator.h"
#include "mll_ui.h"
#include "mll_wall_analyser.h"
#include "mpl_battery.h"
#include "mpl_debug.h"
#include "mpl_encoder.h"
#include "mpl_imu.h"
#include "mpl_led.h"
#include "mpl_motor.h"
#include "mpl_speaker.h"
#include "mpl_timer.h"
#include "mpl_wallsensor.h"
#include "msg_format_imu.h"
#include "msg_format_localizer.h"
#include "msg_server.h"
#include "params.h"

using namespace act;

void InitializeActivity::init(ActivityParameters &params) {}

Status InitializeActivity::run() {
    auto cmd_server = cmd::CommandServer::getInstance();
    cmd::CommandFormatDebugTx cmd_debug_tx = {0};
    cmd::CommandFormatUiOut cmd_ui_out = {0};

    auto debug = mpl::Debug::getInstance();
    debug->init(hal::InitializeType::Dma);
    cmd_debug_tx.len = debug->format(cmd_debug_tx.message, "Hello Zirconia2kai!\n");
    cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);

    auto led = mpl::Led::getInstance();
    auto mplstatus = led->initPort(hal::InitializeType::Sync);
    if (mplstatus == mpl::MplStatus::SUCCESS) {
        cmd_debug_tx.len = debug->format(cmd_debug_tx.message, "LED: Initialization SUCCESS\n");
        cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
    } else {
        while (true);
    }

    auto imu = mpl::Imu::getInstance();
    imu->setConfig();
    auto result = imu->whoami();
    if (result == mpl::MplStatus::SUCCESS) {
        cmd_debug_tx.len = debug->format(cmd_debug_tx.message, "IMU WhoAmI: SUCCESS\n");
        cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
    } else {
        cmd_debug_tx.len = debug->format(cmd_debug_tx.message, "IMU WhoAmI: ERROR\n");
        cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
    }

    auto battery = mpl::Battery::getInstance();
    cmd_debug_tx.len = debug->format(cmd_debug_tx.message, "Battery: ");
    cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
    if (battery->initPort() != mpl::MplStatus::SUCCESS) {
        cmd_debug_tx.len = debug->format(cmd_debug_tx.message, "Initialize ERROR\n");
        cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
    }
    float battery_data = 0.0f;
    battery->scanSync(battery_data);
    cmd_debug_tx.len = debug->format(cmd_debug_tx.message, "%1.2f\n", battery_data);
    cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);

    auto encoder = mpl::Encoder::getInstance();
    encoder->initPort();

    auto wallsensor = mpl::WallSensor::getInstance();
    wallsensor->initPort();

    auto wallanalyser = mll::WallAnalyser::getInstance();
    wallanalyser->init();

    auto motor = mpl::Motor::getInstance();
    motor->initPort();

    auto motor_controller = mll::MotorController::getInstance();
    motor_controller->init();

    auto localizer = mll::Localizer::getInstance();
    localizer->init();

    [[maybe_unused]]
    auto operation_coordinator = mll::OperationCoordinator::getInstance();

    [[maybe_unused]]
    auto params_cache = misc::Params::getInstance()->getCachePointer();
    misc::Params::getInstance()->load(misc::ParameterDestinationType::HARDCODED);
    misc::Params::getInstance()->loadSlalom(misc::ParameterDestinationType::HARDCODED);

    [[maybe_unused]]
    auto message = msg::MessageServer::getInstance();

    // タイマー割り込みの開始
    mpl::Timer::init();

    auto speaker = mpl::Speaker::getInstance();
    speaker->initPort();
    // speaker->playToneSync(mpl::MusicTone::A5, 100);
    // speaker->playToneAsync(mpl::MusicTone::D6, 200);
    cmd_ui_out.type = mll::UiOutputEffect::POWERON;
    cmd_server->push(cmd::CommandId::UI_OUT, &cmd_ui_out);

    if (battery_data > 8.2f) {
        cmd_ui_out.type = mll::UiOutputEffect::BATTERY_FULL;
    } else if (battery_data > 7.8f) {
        cmd_ui_out.type = mll::UiOutputEffect::BATTERY_HIGH;
    } else if (battery_data > 7.4f) {
        cmd_ui_out.type = mll::UiOutputEffect::BATTERY_MIDDLE;
    } else if (battery_data > 7.0f) {
        cmd_ui_out.type = mll::UiOutputEffect::BATTERY_LOW;
    } else if (battery_data > 6.6f) {
        while (true);
    }
    cmd_server->push(cmd::CommandId::UI_OUT, &cmd_ui_out);

    auto ui = mll::Ui::getInstance();
    ui->init();

    mpl::Timer::sleepMs(400);

    return Status::SUCCESS;
}

void InitializeActivity::finalize(ActivityParameters &params) {
    params.initialized = true;

    switch (params.transition_mode) {
        case ActivityTransitionMode::FULLAUTO:
            params.next_activity = Activities::SEARCH;
            break;
        case ActivityTransitionMode::SEMIAUTO:
            params.next_activity = Activities::SEARCH;
            break;
        case ActivityTransitionMode::MANUAL:
            params.next_activity = Activities::SELECT_NEXT;
            break;
        case ActivityTransitionMode::DEBUG:
            params.next_activity = Activities::DEBUG;
            break;
        default:
            break;
    }
}
