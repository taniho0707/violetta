//******************************************************************************
// @addtogroup ACT
// @file       act_search.cpp
// @brief      Search Activity
//******************************************************************************
#include "act_search.h"

#include "cmd_server.h"
#include "mll_localizer.h"
#include "mll_motor_controller.h"
#include "mll_operation_controller.h"
#include "mll_wall_analyser.h"
#include "mpl_battery.h"
#include "mpl_debug.h"
#include "mpl_encoder.h"
#include "mpl_imu.h"
#include "mpl_led.h"
#include "mpl_motor.h"
#include "mpl_timer.h"
#include "mpl_wallsensor.h"
#include "msg_format_imu.h"
#include "msg_format_localizer.h"
#include "msg_server.h"
#include "params.h"

using namespace act;

void SearchActivity::init() {}

Status SearchActivity::run() {
    auto cmd_server = cmd::CommandServer::getInstance();
    cmd::CommandFormatDebugTx cmd_debug_tx = {0};

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
    if (battery_data > 4.f) {
        led->on(hal::LedNumbers::BLUE);
        led->on(hal::LedNumbers::GREEN);
        led->on(hal::LedNumbers::YELLOW);
        led->on(hal::LedNumbers::RED);
    } else if (battery_data > 3.8f) {
        led->on(hal::LedNumbers::GREEN);
        led->on(hal::LedNumbers::YELLOW);
        led->on(hal::LedNumbers::RED);
    } else if (battery_data > 3.6f) {
        led->on(hal::LedNumbers::YELLOW);
        led->on(hal::LedNumbers::RED);
    } else if (battery_data > 3.4f) {
        led->on(hal::LedNumbers::RED);
    } else {
        while (true);
    }

    auto encoder = mpl::Encoder::getInstance();
    encoder->initPort();

    auto wallsensor = mpl::WallSensor::getInstance();
    wallsensor->initPort();

    auto wallanalyser = mll::WallAnalyser::getInstance();
    wallanalyser->init();

    auto motor = mpl::Motor::getInstance();

    auto motor_controller = mll::MotorController::getInstance();
    motor_controller->init();

    auto localizer = mll::Localizer::getInstance();
    localizer->init();

    auto operation_controller = mll::OperationController::getInstance();
    operation_controller->init();

    static auto params_cache = misc::Params::getInstance()->getCachePointer();
    misc::Params::getInstance()->load(misc::ParameterDestinationType::HARDCODED);
    misc::Params::getInstance()->loadSlalom(misc::ParameterDestinationType::HARDCODED);

    auto message = msg::MessageServer::getInstance();
    msg::MsgFormatBattery msg_battery = msg::MsgFormatBattery();
    msg::MsgFormatEncoder msg_encoder = msg::MsgFormatEncoder();
    msg::MsgFormatImu msg_imu = msg::MsgFormatImu();
    msg::MsgFormatWallsensor msg_wallsensor = msg::MsgFormatWallsensor();
    msg::MsgFormatLocalizer msg_localizer = msg::MsgFormatLocalizer();

    // タイマー割り込みの開始
    mpl::Timer::init();
    mpl::TimerStatistics timer_statistics;

    // OperationController のテスト
    mpl::Timer::sleepMs(3000);
    motor_controller->setStay();
    mpl::Timer::sleepMs(2000);

    auto cmd_format_operation_direction = cmd::CommandFormatOperationDirection();
    cmd_format_operation_direction.type = cmd::OperationDirectionType::SEARCH;
    cmd_server->push(cmd::CommandId::OPERATION_DIRECTION, &cmd_format_operation_direction);
    while (true);
}

void SearchActivity::finalize() {}
