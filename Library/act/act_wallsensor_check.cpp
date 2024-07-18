//******************************************************************************
// @addtogroup ACT
// @file       act_wallsensor_check.cpp
// @brief      Wallsensor Check Activity
//******************************************************************************
#include "act_wallsensor_check.h"

#include "cmd_server.h"
#include "mll_localizer.h"
#include "mll_motor_controller.h"
#include "mll_wall_analyser.h"
#include "mpl_battery.h"
#include "mpl_debug.h"
#include "mpl_encoder.h"
#include "mpl_imu.h"
#include "mpl_led.h"
#include "mpl_motor.h"
#include "mpl_timer.h"
#include "mpl_wallsensor.h"
#include "msg_format_wall_analyser.h"
#include "msg_server.h"
#include "params.h"

using namespace act;

void WallsensorCheckActivity::init() {}

#ifdef MOUSE_LAZULI
Status DebugActivity::run() {}
#endif  // MOUSE_LAZULI

#ifdef MOUSE_ZIRCONIA2KAI
Status WallsensorCheckActivity::run() {
    auto message = msg::MessageServer::getInstance();
    msg::MsgFormatBattery msg_battery = msg::MsgFormatBattery();
    msg::MsgFormatWallsensor msg_wallsensor = msg::MsgFormatWallsensor();
    msg::MsgFormatWallAnalyser msg_wallanalyser = msg::MsgFormatWallAnalyser();

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

    // Battery Test code
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

    // Motor Test code
    auto motor = mpl::Motor::getInstance();
    auto motor_controller = mll::MotorController::getInstance();
    motor_controller->init();

    auto localizer = mll::Localizer::getInstance();
    localizer->init();

    auto wallanalyser = mll::WallAnalyser::getInstance();
    wallanalyser->init();

    static auto params_cache = misc::Params::getInstance()->getCachePointer();
    misc::Params::getInstance()->load(misc::ParameterDestinationType::HARDCODED);
    misc::Params::getInstance()->loadSlalom(misc::ParameterDestinationType::HARDCODED);

    mpl::Timer::init();

    // 【壁センサの値を取るための無限ループ】
    while (1) {
        auto start_time = mpl::Timer::getMilliTime();
        bool kabekire_left = false;
        bool kabekire_right = false;
        while (mpl::Timer::getMilliTime() <= start_time + 100) {
            message->receiveMessage(msg::ModuleId::BATTERY, &msg_battery);
            message->receiveMessage(msg::ModuleId::WALLSENSOR, &msg_wallsensor);
            message->receiveMessage(msg::ModuleId::WALLANALYSER, &msg_wallanalyser);
            if (msg_wallanalyser.kabekire_left) {
                kabekire_left = true;
            }
            if (msg_wallanalyser.kabekire_right) {
                kabekire_right = true;
            }
        }

        cmd_debug_tx.len =
            debug->format(cmd_debug_tx.message, "T: %10d B: %1.2f | FL: %4d, L: %4d, R: %4d, FR: %4d | kire(%d|%d) | F: % 4.1f, LR: % 4.1f\n",
                          mpl::Timer::getMilliTime(), msg_battery.battery, msg_wallsensor.frontleft, msg_wallsensor.left, msg_wallsensor.right,
                          msg_wallsensor.frontright, kabekire_left ? 1 : 0, kabekire_right ? 1 : 0, msg_wallanalyser.distance_from_front,
                          msg_wallanalyser.distance_from_center);
        cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
    }

    return Status::ERROR;
}
#endif  // MOUSE_ZIRCONIA2KAI

void WallsensorCheckActivity::finalize() {}
