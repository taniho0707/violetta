//******************************************************************************
// @addtogroup ACT
// @file       act_debug.cpp
// @brief      Debug Activity
//******************************************************************************
#include "act_debug.h"

#include "cmd_server.h"
#include "mll_localizer.h"
#include "mll_logger.h"
#include "mll_motor_controller.h"
#include "mll_operation_coordinator.h"
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

void DebugActivity::init() {}

// Status DebugActivity::run() {
//     LL_mDelay(1000);

//     auto debug = mpl::Debug::getInstance();
//     LL_mDelay(1000);
//     debug->printf("H");  // なぜかはじめの1文字目が送信できない…
//     LL_mDelay(1000);
//     debug->printf("Hello Zirconia2kai!\n");

//     hal::initImuPort();
//     LL_mDelay(1000);
//     auto whoami = hal::whoamiImu();
//     if (whoami == hal::HalStatus::SUCCESS) {
//         debug->printf("IMU WhoAmI: SUCCESS\n");
//     } else {
//         debug->printf("IMU WhoAmI: ERROR\n");
//     }

//     while (true) {
//     }
// }

#ifdef MOUSE_LAZULI
Status DebugActivity::run() {
    auto cmd_server = cmd::CommandServer::getInstance();
    cmd::CommandFormatDebugTx cmd_debug_tx = {0};

    auto debug = mpl::Debug::getInstance();
    debug->init(hal::InitializeType::Dma);
    cmd_debug_tx.len = debug->format(cmd_debug_tx.message, "Hello Lazuli!\n");
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
    if (battery_data > 8.2f) {
        led->on(hal::LedNumbers::MIDDLE1);
        led->on(hal::LedNumbers::MIDDLE2);
        led->on(hal::LedNumbers::MIDDLE3);
        led->on(hal::LedNumbers::MIDDLE4);
    } else if (battery_data > 7.8f) {
        led->on(hal::LedNumbers::MIDDLE2);
        led->on(hal::LedNumbers::MIDDLE3);
        led->on(hal::LedNumbers::MIDDLE4);
    } else if (battery_data > 7.4f) {
        led->on(hal::LedNumbers::MIDDLE3);
        led->on(hal::LedNumbers::MIDDLE4);
    } else if (battery_data > 7.0f) {
        led->on(hal::LedNumbers::MIDDLE4);
    } else if (battery_data > 6.6f) {
        while (true);
    }

    auto encoder = mpl::Encoder::getInstance();
    encoder->initPort();

    auto wallsensor = mpl::WallSensor::getInstance();
    wallsensor->initPort();
    // hal::WallSensorData wallsensor_data = {0};

    auto wallanalyser = mll::WallAnalyser::getInstance();
    wallanalyser->init();

    [[maybe_unused]]
    auto motor = mpl::Motor::getInstance();
    // debug->printf("Motor: 25%% ON...");
    // motor->setDuty(+0.05, -0.05);
    // LL_mDelay(1000);
    // debug->printf("OFF\n");
    // motor->setFloat();

    auto motor_controller = mll::MotorController::getInstance();
    motor_controller->init();

    auto localizer = mll::Localizer::getInstance();
    localizer->init();

    [[maybe_unused]]
    auto operation_coordinator = mll::OperationCoordinator::getInstance();

    [[maybe_unused]]
    static auto params_cache = misc::Params::getInstance()->getCachePointer();
    misc::Params::getInstance()->load(misc::ParameterDestinationType::HARDCODED);

    mpl::Timer::init();

    auto speaker = mpl::Speaker::getInstance();
    speaker->initPort();
    speaker->playToneSync(mpl::MusicTone::A5, 200);
    speaker->playToneAsync(mpl::MusicTone::D6, 400);

    auto message = msg::MessageServer::getInstance();
    msg::MsgFormatBattery msg_battery = msg::MsgFormatBattery();
    msg::MsgFormatEncoder msg_encoder = msg::MsgFormatEncoder();
    msg::MsgFormatImu msg_imu = msg::MsgFormatImu();
    msg::MsgFormatWallsensor msg_wallsensor = msg::MsgFormatWallsensor();

    mpl::TimerStatistics timer_statistics;

    while (1) {
        mpl::Timer::sleepMs(500);
        mpl::Timer::getStatistics(timer_statistics);
        message->receiveMessage(msg::ModuleId::BATTERY, &msg_battery);
        message->receiveMessage(msg::ModuleId::ENCODER, &msg_encoder);
        message->receiveMessage(msg::ModuleId::IMU, &msg_imu);
        message->receiveMessage(msg::ModuleId::WALLSENSOR, &msg_wallsensor);
        // cmd_debug_tx.len = debug->format(cmd_debug_tx.message,
        //                                  "T: %10d B: %1.2f | L: %5d, R: %5d | FL: %4d, L: %4d, R: %4d, FR: "
        //                                  "%4d, IMU: %8d, %10d, % 6d, % 6d, % 6d, % 6d, % 6d, % 6d, %d\n",
        //                                  mpl::Timer::getMicroTime(), msg_battery.battery, msg_encoder.left,
        //                                  msg_encoder.right, msg_wallsensor.frontleft, msg_wallsensor.left,
        //                                  msg_wallsensor.right, msg_wallsensor.frontright, msg_imu.getCount(),
        //                                  msg_imu.getTime(), msg_imu.gyro_yaw, msg_imu.gyro_roll,
        //                                  msg_imu.gyro_pitch, msg_imu.acc_x, msg_imu.acc_y, msg_imu.acc_z,
        //                                  msg_imu.temperature);
        cmd_debug_tx.len = debug->format(
            cmd_debug_tx.message,
            "T: %10d B: %1.2f | IMU: %8d, %10d, % 6.2f, % 5.1f, % 5.1f, % 5.1f | WALL: % 4d,% 4d,% 4d,% 4d | STAT: Max.[%2.0f|%2.0f|%2.0f|%2.0f] "
            "Avg.[%2.0f|%2.0f|%2.0f|%2.0f]\n",
            mpl::Timer::getMicroTime(), msg_battery.battery, msg_imu.getCount(), msg_imu.getTime(), msg_imu.gyro_yaw, msg_imu.acc_x / 1000,
            msg_imu.acc_y / 1000, msg_imu.acc_z / 1000, msg_wallsensor.frontleft, msg_wallsensor.left, msg_wallsensor.right,
            msg_wallsensor.frontright, float(timer_statistics.count1_max) * 100 / hal::TIMER_COUNT_MAX,
            float(timer_statistics.count2_max) * 100 / hal::TIMER_COUNT_MAX, float(timer_statistics.count3_max) * 100 / hal::TIMER_COUNT_MAX,
            float(timer_statistics.count4_max) * 100 / hal::TIMER_COUNT_MAX, float(timer_statistics.count1_avg) * 100 / hal::TIMER_COUNT_MAX,
            float(timer_statistics.count2_avg) * 100 / hal::TIMER_COUNT_MAX, float(timer_statistics.count3_avg) * 100 / hal::TIMER_COUNT_MAX,
            float(timer_statistics.count4_avg) * 100 / hal::TIMER_COUNT_MAX);
        cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
    }

    return Status::ERROR;
}
#endif  // MOUSE_LAZULI

#ifdef MOUSE_LAZULI_SENSOR
Status DebugActivity::run() {
    return Status::ERROR;
}
#endif  // MOUSE_LAZULI_SENSOR

#ifdef MOUSE_ZIRCONIA2KAI
Status DebugActivity::run() {
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

    auto wallanalyser = mll::WallAnalyser::getInstance();
    wallanalyser->init();

    // Motor Test code
    auto motor = mpl::Motor::getInstance();
    // debug->printf("Motor: 25%% ON...");
    // motor->setDuty(+0.05, -0.05);
    // LL_mDelay(1000);
    // debug->printf("OFF\n");
    // motor->setFloat();

    auto motor_controller = mll::MotorController::getInstance();
    motor_controller->init();

    auto localizer = mll::Localizer::getInstance();
    localizer->init();

    auto operation_coordinator = mll::OperationCoordinator::getInstance();

    [[maybe_unused]]
    static auto params_cache = misc::Params::getInstance()->getCachePointer();
    misc::Params::getInstance()->load(misc::ParameterDestinationType::HARDCODED);

    auto message = msg::MessageServer::getInstance();
    msg::MsgFormatBattery msg_battery = msg::MsgFormatBattery();
    msg::MsgFormatEncoder msg_encoder = msg::MsgFormatEncoder();
    msg::MsgFormatImu msg_imu = msg::MsgFormatImu();
    msg::MsgFormatWallsensor msg_wallsensor = msg::MsgFormatWallsensor();
    msg::MsgFormatLocalizer msg_localizer = msg::MsgFormatLocalizer();

    mpl::Timer::init();
    mpl::TimerStatistics timer_statistics;

    // 【モーターのパラメータチューニング】
    // 48KB の RAM を確保、片輪 2Bytes の両輪 4Bytes、12288 カウントのデータを保存できる
    // 1ms 周期で 12.288 秒間保存する
    // uint16_t encoder_log_l[12288] = {0};
    // uint16_t encoder_log_r[12288] = {0};
    // ジャイロの各軸が安定し3秒経過するまで待機

    // // LED を5回点滅させる
    // for (int i = 0; i < 5; i++) {
    //     led->on(hal::LedNumbers::BLUE);
    //     mpl::Timer::sleepMs(500);
    //     led->off(hal::LedNumbers::BLUE);
    //     mpl::Timer::sleepMs(500);
    // }
    // // 1V のステップ入力を並進方向に与えながらログを取得
    // // auto start_time = mpl::Timer::getMicroTime();
    // // int encoder_log_index = 0;
    // // while (mpl::Timer::getMicroTime() - start_time < 1000000) {
    // //     message->receiveMessage(msg::ModuleId::BATTERY, &msg_battery);
    // //     float motor_duty = 1.0f / msg_battery.battery;
    // //     motor->setDuty(motor_duty, motor_duty);
    // //     // encoder->getCountSync(msg_encoder.left, msg_encoder.right);
    // //     encoder_log_l[encoder_log_index] = msg_encoder.left;
    // //     encoder_log_r[encoder_log_index] = msg_encoder.right;
    // //     encoder_log_index++;
    // //     if (encoder_log_index >= 12288) {
    // //         break;
    // //     }
    // // }

    // const uint32_t ENCODER_LOG_LENGTH = 3000;
    // mll::LogFormatEncoder log_test[ENCODER_LOG_LENGTH] = {{0}};
    // auto logger = mll::Logger::getInstance();
    // auto logconfig = mll::LogConfig{mll::LogType::ENCODER, mll::LogDestinationType::INTERNAL_RAM, ENCODER_LOG_LENGTH, (uint32_t)(&log_test)};
    // logger->init(logconfig);
    // // mll::LogFormatEncoder log_item = {0, 1.0f, 1.0f};
    // // for (int i = 0; i < 100; ++i) {
    // //     log_item.time = mpl::Timer::getMicroTime();
    // //     log_item.left = 1.0f * i;
    // //     logger->save(mll::LogType::ENCODER, (void *)(&log_item));
    // //     mpl::Timer::sleepMs(1);
    // // }
    // // log_item = {0, 0.0f, 0.0f};
    // // for (int i = 0; i < 100; ++i) {
    // //     logger->read(logconfig, i, (void *)(&log_item));
    // //     cmd_debug_tx.len = debug->format(cmd_debug_tx.message, "Log[%d]: %d, %1.2f, %1.2f\n", i, log_item.time, log_item.left, log_item.right);
    // //     // cmd_debug_tx.len = debug->format(cmd_debug_tx.message, "Log[%d]: %d, %1.2f, %1.2f\n", i, log_test[i].time, log_test[i].left,
    // log_test[i].right);
    // //     cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
    // //     mpl::Timer::sleepMs(10);
    // // }
    // logger->startPeriodic(mll::LogType::ENCODER, ENCODER_LOG_LENGTH);
    // motor->setDuty(0.25f, 0.25f);
    // mpl::Timer::sleepMs(ENCODER_LOG_LENGTH);
    // motor->setDuty(0.0f, 0.0f);
    // logger->stopPeriodic(mll::LogType::ENCODER);
    // for (int i = 0; i < 5; i++) {
    //     led->on(hal::LedNumbers::BLUE);
    //     mpl::Timer::sleepMs(500);
    //     led->off(hal::LedNumbers::BLUE);
    //     mpl::Timer::sleepMs(500);
    // }
    // for (int i = 0; i < 10; i++) {
    //     led->on(hal::LedNumbers::BLUE);
    //     mpl::Timer::sleepMs(250);
    //     led->off(hal::LedNumbers::BLUE);
    //     mpl::Timer::sleepMs(250);
    // }
    // mll::LogFormatEncoder log_item = {0, 1.0f, 1.0f};
    // for (int i = 0; i < ENCODER_LOG_LENGTH; ++i) {
    //     logger->read(logconfig, i, (void *)(&log_item));
    //     cmd_debug_tx.len = debug->format(cmd_debug_tx.message, "Log[%d]: %d, %1.2f, %1.2f\n", i, log_item.time, log_item.left, log_item.right);
    //     cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
    //     mpl::Timer::sleepMs(2);
    // }

    // メモリがいっぱいになったらモーターを停止させ LED を点滅させる

    // HOGE になったら UART にログ出力する

    // OperationCoordinator のテスト
    mpl::Timer::sleepMs(3000);
    operation_coordinator->resetPosition(mll::MousePhysicalPosition{45.0f, 45.0f, 0.0f});
    operation_coordinator->enableMotorControl();
    // while (true);
    mpl::Timer::sleepMs(2000);

    // 【壁センサの値をロギング】
    // 48KB の RAM を確保、センサ値 2Bytes、時間 4Bytes、合計 12Bytes なので、4096 カウントのデータを保存できる
    // 1ms 周期で 4.096 秒間保存する
    // uint16_t wallsensor_frontleft[4096] = {0};
    // uint16_t wallsensor_left[4096] = {0};
    // uint16_t wallsensor_right[4096] = {0};
    // uint16_t wallsensor_frontright[4096] = {0};

    // auto start_time = mpl::Timer::getMicroTime();
    // int encoder_log_index = 0;
    // while (mpl::Timer::getMicroTime() - start_time < 1000000) {
    //     message->receiveMessage(msg::ModuleId::BATTERY, &msg_battery);
    //     float motor_duty = 1.0f / msg_battery.battery;
    //     motor->setDuty(motor_duty, motor_duty);
    //     // encoder->getCountSync(msg_encoder.left, msg_encoder.right);
    //     encoder_log_l[encoder_log_index] = msg_encoder.left;
    //     encoder_log_r[encoder_log_index] = msg_encoder.right;
    //     encoder_log_index++;
    //     if (encoder_log_index >= 12288) {
    //         break;
    //     }
    // }

    // const uint32_t WALLSENSOR_LOG_LENGTH = 4096;
    // mll::LogFormatWallsensor log_test[WALLSENSOR_LOG_LENGTH] = {{0}};
    // auto logger = mll::Logger::getInstance();
    // auto logconfig = mll::LogConfig{mll::LogType::WALLSENSOR, mll::LogDestinationType::INTERNAL_RAM, WALLSENSOR_LOG_LENGTH,
    // (uintptr_t)(&log_test)}; logger->init(logconfig);

    mll::OperationMoveCombination move_array[] = {
        // {     mll::OperationMoveType::TRAPACCEL, 90.f},
        // {mll::OperationMoveType::TRAPACCEL_STOP, 90.f},
        // {mll::OperationMoveType::PIVOTTURN, misc::PI},
        {     mll::OperationMoveType::TRAPACCEL, 45.f},
        // {mll::OperationMoveType::TRAPACCEL_STOP,     45.f},
        // {     mll::OperationMoveType::PIVOTTURN, misc::PI},
        // {     mll::OperationMoveType::TRAPACCEL, 90.f * 3},
        {mll::OperationMoveType::TRAPACCEL_STOP, 45.f},
    };
    uint16_t move_array_length = sizeof(move_array) / sizeof(mll::OperationMoveCombination);
    operation_coordinator->runSpecific(move_array, move_array_length);
    while (operation_coordinator->state() == mll::OperationCoordinatorResult::RUNNING_SPECIFIC) {
        mpl::Timer::sleepMs(100);
    }

    led->off(hal::LedNumbers::BLUE);
    led->off(hal::LedNumbers::GREEN);
    led->off(hal::LedNumbers::RED);
    led->off(hal::LedNumbers::YELLOW);

    // logger->startPeriodic(mll::LogType::WALLSENSOR, 1);
    // mpl::Timer::sleepMs(WALLSENSOR_LOG_LENGTH + 10000);
    //
    // for (int i = 0; i < 30; i++) {
    //     led->on(hal::LedNumbers::BLUE);
    //     mpl::Timer::sleepMs(500);
    //     led->off(hal::LedNumbers::BLUE);
    //     mpl::Timer::sleepMs(500);
    // }
    // for (int i = 0; i < 10; i++) {
    //     led->on(hal::LedNumbers::BLUE);
    //     mpl::Timer::sleepMs(250);
    //     led->off(hal::LedNumbers::BLUE);
    //     mpl::Timer::sleepMs(250);
    // }
    // mll::LogFormatWallsensor log_item = {0};
    // for (int i = 0; i < WALLSENSOR_LOG_LENGTH; ++i) {
    //     logger->read(logconfig, i, (void *)(&log_item));
    //     cmd_debug_tx.len = debug->format(cmd_debug_tx.message, "Log[%d]: %d, %d, %d, %d, %d\n", i, log_item.time, log_item.frontleft,
    //     log_item.left,
    //                                      log_item.right, log_item.frontright);
    //     cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
    //     mpl::Timer::sleepMs(2);
    // }

    while (true);

    // 宴会芸
    motor_controller->setStay();
    while (true) {
        mpl::Timer::sleepMs(100);
        mpl::Timer::getStatistics(timer_statistics);
        message->receiveMessage(msg::ModuleId::BATTERY, &msg_battery);
        message->receiveMessage(msg::ModuleId::ENCODER, &msg_encoder);
        message->receiveMessage(msg::ModuleId::IMU, &msg_imu);
        message->receiveMessage(msg::ModuleId::LOCALIZER, &msg_localizer);
        cmd_debug_tx.len =
            debug->format(cmd_debug_tx.message, "%10d, trans % 8.2f, % 7.2f, % 5.2f, rot % 5.2f, % 5.2f, the % 5.2f x % 5.2f y % 5.2f\n",
                          mpl::Timer::getMicroTime(), msg_localizer.accel_translation, msg_localizer.velocity_translation,
                          msg_localizer.position_translation, msg_localizer.accel_rotation, msg_localizer.velocity_rotation,
                          msg_localizer.position_theta, msg_localizer.position_x, msg_localizer.position_y);
        cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
    }

    // 【各センサの値を取るための無限ループ】
    while (1) {
        mpl::Timer::sleepMs(100);
        mpl::Timer::getStatistics(timer_statistics);
        message->receiveMessage(msg::ModuleId::BATTERY, &msg_battery);
        message->receiveMessage(msg::ModuleId::ENCODER, &msg_encoder);
        message->receiveMessage(msg::ModuleId::IMU, &msg_imu);
        message->receiveMessage(msg::ModuleId::WALLSENSOR, &msg_wallsensor);
        message->receiveMessage(msg::ModuleId::LOCALIZER, &msg_localizer);
        // cmd_debug_tx.len = debug->format(cmd_debug_tx.message,
        //                                  "T: %10d B: %1.2f | L: % 5.2f, R: % 5.2f | FL: %4d, L: %4d, R: %4d, FR: "
        //                                  "%4d, IMU: % 5.2f, % 5.2f, % 5.2f, % 5.2f, % 5.2f, % 5.2f\n",
        //                                  mpl::Timer::getMicroTime(), msg_battery.battery, msg_encoder.left,
        //                                  msg_encoder.right, msg_wallsensor.frontleft, msg_wallsensor.left,
        //                                  msg_wallsensor.right, msg_wallsensor.frontright, msg_imu.gyro_yaw, msg_imu.gyro_roll,
        //                                  msg_imu.gyro_pitch, msg_imu.acc_x, msg_imu.acc_y, msg_imu.acc_z);
        // cmd_debug_tx.len = debug->format(cmd_debug_tx.message,
        //                                  "T: %10d B: %1.2f | IMU: %f, %f, %f, %f | WALL: % 4d,% 4d,% 4d,% 4d | STAT: Max.[%2.0f|%2.0f|%2.0f|%2.0f]
        //                                  Avg.[%2.0f|%2.0f|%2.0f|%2.0f]\n", mpl::Timer::getMicroTime(), msg_battery.battery, msg_imu.gyro_yaw,
        //                                  msg_imu.acc_x, msg_imu.acc_y, msg_imu.acc_z, msg_wallsensor.frontleft, msg_wallsensor.left,
        //                                  msg_wallsensor.right, msg_wallsensor.frontright, float(timer_statistics.count1_max) * 100 /
        //                                  hal::TIMER_COUNT_MAX, float(timer_statistics.count2_max) * 100 / hal::TIMER_COUNT_MAX,
        //                                  float(timer_statistics.count3_max) * 100 / hal::TIMER_COUNT_MAX, float(timer_statistics.count4_max) * 100
        //                                  / hal::TIMER_COUNT_MAX, float(timer_statistics.count1_avg) * 100 / hal::TIMER_COUNT_MAX,
        //                                  float(timer_statistics.count2_avg) * 100 / hal::TIMER_COUNT_MAX, float(timer_statistics.count3_avg) * 100
        //                                  / hal::TIMER_COUNT_MAX, float(timer_statistics.count4_avg) * 100 / hal::TIMER_COUNT_MAX);
        // cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
        // cmd::CommandFormatDebugTx cmd_debug_tx = {};
        cmd_debug_tx.len =
            debug->format(cmd_debug_tx.message, "%10d, trans % 8.2f, % 7.2f, % 5.2f, rot % 5.2f, % 5.2f, the % 5.2f x % 5.2f y % 5.2f\n",
                          mpl::Timer::getMicroTime(), msg_localizer.accel_translation, msg_localizer.velocity_translation,
                          msg_localizer.position_translation, msg_localizer.accel_rotation, msg_localizer.velocity_rotation,
                          msg_localizer.position_theta, msg_localizer.position_x, msg_localizer.position_y);
        cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
    }

    return Status::ERROR;
}
#endif  // MOUSE_ZIRCONIA2KAI

void DebugActivity::finalize() {}
