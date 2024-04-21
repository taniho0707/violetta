//******************************************************************************
// @addtogroup ACT
// @file       act_debug.cpp
// @brief      Debug Activity
//******************************************************************************
#include "act_debug.h"

#include "mpl_speaker.h"
#include "msg_format_imu.h"
#include "msg_server.h"

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

Status DebugActivity::run() {
    auto debug = mpl::Debug::getInstance();
    debug->printf("Hello Zirconia2kai!\n");

    auto led = mpl::Led::getInstance();
    auto mplstatus = led->initPort(hal::LedNumbers::ALL);
    if (mplstatus == mpl::MplStatus::SUCCESS) {
        debug->printf("LED: Initialization SUCCESS\n");
        led->on(hal::LedNumbers::LEFT);
        led->on(hal::LedNumbers::RIGHT);
        led->on(hal::LedNumbers::FRONTL);
        led->on(hal::LedNumbers::FRONTR);
        led->on(hal::LedNumbers::MIDDLE1);
        led->on(hal::LedNumbers::MIDDLE2);
        led->on(hal::LedNumbers::MIDDLE3);
        led->on(hal::LedNumbers::MIDDLE4);
        led->on(hal::LedNumbers::MIDDLE5);
    } else {
        while (true);
    }

    auto imu = mpl::Imu::getInstance();
    imu->setConfig();
    auto result = imu->whoami();
    if (result == mpl::MplStatus::SUCCESS) {
        debug->printf("IMU WhoAmI: SUCCESS\n");
        // led->on(hal::LedNumbers::BLUE);
    } else {
        debug->printf("IMU WhoAmI: ERROR\n");
    }

    // Battery Test code
    auto battery = mpl::Battery::getInstance();
    debug->printf("Battery");
    if (battery->initPort() != mpl::MplStatus::SUCCESS) {
        debug->printf(" (Initialize ERROR)");
    }
    float battery_data = 0.0f;
    battery->scanSync(battery_data);
    debug->printf(": %1.2f\n", battery_data);

    // auto encoder = mpl::Encoder::getInstance();
    // hal::EncoderData encoder_data = {0};

    // // Wallsensor Test code
    // auto wallsensor = mpl::WallSensor::getInstance();
    // hal::WallSensorData wallsensor_data = {0};

    // // Motor Test code
    // auto motor = mpl::Motor::getInstance();
    // debug->printf("Motor: 25%% ON...");
    // motor->setDuty(+0.05, -0.05);
    // LL_mDelay(1000);
    // debug->printf("OFF\n");
    // motor->setFloat();

    mpl::Timer::init();

    // led->on(hal::LedNumbers::GREEN);
    auto speaker = mpl::Speaker::getInstance();
    speaker->initPort();
    speaker->playToneSync(mpl::MusicTone::A5, 200);
    speaker->playToneSync(mpl::MusicTone::D6, 400);

    auto message = msg::MessageServer::getInstance();
    msg::MsgFormatBattery msg_battery = msg::MsgFormatBattery();
    msg::MsgFormatEncoder msg_encoder = msg::MsgFormatEncoder();
    msg::MsgFormatImu msg_imu = msg::MsgFormatImu();
    msg::MsgFormatWallsensor msg_wallsensor = msg::MsgFormatWallsensor();

    while (1) {
        mpl::Timer::sleepMs(500);
        message->receiveMessage(msg::ModuleId::BATTERY, &msg_battery);
        message->receiveMessage(msg::ModuleId::ENCODER, &msg_encoder);
        message->receiveMessage(msg::ModuleId::IMU, &msg_imu);
        message->receiveMessage(msg::ModuleId::WALLSENSOR, &msg_wallsensor);
        debug->printf(
            "T: %10d B: %1.2f | L: %5d, R: %5d | FL: %4d, L: %4d, R: %4d, FR: "
            "%4d, IMU: %8d, %10d, % 6d, % 6d, % 6d, % 6d, % 6d, "
            "% 6d, %d\n",
            mpl::Timer::getMicroTime(), msg_battery.battery, msg_encoder.left,
            msg_encoder.right, msg_wallsensor.frontleft, msg_wallsensor.left,
            msg_wallsensor.right, msg_wallsensor.frontright, msg_imu.getCount(),
            msg_imu.getTime(), msg_imu.gyro_yaw, msg_imu.gyro_roll,
            msg_imu.gyro_pitch, msg_imu.acc_x, msg_imu.acc_y, msg_imu.acc_z,
            msg_imu.temperature);
    }

    return Status::ERROR;
}

void DebugActivity::finalize() {}
