//******************************************************************************
// @addtogroup ACT
// @file       act_debug.cpp
// @brief      Debug Activity
//******************************************************************************
#include "act_debug.h"

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
    auto led = mpl::Led::getInstance();
    // led->initPort(hal::LedNumbers::ALL);
    // led->on(hal::LedNumbers::RED);
    // led->on(hal::LedNumbers::YELLOW);
    // led->on(hal::LedNumbers::GREEN);
    // led->on(hal::LedNumbers::BLUE);

    auto debug = mpl::Debug::getInstance();
    debug->printf("H");  // なぜかはじめの1文字目が送信できない…
    debug->printf("Hello Zirconia2kai!\n");

    auto imu = mpl::Imu::getInstance();
    imu->setConfig();
    auto result = imu->whoami();
    if (result == mpl::MplStatus::SUCCESS) {
        debug->printf("IMU WhoAmI: SUCCESS\n");
        led->on(hal::LedNumbers::BLUE);
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

    auto encoder = mpl::Encoder::getInstance();
    hal::EncoderData encoder_data = {0};

    // Wallsensor Test code
    auto wallsensor = mpl::WallSensor::getInstance();
    hal::WallSensorData wallsensor_data = {0};

    // // Motor Test code
    // auto motor = mpl::Motor::getInstance();
    // debug->printf("Motor: 25%% ON...");
    // motor->setDuty(+0.05, -0.05);
    // LL_mDelay(1000);
    // debug->printf("OFF\n");
    // motor->setFloat();

    mpl::Timer::init();

    led->on(hal::LedNumbers::GREEN);

    auto message = msg::MessageServer::getInstance();
    msg::MsgFormatImu msg_imu = msg::MsgFormatImu();

    while (1) {
        LL_mDelay(1000);
        battery->scanSync(battery_data);
        encoder->scanEncoderSync(encoder_data);
        wallsensor->scanAllSync(wallsensor_data);
        message->receiveMessage(msg::ModuleId::IMU, &msg_imu);
        debug->printf(
            "T: %10d B: %1.2f | L: %5d, R: %5d | FL: %4d, L: %4d, R: %4d, FR: "
            "%4d, IMU: %8d, %10d, % 6.2f, % 6.2f, % 6.2f, % 6.2f, % 6.2f, "
            "% 6.2f, %4.2f\n",
            mpl::Timer::getMicroTime(), battery_data, encoder_data.LEFT,
            encoder_data.RIGHT, wallsensor_data.FRONTLEFT, wallsensor_data.LEFT,
            wallsensor_data.RIGHT, wallsensor_data.FRONTRIGHT,
            msg_imu.getCount(), msg_imu.getTime(), msg_imu.gyro_yaw,
            msg_imu.gyro_roll, msg_imu.gyro_pitch, msg_imu.acc_x, msg_imu.acc_y,
            msg_imu.acc_z, msg_imu.temperature);
    }

    return Status::ERROR;
}

void DebugActivity::finalize() {}