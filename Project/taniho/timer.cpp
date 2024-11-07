//******************************************************************************
// @addtogroup TANIHO
// @file       timer.cpp
// @brief      Timer interrupt functions
//******************************************************************************
#include "mpl_timer.h"

// 追加で必要なライブラリ : MPL
#include "mpl_battery.h"
#include "mpl_debug.h"
#include "mpl_encoder.h"
#include "mpl_imu.h"
#include "mpl_led.h"
#include "mpl_motor.h"
#include "mpl_speaker.h"
#include "mpl_wallsensor.h"

// 追加で必要なライブラリ : MLL
#include "mll_localizer.h"
#include "mll_logger.h"
#include "mll_motor_controller.h"
#include "mll_operation_coordinator.h"
#include "mll_ui.h"
#include "mll_wall_analyser.h"

void mpl::Timer::run1() {
    static auto wallsensor = mpl::WallSensor::getInstance();
    wallsensor->interruptPeriodic();

    static auto wallanalyser = mll::WallAnalyser::getInstance();
    wallanalyser->interruptPeriodic();
}

void mpl::Timer::run2() {
    static auto imu = mpl::Imu::getInstance();
    imu->interruptPeriodic();

    static auto encoder = mpl::Encoder::getInstance();
    encoder->interruptPeriodic();

    static auto battery = mpl::Battery::getInstance();
    battery->interruptPeriodic();

    static auto ui = mll::Ui::getInstance();
    ui->interruptPeriodic();
}

void mpl::Timer::run3() {
    static auto led = mpl::Led::getInstance();
    led->interruptPeriodic();

#ifdef MOUSE_LAZULI
    static auto speaker = mpl::Speaker::getInstance();
    speaker->interruptPeriodic();
#endif  // MOUSE_LAZULI

    static auto localizer = mll::Localizer::getInstance();
    localizer->interruptPeriodic();

    static auto operationcoordinator = mll::OperationCoordinator::getInstance();
    operationcoordinator->interruptPeriodic();

    static auto motorcontroller = mll::MotorController::getInstance();
    motorcontroller->interruptPeriodic();

    static auto motor = mpl::Motor::getInstance();
    motor->interruptPeriodic();
}

void mpl::Timer::run4() {
    static auto logger = mll::Logger::getInstance();
    logger->interruptPeriodic();

    static auto debug = mpl::Debug::getInstance();
    debug->interruptPeriodic();
}
