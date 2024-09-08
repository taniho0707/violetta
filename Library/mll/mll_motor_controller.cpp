//******************************************************************************
// @addtogroup MLL
// @file       mll_motor_controller.h
// @brief      モーターの回転数制御を行うクラス
//******************************************************************************
#include "mll_motor_controller.h"

#if defined(STM32)
#ifndef STM32C011xx
#include "arm_math.h"
#else
#include "math.h"
#endif
#endif

#ifdef LINUX
#include "arm_math_linux.h"
using namespace plt;
#endif

#include "mpl_debug.h"
#include "mpl_timer.h"
#include "msg_format_battery.h"
#include "msg_format_motor.h"
#include "msg_server.h"

mll::MotorController::MotorController() {
    enabled = false;
}

void mll::MotorController::init() {
    params = misc::Params::getInstance()->getCachePointer();

    msg_server = msg::MessageServer::getInstance();
    cmd_server = cmd::CommandServer::getInstance();
}

float mll::MotorController::calcDistanceToTarget(float current_x, float current_y, float target_x, float target_y) {
    float dx = target_x - current_x;
    float dy = target_y - current_y;
    float distance;
#if defined(STM32)
#ifndef STM32C011xx
    arm_sqrt_f32(dx * dx + dy * dy, &distance);
#else
    distance = sqrtf(dx * dx + dy * dy);
#endif
#elif defined(LINUX)
    distance = sqrt(dx * dx + dy * dy);
#endif
    return distance;
}

float mll::MotorController::calcAngleToTarget(float current_angle, float target_angle) {
    // return target_angle - current_angle;
    return current_angle - target_angle;
}

void mll::MotorController::startControl() {
    enabled = true;
    resetIntegral();
}

void mll::MotorController::stopControl() {
    enabled = false;

    // TODO: 正しいところに移す
    msg::MsgFormatMotor msg_motor;
    msg_motor.duty_l = 0;
    msg_motor.duty_r = 0;
    msg_motor.duty_suction = 0;
    msg_server->sendMessage(msg::ModuleId::MOTOR, &msg_motor);
}

void mll::MotorController::setTargetPosition(float target_x, float target_y, float target_angle) {
    this->target_x = target_x;
    this->target_y = target_y;
    this->target_angle = target_angle;
}

void mll::MotorController::setTargetVelocity(float target_velocity_translation, float target_velocity_rotation) {
    this->target_velocity_translation = target_velocity_translation;
    this->target_velocity_rotation = target_velocity_rotation;
}

void mll::MotorController::resetIntegralTransition() {
    integral_translation = 0;
}

void mll::MotorController::resetIntegralRotation() {
    integral_rotation = 0;
}

void mll::MotorController::resetIntegral() {
    resetIntegralTransition();
    resetIntegralRotation();
}

void mll::MotorController::setStay() {
    msg_server->receiveMessage(msg::ModuleId::LOCALIZER, &msg_localizer);
    resetIntegral();
    setTargetPosition(msg_localizer.position_x, msg_localizer.position_y, msg_localizer.position_theta);
    startControl();
}

void mll::MotorController::interruptPeriodic() {
    static msg::MsgFormatBattery msg_battery;
    msg_server->receiveMessage(msg::ModuleId::LOCALIZER, &msg_localizer);
    msg_server->receiveMessage(msg::ModuleId::BATTERY, &msg_battery);
    msg_server->receiveMessage(msg::ModuleId::MOTORCONTROLLER, &msg_motor_controller);

    // OperationController からの指令を処理 (MOTORCONTROLLER)
    if (!enabled && msg_motor_controller.is_controlled) {
        startControl();
    } else if (!enabled || !msg_motor_controller.is_controlled) {
        // モーター制御が有効でない場合は何もしない
        stopControl();
        return;
    }
    setTargetVelocity(msg_motor_controller.velocity_translation, msg_motor_controller.velocity_rotation);

    // TODO: msg::ModuleId::ERROR_CONTROL を受け、エラーが発生している場合はモーター制御を停止する
    if (integral_rotation > 300.f) {
        stopControl();
        return;
    }

    [[maybe_unused]]
    const uint8_t params_index = 0;  // FIXME: cmd か msg から取得する

    // translation 成分の計算、出力は電流 [mA]
    // const float error_translation = calcDistanceToTarget(msg_localizer.position_x, msg_localizer.position_y, target_x, target_y);
    // const auto error_vector =
    //     misc::calcErrorVector(misc::Point<float>{msg_localizer.position_x, msg_localizer.position_y}, misc::Point<float>{target_x, target_y});
    // const auto direction_unit_vector = misc::calcDirectionUnitVector(msg_localizer.position_theta);
    // const auto error_translation = misc::calcProjectionToDirection(error_vector, direction_unit_vector);
    const float error_translation = target_velocity_translation - msg_localizer.velocity_translation;

    integral_translation += error_translation;  // FIXME: サンプリング周期をパラメータか固定値から読み出す
    const float error_diff_translation = error_translation - last_differential_translation;
    last_differential_translation = error_translation;
    const float control_translation = params->motor_control_translation_kp * error_translation +
                                      params->motor_control_translation_ki * integral_translation +
                                      params->motor_control_translation_kd * error_diff_translation;

    // rotation 成分の計算、出力は電流 [mA]
    // const float error_rotation = calcAngleToTarget(msg_localizer.position_theta, target_angle);
    const float error_rotation = target_velocity_rotation - msg_localizer.velocity_rotation;

    integral_rotation += error_rotation;  // FIXME: サンプリング周期をパラメータか固定値から読み出す
    const float error_diff_rotation = error_rotation - last_differential_rotation;
    last_differential_rotation = error_rotation;
    const float control_rotation = params->motor_control_rotation_kp * error_rotation + params->motor_control_rotation_ki * integral_rotation +
                                   params->motor_control_rotation_kd * error_diff_rotation;

    // モーターに指令を送る
    static msg::MsgFormatMotor msg_motor;
    msg_motor.duty_l = (control_translation - control_rotation) / 1000.f / msg_battery.battery;
    msg_motor.duty_r = (control_translation + control_rotation) / 1000.f / msg_battery.battery;
    msg_motor.duty_suction = 0;  // TODO: 吸引ファンを使う
    msg_server->sendMessage(msg::ModuleId::MOTOR, &msg_motor);

    // static auto debug = mpl::Debug::getInstance();
    // cmd::CommandFormatDebugTx cmd_debug_tx = {};
    // cmd_debug_tx.len = debug->format(cmd_debug_tx.message, "%10d, % 5.2f, % 5.2f, % 5.2f, % 5.2f, % 5.2f, % 5.2f, % 5.2f\n",
    //                                  mpl::Timer::getMicroTime(), msg_localizer.position_x, msg_localizer.position_y, msg_localizer.position_theta,
    //                                  error_vector.x, error_vector.y, error_translation, control_translation);
    // cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
}

mll::MotorController* mll::MotorController::getInstance() {
    static MotorController instance;
    return &instance;
}
