//******************************************************************************
// @addtogroup MLL
// @file       mll_motor_controller.h
// @brief      モーターの回転数制御を行うクラス
//******************************************************************************
#include "mll_motor_controller.h"

#include "mpl_debug.h"
#include "mpl_timer.h"
#include "msg_format_battery.h"
#include "msg_format_motor.h"
#include "msg_server.h"

mll::MotorController::MotorController() {
    enabled = false;
    target_velocity_translation = 0;
    target_velocity_rotation = 0;
}

void mll::MotorController::init() {
    params = misc::Params::getInstance()->getCachePointer();

    msg_server = msg::MessageServer::getInstance();
    cmd_server = cmd::CommandServer::getInstance();
}

void mll::MotorController::startControl() {
    enabled = true;
    resetIntegral();
}

void mll::MotorController::stopControl() {
    enabled = false;
}

void mll::MotorController::setVelocityTransition(float velocity_transition) {
    target_velocity_translation = velocity_transition;
}

void mll::MotorController::setVelocityRotation(float velocity_rotation) {
    target_velocity_rotation = velocity_rotation;
}

void mll::MotorController::setVelocity(float velocity_transition, float velocity_rotation) {
    setVelocityTransition(velocity_transition);
    setVelocityRotation(velocity_rotation);
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
    resetIntegral();
    setVelocity(0, 0);
    startControl();
}

void mll::MotorController::interruptPeriodic() {
    static msg::MsgFormatBattery msg_battery;
    msg_server->receiveMessage(msg::ModuleId::LOCALIZER, &msg_localizer);
    msg_server->receiveMessage(msg::ModuleId::BATTERY, &msg_battery);

    // TODO: msg::ModuleId::ERROR_CONTROL を受け、エラーが発生している場合はモーター制御を停止する

    const uint8_t params_index = 0;  // FIXME: cmd か msg から取得する

    // モーター制御が有効でない場合は何もしない
    if (!enabled) {
        return;
    }

    // translation 成分の計算、出力は電流 [mA]
    const float error_translation = target_velocity_translation - msg_localizer.velocity_translation;
    integral_translation += error_translation;  // FIXME: サンプリング周期をパラメータか固定値から読み出す
    const float error_diff_translation = error_translation - last_differential_translation;
    last_differential_translation = error_translation;
    const float control_translation = params->motor_control_translation_kp * error_translation +
                                      params->motor_control_translation_ki * integral_translation +
                                      params->motor_control_translation_kd * error_diff_translation;

    // rotation 成分の計算、出力は電流 [mA]
    const float error_rotation = msg_localizer.velocity_rotation - target_velocity_rotation;  // FIXME: 逆になっているので戻す
    integral_rotation += error_rotation;                                                      // FIXME: サンプリング周期をパラメータか固定値から読み出す
    const float error_diff_rotation = error_rotation - last_differential_rotation;
    last_differential_rotation = error_rotation;
    const float control_rotation = params->motor_control_rotation_kp * error_rotation +
                                   params->motor_control_rotation_ki * integral_rotation +
                                   params->motor_control_rotation_kd * error_diff_rotation;

    // モーターに指令を送る
    static msg::MsgFormatMotor msg_motor;
    msg_motor.duty_l = (control_translation + control_rotation) / 1000.f / msg_battery.battery;
    msg_motor.duty_r = (control_translation - control_rotation) / 1000.f / msg_battery.battery;
    msg_motor.duty_suction = 0;  // TODO: 吸引ファンを使う
    msg_server->sendMessage(msg::ModuleId::MOTOR, &msg_motor);

    // static float encoder_current_l = 0;
    // static float encoder_current_r = 0;
    // static float encoder_total_l = 0;
    // static float encoder_total_r = 0;
    // static float accel_current = 0;
    // static float accel_total = 0;
    // static float gyro_current = 0;
    // static float gyro_total = 0;

    // encoder_current_l = msg_encoder.left;
    // encoder_current_r = msg_encoder.right;
    // encoder_total_l += encoder_current_l;
    // encoder_total_r += encoder_current_r;
    // accel_current = msg_imu.acc_y;
    // accel_total += accel_current;
    // gyro_current = msg_imu.gyro_yaw;
    // gyro_total += gyro_current;

    // static auto debug = mpl::Debug::getInstance();
    // cmd::CommandFormatDebugTx cmd_debug_tx = {};
    // cmd_debug_tx.len = debug->format(cmd_debug_tx.message,
    //                                  "%10d, %f, %f, %f, %f, %f, %f\n",
    //                                  mpl::Timer::getMicroTime(),
    //                                  encoder_current_l, encoder_current_r,
    //                                  encoder_total_l, encoder_total_r,
    //                                  accel_current, gyro_current);
    // cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);
}

mll::MotorController* mll::MotorController::getInstance() {
    static MotorController instance;
    return &instance;
}
