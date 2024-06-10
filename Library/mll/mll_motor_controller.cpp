//******************************************************************************
// @addtogroup MLL
// @file       mll_motor_controller.h
// @brief      モーターの回転数制御を行うクラス
//******************************************************************************
#include "mll_motor_controller.h"

#include "mpl_debug.h"
#include "mpl_timer.h"

mll::MotorController::MotorController() {
}

void mll::MotorController::init() {
    // params = misc::Params::getInstance()->getCachePointer();

    msg_server = msg::MessageServer::getInstance();
    cmd_server = cmd::CommandServer::getInstance();
}

void mll::MotorController::interruptPeriodic() {
    msg_server->receiveMessage(msg::ModuleId::ENCODER, &msg_encoder);
    msg_server->receiveMessage(msg::ModuleId::IMU, &msg_imu);

    static float encoder_current_l = 0;
    static float encoder_current_r = 0;
    static float encoder_total_l = 0;
    static float encoder_total_r = 0;
    static float accel_current = 0;
    static float accel_total = 0;
    static float gyro_current = 0;
    static float gyro_total = 0;

    encoder_current_l = msg_encoder.left;
    encoder_current_r = msg_encoder.right;
    encoder_total_l += encoder_current_l;
    encoder_total_r += encoder_current_r;
    accel_current = msg_imu.acc_y;
    accel_total += accel_current;
    gyro_current = msg_imu.gyro_yaw;
    gyro_total += gyro_current;

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
