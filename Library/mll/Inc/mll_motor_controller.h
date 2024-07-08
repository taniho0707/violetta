//******************************************************************************
// @addtogroup MLL
// @file       mll_motor_controller.h
// @brief      モーターの回転数制御を行うクラス
//******************************************************************************
#pragma once

#include "cmd_server.h"
#include "msg_format_localizer.h"
#include "msg_server.h"
#include "params.h"

namespace mll {

class MotorController {
   private:
    MotorController();

    msg::MessageServer* msg_server;
    cmd::CommandServer* cmd_server;

    // msg::MsgFormatEncoder msg_encoder;
    // msg::MsgFormatImu msg_imu;
    msg::MsgFormatLocalizer msg_localizer;

    misc::MouseParams* params;

    float target_velocity_translation;
    float target_velocity_rotation;

    // MotorController によるモーター制御有効フラグ
    bool enabled;

    // PID 制御のための各パラメータ
    // Ki
    float integral_translation;
    float integral_rotation;
    // Kd
    float last_differential_translation;
    float last_differential_rotation;

   public:
    void init();

    // モーター制御を有効化
    void startControl();
    // モーター制御を無効化
    void stopControl();

    // 速度・角速度を設定する
    void setVelocityTransition(float velocity_transition);
    void setVelocityRotation(float velocity_rotation);
    void setVelocity(float velocity_transition, float velocity_rotation);

    //////////////////////////////////
    //  制御内部の状態に関する実装  //
    //////////////////////////////////
    void resetIntegralTransition();
    void resetIntegralRotation();
    void resetIntegral();

    // 速度・角速度 0 でモーター制御を開始する
    void setStay();

    void interruptPeriodic();

    static MotorController* getInstance();
};

}  // namespace mll
