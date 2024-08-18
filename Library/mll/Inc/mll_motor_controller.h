//******************************************************************************
// @addtogroup MLL
// @file       mll_motor_controller.h
// @brief      モーターの回転数制御を行うクラス
//******************************************************************************
#pragma once

#include "cmd_server.h"
#include "mll_position.h"
#include "msg_format_localizer.h"
#include "msg_format_motor_controller.h"
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
    msg::MsgFormatMotorController msg_motor_controller;

    misc::MouseParams* params;

    // 位置制御時の目標位置
    float target_x;
    float target_y;
    float target_dif_distance;
    float target_angle;
    float target_dif_angle;

    // 速度制御時の目標速度
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

    // 【目標位置と現在位置の差分を計算】
    // 目標距離に対して遅れている場合が正
    float calcDistanceToTarget(float current_x, float current_y, float target_x, float target_y);
    // 反時計回りが正
    float calcAngleToTarget(float current_angle, float target_angle);

   public:
    void init();

    // モーター制御を有効化
    void startControl();
    // モーター制御を無効化
    void stopControl();

    // 目標位置を設定する
    void setTargetPosition(float target_x, float target_y, float target_angle);
    // 目標速度を設定する
    void setTargetVelocity(float target_velocity_translation, float target_velocity_rotation);

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
