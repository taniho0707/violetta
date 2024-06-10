//******************************************************************************
// @addtogroup MLL
// @file       mll_localizer.cpp
// @brief      マウスの自己位置推定ロジック・自己位置管理
//******************************************************************************
#include "mll_localizer.h"

#include "arm_math.h"
#include "math.h"
#include "msg_format_encoder.h"
#include "msg_format_imu.h"
#include "msg_format_localizer.h"
#include "msg_format_wallsensor.h"
#include "msg_server.h"

mll::Localizer::Localizer() {
}

void mll::Localizer::init() {
    // params = misc::Params::getInstance()->getCachePointer();
    setPosition(0, 0, 0);
}

void mll::Localizer::setPosition(float x, float y, float theta) {
    current_status.position_x = x;
    current_status.position_y = y;
    current_status.position_theta = theta;

    encoder_status.position_x = x;
    encoder_status.position_y = y;
    encoder_status.position_theta = theta;
}

void mll::Localizer::interruptPeriodic() {
    // エンコーダ、ジャイロ、加速度、壁センサの値を取得
    static auto msg_server = msg::MessageServer::getInstance();
    static msg::MsgFormatEncoder msg_encoder;
    static msg::MsgFormatImu msg_imu;
    static msg::MsgFormatWallsensor msg_wallsensor;
    msg_server->receiveMessage(msg::ModuleId::ENCODER, &msg_encoder);
    msg_server->receiveMessage(msg::ModuleId::IMU, &msg_imu);
    msg_server->receiveMessage(msg::ModuleId::WALLSENSOR, &msg_wallsensor);

    // ジャイロの値を使って角度方向を推定
    float gyro_yaw_vel = msg_imu.gyro_yaw * PI / 180.f;                        // [radian/s]
    float gyro_yaw_accel = (gyro_yaw_vel - current_status.velocity_rotation);  // [radian/s^2]
    float gyro_yaw_pos_delta = gyro_yaw_vel / 1000.f;                          // [radian]
    current_status.accel_rotation = gyro_yaw_accel;
    current_status.velocity_rotation = gyro_yaw_vel;
    current_status.position_theta += gyro_yaw_pos_delta;

    // エンコーダのみを使った推定
    float encoder_pos_delta = (msg_encoder.left + msg_encoder.right) / 2.f;            // [mm]
    float encoder_vel = encoder_pos_delta * 1000;                                      // [mm/s]
    float encoder_accel = (encoder_vel - encoder_status.velocity_translation) * 1000;  // [mm/s^2]
    encoder_status.accel_translation = encoder_accel;
    encoder_status.velocity_translation = encoder_vel;
    encoder_status.position_translation = encoder_pos_delta;

    // エンコーダの値に、加速度の値を使って相補フィルタを通した直進速を推定
    float accel_y_accel = msg_imu.acc_y;
    float accel_y_vel = imu_status.velocity_translation + (accel_y_accel / 1000.f);  // [mm/s]
    float accel_y_pos_delta = accel_y_vel / 1000.f;                                  // [mm]
    imu_status.accel_translation = accel_y_accel;
    imu_status.velocity_translation = accel_y_vel;
    imu_status.position_translation = accel_y_pos_delta;

    // 相補フィルタの定数
    // TODO: 相補フィルタの定数をparamsからロードする
    const float ALPHA = 1.0f;
    current_status.velocity_translation = ALPHA * encoder_status.velocity_translation + (1 - ALPHA) * imu_status.velocity_translation;
    current_status.accel_translation = ALPHA * encoder_status.accel_translation + (1 - ALPHA) * imu_status.accel_translation;
    current_status.position_translation = ALPHA * encoder_status.position_translation + (1 - ALPHA) * imu_status.position_translation;

    // 2軸の移動量から現在位置を推定
    float dif_x = encoder_status.position_translation * arm_sin_f32(current_status.position_theta);
    float dif_y = encoder_status.position_translation * arm_cos_f32(current_status.position_theta);
    current_status.position_x += dif_x;
    current_status.position_y += dif_y;

    // TODO: 壁センサの値を使って距離を補正

    // 自己位置を更新し、メッセージを作成
    static msg::MsgFormatLocalizer msg_localizer;
    msg_localizer.accel_translation = current_status.accel_translation;
    msg_localizer.accel_rotation = current_status.accel_rotation;
    msg_localizer.velocity_translation = current_status.velocity_translation;
    msg_localizer.velocity_rotation = current_status.velocity_rotation;
    msg_localizer.position_x = current_status.position_x;
    msg_localizer.position_y = current_status.position_y;
    msg_localizer.position_translation = current_status.position_translation;
    msg_localizer.position_theta = current_status.position_theta;
    msg_server->sendMessage(msg::ModuleId::LOCALIZER, &msg_localizer);
}

mll::Localizer* mll::Localizer::getInstance() {
    static Localizer instance;
    return &instance;
}
