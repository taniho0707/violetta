//******************************************************************************
// @addtogroup MLL
// @file       mll_localizer.h
// @brief      マウスの自己位置推定ロジック・自己位置管理
//******************************************************************************
#pragma once

#include "mll_position.h"
#include "params.h"

namespace mll {

constexpr uint16_t ENCODER_BUFFER_LENGTH = 10;

struct LocalizedStatus {
    float accel_translation;
    float accel_rotation;
    float velocity_translation;
    float velocity_rotation;

    // 左下柱中心が (x, y) = (0, 0) [mm]
    float position_x;
    float position_y;

    // 計算用、直線距離 [mm]
    float position_translation;

    // 時計回りに正の [radian]
    float position_theta;

    MouseSectionPosition toSectionPosition() {
        MouseSectionPosition ret;
        ret.x = static_cast<int16_t>(position_x / 90.f);
        ret.y = static_cast<int16_t>(position_y / 90.f);
        ret.d = static_cast<CardinalDirection>(static_cast<int16_t>((position_theta + misc::PI / 4.f) / (misc::PI / 2.f)) % 4);
        return ret;
    }
};

class Localizer {
   private:
    Localizer();

    // 区画走行のときに使う、マウスの論理位置情報
    MouseSectionPosition current_section;

    misc::MouseParams* params;

    float encoder_left[ENCODER_BUFFER_LENGTH];
    float encoder_right[ENCODER_BUFFER_LENGTH];
    uint16_t encoder_index;

    // エンコーダの値を移動平均して推定するための諸々
    void updateAveragedEncoder(float left, float right);
    void getAveragedEncoder(float& left, float& right);

   public:
    // 現在の推測内容
    // 区画走行とパス走行の両方で使うマウスの物理位置情報
    LocalizedStatus current_status;

    // デバッグ用、リセットをかけない値
    LocalizedStatus total_status;

    // デバッグ用、エンコーダのみを使った値
    LocalizedStatus encoder_status;
    // デバッグ用、加速度センサのみを使った値
    LocalizedStatus imu_status;

    // クラスの初期化、起動時に一度のみ呼び出す想定
    void init();

    // マウス座標を強制的に変更する
    void setPosition(float x, float y, float theta);
    // void setSectionPosition(int16_t x, int16_t y, CardinalDirection d);

    // 進行方向に対して、(x, y) を通る垂直な直線を超えたかどうかを判定し、超えた場合は正の値を返す
    float distanceFromCrossedLine(IntercardinalDirection d, float x, float y);

    void interruptPeriodic();

    static Localizer* getInstance();
};

}  // namespace mll
