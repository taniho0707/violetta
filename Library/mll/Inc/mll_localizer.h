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
};

class Localizer {
   private:
    Localizer();

    MouseSectionPosition current_section;

    misc::MouseParams* params;

    float encoder_left[ENCODER_BUFFER_LENGTH];
    float encoder_right[ENCODER_BUFFER_LENGTH];
    uint16_t encoder_index;

    void updateAveragedEncoder(float left, float right);
    void getAveragedEncoder(float& left, float& right);

   public:
    // 現在の推測内容
    LocalizedStatus current_status;

    // デバッグ用、リセットをかけない値
    LocalizedStatus total_status;

    // デバッグ用、エンコーダのみを使った値
    LocalizedStatus encoder_status;
    // デバッグ用、加速度センサのみを使った値
    LocalizedStatus imu_status;

    void init();

    // マウス座標を強制的に変更する
    void setPosition(float x, float y, float theta);
    // void setSectionPosition(int16_t x, int16_t y, CardinalDirection d);

    void interruptPeriodic();

    static Localizer* getInstance();
};

}  // namespace mll
