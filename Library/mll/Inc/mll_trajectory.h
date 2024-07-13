//******************************************************************************
// @addtogroup MLL
// @file       mll_trajectory.h
// @brief      マウスの軌跡を作成するクラス
//******************************************************************************
#pragma once

#include "stdint.h"

namespace mll {

// 計算を時間ベースとするか距離ベースとするか
// 取り急ぎ時間ベースのみ実装する
enum class TrajectoryCalcType : uint8_t {
    TIME = 0,
    DISTANCE,
};

// 軌跡の形状
enum class TrajectoryFormType : uint8_t {
    TRAPEZOID = 0,
    CUBIC,
    TRIGONOMETRIC,
};

class Trajectory {
   private:
    TrajectoryCalcType calc;
    TrajectoryFormType form;

    float v_start;   // 開始速度 [mm/s]
    float v_max;     // 最高速度 [mm/s]
    float v_end;     // 終了速度 [mm/s]
    float accel;     // 加速度 [mm/s^2]
    float distance;  // 距離 [mm]

    uint16_t t_1;    // 加速終了時刻 [ms]
    uint16_t t_2;    // 減速開始時刻 [ms]
    uint16_t t_end;  // 終了時刻 [ms]

    bool availableConstantVelocity() const;  // 等速度区間が存在するか
    bool include_constant_velocity;          // 等速度区間が存在するかのキャッシュ

   public:
    Trajectory();

    void init(TrajectoryCalcType calc, TrajectoryFormType form,
              float a, float d, float v_start, float v_max, float v_end);

    float getAccel(const uint32_t t) const;
    float getVelocity(const uint32_t t) const;
    float getDistance(const uint32_t t) const;

    bool isEnd(const uint32_t time) const;

    float getMaxVelocity() const;

    uint16_t getTimeStartConstant() const;      // 加速終了時刻
    uint16_t getTimeStartDeceleration() const;  // 減速開始時刻
    uint16_t getTimeEnd() const;
};

}  // namespace mll
