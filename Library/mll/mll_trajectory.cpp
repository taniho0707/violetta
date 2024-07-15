//******************************************************************************
// @addtogroup MLL
// @file       mll_trajectory.cpp
// @brief      マウスの軌跡を作成するクラス
//******************************************************************************
#include "mll_trajectory.h"

#include "arm_math.h"
#include "stdint.h"
#include "util.h"

namespace mll {

Trajectory::Trajectory() {
    calc = TrajectoryCalcType::TIME;
    form = TrajectoryFormType::TRAPEZOID;
    v_start = 0.0f;
    v_max = 0.0f;
    v_end = 0.0f;
    accel = 0.0f;
    distance = 0.0f;
    t_1 = 0;
    t_2 = 0;
    t_end = 0;
}

bool Trajectory::availableConstantVelocity() const {
    if (form == TrajectoryFormType::TRAPEZOID) {
        if (v_start == v_end) {
            return (v_max * v_max - v_start * v_start) / misc::abs(accel) <= misc::abs(distance);
        } else {
            return (2 * v_max * v_max - v_start * v_start - v_end * v_end) / misc::abs(2 * accel) <= misc::abs(distance);
        }
    } else {
        return false;
    }
}

void Trajectory::init(TrajectoryCalcType calc, TrajectoryFormType form,
                      float a, float d, float v_start, float v_max, float v_end) {
    this->calc = calc;
    this->form = form;

    if (this->form == TrajectoryFormType::TRAPEZOID) {
        this->v_start = v_start;
        this->v_max = v_max;
        this->v_end = v_end;
        this->accel = a;
        this->distance = d;

        include_constant_velocity = availableConstantVelocity();

        if (include_constant_velocity) {
            float x_1 = (v_max * v_max - v_start * v_start) / misc::abs(2 * accel);
            float x_3 = (v_max * v_max - v_end * v_end) / misc::abs(2 * accel);
            float x_2 = misc::abs(distance) - misc::abs(x_1) - misc::abs(x_3);
            t_1 = ((v_max - v_start) / accel) * 1000;
            t_2 = (x_2 / misc::abs(v_max)) * 1000 + t_1;
            t_end = ((v_max - v_end) / accel) * 1000 + t_2;
        } else {
            arm_sqrt_f32((misc::abs(2 * accel * distance) + v_start * v_start + v_end * v_end) / 2, &this->v_max);
            // float x_1 = (v_max * v_max - v_start * v_start) / (2 * accel);
            // float x_3 = (v_max * v_max - v_end * v_end) / (2 * accel);
            t_1 = ((v_max - v_start) / accel) * 1000;
            t_2 = t_1;
            t_end = t_1 + (v_max - v_end) / accel;
        }
    }
}

// FIXME: 境界条件の返り値が不正
float Trajectory::getAccel(const uint32_t t) const {
    if (form == TrajectoryFormType::TRAPEZOID) {
        if (t < t_1) {
            return accel;
        } else if (t < t_2) {
            return 0.0f;
        } else if (t < t_end) {
            return -accel;
        } else {
            return 0.0f;
        }
    } else {
        return 0.0f;
    }
}

float Trajectory::getVelocity(const uint32_t t) const {
    if (form == TrajectoryFormType::TRAPEZOID) {
        if (t < t_1) {
            return v_start + accel * t / 1000;
        } else if (t <= t_2) {
            return v_max;
        } else if (t < t_end) {
            return v_max - accel * (t - t_2) / 1000;
        } else {
            return v_end;
        }
    } else {
        return 0.0f;
    }
}

float Trajectory::getDistance(const uint32_t t) const {
    // TODO: IMPLEMENT
    return 0.0f;
}

bool Trajectory::isEnd(const uint32_t time) const {
    return time >= t_end;
}

float Trajectory::getMaxVelocity() const {
    return v_max;
}

uint16_t Trajectory::getTimeStartConstant() const {
    return t_1;
}

uint16_t Trajectory::getTimeStartDeceleration() const {
    return t_2;
}

uint16_t Trajectory::getTimeEnd() const {
    return t_end;
}

}  // namespace mll
