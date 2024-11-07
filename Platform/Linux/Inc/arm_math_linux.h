//******************************************************************************
// @addtogroup PLT
// @file       arm_math_linux.h
// @brief      arm_math.h のあれ
//******************************************************************************
#pragma once

#include <cmath>

#include "hal_conf.h"

namespace plt {

float arm_cos_f32(float x);
float arm_sin_f32(float x);
uint8_t arm_sqrt_f32(float x, float* pResult);

}  // namespace plt
