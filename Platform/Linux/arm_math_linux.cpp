//******************************************************************************
// @addtogroup PLT
// @file       arm_math_linux.cpp
// @brief      arm_math.h のあれ
//******************************************************************************
#pragma once

#include "arm_math_linux.h"

#include <cmath>

float plt::arm_cos_f32(float x) {
    return cos(x);
}

float plt::arm_sin_f32(float x) {
    return sin(x);
}

uint8_t plt::arm_sqrt_f32(float x, float* pResult) {
    *pResult = sqrt(x);
    return 0;
}
