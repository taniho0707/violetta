//******************************************************************************
// @addtogroup MISC
// @file       misc_util.h
// @brief      汎用的に使う関数郡
//******************************************************************************
#pragma once

#include "stdint.h"

namespace misc {

template <typename T>
inline T min(T a, T b) {
    return a < b ? a : b;
}

template <typename T>
inline T min(T a, T b, T c, T d) {
    return min(min(a, b), min(c, d));
}

template <typename T>
T min(T* array, uint16_t len) {
    T min = array[0];
    for (uint16_t i = 0; i < len; ++i) {
        if (array[i] < min) {
            min = array[i];
        }
    }
    return min;
}

template <typename T>
inline T max(T a, T b) {
    return a > b ? a : b;
}

template <typename T>
inline T max(T a, T b, T c, T d) {
    return max(max(a, b), max(c, d));
}

template <typename T>
T max(T* array, uint16_t len) {
    T max = array[0];
    for (uint16_t i = 0; i < len; ++i) {
        if (array[i] > max) {
            max = array[i];
        }
    }
    return max;
}

template <typename T>
T average(T* array, uint16_t len) {
    T sum = 0;
    for (uint16_t i = 0; i < len; ++i) {
        sum += (array[i] / len);
    }
    return sum;
}

}  // namespace misc
