//******************************************************************************
// @addtogroup MISC
// @file       misc_util.h
// @brief      汎用的に使う関数郡
//******************************************************************************
#pragma once

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
inline T max(T a, T b) {
    return a > b ? a : b;
}

template <typename T>
inline T max(T a, T b, T c, T d) {
    return max(max(a, b), max(c, d));
}

}  // namespace misc
