//******************************************************************************
// @addtogroup MISC
// @file       misc_util.h
// @brief      汎用的に使う関数郡
//******************************************************************************
#pragma once

#include <stdint.h>

namespace misc {

inline uint32_t min(uint32_t a, uint32_t b) { return a < b ? a : b; }

}  // namespace misc
