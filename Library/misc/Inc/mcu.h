//******************************************************************************
// @addtogroup MISC
// @file       mcu.h
// @brief      MCU specific misc functions
//******************************************************************************
#pragma once

#include "stdint.h"

#ifdef STM32L4P5xx
#include "stm32l4xx_ll_utils.h"
#endif  // ifdef STM32L4P5xx

#ifdef STM32C011xx
#include "stm32c0xx_ll_utils.h"
#endif  // ifdef STM32C011xx

#ifdef STM32F411xE
#include "stm32f4xx_ll_utils.h"
#endif  // ifdef STM32F411xE

namespace misc {

#ifdef STM32
// return 96bit unique id (uid[0]: 31-0, uid[1]: 63-32, uid[2]: 95-64)
inline uint8_t getUid(uint32_t* uid) {
    uid[0] = LL_GetUID_Word0();
    uid[1] = LL_GetUID_Word1();
    uid[2] = LL_GetUID_Word2();
    return 3;
}
#endif  // STM32

#ifdef LINUX
// FIXME: return dummy uid
inline uint8_t getUid(uint32_t* uid) {
    uid[0] = 0;
    uid[1] = 0;
    uid[2] = 0;
    return 3;
}
#endif  // LINUX

}  // namespace misc
