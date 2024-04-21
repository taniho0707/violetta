//******************************************************************************
// @addtogroup HAL
// @file       hal_debug.h
// @brief      デバッグ用機能(UARTログ等)
//******************************************************************************
#pragma once

#include "hal_conf.h"

#ifdef STM32L4P5xx
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#endif  // ifdef STM32F411xE

#ifdef LINUX
#include <cstdint>

#include "observer.h"
#endif

namespace hal {

#ifdef MOUSE_VIOLETTA
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_ZIRCONIA2KAI
#endif  // ifdef MOUSE_ZIRCONIA2KAI

HalStatus initUartDebugPort();
HalStatus deinitUartDebugPort();

HalStatus sendUartDebug1Byte(uint8_t data);
HalStatus sendUartDebugNByte(uint8_t *data, const int len);
HalStatus sendUartDebugNByte(char *data, const int len);
// HalStatus getImuDataAsync();
// HalStatus getImuDataDma();

}  // namespace hal
