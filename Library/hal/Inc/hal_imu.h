//******************************************************************************
// @addtogroup HAL
// @file       hal_imu.h
// @brief      IMU 6軸センサ制御
//******************************************************************************
#pragma once

#include "hal_conf.h"

#ifdef STM32L4P5xx
// STM32HAL/LL
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_spi.h"
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"
#endif  // ifdef STM32F411xE

#ifdef LINUX
#include <cstdint>

#include "observer.h"
#endif

namespace hal {

#ifdef MOUSE_VIOLETTA
enum class GyroAxises : uint8_t {
    YAW = 0,
    ROLL,
    PITCH,
};
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_ZIRCONIA2KAI
enum class GyroAxises : uint8_t {
    YAW = 0,
    ROLL,
    PITCH,
};
#endif  // ifdef MOUSE_ZIRCONIA2KAI

HalStatus initImuPort();
HalStatus deinitImuPort();

HalStatus whoamiImu();

HalStatus getImuDataSync(ImuData& data);
// HalStatus getImuDataAsync();
// HalStatus getImuDataDma();

}  // namespace hal
