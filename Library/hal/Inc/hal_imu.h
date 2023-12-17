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

#ifdef LINUX
#include <cstdint>
#endif

namespace hal {

#ifdef MOUSE_VIOLETTA
enum class GyroAxises : uint8_t {
    YAW = 0,
    ROLL,
    PITCH,
};
#endif  // ifdef MOUSE_VIOLETTA

struct ImuData {
    uint16_t OUT_TEMP;
    uint16_t OUT_X_G;
    uint16_t OUT_Y_G;
    uint16_t OUT_Z_G;
    uint16_t OUT_X_A;
    uint16_t OUT_Y_A;
    uint16_t OUT_Z_A;
};

HalStatus initImuPort();
HalStatus deinitImuPort();

HalStatus whoamiImu();

HalStatus getImuDataSync(ImuData& data);
// HalStatus getImuDataAsync();
// HalStatus getImuDataDma();

}  // namespace hal
