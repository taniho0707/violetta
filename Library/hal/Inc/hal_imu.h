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

HalStatus initImuPort();
HalStatus deinitImuPort();

HalStatus getData();

}  // namespace hal
