//******************************************************************************
// @addtogroup HAL
// @file       hal_motor.h
// @brief      モータ制御
//******************************************************************************
#pragma once

#include "hal_conf.h"

#ifdef STM32L4P5xx
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#endif  // ifdef STM32F411xE

#ifdef LINUX
#include <cstdint>

#include "observer.h"
#endif

namespace hal {

#ifdef MOUSE_LAZULI
// FIXME: hal_conf に移動させる
constexpr uint32_t MOTOR_TIMER_MAXCOUNT = 262144 / 128 / 4;
#endif  // ifdef MOUSE_LAZULI

HalStatus initMotorPort();
HalStatus deinitMotorPort();

// Drive Motor
HalStatus setMotorDutyL(float duty);
HalStatus setMotorDutyR(float duty);

HalStatus setMotorFloat();

// return: [mA]
HalStatus getMotorCurrentSync(float& current_l, float& current_r);

// Suction Motor
HalStatus setMotorDutySuction(float duty);

}  // namespace hal
