//******************************************************************************
// @addtogroup HAL
// @file       hal_wallsensor.h
// @brief      壁センサー制御
//******************************************************************************
#pragma once

#include "hal_conf.h"

#ifdef STM32L4P5xx
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
#include "stm32f411xe.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_utils.h"
#endif  // ifdef STM32F411xE

#ifdef LINUX
#include <cstdint>

#include "observer.h"
#endif

namespace hal {

enum class AD7091RCommands : uint16_t {
    CONVERSION_RESULT = 0x0000,
    CHANNEL = 0x0800,              // 0x01,
    CONFIGURATION = 0x1000,        // 0x02,
    ALERT_INDICATION = 0x1800,     // 0x03,
    CHANNEL0_LOW_LIMIT = 0x2000,   // 0x04,
    CHANNEL0_HIGH_LIMIT = 0x2800,  // 0x05,
    CHANNEL0_HYSTERESIS = 0x3000,  // 0x06,
    CHANNEL1_LOW_LIMIT = 0x3800,   // 0x07,
    CHANNEL1_HIGH_LIMIT = 0x4000,  // 0x08,
    CHANNEL1_HYSTERESIS = 0x4800,  // 0x09,
    CHANNEL2_LOW_LIMIT = 0x5000,   // 0x0A,
    CHANNEL2_HIGH_LIMIT = 0x5800,  // 0x0B,
    CHANNEL2_HYSTERESIS = 0x6000,  // 0x0C,
    CHANNEL3_LOW_LIMIT = 0x6800,   // 0x0D,
    CHANNEL3_HIGH_LIMIT = 0x7000,  // 0x0E,
    CHANNEL3_HYSTERESIS = 0x7800,  // 0x0F,
    NOP = 0xF800,                  // 0x1F,
};
const uint16_t WALLSENSOR_WRITE_MASK = 0x0400;
// READ: 0, WRITE: 1
// Address MSB 5bits + R/W 1bit + Data 10bits

HalStatus initWallSensorPort();
HalStatus deinitWallSensorPort();

HalStatus setWallSensorLedOn(WallSensorNumbers n);
HalStatus setWallSensorLedOff(WallSensorNumbers n);
HalStatus setWallSensorLedOff();

HalStatus setWallSensorChargeStart();
HalStatus setWallSensorChargeStop();

// AD7091R用の関数
// 変換トリガのトグルは1000ns以上の間隔をあける
// 変換トリガから変換終了までの時間は最大600ns
// SPIクロックが止まってから変換開始までは50ns以上の間隔をあける
HalStatus startWallSensorConversion();
HalStatus setWallSensorAdcSelect(WallSensorNumbers n);
HalStatus getWallSensorSingleSync(uint16_t& data);

HalStatus getWallSensorSingleSync(uint16_t& data, WallSensorNumbers n);

// LazuliSensor 用の関数
// param: data = WallSensorNum の長さの配列
HalStatus getWallSensorAllSync(uint16_t* data);

// HalStatus getWallSensorAllSync(WallSensorData& data);
// HalStatus getBatteryVoltageAsync();
// HalStatus getBatteryVoltageDma();

}  // namespace hal
