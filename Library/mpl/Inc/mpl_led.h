//******************************************************************************
// @addtogroup MPL
// @file       mpl_led.h
// @brief      LED点灯制御
//******************************************************************************
#pragma once

// STL
#include <array>

#ifdef STM32L4P5xx
// STM32HAL/LL
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_gpio.h"
#endif // ifdef STM32L4P5xx

#ifdef LINUX
#include <cstdint>
#endif

#ifdef MOUSE_VIOLETTA
#define LED_NUMS 4
enum class LedNumbers : uint8_t {
    FRONT1,
    FRONT2,
    TOP1,
    TOP2,
};
#endif

struct LedFlickParams {
    uint32_t start_time = 0;
    float freq = 0;
    uint32_t time = 0;
};

class Led {
   private:

#ifdef STM32L4P5xx
    GPIO_TypeDef* gpio_port;
#endif // ifdef STM32L4P5xx

    uint16_t gpio_channel;
    std::array<LedFlickParams, LED_NUMS> flick_params;

    Led();

    void setType(LedNumbers num);

   public:
    void initPort(LedNumbers num);
    void deinitPort(LedNumbers num);

    void on(LedNumbers num);
    void off(LedNumbers num);

    bool isFlicking(LedNumbers num);

    /// @params time. 0 to infinite
    void flickSync(LedNumbers num, float freq, uint16_t time);
    void flickAsync(LedNumbers num, float freq, uint16_t time);
    void flickStop(LedNumbers num);

    void interrupt();

    static Led* getInstance();
};
