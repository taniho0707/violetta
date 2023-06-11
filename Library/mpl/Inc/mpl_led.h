//******************************************************************************
// @addtogroup MPL
// @file       mpl_led.h
// @brief      LED点灯制御
//******************************************************************************
#pragma once

// STL
#include <array>

#include "hal_led.h"

namespace mpl {

struct LedFlickParams {
    uint32_t start_time = 0;
    float freq = 0;
    uint32_t time = 0;
};

class Led {
   private:
    std::array<LedFlickParams, LED_NUMS> flick_params;

    Led();

   public:
    void initPort(hal::LedNumbers num);
    void deinitPort(hal::LedNumbers num);

    bool isFlicking(hal::LedNumbers num);

    void on(hal::LedNumbers num);
    void off(hal::LedNumbers num);

    /// @params time. 0 to infinite
    void flickSync(hal::LedNumbers num, float freq, uint16_t time);
    void flickAsync(hal::LedNumbers num, float freq, uint16_t time);
    void flickStop(hal::LedNumbers num);

    void interrupt();

    static Led* getInstance();
};

}  // namespace mpl
