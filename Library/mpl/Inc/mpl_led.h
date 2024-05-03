//******************************************************************************
// @addtogroup MPL
// @file       mpl_led.h
// @brief      LED点灯制御
//******************************************************************************
#pragma once

// STL
#include <array>

#include "hal_conf.h"
#include "hal_led.h"
#include "mpl_conf.h"

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

    I2cAsyncState i2c_state;

   public:
    mpl::MplStatus initPort(hal::InitializeType type = hal::InitializeType::Sync);
    void deinitPort();

    bool isFlicking(hal::LedNumbers num);

    void on(hal::LedNumbers num);
    void off(hal::LedNumbers num);

    /// @params time. 0 to infinite
    void flickSync(hal::LedNumbers num, float freq, uint16_t time);
    void flickAsync(hal::LedNumbers num, float freq, uint16_t time);
    void flickStop(hal::LedNumbers num);

    void interruptPeriodic();
    void interruptI2cRxComplete();
    void interruptI2cTxComplete();
    void interruptDmaTxComplete();
    void interruptDmaTxError();

    static Led* getInstance();
};

}  // namespace mpl
