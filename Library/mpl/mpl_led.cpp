//******************************************************************************
// @addtogroup MPL
// @file       mpl_led.cpp
// @brief      LED点灯制御
//******************************************************************************
#include "mpl_led.h"

#include "mpl_timer.h"

mpl::Led::Led() { hal::initLedPort(hal::LedNumbers::ALL); }

void mpl::Led::initPort(hal::LedNumbers num) { hal::initLedPort(num); }

void mpl::Led::deinitPort(hal::LedNumbers num) { hal::deinitLedPort(num); }

bool mpl::Led::isFlicking(hal::LedNumbers num) {
    if (flick_params.at(static_cast<uint8_t>(num)).start_time == 0)
        return false;
    else
        return true;
}

void mpl::Led::on(hal::LedNumbers num) { hal::onLed(num); }

void mpl::Led::off(hal::LedNumbers num) { hal::offLed(num); }

void mpl::Led::flickSync(hal::LedNumbers num, float freq, uint16_t time) {
    flickAsync(num, freq, time);
    while (isFlicking(num))
        ;
}

void mpl::Led::flickAsync(hal::LedNumbers num, float freq, uint16_t time) {
    flick_params.at(static_cast<uint8_t>(num)).start_time =
        Timer::getMilliTime();
    flick_params.at(static_cast<uint8_t>(num)).freq = freq;
    flick_params.at(static_cast<uint8_t>(num)).time = time;
}

void mpl::Led::flickStop(hal::LedNumbers num) {
    flick_params.at(static_cast<uint8_t>(num)).start_time = 0;
    hal::offLed(num);
}

void mpl::Led::interrupt() {
    uint32_t t;
    uint32_t t2;
    int8_t i = -1;
    for (auto& n : flick_params) {
        ++i;
        if (n.start_time == 0) continue;
        t2 = static_cast<uint32_t>(1000 / n.freq);
        t = (Timer::getMilliTime() - n.start_time) % t2;
        if ((n.time != 0) &&
            ((Timer::getMilliTime() - n.start_time) > n.time)) {
            n.start_time = 0;
            hal::offLed(static_cast<hal::LedNumbers>(i));
            continue;
        }
        if (t > (t2 / 2))
            hal::onLed(static_cast<hal::LedNumbers>(i));
        else
            hal::offLed(static_cast<hal::LedNumbers>(i));
    }
}

mpl::Led* mpl::Led::getInstance() {
    static mpl::Led instance;
    return &instance;
}
