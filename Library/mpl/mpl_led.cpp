//******************************************************************************
// @addtogroup MPL
// @file       mpl_led.cpp
// @brief      LED点灯制御
//******************************************************************************
#include "mpl_led.h"

#include "hal_led.h"
#include "mpl_timer.h"

mpl::Led::Led() {
    i2c_state = mpl::I2cAsyncState::UNINITIALIZED;
}

mpl::MplStatus mpl::Led::initPort(hal::InitializeType type) {
    auto status = hal::initLedPort(type);
    if (status != hal::HalStatus::SUCCESS) {
        return mpl::MplStatus::ERROR;
    } else {
        if (type == hal::InitializeType::Async) {
            i2c_state = mpl::I2cAsyncState::IDLE;
        }
        return mpl::MplStatus::SUCCESS;
    }
}

void mpl::Led::deinitPort() {
    i2c_state = mpl::I2cAsyncState::UNINITIALIZED;
    hal::deinitLedPort();
}

bool mpl::Led::isFlicking(hal::LedNumbers num) {
    for (auto n : flick_params) {
        if (n.num == num) {
            if (n.start_time != 0) {
                return true;
            }
        }
    }
    return false;
}

void mpl::Led::on(hal::LedNumbers num) {
    hal::onLed(num);
}

void mpl::Led::off(hal::LedNumbers num) {
    hal::offLed(num);
}

void mpl::Led::flickSync(hal::LedNumbers num, float freq, uint16_t time) {
    flickAsync(num, freq, time);
    while (isFlicking(num));
}

void mpl::Led::flickAsync(hal::LedNumbers num, float freq, uint16_t time) {
    for (auto& n : flick_params) {
        if (n.start_time == 0) {  // 使われていない要素が見つかれば代入
            n.start_time = Timer::getMilliTime();
            n.freq = freq;
            n.time = time;
            return;
        }
    }
}

void mpl::Led::flickStop(hal::LedNumbers num) {
    for (auto& n : flick_params) {
        if (n.num == num) {
            n.start_time = 0;
        }
    }
    hal::offLed(num);
}

void mpl::Led::interruptPeriodic() {
    uint32_t t;
    uint32_t t2;
    for (auto& n : flick_params) {
        if (n.start_time == 0) continue;
        t2 = static_cast<uint32_t>(1000 / n.freq);
        t = (Timer::getMilliTime() - n.start_time) % t2;
        if ((n.time != 0) && ((Timer::getMilliTime() - n.start_time) > n.time)) {
            n.start_time = 0;
            hal::offLed(n.num);
            continue;
        }
        if (t > (t2 / 2)) hal::onLed(n.num);
        else hal::offLed(n.num);
    }
}

void mpl::Led::interruptI2cRxComplete() {}
void mpl::Led::interruptI2cTxComplete() {}
void mpl::Led::interruptDmaTxComplete() {}
void mpl::Led::interruptDmaTxError() {}

mpl::Led* mpl::Led::getInstance() {
    static mpl::Led instance;
    return &instance;
}
