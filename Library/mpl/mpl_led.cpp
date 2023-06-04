//******************************************************************************
// @addtogroup MPL
// @file       mpl_led.cpp
// @brief      LED点灯制御
//******************************************************************************
#include "mpl_led.h"

#include "mpl_timer.h"

Led::Led() {
#ifdef STM32L4P5xx
    gpio_port = GPIOB;
    gpio_channel = GPIO_PIN_8;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    LL_GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_8;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    LL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_3;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    LL_GPIO_Init(GPIOH, &GPIO_InitStructure);
#endif // ifdef STM32L4P5xx
}

void Led::initPort(LedNumbers num) {
#ifdef STM32L4P5xx
    LL_GPIO_InitTypeDef GPIO_InitStructure;
    switch (num) {
        case LedNumbers::FRONT1:
            break;
        case LedNumbers::FRONT2:
            break;
        case LedNumbers::TOP1:
            GPIO_InitStructure.Pin = GPIO_PIN_10;
            GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStructure.Pull = GPIO_NOPULL;
            GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
            LL_GPIO_Init(GPIOA, &GPIO_InitStructure);
            break;
        case LedNumbers::TOP2:
            GPIO_InitStructure.Pin = GPIO_PIN_9;
            GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStructure.Pull = GPIO_NOPULL;
            GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
            LL_GPIO_Init(GPIOA, &GPIO_InitStructure);
            break;
    }
#endif // ifdef STM32L4P5xx
}

void Led::deinitPort(LedNumbers num) {
#ifdef STM32L4P5xx
    switch (num) {
        case LedNumbers::FRONT1:
            break;
        case LedNumbers::FRONT2:
            break;
        case LedNumbers::TOP1:
            HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10);
            break;
        case LedNumbers::TOP2:
            HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);
            break;
    }
#endif // ifdef STM32L4P5xx
}

void Led::setType(LedNumbers num) {
#ifdef STM32L4P5xx
    switch (num) {
        case LedNumbers::FRONT1:
            gpio_port = GPIOB;
            gpio_channel = GPIO_PIN_8;
            break;
        case LedNumbers::FRONT2:
            gpio_port = GPIOH;
            gpio_channel = GPIO_PIN_3;
            break;
        case LedNumbers::TOP1:
            gpio_port = GPIOA;
            gpio_channel = GPIO_PIN_10;
            break;
        case LedNumbers::TOP2:
            gpio_port = GPIOA;
            gpio_channel = GPIO_PIN_9;
            break;
    }
#endif // ifdef STM32L4P5xx
}

void Led::on(LedNumbers num) {
    setType(num);

#ifdef STM32L4P5xx
    HAL_GPIO_WritePin(gpio_port, gpio_channel, GPIO_PIN_SET);
#endif // ifdef STM32L4P5xx
}

void Led::off(LedNumbers num) {
    setType(num);

#ifdef STM32L4P5xx
    HAL_GPIO_WritePin(gpio_port, gpio_channel, GPIO_PIN_RESET);
#endif // ifdef STM32L4P5xx
}

bool Led::isFlicking(LedNumbers num) {
    if (flick_params.at(static_cast<uint8_t>(num)).start_time == 0)
        return false;
    else
        return true;
}

void Led::flickSync(LedNumbers num, float freq, uint16_t time) {
    flickAsync(num, freq, time);
    while (isFlicking(num))
        ;
}

void Led::flickAsync(LedNumbers num, float freq, uint16_t time) {
    flick_params.at(static_cast<uint8_t>(num)).start_time =
        Timer::getMilliTime();
    flick_params.at(static_cast<uint8_t>(num)).freq = freq;
    flick_params.at(static_cast<uint8_t>(num)).time = time;
}

void Led::flickStop(LedNumbers num) {
    flick_params.at(static_cast<uint8_t>(num)).start_time = 0;
    off(num);
}

void Led::interrupt() {
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
            off(static_cast<LedNumbers>(i));
            continue;
        }
        if (t > (t2 / 2))
            on(static_cast<LedNumbers>(i));
        else
            off(static_cast<LedNumbers>(i));
    }
}

Led* Led::getInstance() {
    static Led instance;
    return &instance;
}
