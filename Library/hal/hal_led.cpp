//******************************************************************************
// @addtogroup HAL
// @file       hal_led.cpp
// @brief      LED点灯制御
//******************************************************************************

#include "hal_led.h"

hal::HalStatus hal::initLedPort(hal::LedNumbers num) {
#ifdef STM32L4P5xx
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_channel;

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

    switch (num) {
        case hal::LedNumbers::FRONT1:
            break;
        case hal::LedNumbers::FRONT2:
            break;
        case hal::LedNumbers::TOP1:
            GPIO_InitStructure.Pin = GPIO_PIN_10;
            GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStructure.Pull = GPIO_NOPULL;
            GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
            LL_GPIO_Init(GPIOA, &GPIO_InitStructure);
            break;
        case hal::LedNumbers::TOP2:
            GPIO_InitStructure.Pin = GPIO_PIN_9;
            GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStructure.Pull = GPIO_NOPULL;
            GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
            LL_GPIO_Init(GPIOA, &GPIO_InitStructure);
            break;
    }
    return HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef LINUX
    return HalStatus::SUCCESS;
#endif
}

hal::HalStatus hal::deinitLedPort(hal::LedNumbers num) {
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
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif
}

hal::HalStatus hal::onLed(hal::LedNumbers num) {
#ifdef STM32L4P5xx
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_channel;
    switch (num) {
        case hal::LedNumbers::FRONT1:
            gpio_port = GPIOB;
            gpio_channel = GPIO_PIN_8;
            break;
        case hal::LedNumbers::FRONT2:
            gpio_port = GPIOH;
            gpio_channel = GPIO_PIN_3;
            break;
        case hal::LedNumbers::TOP1:
            gpio_port = GPIOA;
            gpio_channel = GPIO_PIN_10;
            break;
        case hal::LedNumbers::TOP2:
            gpio_port = GPIOA;
            gpio_channel = GPIO_PIN_9;
            break;
    }
    HAL_GPIO_WritePin(gpio_port, gpio_channel, GPIO_PIN_SET);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif
}

hal::HalStatus hal::offLed(hal::LedNumbers num) {
#ifdef STM32L4P5xx
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_channel;
    switch (num) {
        case hal::LedNumbers::FRONT1:
            gpio_port = GPIOB;
            gpio_channel = GPIO_PIN_8;
            break;
        case hal::LedNumbers::FRONT2:
            gpio_port = GPIOH;
            gpio_channel = GPIO_PIN_3;
            break;
        case hal::LedNumbers::TOP1:
            gpio_port = GPIOA;
            gpio_channel = GPIO_PIN_10;
            break;
        case hal::LedNumbers::TOP2:
            gpio_port = GPIOA;
            gpio_channel = GPIO_PIN_9;
            break;
    }
    HAL_GPIO_WritePin(gpio_port, gpio_channel, GPIO_PIN_RESET);
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32L4P5xx

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif
}
