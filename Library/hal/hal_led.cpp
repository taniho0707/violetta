//******************************************************************************
// @addtogroup HAL
// @file       hal_led.cpp
// @brief      LED点灯制御
//******************************************************************************

#include "hal_led.h"

hal::HalStatus hal::initLedPort(hal::LedNumbers num) {
#ifdef MOUSE_VIOLETTA
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
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    // LED0: BLUE   PA12
    // LED1: GREEN  PA11
    // LED2: YELLOW PH0
    // LED3: RED    PH1

    // FIXME: Implement Led Initialization for Lazulil

    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#ifdef STM32F411xE
    // LED0: BLUE   PA12
    // LED1: GREEN  PA11
    // LED2: YELLOW PH0
    // LED3: RED    PH1

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    ErrorStatus status;

    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    LL_GPIO_ResetOutputPin(GPIOH, LED2_Pin | LED3_Pin);
    LL_GPIO_ResetOutputPin(GPIOA, LED1_Pin | LED0_Pin);

    GPIO_InitStruct.Pin = LED2_Pin | LED3_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    status = LL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    if (status != ErrorStatus::SUCCESS) return hal::HalStatus::ERROR;

    GPIO_InitStruct.Pin = LED1_Pin | LED0_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    status = LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    if (status != ErrorStatus::SUCCESS) return hal::HalStatus::ERROR;

    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return HalStatus::SUCCESS;
#endif
}

hal::HalStatus hal::deinitLedPort(hal::LedNumbers num) {
#ifdef MOUSE_VIOLETTA
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
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#ifdef STM32F411xE
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif
}

hal::HalStatus hal::onLed(hal::LedNumbers num) {
#ifdef MOUSE_VIOLETTA
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
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#ifdef STM32F411xE
    switch (num) {
        case hal::LedNumbers::RED:
            LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin);
            break;
        case hal::LedNumbers::YELLOW:
            LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
            break;
        case hal::LedNumbers::GREEN:
            LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
            break;
        case hal::LedNumbers::BLUE:
            LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
            break;
        case hal::LedNumbers::ALL:
            hal::onLed(hal::LedNumbers::RED);
            hal::onLed(hal::LedNumbers::YELLOW);
            hal::onLed(hal::LedNumbers::GREEN);
            hal::onLed(hal::LedNumbers::BLUE);
            break;
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif
}

hal::HalStatus hal::offLed(hal::LedNumbers num) {
#ifdef MOUSE_VIOLETTA
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
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
    return hal::HalStatus::NOIMPLEMENT;
#endif  // ifdef MOUSE_LAZULI

#ifdef STM32F411xE
    switch (num) {
        case hal::LedNumbers::RED:
            LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
            break;
        case hal::LedNumbers::YELLOW:
            LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
            break;
        case hal::LedNumbers::GREEN:
            LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
            break;
        case hal::LedNumbers::BLUE:
            LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
            break;
        case hal::LedNumbers::ALL:
            hal::offLed(hal::LedNumbers::RED);
            hal::offLed(hal::LedNumbers::YELLOW);
            hal::offLed(hal::LedNumbers::GREEN);
            hal::offLed(hal::LedNumbers::BLUE);
            break;
    }
    return hal::HalStatus::SUCCESS;
#endif  // ifdef STM32F411xE

#ifdef LINUX
    return hal::HalStatus::SUCCESS;
#endif
}
