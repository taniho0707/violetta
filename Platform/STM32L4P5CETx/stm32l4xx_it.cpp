/**
 ******************************************************************************
 * @file    stm32l4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "stm32l4xx_it.h"

#include "mpl_debug.h"
#include "mpl_led.h"
#include "mpl_timer.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_i2c.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_usart.h"

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

void NMI_Handler(void) {
    while (1) {}
}

void HardFault_Handler(void) {
    while (1) {}
}

void MemManage_Handler(void) {
    while (1) {}
}

void BusFault_Handler(void) {
    while (1) {}
}

void UsageFault_Handler(void) {
    while (1) {}
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void) {}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

void FLASH_IRQHandler(void) {}

void DMA1_Channel1_IRQHandler(void) {
    static auto debug = mpl::Debug::getInstance();
    if (LL_DMA_IsActiveFlag_TC1(DMA1)) {
        LL_DMA_ClearFlag_GI1(DMA1);
        /* Call function Transmission complete Callback */
        debug->interruptRxComplete();
    } else if (LL_DMA_IsActiveFlag_TE1(DMA1)) {
        /* Call Error function */
        debug->interruptRxError();
    }
}

void DMA1_Channel2_IRQHandler(void) {
    static auto debug = mpl::Debug::getInstance();
    if (LL_DMA_IsActiveFlag_TC2(DMA1)) {
        LL_DMA_ClearFlag_GI2(DMA1);
        // LL_DMA_ClearFlag_TC2(DMA1);
        /* Call function Transmission complete Callback */
        debug->interruptTxComplete();
    } else if (LL_DMA_IsActiveFlag_TE2(DMA1)) {
        /* Call Error function */
        debug->interruptTxError();
    }
}

void DMA1_Channel3_IRQHandler(void) {}

void DMA1_Channel4_IRQHandler(void) {}

void DMA1_Channel5_IRQHandler(void) {}

void DMA1_Channel6_IRQHandler(void) {}

void DMA1_Channel7_IRQHandler(void) {}

void ADC1_2_IRQHandler(void) {}

void TIM1_TRG_COM_TIM17_IRQHandler(void) {}

void TIM2_IRQHandler(void) {}

void TIM3_IRQHandler(void) {}

void TIM4_IRQHandler(void) {}

void I2C2_EV_IRQHandler(void) {
    static auto led = mpl::Led::getInstance();
    if (LL_I2C_IsActiveFlag_RXNE(I2C2)) {
        led->interruptI2cRxComplete();
    } else if (LL_I2C_IsActiveFlag_STOP(I2C2)) {
        LL_I2C_ClearFlag_STOP(I2C2);
        led->interruptI2cTxComplete();
    } else {
        NVIC_DisableIRQ(I2C2_EV_IRQn);
        NVIC_DisableIRQ(I2C2_ER_IRQn);
        // FIXME: Error handling
    }
}

void I2C2_ER_IRQHandler(void) {
    NVIC_DisableIRQ(I2C2_EV_IRQn);
    NVIC_DisableIRQ(I2C2_ER_IRQn);
    // FIXME: Error handling
}

void SPI1_IRQHandler(void) {}

void SPI2_IRQHandler(void) {}

void USART1_IRQHandler(void) {
    if (LL_USART_IsActiveFlag_TC(USART1)) {
        LL_USART_ClearFlag_TC(USART1);
    }
}

void TIM5_IRQHandler(void) {
    if (LL_TIM_IsActiveFlag_UPDATE(TIM5)) {
        LL_TIM_ClearFlag_UPDATE(TIM5);
        mpl::Timer::interrupt();
    }
}

void SPI3_IRQHandler(void) {}

void DMA2_Channel1_IRQHandler(void) {}

void DMA2_Channel2_IRQHandler(void) {}

void DMA2_Channel3_IRQHandler(void) {
    static auto led = mpl::Led::getInstance();
    if (LL_DMA_IsActiveFlag_TC1(DMA2)) {
        LL_DMA_ClearFlag_GI1(DMA2);
        /* Call function Transmission complete Callback */
        led->interruptDmaTxComplete();
    } else if (LL_DMA_IsActiveFlag_TE1(DMA2)) {
        /* Call Error function */
        led->interruptDmaTxError();
    }
}

void DMA2_Channel4_IRQHandler(void) {}
