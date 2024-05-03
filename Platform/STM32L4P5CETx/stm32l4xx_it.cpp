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
#include "stm32l4xx_ll_usart.h"

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles Flash global interrupt.
 */
void FLASH_IRQHandler(void) {}

/**
 * @brief This function handles DMA1 channel1 global interrupt.
 * DMA1 Channel1 is used for USART1 RX
 */
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

/**
 * @brief This function handles DMA1 channel2 global interrupt.
 * DMA1 Channel2 is used for USART1 TX
 */
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

/**
 * @brief This function handles DMA1 channel3 global interrupt.
 */
void DMA1_Channel3_IRQHandler(void) {}

/**
 * @brief This function handles DMA1 channel4 global interrupt.
 */
void DMA1_Channel4_IRQHandler(void) {}

/**
 * @brief This function handles DMA1 channel5 global interrupt.
 */
void DMA1_Channel5_IRQHandler(void) {}

/**
 * @brief This function handles DMA1 channel6 global interrupt.
 */
void DMA1_Channel6_IRQHandler(void) {}

/**
 * @brief This function handles DMA1 channel7 global interrupt.
 */
void DMA1_Channel7_IRQHandler(void) {}

/**
 * @brief This function handles ADC1 and ADC2 global interrupt.
 */
void ADC1_2_IRQHandler(void) {}

/**
 * @brief This function handles I2C1 event interrupt.
 */
void I2C1_EV_IRQHandler(void) {
    static auto led = mpl::Led::getInstance();
    if (LL_I2C_IsActiveFlag_RXNE(I2C1)) {
        led->interruptI2cRxComplete();
    } else if (LL_I2C_IsActiveFlag_STOP(I2C1)) {
        LL_I2C_ClearFlag_STOP(I2C1);
        led->interruptI2cTxComplete();
    } else {
        NVIC_DisableIRQ(I2C1_EV_IRQn);
        NVIC_DisableIRQ(I2C1_ER_IRQn);
        // FIXME: Error handling
    }
}

/**
 * @brief This function handles I2C1 error interrupt.
 */
void I2C1_ER_IRQHandler(void) {
    NVIC_DisableIRQ(I2C1_EV_IRQn);
    NVIC_DisableIRQ(I2C1_ER_IRQn);
    // FIXME: Error handling
}

/**
 * @brief This function handles SPI2 global interrupt.
 */
void SPI2_IRQHandler(void) {}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void) {
    if (LL_USART_IsActiveFlag_TC(USART1)) {
        LL_USART_ClearFlag_TC(USART1);
    }
}

/**
 * @brief This function handles TIM5 global interrupt.
 */
void TIM5_IRQHandler(void) {
    if (LL_TIM_IsActiveFlag_UPDATE(TIM5)) {
        LL_TIM_ClearFlag_UPDATE(TIM5);
        mpl::Timer::interrupt();
    }
}

/**
 * @brief This function handles SPI3 global interrupt.
 */
void SPI3_IRQHandler(void) {}

/**
 * @brief This function handles DMA2 channel1 global interrupt.
 * DMA2 Channel1 is used for I2C1 TX
 */
void DMA2_Channel1_IRQHandler(void) {
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

/**
 * @brief This function handles DMA2 channel2 global interrupt.
 */
void DMA2_Channel2_IRQHandler(void) {}

/**
 * @brief This function handles FPU global interrupt.
 */
void FPU_IRQHandler(void) {}
