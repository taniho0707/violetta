/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32c0xx_it.c
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

#include "stm32c0xx_it.h"

#include "hal_conf.h"
#include "hal_wallsensor.h"
#include "mll_wall_analyser_lazulisensor.h"
#include "stm32c011xx.h"
#include "stm32c0xx_ll_spi.h"

void NMI_Handler(void) {
    while (1) {}
}

void HardFault_Handler(void) {
    while (1) {}
}

void SVC_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void) {}

/******************************************************************************/
/* STM32C0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32c0xx.s).                    */
/******************************************************************************/

void DMA1_Channel1_IRQHandler(void) {}

void DMA1_Channel2_3_IRQHandler(void) {
    // static auto wallanalyser = mll::WallAnalyser::getInstance();

    // if (LL_I2C_IsActiveFlag_RXNE(SPI1)) {
    //     wallsensor->interruptI2cRxComplete();
    // } else if (LL_I2C_IsActiveFlag_STOP(SPI1)) {
    //     LL_I2C_ClearFlag_STOP(SPI1);
    //     wallsensor->interruptI2cTxComplete();
    // } else {
    //     NVIC_DisableIRQ(I2C2_EV_IRQn);
    //     NVIC_DisableIRQ(I2C2_ER_IRQn);
    //     // FIXME: Error handling
    // }
}

void SPI1_IRQHandler() {
    static auto wallanalyser = mll::WallAnalyser::getInstance();
    static hal::WallSensorNumbers sensor_number = hal::WallSensorNumbers::FRONTLEFT;

    if (LL_SPI_IsActiveFlag_RXNE(SPI1)) {
        // TODO: hal 層に移管する
        LL_SPI_ReceiveData16(SPI1);  // dummy read

        // バッファ内を送信し、次の Tx バッファに古いデータをセットする
        // uint16_t data = wallanalyser->getNextSensorBufferSingle(sensor_number);
        uint16_t data = wallanalyser->getNextSensorBufferAverage(sensor_number);

        uint16_t combined_data = (static_cast<uint16_t>(sensor_number) << 12) | data;
        hal::sendWallSensorDataSpiSync(combined_data);
        sensor_number = static_cast<hal::WallSensorNumbers>((static_cast<uint16_t>(sensor_number) + 1) % WALLSENSOR_NUMS);
    } else if (LL_SPI_IsActiveFlag_TXE(SPI1)) {
        // uint16_t data = wallanalyser->getNextSensorBufferSingle(sensor_number);
        // uint16_t combined_data = (static_cast<uint16_t>(sensor_number) << 12) | data;
        // hal::sendWallSensorDataSpiSync(combined_data);
    } else {
        NVIC_DisableIRQ(SPI1_IRQn);
    }
}
