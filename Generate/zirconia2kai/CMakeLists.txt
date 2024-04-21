cmake_minimum_required(VERSION 3.10)

include_directories(
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
)

add_library(STM32HAL STATIC
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_exti.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_spi.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_tim.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usart.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_utils.c
)
