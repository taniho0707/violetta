#include "taniho.h"

#ifdef STM32L4P5xx
// LL Library
// #include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_dmamux.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_i2c.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_utils.h"
#endif  // ifdef STM32L4P5xx

#ifdef STM32F411xE
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#endif  // ifdef STM32F411xE

#include "act_manager.h"

int main(void) {
#ifdef VIOLETTA
    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_TIM17_Init();
#endif  // ifdef VIOLETTA

#ifdef MOUSE_LAZULI
    SystemClock_Config();
    PeriphCommonClock_Config();
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_ZIRCONIA2KAI
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    SystemClock_Config();
#endif  // ifdef MOUSE_ZIRCONIA2KAI

    // auto activity = act::Manager(act::Activities::DEBUG);
    auto activity = act::Manager(act::Activities::SEARCH);
    // auto activity = act::Manager(act::Activities::WALLSENSOR_CHECK);
    activity.run();
}

#ifdef MOUSE_VIOLETTA
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5) {}
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    LL_PWR_EnableRange1BoostMode();
    LL_RCC_HSE_EnableBypass();
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1) {}
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_3, 30, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {}

    /* Intermediate AHB prescaler 2 when target frequency clock is higher than
     * 80 MHz */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}

    /* Insure 1��s transition state at intermediate medium speed clock based on
     * DWT*/
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    while (DWT->CYCCNT < 100);

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_SetSystemCoreClock(120000000);

    /* Update the time base */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
    LL_RCC_PLLSAI1_ConfigDomain_ADC(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLSAI1M_DIV_3, 8, LL_RCC_PLLSAI1R_DIV_2);
    LL_RCC_PLLSAI1_EnableDomain_ADC();
    LL_RCC_PLLSAI1_Enable();

    /* Wait till PLLSAI1 is ready */
    while (LL_RCC_PLLSAI1_IsReady() != 1) {}
}
#endif  // ifdef MOUSE_VIOLETTA

#ifdef MOUSE_LAZULI
void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {}
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    while (LL_PWR_IsActiveFlag_VOS() != 0) {}
    LL_PWR_EnableRange1BoostMode();
    LL_RCC_HSE_EnableBypass();
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1) {}
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_3, 25, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {}

    /* Intermediate AHB prescaler 2 when target frequency clock is higher than
     * 80 MHz */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}

    /* Insure 1us transition state at intermediate medium speed clock*/
    for (__IO uint32_t i = (120 >> 1); i != 0; i--);

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_SetSystemCoreClock(100000000);
    LL_Init1msTick(100000000);
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
    LL_RCC_PLLSAI1_ConfigDomain_ADC(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLSAI1M_DIV_3, 8, LL_RCC_PLLSAI1R_DIV_2);
    LL_RCC_PLLSAI1_EnableDomain_ADC();
    LL_RCC_PLLSAI1_Enable();

    /* Wait till PLLSAI1 is ready */
    while (LL_RCC_PLLSAI1_IsReady() != 1) {}
}
#endif  // ifdef MOUSE_LAZULI

#ifdef MOUSE_ZIRCONIA2KAI
/**
 * @brief System Clock Configuration
 * @retval None
 */
// void SystemClock_Config(void) {
//     LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
//     while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1) {
//     }
//     LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
//     LL_RCC_HSI_SetCalibTrimming(16);
//     LL_RCC_HSI_Enable();

//     /* Wait till HSI is ready */
//     while (LL_RCC_HSI_IsReady() != 1) {
//     }
//     LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 200,
//                                 LL_RCC_PLLP_DIV_8);
//     LL_RCC_PLL_Enable();

//     /* Wait till PLL is ready */
//     while (LL_RCC_PLL_IsReady() != 1) {
//     }
//     while (LL_PWR_IsActiveFlag_VOS() == 0) {
//     }
//     LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
//     LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
//     LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
//     LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

//     /* Wait till System clock is ready */
//     while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
//     }
//     LL_Init1msTick(50000000);
//     LL_SetSystemCoreClock(50000000);
//     LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
// } // 50MHz
void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_3) {}
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while (LL_RCC_HSI_IsReady() != 1) {}
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 100, LL_RCC_PLLP_DIV_2);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {}
    while (LL_PWR_IsActiveFlag_VOS() == 0) {}
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}
    LL_Init1msTick(100000000);
    LL_SetSystemCoreClock(100000000);
    LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}  // 100MHz
#endif  // ifdef MOUSE_ZIRCONIA2KAI

void Error_Handler(void) {
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1) {}
}