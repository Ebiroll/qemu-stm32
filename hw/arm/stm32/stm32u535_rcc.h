#pragma once

#include "hw/sysbus.h"
#include "stm32fxxx_clktree.h"



#define TYPE_STM32U535_RCC "stm32u535-rcc"
OBJECT_DECLARE_SIMPLE_TYPE(STM32u535RccState, STM32U535_RCC)


enum {
    STM32_PERIPH_UNDEFINED = -1,
    STM32_RCC_PERIPH = 0,
    STM32_GPIOA,
    STM32_GPIOB,
    STM32_GPIOC,
    STM32_GPIOD,
    STM32_GPIOE,
    STM32_GPIOF,
    STM32_GPIOG,
    STM32_GPIOH,
    STM32_GPIOI,
    STM32_GPIOJ,
    STM32_GPIOK,
    STM32_SYSCFG,
    STM32_AFIO_PERIPH,
    STM32_UART1,
    STM32_UART2,
    STM32_UART3,
    STM32_UART4,
    STM32_UART5,
    STM32_UART6,
    STM32_UART7,
    STM32_UART8,
    STM32_ADC1,
    STM32_ADC2,
    STM32_ADC3,
    STM32_DAC,
    STM32_TIM1,
    STM32_TIM2,
    STM32_TIM3,
    STM32_TIM4,
    STM32_TIM5,
    STM32_TIM6,
    STM32_TIM7,
    STM32_TIM8,
    STM32_TIM9,
    STM32_TIM10,
    STM32_TIM11,
    STM32_TIM12,
    STM32_TIM13,
    STM32_TIM14,
    STM32_BKP,
    STM32_PWR,
    STM32_I2C1,
    STM32_I2C2,
    STM32_I2C3,
    STM32_I2C4,
    STM32_I2S2,
    STM32_I2S3,
    STM32_WWDG,
    STM32_CAN1,
    STM32_CAN2,
    STM32_CAN,
    STM32_USB,
    STM32_SPI1,
    STM32_SPI2,
    STM32_SPI3,
    STM32_EXTI_PERIPH,
    STM32_SDIO,
    STM32_FSMC,
    STM32_RTC,
    STM32_CRC,
    STM32_DMA1,
    STM32_DMA2,
    STM32_DCMI_PERIPH,
    STM32_CRYP_PERIPH,
    STM32_HASH_PERIPH,
    STM32_RNG_PERIPH,
    STM32_QSPI,
    STM32_LPTIM1,

    STM32_PERIPH_COUNT,
};

/** RCC Base data structure */
struct STM32u535RccState {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    uint32_t osc_freq;
    uint32_t osc32_freq;

    /* Private */
    MemoryRegion iomem;
    qemu_irq irq;

    /* Peripheral clocks */
    Clk PERIPHCLK[STM32_PERIPH_COUNT],
    HSICLK,
    HSECLK,
    MSICLK,
    CSCLK,
    LSECLK,
    LSICLK,
    SYSCLK,
    IWDGCLK,
    RTCCLK,

    PLLM, /* Applies "M" division and "N" multiplication factors for PLL */
    PLL2M,
    PLL3M,
    PLL1CLK,
    PLL2CLK, 
    PLL3CLK, 

    PLL48CLK,

    PLLI2SM, /* Applies "M" division and "N" multiplication factors for PLLI2S */
    PLLI2SCLK,
    
    HCLK, /* Output from AHB Prescaler */
    PCLK1, /* Output from APB1 Prescaler */
    PCLK2; /* Output from APB2 Prescaler */

    /* Register Values */
    uint32_t
    RCC_CR,  // RCC_ICSR,
    RCC_PLLCFGR,
    RCC_PLLSAI2CFGR,
    RCC_PLLSAI1CFGR,
    RCC_CIER,
    RCC_CIFR,
    RCC_CICR,
    RCC_APB1ENR,
    RCC_APB1ENR2,
    RCC_APB2ENR,
    RCC_AHB1SMENR,
    RCC_AHB2SMENR,
    RCC_AHB3SMENR,
    RCC_APB1SMENR1,
    RCC_APB1SMENR2,
    RCC_APB2SMENR,
    RCC_CCIPR,
    RCC_BDCR,
    RCC_CSR,
    RCC_CRRCR,
    RCC_CCIPR2,
    RCC_SECCFGR,
    RCC_SECSR,
    RCC_AHB1SECSR,
    RCC_AHB2SECSR,
    RCC_AHB3SECSR,
    RCC_APB1SECSR1,
    RCC_APB1SECSR2,
    RCC_APB2SECSR;

    /* Register Field Values */
    uint32_t
    RCC_CFGR_PPRE1,
    RCC_CFGR_PPRE2,
    RCC_CFGR_HPRE,
    RCC_AHB1ENR,
    RCC_AHB2ENR,
    RCC_AHB3ENR,
    RCC_CFGR_SW,
    RCC_CFGR1,
    RCC_CFGR2,
    RCC_CFGR3,

    RCC_PLLI2SCFGR;

    uint8_t
    RCC_PLLCFGR_PLLM,
    RCC_PLLCFGR_PLLP,
    RCC_PLLCFGR_PLLSRC;

    uint16_t
    RCC_PLLCFGR_PLLN;

    uint8_t
    RCC_PLLI2SCFGR_PLLR,
    RCC_PLLI2SCFGR_PLLQ;

    uint16_t
    RCC_PLLI2SCFGR_PLLN;
};
