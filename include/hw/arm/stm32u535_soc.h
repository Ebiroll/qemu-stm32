/*
 * STM32U535 SoC
 *
 * Copyright (c) 2023 Olof Astrand
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef HW_ARM_STM32U535_SOC_H
#define HW_ARM_STM32U535_SOC_H

#include "hw/arm/stm32/stm32lxxx_syscfg.h"
#include "hw/timer/stm32f2xx_timer.h"
#include "hw/arm/stm32/stm32l552_usart.h"
#include "hw/arm/stm32/stm32l552_exti.h"
#include "hw/or-irq.h"
#include "hw/arm/stm32/stm32u5xx_spi.h"
#include "hw/arm/armv7m.h"
#include "qom/object.h"
#include "hw/arm/stm32/stm32u535_rcc.h"
#include "hw/arm/stm32/stm32l552_rtc.h"
#include "hw/arm/stm32/stm32u535_pwr.h"
#include "hw/arm/stm32/stm32fxxx_gpio.h"
#include "hw/arm/stm32/stm32l552_adc.h"
#include "hw/arm/stm32/stm32l552_flash.h"
#include "hw/arm/asic_sim/nn1002.h"
#include "hw/arm/stm32/stm32l552_dmamux.h"
#include "hw/arm/stm32/stm32u535_usb.h"


#define TYPE_STM32U535_SOC "stm32u535-soc"
OBJECT_DECLARE_SIMPLE_TYPE(STM32U535State, STM32U535_SOC)

#define STM32U535_NUM_USARTS 5
#define STM32U535_NUM_TIMERS 7
// Actually 2 ADC:s, but we emulate them as one
#define STM_NUM_ADCS 1 
#define STM_NUM_SPIS 3
#define STM_NUM_GPIO 8

#define FLASH_BASE_ADDRESS 0x08000000
#define FLASH_SIZE (512 * 1024)
#define SRAM_BASE_ADDRESS 0x20000000
#define SRAM_SIZE (512 * 1024)
#define CCM_BASE_ADDRESS 0x10000000
#define CCM_SIZE (274 * 1024)

struct STM32U535State {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    char *cpu_type;

    ARMv7MState armv7m;

    STM32lxxxSyscfgState syscfg;
    STM32L552ExtiState exti;
    STM32L552UsartState usart[STM32U535_NUM_USARTS];
    STM32L552UsartState lpuart1;
    STM32F2XXTimerState timer[STM32U535_NUM_TIMERS];
    OrIRQState adc_irqs;
    STM32L552ADCState adc;
    Stm32FlashRegs flash_regs;

    STM32U5XXSPIState spi[STM_NUM_SPIS];
    stm32fxxx_gpio gpio[STM_NUM_GPIO];

    NN1002State   asic[STM_NUM_SPIS];
    //DeviceState *asic1;

    //stm32fxxx_pwr     pwr;
    stm32u535_pwr     pwr;
    STM32u535RccState rcc;
    STM32L552RtcState rtc;
    STM32U535USBState usb;

    STM32L55DmaMuxState dmamux;

    MemoryRegion ccm;
    MemoryRegion sram;
    MemoryRegion flash;
    MemoryRegion flash_alias;

    Clock *sysclk;
    Clock *refclk;
};

#endif
