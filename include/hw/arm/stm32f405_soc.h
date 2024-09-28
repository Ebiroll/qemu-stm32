/*
 * STM32F405 SoC
 *
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
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

#ifndef HW_ARM_STM32F405_SOC_H
#define HW_ARM_STM32F405_SOC_H

#include "hw/misc/stm32f4xx_syscfg.h"
#include "hw/timer/stm32f2xx_timer.h"
#include "hw/char/stm32f2xx_usart.h"
#include "hw/adc/stm32f2xx_adc.h"
#include "hw/misc/stm32f4xx_exti.h"
#include "hw/or-irq.h"
#include "hw/ssi/stm32f2xx_spi.h"
#include "hw/arm/armv7m.h"
#include "qom/object.h"
#include "hw/arm/stm32/stm32fxxx_rcc.h"
#include "hw/arm/stm32/stm32f2xx_rtc.h"
#include "hw/arm/stm32/stm32fxxx_pwr.h"
#include "hw/arm/stm32/stm32fxxx_gpio.h"
#include "hw/arm/stm32/stm32f4xx_i2c.h"
//#include "hw/arm/asic_sim/nn1002.h"
#include "hw/ssi/ssi.h"

#define TYPE_ASIC_SSP "asic-ssp"
OBJECT_DECLARE_SIMPLE_TYPE(AsicSSPState, ASIC_SSP)

/* "Demux" the signal based on current chipselect */
struct AsicSSPState {
    SSIPeripheral ssidev;
    SSIBus *bus[3];
    uint32_t enable[3];
    qemu_irq gp_out[3];
};



#define TYPE_STM32F405_SOC "stm32f405-soc"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F405State, STM32F405_SOC)

#define STM_NUM_USARTS 7
#define STM_NUM_TIMERS 4
#define STM_NUM_ADCS 6
#define STM_NUM_SPIS 6
#define STM_NUM_GPIO 5

#define FLASH_BASE_ADDRESS 0x08000000
#define FLASH_SIZE (1024 * 1024)
#define SRAM_BASE_ADDRESS 0x20000000
#define SRAM_SIZE (128 * 1024)
#define CCM_BASE_ADDRESS 0x10000000
#define CCM_SIZE (64 * 1024)

struct STM32F405State {
    SysBusDevice parent_obj;

    ARMv7MState armv7m;

    STM32F4xxSyscfgState syscfg;
    STM32F4xxExtiState exti;
    STM32F2XXUsartState usart[STM_NUM_USARTS];
    STM32F2XXTimerState timer[STM_NUM_TIMERS];
    OrIRQState adc_irqs;
    STM32F2XXADCState adc[STM_NUM_ADCS];
    STM32F2XXSPIState spi[STM_NUM_SPIS];
    stm32fxxx_gpio gpio[STM_NUM_GPIO];
    STM32F4XXI2cState i2c;

    DeviceState   *mux;
    //SSIBus        *hSpi1;
    //NN1002State   asic[3];

    stm32fxxx_pwr     pwr;
    STM32FXXXRccState rcc;
    STM32F2XXRtcState rtc;

    MemoryRegion ccm;
    MemoryRegion sram;
    MemoryRegion flash;
    MemoryRegion flash_alias;

    Clock *sysclk;
    Clock *refclk;
};

#endif
