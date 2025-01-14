/*
 * STM32L552 EXTI
 *
 * Copyright (c) 2023  Olof Åstrand
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

#ifndef HW_STM32L552_EXTI_H
#define HW_STM32L552_EXTI_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define EXTI_RTSR1   0x00
#define EXTI_FTSR1   0x04
//#define EXTI_EMR     0x04
#define EXTI_SWIER1  0x08
#define EXTI_RPR1    0x0C
#define EXTI_FPR1    0x10
#define EXTI_PR      0x14
#define EXTI_IMR1    0x80
// External interrupt selection register 1
#define EXTICR3      0x68

#define TYPE_STM32L552_EXTI "stm32l552-exti"
OBJECT_DECLARE_SIMPLE_TYPE(STM32L552ExtiState, STM32L552_EXTI)

#define NUM_GPIO_EVENT_IN_LINES 28
#define NUM_INTERRUPT_OUT_LINES 28
#define STM32_MAX_IRQ  32

struct STM32L552ExtiState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    uint32_t exti_rtsr1;
    uint32_t exti_imr1;
    //uint32_t exti_emr;

    uint32_t exti_rpr1;
    uint32_t exti_fpr1;

    uint32_t exti_ftsr1;
    uint32_t exti_swier1;
    uint32_t exti_pr;

    qemu_irq irq[NUM_INTERRUPT_OUT_LINES];
};

#endif
