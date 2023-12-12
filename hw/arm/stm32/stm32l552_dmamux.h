/*
 * STM32L552 DMAMUX
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
#ifndef DMAMUX_H
#define DMAMUX_H


#include "hw/sysbus.h"
#include "qom/object.h"

#define MUX_CCR    0x00

#define RG0CR_OFFSET 0x100
#define RG1CR_OFFSET 0x104
#define RG2CR_OFFSET 0x108
#define RG3CR_OFFSET 0x10C
#define RG4CR_OFFSET 0x110
#define RG5CR_OFFSET 0x114
#define RG6CR_OFFSET 0x118
#define RG7CR_OFFSET 0x11C
#define RG8CR_OFFSET 0x120
#define RG9CR_OFFSET 0x124
#define RG10CR_OFFSET 0x128
#define RG11CR_OFFSET 0x12C
#define RG12CR_OFFSET 0x130
#define RG13CR_OFFSET 0x134
#define RG14CR_OFFSET 0x138
#define RG15CR_OFFSET 0x13C


// RG0CR bit fields
#define RG0CR_GNBREQ_MASK   (0x1F << 19) // Number of Request
#define RG0CR_GPOL_MASK     (0x3 << 17)  // Generation Polarity
#define RG0CR_GE_MASK       (0x1 << 16)  // Generation Enable
#define RG0CR_OIE_MASK      (0x1 << 8)   // Overrun Interrupt Enable
#define RG0CR_SIG_ID_MASK   (0x1F)       // Signal ID

#define GCR_COUNT 16


#define TYPE_STM32L552_DMAMUX "stm32l552-dmamux"
OBJECT_DECLARE_SIMPLE_TYPE(STM32L55DmaMuxState, STM32L552_DMAMUX)

struct STM32L55DmaMuxState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    uint32_t ccr;

    uint32_t gcr[GCR_COUNT];           // Array of GCR registers
    qemu_irq irq;
};

#endif /* DMAMUX_H */
