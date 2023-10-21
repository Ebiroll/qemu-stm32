/*
 * STM32L552 USB
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

#ifndef HW_STM32L552_USB_H
#define HW_STM32L552_USB_H

#include "hw/sysbus.h"
#include "qom/object.h"


#define USB_EP0R_OFFSET    0x00
#define USB_EP1R_OFFSET    0x04
#define USB_EP2R_OFFSET    0x08
#define USB_EP3R_OFFSET    0x0C
#define USB_EP4R_OFFSET    0x10
#define USB_EP5R_OFFSET    0x14
#define USB_EP6R_OFFSET    0x18
#define USB_EP7R_OFFSET    0x1C
#define USB_CNTR_OFFSET    0x40
#define USB_ISTR_OFFSET    0x44
#define USB_FNR_OFFSET     0x48
#define USB_DADDR_OFFSET   0x4C


#define TYPE_STM32L552_USB "stm32l552-usb"
OBJECT_DECLARE_SIMPLE_TYPE(STM32L552USBState, STM32L552_USB)

struct STM32L552USBState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion iomem;

    // Endpoint registers
    uint32_t EP[8];  // USB_EP0R to USB_EP7R

    // Control and status registers
    uint32_t CNTR;   // USB_CNTR
    uint32_t ISTR;   // USB_ISTR
    uint32_t FNR;    // USB_FNR
    uint32_t DADDR;  // USB_DADDR

    // ... Other fields related to USB functionalities ...


    qemu_irq irq;
};

#endif /* HW_STM32L552_USB_H */
