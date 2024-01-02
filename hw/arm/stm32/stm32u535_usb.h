/*
 * STM32U535 USB
 *
 * Copyright (c) 2024 Olof Astrand
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

#ifndef HW_STM32U535_USB_H
#define HW_STM32U535_USB_H

#include "hw/sysbus.h"
#include "qom/object.h"


// https://community.st.com/t5/stm32-mcus-embedded-software/is-stm32-usb-device-library-portable-or-usable-for-stm32u5/td-p/169790/page/2

// https://github.com/STMicroelectronics/stm32u5-classic-coremw-apps




// Address offset: 0x00 + 0x4 * n, (n = 0 to 7)
#define USB_CHEPnR_START 0x0
#define USB_CHEPnR_END   0x1c

#define USB_CHEP0R_OFFSET 0x00
#define USB_CHEP1R_OFFSET 0x04
#define USB_CHEP2R_OFFSET 0x08
#define USB_CHEP3R_OFFSET 0x0C
#define USB_CHEP4R_OFFSET 0x10
#define USB_CHEP5R_OFFSET 0x14
#define USB_CHEP6R_OFFSET 0x18
#define USB_CHEP7R_OFFSET 0x1C

#define USB_CNTR_OFFSET 0x40
#define USB_ISTR_OFFSET 0x44
#define USB_FNR_OFFSET  0x48
#define USB_DADDR_OFFSET 0x4C
#define USB_USB_LPMCSR   0x54
#define USB_BCDR_OFFSET 0x58



// Located in RAM But can be treated as registers
// Channel/endpoint transmit buffer descriptor n (USB_CHEP_TXRXBD_n)
// Address offset: n*8
/*
    Bits 31:26 Reserved, must be kept at reset value.
    Bits 25:16 COUNT_TX[9:0]: Transmission byte count
    These bits contain the number of bytes to be transmitted by the endpoint/channel associated
    with the USB_CHEPnR register at the next IN token addressed to it.
    Bits 15:0 ADDR_TX[15:0]: Transmission buffer address
    These bits point to the starting address of the packet buffer containing data to be transmitted
    by the endpoint/channel associated with the USB_CHEPnR register at the next IN token
    addressed to it. Bits 1 and 0 must always be written as “00” since packet memory is word
    wide and all packet buffers must be word aligned.
*/

#define USB_CHEP_TXRXBD0_OFFSET 0x00
#define USB_CHEP_TXRXBD1_OFFSET 0x08
#define USB_CHEP_TXRXBD2_OFFSET 0x10


/*
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
*/

#define TYPE_STM32U535_USB "stm32u535-usb"
OBJECT_DECLARE_SIMPLE_TYPE(STM32U535USBState, STM32U535_USB)

struct STM32U535USBState {
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
    uint32_t BCDR;   // USB_BCDR

    // ... Other fields related to USB functionalities ...


    qemu_irq irq;
};

#endif /* HW_STM32U535_USB_H */
