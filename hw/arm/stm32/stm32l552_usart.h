/*
 * STM32L552 USART
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

#ifndef HW_STM32L552_USART_H
#define HW_STM32L552_USART_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

// Common status register   CSR
#define USART_CR1  0x00
#define USART_CR2  0x04
#define USART_CR3  0x08
#define USART_BRR  0x0C
#define USART_GTPR 0x10
#define USART_RTOR 0x14
#define USART_RQR  0x18
#define USART_ISR  0x1C
#define USART_ICR  0x20
// RDR
#define USART_RDR     0x24
// TDR
#define USART_DR   0x28

#define USART_PRESC   0x2C
// FIXME
#define USART_SR   0x30


/*
 * NB: The reset value mentioned in "24.6.1 Status register" seems bogus.
 * Looking at "Table 98 USART register map and reset values", it seems it
 * should be 0xc0, and that's how real hardware behaves.
 */
#define USART_SR_RESET (USART_SR_TXE | USART_SR_TC)

#define USART_SR_TXE  (1 << 7)
#define USART_SR_TC   (1 << 6)
#define USART_SR_RXNE (1 << 5)
// Mute mode
#define UART_SR_MM    (1 << 13)

#define USART_ISR_RXNE (1 << 5)
#define USART_ISR_TEACK (1 << 21)
#define USART_ISR_REACK (1 << 22)


#define USART_CR1_UE  (1 << 0)
#define USART_CR1_RXNEIE  (1 << 5)
#define USART_CR1_TE  (1 << 3)
// Receiver eanable
#define USART_CR1_RE  (1 << 2)

#define TYPE_STM32L552_USART "stm32l552-usart"
OBJECT_DECLARE_SIMPLE_TYPE(STM32L552UsartState, STM32L552_USART)

struct STM32L552UsartState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    uint32_t usart_sr;
    uint32_t usart_dr;
    uint32_t usart_rdr;
     uint32_t usart_isr;
    uint32_t usart_brr;
    uint32_t usart_cr1;
    uint32_t usart_cr2;
    uint32_t usart_cr3;
    uint32_t usart_gtpr;
    uint32_t usart_presc;

    CharBackend chr;
    qemu_irq irq;
};
#endif /* HW_STM32L552_USART_H */
