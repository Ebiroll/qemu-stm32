/*
 * STM32L552 USART
 *
 * Copyright (c) 2023 Olof Astrand
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

#include "qemu/osdep.h"
#include "hw/arm/stm32/stm32l552_usart.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "qemu/log.h"
#include "qemu/module.h"

//#define STM_USART_ERR_DEBUG 5
#ifndef STM_USART_ERR_DEBUG
#define STM_USART_ERR_DEBUG 0
#endif

#define DB_PRINT_L(lvl, fmt, args...) do { \
    if (STM_USART_ERR_DEBUG >= lvl) { \
        qemu_log("%s: " fmt, __func__, ## args); \
    } \
} while (0)

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

static int stm32l552_usart_can_receive(void *opaque)
{
    STM32L552UsartState *s = opaque;

    if (!(s->usart_sr & USART_SR_RXNE)) {
        return 1;
    }

    return 0;
}

static void stm32l552_usart_receive(void *opaque, const uint8_t *buf, int size)
{
    STM32L552UsartState *s = opaque;

    if (!((s->usart_cr1 & USART_CR1_UE) && (s->usart_cr1 & USART_CR1_RE))) {
        /* USART not enabled - drop the chars */
        DB_PRINT("Not enabled, Dropping the chars\n");
       return;
    }

    s->usart_rdr = *buf;
    s->usart_dr = *buf;
    s->usart_isr |= USART_ISR_RXNE;
    s->usart_sr |= USART_SR_RXNE;

    //if (qemu_irq_is_connected(s->irq)) {
    //    DB_PRINT("IRQ is connected: %d\n", 1);
    //    qemu_set_irq(s->irq, 1);
    //}

    if (s->usart_cr2 & USART_CR1_RXNEIE) {
        DB_PRINT("Setting irq: %d\n", 2);
        qemu_set_irq(s->irq, 1);
    }

    DB_PRINT("Receiving: %c\n", s->usart_dr);
}

static void stm32l552_usart_reset(DeviceState *dev)
{
    STM32L552UsartState *s = STM32L552_USART(dev);

    s->usart_sr = USART_SR_RESET;
    s->usart_isr = 0x00C0;
    s->usart_presc = 0x00C0;
    s->usart_rdr = 0x00000000;
    s->usart_dr = 0x00000000;
    s->usart_brr = 0x00000000;
    s->usart_cr1 = 0x00000000;
    s->usart_cr2 = 0x00000000;
    s->usart_cr3 = 0x00000000;
    s->usart_gtpr = 0x00000000;

    qemu_set_irq(s->irq, 0);
}

static uint64_t stm32l552_usart_read(void *opaque, hwaddr addr,
                                       unsigned int size)
{
    STM32L552UsartState *s = opaque;
    uint64_t retvalue;

    DB_PRINT("Read 0x%"HWADDR_PRIx"\n", addr);

    switch (addr) {
    case USART_ISR:
        retvalue = s->usart_isr;
        retvalue = retvalue | USART_ISR_TEACK | USART_ISR_REACK;
        return retvalue;
    case USART_RDR:
        retvalue = s->usart_rdr & 0x3FF;
        s->usart_sr &= ~USART_SR_RXNE;
        qemu_chr_fe_accept_input(&s->chr);
        qemu_set_irq(s->irq, 0);
        return retvalue;
    case USART_SR:
        retvalue = s->usart_sr;
        qemu_chr_fe_accept_input(&s->chr);
        return retvalue;
    case USART_DR:
        DB_PRINT("Value: 0x%" PRIx32 ", %c\n", s->usart_dr, (char) s->usart_dr);
        retvalue = s->usart_dr & 0x3FF;
        s->usart_sr &= ~USART_SR_RXNE;
        qemu_chr_fe_accept_input(&s->chr);
        qemu_set_irq(s->irq, 0);
        return retvalue;
    case USART_BRR:
        return s->usart_brr;
    case USART_CR1:
        return s->usart_cr1;
    case USART_CR2:
        return s->usart_cr2;
    case USART_CR3:
        return s->usart_cr3;
    case USART_GTPR:
        return s->usart_gtpr;
    case USART_PRESC:
        return s->usart_presc;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
        return 0;
    }

    return 0;
}

static void stm32l552_usart_write(void *opaque, hwaddr addr,
                                  uint64_t val64, unsigned int size)
{
    STM32L552UsartState *s = opaque;
    uint32_t value = val64;
    unsigned char ch;

    DB_PRINT("Write 0x%" PRIx32 ", 0x%"HWADDR_PRIx"\n", value, addr);

    switch (addr) {
    case USART_SR:
        if (value <= 0x3FF) {
            /* I/O being synchronous, TXE is always set. In addition, it may
               only be set by hardware, so keep it set here. */
            s->usart_sr = value | USART_SR_TXE ;
        } else {
            s->usart_sr &= value;
        }
        if (!(s->usart_sr & USART_SR_RXNE)) {
            qemu_set_irq(s->irq, 0);
        }
        return;
    case USART_ISR:
        s->usart_isr = value;
        return;
    case USART_PRESC:
        s->usart_presc = value;
        return;
    case USART_DR:
        if (value < 0xF000) {
            ch = value;
            /* XXX this blocks entire thread. Rewrite to use
             * qemu_chr_fe_write and background I/O callbacks */
            qemu_chr_fe_write_all(&s->chr, &ch, 1);
            /* XXX I/O are currently synchronous, making it impossible for
               software to observe transient states where TXE or TC aren't
               set. Unlike TXE however, which is read-only, software may
               clear TC by writing 0 to the SR register, so set it again
               on each write. */
            s->usart_sr |= USART_SR_TC;
        }
        return;
    case USART_BRR:
        s->usart_brr = value;
        return;
    case USART_CR1:
        s->usart_cr1 = value;
        //  &&
        //        s->usart_sr & USART_SR_RXNE
            if (s->usart_cr1 & USART_CR1_RXNEIE) {
                qemu_set_irq(s->irq, 1);
            }
        return;
    case USART_CR2:
        s->usart_cr2 = value;
        return;
    case USART_CR3:
        s->usart_cr3 = value;
        return;
    case USART_GTPR:
        s->usart_gtpr = value;
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
    }
}

static const MemoryRegionOps stm32l552_usart_ops = {
    .read = stm32l552_usart_read,
    .write = stm32l552_usart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static Property stm32l552_usart_properties[] = {
    DEFINE_PROP_CHR("chardev", STM32L552UsartState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32l552_usart_init(Object *obj)
{
    STM32L552UsartState *s = STM32L552_USART(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    memory_region_init_io(&s->mmio, obj, &stm32l552_usart_ops, s,
                          TYPE_STM32L552_USART, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void stm32l552_usart_realize(DeviceState *dev, Error **errp)
{
    STM32L552UsartState *s = STM32L552_USART(dev);

    qemu_chr_fe_set_handlers(&s->chr, stm32l552_usart_can_receive,
                             stm32l552_usart_receive, NULL, NULL,
                             s, NULL, true);
}

static void stm32l552_usart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32l552_usart_reset;
    device_class_set_props(dc, stm32l552_usart_properties);
    dc->realize = stm32l552_usart_realize;
}

static const TypeInfo stm32l552_usart_info = {
    .name          = TYPE_STM32L552_USART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32L552UsartState),
    .instance_init = stm32l552_usart_init,
    .class_init    = stm32l552_usart_class_init,
};

static void stm32l552_usart_register_types(void)
{
    type_register_static(&stm32l552_usart_info);
}

type_init(stm32l552_usart_register_types)
