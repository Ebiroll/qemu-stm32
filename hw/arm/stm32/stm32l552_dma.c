/*-
 * Copyright (c) 2023
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
/*
 * QEMU DMA controller device model
 */
#include <stdio.h>
#include <inttypes.h>
#include "qemu/osdep.h"
#include "qapi/error.h"
//#include "qemu-common.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"

#define DEBUG_STM32L552_DMA
#ifdef  DEBUG_STM32L552_DMA

// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32L552_DMA: " fmt , ## __VA_ARGS__); \
         usleep(1000); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define R_DMA_Sx_COUNT           8
#define R_DMA_Sx_REGS            5

#define R_DMA_Sx             (0x08 / 4)

#define R_DMA_ISR           (0x00 / 4)
//#define R_DMA_HISR           (0x04 / 4)//r

#define R_DMA_IFCR          (0x04 / 4)
//#define R_DMA_HIFCR          (0x08 / 4)//w
#define R_DMA_ISR_GIF       (1 << 0)
// Transfer complete interrupt flag
#define R_DMA_ISR_TCIF       (1 << 1)
#define R_DMA_ISR_HTIF       (1 << 2)
#define R_DMA_ISR_TEIF       (1 << 3)


// Per stream register
#define R_DMA_SxISR          (0x00 / 4)
#define R_DMA_SxIFCR         (0x04 / 4)

#define R_DMA_SxCR           ((0x08-0x08) / 4)
#define R_DMA_SxCR_EN        0x00000001
#define R_DMA_SxNDTR         ((0x0C-0x08) / 4)
#define R_DMA_SxPAR          ((0x10-0x08) / 4)
#define R_DMA_SxM0AR         ((0x14-0x08) / 4)
#define R_DMA_SxM1AR         ((0x18-0x08) / 4)

#define R_DMA_MAX            (0xd0 / 4)



// Unhandled registers, one per channel
#define R_DMA_SxCCR2         (0x1C / 4)
#define R_DMA_SxCNDTR2       (0x20 / 4)
#define R_DMA_SxPAR2         (0x24 / 4)
#define R_DMA_SxM0AR2        (0x28 / 4)
#define R_DMA_SxM1AR2        (0x2C / 4)

#define R_DMA_SxCCR3         (0x30 / 4)
#define R_DMA_SxNDTR3        (0x34 / 4)
#define R_DMA_SxPAR3         (0x38 / 4)
#define R_DMA_SxM0AR3        (0x3C / 4)
#define R_DMA_SxM1AR3        (0x40 / 4)
#define R_DMA_SxCCR4         (0x44 / 4)
#define R_DMA_SxCNDTR4       (0x48 / 4)
#define R_DMA_SxPAR4         (0x4C / 4)
#define R_DMA_SxM0AR4        (0x50 / 4)
#define R_DMA_SxM1AR4        (0x54 / 4)
#define R_DMA_SxCCR5         (0x58 / 4)


/* Common interrupt status / clear registers. */
/*
#define R_DMA_LISR           (0x00 / 4)
#define R_DMA_HISR           (0x04 / 4)//r
#define R_DMA_LIFCR          (0x08 / 4)
#define R_DMA_HIFCR          (0x0c / 4)//w
#define R_DMA_ISR_FIEF     (1 << 0)
#define R_DMA_ISR_DMEIF    (1 << 2)
#define R_DMA_ISR_TIEF     (1 << 3)
#define R_DMA_ISR_HTIF     (1 << 4)
#define R_DMA_ISR_TCIF     (1 << 5)

//  Per-stream registers. 
#define R_DMA_Sx             (0x10 / 4)
#define R_DMA_Sx_COUNT           8
#define R_DMA_Sx_REGS            6
#define R_DMA_SxCR           (0x00 / 4)
#define R_DMA_SxCR_EN   0x00000001
#define R_DMA_SxNDTR         (0x04 / 4)
#define R_DMA_SxPAR          (0x08 / 4)
#define R_DMA_SxM0AR         (0x0c / 4)
#define R_DMA_SxM1AR         (0x10 / 4)
#define R_DMA_SxFCR          (0x14 / 4)

#define R_DMA_MAX            (0xd0 / 4)
*/

typedef struct l552_dma_stream {
    qemu_irq irq;

    uint32_t cr;
    uint16_t ndtr;
    uint32_t par;
    uint32_t m0ar;
    uint32_t m1ar;
    uint8_t isr;
} l552_dma_stream;

static int msize_table[] = {1, 2, 4, 0};

typedef struct l552_dma {
    SysBusDevice busdev;
    MemoryRegion iomem;

    //uint32_t ifcr[R_DMA_HIFCR - R_DMA_LIFCR + 1];
    uint32_t ifcr;
    uint32_t active_stream;

    l552_dma_stream stream[R_DMA_Sx_COUNT];
    int stream1;
} l552_dma;

/* Pack ISR bits from four streams, for {L,H}ISR. 
static uint32_t
l552_dma_pack_isr(struct l552_dma *s, int start_stream)
{
    uint32_t r = 0;
    int i;

    for (i = 0; i < 4; i++) {
        r |= s->stream[i + start_stream].isr << (6 * i);
    }
    return r;
}
*/
static uint32_t l552_dma_pack_isr(struct l552_dma *s, int start_stream)
{
    uint32_t r = 0;
    int i;

    for (i = 0; i < 8; i++) {
        // Extract the ISR for the current stream
        uint32_t stream_isr = s->stream[i + start_stream].isr;

        // Align the ISR for the specific stream
        // Shift left by 4 bits for each stream, as each stream's ISR occupies 4 bits
        r |= stream_isr << (4 * i);
    }
    return r;
}


/* Per-stream read. */
static uint32_t
l552_dma_stream_read(l552_dma_stream *s, int stream_no, uint32_t reg)
{
    switch (reg) {
    case R_DMA_SxCR:
        DPRINTF("   %s: stream: %d, register CR\n", __func__, stream_no);
        return s->cr;
    case R_DMA_SxNDTR:
        DPRINTF("   %s: stream: %d, register NDTR (UNIMPLEMENTED)\n", __func__, stream_no);
        qemu_log_mask(LOG_UNIMP, "l552 dma unimp read reg NDTR\n");
        return 0;
    case R_DMA_SxPAR:
        DPRINTF("   %s: stream: %d, register PAR (UNIMPLEMENTED)\n", __func__, stream_no);
        qemu_log_mask(LOG_UNIMP, "l552 dma unimp read reg PAR\n");
        return 0;
    case R_DMA_SxM0AR:
        DPRINTF("   %s: stream: %d, register M0AR (UNIMPLEMENTED)\n", __func__, stream_no);
        qemu_log_mask(LOG_UNIMP, "l552 dma unimp read reg M0AR\n");
        return 0;
    case R_DMA_SxM1AR:
        DPRINTF("   %s: stream: %d, register M1AR (UNIMPLEMENTED)\n", __func__, stream_no);
        qemu_log_mask(LOG_UNIMP, "l552 dma unimp read reg M1AR\n");
        return 0;
    //case R_DMA_SxCCR2:
    //    DPRINTF("   %s: stream: %d, register FCR (UNIMPLEMENTED)\n", __func__, stream_no);
    //   qemu_log_mask(LOG_UNIMP, "l552 dma unimp read reg CCR2\n");
    //    return 0;
    default:
        DPRINTF("   %s: stream: %d, register 0x%02x\n", __func__, stream_no, reg<<2);
        qemu_log_mask(LOG_UNIMP, "l552 dma unimp read stream reg 0x%02x\n",
          (unsigned int)reg<<2);
    }
    return 0;
}

/* Register read. */
static uint64_t
l552_dma_read(void *arg, hwaddr addr, unsigned int size)
{
    l552_dma *s = arg;
    uint64_t result;

    DPRINTF("%s: addr: 0x%lx, size:%d...\n", __func__, addr, size);

    if (size != 4) {
        qemu_log_mask(LOG_UNIMP, "l552 crc only supports 4-byte reads\n");
        return 0;
    }

    addr >>= 2;
    if (addr >= R_DMA_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read l552 dma register 0x%02x\n",
                      (unsigned int)addr << 2);
        result = 0;
    } else {
        switch(addr) {
        case R_DMA_ISR:
            DPRINTF("   %s: register LISR\n", __func__);
            result = l552_dma_pack_isr(s, 0);
            break;
        //case R_DMA_HISR:
        //    DPRINTF("   %s: register HISR\n", __func__);
        //    result = l552_dma_pack_isr(s, 4);
        //    break;
        case R_DMA_IFCR:
            DPRINTF("   %s: register LIFCR\n", __func__);
            //result = s->ifcr[addr - R_DMA_LIFCR];
            result = s->ifcr;
            break;
        //case R_DMA_HIFCR:
        //    DPRINTF("   %s: register HIFCR\n", __func__);
        //    result = s->ifcr[addr - R_DMA_LIFCR];
        //    break;
        default:
            /* Only per-stream registers remain. */
            addr -= R_DMA_Sx;
            int stream_no = addr / R_DMA_Sx_REGS;
            result = l552_dma_stream_read(&s->stream[stream_no], stream_no,
                                          addr % R_DMA_Sx_REGS);
            break;
        }
    }

    DPRINTF("    %s: result:0x%lx\n", __func__, result);
    return result;
}

/* Start a DMA transfer for a given stream. */
static void
l552_dma_synced_stream_start(l552_dma *all,l552_dma_stream *s1,l552_dma_stream *s2, int stream_no1, int stream_no2)
{
    uint8_t buf[4];
    int msize1 = msize_table[(s1->cr >> 13) & 0x3];
    int msize2 = msize_table[(s2->cr >> 13) & 0x3];

    DPRINTF("%s: stream: %d\n", __func__, stream_no1);
    DPRINTF("%s: stream: %d\n", __func__, stream_no2);
   

    if (msize1 == 0  || msize2 == 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "l552 dma: invalid MSIZE\n");
        return;
    }
    /* XXX Skip USART, as pacing control is not yet in place. */
    if (s1->par == 0x40011004) {
        qemu_log_mask(LOG_UNIMP, "l552 dma: skipping USART\n");
        return;
    }

    /* XXX hack do the entire transfer here for now. */
    DPRINTF("%s: transferring %d x %d byte(s) from 0x%08x to 0x%08x\n", __func__, s1->ndtr,
              msize1, s1->m0ar, s1->par);
    DPRINTF("%s: transferring %d x %d byte(s) from 0x%08x to 0x%08x\n", __func__, s2->ndtr,
              msize2, s2->m0ar, s2->par);

    while (s1->ndtr--) {
        cpu_physical_memory_read(s1->m0ar, buf, msize1);
        cpu_physical_memory_write(s1->par, buf, msize1);
        //cpu_physical_memory_read(s2->m0ar, buf, msize2);
        //cpu_physical_memory_write(s2->par, buf, msize2);
        cpu_physical_memory_read(s2->par, buf, msize2);
        cpu_physical_memory_write(s2->m0ar, buf, msize2);
        s1->m0ar += msize1;
        s2->m0ar += msize2;
    }
    /* Transfer complete. */
    s1->cr &= ~R_DMA_SxCR_EN;
    s1->isr |= R_DMA_ISR_TCIF;
 
    all->active_stream = stream_no1;
    qemu_set_irq(s1->irq, 1);

    s2->cr &= ~R_DMA_SxCR_EN;
    s2->isr |= R_DMA_ISR_TCIF;
    all->active_stream = stream_no2;
    qemu_set_irq(s2->irq, 1);

}


/* Start a DMA transfer for a given stream. */
// Only DMA on hspi is implemented, so only synced transfers avaliable
#if 1
static void
l552_dma_stream_start(l552_dma_stream *s, int stream_no)
{
    uint8_t buf[4];
    int msize = msize_table[(s->cr >> 13) & 0x3];

    DPRINTF("%s: stream: %d\n", __func__, stream_no);
   

    if (msize == 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "l552 dma: invalid MSIZE\n");
        return;
    }
    /* XXX Skip USART, as pacing control is not yet in place. */
    if (s->par == 0x40011004) {
        qemu_log_mask(LOG_UNIMP, "l552 dma: skipping USART\n");
        return;
    }

    /* XXX hack do the entire transfer here for now. */
    DPRINTF("%s: transferring %d x %d byte(s) from 0x%08x to 0x%08x\n", __func__, s->ndtr,
              msize, s->m0ar, s->par);
    while (s->ndtr--) {
        cpu_physical_memory_read(s->m0ar, buf, msize);
        cpu_physical_memory_write(s->par, buf, msize);
        s->m0ar += msize;
    }
    /* Transfer complete. */
    s->cr &= ~R_DMA_SxCR_EN;
    s->isr |= R_DMA_ISR_TCIF;

    qemu_set_irq(s->irq, 1);
}
#endif

/* Per-stream register write. */
static void
l552_dma_stream_write(l552_dma_stream *s,l552_dma *all, int stream_no, uint32_t addr, uint32_t data)
{
    switch (addr) {
    case R_DMA_SxCR:
        DPRINTF("%s: stream: %d, register CR, data:0x%x\n", __func__, stream_no, data);
        if ((s->cr & R_DMA_SxCR_EN) == 0 && (data & R_DMA_SxCR_EN) != 0) {
            if (all->stream1!=-1 && all->stream1 !=stream_no) {
                // l552_dma_stream_start(s, stream_no);
                l552_dma_synced_stream_start(all,&all->stream[all->active_stream],s,all->active_stream,stream_no);
                all->stream1 = -1;
                all->active_stream = 0;
            } else {
                // TX Only stream
                if (stream_no==1) {
                    l552_dma_stream_start(s, stream_no);
                } else {
                    all->stream1 = stream_no;
                }
            }
        }
        s->cr = data;
        break;
    case R_DMA_SxNDTR:
        DPRINTF("%s: stream: %d, register NDTR, data:0x%x\n", __func__, stream_no, data);
        if (s->cr & R_DMA_SxCR_EN) {
            qemu_log_mask(LOG_GUEST_ERROR, "l552 dma write to NDTR while enabled\n");
            return;
        }
        s->ndtr = data;
        break;
    case R_DMA_SxPAR:
        DPRINTF("%s: stream: %d, register PAR, data:0x%x\n", __func__, stream_no, data);
        s->par = data;
        break;
    case R_DMA_SxM0AR:
        DPRINTF("%s: stream: %d, register M0AR, data:0x%x\n", __func__, stream_no, data);
        s->m0ar = data;
        break;
    case R_DMA_SxM1AR:
        DPRINTF("%s: stream: %d, register M1AR, data:0x%x\n", __func__, stream_no, data);
        s->m1ar = data;
        break;
    case R_DMA_SxCCR2:
        DPRINTF("%s: stream: %d, register CCR2 (UINIMPLEMENTED), data:0x%x\n", __func__,
                        stream_no, data);
        qemu_log_mask(LOG_UNIMP, "l552 dma SxCCR2 unimplemented\n");
        break;
    }
}

void
clear_interrupt(l552_dma *s,uint64_t data);

void
clear_interrupt(l552_dma *s,uint64_t data) {

        //s->stream[s->active_stream].isr = 0;
        //qemu_set_irq(s->stream[s->active_stream].irq, 0);

        if (data & (0xf << 28)) {
            s->stream[7].isr = 0;
            qemu_set_irq(s->stream[7].irq, 0);
        }
        if (data & (0xf << 24)) {
            s->stream[6].isr = 0;
            qemu_set_irq(s->stream[6].irq, 0);
        }
        if (data & (0xf << 20)) {
            s->stream[5].isr = 0;
            qemu_set_irq(s->stream[5].irq, 0);
        }
        if (data & (0xf << 16)) {
            s->stream[4].isr = 0;
            qemu_set_irq(s->stream[4].irq, 0);
            s->stream[3].isr = 0;
            qemu_set_irq(s->stream[3].irq, 0);
        }
        if (data & (0xf << 12)) {
            s->stream[3].isr = 0;
            qemu_set_irq(s->stream[3].irq, 0);
        }
        if (data & (0xf << 8)) {
            s->stream[2].isr = 0;
            qemu_set_irq(s->stream[2].irq, 0);
        }
        if (data & (0xf << 4)) {
            s->stream[1].isr = 0;
            qemu_set_irq(s->stream[1].irq, 0);
        }
        if (data & (0xf << 0)) {
            s->stream[0].isr = 0;
            qemu_set_irq(s->stream[0].irq, 0);
        }

}


/* Register write. */
static void
l552_dma_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    l552_dma *s = arg;
    int offset = addr & 0x3;

    (void)offset;

    /* XXX Check DMA peripheral clock enable. */
    if (size != 4) {
        qemu_log_mask(LOG_UNIMP, "l552 dma only supports 4-byte writes\n");
        return;
    }

    addr >>= 2;
    if (addr >= R_DMA_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid write l552 dma register 0x%02x\n",
          (unsigned int)addr << 2);
        return;
    }
    if (addr >= R_DMA_Sx && addr <= 0xcc) {
        int num = (addr - R_DMA_Sx) / R_DMA_Sx_REGS;
        l552_dma_stream_write(&s->stream[num],s, num,
          (addr - R_DMA_Sx) % R_DMA_Sx_REGS, data);
        return;
    }
    switch(addr) {
    case R_DMA_ISR:
        DPRINTF("%s: register LISR (READ-ONLY), data: 0x%lx\n", __func__, data);
        qemu_log_mask(LOG_GUEST_ERROR, "l552 dma: invalid write to ISR\n");
        break;
//    case R_DMA_HISR:
//        DPRINTF("%s: register HISR (READ-ONLY), data: 0x%lx\n", __func__, data);
//        qemu_log_mask(LOG_GUEST_ERROR, "l552 dma: invalid write to ISR\n");
//        break;
    case R_DMA_IFCR:
        DPRINTF("%s: register LIFCR, data: 0x%lx\n", __func__, data);
        // Any interrupt clear write to stream x clears all interrupts for that stream
        //s->ifcr[addr - R_DMA_LIFCR] = data;
        s->ifcr = data;
        clear_interrupt(s,data);
        break;
        /*
        if (data & 0x0f400000) {
            s->stream[3].isr = 0;
            qemu_set_irq(s->stream[3].irq, 0);
        }
        if (data & 0x003d0000) {
            s->stream[2].isr = 0;
            qemu_set_irq(s->stream[2].irq, 0);
        }
        if (data & 0x00000f40) {
            s->stream[1].isr = 0;
            qemu_set_irq(s->stream[1].irq, 0);
        }
        if (data & 0x0000003d) {
            s->stream[0].isr = 0;
            qemu_set_irq(s->stream[0].irq, 0);
        }
        break;
        */
#if 0 
    case R_DMA_HIFCR:
        DPRINTF("%s: register HIFCR, data: 0x%lx\n", __func__, data);
        // Any interrupt clear write to stream x clears all interrupts for that stream
        s->ifcr[addr - R_DMA_LIFCR] = data;
        if (data & 0x0f400000) {
            s->stream[7].isr = 0;
            qemu_set_irq(s->stream[7].irq, 0);
        }
        if (data & 0x003d0000) {
            s->stream[6].isr = 0;
            qemu_set_irq(s->stream[6].irq, 0);
        }
        if (data & 0x00000f40) {
            s->stream[5].isr = 0;
            qemu_set_irq(s->stream[5].irq, 0);
        }
        if (data & 0x0000003d) {
            s->stream[4].isr = 0;
            qemu_set_irq(s->stream[4].irq, 0);
        }
        break;
#endif        
    default:
        qemu_log_mask(LOG_UNIMP, "l552 dma unimpl write reg 0x%02x\n",
          (unsigned int)addr << 2);
    }
}

static const MemoryRegionOps l552_dma_ops = {
    .read = l552_dma_read,
    .write = l552_dma_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};


//static void
//l552_rtc_realize(DeviceState *dev, Error **errp)

static void
l552_dma_realize(DeviceState *dev, Error **errp)
{
    struct l552_dma *s = OBJECT_CHECK(struct l552_dma, dev, "l552_dma");
    SysBusDevice *busdev = SYS_BUS_DEVICE(dev);

    int i;

    memory_region_init_io(&s->iomem, OBJECT(s), &l552_dma_ops, s, "dma", 0x400);
    sysbus_init_mmio(busdev, &s->iomem);

    for (i = 0; i < R_DMA_Sx_COUNT; i++) {
        sysbus_init_irq(busdev, &s->stream[i].irq);
    }

}

static void
l552_dma_reset(DeviceState *ds)
{
    //l552_dma *s = FROM_SYSBUS(l552_dma, SYS_BUS_DEVICE(ds));
    struct l552_dma *s = OBJECT_CHECK(struct l552_dma, SYS_BUS_DEVICE(ds), "l552_dma");

    memset(&s->ifcr, 0, sizeof(s->ifcr));

    int i;
    for (i=0; i<R_DMA_Sx_COUNT; i++) {
        qemu_irq save = s->stream[i].irq;
        memset(&s->stream[i], 0, sizeof(l552_dma_stream));
        s->stream[i].irq = save;
    }

    s->active_stream = 0;
    s->stream1 = -1;
}

static Property l552_dma_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void
l552_dma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    // SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(klass);
    dc->realize = l552_dma_realize;
    dc->reset = l552_dma_reset;
    //TODO: fix this: dc->no_user = 1;
    dc->props_ = l552_dma_properties;
}

static const TypeInfo
l552_dma_info = {
    .name          = "l552_dma",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(l552_dma),
    .class_init    = l552_dma_class_init,
};

static void
l552_dma_register_types(void)
{
    type_register_static(&l552_dma_info);
}

type_init(l552_dma_register_types)
