/*
 * QEMU ETRAX Interrupt Controller.
 *
 * Copyright (c) 2008 Edgar E. Iglesias, Axis Communications AB.
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

#include "sysbus.h"
#include "hw.h"
//#include "pc.h"
//#include "etraxfs.h"

#define D(x)

#define R_RW_MASK   0
#define R_R_VECT    1
#define R_R_MASKED_VECT 2
#define R_R_NMI     3
#define R_R_GURU    4
#define R_MAX       5

struct etrax_pic
{
    SysBusDevice busdev;
    MemoryRegion mmio;
    void *interrupt_vector;
    qemu_irq parent_irq;
    qemu_irq parent_nmi;
    uint32_t regs[R_MAX];
};

static void pic_update(struct etrax_pic *fs)
{   
    uint32_t vector = 0;
    int i;

    fs->regs[R_R_MASKED_VECT] = fs->regs[R_R_VECT] & fs->regs[R_RW_MASK];

    /* The ETRAX interrupt controller signals interrupts to the core
       through an interrupt request wire and an irq vector bus. If 
       multiple interrupts are simultaneously active it chooses vector 
       0x30 and lets the sw choose the priorities.  */
    if (fs->regs[R_R_MASKED_VECT]) {
        uint32_t mv = fs->regs[R_R_MASKED_VECT];
        for (i = 0; i < 31; i++) {
            if (mv & 1) {
                vector = 0x31 + i;
                /* Check for multiple interrupts.  */
                if (mv > 1)
                    vector = 0x30;
                break;
            }
            mv >>= 1;
        }
    }

    if (fs->interrupt_vector) {
        /* hack alert: ptr property */
        *(uint32_t*)(fs->interrupt_vector) = vector;
    }
    qemu_set_irq(fs->parent_irq, !!vector);
}

static uint64_t
pic_read(void *opaque, target_phys_addr_t addr, unsigned int size)
{
    struct etrax_pic *fs = opaque;
    uint32_t rval;

    rval = fs->regs[addr >> 2];
    D(printf("%s %x=%x\n", __func__, addr, rval));
    return rval;
}

static void pic_write(void *opaque, target_phys_addr_t addr,
                      uint64_t value, unsigned int size)
{
    struct etrax_pic *fs = opaque;
    D(printf("%s addr=%x val=%x\n", __func__, addr, value));

    if (addr == R_RW_MASK) {
        fs->regs[R_RW_MASK] = value;
        pic_update(fs);
    }
}

static const MemoryRegionOps pic_ops = {
    .read = pic_read,
    .write = pic_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void nmi_handler(void *opaque, int irq, int level)
{   
    struct etrax_pic *fs = (void *)opaque;
    uint32_t mask;

    mask = 1 << irq;
    if (level)
        fs->regs[R_R_NMI] |= mask;
    else
        fs->regs[R_R_NMI] &= ~mask;

    qemu_set_irq(fs->parent_nmi, !!fs->regs[R_R_NMI]);
}

static void irq_handler(void *opaque, int irq, int level)
{   
    struct etrax_pic *fs = (void *)opaque;

    if (irq >= 30)
        return nmi_handler(opaque, irq, level);

    irq -= 1;
    fs->regs[R_R_VECT] &= ~(1 << irq);
    fs->regs[R_R_VECT] |= (!!level << irq);
    pic_update(fs);
}

static int etraxfs_pic_init(SysBusDevice *dev)
{
    struct etrax_pic *s = FROM_SYSBUS(typeof (*s), dev);

    qdev_init_gpio_in(&dev->qdev, irq_handler, 32);
    sysbus_init_irq(dev, &s->parent_irq);
    sysbus_init_irq(dev, &s->parent_nmi);

    memory_region_init_io(&s->mmio, &pic_ops, s, "etraxfs-pic", R_MAX * 4);
    sysbus_init_mmio(dev, &s->mmio);
    return 0;
}

static SysBusDeviceInfo etraxfs_pic_info = {
    .init = etraxfs_pic_init,
    .qdev.name  = "etraxfs,pic",
    .qdev.size  = sizeof(struct etrax_pic),
    .qdev.props = (Property[]) {
        DEFINE_PROP_PTR("interrupt_vector", struct etrax_pic, interrupt_vector),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void etraxfs_pic_register(void)
{
    sysbus_register_withprop(&etraxfs_pic_info);
}

device_init(etraxfs_pic_register)
