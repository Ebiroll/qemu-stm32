/*
 * QEMU x86 ISA testdev
 *
 * Copyright (c) 2012 Avi Kivity, Gerd Hoffmann, Marcelo Tosatti
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
 * This device is used to test KVM features specific to the x86 port, such
 * as emulation, power management, interrupt routing, among others. It's meant
 * to be used like:
 *
 * qemu-system-x86_64 -device pc-testdev -serial stdio \
 * -device isa-debug-exit,iobase=0xf4,iosize=0x4 \
 * -kernel /home/lmr/Code/virt-test.git/kvm/unittests/msr.flat
 *
 * Where msr.flat is one of the KVM unittests, present on a separate repo,
 * git://git.kernel.org/pub/scm/virt/kvm/kvm-unit-tests.git
*/

#include <sys/mman.h>
#include "hw.h"
#include "qdev.h"
#include "isa.h"

#define IOMEM_LEN    0x10000

typedef struct PCTestdev {
    ISADevice parent_obj;

    MemoryRegion ioport;
    MemoryRegion flush;
    MemoryRegion irq;
    MemoryRegion iomem;
    uint32_t ioport_data;
    char iomem_buf[IOMEM_LEN];
} PCTestdev;

#define TYPE_TESTDEV "pc-testdev"
#define TESTDEV(obj) \
     OBJECT_CHECK(struct PCTestdev, (obj), TYPE_TESTDEV)

static void test_irq_line(void *opaque, hwaddr addr, uint64_t data,
                          unsigned len)
{
    struct PCTestdev *dev = opaque;
    struct ISADevice *isa = ISA_DEVICE(dev);

    qemu_set_irq(isa_get_irq(isa, addr), !!data);
}

static const MemoryRegionOps test_irq_ops = {
    .write = test_irq_line,
    .valid.min_access_size = 1,
    .valid.max_access_size = 1,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void test_ioport_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned len)
{
    struct PCTestdev *dev = opaque;
    dev->ioport_data = data;
}

static uint64_t test_ioport_read(void *opaque, hwaddr addr, unsigned len)
{
    struct PCTestdev *dev = opaque;
    return dev->ioport_data;
}

static const MemoryRegionOps test_ioport_ops = {
    .read = test_ioport_read,
    .write = test_ioport_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void test_flush_page(void *opaque, hwaddr addr, uint64_t data,
                            unsigned len)
{
    hwaddr page = 4096;
    void *a = cpu_physical_memory_map(data & ~0xffful, &page, 0);

    /* We might not be able to get the full page, only mprotect what we actually
       have mapped */
    mprotect(a, page, PROT_NONE);
    mprotect(a, page, PROT_READ|PROT_WRITE);
    cpu_physical_memory_unmap(a, page, 0, 0);
}

static const MemoryRegionOps test_flush_ops = {
    .write = test_flush_page,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static uint64_t test_iomem_read(void *opaque, hwaddr addr, unsigned len)
{
    struct PCTestdev *dev = opaque;
    uint64_t ret = 0;
    memcpy(&ret, &dev->iomem_buf[addr], len);
    ret = le64_to_cpu(ret);

    return ret;
}

static void test_iomem_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned len)
{
    struct PCTestdev *dev = opaque;
    val = cpu_to_le64(val);
    memcpy(&dev->iomem_buf[addr], &val, len);
    dev->iomem_buf[addr] = val;
}

static const MemoryRegionOps test_iomem_ops = {
    .read = test_iomem_read,
    .write = test_iomem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static int init_test_device(ISADevice *isa)
{
    struct PCTestdev *dev = TESTDEV(isa);
    MemoryRegion *mem = isa_address_space(isa);
    MemoryRegion *io = isa_address_space_io(isa);

    memory_region_init_io(&dev->ioport, &test_ioport_ops, dev,
                          "pc-testdev-ioport", 4);
    memory_region_init_io(&dev->flush, &test_flush_ops, dev,
                          "pc-testdev-flush-page", 4);
    memory_region_init_io(&dev->irq, &test_irq_ops, dev,
                          "pc-testdev-irq-line", 24);
    memory_region_init_io(&dev->iomem, &test_iomem_ops, dev,
                          "pc-testdev-iomem", IOMEM_LEN);

    memory_region_add_subregion(io,  0xe0,       &dev->ioport);
    memory_region_add_subregion(io,  0xe4,       &dev->flush);
    memory_region_add_subregion(io,  0x2000,     &dev->irq);
    memory_region_add_subregion(mem, 0xff000000, &dev->iomem);

    return 0;
}

static void testdev_class_init(ObjectClass *klass, void *data)
{
    ISADeviceClass *k = ISA_DEVICE_CLASS(klass);

    k->init = init_test_device;
}

static TypeInfo testdev_info = {
    .name           = TYPE_TESTDEV,
    .parent         = TYPE_ISA_DEVICE,
    .instance_size  = sizeof(struct PCTestdev),
    .class_init     = testdev_class_init,
};

static void testdev_register_types(void)
{
    type_register_static(&testdev_info);
}

type_init(testdev_register_types)
