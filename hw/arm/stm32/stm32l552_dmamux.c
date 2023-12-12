#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/qdev-properties-system.h"
#include "stm32l552_dmamux.h"

static uint64_t stm32l55_dmamux_read(void *opaque, hwaddr offset, unsigned size)
{
    STM32L55DmaMuxState *s = STM32L552_DMAMUX(opaque);

    switch (offset) {
     case RG0CR_OFFSET:
        return s->gcr[0];
    case RG1CR_OFFSET:
        return s->gcr[1];
    case RG2CR_OFFSET:
        return s->gcr[2];
    case RG3CR_OFFSET:
        return s->gcr[3];
    case RG4CR_OFFSET:
        return s->gcr[4];
    case RG5CR_OFFSET:
        return s->gcr[5];
    case RG6CR_OFFSET:
        return s->gcr[6];
    case RG7CR_OFFSET:
        return s->gcr[7];
    case RG8CR_OFFSET:
        return s->gcr[8];
    case RG9CR_OFFSET:
        return s->gcr[9];
    case RG10CR_OFFSET:
        return s->gcr[10];
    case RG11CR_OFFSET:
        return s->gcr[11];
    case RG12CR_OFFSET:
        return s->gcr[12];
    case RG13CR_OFFSET:
        return s->gcr[13];
    case RG14CR_OFFSET:
        return s->gcr[14];
    case RG15CR_OFFSET:
        return s->gcr[15];
    
    // Handle GCR array

    case MUX_CCR:
        return s->ccr;
    // Add cases for other registers as needed
    default:
        qemu_log_mask(LOG_UNIMP, "%s: Invalid read from offset 0x%" HWADDR_PRIx "\n", __func__, offset);
        return 0;
    }
}

static void stm32l55_dmamux_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    STM32L55DmaMuxState *s = STM32L552_DMAMUX(opaque);

    switch (offset) {
    case MUX_CCR:
        s->ccr = value;
        // Add logic to handle CCR register changes
        break;
    // Add cases for other registers as needed
    default:
        qemu_log_mask(LOG_UNIMP, "%s: Invalid write to offset 0x%" HWADDR_PRIx "\n", __func__, offset);
        break;
    }
}

static const MemoryRegionOps stm32l55_dmamux_ops = {
    .read = stm32l55_dmamux_read,
    .write = stm32l55_dmamux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void stm32l55_dmamux_init(Object *obj)
{
    STM32L55DmaMuxState *s = STM32L552_DMAMUX(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &stm32l55_dmamux_ops, s, TYPE_STM32L552_DMAMUX, 0x100);
    sysbus_init_mmio(sbd, &s->mmio);

    // Initialize IRQ
    sysbus_init_irq(sbd, &s->irq);
}

static void stm32l55_dmamux_reset(DeviceState *dev)
{
    STM32L55DmaMuxState *s = STM32L552_DMAMUX(dev);

    s->ccr = 0; // Reset value
    // Reset other registers and state as required
}

static void stm32l55_dmamux_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32l55_dmamux_reset;
}

static TypeInfo stm32l55_dmamux_type_info = {
    .name = TYPE_STM32L552_DMAMUX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32L55DmaMuxState),
    .instance_init = stm32l55_dmamux_init,
    .class_init = stm32l55_dmamux_class_init,
};

static void stm32l55_dmamux_register_types(void)
{
    type_register_static(&stm32l55_dmamux_type_info);
}

type_init(stm32l55_dmamux_register_types)
