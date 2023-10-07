/*
 * QEMU PIIX PCI ISA Bridge Emulation
 *
 * Copyright (c) 2006 Fabrice Bellard
 * Copyright (c) 2018 Hervé Poussineau
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

#include "qemu/osdep.h"
#include "qemu/range.h"
#include "qapi/error.h"
#include "hw/dma/i8257.h"
#include "hw/southbridge/piix.h"
#include "hw/timer/i8254.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ide/piix.h"
#include "hw/intc/i8259.h"
#include "hw/isa/isa.h"
#include "sysemu/runstate.h"
#include "migration/vmstate.h"
#include "hw/acpi/acpi_aml_interface.h"

typedef struct PIIXState PIIX4State;

DECLARE_INSTANCE_CHECKER(PIIX4State, PIIX4_PCI_DEVICE, TYPE_PIIX4_PCI_DEVICE)

static void piix3_set_irq_pic(PIIXState *piix3, int pic_irq)
{
    qemu_set_irq(piix3->isa_irqs_in[pic_irq],
                 !!(piix3->pic_levels &
                    (((1ULL << PIIX_NUM_PIRQS) - 1) <<
                     (pic_irq * PIIX_NUM_PIRQS))));
}

static void piix3_set_irq_level_internal(PIIXState *piix3, int pirq, int level)
{
    int pic_irq;
    uint64_t mask;

    pic_irq = piix3->dev.config[PIIX_PIRQCA + pirq];
    if (pic_irq >= ISA_NUM_IRQS) {
        return;
    }

    mask = 1ULL << ((pic_irq * PIIX_NUM_PIRQS) + pirq);
    piix3->pic_levels &= ~mask;
    piix3->pic_levels |= mask * !!level;
}

static void piix3_set_irq_level(PIIXState *piix3, int pirq, int level)
{
    int pic_irq;

    pic_irq = piix3->dev.config[PIIX_PIRQCA + pirq];
    if (pic_irq >= ISA_NUM_IRQS) {
        return;
    }

    piix3_set_irq_level_internal(piix3, pirq, level);

    piix3_set_irq_pic(piix3, pic_irq);
}

static void piix3_set_irq(void *opaque, int pirq, int level)
{
    PIIXState *piix3 = opaque;
    piix3_set_irq_level(piix3, pirq, level);
}

static void piix4_set_irq(void *opaque, int irq_num, int level)
{
    int i, pic_irq, pic_level;
    PIIX4State *s = opaque;
    PCIBus *bus = pci_get_bus(&s->dev);

    /* now we change the pic irq level according to the piix irq mappings */
    /* XXX: optimize */
    pic_irq = s->dev.config[PIIX_PIRQCA + irq_num];
    if (pic_irq < ISA_NUM_IRQS) {
        /* The pic level is the logical OR of all the PCI irqs mapped to it. */
        pic_level = 0;
        for (i = 0; i < PIIX_NUM_PIRQS; i++) {
            if (pic_irq == s->dev.config[PIIX_PIRQCA + i]) {
                pic_level |= pci_bus_get_irq_level(bus, i);
            }
        }
        qemu_set_irq(s->isa_irqs_in[pic_irq], pic_level);
    }
}

static void piix_request_i8259_irq(void *opaque, int irq, int level)
{
    PIIX4State *s = opaque;
    qemu_set_irq(s->cpu_intr, level);
}

static PCIINTxRoute piix3_route_intx_pin_to_irq(void *opaque, int pin)
{
    PIIXState *piix3 = opaque;
    int irq = piix3->dev.config[PIIX_PIRQCA + pin];
    PCIINTxRoute route;

    if (irq < ISA_NUM_IRQS) {
        route.mode = PCI_INTX_ENABLED;
        route.irq = irq;
    } else {
        route.mode = PCI_INTX_DISABLED;
        route.irq = -1;
    }
    return route;
}

/* irq routing is changed. so rebuild bitmap */
static void piix3_update_irq_levels(PIIXState *piix3)
{
    PCIBus *bus = pci_get_bus(&piix3->dev);
    int pirq;

    piix3->pic_levels = 0;
    for (pirq = 0; pirq < PIIX_NUM_PIRQS; pirq++) {
        piix3_set_irq_level(piix3, pirq, pci_bus_get_irq_level(bus, pirq));
    }
}

static void piix3_write_config(PCIDevice *dev,
                               uint32_t address, uint32_t val, int len)
{
    pci_default_write_config(dev, address, val, len);
    if (ranges_overlap(address, len, PIIX_PIRQCA, 4)) {
        PIIXState *piix3 = PIIX_PCI_DEVICE(dev);
        int pic_irq;

        pci_bus_fire_intx_routing_notifier(pci_get_bus(&piix3->dev));
        piix3_update_irq_levels(piix3);
        for (pic_irq = 0; pic_irq < ISA_NUM_IRQS; pic_irq++) {
            piix3_set_irq_pic(piix3, pic_irq);
        }
    }
}

static void piix_reset(PIIXState *d)
{
    uint8_t *pci_conf = d->dev.config;

    pci_conf[0x04] = 0x07; /* master, memory and I/O */
    pci_conf[0x05] = 0x00;
    pci_conf[0x06] = 0x00;
    pci_conf[0x07] = 0x02; /* PCI_status_devsel_medium */
    pci_conf[0x4c] = 0x4d;
    pci_conf[0x4e] = 0x03;
    pci_conf[0x4f] = 0x00;
    pci_conf[0x60] = 0x80;
    pci_conf[0x61] = 0x80;
    pci_conf[0x62] = 0x80;
    pci_conf[0x63] = 0x80;
    pci_conf[0x69] = 0x02;
    pci_conf[0x70] = 0x80;
    pci_conf[0x76] = 0x0c;
    pci_conf[0x77] = 0x0c;
    pci_conf[0x78] = 0x02;
    pci_conf[0x79] = 0x00;
    pci_conf[0x80] = 0x00;
    pci_conf[0x82] = 0x00;
    pci_conf[0xa0] = 0x08;
    pci_conf[0xa2] = 0x00;
    pci_conf[0xa3] = 0x00;
    pci_conf[0xa4] = 0x00;
    pci_conf[0xa5] = 0x00;
    pci_conf[0xa6] = 0x00;
    pci_conf[0xa7] = 0x00;
    pci_conf[0xa8] = 0x0f;
    pci_conf[0xaa] = 0x00;
    pci_conf[0xab] = 0x00;
    pci_conf[0xac] = 0x00;
    pci_conf[0xae] = 0x00;

    d->pic_levels = 0;
    d->rcr = 0;
}

static void piix3_reset(DeviceState *dev)
{
    PIIXState *d = PIIX_PCI_DEVICE(dev);

    piix_reset(d);
}

static int piix3_post_load(void *opaque, int version_id)
{
    PIIXState *piix3 = opaque;
    int pirq;

    /*
     * Because the i8259 has not been deserialized yet, qemu_irq_raise
     * might bring the system to a different state than the saved one;
     * for example, the interrupt could be masked but the i8259 would
     * not know that yet and would trigger an interrupt in the CPU.
     *
     * Here, we update irq levels without raising the interrupt.
     * Interrupt state will be deserialized separately through the i8259.
     */
    piix3->pic_levels = 0;
    for (pirq = 0; pirq < PIIX_NUM_PIRQS; pirq++) {
        piix3_set_irq_level_internal(piix3, pirq,
            pci_bus_get_irq_level(pci_get_bus(&piix3->dev), pirq));
    }
    return 0;
}

static int piix4_post_load(void *opaque, int version_id)
{
    PIIX4State *s = opaque;

    if (version_id == 2) {
        s->rcr = 0;
    }

    return 0;
}

static int piix3_pre_save(void *opaque)
{
    int i;
    PIIXState *piix3 = opaque;

    for (i = 0; i < ARRAY_SIZE(piix3->pci_irq_levels_vmstate); i++) {
        piix3->pci_irq_levels_vmstate[i] =
            pci_bus_get_irq_level(pci_get_bus(&piix3->dev), i);
    }

    return 0;
}

static bool piix3_rcr_needed(void *opaque)
{
    PIIXState *piix3 = opaque;

    return (piix3->rcr != 0);
}

static const VMStateDescription vmstate_piix3_rcr = {
    .name = "PIIX3/rcr",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = piix3_rcr_needed,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(rcr, PIIXState),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_piix3 = {
    .name = "PIIX3",
    .version_id = 3,
    .minimum_version_id = 2,
    .post_load = piix3_post_load,
    .pre_save = piix3_pre_save,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, PIIXState),
        VMSTATE_INT32_ARRAY_V(pci_irq_levels_vmstate, PIIXState,
                              PIIX_NUM_PIRQS, 3),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription*[]) {
        &vmstate_piix3_rcr,
        NULL
    }
};

static const VMStateDescription vmstate_piix4 = {
    .name = "PIIX4",
    .version_id = 3,
    .minimum_version_id = 2,
    .post_load = piix4_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, PIIX4State),
        VMSTATE_UINT8_V(rcr, PIIX4State, 3),
        VMSTATE_END_OF_LIST()
    }
};

static void rcr_write(void *opaque, hwaddr addr, uint64_t val, unsigned len)
{
    PIIXState *d = opaque;

    if (val & 4) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        return;
    }
    d->rcr = val & 2; /* keep System Reset type only */
}

static uint64_t rcr_read(void *opaque, hwaddr addr, unsigned len)
{
    PIIXState *d = opaque;

    return d->rcr;
}

static const MemoryRegionOps rcr_ops = {
    .read = rcr_read,
    .write = rcr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void pci_piix3_realize(PCIDevice *dev, Error **errp)
{
    PIIXState *d = PIIX_PCI_DEVICE(dev);
    PCIBus *pci_bus = pci_get_bus(dev);
    ISABus *isa_bus;
    uint32_t irq;

    isa_bus = isa_bus_new(DEVICE(d), pci_address_space(dev),
                          pci_address_space_io(dev), errp);
    if (!isa_bus) {
        return;
    }

    memory_region_init_io(&d->rcr_mem, OBJECT(dev), &rcr_ops, d,
                          "piix3-reset-control", 1);
    memory_region_add_subregion_overlap(pci_address_space_io(dev),
                                        PIIX_RCR_IOPORT, &d->rcr_mem, 1);

    /* PIC */
    if (d->has_pic) {
        qemu_irq *i8259_out_irq = qemu_allocate_irqs(piix_request_i8259_irq, d,
                                                     1);
        qemu_irq *i8259 = i8259_init(isa_bus, *i8259_out_irq);
        size_t i;

        for (i = 0; i < ISA_NUM_IRQS; i++) {
            d->isa_irqs_in[i] = i8259[i];
        }

        g_free(i8259);

        qdev_init_gpio_out_named(DEVICE(dev), &d->cpu_intr, "intr", 1);
    }

    isa_bus_register_input_irqs(isa_bus, d->isa_irqs_in);

    /* PIT */
    if (d->has_pit) {
        i8254_pit_init(isa_bus, 0x40, 0, NULL);
    }

    i8257_dma_init(isa_bus, 0);

    /* RTC */
    qdev_prop_set_int32(DEVICE(&d->rtc), "base_year", 2000);
    if (!qdev_realize(DEVICE(&d->rtc), BUS(isa_bus), errp)) {
        return;
    }
    irq = object_property_get_uint(OBJECT(&d->rtc), "irq", &error_fatal);
    isa_connect_gpio_out(ISA_DEVICE(&d->rtc), 0, irq);

    /* IDE */
    qdev_prop_set_int32(DEVICE(&d->ide), "addr", dev->devfn + 1);
    if (!qdev_realize(DEVICE(&d->ide), BUS(pci_bus), errp)) {
        return;
    }

    /* USB */
    if (d->has_usb) {
        object_initialize_child(OBJECT(dev), "uhci", &d->uhci,
                                TYPE_PIIX3_USB_UHCI);
        qdev_prop_set_int32(DEVICE(&d->uhci), "addr", dev->devfn + 2);
        if (!qdev_realize(DEVICE(&d->uhci), BUS(pci_bus), errp)) {
            return;
        }
    }

    /* Power Management */
    if (d->has_acpi) {
        object_initialize_child(OBJECT(d), "pm", &d->pm, TYPE_PIIX4_PM);
        qdev_prop_set_int32(DEVICE(&d->pm), "addr", dev->devfn + 3);
        qdev_prop_set_uint32(DEVICE(&d->pm), "smb_io_base", d->smb_io_base);
        qdev_prop_set_bit(DEVICE(&d->pm), "smm-enabled", d->smm_enabled);
        if (!qdev_realize(DEVICE(&d->pm), BUS(pci_bus), errp)) {
            return;
        }
        qdev_connect_gpio_out(DEVICE(&d->pm), 0, d->isa_irqs_in[9]);
    }
}

static void build_pci_isa_aml(AcpiDevAmlIf *adev, Aml *scope)
{
    Aml *field;
    Aml *sb_scope = aml_scope("\\_SB");
    BusState *bus = qdev_get_child_bus(DEVICE(adev), "isa.0");

    /* PIIX PCI to ISA irq remapping */
    aml_append(scope, aml_operation_region("P40C", AML_PCI_CONFIG,
                                           aml_int(0x60), 0x04));
    /* Fields declarion has to happen *after* operation region */
    field = aml_field("PCI0.S08.P40C", AML_BYTE_ACC, AML_NOLOCK, AML_PRESERVE);
    aml_append(field, aml_named_field("PRQ0", 8));
    aml_append(field, aml_named_field("PRQ1", 8));
    aml_append(field, aml_named_field("PRQ2", 8));
    aml_append(field, aml_named_field("PRQ3", 8));
    aml_append(sb_scope, field);
    aml_append(scope, sb_scope);

    qbus_build_aml(bus, scope);
}

static void pci_piix3_init(Object *obj)
{
    PIIXState *d = PIIX_PCI_DEVICE(obj);

    qdev_init_gpio_out_named(DEVICE(obj), d->isa_irqs_in, "isa-irqs",
                             ISA_NUM_IRQS);

    object_initialize_child(obj, "rtc", &d->rtc, TYPE_MC146818_RTC);
    object_initialize_child(obj, "ide", &d->ide, TYPE_PIIX3_IDE);
}

static Property pci_piix3_props[] = {
    DEFINE_PROP_UINT32("smb_io_base", PIIXState, smb_io_base, 0),
    DEFINE_PROP_BOOL("has-acpi", PIIXState, has_acpi, true),
    DEFINE_PROP_BOOL("has-pic", PIIXState, has_pic, true),
    DEFINE_PROP_BOOL("has-pit", PIIXState, has_pit, true),
    DEFINE_PROP_BOOL("has-usb", PIIXState, has_usb, true),
    DEFINE_PROP_BOOL("smm-enabled", PIIXState, smm_enabled, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void pci_piix3_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    AcpiDevAmlIfClass *adevc = ACPI_DEV_AML_IF_CLASS(klass);

    k->config_write = piix3_write_config;
    dc->reset       = piix3_reset;
    dc->desc        = "ISA bridge";
    dc->vmsd        = &vmstate_piix3;
    dc->hotpluggable   = false;
    k->vendor_id    = PCI_VENDOR_ID_INTEL;
    /* 82371SB PIIX3 PCI-to-ISA bridge (Step A1) */
    k->device_id    = PCI_DEVICE_ID_INTEL_82371SB_0;
    k->class_id     = PCI_CLASS_BRIDGE_ISA;
    /*
     * Reason: part of PIIX3 southbridge, needs to be wired up by
     * pc_piix.c's pc_init1()
     */
    dc->user_creatable = false;
    device_class_set_props(dc, pci_piix3_props);
    adevc->build_dev_aml = build_pci_isa_aml;
}

static const TypeInfo piix_pci_type_info = {
    .name = TYPE_PIIX_PCI_DEVICE,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PIIXState),
    .instance_init = pci_piix3_init,
    .abstract = true,
    .class_init = pci_piix3_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { TYPE_ACPI_DEV_AML_IF },
        { },
    },
};

static void piix3_realize(PCIDevice *dev, Error **errp)
{
    ERRP_GUARD();
    PIIXState *piix3 = PIIX_PCI_DEVICE(dev);
    PCIBus *pci_bus = pci_get_bus(dev);

    pci_piix3_realize(dev, errp);
    if (*errp) {
        return;
    }

    pci_bus_irqs(pci_bus, piix3_set_irq, piix3, PIIX_NUM_PIRQS);
    pci_bus_set_route_irq_fn(pci_bus, piix3_route_intx_pin_to_irq);
}

static void piix3_class_init(ObjectClass *klass, void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = piix3_realize;
}

static const TypeInfo piix3_info = {
    .name          = TYPE_PIIX3_DEVICE,
    .parent        = TYPE_PIIX_PCI_DEVICE,
    .class_init    = piix3_class_init,
};

static void piix4_realize(PCIDevice *dev, Error **errp)
{
    PIIX4State *s = PIIX4_PCI_DEVICE(dev);
    PCIBus *pci_bus = pci_get_bus(dev);
    ISABus *isa_bus;
    qemu_irq *i8259_out_irq;
    qemu_irq *i8259;
    size_t i;

    isa_bus = isa_bus_new(DEVICE(dev), pci_address_space(dev),
                          pci_address_space_io(dev), errp);
    if (!isa_bus) {
        return;
    }

    qdev_init_gpio_out_named(DEVICE(dev), &s->cpu_intr,
                             "intr", 1);

    memory_region_init_io(&s->rcr_mem, OBJECT(dev), &rcr_ops, s,
                          "reset-control", 1);
    memory_region_add_subregion_overlap(pci_address_space_io(dev),
                                        PIIX_RCR_IOPORT, &s->rcr_mem, 1);

    /* initialize i8259 pic */
    i8259_out_irq = qemu_allocate_irqs(piix_request_i8259_irq, s, 1);
    i8259 = i8259_init(isa_bus, *i8259_out_irq);

    for (i = 0; i < ISA_NUM_IRQS; i++) {
        s->isa_irqs_in[i] = i8259[i];
    }

    g_free(i8259);

    /* initialize ISA irqs */
    isa_bus_register_input_irqs(isa_bus, s->isa_irqs_in);

    /* initialize pit */
    i8254_pit_init(isa_bus, 0x40, 0, NULL);

    /* DMA */
    i8257_dma_init(isa_bus, 0);

    /* RTC */
    qdev_prop_set_int32(DEVICE(&s->rtc), "base_year", 2000);
    if (!qdev_realize(DEVICE(&s->rtc), BUS(isa_bus), errp)) {
        return;
    }
    s->rtc.irq = isa_get_irq(ISA_DEVICE(&s->rtc), s->rtc.isairq);

    /* IDE */
    qdev_prop_set_int32(DEVICE(&s->ide), "addr", dev->devfn + 1);
    if (!qdev_realize(DEVICE(&s->ide), BUS(pci_bus), errp)) {
        return;
    }

    /* USB */
    qdev_prop_set_int32(DEVICE(&s->uhci), "addr", dev->devfn + 2);
    if (!qdev_realize(DEVICE(&s->uhci), BUS(pci_bus), errp)) {
        return;
    }

    /* ACPI controller */
    qdev_prop_set_int32(DEVICE(&s->pm), "addr", dev->devfn + 3);
    if (!qdev_realize(DEVICE(&s->pm), BUS(pci_bus), errp)) {
        return;
    }
    qdev_connect_gpio_out(DEVICE(&s->pm), 0, s->isa_irqs_in[9]);

    pci_bus_irqs(pci_bus, piix4_set_irq, s, PIIX_NUM_PIRQS);
}

static void piix4_isa_reset(DeviceState *dev)
{
    PIIX4State *s = PIIX4_PCI_DEVICE(dev);

    piix_reset(s);
}

static void piix4_init(Object *obj)
{
    PIIX4State *s = PIIX4_PCI_DEVICE(obj);

    object_initialize_child(obj, "rtc", &s->rtc, TYPE_MC146818_RTC);
    object_initialize_child(obj, "ide", &s->ide, TYPE_PIIX4_IDE);
    object_initialize_child(obj, "uhci", &s->uhci, TYPE_PIIX4_USB_UHCI);

    object_initialize_child(obj, "pm", &s->pm, TYPE_PIIX4_PM);
    qdev_prop_set_uint32(DEVICE(&s->pm), "smb_io_base", 0x1100);
    qdev_prop_set_bit(DEVICE(&s->pm), "smm-enabled", 0);
}

static void piix4_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = piix4_realize;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = PCI_DEVICE_ID_INTEL_82371AB_0;
    k->class_id = PCI_CLASS_BRIDGE_ISA;
    dc->reset = piix4_isa_reset;
    dc->desc = "ISA bridge";
    dc->vmsd = &vmstate_piix4;
    /*
     * Reason: part of PIIX4 southbridge, needs to be wired up,
     * e.g. by mips_malta_init()
     */
    dc->user_creatable = false;
    dc->hotpluggable = false;
}

static const TypeInfo piix4_info = {
    .name          = TYPE_PIIX4_PCI_DEVICE,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PIIX4State),
    .instance_init = piix4_init,
    .class_init    = piix4_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void piix3_register_types(void)
{
    type_register_static(&piix_pci_type_info);
    type_register_static(&piix3_info);
    type_register_static(&piix4_info);
}

type_init(piix3_register_types)
