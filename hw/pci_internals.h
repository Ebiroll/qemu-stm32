#ifndef QEMU_PCI_INTERNALS_H
#define QEMU_PCI_INTERNALS_H

/*
 * This header files is private to pci.c and pci_bridge.c
 * So following structures are opaque to others and shouldn't be
 * accessed.
 */

extern struct BusInfo pci_bus_info;

struct PCIBus {
    BusState qbus;
    int devfn_min;
    pci_set_irq_fn set_irq;
    pci_map_irq_fn map_irq;
    pci_hotplug_fn hotplug;
    DeviceState *hotplug_qdev;
    void *irq_opaque;
    PCIDevice *devices[256];
    PCIDevice *parent_dev;
    target_phys_addr_t mem_base;

    QLIST_HEAD(, PCIBus) child; /* this will be replaced by qdev later */
    QLIST_ENTRY(PCIBus) sibling;/* this will be replaced by qdev later */

    /* The bus IRQ state is the logical OR of the connected devices.
       Keep a count of the number of devices with raised IRQs.  */
    int nirq;
    int *irq_count;
};

typedef struct {
    PCIDevice dev;
    PCIBus bus;
    uint32_t vid;
    uint32_t did;
} PCIBridge;

#endif /* QEMU_PCI_INTERNALS_H */
