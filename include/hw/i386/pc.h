#ifndef HW_PC_H
#define HW_PC_H

#include "qemu-common.h"
#include "exec/memory.h"
#include "hw/isa/isa.h"
#include "hw/block/fdc.h"
#include "net/net.h"
#include "hw/i386/ioapic.h"

#include "qemu/range.h"
#include "qemu/bitmap.h"
#include "sysemu/sysemu.h"
#include "hw/pci/pci.h"

#define HPET_INTCAP "hpet-intcap"

/* PC-style peripherals (also used by other machines).  */

typedef struct PcPciInfo {
    Range w32;
    Range w64;
} PcPciInfo;

#define ACPI_PM_PROP_S3_DISABLED "disable_s3"
#define ACPI_PM_PROP_S4_DISABLED "disable_s4"
#define ACPI_PM_PROP_S4_VAL "s4_val"
#define ACPI_PM_PROP_SCI_INT "sci_int"
#define ACPI_PM_PROP_ACPI_ENABLE_CMD "acpi_enable_cmd"
#define ACPI_PM_PROP_ACPI_DISABLE_CMD "acpi_disable_cmd"
#define ACPI_PM_PROP_PM_IO_BASE "pm_io_base"
#define ACPI_PM_PROP_GPE0_BLK "gpe0_blk"
#define ACPI_PM_PROP_GPE0_BLK_LEN "gpe0_blk_len"

struct PcGuestInfo {
    bool has_pci_info;
    bool isapc_ram_fw;
    hwaddr ram_size, ram_size_below_4g;
    unsigned apic_id_limit;
    bool apic_xrupt_override;
    uint64_t numa_nodes;
    uint64_t *node_mem;
    uint64_t *node_cpu;
    FWCfgState *fw_cfg;
    bool has_acpi_build;
};

/* parallel.c */
static inline bool parallel_init(ISABus *bus, int index, CharDriverState *chr)
{
    DeviceState *dev;
    ISADevice *isadev;

    isadev = isa_try_create(bus, "isa-parallel");
    if (!isadev) {
        return false;
    }
    dev = DEVICE(isadev);
    qdev_prop_set_uint32(dev, "index", index);
    qdev_prop_set_chr(dev, "chardev", chr);
    if (qdev_init(dev) < 0) {
        return false;
    }
    return true;
}

bool parallel_mm_init(MemoryRegion *address_space,
                      hwaddr base, int it_shift, qemu_irq irq,
                      CharDriverState *chr);

/* i8259.c */

extern DeviceState *isa_pic;
qemu_irq *i8259_init(ISABus *bus, qemu_irq parent_irq);
qemu_irq *kvm_i8259_init(ISABus *bus);
int pic_read_irq(DeviceState *d);
int pic_get_output(DeviceState *d);
void pic_info(Monitor *mon, const QDict *qdict);
void irq_info(Monitor *mon, const QDict *qdict);

/* Global System Interrupts */

#define GSI_NUM_PINS IOAPIC_NUM_PINS

typedef struct GSIState {
    qemu_irq i8259_irq[ISA_NUM_IRQS];
    qemu_irq ioapic_irq[IOAPIC_NUM_PINS];
} GSIState;

void gsi_handler(void *opaque, int n, int level);

/* vmport.c */
typedef uint32_t (VMPortReadFunc)(void *opaque, uint32_t address);

static inline void vmport_init(ISABus *bus)
{
    isa_create_simple(bus, "vmport");
}

void vmport_register(unsigned char command, VMPortReadFunc *func, void *opaque);
void vmmouse_get_data(uint32_t *data);
void vmmouse_set_data(const uint32_t *data);

/* pckbd.c */

void i8042_init(qemu_irq kbd_irq, qemu_irq mouse_irq, uint32_t io_base);
void i8042_mm_init(qemu_irq kbd_irq, qemu_irq mouse_irq,
                   MemoryRegion *region, ram_addr_t size,
                   hwaddr mask);
void i8042_isa_mouse_fake_event(void *opaque);
void i8042_setup_a20_line(ISADevice *dev, qemu_irq *a20_out);

/* pc.c */
extern int fd_bootchk;

void pc_register_ferr_irq(qemu_irq irq);
void pc_acpi_smi_interrupt(void *opaque, int irq, int level);

void pc_cpus_init(const char *cpu_model, DeviceState *icc_bridge);
void pc_hot_add_cpu(const int64_t id, Error **errp);
void pc_acpi_init(const char *default_dsdt);

PcGuestInfo *pc_guest_info_init(ram_addr_t below_4g_mem_size,
                                ram_addr_t above_4g_mem_size);

#define PCI_HOST_PROP_PCI_HOLE_START   "pci-hole-start"
#define PCI_HOST_PROP_PCI_HOLE_END     "pci-hole-end"
#define PCI_HOST_PROP_PCI_HOLE64_START "pci-hole64-start"
#define PCI_HOST_PROP_PCI_HOLE64_END   "pci-hole64-end"
#define PCI_HOST_PROP_PCI_HOLE64_SIZE  "pci-hole64-size"
#define DEFAULT_PCI_HOLE64_SIZE (~0x0ULL)


void pc_pci_as_mapping_init(Object *owner, MemoryRegion *system_memory,
                            MemoryRegion *pci_address_space);

FWCfgState *pc_memory_init(MemoryRegion *system_memory,
                           const char *kernel_filename,
                           const char *kernel_cmdline,
                           const char *initrd_filename,
                           ram_addr_t below_4g_mem_size,
                           ram_addr_t above_4g_mem_size,
                           MemoryRegion *rom_memory,
                           MemoryRegion **ram_memory,
                           PcGuestInfo *guest_info);
qemu_irq *pc_allocate_cpu_irq(void);
DeviceState *pc_vga_init(ISABus *isa_bus, PCIBus *pci_bus);
void pc_basic_device_init(ISABus *isa_bus, qemu_irq *gsi,
                          ISADevice **rtc_state,
                          ISADevice **floppy,
                          bool no_vmport,
                          uint32 hpet_irqs);
void pc_init_ne2k_isa(ISABus *bus, NICInfo *nd);
void pc_cmos_init(ram_addr_t ram_size, ram_addr_t above_4g_mem_size,
                  const char *boot_device,
                  ISADevice *floppy, BusState *ide0, BusState *ide1,
                  ISADevice *s);
void pc_nic_init(ISABus *isa_bus, PCIBus *pci_bus);
void pc_pci_device_init(PCIBus *pci_bus);

typedef void (*cpu_set_smm_t)(int smm, void *arg);
void cpu_smm_register(cpu_set_smm_t callback, void *arg);

void ioapic_init_gsi(GSIState *gsi_state, const char *parent_name);

/* acpi_piix.c */

I2CBus *piix4_pm_init(PCIBus *bus, int devfn, uint32_t smb_io_base,
                      qemu_irq sci_irq, qemu_irq smi_irq,
                      int kvm_enabled, FWCfgState *fw_cfg);
void piix4_smbus_register_device(SMBusDevice *dev, uint8_t addr);

/* hpet.c */
extern int no_hpet;

/* piix_pci.c */
struct PCII440FXState;
typedef struct PCII440FXState PCII440FXState;

PCIBus *i440fx_init(PCII440FXState **pi440fx_state, int *piix_devfn,
                    ISABus **isa_bus, qemu_irq *pic,
                    MemoryRegion *address_space_mem,
                    MemoryRegion *address_space_io,
                    ram_addr_t ram_size,
                    ram_addr_t below_4g_mem_size,
                    ram_addr_t above_4g_mem_size,
                    MemoryRegion *pci_memory,
                    MemoryRegion *ram_memory);

PCIBus *find_i440fx(void);
/* piix4.c */
extern PCIDevice *piix4_dev;
int piix4_init(PCIBus *bus, ISABus **isa_bus, int devfn);

/* vga.c */
enum vga_retrace_method {
    VGA_RETRACE_DUMB,
    VGA_RETRACE_PRECISE
};

extern enum vga_retrace_method vga_retrace_method;

int isa_vga_mm_init(hwaddr vram_base,
                    hwaddr ctrl_base, int it_shift,
                    MemoryRegion *address_space);

/* ne2000.c */
static inline bool isa_ne2000_init(ISABus *bus, int base, int irq, NICInfo *nd)
{
    DeviceState *dev;
    ISADevice *isadev;

    qemu_check_nic_model(nd, "ne2k_isa");

    isadev = isa_try_create(bus, "ne2k_isa");
    if (!isadev) {
        return false;
    }
    dev = DEVICE(isadev);
    qdev_prop_set_uint32(dev, "iobase", base);
    qdev_prop_set_uint32(dev, "irq",    irq);
    qdev_set_nic_properties(dev, nd);
    qdev_init_nofail(dev);
    return true;
}

/* pc_sysfw.c */
void pc_system_firmware_init(MemoryRegion *rom_memory,
                             bool isapc_ram_fw);

/* pvpanic.c */
uint16_t pvpanic_port(void);

/* e820 types */
#define E820_RAM        1
#define E820_RESERVED   2
#define E820_ACPI       3
#define E820_NVS        4
#define E820_UNUSABLE   5

int e820_add_entry(uint64_t, uint64_t, uint32_t);
int e820_get_num_entries(void);
bool e820_get_entry(int, uint32_t, uint64_t *, uint64_t *);

#define PC_Q35_COMPAT_2_0 \
        PC_COMPAT_2_0

#define PC_Q35_COMPAT_1_7 \
        PC_COMPAT_1_7, \
        PC_Q35_COMPAT_2_0, \
        {\
            .driver   = "hpet",\
            .property = HPET_INTCAP,\
            .value    = stringify(4),\
        }

#define PC_Q35_COMPAT_1_6 \
        PC_COMPAT_1_6, \
        PC_Q35_COMPAT_1_7

#define PC_Q35_COMPAT_1_5 \
        PC_COMPAT_1_5, \
        PC_Q35_COMPAT_1_6

#define PC_Q35_COMPAT_1_4 \
        PC_COMPAT_1_4, \
        PC_Q35_COMPAT_1_5

#define PC_COMPAT_2_0 \
        {\
        }

#define PC_COMPAT_1_7 \
        PC_COMPAT_2_0, \
        {\
            .driver   = TYPE_USB_DEVICE,\
            .property = "msos-desc",\
            .value    = "no",\
        },\
        {\
            .driver   = "PIIX4_PM",\
            .property = "acpi-pci-hotplug-with-bridge-support",\
            .value    = "off",\
        }

#define PC_COMPAT_1_6 \
        PC_COMPAT_1_7, \
        {\
            .driver   = "e1000",\
            .property = "mitigation",\
            .value    = "off",\
        },{\
            .driver   = "qemu64-" TYPE_X86_CPU,\
            .property = "model",\
            .value    = stringify(2),\
        },{\
            .driver   = "qemu32-" TYPE_X86_CPU,\
            .property = "model",\
            .value    = stringify(3),\
        },{\
            .driver   = "i440FX-pcihost",\
            .property = "short_root_bus",\
            .value    = stringify(1),\
        },{\
            .driver   = "q35-pcihost",\
            .property = "short_root_bus",\
            .value    = stringify(1),\
        }

#define PC_COMPAT_1_5 \
        PC_COMPAT_1_6, \
        {\
            .driver   = "Conroe-" TYPE_X86_CPU,\
            .property = "model",\
            .value    = stringify(2),\
        },{\
            .driver   = "Conroe-" TYPE_X86_CPU,\
            .property = "level",\
            .value    = stringify(2),\
        },{\
            .driver   = "Penryn-" TYPE_X86_CPU,\
            .property = "model",\
            .value    = stringify(2),\
        },{\
            .driver   = "Penryn-" TYPE_X86_CPU,\
            .property = "level",\
            .value    = stringify(2),\
        },{\
            .driver   = "Nehalem-" TYPE_X86_CPU,\
            .property = "model",\
            .value    = stringify(2),\
        },{\
            .driver   = "Nehalem-" TYPE_X86_CPU,\
            .property = "level",\
            .value    = stringify(2),\
        },{\
            .driver   = "virtio-net-pci",\
            .property = "any_layout",\
            .value    = "off",\
        },{\
            .driver = TYPE_X86_CPU,\
            .property = "pmu",\
            .value = "on",\
        },{\
            .driver   = "i440FX-pcihost",\
            .property = "short_root_bus",\
            .value    = stringify(0),\
        },{\
            .driver   = "q35-pcihost",\
            .property = "short_root_bus",\
            .value    = stringify(0),\
        }

#define PC_COMPAT_1_4 \
        PC_COMPAT_1_5, \
        {\
            .driver   = "scsi-hd",\
            .property = "discard_granularity",\
            .value    = stringify(0),\
	},{\
            .driver   = "scsi-cd",\
            .property = "discard_granularity",\
            .value    = stringify(0),\
	},{\
            .driver   = "scsi-disk",\
            .property = "discard_granularity",\
            .value    = stringify(0),\
	},{\
            .driver   = "ide-hd",\
            .property = "discard_granularity",\
            .value    = stringify(0),\
	},{\
            .driver   = "ide-cd",\
            .property = "discard_granularity",\
            .value    = stringify(0),\
	},{\
            .driver   = "ide-drive",\
            .property = "discard_granularity",\
            .value    = stringify(0),\
        },{\
            .driver   = "virtio-blk-pci",\
            .property = "discard_granularity",\
            .value    = stringify(0),\
	},{\
            .driver   = "virtio-serial-pci",\
            .property = "vectors",\
            /* DEV_NVECTORS_UNSPECIFIED as a uint32_t string */\
            .value    = stringify(0xFFFFFFFF),\
        },{ \
            .driver   = "virtio-net-pci", \
            .property = "ctrl_guest_offloads", \
            .value    = "off", \
        },{\
            .driver   = "e1000",\
            .property = "romfile",\
            .value    = "pxe-e1000.rom",\
        },{\
            .driver   = "ne2k_pci",\
            .property = "romfile",\
            .value    = "pxe-ne2k_pci.rom",\
        },{\
            .driver   = "pcnet",\
            .property = "romfile",\
            .value    = "pxe-pcnet.rom",\
        },{\
            .driver   = "rtl8139",\
            .property = "romfile",\
            .value    = "pxe-rtl8139.rom",\
        },{\
            .driver   = "virtio-net-pci",\
            .property = "romfile",\
            .value    = "pxe-virtio.rom",\
        },{\
            .driver   = "486-" TYPE_X86_CPU,\
            .property = "model",\
            .value    = stringify(0),\
        }

#define PC_COMMON_MACHINE_OPTIONS \
    .default_boot_order = "cad"

#define PC_DEFAULT_MACHINE_OPTIONS \
    PC_COMMON_MACHINE_OPTIONS, \
    .hot_add_cpu = pc_hot_add_cpu, \
    .max_cpus = 255

#endif
