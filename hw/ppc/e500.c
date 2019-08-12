/*
 * QEMU PowerPC e500-based platforms
 *
 * Copyright (C) 2009 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Yu Liu,     <yu.liu@freescale.com>
 *
 * This file is derived from hw/ppc440_bamboo.c,
 * the copyright for that material belongs to the original owners.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of  the GNU General  Public License as published by
 * the Free Software Foundation;  either version 2 of the  License, or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "e500.h"
#include "e500-ccsr.h"
#include "net/net.h"
#include "qemu/config-file.h"
#include "hw/char/serial.h"
#include "hw/pci/pci.h"
#include "hw/boards.h"
#include "sysemu/sysemu.h"
#include "sysemu/kvm.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"
#include "kvm_ppc.h"
#include "sysemu/device_tree.h"
#include "hw/ppc/openpic.h"
#include "hw/ppc/openpic_kvm.h"
#include "hw/ppc/ppc.h"
#include "hw/qdev-properties.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/sysbus.h"
#include "exec/address-spaces.h"
#include "qemu/host-utils.h"
#include "qemu/option.h"
#include "hw/pci-host/ppce500.h"
#include "qemu/error-report.h"
#include "hw/platform-bus.h"
#include "hw/net/fsl_etsec/etsec.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"

#define EPAPR_MAGIC                (0x45504150)
#define BINARY_DEVICE_TREE_FILE    "mpc8544ds.dtb"
#define DTC_LOAD_PAD               0x1800000
#define DTC_PAD_MASK               0xFFFFF
#define DTB_MAX_SIZE               (8 * MiB)
#define INITRD_LOAD_PAD            0x2000000
#define INITRD_PAD_MASK            0xFFFFFF

#define RAM_SIZES_ALIGN            (64 * MiB)

/* TODO: parameterize */
#define MPC8544_CCSRBAR_SIZE       0x00100000ULL
#define MPC8544_MPIC_REGS_OFFSET   0x40000ULL
#define MPC8544_MSI_REGS_OFFSET   0x41600ULL
#define MPC8544_SERIAL0_REGS_OFFSET 0x4500ULL
#define MPC8544_SERIAL1_REGS_OFFSET 0x4600ULL
#define MPC8544_PCI_REGS_OFFSET    0x8000ULL
#define MPC8544_PCI_REGS_SIZE      0x1000ULL
#define MPC8544_UTIL_OFFSET        0xe0000ULL
#define MPC8XXX_GPIO_OFFSET        0x000FF000ULL
#define MPC8544_I2C_REGS_OFFSET    0x3000ULL
#define MPC8XXX_GPIO_IRQ           47
#define MPC8544_I2C_IRQ            43
#define RTC_REGS_OFFSET            0x68

struct boot_info
{
    uint32_t dt_base;
    uint32_t dt_size;
    uint32_t entry;
};

static uint32_t *pci_map_create(void *fdt, uint32_t mpic, int first_slot,
                                int nr_slots, int *len)
{
    int i = 0;
    int slot;
    int pci_irq;
    int host_irq;
    int last_slot = first_slot + nr_slots;
    uint32_t *pci_map;

    *len = nr_slots * 4 * 7 * sizeof(uint32_t);
    pci_map = g_malloc(*len);

    for (slot = first_slot; slot < last_slot; slot++) {
        for (pci_irq = 0; pci_irq < 4; pci_irq++) {
            pci_map[i++] = cpu_to_be32(slot << 11);
            pci_map[i++] = cpu_to_be32(0x0);
            pci_map[i++] = cpu_to_be32(0x0);
            pci_map[i++] = cpu_to_be32(pci_irq + 1);
            pci_map[i++] = cpu_to_be32(mpic);
            host_irq = ppce500_pci_map_irq_slot(slot, pci_irq);
            pci_map[i++] = cpu_to_be32(host_irq + 1);
            pci_map[i++] = cpu_to_be32(0x1);
        }
    }

    assert((i * sizeof(uint32_t)) == *len);

    return pci_map;
}

static void dt_serial_create(void *fdt, unsigned long long offset,
                             const char *soc, const char *mpic,
                             const char *alias, int idx, bool defcon)
{
    char *ser;

    ser = g_strdup_printf("%s/serial@%llx", soc, offset);
    qemu_fdt_add_subnode(fdt, ser);
    qemu_fdt_setprop_string(fdt, ser, "device_type", "serial");
    qemu_fdt_setprop_string(fdt, ser, "compatible", "ns16550");
    qemu_fdt_setprop_cells(fdt, ser, "reg", offset, 0x100);
    qemu_fdt_setprop_cell(fdt, ser, "cell-index", idx);
    qemu_fdt_setprop_cell(fdt, ser, "clock-frequency", 0);
    qemu_fdt_setprop_cells(fdt, ser, "interrupts", 42, 2);
    qemu_fdt_setprop_phandle(fdt, ser, "interrupt-parent", mpic);
    qemu_fdt_setprop_string(fdt, "/aliases", alias, ser);

    if (defcon) {
        /*
         * "linux,stdout-path" and "stdout" properties are deprecated by linux
         * kernel. New platforms should only use the "stdout-path" property. Set
         * the new property and continue using older property to remain
         * compatible with the existing firmware.
         */
        qemu_fdt_setprop_string(fdt, "/chosen", "linux,stdout-path", ser);
        qemu_fdt_setprop_string(fdt, "/chosen", "stdout-path", ser);
    }
    g_free(ser);
}

static void create_dt_mpc8xxx_gpio(void *fdt, const char *soc, const char *mpic)
{
    hwaddr mmio0 = MPC8XXX_GPIO_OFFSET;
    int irq0 = MPC8XXX_GPIO_IRQ;
    gchar *node = g_strdup_printf("%s/gpio@%"PRIx64, soc, mmio0);
    gchar *poweroff = g_strdup_printf("%s/power-off", soc);
    int gpio_ph;

    qemu_fdt_add_subnode(fdt, node);
    qemu_fdt_setprop_string(fdt, node, "compatible", "fsl,qoriq-gpio");
    qemu_fdt_setprop_cells(fdt, node, "reg", mmio0, 0x1000);
    qemu_fdt_setprop_cells(fdt, node, "interrupts", irq0, 0x2);
    qemu_fdt_setprop_phandle(fdt, node, "interrupt-parent", mpic);
    qemu_fdt_setprop_cells(fdt, node, "#gpio-cells", 2);
    qemu_fdt_setprop(fdt, node, "gpio-controller", NULL, 0);
    gpio_ph = qemu_fdt_alloc_phandle(fdt);
    qemu_fdt_setprop_cell(fdt, node, "phandle", gpio_ph);
    qemu_fdt_setprop_cell(fdt, node, "linux,phandle", gpio_ph);

    /* Power Off Pin */
    qemu_fdt_add_subnode(fdt, poweroff);
    qemu_fdt_setprop_string(fdt, poweroff, "compatible", "gpio-poweroff");
    qemu_fdt_setprop_cells(fdt, poweroff, "gpios", gpio_ph, 0, 0);

    g_free(node);
    g_free(poweroff);
}

static void dt_rtc_create(void *fdt, const char *i2c, const char *alias)
{
    int offset = RTC_REGS_OFFSET;

    gchar *rtc = g_strdup_printf("%s/rtc@%"PRIx32, i2c, offset);
    qemu_fdt_add_subnode(fdt, rtc);
    qemu_fdt_setprop_string(fdt, rtc, "compatible", "pericom,pt7c4338");
    qemu_fdt_setprop_cells(fdt, rtc, "reg", offset);
    qemu_fdt_setprop_string(fdt, "/aliases", alias, rtc);

    g_free(rtc);
}

static void dt_i2c_create(void *fdt, const char *soc, const char *mpic,
                             const char *alias)
{
    hwaddr mmio0 = MPC8544_I2C_REGS_OFFSET;
    int irq0 = MPC8544_I2C_IRQ;

    gchar *i2c = g_strdup_printf("%s/i2c@%"PRIx64, soc, mmio0);
    qemu_fdt_add_subnode(fdt, i2c);
    qemu_fdt_setprop_string(fdt, i2c, "device_type", "i2c");
    qemu_fdt_setprop_string(fdt, i2c, "compatible", "fsl-i2c");
    qemu_fdt_setprop_cells(fdt, i2c, "reg", mmio0, 0x14);
    qemu_fdt_setprop_cells(fdt, i2c, "cell-index", 0);
    qemu_fdt_setprop_cells(fdt, i2c, "interrupts", irq0, 0x2);
    qemu_fdt_setprop_phandle(fdt, i2c, "interrupt-parent", mpic);
    qemu_fdt_setprop_string(fdt, "/aliases", alias, i2c);

    g_free(i2c);
}


typedef struct PlatformDevtreeData {
    void *fdt;
    const char *mpic;
    int irq_start;
    const char *node;
    PlatformBusDevice *pbus;
} PlatformDevtreeData;

static int create_devtree_etsec(SysBusDevice *sbdev, PlatformDevtreeData *data)
{
    eTSEC *etsec = ETSEC_COMMON(sbdev);
    PlatformBusDevice *pbus = data->pbus;
    hwaddr mmio0 = platform_bus_get_mmio_addr(pbus, sbdev, 0);
    int irq0 = platform_bus_get_irqn(pbus, sbdev, 0);
    int irq1 = platform_bus_get_irqn(pbus, sbdev, 1);
    int irq2 = platform_bus_get_irqn(pbus, sbdev, 2);
    gchar *node = g_strdup_printf("/platform/ethernet@%"PRIx64, mmio0);
    gchar *group = g_strdup_printf("%s/queue-group", node);
    void *fdt = data->fdt;

    assert((int64_t)mmio0 >= 0);
    assert(irq0 >= 0);
    assert(irq1 >= 0);
    assert(irq2 >= 0);

    qemu_fdt_add_subnode(fdt, node);
    qemu_fdt_setprop_string(fdt, node, "device_type", "network");
    qemu_fdt_setprop_string(fdt, node, "compatible", "fsl,etsec2");
    qemu_fdt_setprop_string(fdt, node, "model", "eTSEC");
    qemu_fdt_setprop(fdt, node, "local-mac-address", etsec->conf.macaddr.a, 6);
    qemu_fdt_setprop_cells(fdt, node, "fixed-link", 0, 1, 1000, 0, 0);

    qemu_fdt_add_subnode(fdt, group);
    qemu_fdt_setprop_cells(fdt, group, "reg", mmio0, 0x1000);
    qemu_fdt_setprop_cells(fdt, group, "interrupts",
        data->irq_start + irq0, 0x2,
        data->irq_start + irq1, 0x2,
        data->irq_start + irq2, 0x2);

    g_free(node);
    g_free(group);

    return 0;
}

static void sysbus_device_create_devtree(SysBusDevice *sbdev, void *opaque)
{
    PlatformDevtreeData *data = opaque;
    bool matched = false;

    if (object_dynamic_cast(OBJECT(sbdev), TYPE_ETSEC_COMMON)) {
        create_devtree_etsec(sbdev, data);
        matched = true;
    }

    if (!matched) {
        error_report("Device %s is not supported by this machine yet.",
                     qdev_fw_name(DEVICE(sbdev)));
        exit(1);
    }
}

static void platform_bus_create_devtree(PPCE500MachineState *pms,
                                        void *fdt, const char *mpic)
{
    const PPCE500MachineClass *pmc = PPCE500_MACHINE_GET_CLASS(pms);
    gchar *node = g_strdup_printf("/platform@%"PRIx64, pmc->platform_bus_base);
    const char platcomp[] = "qemu,platform\0simple-bus";
    uint64_t addr = pmc->platform_bus_base;
    uint64_t size = pmc->platform_bus_size;
    int irq_start = pmc->platform_bus_first_irq;

    /* Create a /platform node that we can put all devices into */

    qemu_fdt_add_subnode(fdt, node);
    qemu_fdt_setprop(fdt, node, "compatible", platcomp, sizeof(platcomp));

    /* Our platform bus region is less than 32bit big, so 1 cell is enough for
       address and size */
    qemu_fdt_setprop_cells(fdt, node, "#size-cells", 1);
    qemu_fdt_setprop_cells(fdt, node, "#address-cells", 1);
    qemu_fdt_setprop_cells(fdt, node, "ranges", 0, addr >> 32, addr, size);

    qemu_fdt_setprop_phandle(fdt, node, "interrupt-parent", mpic);

    /* Create dt nodes for dynamic devices */
    PlatformDevtreeData data = {
        .fdt = fdt,
        .mpic = mpic,
        .irq_start = irq_start,
        .node = node,
        .pbus = pms->pbus_dev,
    };

    /* Loop through all dynamic sysbus devices and create nodes for them */
    foreach_dynamic_sysbus_device(sysbus_device_create_devtree, &data);

    g_free(node);
}

static int ppce500_load_device_tree(PPCE500MachineState *pms,
                                    hwaddr addr,
                                    hwaddr initrd_base,
                                    hwaddr initrd_size,
                                    hwaddr kernel_base,
                                    hwaddr kernel_size,
                                    bool dry_run)
{
    MachineState *machine = MACHINE(pms);
    unsigned int smp_cpus = machine->smp.cpus;
    const PPCE500MachineClass *pmc = PPCE500_MACHINE_GET_CLASS(pms);
    CPUPPCState *env = first_cpu->env_ptr;
    int ret = -1;
    uint64_t mem_reg_property[] = { 0, cpu_to_be64(machine->ram_size) };
    int fdt_size;
    void *fdt;
    uint8_t hypercall[16];
    uint32_t clock_freq = 400000000;
    uint32_t tb_freq = 400000000;
    int i;
    char compatible_sb[] = "fsl,mpc8544-immr\0simple-bus";
    char *soc;
    char *mpic;
    uint32_t mpic_ph;
    uint32_t msi_ph;
    char *gutil;
    char *pci;
    char *msi;
    uint32_t *pci_map = NULL;
    int len;
    uint32_t pci_ranges[14] =
        {
            0x2000000, 0x0, pmc->pci_mmio_bus_base,
            pmc->pci_mmio_base >> 32, pmc->pci_mmio_base,
            0x0, 0x20000000,

            0x1000000, 0x0, 0x0,
            pmc->pci_pio_base >> 32, pmc->pci_pio_base,
            0x0, 0x10000,
        };
    QemuOpts *machine_opts = qemu_get_machine_opts();
    const char *dtb_file = qemu_opt_get(machine_opts, "dtb");
    const char *toplevel_compat = qemu_opt_get(machine_opts, "dt_compatible");

    if (dtb_file) {
        char *filename;
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, dtb_file);
        if (!filename) {
            goto out;
        }

        fdt = load_device_tree(filename, &fdt_size);
        g_free(filename);
        if (!fdt) {
            goto out;
        }
        goto done;
    }

    fdt = create_device_tree(&fdt_size);
    if (fdt == NULL) {
        goto out;
    }

    /* Manipulate device tree in memory. */
    qemu_fdt_setprop_cell(fdt, "/", "#address-cells", 2);
    qemu_fdt_setprop_cell(fdt, "/", "#size-cells", 2);

    qemu_fdt_add_subnode(fdt, "/memory");
    qemu_fdt_setprop_string(fdt, "/memory", "device_type", "memory");
    qemu_fdt_setprop(fdt, "/memory", "reg", mem_reg_property,
                     sizeof(mem_reg_property));

    qemu_fdt_add_subnode(fdt, "/chosen");
    if (initrd_size) {
        ret = qemu_fdt_setprop_cell(fdt, "/chosen", "linux,initrd-start",
                                    initrd_base);
        if (ret < 0) {
            fprintf(stderr, "couldn't set /chosen/linux,initrd-start\n");
        }

        ret = qemu_fdt_setprop_cell(fdt, "/chosen", "linux,initrd-end",
                                    (initrd_base + initrd_size));
        if (ret < 0) {
            fprintf(stderr, "couldn't set /chosen/linux,initrd-end\n");
        }

    }

    if (kernel_base != -1ULL) {
        qemu_fdt_setprop_cells(fdt, "/chosen", "qemu,boot-kernel",
                                     kernel_base >> 32, kernel_base,
                                     kernel_size >> 32, kernel_size);
    }

    ret = qemu_fdt_setprop_string(fdt, "/chosen", "bootargs",
                                      machine->kernel_cmdline);
    if (ret < 0)
        fprintf(stderr, "couldn't set /chosen/bootargs\n");

    if (kvm_enabled()) {
        /* Read out host's frequencies */
        clock_freq = kvmppc_get_clockfreq();
        tb_freq = kvmppc_get_tbfreq();

        /* indicate KVM hypercall interface */
        qemu_fdt_add_subnode(fdt, "/hypervisor");
        qemu_fdt_setprop_string(fdt, "/hypervisor", "compatible",
                                "linux,kvm");
        kvmppc_get_hypercall(env, hypercall, sizeof(hypercall));
        qemu_fdt_setprop(fdt, "/hypervisor", "hcall-instructions",
                         hypercall, sizeof(hypercall));
        /* if KVM supports the idle hcall, set property indicating this */
        if (kvmppc_get_hasidle(env)) {
            qemu_fdt_setprop(fdt, "/hypervisor", "has-idle", NULL, 0);
        }
    }

    /* Create CPU nodes */
    qemu_fdt_add_subnode(fdt, "/cpus");
    qemu_fdt_setprop_cell(fdt, "/cpus", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#size-cells", 0);

    /* We need to generate the cpu nodes in reverse order, so Linux can pick
       the first node as boot node and be happy */
    for (i = smp_cpus - 1; i >= 0; i--) {
        CPUState *cpu;
        char *cpu_name;
        uint64_t cpu_release_addr = pmc->spin_base + (i * 0x20);

        cpu = qemu_get_cpu(i);
        if (cpu == NULL) {
            continue;
        }
        env = cpu->env_ptr;

        cpu_name = g_strdup_printf("/cpus/PowerPC,8544@%x", i);
        qemu_fdt_add_subnode(fdt, cpu_name);
        qemu_fdt_setprop_cell(fdt, cpu_name, "clock-frequency", clock_freq);
        qemu_fdt_setprop_cell(fdt, cpu_name, "timebase-frequency", tb_freq);
        qemu_fdt_setprop_string(fdt, cpu_name, "device_type", "cpu");
        qemu_fdt_setprop_cell(fdt, cpu_name, "reg", i);
        qemu_fdt_setprop_cell(fdt, cpu_name, "d-cache-line-size",
                              env->dcache_line_size);
        qemu_fdt_setprop_cell(fdt, cpu_name, "i-cache-line-size",
                              env->icache_line_size);
        qemu_fdt_setprop_cell(fdt, cpu_name, "d-cache-size", 0x8000);
        qemu_fdt_setprop_cell(fdt, cpu_name, "i-cache-size", 0x8000);
        qemu_fdt_setprop_cell(fdt, cpu_name, "bus-frequency", 0);
        if (cpu->cpu_index) {
            qemu_fdt_setprop_string(fdt, cpu_name, "status", "disabled");
            qemu_fdt_setprop_string(fdt, cpu_name, "enable-method",
                                    "spin-table");
            qemu_fdt_setprop_u64(fdt, cpu_name, "cpu-release-addr",
                                 cpu_release_addr);
        } else {
            qemu_fdt_setprop_string(fdt, cpu_name, "status", "okay");
        }
        g_free(cpu_name);
    }

    qemu_fdt_add_subnode(fdt, "/aliases");
    /* XXX These should go into their respective devices' code */
    soc = g_strdup_printf("/soc@%"PRIx64, pmc->ccsrbar_base);
    qemu_fdt_add_subnode(fdt, soc);
    qemu_fdt_setprop_string(fdt, soc, "device_type", "soc");
    qemu_fdt_setprop(fdt, soc, "compatible", compatible_sb,
                     sizeof(compatible_sb));
    qemu_fdt_setprop_cell(fdt, soc, "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, soc, "#size-cells", 1);
    qemu_fdt_setprop_cells(fdt, soc, "ranges", 0x0,
                           pmc->ccsrbar_base >> 32, pmc->ccsrbar_base,
                           MPC8544_CCSRBAR_SIZE);
    /* XXX should contain a reasonable value */
    qemu_fdt_setprop_cell(fdt, soc, "bus-frequency", 0);

    mpic = g_strdup_printf("%s/pic@%llx", soc, MPC8544_MPIC_REGS_OFFSET);
    qemu_fdt_add_subnode(fdt, mpic);
    qemu_fdt_setprop_string(fdt, mpic, "device_type", "open-pic");
    qemu_fdt_setprop_string(fdt, mpic, "compatible", "fsl,mpic");
    qemu_fdt_setprop_cells(fdt, mpic, "reg", MPC8544_MPIC_REGS_OFFSET,
                           0x40000);
    qemu_fdt_setprop_cell(fdt, mpic, "#address-cells", 0);
    qemu_fdt_setprop_cell(fdt, mpic, "#interrupt-cells", 2);
    mpic_ph = qemu_fdt_alloc_phandle(fdt);
    qemu_fdt_setprop_cell(fdt, mpic, "phandle", mpic_ph);
    qemu_fdt_setprop_cell(fdt, mpic, "linux,phandle", mpic_ph);
    qemu_fdt_setprop(fdt, mpic, "interrupt-controller", NULL, 0);

    /*
     * We have to generate ser1 first, because Linux takes the first
     * device it finds in the dt as serial output device. And we generate
     * devices in reverse order to the dt.
     */
    if (serial_hd(1)) {
        dt_serial_create(fdt, MPC8544_SERIAL1_REGS_OFFSET,
                         soc, mpic, "serial1", 1, false);
    }

    if (serial_hd(0)) {
        dt_serial_create(fdt, MPC8544_SERIAL0_REGS_OFFSET,
                         soc, mpic, "serial0", 0, true);
    }

    /* i2c */
    dt_i2c_create(fdt, soc, mpic, "i2c");

    dt_rtc_create(fdt, "i2c", "rtc");


    gutil = g_strdup_printf("%s/global-utilities@%llx", soc,
                            MPC8544_UTIL_OFFSET);
    qemu_fdt_add_subnode(fdt, gutil);
    qemu_fdt_setprop_string(fdt, gutil, "compatible", "fsl,mpc8544-guts");
    qemu_fdt_setprop_cells(fdt, gutil, "reg", MPC8544_UTIL_OFFSET, 0x1000);
    qemu_fdt_setprop(fdt, gutil, "fsl,has-rstcr", NULL, 0);
    g_free(gutil);

    msi = g_strdup_printf("/%s/msi@%llx", soc, MPC8544_MSI_REGS_OFFSET);
    qemu_fdt_add_subnode(fdt, msi);
    qemu_fdt_setprop_string(fdt, msi, "compatible", "fsl,mpic-msi");
    qemu_fdt_setprop_cells(fdt, msi, "reg", MPC8544_MSI_REGS_OFFSET, 0x200);
    msi_ph = qemu_fdt_alloc_phandle(fdt);
    qemu_fdt_setprop_cells(fdt, msi, "msi-available-ranges", 0x0, 0x100);
    qemu_fdt_setprop_phandle(fdt, msi, "interrupt-parent", mpic);
    qemu_fdt_setprop_cells(fdt, msi, "interrupts",
        0xe0, 0x0,
        0xe1, 0x0,
        0xe2, 0x0,
        0xe3, 0x0,
        0xe4, 0x0,
        0xe5, 0x0,
        0xe6, 0x0,
        0xe7, 0x0);
    qemu_fdt_setprop_cell(fdt, msi, "phandle", msi_ph);
    qemu_fdt_setprop_cell(fdt, msi, "linux,phandle", msi_ph);
    g_free(msi);

    pci = g_strdup_printf("/pci@%llx",
                          pmc->ccsrbar_base + MPC8544_PCI_REGS_OFFSET);
    qemu_fdt_add_subnode(fdt, pci);
    qemu_fdt_setprop_cell(fdt, pci, "cell-index", 0);
    qemu_fdt_setprop_string(fdt, pci, "compatible", "fsl,mpc8540-pci");
    qemu_fdt_setprop_string(fdt, pci, "device_type", "pci");
    qemu_fdt_setprop_cells(fdt, pci, "interrupt-map-mask", 0xf800, 0x0,
                           0x0, 0x7);
    pci_map = pci_map_create(fdt, qemu_fdt_get_phandle(fdt, mpic),
                             pmc->pci_first_slot, pmc->pci_nr_slots,
                             &len);
    qemu_fdt_setprop(fdt, pci, "interrupt-map", pci_map, len);
    qemu_fdt_setprop_phandle(fdt, pci, "interrupt-parent", mpic);
    qemu_fdt_setprop_cells(fdt, pci, "interrupts", 24, 2);
    qemu_fdt_setprop_cells(fdt, pci, "bus-range", 0, 255);
    for (i = 0; i < 14; i++) {
        pci_ranges[i] = cpu_to_be32(pci_ranges[i]);
    }
    qemu_fdt_setprop_cell(fdt, pci, "fsl,msi", msi_ph);
    qemu_fdt_setprop(fdt, pci, "ranges", pci_ranges, sizeof(pci_ranges));
    qemu_fdt_setprop_cells(fdt, pci, "reg",
                           (pmc->ccsrbar_base + MPC8544_PCI_REGS_OFFSET) >> 32,
                           (pmc->ccsrbar_base + MPC8544_PCI_REGS_OFFSET),
                           0, 0x1000);
    qemu_fdt_setprop_cell(fdt, pci, "clock-frequency", 66666666);
    qemu_fdt_setprop_cell(fdt, pci, "#interrupt-cells", 1);
    qemu_fdt_setprop_cell(fdt, pci, "#size-cells", 2);
    qemu_fdt_setprop_cell(fdt, pci, "#address-cells", 3);
    qemu_fdt_setprop_string(fdt, "/aliases", "pci0", pci);
    g_free(pci);

    if (pmc->has_mpc8xxx_gpio) {
        create_dt_mpc8xxx_gpio(fdt, soc, mpic);
    }
    g_free(soc);

    if (pms->pbus_dev) {
        platform_bus_create_devtree(pms, fdt, mpic);
    }
    g_free(mpic);

    pmc->fixup_devtree(fdt);

    if (toplevel_compat) {
        qemu_fdt_setprop(fdt, "/", "compatible", toplevel_compat,
                         strlen(toplevel_compat) + 1);
    }

done:
    if (!dry_run) {
        qemu_fdt_dumpdtb(fdt, fdt_size);
        cpu_physical_memory_write(addr, fdt, fdt_size);
    }
    ret = fdt_size;

out:
    g_free(pci_map);

    return ret;
}

typedef struct DeviceTreeParams {
    PPCE500MachineState *machine;
    hwaddr addr;
    hwaddr initrd_base;
    hwaddr initrd_size;
    hwaddr kernel_base;
    hwaddr kernel_size;
    Notifier notifier;
} DeviceTreeParams;

static void ppce500_reset_device_tree(void *opaque)
{
    DeviceTreeParams *p = opaque;
    ppce500_load_device_tree(p->machine, p->addr, p->initrd_base,
                             p->initrd_size, p->kernel_base, p->kernel_size,
                             false);
}

static void ppce500_init_notify(Notifier *notifier, void *data)
{
    DeviceTreeParams *p = container_of(notifier, DeviceTreeParams, notifier);
    ppce500_reset_device_tree(p);
}

static int ppce500_prep_device_tree(PPCE500MachineState *machine,
                                    hwaddr addr,
                                    hwaddr initrd_base,
                                    hwaddr initrd_size,
                                    hwaddr kernel_base,
                                    hwaddr kernel_size)
{
    DeviceTreeParams *p = g_new(DeviceTreeParams, 1);
    p->machine = machine;
    p->addr = addr;
    p->initrd_base = initrd_base;
    p->initrd_size = initrd_size;
    p->kernel_base = kernel_base;
    p->kernel_size = kernel_size;

    qemu_register_reset(ppce500_reset_device_tree, p);
    p->notifier.notify = ppce500_init_notify;
    qemu_add_machine_init_done_notifier(&p->notifier);

    /* Issue the device tree loader once, so that we get the size of the blob */
    return ppce500_load_device_tree(machine, addr, initrd_base, initrd_size,
                                    kernel_base, kernel_size, true);
}

/* Create -kernel TLB entries for BookE.  */
hwaddr booke206_page_size_to_tlb(uint64_t size)
{
    return 63 - clz64(size / KiB);
}

static int booke206_initial_map_tsize(CPUPPCState *env)
{
    struct boot_info *bi = env->load_info;
    hwaddr dt_end;
    int ps;

    /* Our initial TLB entry needs to cover everything from 0 to
       the device tree top */
    dt_end = bi->dt_base + bi->dt_size;
    ps = booke206_page_size_to_tlb(dt_end) + 1;
    if (ps & 1) {
        /* e500v2 can only do even TLB size bits */
        ps++;
    }
    return ps;
}

static uint64_t mmubooke_initial_mapsize(CPUPPCState *env)
{
    int tsize;

    tsize = booke206_initial_map_tsize(env);
    return (1ULL << 10 << tsize);
}

static void mmubooke_create_initial_mapping(CPUPPCState *env)
{
    ppcmas_tlb_t *tlb = booke206_get_tlbm(env, 1, 0, 0);
    hwaddr size;
    int ps;

    ps = booke206_initial_map_tsize(env);
    size = (ps << MAS1_TSIZE_SHIFT);
    tlb->mas1 = MAS1_VALID | size;
    tlb->mas2 = 0;
    tlb->mas7_3 = 0;
    tlb->mas7_3 |= MAS3_UR | MAS3_UW | MAS3_UX | MAS3_SR | MAS3_SW | MAS3_SX;

    env->tlb_dirty = true;
}

static void ppce500_cpu_reset_sec(void *opaque)
{
    PowerPCCPU *cpu = opaque;
    CPUState *cs = CPU(cpu);

    cpu_reset(cs);

    /* Secondary CPU starts in halted state for now. Needs to change when
       implementing non-kernel boot. */
    cs->halted = 1;
    cs->exception_index = EXCP_HLT;
}

static void ppce500_cpu_reset(void *opaque)
{
    PowerPCCPU *cpu = opaque;
    CPUState *cs = CPU(cpu);
    CPUPPCState *env = &cpu->env;
    struct boot_info *bi = env->load_info;

    cpu_reset(cs);

    /* Set initial guest state. */
    cs->halted = 0;
    env->gpr[1] = (16 * MiB) - 8;
    env->gpr[3] = bi->dt_base;
    env->gpr[4] = 0;
    env->gpr[5] = 0;
    env->gpr[6] = EPAPR_MAGIC;
    env->gpr[7] = mmubooke_initial_mapsize(env);
    env->gpr[8] = 0;
    env->gpr[9] = 0;
    env->nip = bi->entry;
    mmubooke_create_initial_mapping(env);
}

static DeviceState *ppce500_init_mpic_qemu(PPCE500MachineState *pms,
                                           IrqLines  *irqs)
{
    DeviceState *dev;
    SysBusDevice *s;
    int i, j, k;
    MachineState *machine = MACHINE(pms);
    unsigned int smp_cpus = machine->smp.cpus;
    const PPCE500MachineClass *pmc = PPCE500_MACHINE_GET_CLASS(pms);

    dev = qdev_create(NULL, TYPE_OPENPIC);
    object_property_add_child(OBJECT(machine), "pic", OBJECT(dev),
                              &error_fatal);
    qdev_prop_set_uint32(dev, "model", pmc->mpic_version);
    qdev_prop_set_uint32(dev, "nb_cpus", smp_cpus);

    qdev_init_nofail(dev);
    s = SYS_BUS_DEVICE(dev);

    k = 0;
    for (i = 0; i < smp_cpus; i++) {
        for (j = 0; j < OPENPIC_OUTPUT_NB; j++) {
            sysbus_connect_irq(s, k++, irqs[i].irq[j]);
        }
    }

    return dev;
}

static DeviceState *ppce500_init_mpic_kvm(const PPCE500MachineClass *pmc,
                                          IrqLines *irqs, Error **errp)
{
    Error *err = NULL;
    DeviceState *dev;
    CPUState *cs;

    dev = qdev_create(NULL, TYPE_KVM_OPENPIC);
    qdev_prop_set_uint32(dev, "model", pmc->mpic_version);

    object_property_set_bool(OBJECT(dev), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        object_unparent(OBJECT(dev));
        return NULL;
    }

    CPU_FOREACH(cs) {
        if (kvm_openpic_connect_vcpu(dev, cs)) {
            fprintf(stderr, "%s: failed to connect vcpu to irqchip\n",
                    __func__);
            abort();
        }
    }

    return dev;
}

static DeviceState *ppce500_init_mpic(PPCE500MachineState *pms,
                                      MemoryRegion *ccsr,
                                      IrqLines *irqs)
{
    MachineState *machine = MACHINE(pms);
    const PPCE500MachineClass *pmc = PPCE500_MACHINE_GET_CLASS(pms);
    DeviceState *dev = NULL;
    SysBusDevice *s;

    if (kvm_enabled()) {
        Error *err = NULL;

        if (machine_kernel_irqchip_allowed(machine)) {
            dev = ppce500_init_mpic_kvm(pmc, irqs, &err);
        }
        if (machine_kernel_irqchip_required(machine) && !dev) {
            error_reportf_err(err,
                              "kernel_irqchip requested but unavailable: ");
            exit(1);
        }
    }

    if (!dev) {
        dev = ppce500_init_mpic_qemu(pms, irqs);
    }

    s = SYS_BUS_DEVICE(dev);
    memory_region_add_subregion(ccsr, MPC8544_MPIC_REGS_OFFSET,
                                s->mmio[0].memory);

    return dev;
}

static void ppce500_power_off(void *opaque, int line, int on)
{
    if (on) {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    }
}

void ppce500_init(MachineState *machine)
{
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    PPCE500MachineState *pms = PPCE500_MACHINE(machine);
    const PPCE500MachineClass *pmc = PPCE500_MACHINE_GET_CLASS(machine);
    PCIBus *pci_bus;
    CPUPPCState *env = NULL;
    uint64_t loadaddr;
    hwaddr kernel_base = -1LL;
    int kernel_size = 0;
    hwaddr dt_base = 0;
    hwaddr initrd_base = 0;
    int initrd_size = 0;
    hwaddr cur_base = 0;
    char *filename;
    const char *payload_name;
    bool kernel_as_payload;
    hwaddr bios_entry = 0;
    target_long payload_size;
    struct boot_info *boot_info;
    int dt_size;
    int i;
    unsigned int smp_cpus = machine->smp.cpus;
    /* irq num for pin INTA, INTB, INTC and INTD is 1, 2, 3 and
     * 4 respectively */
    unsigned int pci_irq_nrs[PCI_NUM_PINS] = {1, 2, 3, 4};
    IrqLines *irqs;
    DeviceState *dev, *mpicdev;
    CPUPPCState *firstenv = NULL;
    MemoryRegion *ccsr_addr_space;
    SysBusDevice *s;
    PPCE500CCSRState *ccsr;
    I2CBus *i2c;

    irqs = g_new0(IrqLines, smp_cpus);
    for (i = 0; i < smp_cpus; i++) {
        PowerPCCPU *cpu;
        CPUState *cs;
        qemu_irq *input;

        cpu = POWERPC_CPU(cpu_create(machine->cpu_type));
        env = &cpu->env;
        cs = CPU(cpu);

        if (env->mmu_model != POWERPC_MMU_BOOKE206) {
            error_report("MMU model %i not supported by this machine",
                         env->mmu_model);
            exit(1);
        }

        if (!firstenv) {
            firstenv = env;
        }

        input = (qemu_irq *)env->irq_inputs;
        irqs[i].irq[OPENPIC_OUTPUT_INT] = input[PPCE500_INPUT_INT];
        irqs[i].irq[OPENPIC_OUTPUT_CINT] = input[PPCE500_INPUT_CINT];
        env->spr_cb[SPR_BOOKE_PIR].default_value = cs->cpu_index = i;
        env->mpic_iack = pmc->ccsrbar_base + MPC8544_MPIC_REGS_OFFSET + 0xa0;

        ppc_booke_timers_init(cpu, 400000000, PPC_TIMER_E500);

        /* Register reset handler */
        if (!i) {
            /* Primary CPU */
            struct boot_info *boot_info;
            boot_info = g_malloc0(sizeof(struct boot_info));
            qemu_register_reset(ppce500_cpu_reset, cpu);
            env->load_info = boot_info;
        } else {
            /* Secondary CPUs */
            qemu_register_reset(ppce500_cpu_reset_sec, cpu);
        }
    }

    env = firstenv;

    /* Fixup Memory size on a alignment boundary */
    ram_size &= ~(RAM_SIZES_ALIGN - 1);
    machine->ram_size = ram_size;

    /* Register Memory */
    memory_region_allocate_system_memory(ram, NULL, "mpc8544ds.ram", ram_size);
    memory_region_add_subregion(address_space_mem, 0, ram);

    dev = qdev_create(NULL, "e500-ccsr");
    object_property_add_child(qdev_get_machine(), "e500-ccsr",
                              OBJECT(dev), NULL);
    qdev_init_nofail(dev);
    ccsr = CCSR(dev);
    ccsr_addr_space = &ccsr->ccsr_space;
    memory_region_add_subregion(address_space_mem, pmc->ccsrbar_base,
                                ccsr_addr_space);

    mpicdev = ppce500_init_mpic(pms, ccsr_addr_space, irqs);

    /* Serial */
    if (serial_hd(0)) {
        serial_mm_init(ccsr_addr_space, MPC8544_SERIAL0_REGS_OFFSET,
                       0, qdev_get_gpio_in(mpicdev, 42), 399193,
                       serial_hd(0), DEVICE_BIG_ENDIAN);
    }

    if (serial_hd(1)) {
        serial_mm_init(ccsr_addr_space, MPC8544_SERIAL1_REGS_OFFSET,
                       0, qdev_get_gpio_in(mpicdev, 42), 399193,
                       serial_hd(1), DEVICE_BIG_ENDIAN);
    }
        /* I2C */
    dev = qdev_create(NULL, "mpc-i2c");
    s = SYS_BUS_DEVICE(dev);
    qdev_init_nofail(dev);
    sysbus_connect_irq(s, 0, qdev_get_gpio_in(mpicdev, MPC8544_I2C_IRQ));
    memory_region_add_subregion(ccsr_addr_space, MPC8544_I2C_REGS_OFFSET,
                                sysbus_mmio_get_region(s, 0));
    i2c = (I2CBus *)qdev_get_child_bus(dev, "i2c");
    i2c_create_slave(i2c, "ds1338", RTC_REGS_OFFSET);


    /* General Utility device */
    dev = qdev_create(NULL, "mpc8544-guts");
    qdev_init_nofail(dev);
    s = SYS_BUS_DEVICE(dev);
    memory_region_add_subregion(ccsr_addr_space, MPC8544_UTIL_OFFSET,
                                sysbus_mmio_get_region(s, 0));

    /* PCI */
    dev = qdev_create(NULL, "e500-pcihost");
    object_property_add_child(qdev_get_machine(), "pci-host", OBJECT(dev),
                              &error_abort);
    qdev_prop_set_uint32(dev, "first_slot", pmc->pci_first_slot);
    qdev_prop_set_uint32(dev, "first_pin_irq", pci_irq_nrs[0]);
    qdev_init_nofail(dev);
    s = SYS_BUS_DEVICE(dev);
    for (i = 0; i < PCI_NUM_PINS; i++) {
        sysbus_connect_irq(s, i, qdev_get_gpio_in(mpicdev, pci_irq_nrs[i]));
    }

    memory_region_add_subregion(ccsr_addr_space, MPC8544_PCI_REGS_OFFSET,
                                sysbus_mmio_get_region(s, 0));

    pci_bus = (PCIBus *)qdev_get_child_bus(dev, "pci.0");
    if (!pci_bus)
        printf("couldn't create PCI controller!\n");

    if (pci_bus) {
        /* Register network interfaces. */
        for (i = 0; i < nb_nics; i++) {
            pci_nic_init_nofail(&nd_table[i], pci_bus, "virtio-net-pci", NULL);
        }
    }

    /* Register spinning region */
    sysbus_create_simple("e500-spin", pmc->spin_base, NULL);

    if (pmc->has_mpc8xxx_gpio) {
        qemu_irq poweroff_irq;

        dev = qdev_create(NULL, "mpc8xxx_gpio");
        s = SYS_BUS_DEVICE(dev);
        qdev_init_nofail(dev);
        sysbus_connect_irq(s, 0, qdev_get_gpio_in(mpicdev, MPC8XXX_GPIO_IRQ));
        memory_region_add_subregion(ccsr_addr_space, MPC8XXX_GPIO_OFFSET,
                                    sysbus_mmio_get_region(s, 0));

        /* Power Off GPIO at Pin 0 */
        poweroff_irq = qemu_allocate_irq(ppce500_power_off, NULL, 0);
        qdev_connect_gpio_out(dev, 0, poweroff_irq);
    }

    /* Platform Bus Device */
    if (pmc->has_platform_bus) {
        dev = qdev_create(NULL, TYPE_PLATFORM_BUS_DEVICE);
        dev->id = TYPE_PLATFORM_BUS_DEVICE;
        qdev_prop_set_uint32(dev, "num_irqs", pmc->platform_bus_num_irqs);
        qdev_prop_set_uint32(dev, "mmio_size", pmc->platform_bus_size);
        qdev_init_nofail(dev);
        pms->pbus_dev = PLATFORM_BUS_DEVICE(dev);

        s = SYS_BUS_DEVICE(pms->pbus_dev);
        for (i = 0; i < pmc->platform_bus_num_irqs; i++) {
            int irqn = pmc->platform_bus_first_irq + i;
            sysbus_connect_irq(s, i, qdev_get_gpio_in(mpicdev, irqn));
        }

        memory_region_add_subregion(address_space_mem,
                                    pmc->platform_bus_base,
                                    sysbus_mmio_get_region(s, 0));
    }

    /*
     * Smart firmware defaults ahead!
     *
     * We follow the following table to select which payload we execute.
     *
     *  -kernel | -bios | payload
     * ---------+-------+---------
     *     N    |   Y   | u-boot
     *     N    |   N   | u-boot
     *     Y    |   Y   | u-boot
     *     Y    |   N   | kernel
     *
     * This ensures backwards compatibility with how we used to expose
     * -kernel to users but allows them to run through u-boot as well.
     */
    kernel_as_payload = false;
    if (bios_name == NULL) {
        if (machine->kernel_filename) {
            payload_name = machine->kernel_filename;
            kernel_as_payload = true;
        } else {
            payload_name = "u-boot.e500";
        }
    } else {
        payload_name = bios_name;
    }

    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, payload_name);

    payload_size = load_elf(filename, NULL, NULL, NULL,
                            &bios_entry, &loadaddr, NULL,
                            1, PPC_ELF_MACHINE, 0, 0);
    if (payload_size < 0) {
        /*
         * Hrm. No ELF image? Try a uImage, maybe someone is giving us an
         * ePAPR compliant kernel
         */
        loadaddr = LOAD_UIMAGE_LOADADDR_INVALID;
        payload_size = load_uimage(filename, &bios_entry, &loadaddr, NULL,
                                   NULL, NULL);
        if (payload_size < 0) {
            error_report("could not load firmware '%s'", filename);
            exit(1);
        }
    }

    g_free(filename);

    if (kernel_as_payload) {
        kernel_base = loadaddr;
        kernel_size = payload_size;
    }

    cur_base = loadaddr + payload_size;
    if (cur_base < 32 * MiB) {
        /* u-boot occupies memory up to 32MB, so load blobs above */
        cur_base = 32 * MiB;
    }

    /* Load bare kernel only if no bios/u-boot has been provided */
    if (machine->kernel_filename && !kernel_as_payload) {
        kernel_base = cur_base;
        kernel_size = load_image_targphys(machine->kernel_filename,
                                          cur_base,
                                          ram_size - cur_base);
        if (kernel_size < 0) {
            error_report("could not load kernel '%s'",
                         machine->kernel_filename);
            exit(1);
        }

        cur_base += kernel_size;
    }

    /* Load initrd. */
    if (machine->initrd_filename) {
        initrd_base = (cur_base + INITRD_LOAD_PAD) & ~INITRD_PAD_MASK;
        initrd_size = load_image_targphys(machine->initrd_filename, initrd_base,
                                          ram_size - initrd_base);

        if (initrd_size < 0) {
            error_report("could not load initial ram disk '%s'",
                         machine->initrd_filename);
            exit(1);
        }

        cur_base = initrd_base + initrd_size;
    }

    /*
     * Reserve space for dtb behind the kernel image because Linux has a bug
     * where it can only handle the dtb if it's within the first 64MB of where
     * <kernel> starts. dtb cannot not reach initrd_base because INITRD_LOAD_PAD
     * ensures enough space between kernel and initrd.
     */
    dt_base = (loadaddr + payload_size + DTC_LOAD_PAD) & ~DTC_PAD_MASK;
    if (dt_base + DTB_MAX_SIZE > ram_size) {
            error_report("not enough memory for device tree");
            exit(1);
    }

    dt_size = ppce500_prep_device_tree(pms, dt_base,
                                       initrd_base, initrd_size,
                                       kernel_base, kernel_size);
    if (dt_size < 0) {
        error_report("couldn't load device tree");
        exit(1);
    }
    assert(dt_size < DTB_MAX_SIZE);

    boot_info = env->load_info;
    boot_info->entry = bios_entry;
    boot_info->dt_base = dt_base;
    boot_info->dt_size = dt_size;
}

static void e500_ccsr_initfn(Object *obj)
{
    PPCE500CCSRState *ccsr = CCSR(obj);
    memory_region_init(&ccsr->ccsr_space, obj, "e500-ccsr",
                       MPC8544_CCSRBAR_SIZE);
}

static const TypeInfo e500_ccsr_info = {
    .name          = TYPE_CCSR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PPCE500CCSRState),
    .instance_init = e500_ccsr_initfn,
};

static const TypeInfo ppce500_info = {
    .name          = TYPE_PPCE500_MACHINE,
    .parent        = TYPE_MACHINE,
    .abstract      = true,
    .instance_size = sizeof(PPCE500MachineState),
    .class_size    = sizeof(PPCE500MachineClass),
};

static void e500_register_types(void)
{
    type_register_static(&e500_ccsr_info);
    type_register_static(&ppce500_info);
}

type_init(e500_register_types)
