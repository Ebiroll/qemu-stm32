/*
 * QEMU PowerPC 405 evaluation boards emulation
 *
 * Copyright (c) 2007 Jocelyn Mayer
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
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/datadir.h"
#include "cpu.h"
#include "hw/ppc/ppc.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "ppc405.h"
#include "hw/rtc/m48t59.h"
#include "hw/block/flash.h"
#include "sysemu/qtest.h"
#include "sysemu/reset.h"
#include "sysemu/block-backend.h"
#include "hw/boards.h"
#include "qemu/error-report.h"
#include "hw/loader.h"
#include "qemu/cutils.h"
#include "elf.h"

#define BIOS_FILENAME "ppc405_rom.bin"
#define BIOS_SIZE (2 * MiB)

#define KERNEL_LOAD_ADDR 0x01000000
#define INITRD_LOAD_ADDR 0x01800000

#define USE_FLASH_BIOS

#define TYPE_PPC405_MACHINE MACHINE_TYPE_NAME("ppc405")
OBJECT_DECLARE_SIMPLE_TYPE(Ppc405MachineState, PPC405_MACHINE);

struct Ppc405MachineState {
    /* Private */
    MachineState parent_obj;
    /* Public */

    Ppc405SoCState soc;
};

/*****************************************************************************/
/* PPC405EP reference board (IBM) */
/* Standalone board with:
 * - PowerPC 405EP CPU
 * - SDRAM (0x00000000)
 * - Flash (0xFFF80000)
 * - SRAM  (0xFFF00000)
 * - NVRAM (0xF0000000)
 * - FPGA  (0xF0300000)
 */
typedef struct ref405ep_fpga_t ref405ep_fpga_t;
struct ref405ep_fpga_t {
    uint8_t reg0;
    uint8_t reg1;
};

static uint64_t ref405ep_fpga_readb(void *opaque, hwaddr addr, unsigned size)
{
    ref405ep_fpga_t *fpga;
    uint32_t ret;

    fpga = opaque;
    switch (addr) {
    case 0x0:
        ret = fpga->reg0;
        break;
    case 0x1:
        ret = fpga->reg1;
        break;
    default:
        ret = 0;
        break;
    }

    return ret;
}

static void ref405ep_fpga_writeb(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    ref405ep_fpga_t *fpga;

    fpga = opaque;
    switch (addr) {
    case 0x0:
        /* Read only */
        break;
    case 0x1:
        fpga->reg1 = value;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps ref405ep_fpga_ops = {
    .read = ref405ep_fpga_readb,
    .write = ref405ep_fpga_writeb,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void ref405ep_fpga_reset (void *opaque)
{
    ref405ep_fpga_t *fpga;

    fpga = opaque;
    fpga->reg0 = 0x00;
    fpga->reg1 = 0x0F;
}

static void ref405ep_fpga_init(MemoryRegion *sysmem, uint32_t base)
{
    ref405ep_fpga_t *fpga;
    MemoryRegion *fpga_memory = g_new(MemoryRegion, 1);

    fpga = g_new0(ref405ep_fpga_t, 1);
    memory_region_init_io(fpga_memory, NULL, &ref405ep_fpga_ops, fpga,
                          "fpga", 0x00000100);
    memory_region_add_subregion(sysmem, base, fpga_memory);
    qemu_register_reset(&ref405ep_fpga_reset, fpga);
}

/*
 * CPU reset handler when booting directly from a loaded kernel
 */
static struct boot_info {
    uint32_t entry;
    uint32_t bdloc;
    uint32_t initrd_base;
    uint32_t initrd_size;
    uint32_t cmdline_base;
    uint32_t cmdline_size;
} boot_info;

static void main_cpu_reset(void *opaque)
{
    PowerPCCPU *cpu = opaque;
    CPUPPCState *env = &cpu->env;
    struct boot_info *bi = env->load_info;

    cpu_reset(CPU(cpu));

    /* stack: top of sram */
    env->gpr[1] = PPC405EP_SRAM_BASE + PPC405EP_SRAM_SIZE - 8;

    /* Tune our boot state */
    env->gpr[3] = bi->bdloc;
    env->gpr[4] = bi->initrd_base;
    env->gpr[5] = bi->initrd_base + bi->initrd_size;
    env->gpr[6] = bi->cmdline_base;
    env->gpr[7] = bi->cmdline_size;

    env->nip = bi->entry;
}

static void boot_from_kernel(MachineState *machine, PowerPCCPU *cpu)
{
    CPUPPCState *env = &cpu->env;
    hwaddr boot_entry;
    hwaddr kernel_base;
    int kernel_size;
    hwaddr initrd_base;
    int initrd_size;
    ram_addr_t bdloc;
    int len;

    bdloc = ppc405_set_bootinfo(env, machine->ram_size);
    boot_info.bdloc = bdloc;

    kernel_size = load_elf(machine->kernel_filename, NULL, NULL, NULL,
                           &boot_entry, &kernel_base, NULL, NULL,
                           1, PPC_ELF_MACHINE, 0, 0);
    if (kernel_size < 0) {
        error_report("Could not load kernel '%s' : %s",
                     machine->kernel_filename, load_elf_strerror(kernel_size));
        exit(1);
    }
    boot_info.entry = boot_entry;

    /* load initrd */
    if (machine->initrd_filename) {
        initrd_base = INITRD_LOAD_ADDR;
        initrd_size = load_image_targphys(machine->initrd_filename, initrd_base,
                                          machine->ram_size - initrd_base);
        if (initrd_size < 0) {
            error_report("could not load initial ram disk '%s'",
                         machine->initrd_filename);
            exit(1);
        }

        boot_info.initrd_base = initrd_base;
        boot_info.initrd_size = initrd_size;
    }

    if (machine->kernel_cmdline) {
        len = strlen(machine->kernel_cmdline);
        bdloc -= ((len + 255) & ~255);
        cpu_physical_memory_write(bdloc, machine->kernel_cmdline, len + 1);
        boot_info.cmdline_base = bdloc;
        boot_info.cmdline_size = bdloc + len;
    }

    /* Install our custom reset handler to start from Linux */
    qemu_register_reset(main_cpu_reset, cpu);
    env->load_info = &boot_info;
}

static void ppc405_init(MachineState *machine)
{
    Ppc405MachineState *ppc405 = PPC405_MACHINE(machine);
    MachineClass *mc = MACHINE_GET_CLASS(machine);
    const char *kernel_filename = machine->kernel_filename;
    MemoryRegion *sysmem = get_system_memory();

    if (machine->ram_size != mc->default_ram_size) {
        char *sz = size_to_str(mc->default_ram_size);
        error_report("Invalid RAM size, should be %s", sz);
        g_free(sz);
        exit(EXIT_FAILURE);
    }

    object_initialize_child(OBJECT(machine), "soc", &ppc405->soc,
                            TYPE_PPC405_SOC);
    object_property_set_uint(OBJECT(&ppc405->soc), "ram-size",
                             machine->ram_size, &error_fatal);
    object_property_set_link(OBJECT(&ppc405->soc), "dram",
                             OBJECT(machine->ram), &error_abort);
    object_property_set_bool(OBJECT(&ppc405->soc), "dram-init",
                             kernel_filename != NULL, &error_abort);
    object_property_set_uint(OBJECT(&ppc405->soc), "sys-clk", 33333333,
                             &error_abort);
    qdev_realize(DEVICE(&ppc405->soc), NULL, &error_fatal);

    /* allocate and load BIOS */
    if (machine->firmware) {
        MemoryRegion *bios = g_new(MemoryRegion, 1);
        g_autofree char *filename = qemu_find_file(QEMU_FILE_TYPE_BIOS,
                                                   machine->firmware);
        long bios_size;

        memory_region_init_rom(bios, NULL, "ef405ep.bios", BIOS_SIZE,
                               &error_fatal);

        if (!filename) {
            error_report("Could not find firmware '%s'", machine->firmware);
            exit(1);
        }

        bios_size = load_image_size(filename,
                                    memory_region_get_ram_ptr(bios),
                                    BIOS_SIZE);
        if (bios_size < 0) {
            error_report("Could not load PowerPC BIOS '%s'", machine->firmware);
            exit(1);
        }

        bios_size = (bios_size + 0xfff) & ~0xfff;
        memory_region_add_subregion(sysmem, (uint32_t)(-bios_size), bios);
    }

    /* Load kernel and initrd using U-Boot images */
    if (kernel_filename && machine->firmware) {
        target_ulong kernel_base, initrd_base;
        long kernel_size, initrd_size;

        kernel_base = KERNEL_LOAD_ADDR;
        kernel_size = load_image_targphys(kernel_filename, kernel_base,
                                          machine->ram_size - kernel_base);
        if (kernel_size < 0) {
            error_report("could not load kernel '%s'", kernel_filename);
            exit(1);
        }

        /* load initrd */
        if (machine->initrd_filename) {
            initrd_base = INITRD_LOAD_ADDR;
            initrd_size = load_image_targphys(machine->initrd_filename,
                                              initrd_base,
                                              machine->ram_size - initrd_base);
            if (initrd_size < 0) {
                error_report("could not load initial ram disk '%s'",
                             machine->initrd_filename);
                exit(1);
            }
        }

    /* Load ELF kernel and rootfs.cpio */
    } else if (kernel_filename && !machine->firmware) {
        boot_from_kernel(machine, ppc405->soc.cpu);
    }
}

static void ref405ep_init(MachineState *machine)
{
    DeviceState *dev;
    SysBusDevice *s;
    MemoryRegion *sram = g_new(MemoryRegion, 1);

    ppc405_init(machine);

    /* allocate SRAM */
    memory_region_init_ram(sram, NULL, "ref405ep.sram", PPC405EP_SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(get_system_memory(), PPC405EP_SRAM_BASE, sram);

    /* Register FPGA */
    ref405ep_fpga_init(get_system_memory(), PPC405EP_FPGA_BASE);
    /* Register NVRAM */
    dev = qdev_new("sysbus-m48t08");
    qdev_prop_set_int32(dev, "base-year", 1968);
    s = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, PPC405EP_NVRAM_BASE);
}

static void ref405ep_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "ref405ep";
    mc->init = ref405ep_init;
}

static const TypeInfo ref405ep_type = {
    .name = MACHINE_TYPE_NAME("ref405ep"),
    .parent = TYPE_PPC405_MACHINE,
    .class_init = ref405ep_class_init,
};

static void ppc405_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "PPC405 generic machine";
    mc->init = ppc405_init;
    mc->default_ram_size = 128 * MiB;
    mc->default_ram_id = "ppc405.ram";
}

static const TypeInfo ppc405_machine_type = {
    .name = TYPE_PPC405_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Ppc405MachineState),
    .class_init = ppc405_machine_class_init,
    .abstract = true,
};

static void ppc405_machine_init(void)
{
    type_register_static(&ppc405_machine_type);
    type_register_static(&ref405ep_type);
}

type_init(ppc405_machine_init)
