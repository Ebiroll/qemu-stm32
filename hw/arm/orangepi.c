/*
 * Orange Pi emulation
 *
 * Copyright (C) 2019 Niek Linnenbank <nieklinnenbank@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "exec/address-spaces.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "hw/arm/allwinner-h3.h"
#include "sysemu/sysemu.h"

static struct arm_boot_info orangepi_binfo = {
    .nb_cpus = AW_H3_NUM_CPUS,
};

static void orangepi_init(MachineState *machine)
{
    AwH3State *h3;
    DriveInfo *di;
    BlockBackend *blk;
    BusState *bus;
    DeviceState *carddev;

    /* BIOS is not supported by this board */
    if (bios_name) {
        error_report("BIOS not supported for this machine");
        exit(1);
    }

    /* This board has fixed size RAM */
    if (machine->ram_size != 1 * GiB) {
        error_report("This machine can only be used with 1GiB of RAM");
        exit(1);
    }

    /* Only allow Cortex-A7 for this board */
    if (strcmp(machine->cpu_type, ARM_CPU_TYPE_NAME("cortex-a7")) != 0) {
        error_report("This board can only be used with cortex-a7 CPU");
        exit(1);
    }

    h3 = AW_H3(object_new(TYPE_AW_H3));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(h3));
    object_unref(OBJECT(h3));

    /* Setup timer properties */
    object_property_set_int(OBJECT(h3), 32768, "clk0-freq",
                            &error_abort);
    object_property_set_int(OBJECT(h3), 24 * 1000 * 1000, "clk1-freq",
                            &error_abort);

    /* Setup SID properties. Currently using a default fixed SID identifier. */
    if (qemu_uuid_is_null(&h3->sid.identifier)) {
        qdev_prop_set_string(DEVICE(h3), "identifier",
                             "02c00081-1111-2222-3333-000044556677");
    } else if (ldl_be_p(&h3->sid.identifier.data[0]) != 0x02c00081) {
        warn_report("Security Identifier value does not include H3 prefix");
    }

    /* Setup EMAC properties */
    object_property_set_int(OBJECT(&h3->emac), 1, "phy-addr", &error_abort);

    /* DRAMC */
    object_property_set_uint(OBJECT(h3), h3->memmap[AW_H3_SDRAM],
                             "ram-addr", &error_abort);
    object_property_set_int(OBJECT(h3), machine->ram_size / MiB, "ram-size",
                            &error_abort);

    /* Mark H3 object realized */
    object_property_set_bool(OBJECT(h3), true, "realized", &error_abort);

    /* Retrieve SD bus */
    di = drive_get_next(IF_SD);
    blk = di ? blk_by_legacy_dinfo(di) : NULL;
    bus = qdev_get_child_bus(DEVICE(h3), "sd-bus");

    /* Plug in SD card */
    carddev = qdev_create(bus, TYPE_SD_CARD);
    qdev_prop_set_drive(carddev, "drive", blk, &error_fatal);
    object_property_set_bool(OBJECT(carddev), true, "realized", &error_fatal);

    /* SDRAM */
    memory_region_add_subregion(get_system_memory(), h3->memmap[AW_H3_SDRAM],
                                machine->ram);

    /* Load target kernel or start using BootROM */
    if (!machine->kernel_filename && blk && blk_is_available(blk)) {
        /* Use Boot ROM to copy data from SD card to SRAM */
        allwinner_h3_bootrom_setup(h3, blk);
    }
    orangepi_binfo.loader_start = h3->memmap[AW_H3_SDRAM];
    orangepi_binfo.ram_size = machine->ram_size;
    arm_load_kernel(ARM_CPU(first_cpu), machine, &orangepi_binfo);
}

static void orangepi_machine_init(MachineClass *mc)
{
    mc->desc = "Orange Pi PC";
    mc->init = orangepi_init;
    mc->block_default_type = IF_SD;
    mc->units_per_default_bus = 1;
    mc->min_cpus = AW_H3_NUM_CPUS;
    mc->max_cpus = AW_H3_NUM_CPUS;
    mc->default_cpus = AW_H3_NUM_CPUS;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a7");
    mc->default_ram_size = 1 * GiB;
    mc->default_ram_id = "orangepi.ram";
}

DEFINE_MACHINE("orangepi-pc", orangepi_machine_init)
