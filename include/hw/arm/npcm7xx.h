/*
 * Nuvoton NPCM7xx SoC family.
 *
 * Copyright 2020 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 */
#ifndef NPCM7XX_H
#define NPCM7XX_H

#include "hw/boards.h"
#include "hw/cpu/a9mpcore.h"
#include "hw/misc/npcm7xx_clk.h"
#include "hw/misc/npcm7xx_gcr.h"
#include "hw/nvram/npcm7xx_otp.h"
#include "hw/timer/npcm7xx_timer.h"
#include "target/arm/cpu.h"

#define NPCM7XX_MAX_NUM_CPUS    (2)

/* The first half of the address space is reserved for DDR4 DRAM. */
#define NPCM7XX_DRAM_BA         (0x00000000)
#define NPCM7XX_DRAM_SZ         (2 * GiB)

/* Magic addresses for setting up direct kernel booting and SMP boot stubs. */
#define NPCM7XX_LOADER_START            (0x00000000)  /* Start of SDRAM */
#define NPCM7XX_SMP_LOADER_START        (0xffff0000)  /* Boot ROM */
#define NPCM7XX_SMP_BOOTREG_ADDR        (0xf080013c)  /* GCR.SCRPAD */
#define NPCM7XX_GIC_CPU_IF_ADDR         (0xf03fe100)  /* GIC within A9 */

typedef struct NPCM7xxMachine {
    MachineState        parent;
} NPCM7xxMachine;

#define TYPE_NPCM7XX_MACHINE MACHINE_TYPE_NAME("npcm7xx")
#define NPCM7XX_MACHINE(obj)                                            \
    OBJECT_CHECK(NPCM7xxMachine, (obj), TYPE_NPCM7XX_MACHINE)

typedef struct NPCM7xxMachineClass {
    MachineClass        parent;

    const char          *soc_type;
} NPCM7xxMachineClass;

#define NPCM7XX_MACHINE_CLASS(klass)                                    \
    OBJECT_CLASS_CHECK(NPCM7xxMachineClass, (klass), TYPE_NPCM7XX_MACHINE)
#define NPCM7XX_MACHINE_GET_CLASS(obj)                                  \
    OBJECT_GET_CLASS(NPCM7xxMachineClass, (obj), TYPE_NPCM7XX_MACHINE)

typedef struct NPCM7xxState {
    DeviceState         parent;

    ARMCPU              cpu[NPCM7XX_MAX_NUM_CPUS];
    A9MPPrivState       a9mpcore;

    MemoryRegion        sram;
    MemoryRegion        irom;
    MemoryRegion        ram3;
    MemoryRegion        *dram;

    NPCM7xxGCRState     gcr;
    NPCM7xxCLKState     clk;
    NPCM7xxTimerCtrlState tim[3];
    NPCM7xxOTPState     key_storage;
    NPCM7xxOTPState     fuse_array;
} NPCM7xxState;

#define TYPE_NPCM7XX    "npcm7xx"
#define NPCM7XX(obj)    OBJECT_CHECK(NPCM7xxState, (obj), TYPE_NPCM7XX)

#define TYPE_NPCM730    "npcm730"
#define TYPE_NPCM750    "npcm750"

typedef struct NPCM7xxClass {
    DeviceClass         parent;

    /* Bitmask of modules that are permanently disabled on this chip. */
    uint32_t            disabled_modules;
    /* Number of CPU cores enabled in this SoC class (may be 1 or 2). */
    uint32_t            num_cpus;
} NPCM7xxClass;

#define NPCM7XX_CLASS(klass)                                            \
    OBJECT_CLASS_CHECK(NPCM7xxClass, (klass), TYPE_NPCM7XX)
#define NPCM7XX_GET_CLASS(obj)                                          \
    OBJECT_GET_CLASS(NPCM7xxClass, (obj), TYPE_NPCM7XX)

/**
 * npcm7xx_load_kernel - Loads memory with everything needed to boot
 * @machine - The machine containing the SoC to be booted.
 * @soc - The SoC containing the CPU to be booted.
 *
 * This will set up the ARM boot info structure for the specific NPCM7xx
 * derivative and call arm_load_kernel() to set up loading of the kernel, etc.
 * into memory, if requested by the user.
 */
void npcm7xx_load_kernel(MachineState *machine, NPCM7xxState *soc);

#endif /* NPCM7XX_H */
