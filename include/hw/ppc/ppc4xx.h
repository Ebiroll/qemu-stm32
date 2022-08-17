/*
 * QEMU PowerPC 4xx emulation shared definitions
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

#ifndef PPC4XX_H
#define PPC4XX_H

#include "hw/ppc/ppc.h"
#include "exec/memory.h"
#include "hw/sysbus.h"

void ppc4xx_sdram_banks(MemoryRegion *ram, int nr_banks,
                        MemoryRegion ram_memories[],
                        hwaddr ram_bases[], hwaddr ram_sizes[],
                        const ram_addr_t sdram_bank_sizes[]);

void ppc4xx_sdram_init (CPUPPCState *env, qemu_irq irq, int nbanks,
                        MemoryRegion ram_memories[],
                        hwaddr *ram_bases,
                        hwaddr *ram_sizes,
                        int do_init);

void ppc4xx_mal_init(CPUPPCState *env, uint8_t txcnum, uint8_t rxcnum,
                     qemu_irq irqs[4]);

#define TYPE_PPC4xx_PCI_HOST_BRIDGE "ppc4xx-pcihost"

/*
 * Generic DCR device
 */
#define TYPE_PPC4xx_DCR_DEVICE "ppc4xx-dcr-device"
OBJECT_DECLARE_SIMPLE_TYPE(Ppc4xxDcrDeviceState, PPC4xx_DCR_DEVICE);
struct Ppc4xxDcrDeviceState {
    SysBusDevice parent_obj;

    PowerPCCPU *cpu;
};

void ppc4xx_dcr_register(Ppc4xxDcrDeviceState *dev, int dcrn, void *opaque,
                         dcr_read_cb dcr_read, dcr_write_cb dcr_write);
bool ppc4xx_dcr_realize(Ppc4xxDcrDeviceState *dev, PowerPCCPU *cpu,
                        Error **errp);

#endif /* PPC4XX_H */
