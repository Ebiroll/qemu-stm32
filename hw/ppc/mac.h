/*
 * QEMU PowerMac emulation shared definitions and prototypes
 *
 * Copyright (c) 2004-2007 Fabrice Bellard
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

#ifndef PPC_MAC_H
#define PPC_MAC_H

#include "exec/memory.h"
#include "hw/sysbus.h"
#include "hw/ide/internal.h"
#include "hw/input/adb.h"
#include "hw/misc/mos6522.h"
#include "hw/pci/pci_host.h"
#include "hw/pci-host/uninorth.h"

/* SMP is not enabled, for now */
#define MAX_CPUS 1

#define BIOS_SIZE     (1024 * 1024)
#define NVRAM_SIZE        0x2000
#define PROM_FILENAME    "openbios-ppc"
#define PROM_ADDR         0xfff00000

#define KERNEL_LOAD_ADDR 0x01000000
#define KERNEL_GAP       0x00100000

#define ESCC_CLOCK 3686400

/* Old World IRQs */
#define OLDWORLD_CUDA_IRQ      0x12
#define OLDWORLD_ESCCB_IRQ     0x10
#define OLDWORLD_ESCCA_IRQ     0xf
#define OLDWORLD_IDE0_IRQ      0xd
#define OLDWORLD_IDE0_DMA_IRQ  0x2
#define OLDWORLD_IDE1_IRQ      0xe
#define OLDWORLD_IDE1_DMA_IRQ  0x3

/* New World IRQs */
#define NEWWORLD_CUDA_IRQ      0x19
#define NEWWORLD_ESCCB_IRQ     0x24
#define NEWWORLD_ESCCA_IRQ     0x25
#define NEWWORLD_IDE0_IRQ      0xd
#define NEWWORLD_IDE0_DMA_IRQ  0x2
#define NEWWORLD_IDE1_IRQ      0xe
#define NEWWORLD_IDE1_DMA_IRQ  0x3

/* MacIO */
#define TYPE_MACIO_IDE "macio-ide"
#define MACIO_IDE(obj) OBJECT_CHECK(MACIOIDEState, (obj), TYPE_MACIO_IDE)

typedef struct MACIOIDEState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/
    uint32_t channel;
    qemu_irq real_ide_irq;
    qemu_irq real_dma_irq;
    qemu_irq ide_irq;
    qemu_irq dma_irq;

    MemoryRegion mem;
    IDEBus bus;
    IDEDMA dma;
    void *dbdma;
    bool dma_active;
    uint32_t timing_reg;
    uint32_t irq_reg;
} MACIOIDEState;

void macio_ide_init_drives(MACIOIDEState *ide, DriveInfo **hd_table);
void macio_ide_register_dma(MACIOIDEState *ide);

/* Grackle PCI */
#define TYPE_GRACKLE_PCI_HOST_BRIDGE "grackle-pcihost"

/* Mac NVRAM */
#define TYPE_MACIO_NVRAM "macio-nvram"
#define MACIO_NVRAM(obj) \
    OBJECT_CHECK(MacIONVRAMState, (obj), TYPE_MACIO_NVRAM)

typedef struct MacIONVRAMState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    uint32_t size;
    uint32_t it_shift;

    MemoryRegion mem;
    uint8_t *data;
} MacIONVRAMState;

void pmac_format_nvram_partition (MacIONVRAMState *nvr, int len);
#endif /* PPC_MAC_H */
