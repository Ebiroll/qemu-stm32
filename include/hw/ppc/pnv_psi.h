/*
 * QEMU PowerPC PowerNV Processor Service Interface (PSI) model
 *
 * Copyright (c) 2015-2017, IBM Corporation.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _PPC_PNV_PSI_H
#define _PPC_PNV_PSI_H

#include "hw/sysbus.h"
#include "hw/ppc/xics.h"

#define TYPE_PNV_PSI "pnv-psi"
#define PNV_PSI(obj) \
     OBJECT_CHECK(PnvPsi, (obj), TYPE_PNV_PSI)

#define PSIHB_XSCOM_MAX         0x20

typedef struct PnvPsi {
    SysBusDevice parent;

    MemoryRegion regs_mr;
    uint64_t bar;

    /* FSP region not supported */
    /* MemoryRegion fsp_mr; */
    uint64_t fsp_bar;

    /* Interrupt generation */
    qemu_irq *qirqs;

    /* Registers */
    uint64_t regs[PSIHB_XSCOM_MAX];

    MemoryRegion xscom_regs;
} PnvPsi;

#define TYPE_PNV8_PSI TYPE_PNV_PSI "-POWER8"
#define PNV8_PSI(obj) \
    OBJECT_CHECK(Pnv8Psi, (obj), TYPE_PNV8_PSI)

typedef struct Pnv8Psi {
    PnvPsi   parent;

    ICSState ics;
} Pnv8Psi;

#define PNV_PSI_CLASS(klass) \
     OBJECT_CLASS_CHECK(PnvPsiClass, (klass), TYPE_PNV_PSI)
#define PNV_PSI_GET_CLASS(obj) \
     OBJECT_GET_CLASS(PnvPsiClass, (obj), TYPE_PNV_PSI)

typedef struct PnvPsiClass {
    SysBusDeviceClass parent_class;

    int chip_type;
    uint32_t xscom_pcba;
    uint32_t xscom_size;
    uint64_t bar_mask;

    void (*irq_set)(PnvPsi *psi, int, bool state);
} PnvPsiClass;

/* The PSI and FSP interrupts are muxed on the same IRQ number */
typedef enum PnvPsiIrq {
    PSIHB_IRQ_PSI, /* internal use only */
    PSIHB_IRQ_FSP, /* internal use only */
    PSIHB_IRQ_OCC,
    PSIHB_IRQ_FSI,
    PSIHB_IRQ_LPC_I2C,
    PSIHB_IRQ_LOCAL_ERR,
    PSIHB_IRQ_EXTERNAL,
} PnvPsiIrq;

#define PSI_NUM_INTERRUPTS 6

void pnv_psi_irq_set(PnvPsi *psi, int irq, bool state);

#endif /* _PPC_PNV_PSI_H */
