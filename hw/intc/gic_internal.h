/*
 * ARM GIC support - internal interfaces
 *
 * Copyright (c) 2012 Linaro Limited
 * Written by Peter Maydell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef QEMU_ARM_GIC_INTERNAL_H
#define QEMU_ARM_GIC_INTERNAL_H

#include "hw/registerfields.h"
#include "hw/intc/arm_gic.h"

#define ALL_CPU_MASK ((unsigned)(((1 << GIC_NCPU) - 1)))

#define GIC_BASE_IRQ 0

#define GIC_DIST_SET_ENABLED(irq, cm) (s->irq_state[irq].enabled |= (cm))
#define GIC_DIST_CLEAR_ENABLED(irq, cm) (s->irq_state[irq].enabled &= ~(cm))
#define GIC_DIST_TEST_ENABLED(irq, cm) ((s->irq_state[irq].enabled & (cm)) != 0)
#define GIC_DIST_SET_PENDING(irq, cm) (s->irq_state[irq].pending |= (cm))
#define GIC_DIST_CLEAR_PENDING(irq, cm) (s->irq_state[irq].pending &= ~(cm))
#define GIC_DIST_SET_ACTIVE(irq, cm) (s->irq_state[irq].active |= (cm))
#define GIC_DIST_CLEAR_ACTIVE(irq, cm) (s->irq_state[irq].active &= ~(cm))
#define GIC_DIST_TEST_ACTIVE(irq, cm) ((s->irq_state[irq].active & (cm)) != 0)
#define GIC_DIST_SET_MODEL(irq) (s->irq_state[irq].model = true)
#define GIC_DIST_CLEAR_MODEL(irq) (s->irq_state[irq].model = false)
#define GIC_DIST_TEST_MODEL(irq) (s->irq_state[irq].model)
#define GIC_DIST_SET_LEVEL(irq, cm) (s->irq_state[irq].level |= (cm))
#define GIC_DIST_CLEAR_LEVEL(irq, cm) (s->irq_state[irq].level &= ~(cm))
#define GIC_DIST_TEST_LEVEL(irq, cm) ((s->irq_state[irq].level & (cm)) != 0)
#define GIC_DIST_SET_EDGE_TRIGGER(irq) (s->irq_state[irq].edge_trigger = true)
#define GIC_DIST_CLEAR_EDGE_TRIGGER(irq) \
    (s->irq_state[irq].edge_trigger = false)
#define GIC_DIST_TEST_EDGE_TRIGGER(irq) (s->irq_state[irq].edge_trigger)
#define GIC_DIST_GET_PRIORITY(irq, cpu) (((irq) < GIC_INTERNAL) ?            \
                                    s->priority1[irq][cpu] :            \
                                    s->priority2[(irq) - GIC_INTERNAL])
#define GIC_DIST_TARGET(irq) (s->irq_target[irq])
#define GIC_DIST_CLEAR_GROUP(irq, cm) (s->irq_state[irq].group &= ~(cm))
#define GIC_DIST_SET_GROUP(irq, cm) (s->irq_state[irq].group |= (cm))
#define GIC_DIST_TEST_GROUP(irq, cm) ((s->irq_state[irq].group & (cm)) != 0)

#define GICD_CTLR_EN_GRP0 (1U << 0)
#define GICD_CTLR_EN_GRP1 (1U << 1)

#define GICC_CTLR_EN_GRP0    (1U << 0)
#define GICC_CTLR_EN_GRP1    (1U << 1)
#define GICC_CTLR_ACK_CTL    (1U << 2)
#define GICC_CTLR_FIQ_EN     (1U << 3)
#define GICC_CTLR_CBPR       (1U << 4) /* GICv1: SBPR */
#define GICC_CTLR_EOIMODE    (1U << 9)
#define GICC_CTLR_EOIMODE_NS (1U << 10)

REG32(GICH_HCR, 0x0)
    FIELD(GICH_HCR, EN, 0, 1)
    FIELD(GICH_HCR, UIE, 1, 1)
    FIELD(GICH_HCR, LRENPIE, 2, 1)
    FIELD(GICH_HCR, NPIE, 3, 1)
    FIELD(GICH_HCR, VGRP0EIE, 4, 1)
    FIELD(GICH_HCR, VGRP0DIE, 5, 1)
    FIELD(GICH_HCR, VGRP1EIE, 6, 1)
    FIELD(GICH_HCR, VGRP1DIE, 7, 1)
    FIELD(GICH_HCR, EOICount, 27, 5)

#define GICH_HCR_MASK \
    (R_GICH_HCR_EN_MASK | R_GICH_HCR_UIE_MASK | \
     R_GICH_HCR_LRENPIE_MASK | R_GICH_HCR_NPIE_MASK | \
     R_GICH_HCR_VGRP0EIE_MASK | R_GICH_HCR_VGRP0DIE_MASK | \
     R_GICH_HCR_VGRP1EIE_MASK | R_GICH_HCR_VGRP1DIE_MASK | \
     R_GICH_HCR_EOICount_MASK)

REG32(GICH_VTR, 0x4)
    FIELD(GICH_VTR, ListRegs, 0, 6)
    FIELD(GICH_VTR, PREbits, 26, 3)
    FIELD(GICH_VTR, PRIbits, 29, 3)

REG32(GICH_VMCR, 0x8)
    FIELD(GICH_VMCR, VMCCtlr, 0, 10)
    FIELD(GICH_VMCR, VMABP, 18, 3)
    FIELD(GICH_VMCR, VMBP, 21, 3)
    FIELD(GICH_VMCR, VMPriMask, 27, 5)

REG32(GICH_MISR, 0x10)
    FIELD(GICH_MISR, EOI, 0, 1)
    FIELD(GICH_MISR, U, 1, 1)
    FIELD(GICH_MISR, LRENP, 2, 1)
    FIELD(GICH_MISR, NP, 3, 1)
    FIELD(GICH_MISR, VGrp0E, 4, 1)
    FIELD(GICH_MISR, VGrp0D, 5, 1)
    FIELD(GICH_MISR, VGrp1E, 6, 1)
    FIELD(GICH_MISR, VGrp1D, 7, 1)

REG32(GICH_EISR0, 0x20)
REG32(GICH_EISR1, 0x24)
REG32(GICH_ELRSR0, 0x30)
REG32(GICH_ELRSR1, 0x34)
REG32(GICH_APR, 0xf0)

REG32(GICH_LR0, 0x100)
    FIELD(GICH_LR0, VirtualID, 0, 10)
    FIELD(GICH_LR0, PhysicalID, 10, 10)
    FIELD(GICH_LR0, CPUID, 10, 3)
    FIELD(GICH_LR0, EOI, 19, 1)
    FIELD(GICH_LR0, Priority, 23, 5)
    FIELD(GICH_LR0, State, 28, 2)
    FIELD(GICH_LR0, Grp1, 30, 1)
    FIELD(GICH_LR0, HW, 31, 1)

/* Last LR register */
REG32(GICH_LR63, 0x1fc)

#define GICH_LR_MASK \
    (R_GICH_LR0_VirtualID_MASK | R_GICH_LR0_PhysicalID_MASK | \
     R_GICH_LR0_CPUID_MASK | R_GICH_LR0_EOI_MASK | \
     R_GICH_LR0_Priority_MASK | R_GICH_LR0_State_MASK | \
     R_GICH_LR0_Grp1_MASK | R_GICH_LR0_HW_MASK)

/* Valid bits for GICC_CTLR for GICv1, v1 with security extensions,
 * GICv2 and GICv2 with security extensions:
 */
#define GICC_CTLR_V1_MASK    0x1
#define GICC_CTLR_V1_S_MASK  0x1f
#define GICC_CTLR_V2_MASK    0x21f
#define GICC_CTLR_V2_S_MASK  0x61f

/* The special cases for the revision property: */
#define REV_11MPCORE 0

uint32_t gic_acknowledge_irq(GICState *s, int cpu, MemTxAttrs attrs);
void gic_dist_set_priority(GICState *s, int cpu, int irq, uint8_t val,
                           MemTxAttrs attrs);

static inline bool gic_test_pending(GICState *s, int irq, int cm)
{
    if (s->revision == REV_11MPCORE) {
        return s->irq_state[irq].pending & cm;
    } else {
        /* Edge-triggered interrupts are marked pending on a rising edge, but
         * level-triggered interrupts are either considered pending when the
         * level is active or if software has explicitly written to
         * GICD_ISPENDR to set the state pending.
         */
        return (s->irq_state[irq].pending & cm) ||
            (!GIC_DIST_TEST_EDGE_TRIGGER(irq) && GIC_DIST_TEST_LEVEL(irq, cm));
    }
}

static inline bool gic_is_vcpu(int cpu)
{
    return cpu >= GIC_NCPU;
}

#endif /* QEMU_ARM_GIC_INTERNAL_H */
