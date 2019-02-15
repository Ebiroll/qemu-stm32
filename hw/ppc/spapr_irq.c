/*
 * QEMU PowerPC sPAPR IRQ interface
 *
 * Copyright (c) 2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/ppc/spapr.h"
#include "hw/ppc/spapr_cpu_core.h"
#include "hw/ppc/spapr_xive.h"
#include "hw/ppc/xics.h"
#include "hw/ppc/xics_spapr.h"
#include "sysemu/kvm.h"

#include "trace.h"

void spapr_irq_msi_init(sPAPRMachineState *spapr, uint32_t nr_msis)
{
    spapr->irq_map_nr = nr_msis;
    spapr->irq_map = bitmap_new(spapr->irq_map_nr);
}

int spapr_irq_msi_alloc(sPAPRMachineState *spapr, uint32_t num, bool align,
                        Error **errp)
{
    int irq;

    /*
     * The 'align_mask' parameter of bitmap_find_next_zero_area()
     * should be one less than a power of 2; 0 means no
     * alignment. Adapt the 'align' value of the former allocator
     * to fit the requirements of bitmap_find_next_zero_area()
     */
    align -= 1;

    irq = bitmap_find_next_zero_area(spapr->irq_map, spapr->irq_map_nr, 0, num,
                                     align);
    if (irq == spapr->irq_map_nr) {
        error_setg(errp, "can't find a free %d-IRQ block", num);
        return -1;
    }

    bitmap_set(spapr->irq_map, irq, num);

    return irq + SPAPR_IRQ_MSI;
}

void spapr_irq_msi_free(sPAPRMachineState *spapr, int irq, uint32_t num)
{
    bitmap_clear(spapr->irq_map, irq - SPAPR_IRQ_MSI, num);
}

void spapr_irq_msi_reset(sPAPRMachineState *spapr)
{
    bitmap_clear(spapr->irq_map, 0, spapr->irq_map_nr);
}


/*
 * XICS IRQ backend.
 */

static ICSState *spapr_ics_create(sPAPRMachineState *spapr,
                                  const char *type_ics,
                                  int nr_irqs, Error **errp)
{
    Error *local_err = NULL;
    Object *obj;

    obj = object_new(type_ics);
    object_property_add_child(OBJECT(spapr), "ics", obj, &error_abort);
    object_property_add_const_link(obj, ICS_PROP_XICS, OBJECT(spapr),
                                   &error_abort);
    object_property_set_int(obj, nr_irqs, "nr-irqs", &local_err);
    if (local_err) {
        goto error;
    }
    object_property_set_bool(obj, true, "realized", &local_err);
    if (local_err) {
        goto error;
    }

    return ICS_BASE(obj);

error:
    error_propagate(errp, local_err);
    return NULL;
}

static void spapr_irq_init_xics(sPAPRMachineState *spapr, int nr_irqs,
                                Error **errp)
{
    MachineState *machine = MACHINE(spapr);
    Error *local_err = NULL;

    if (kvm_enabled()) {
        if (machine_kernel_irqchip_allowed(machine) &&
            !xics_kvm_init(spapr, &local_err)) {
            spapr->ics = spapr_ics_create(spapr, TYPE_ICS_KVM, nr_irqs,
                                          &local_err);
        }
        if (machine_kernel_irqchip_required(machine) && !spapr->ics) {
            error_prepend(&local_err,
                          "kernel_irqchip requested but unavailable: ");
            goto error;
        }
        error_free(local_err);
        local_err = NULL;
    }

    if (!spapr->ics) {
        xics_spapr_init(spapr);
        spapr->ics = spapr_ics_create(spapr, TYPE_ICS_SIMPLE, nr_irqs,
                                      &local_err);
    }

error:
    error_propagate(errp, local_err);
}

#define ICS_IRQ_FREE(ics, srcno)   \
    (!((ics)->irqs[(srcno)].flags & (XICS_FLAGS_IRQ_MASK)))

static int spapr_irq_claim_xics(sPAPRMachineState *spapr, int irq, bool lsi,
                                Error **errp)
{
    ICSState *ics = spapr->ics;

    assert(ics);

    if (!ics_valid_irq(ics, irq)) {
        error_setg(errp, "IRQ %d is invalid", irq);
        return -1;
    }

    if (!ICS_IRQ_FREE(ics, irq - ics->offset)) {
        error_setg(errp, "IRQ %d is not free", irq);
        return -1;
    }

    ics_set_irq_type(ics, irq - ics->offset, lsi);
    return 0;
}

static void spapr_irq_free_xics(sPAPRMachineState *spapr, int irq, int num)
{
    ICSState *ics = spapr->ics;
    uint32_t srcno = irq - ics->offset;
    int i;

    if (ics_valid_irq(ics, irq)) {
        trace_spapr_irq_free(0, irq, num);
        for (i = srcno; i < srcno + num; ++i) {
            if (ICS_IRQ_FREE(ics, i)) {
                trace_spapr_irq_free_warn(0, i);
            }
            memset(&ics->irqs[i], 0, sizeof(ICSIRQState));
        }
    }
}

static qemu_irq spapr_qirq_xics(sPAPRMachineState *spapr, int irq)
{
    ICSState *ics = spapr->ics;
    uint32_t srcno = irq - ics->offset;

    if (ics_valid_irq(ics, irq)) {
        return spapr->qirqs[srcno];
    }

    return NULL;
}

static void spapr_irq_print_info_xics(sPAPRMachineState *spapr, Monitor *mon)
{
    CPUState *cs;

    CPU_FOREACH(cs) {
        PowerPCCPU *cpu = POWERPC_CPU(cs);

        icp_pic_print_info(spapr_cpu_state(cpu)->icp, mon);
    }

    ics_pic_print_info(spapr->ics, mon);
}

static void spapr_irq_cpu_intc_create_xics(sPAPRMachineState *spapr,
                                           PowerPCCPU *cpu, Error **errp)
{
    Error *local_err = NULL;
    Object *obj;
    sPAPRCPUState *spapr_cpu = spapr_cpu_state(cpu);

    obj = icp_create(OBJECT(cpu), TYPE_ICP, XICS_FABRIC(spapr),
                     &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    spapr_cpu->icp = ICP(obj);
}

static int spapr_irq_post_load_xics(sPAPRMachineState *spapr, int version_id)
{
    if (!object_dynamic_cast(OBJECT(spapr->ics), TYPE_ICS_KVM)) {
        CPUState *cs;
        CPU_FOREACH(cs) {
            PowerPCCPU *cpu = POWERPC_CPU(cs);
            icp_resend(spapr_cpu_state(cpu)->icp);
        }
    }
    return 0;
}

static void spapr_irq_set_irq_xics(void *opaque, int srcno, int val)
{
    sPAPRMachineState *spapr = opaque;
    MachineState *machine = MACHINE(opaque);

    if (kvm_enabled() && machine_kernel_irqchip_allowed(machine)) {
        ics_kvm_set_irq(spapr->ics, srcno, val);
    } else {
        ics_simple_set_irq(spapr->ics, srcno, val);
    }
}

static void spapr_irq_reset_xics(sPAPRMachineState *spapr, Error **errp)
{
    /* TODO: create the KVM XICS device */
}

#define SPAPR_IRQ_XICS_NR_IRQS     0x1000
#define SPAPR_IRQ_XICS_NR_MSIS     \
    (XICS_IRQ_BASE + SPAPR_IRQ_XICS_NR_IRQS - SPAPR_IRQ_MSI)

sPAPRIrq spapr_irq_xics = {
    .nr_irqs     = SPAPR_IRQ_XICS_NR_IRQS,
    .nr_msis     = SPAPR_IRQ_XICS_NR_MSIS,
    .ov5         = SPAPR_OV5_XIVE_LEGACY,

    .init        = spapr_irq_init_xics,
    .claim       = spapr_irq_claim_xics,
    .free        = spapr_irq_free_xics,
    .qirq        = spapr_qirq_xics,
    .print_info  = spapr_irq_print_info_xics,
    .dt_populate = spapr_dt_xics,
    .cpu_intc_create = spapr_irq_cpu_intc_create_xics,
    .post_load   = spapr_irq_post_load_xics,
    .reset       = spapr_irq_reset_xics,
    .set_irq     = spapr_irq_set_irq_xics,
};

/*
 * XIVE IRQ backend.
 */
static void spapr_irq_init_xive(sPAPRMachineState *spapr, int nr_irqs,
                                Error **errp)
{
    MachineState *machine = MACHINE(spapr);
    uint32_t nr_servers = spapr_max_server_number(spapr);
    DeviceState *dev;
    int i;

    /* KVM XIVE device not yet available */
    if (kvm_enabled()) {
        if (machine_kernel_irqchip_required(machine)) {
            error_setg(errp, "kernel_irqchip requested. no KVM XIVE support");
            return;
        }
    }

    dev = qdev_create(NULL, TYPE_SPAPR_XIVE);
    qdev_prop_set_uint32(dev, "nr-irqs", nr_irqs);
    /*
     * 8 XIVE END structures per CPU. One for each available priority
     */
    qdev_prop_set_uint32(dev, "nr-ends", nr_servers << 3);
    qdev_init_nofail(dev);

    spapr->xive = SPAPR_XIVE(dev);

    /* Enable the CPU IPIs */
    for (i = 0; i < nr_servers; ++i) {
        spapr_xive_irq_claim(spapr->xive, SPAPR_IRQ_IPI + i, false);
    }

    spapr_xive_hcall_init(spapr);
}

static int spapr_irq_claim_xive(sPAPRMachineState *spapr, int irq, bool lsi,
                                Error **errp)
{
    if (!spapr_xive_irq_claim(spapr->xive, irq, lsi)) {
        error_setg(errp, "IRQ %d is invalid", irq);
        return -1;
    }
    return 0;
}

static void spapr_irq_free_xive(sPAPRMachineState *spapr, int irq, int num)
{
    int i;

    for (i = irq; i < irq + num; ++i) {
        spapr_xive_irq_free(spapr->xive, i);
    }
}

static qemu_irq spapr_qirq_xive(sPAPRMachineState *spapr, int irq)
{
    sPAPRXive *xive = spapr->xive;

    if (irq >= xive->nr_irqs) {
        return NULL;
    }

    /* The sPAPR machine/device should have claimed the IRQ before */
    assert(xive_eas_is_valid(&xive->eat[irq]));

    return spapr->qirqs[irq];
}

static void spapr_irq_print_info_xive(sPAPRMachineState *spapr,
                                      Monitor *mon)
{
    CPUState *cs;

    CPU_FOREACH(cs) {
        PowerPCCPU *cpu = POWERPC_CPU(cs);

        xive_tctx_pic_print_info(spapr_cpu_state(cpu)->tctx, mon);
    }

    spapr_xive_pic_print_info(spapr->xive, mon);
}

static void spapr_irq_cpu_intc_create_xive(sPAPRMachineState *spapr,
                                           PowerPCCPU *cpu, Error **errp)
{
    Error *local_err = NULL;
    Object *obj;
    sPAPRCPUState *spapr_cpu = spapr_cpu_state(cpu);

    obj = xive_tctx_create(OBJECT(cpu), XIVE_ROUTER(spapr->xive), &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    spapr_cpu->tctx = XIVE_TCTX(obj);

    /*
     * (TCG) Early setting the OS CAM line for hotplugged CPUs as they
     * don't beneficiate from the reset of the XIVE IRQ backend
     */
    spapr_xive_set_tctx_os_cam(spapr_cpu->tctx);
}

static int spapr_irq_post_load_xive(sPAPRMachineState *spapr, int version_id)
{
    return 0;
}

static void spapr_irq_reset_xive(sPAPRMachineState *spapr, Error **errp)
{
    CPUState *cs;

    CPU_FOREACH(cs) {
        PowerPCCPU *cpu = POWERPC_CPU(cs);

        /* (TCG) Set the OS CAM line of the thread interrupt context. */
        spapr_xive_set_tctx_os_cam(spapr_cpu_state(cpu)->tctx);
    }

    /* Activate the XIVE MMIOs */
    spapr_xive_mmio_set_enabled(spapr->xive, true);
}

static void spapr_irq_set_irq_xive(void *opaque, int srcno, int val)
{
    sPAPRMachineState *spapr = opaque;

    xive_source_set_irq(&spapr->xive->source, srcno, val);
}

/*
 * XIVE uses the full IRQ number space. Set it to 8K to be compatible
 * with XICS.
 */

#define SPAPR_IRQ_XIVE_NR_IRQS     0x2000
#define SPAPR_IRQ_XIVE_NR_MSIS     (SPAPR_IRQ_XIVE_NR_IRQS - SPAPR_IRQ_MSI)

sPAPRIrq spapr_irq_xive = {
    .nr_irqs     = SPAPR_IRQ_XIVE_NR_IRQS,
    .nr_msis     = SPAPR_IRQ_XIVE_NR_MSIS,
    .ov5         = SPAPR_OV5_XIVE_EXPLOIT,

    .init        = spapr_irq_init_xive,
    .claim       = spapr_irq_claim_xive,
    .free        = spapr_irq_free_xive,
    .qirq        = spapr_qirq_xive,
    .print_info  = spapr_irq_print_info_xive,
    .dt_populate = spapr_dt_xive,
    .cpu_intc_create = spapr_irq_cpu_intc_create_xive,
    .post_load   = spapr_irq_post_load_xive,
    .reset       = spapr_irq_reset_xive,
    .set_irq     = spapr_irq_set_irq_xive,
};

/*
 * Dual XIVE and XICS IRQ backend.
 *
 * Both interrupt mode, XIVE and XICS, objects are created but the
 * machine starts in legacy interrupt mode (XICS). It can be changed
 * by the CAS negotiation process and, in that case, the new mode is
 * activated after an extra machine reset.
 */

/*
 * Returns the sPAPR IRQ backend negotiated by CAS. XICS is the
 * default.
 */
static sPAPRIrq *spapr_irq_current(sPAPRMachineState *spapr)
{
    return spapr_ovec_test(spapr->ov5_cas, OV5_XIVE_EXPLOIT) ?
        &spapr_irq_xive : &spapr_irq_xics;
}

static void spapr_irq_init_dual(sPAPRMachineState *spapr, int nr_irqs,
                                Error **errp)
{
    MachineState *machine = MACHINE(spapr);
    Error *local_err = NULL;

    if (kvm_enabled() && machine_kernel_irqchip_allowed(machine)) {
        error_setg(errp, "No KVM support for the 'dual' machine");
        return;
    }

    spapr_irq_xics.init(spapr, spapr_irq_xics.nr_irqs, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    spapr_irq_xive.init(spapr, spapr_irq_xive.nr_irqs, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }
}

static int spapr_irq_claim_dual(sPAPRMachineState *spapr, int irq, bool lsi,
                                Error **errp)
{
    Error *local_err = NULL;
    int ret;

    ret = spapr_irq_xics.claim(spapr, irq, lsi, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return ret;
    }

    ret = spapr_irq_xive.claim(spapr, irq, lsi, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return ret;
    }

    return ret;
}

static void spapr_irq_free_dual(sPAPRMachineState *spapr, int irq, int num)
{
    spapr_irq_xics.free(spapr, irq, num);
    spapr_irq_xive.free(spapr, irq, num);
}

static qemu_irq spapr_qirq_dual(sPAPRMachineState *spapr, int irq)
{
    return spapr_irq_current(spapr)->qirq(spapr, irq);
}

static void spapr_irq_print_info_dual(sPAPRMachineState *spapr, Monitor *mon)
{
    spapr_irq_current(spapr)->print_info(spapr, mon);
}

static void spapr_irq_dt_populate_dual(sPAPRMachineState *spapr,
                                       uint32_t nr_servers, void *fdt,
                                       uint32_t phandle)
{
    spapr_irq_current(spapr)->dt_populate(spapr, nr_servers, fdt, phandle);
}

static void spapr_irq_cpu_intc_create_dual(sPAPRMachineState *spapr,
                                           PowerPCCPU *cpu, Error **errp)
{
    Error *local_err = NULL;

    spapr_irq_xive.cpu_intc_create(spapr, cpu, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    spapr_irq_xics.cpu_intc_create(spapr, cpu, errp);
}

static int spapr_irq_post_load_dual(sPAPRMachineState *spapr, int version_id)
{
    /*
     * Force a reset of the XIVE backend after migration. The machine
     * defaults to XICS at startup.
     */
    if (spapr_ovec_test(spapr->ov5_cas, OV5_XIVE_EXPLOIT)) {
        spapr_irq_xive.reset(spapr, &error_fatal);
    }

    return spapr_irq_current(spapr)->post_load(spapr, version_id);
}

static void spapr_irq_reset_dual(sPAPRMachineState *spapr, Error **errp)
{
    /*
     * Deactivate the XIVE MMIOs. The XIVE backend will reenable them
     * if selected.
     */
    spapr_xive_mmio_set_enabled(spapr->xive, false);

    spapr_irq_current(spapr)->reset(spapr, errp);
}

static void spapr_irq_set_irq_dual(void *opaque, int srcno, int val)
{
    sPAPRMachineState *spapr = opaque;

    spapr_irq_current(spapr)->set_irq(spapr, srcno, val);
}

/*
 * Define values in sync with the XIVE and XICS backend
 */
#define SPAPR_IRQ_DUAL_NR_IRQS     0x2000
#define SPAPR_IRQ_DUAL_NR_MSIS     (SPAPR_IRQ_DUAL_NR_IRQS - SPAPR_IRQ_MSI)

sPAPRIrq spapr_irq_dual = {
    .nr_irqs     = SPAPR_IRQ_DUAL_NR_IRQS,
    .nr_msis     = SPAPR_IRQ_DUAL_NR_MSIS,
    .ov5         = SPAPR_OV5_XIVE_BOTH,

    .init        = spapr_irq_init_dual,
    .claim       = spapr_irq_claim_dual,
    .free        = spapr_irq_free_dual,
    .qirq        = spapr_qirq_dual,
    .print_info  = spapr_irq_print_info_dual,
    .dt_populate = spapr_irq_dt_populate_dual,
    .cpu_intc_create = spapr_irq_cpu_intc_create_dual,
    .post_load   = spapr_irq_post_load_dual,
    .reset       = spapr_irq_reset_dual,
    .set_irq     = spapr_irq_set_irq_dual
};

/*
 * sPAPR IRQ frontend routines for devices
 */
void spapr_irq_init(sPAPRMachineState *spapr, Error **errp)
{
    MachineState *machine = MACHINE(spapr);

    if (machine_kernel_irqchip_split(machine)) {
        error_setg(errp, "kernel_irqchip split mode not supported on pseries");
        return;
    }

    if (!kvm_enabled() && machine_kernel_irqchip_required(machine)) {
        error_setg(errp,
                   "kernel_irqchip requested but only available with KVM");
        return;
    }

    /* Initialize the MSI IRQ allocator. */
    if (!SPAPR_MACHINE_GET_CLASS(spapr)->legacy_irq_allocation) {
        spapr_irq_msi_init(spapr, spapr->irq->nr_msis);
    }

    spapr->irq->init(spapr, spapr->irq->nr_irqs, errp);

    spapr->qirqs = qemu_allocate_irqs(spapr->irq->set_irq, spapr,
                                      spapr->irq->nr_irqs);
}

int spapr_irq_claim(sPAPRMachineState *spapr, int irq, bool lsi, Error **errp)
{
    return spapr->irq->claim(spapr, irq, lsi, errp);
}

void spapr_irq_free(sPAPRMachineState *spapr, int irq, int num)
{
    spapr->irq->free(spapr, irq, num);
}

qemu_irq spapr_qirq(sPAPRMachineState *spapr, int irq)
{
    return spapr->irq->qirq(spapr, irq);
}

int spapr_irq_post_load(sPAPRMachineState *spapr, int version_id)
{
    return spapr->irq->post_load(spapr, version_id);
}

void spapr_irq_reset(sPAPRMachineState *spapr, Error **errp)
{
    if (spapr->irq->reset) {
        spapr->irq->reset(spapr, errp);
    }
}

/*
 * XICS legacy routines - to deprecate one day
 */

static int ics_find_free_block(ICSState *ics, int num, int alignnum)
{
    int first, i;

    for (first = 0; first < ics->nr_irqs; first += alignnum) {
        if (num > (ics->nr_irqs - first)) {
            return -1;
        }
        for (i = first; i < first + num; ++i) {
            if (!ICS_IRQ_FREE(ics, i)) {
                break;
            }
        }
        if (i == (first + num)) {
            return first;
        }
    }

    return -1;
}

int spapr_irq_find(sPAPRMachineState *spapr, int num, bool align, Error **errp)
{
    ICSState *ics = spapr->ics;
    int first = -1;

    assert(ics);

    /*
     * MSIMesage::data is used for storing VIRQ so
     * it has to be aligned to num to support multiple
     * MSI vectors. MSI-X is not affected by this.
     * The hint is used for the first IRQ, the rest should
     * be allocated continuously.
     */
    if (align) {
        assert((num == 1) || (num == 2) || (num == 4) ||
               (num == 8) || (num == 16) || (num == 32));
        first = ics_find_free_block(ics, num, num);
    } else {
        first = ics_find_free_block(ics, num, 1);
    }

    if (first < 0) {
        error_setg(errp, "can't find a free %d-IRQ block", num);
        return -1;
    }

    return first + ics->offset;
}

#define SPAPR_IRQ_XICS_LEGACY_NR_IRQS     0x400

sPAPRIrq spapr_irq_xics_legacy = {
    .nr_irqs     = SPAPR_IRQ_XICS_LEGACY_NR_IRQS,
    .nr_msis     = SPAPR_IRQ_XICS_LEGACY_NR_IRQS,
    .ov5         = SPAPR_OV5_XIVE_LEGACY,

    .init        = spapr_irq_init_xics,
    .claim       = spapr_irq_claim_xics,
    .free        = spapr_irq_free_xics,
    .qirq        = spapr_qirq_xics,
    .print_info  = spapr_irq_print_info_xics,
    .dt_populate = spapr_dt_xics,
    .cpu_intc_create = spapr_irq_cpu_intc_create_xics,
    .post_load   = spapr_irq_post_load_xics,
    .set_irq     = spapr_irq_set_irq_xics,
};
