/*
 * QEMU S/390 CPU
 *
 * Copyright (c) 2009 Ulrich Hecht
 * Copyright (c) 2011 Alexander Graf
 * Copyright (c) 2012 SUSE LINUX Products GmbH
 * Copyright (c) 2012 IBM Corp.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 * Contributions after 2012-12-11 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "cpu.h"
#include "internal.h"
#include "kvm_s390x.h"
#include "sysemu/kvm.h"
#include "qemu-common.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "trace.h"
#include "qapi/visitor.h"
#include "exec/exec-all.h"
#include "hw/qdev-properties.h"
#ifndef CONFIG_USER_ONLY
#include "hw/hw.h"
#include "sysemu/arch_init.h"
#include "sysemu/sysemu.h"
#include "hw/s390x/sclp.h"
#endif

#define CR0_RESET       0xE0UL
#define CR14_RESET      0xC2000000UL;

static void s390_cpu_set_pc(CPUState *cs, vaddr value)
{
    S390CPU *cpu = S390_CPU(cs);

    cpu->env.psw.addr = value;
}

static bool s390_cpu_has_work(CPUState *cs)
{
    S390CPU *cpu = S390_CPU(cs);
    CPUS390XState *env = &cpu->env;

    return (cs->interrupt_request & CPU_INTERRUPT_HARD) &&
           (env->psw.mask & PSW_MASK_EXT);
}

#if !defined(CONFIG_USER_ONLY)
/* S390CPUClass::load_normal() */
static void s390_cpu_load_normal(CPUState *s)
{
    S390CPU *cpu = S390_CPU(s);
    cpu->env.psw.addr = ldl_phys(s->as, 4) & PSW_MASK_ESA_ADDR;
    cpu->env.psw.mask = PSW_MASK_32 | PSW_MASK_64;
    s390_cpu_set_state(CPU_STATE_OPERATING, cpu);
}
#endif

/* S390CPUClass::cpu_reset() */
static void s390_cpu_reset(CPUState *s)
{
    S390CPU *cpu = S390_CPU(s);
    S390CPUClass *scc = S390_CPU_GET_CLASS(cpu);
    CPUS390XState *env = &cpu->env;

    env->pfault_token = -1UL;
    scc->parent_reset(s);
    cpu->env.sigp_order = 0;
    s390_cpu_set_state(CPU_STATE_STOPPED, cpu);
}

/* S390CPUClass::initial_reset() */
static void s390_cpu_initial_reset(CPUState *s)
{
    S390CPU *cpu = S390_CPU(s);
    CPUS390XState *env = &cpu->env;
    int i;

    s390_cpu_reset(s);
    /* initial reset does not clear everything! */
    memset(&env->start_initial_reset_fields, 0,
        offsetof(CPUS390XState, end_reset_fields) -
        offsetof(CPUS390XState, start_initial_reset_fields));

    /* architectured initial values for CR 0 and 14 */
    env->cregs[0] = CR0_RESET;
    env->cregs[14] = CR14_RESET;

    /* architectured initial value for Breaking-Event-Address register */
    env->gbea = 1;

    env->pfault_token = -1UL;
    env->ext_index = -1;
    for (i = 0; i < ARRAY_SIZE(env->io_index); i++) {
        env->io_index[i] = -1;
    }

    /* tininess for underflow is detected before rounding */
    set_float_detect_tininess(float_tininess_before_rounding,
                              &env->fpu_status);

    /* Reset state inside the kernel that we cannot access yet from QEMU. */
    if (kvm_enabled()) {
        kvm_s390_reset_vcpu(cpu);
    }
}

/* CPUClass:reset() */
static void s390_cpu_full_reset(CPUState *s)
{
    S390CPU *cpu = S390_CPU(s);
    S390CPUClass *scc = S390_CPU_GET_CLASS(cpu);
    CPUS390XState *env = &cpu->env;
    int i;

    scc->parent_reset(s);
    cpu->env.sigp_order = 0;
    s390_cpu_set_state(CPU_STATE_STOPPED, cpu);

    memset(env, 0, offsetof(CPUS390XState, end_reset_fields));

    /* architectured initial values for CR 0 and 14 */
    env->cregs[0] = CR0_RESET;
    env->cregs[14] = CR14_RESET;

    /* architectured initial value for Breaking-Event-Address register */
    env->gbea = 1;

    env->pfault_token = -1UL;
    env->ext_index = -1;
    for (i = 0; i < ARRAY_SIZE(env->io_index); i++) {
        env->io_index[i] = -1;
    }

    /* tininess for underflow is detected before rounding */
    set_float_detect_tininess(float_tininess_before_rounding,
                              &env->fpu_status);

    /* Reset state inside the kernel that we cannot access yet from QEMU. */
    if (kvm_enabled()) {
        kvm_s390_reset_vcpu(cpu);
    }
}

#if !defined(CONFIG_USER_ONLY)
static void s390_cpu_machine_reset_cb(void *opaque)
{
    S390CPU *cpu = opaque;

    run_on_cpu(CPU(cpu), s390_do_cpu_full_reset, RUN_ON_CPU_NULL);
}
#endif

static void s390_cpu_disas_set_info(CPUState *cpu, disassemble_info *info)
{
    info->mach = bfd_mach_s390_64;
    info->print_insn = print_insn_s390;
}

static void s390_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    S390CPUClass *scc = S390_CPU_GET_CLASS(dev);
    S390CPU *cpu = S390_CPU(dev);
    CPUS390XState *env = &cpu->env;
    Error *err = NULL;

    /* the model has to be realized before qemu_init_vcpu() due to kvm */
    s390_realize_cpu_model(cs, &err);
    if (err) {
        goto out;
    }

#if !defined(CONFIG_USER_ONLY)
    if (cpu->env.core_id >= max_cpus) {
        error_setg(&err, "Unable to add CPU with core-id: %" PRIu32
                   ", maximum core-id: %d", cpu->env.core_id,
                   max_cpus - 1);
        goto out;
    }
#else
    /* implicitly set for linux-user only */
    cpu->env.core_id = scc->next_core_id;
#endif

    if (cpu_exists(cpu->env.core_id)) {
        error_setg(&err, "Unable to add CPU with core-id: %" PRIu32
                   ", it already exists", cpu->env.core_id);
        goto out;
    }
    if (cpu->env.core_id != scc->next_core_id) {
        error_setg(&err, "Unable to add CPU with core-id: %" PRIu32
                   ", the next available core-id is %" PRIu32, cpu->env.core_id,
                   scc->next_core_id);
        goto out;
    }

    /* sync cs->cpu_index and env->core_id. The latter is needed for TCG. */
    cs->cpu_index = env->core_id;
    cpu_exec_realizefn(cs, &err);
    if (err != NULL) {
        goto out;
    }
    scc->next_core_id++;

#if !defined(CONFIG_USER_ONLY)
    qemu_register_reset(s390_cpu_machine_reset_cb, cpu);
#endif
    s390_cpu_gdb_init(cs);
    qemu_init_vcpu(cs);
#if !defined(CONFIG_USER_ONLY)
    run_on_cpu(cs, s390_do_cpu_full_reset, RUN_ON_CPU_NULL);
#else
    cpu_reset(cs);
#endif

    scc->parent_realize(dev, &err);

#if !defined(CONFIG_USER_ONLY)
    if (dev->hotplugged) {
        raise_irq_cpu_hotplug();
    }
#endif

out:
    error_propagate(errp, err);
}

static void s390_cpu_initfn(Object *obj)
{
    CPUState *cs = CPU(obj);
    S390CPU *cpu = S390_CPU(obj);
    CPUS390XState *env = &cpu->env;
    static bool inited;
#if !defined(CONFIG_USER_ONLY)
    struct tm tm;
#endif

    cs->env_ptr = env;
    cs->halted = 1;
    cs->exception_index = EXCP_HLT;
    s390_cpu_model_register_props(obj);
#if !defined(CONFIG_USER_ONLY)
    qemu_get_timedate(&tm, 0);
    env->tod_offset = TOD_UNIX_EPOCH +
                      (time2tod(mktimegm(&tm)) * 1000000000ULL);
    env->tod_basetime = 0;
    env->tod_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, s390x_tod_timer, cpu);
    env->cpu_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, s390x_cpu_timer, cpu);
    s390_cpu_set_state(CPU_STATE_STOPPED, cpu);
#endif

    if (tcg_enabled() && !inited) {
        inited = true;
        s390x_translate_init();
    }
}

static void s390_cpu_finalize(Object *obj)
{
#if !defined(CONFIG_USER_ONLY)
    S390CPU *cpu = S390_CPU(obj);

    qemu_unregister_reset(s390_cpu_machine_reset_cb, cpu);
    g_free(cpu->irqstate);
#endif
}

#if !defined(CONFIG_USER_ONLY)
static bool disabled_wait(CPUState *cpu)
{
    return cpu->halted && !(S390_CPU(cpu)->env.psw.mask &
                            (PSW_MASK_IO | PSW_MASK_EXT | PSW_MASK_MCHECK));
}

static unsigned s390_count_running_cpus(void)
{
    CPUState *cpu;
    int nr_running = 0;

    CPU_FOREACH(cpu) {
        uint8_t state = S390_CPU(cpu)->env.cpu_state;
        if (state == CPU_STATE_OPERATING ||
            state == CPU_STATE_LOAD) {
            if (!disabled_wait(cpu)) {
                nr_running++;
            }
        }
    }

    return nr_running;
}

unsigned int s390_cpu_halt(S390CPU *cpu)
{
    CPUState *cs = CPU(cpu);
    trace_cpu_halt(cs->cpu_index);

    if (!cs->halted) {
        cs->halted = 1;
        cs->exception_index = EXCP_HLT;
    }

    return s390_count_running_cpus();
}

void s390_cpu_unhalt(S390CPU *cpu)
{
    CPUState *cs = CPU(cpu);
    trace_cpu_unhalt(cs->cpu_index);

    if (cs->halted) {
        cs->halted = 0;
        cs->exception_index = -1;
    }
}

unsigned int s390_cpu_set_state(uint8_t cpu_state, S390CPU *cpu)
 {
    trace_cpu_set_state(CPU(cpu)->cpu_index, cpu_state);

    switch (cpu_state) {
    case CPU_STATE_STOPPED:
    case CPU_STATE_CHECK_STOP:
        /* halt the cpu for common infrastructure */
        s390_cpu_halt(cpu);
        break;
    case CPU_STATE_OPERATING:
    case CPU_STATE_LOAD:
        /* unhalt the cpu for common infrastructure */
        s390_cpu_unhalt(cpu);
        break;
    default:
        error_report("Requested CPU state is not a valid S390 CPU state: %u",
                     cpu_state);
        exit(1);
    }
    if (kvm_enabled() && cpu->env.cpu_state != cpu_state) {
        kvm_s390_set_cpu_state(cpu, cpu_state);
    }
    cpu->env.cpu_state = cpu_state;

    return s390_count_running_cpus();
}

int s390_get_clock(uint8_t *tod_high, uint64_t *tod_low)
{
    if (kvm_enabled()) {
        return kvm_s390_get_clock(tod_high, tod_low);
    }
    /* Fixme TCG */
    *tod_high = 0;
    *tod_low = 0;
    return 0;
}

int s390_set_clock(uint8_t *tod_high, uint64_t *tod_low)
{
    if (kvm_enabled()) {
        return kvm_s390_set_clock(tod_high, tod_low);
    }
    /* Fixme TCG */
    return 0;
}

int s390_set_memory_limit(uint64_t new_limit, uint64_t *hw_limit)
{
    if (kvm_enabled()) {
        return kvm_s390_set_mem_limit(new_limit, hw_limit);
    }
    return 0;
}

void s390_cmma_reset(void)
{
    if (kvm_enabled()) {
        kvm_s390_cmma_reset();
    }
}

int s390_cpu_restart(S390CPU *cpu)
{
    if (kvm_enabled()) {
        return kvm_s390_cpu_restart(cpu);
    }
    return -ENOSYS;
}

int s390_get_memslot_count(void)
{
    if (kvm_enabled()) {
        return kvm_s390_get_memslot_count();
    } else {
        return MAX_AVAIL_SLOTS;
    }
}

int s390_assign_subch_ioeventfd(EventNotifier *notifier, uint32_t sch_id,
                                int vq, bool assign)
{
    if (kvm_enabled()) {
        return kvm_s390_assign_subch_ioeventfd(notifier, sch_id, vq, assign);
    } else {
        return 0;
    }
}

void s390_crypto_reset(void)
{
    if (kvm_enabled()) {
        kvm_s390_crypto_reset();
    }
}

bool s390_get_squash_mcss(void)
{
    if (object_property_get_bool(OBJECT(qdev_get_machine()), "s390-squash-mcss",
                                 NULL)) {
        return true;
    }

    return false;
}

void s390_enable_css_support(S390CPU *cpu)
{
    if (kvm_enabled()) {
        kvm_s390_enable_css_support(cpu);
    }
}
#endif

static gchar *s390_gdb_arch_name(CPUState *cs)
{
    return g_strdup("s390:64-bit");
}

static Property s390x_cpu_properties[] = {
    DEFINE_PROP_UINT32("core-id", S390CPU, env.core_id, 0),
    DEFINE_PROP_END_OF_LIST()
};

static void s390_cpu_class_init(ObjectClass *oc, void *data)
{
    S390CPUClass *scc = S390_CPU_CLASS(oc);
    CPUClass *cc = CPU_CLASS(scc);
    DeviceClass *dc = DEVICE_CLASS(oc);

    scc->parent_realize = dc->realize;
    dc->realize = s390_cpu_realizefn;
    dc->props = s390x_cpu_properties;

    scc->parent_reset = cc->reset;
#if !defined(CONFIG_USER_ONLY)
    scc->load_normal = s390_cpu_load_normal;
#endif
    scc->cpu_reset = s390_cpu_reset;
    scc->initial_cpu_reset = s390_cpu_initial_reset;
    cc->reset = s390_cpu_full_reset;
    cc->class_by_name = s390_cpu_class_by_name,
    cc->has_work = s390_cpu_has_work;
#ifdef CONFIG_TCG
    cc->do_interrupt = s390_cpu_do_interrupt;
#endif
    cc->dump_state = s390_cpu_dump_state;
    cc->set_pc = s390_cpu_set_pc;
    cc->gdb_read_register = s390_cpu_gdb_read_register;
    cc->gdb_write_register = s390_cpu_gdb_write_register;
#ifdef CONFIG_USER_ONLY
    cc->handle_mmu_fault = s390_cpu_handle_mmu_fault;
#else
    cc->get_phys_page_debug = s390_cpu_get_phys_page_debug;
    cc->vmsd = &vmstate_s390_cpu;
    cc->write_elf64_note = s390_cpu_write_elf64_note;
#ifdef CONFIG_TCG
    cc->cpu_exec_interrupt = s390_cpu_exec_interrupt;
    cc->debug_excp_handler = s390x_cpu_debug_excp_handler;
    cc->do_unaligned_access = s390x_cpu_do_unaligned_access;
#endif
#endif
    cc->disas_set_info = s390_cpu_disas_set_info;

    cc->gdb_num_core_regs = S390_NUM_CORE_REGS;
    cc->gdb_core_xml_file = "s390x-core64.xml";
    cc->gdb_arch_name = s390_gdb_arch_name;

    s390_cpu_model_class_register_props(oc);
}

static const TypeInfo s390_cpu_type_info = {
    .name = TYPE_S390_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(S390CPU),
    .instance_init = s390_cpu_initfn,
    .instance_finalize = s390_cpu_finalize,
    .abstract = true,
    .class_size = sizeof(S390CPUClass),
    .class_init = s390_cpu_class_init,
};

static void s390_cpu_register_types(void)
{
    type_register_static(&s390_cpu_type_info);
}

type_init(s390_cpu_register_types)
