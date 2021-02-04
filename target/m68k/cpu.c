/*
 * QEMU Motorola 68k CPU
 *
 * Copyright (c) 2012 SUSE LINUX Products GmbH
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
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "cpu.h"
#include "migration/vmstate.h"
#include "fpu/softfloat.h"

static void m68k_cpu_set_pc(CPUState *cs, vaddr value)
{
    M68kCPU *cpu = M68K_CPU(cs);

    cpu->env.pc = value;
}

static bool m68k_cpu_has_work(CPUState *cs)
{
    return cs->interrupt_request & CPU_INTERRUPT_HARD;
}

static void m68k_set_feature(CPUM68KState *env, int feature)
{
    env->features |= (1u << feature);
}

static void m68k_cpu_reset(DeviceState *dev)
{
    CPUState *s = CPU(dev);
    M68kCPU *cpu = M68K_CPU(s);
    M68kCPUClass *mcc = M68K_CPU_GET_CLASS(cpu);
    CPUM68KState *env = &cpu->env;
    floatx80 nan = floatx80_default_nan(NULL);
    int i;

    mcc->parent_reset(dev);

    memset(env, 0, offsetof(CPUM68KState, end_reset_fields));
#ifdef CONFIG_SOFTMMU
    cpu_m68k_set_sr(env, SR_S | SR_I);
#else
    cpu_m68k_set_sr(env, 0);
#endif
    for (i = 0; i < 8; i++) {
        env->fregs[i].d = nan;
    }
    cpu_m68k_set_fpcr(env, 0);
    env->fpsr = 0;

    /* TODO: We should set PC from the interrupt vector.  */
    env->pc = 0;
}

static void m68k_cpu_disas_set_info(CPUState *s, disassemble_info *info)
{
    M68kCPU *cpu = M68K_CPU(s);
    CPUM68KState *env = &cpu->env;
    info->print_insn = print_insn_m68k;
    if (m68k_feature(env, M68K_FEATURE_M68000)) {
        info->mach = bfd_mach_m68040;
    }
}

/* CPU models */

static ObjectClass *m68k_cpu_class_by_name(const char *cpu_model)
{
    ObjectClass *oc;
    char *typename;

    typename = g_strdup_printf(M68K_CPU_TYPE_NAME("%s"), cpu_model);
    oc = object_class_by_name(typename);
    g_free(typename);
    if (oc != NULL && (object_class_dynamic_cast(oc, TYPE_M68K_CPU) == NULL ||
                       object_class_is_abstract(oc))) {
        return NULL;
    }
    return oc;
}

static void m5206_cpu_initfn(Object *obj)
{
    M68kCPU *cpu = M68K_CPU(obj);
    CPUM68KState *env = &cpu->env;

    m68k_set_feature(env, M68K_FEATURE_CF_ISA_A);
}

static void m68000_cpu_initfn(Object *obj)
{
    M68kCPU *cpu = M68K_CPU(obj);
    CPUM68KState *env = &cpu->env;

    m68k_set_feature(env, M68K_FEATURE_M68000);
    m68k_set_feature(env, M68K_FEATURE_USP);
    m68k_set_feature(env, M68K_FEATURE_WORD_INDEX);
    m68k_set_feature(env, M68K_FEATURE_MOVEP);
}

/* common features for 68020, 68030 and 68040 */
static void m680x0_cpu_common(CPUM68KState *env)
{
    m68k_set_feature(env, M68K_FEATURE_M68000);
    m68k_set_feature(env, M68K_FEATURE_USP);
    m68k_set_feature(env, M68K_FEATURE_WORD_INDEX);
    m68k_set_feature(env, M68K_FEATURE_QUAD_MULDIV);
    m68k_set_feature(env, M68K_FEATURE_BRAL);
    m68k_set_feature(env, M68K_FEATURE_BCCL);
    m68k_set_feature(env, M68K_FEATURE_BITFIELD);
    m68k_set_feature(env, M68K_FEATURE_EXT_FULL);
    m68k_set_feature(env, M68K_FEATURE_SCALED_INDEX);
    m68k_set_feature(env, M68K_FEATURE_LONG_MULDIV);
    m68k_set_feature(env, M68K_FEATURE_FPU);
    m68k_set_feature(env, M68K_FEATURE_CAS);
    m68k_set_feature(env, M68K_FEATURE_BKPT);
    m68k_set_feature(env, M68K_FEATURE_RTD);
    m68k_set_feature(env, M68K_FEATURE_CHK2);
    m68k_set_feature(env, M68K_FEATURE_MOVEP);
}

static void m68020_cpu_initfn(Object *obj)
{
    M68kCPU *cpu = M68K_CPU(obj);
    CPUM68KState *env = &cpu->env;

    m680x0_cpu_common(env);
    m68k_set_feature(env, M68K_FEATURE_M68020);
}

static void m68030_cpu_initfn(Object *obj)
{
    M68kCPU *cpu = M68K_CPU(obj);
    CPUM68KState *env = &cpu->env;

    m680x0_cpu_common(env);
    m68k_set_feature(env, M68K_FEATURE_M68030);
}

static void m68040_cpu_initfn(Object *obj)
{
    M68kCPU *cpu = M68K_CPU(obj);
    CPUM68KState *env = &cpu->env;

    m680x0_cpu_common(env);
    m68k_set_feature(env, M68K_FEATURE_M68040);
}

static void m68060_cpu_initfn(Object *obj)
{
    M68kCPU *cpu = M68K_CPU(obj);
    CPUM68KState *env = &cpu->env;

    m68k_set_feature(env, M68K_FEATURE_M68000);
    m68k_set_feature(env, M68K_FEATURE_USP);
    m68k_set_feature(env, M68K_FEATURE_WORD_INDEX);
    m68k_set_feature(env, M68K_FEATURE_BRAL);
    m68k_set_feature(env, M68K_FEATURE_BCCL);
    m68k_set_feature(env, M68K_FEATURE_BITFIELD);
    m68k_set_feature(env, M68K_FEATURE_EXT_FULL);
    m68k_set_feature(env, M68K_FEATURE_SCALED_INDEX);
    m68k_set_feature(env, M68K_FEATURE_LONG_MULDIV);
    m68k_set_feature(env, M68K_FEATURE_FPU);
    m68k_set_feature(env, M68K_FEATURE_CAS);
    m68k_set_feature(env, M68K_FEATURE_BKPT);
    m68k_set_feature(env, M68K_FEATURE_RTD);
    m68k_set_feature(env, M68K_FEATURE_CHK2);
    m68k_set_feature(env, M68K_FEATURE_M68060);
}

static void m5208_cpu_initfn(Object *obj)
{
    M68kCPU *cpu = M68K_CPU(obj);
    CPUM68KState *env = &cpu->env;

    m68k_set_feature(env, M68K_FEATURE_CF_ISA_A);
    m68k_set_feature(env, M68K_FEATURE_CF_ISA_APLUSC);
    m68k_set_feature(env, M68K_FEATURE_BRAL);
    m68k_set_feature(env, M68K_FEATURE_CF_EMAC);
    m68k_set_feature(env, M68K_FEATURE_USP);
}

static void cfv4e_cpu_initfn(Object *obj)
{
    M68kCPU *cpu = M68K_CPU(obj);
    CPUM68KState *env = &cpu->env;

    m68k_set_feature(env, M68K_FEATURE_CF_ISA_A);
    m68k_set_feature(env, M68K_FEATURE_CF_ISA_B);
    m68k_set_feature(env, M68K_FEATURE_BRAL);
    m68k_set_feature(env, M68K_FEATURE_CF_FPU);
    m68k_set_feature(env, M68K_FEATURE_CF_EMAC);
    m68k_set_feature(env, M68K_FEATURE_USP);
}

static void any_cpu_initfn(Object *obj)
{
    M68kCPU *cpu = M68K_CPU(obj);
    CPUM68KState *env = &cpu->env;

    m68k_set_feature(env, M68K_FEATURE_CF_ISA_A);
    m68k_set_feature(env, M68K_FEATURE_CF_ISA_B);
    m68k_set_feature(env, M68K_FEATURE_CF_ISA_APLUSC);
    m68k_set_feature(env, M68K_FEATURE_BRAL);
    m68k_set_feature(env, M68K_FEATURE_CF_FPU);
    /*
     * MAC and EMAC are mututally exclusive, so pick EMAC.
     * It's mostly backwards compatible.
     */
    m68k_set_feature(env, M68K_FEATURE_CF_EMAC);
    m68k_set_feature(env, M68K_FEATURE_CF_EMAC_B);
    m68k_set_feature(env, M68K_FEATURE_USP);
    m68k_set_feature(env, M68K_FEATURE_EXT_FULL);
    m68k_set_feature(env, M68K_FEATURE_WORD_INDEX);
}

static void m68k_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    M68kCPU *cpu = M68K_CPU(dev);
    M68kCPUClass *mcc = M68K_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    register_m68k_insns(&cpu->env);

    cpu_exec_realizefn(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    m68k_cpu_init_gdb(cpu);

    cpu_reset(cs);
    qemu_init_vcpu(cs);

    mcc->parent_realize(dev, errp);
}

static void m68k_cpu_initfn(Object *obj)
{
    M68kCPU *cpu = M68K_CPU(obj);

    cpu_set_cpustate_pointers(cpu);
}

#if defined(CONFIG_SOFTMMU)
static bool fpu_needed(void *opaque)
{
    M68kCPU *s = opaque;

    return m68k_feature(&s->env, M68K_FEATURE_CF_FPU) ||
           m68k_feature(&s->env, M68K_FEATURE_FPU);
}

typedef struct m68k_FPReg_tmp {
    FPReg *parent;
    uint64_t tmp_mant;
    uint16_t tmp_exp;
} m68k_FPReg_tmp;

static void cpu_get_fp80(uint64_t *pmant, uint16_t *pexp, floatx80 f)
{
    CPU_LDoubleU temp;

    temp.d = f;
    *pmant = temp.l.lower;
    *pexp = temp.l.upper;
}

static floatx80 cpu_set_fp80(uint64_t mant, uint16_t upper)
{
    CPU_LDoubleU temp;

    temp.l.upper = upper;
    temp.l.lower = mant;
    return temp.d;
}

static int freg_pre_save(void *opaque)
{
    m68k_FPReg_tmp *tmp = opaque;

    cpu_get_fp80(&tmp->tmp_mant, &tmp->tmp_exp, tmp->parent->d);

    return 0;
}

static int freg_post_load(void *opaque, int version)
{
    m68k_FPReg_tmp *tmp = opaque;

    tmp->parent->d = cpu_set_fp80(tmp->tmp_mant, tmp->tmp_exp);

    return 0;
}

static const VMStateDescription vmstate_freg_tmp = {
    .name = "freg_tmp",
    .post_load = freg_post_load,
    .pre_save  = freg_pre_save,
    .fields = (VMStateField[]) {
        VMSTATE_UINT64(tmp_mant, m68k_FPReg_tmp),
        VMSTATE_UINT16(tmp_exp, m68k_FPReg_tmp),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_freg = {
    .name = "freg",
    .fields = (VMStateField[]) {
        VMSTATE_WITH_TMP(FPReg, m68k_FPReg_tmp, vmstate_freg_tmp),
        VMSTATE_END_OF_LIST()
    }
};

static int fpu_post_load(void *opaque, int version)
{
    M68kCPU *s = opaque;

    cpu_m68k_restore_fp_status(&s->env);

    return 0;
}

const VMStateDescription vmmstate_fpu = {
    .name = "cpu/fpu",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = fpu_needed,
    .post_load = fpu_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(env.fpcr, M68kCPU),
        VMSTATE_UINT32(env.fpsr, M68kCPU),
        VMSTATE_STRUCT_ARRAY(env.fregs, M68kCPU, 8, 0, vmstate_freg, FPReg),
        VMSTATE_STRUCT(env.fp_result, M68kCPU, 0, vmstate_freg, FPReg),
        VMSTATE_END_OF_LIST()
    }
};

static bool cf_spregs_needed(void *opaque)
{
    M68kCPU *s = opaque;

    return m68k_feature(&s->env, M68K_FEATURE_CF_ISA_A);
}

const VMStateDescription vmstate_cf_spregs = {
    .name = "cpu/cf_spregs",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = cf_spregs_needed,
    .fields = (VMStateField[]) {
        VMSTATE_UINT64_ARRAY(env.macc, M68kCPU, 4),
        VMSTATE_UINT32(env.macsr, M68kCPU),
        VMSTATE_UINT32(env.mac_mask, M68kCPU),
        VMSTATE_UINT32(env.rambar0, M68kCPU),
        VMSTATE_UINT32(env.mbar, M68kCPU),
        VMSTATE_END_OF_LIST()
    }
};

static bool cpu_68040_mmu_needed(void *opaque)
{
    M68kCPU *s = opaque;

    return m68k_feature(&s->env, M68K_FEATURE_M68040);
}

const VMStateDescription vmstate_68040_mmu = {
    .name = "cpu/68040_mmu",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = cpu_68040_mmu_needed,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(env.mmu.ar, M68kCPU),
        VMSTATE_UINT32(env.mmu.ssw, M68kCPU),
        VMSTATE_UINT16(env.mmu.tcr, M68kCPU),
        VMSTATE_UINT32(env.mmu.urp, M68kCPU),
        VMSTATE_UINT32(env.mmu.srp, M68kCPU),
        VMSTATE_BOOL(env.mmu.fault, M68kCPU),
        VMSTATE_UINT32_ARRAY(env.mmu.ttr, M68kCPU, 4),
        VMSTATE_UINT32(env.mmu.mmusr, M68kCPU),
        VMSTATE_END_OF_LIST()
    }
};

static bool cpu_68040_spregs_needed(void *opaque)
{
    M68kCPU *s = opaque;

    return m68k_feature(&s->env, M68K_FEATURE_M68040);
}

const VMStateDescription vmstate_68040_spregs = {
    .name = "cpu/68040_spregs",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = cpu_68040_spregs_needed,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(env.vbr, M68kCPU),
        VMSTATE_UINT32(env.cacr, M68kCPU),
        VMSTATE_UINT32(env.sfc, M68kCPU),
        VMSTATE_UINT32(env.dfc, M68kCPU),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_m68k_cpu = {
    .name = "cpu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(env.dregs, M68kCPU, 8),
        VMSTATE_UINT32_ARRAY(env.aregs, M68kCPU, 8),
        VMSTATE_UINT32(env.pc, M68kCPU),
        VMSTATE_UINT32(env.sr, M68kCPU),
        VMSTATE_INT32(env.current_sp, M68kCPU),
        VMSTATE_UINT32_ARRAY(env.sp, M68kCPU, 3),
        VMSTATE_UINT32(env.cc_op, M68kCPU),
        VMSTATE_UINT32(env.cc_x, M68kCPU),
        VMSTATE_UINT32(env.cc_n, M68kCPU),
        VMSTATE_UINT32(env.cc_v, M68kCPU),
        VMSTATE_UINT32(env.cc_c, M68kCPU),
        VMSTATE_UINT32(env.cc_z, M68kCPU),
        VMSTATE_INT32(env.pending_vector, M68kCPU),
        VMSTATE_INT32(env.pending_level, M68kCPU),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * []) {
        &vmmstate_fpu,
        &vmstate_cf_spregs,
        &vmstate_68040_mmu,
        &vmstate_68040_spregs,
        NULL
    },
};
#endif

static void m68k_cpu_class_init(ObjectClass *c, void *data)
{
    M68kCPUClass *mcc = M68K_CPU_CLASS(c);
    CPUClass *cc = CPU_CLASS(c);
    DeviceClass *dc = DEVICE_CLASS(c);

    device_class_set_parent_realize(dc, m68k_cpu_realizefn,
                                    &mcc->parent_realize);
    device_class_set_parent_reset(dc, m68k_cpu_reset, &mcc->parent_reset);

    cc->class_by_name = m68k_cpu_class_by_name;
    cc->has_work = m68k_cpu_has_work;
    cc->do_interrupt = m68k_cpu_do_interrupt;
    cc->tcg_ops.cpu_exec_interrupt = m68k_cpu_exec_interrupt;
    cc->dump_state = m68k_cpu_dump_state;
    cc->set_pc = m68k_cpu_set_pc;
    cc->gdb_read_register = m68k_cpu_gdb_read_register;
    cc->gdb_write_register = m68k_cpu_gdb_write_register;
    cc->tlb_fill = m68k_cpu_tlb_fill;
#if defined(CONFIG_SOFTMMU)
    cc->do_transaction_failed = m68k_cpu_transaction_failed;
    cc->get_phys_page_debug = m68k_cpu_get_phys_page_debug;
    dc->vmsd = &vmstate_m68k_cpu;
#endif
    cc->disas_set_info = m68k_cpu_disas_set_info;
    cc->tcg_ops.initialize = m68k_tcg_init;

    cc->gdb_num_core_regs = 18;
}

static void m68k_cpu_class_init_cf_core(ObjectClass *c, void *data)
{
    CPUClass *cc = CPU_CLASS(c);

    cc->gdb_core_xml_file = "cf-core.xml";
}

#define DEFINE_M68K_CPU_TYPE_CF(model)               \
    {                                                \
        .name = M68K_CPU_TYPE_NAME(#model),          \
        .instance_init = model##_cpu_initfn,         \
        .parent = TYPE_M68K_CPU,                     \
        .class_init = m68k_cpu_class_init_cf_core    \
    }

static void m68k_cpu_class_init_m68k_core(ObjectClass *c, void *data)
{
    CPUClass *cc = CPU_CLASS(c);

    cc->gdb_core_xml_file = "m68k-core.xml";
}

#define DEFINE_M68K_CPU_TYPE_M68K(model)             \
    {                                                \
        .name = M68K_CPU_TYPE_NAME(#model),          \
        .instance_init = model##_cpu_initfn,         \
        .parent = TYPE_M68K_CPU,                     \
        .class_init = m68k_cpu_class_init_m68k_core  \
    }

static const TypeInfo m68k_cpus_type_infos[] = {
    { /* base class should be registered first */
        .name = TYPE_M68K_CPU,
        .parent = TYPE_CPU,
        .instance_size = sizeof(M68kCPU),
        .instance_init = m68k_cpu_initfn,
        .abstract = true,
        .class_size = sizeof(M68kCPUClass),
        .class_init = m68k_cpu_class_init,
    },
    DEFINE_M68K_CPU_TYPE_M68K(m68000),
    DEFINE_M68K_CPU_TYPE_M68K(m68020),
    DEFINE_M68K_CPU_TYPE_M68K(m68030),
    DEFINE_M68K_CPU_TYPE_M68K(m68040),
    DEFINE_M68K_CPU_TYPE_M68K(m68060),
    DEFINE_M68K_CPU_TYPE_CF(m5206),
    DEFINE_M68K_CPU_TYPE_CF(m5208),
    DEFINE_M68K_CPU_TYPE_CF(cfv4e),
    DEFINE_M68K_CPU_TYPE_CF(any),
};

DEFINE_TYPES(m68k_cpus_type_infos)
