/*
 * QEMU ARM CPU
 *
 * Copyright (c) 2012 SUSE LINUX Products GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see
 * <http://www.gnu.org/licenses/gpl-2.0.html>
 */

#include "cpu-qom.h"
#include "qemu-common.h"

/* CPUClass::reset() */
static void arm_cpu_reset(CPUState *s)
{
    ARMCPU *cpu = ARM_CPU(s);
    ARMCPUClass *acc = ARM_CPU_GET_CLASS(cpu);

    acc->parent_reset(s);

    /* TODO Inline the current contents of cpu_state_reset(),
            once cpu_reset_model_id() is eliminated. */
    cpu_state_reset(&cpu->env);
}

static inline void set_feature(CPUARMState *env, int feature)
{
    env->features |= 1u << feature;
}

static void arm_cpu_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);

    cpu_exec_init(&cpu->env);
}

void arm_cpu_realize(ARMCPU *cpu)
{
    /* This function is called by cpu_arm_init() because it
     * needs to do common actions based on feature bits, etc
     * that have been set by the subclass init functions.
     * When we have QOM realize support it should become
     * a true realize function instead.
     */
    CPUARMState *env = &cpu->env;
    /* Some features automatically imply others: */
    if (arm_feature(env, ARM_FEATURE_V7)) {
        set_feature(env, ARM_FEATURE_VAPA);
        set_feature(env, ARM_FEATURE_THUMB2);
        if (!arm_feature(env, ARM_FEATURE_M)) {
            set_feature(env, ARM_FEATURE_V6K);
        } else {
            set_feature(env, ARM_FEATURE_V6);
        }
    }
    if (arm_feature(env, ARM_FEATURE_V6K)) {
        set_feature(env, ARM_FEATURE_V6);
        set_feature(env, ARM_FEATURE_MVFR);
    }
    if (arm_feature(env, ARM_FEATURE_V6)) {
        set_feature(env, ARM_FEATURE_V5);
        if (!arm_feature(env, ARM_FEATURE_M)) {
            set_feature(env, ARM_FEATURE_AUXCR);
        }
    }
    if (arm_feature(env, ARM_FEATURE_V5)) {
        set_feature(env, ARM_FEATURE_V4T);
    }
    if (arm_feature(env, ARM_FEATURE_M)) {
        set_feature(env, ARM_FEATURE_THUMB_DIV);
    }
    if (arm_feature(env, ARM_FEATURE_ARM_DIV)) {
        set_feature(env, ARM_FEATURE_THUMB_DIV);
    }
    if (arm_feature(env, ARM_FEATURE_VFP4)) {
        set_feature(env, ARM_FEATURE_VFP3);
    }
    if (arm_feature(env, ARM_FEATURE_VFP3)) {
        set_feature(env, ARM_FEATURE_VFP);
    }
}

/* CPU models */

static void arm926_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_VFP);
    cpu->midr = ARM_CPUID_ARM926;
    cpu->reset_fpsid = 0x41011090;
    cpu->ctr = 0x1dd20d2;
    cpu->reset_sctlr = 0x00090078;
}

static void arm946_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_MPU);
    cpu->midr = ARM_CPUID_ARM946;
    cpu->ctr = 0x0f004006;
    cpu->reset_sctlr = 0x00000078;
}

static void arm1026_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_VFP);
    set_feature(&cpu->env, ARM_FEATURE_AUXCR);
    cpu->midr = ARM_CPUID_ARM1026;
    cpu->reset_fpsid = 0x410110a0;
    cpu->ctr = 0x1dd20d2;
    cpu->reset_sctlr = 0x00090078;
}

static void arm1136_r2_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    /* What qemu calls "arm1136_r2" is actually the 1136 r0p2, ie an
     * older core than plain "arm1136". In particular this does not
     * have the v6K features.
     * These ID register values are correct for 1136 but may be wrong
     * for 1136_r2 (in particular r0p2 does not actually implement most
     * of the ID registers).
     */
    set_feature(&cpu->env, ARM_FEATURE_V6);
    set_feature(&cpu->env, ARM_FEATURE_VFP);
    cpu->midr = ARM_CPUID_ARM1136_R2;
    cpu->reset_fpsid = 0x410120b4;
    cpu->mvfr0 = 0x11111111;
    cpu->mvfr1 = 0x00000000;
    cpu->ctr = 0x1dd20d2;
    cpu->reset_sctlr = 0x00050078;
    cpu->id_pfr0 = 0x111;
    cpu->id_pfr1 = 0x1;
    cpu->id_dfr0 = 0x2;
    cpu->id_afr0 = 0x3;
    cpu->id_mmfr0 = 0x01130003;
    cpu->id_mmfr1 = 0x10030302;
    cpu->id_mmfr2 = 0x01222110;
    cpu->id_isar0 = 0x00140011;
    cpu->id_isar1 = 0x12002111;
    cpu->id_isar2 = 0x11231111;
    cpu->id_isar3 = 0x01102131;
    cpu->id_isar4 = 0x141;
}

static void arm1136_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V6K);
    set_feature(&cpu->env, ARM_FEATURE_V6);
    set_feature(&cpu->env, ARM_FEATURE_VFP);
    cpu->midr = ARM_CPUID_ARM1136;
    cpu->reset_fpsid = 0x410120b4;
    cpu->mvfr0 = 0x11111111;
    cpu->mvfr1 = 0x00000000;
    cpu->ctr = 0x1dd20d2;
    cpu->reset_sctlr = 0x00050078;
    cpu->id_pfr0 = 0x111;
    cpu->id_pfr1 = 0x1;
    cpu->id_dfr0 = 0x2;
    cpu->id_afr0 = 0x3;
    cpu->id_mmfr0 = 0x01130003;
    cpu->id_mmfr1 = 0x10030302;
    cpu->id_mmfr2 = 0x01222110;
    cpu->id_isar0 = 0x00140011;
    cpu->id_isar1 = 0x12002111;
    cpu->id_isar2 = 0x11231111;
    cpu->id_isar3 = 0x01102131;
    cpu->id_isar4 = 0x141;
}

static void arm1176_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V6K);
    set_feature(&cpu->env, ARM_FEATURE_VFP);
    set_feature(&cpu->env, ARM_FEATURE_VAPA);
    cpu->midr = ARM_CPUID_ARM1176;
    cpu->reset_fpsid = 0x410120b5;
    cpu->mvfr0 = 0x11111111;
    cpu->mvfr1 = 0x00000000;
    cpu->ctr = 0x1dd20d2;
    cpu->reset_sctlr = 0x00050078;
    cpu->id_pfr0 = 0x111;
    cpu->id_pfr1 = 0x11;
    cpu->id_dfr0 = 0x33;
    cpu->id_afr0 = 0;
    cpu->id_mmfr0 = 0x01130003;
    cpu->id_mmfr1 = 0x10030302;
    cpu->id_mmfr2 = 0x01222100;
    cpu->id_isar0 = 0x0140011;
    cpu->id_isar1 = 0x12002111;
    cpu->id_isar2 = 0x11231121;
    cpu->id_isar3 = 0x01102131;
    cpu->id_isar4 = 0x01141;
}

static void arm11mpcore_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V6K);
    set_feature(&cpu->env, ARM_FEATURE_VFP);
    set_feature(&cpu->env, ARM_FEATURE_VAPA);
    cpu->midr = ARM_CPUID_ARM11MPCORE;
    cpu->reset_fpsid = 0x410120b4;
    cpu->mvfr0 = 0x11111111;
    cpu->mvfr1 = 0x00000000;
    cpu->ctr = 0x1dd20d2;
    cpu->id_pfr0 = 0x111;
    cpu->id_pfr1 = 0x1;
    cpu->id_dfr0 = 0;
    cpu->id_afr0 = 0x2;
    cpu->id_mmfr0 = 0x01100103;
    cpu->id_mmfr1 = 0x10020302;
    cpu->id_mmfr2 = 0x01222000;
    cpu->id_isar0 = 0x00100011;
    cpu->id_isar1 = 0x12002111;
    cpu->id_isar2 = 0x11221011;
    cpu->id_isar3 = 0x01102131;
    cpu->id_isar4 = 0x141;
}

static void cortex_m3_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V7);
    set_feature(&cpu->env, ARM_FEATURE_M);
    cpu->midr = ARM_CPUID_CORTEXM3;
}

static void cortex_a8_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V7);
    set_feature(&cpu->env, ARM_FEATURE_VFP3);
    set_feature(&cpu->env, ARM_FEATURE_NEON);
    set_feature(&cpu->env, ARM_FEATURE_THUMB2EE);
    cpu->midr = ARM_CPUID_CORTEXA8;
    cpu->reset_fpsid = 0x410330c0;
    cpu->mvfr0 = 0x11110222;
    cpu->mvfr1 = 0x00011100;
    cpu->ctr = 0x82048004;
    cpu->reset_sctlr = 0x00c50078;
    cpu->id_pfr0 = 0x1031;
    cpu->id_pfr1 = 0x11;
    cpu->id_dfr0 = 0x400;
    cpu->id_afr0 = 0;
    cpu->id_mmfr0 = 0x31100003;
    cpu->id_mmfr1 = 0x20000000;
    cpu->id_mmfr2 = 0x01202000;
    cpu->id_mmfr3 = 0x11;
    cpu->id_isar0 = 0x00101111;
    cpu->id_isar1 = 0x12112111;
    cpu->id_isar2 = 0x21232031;
    cpu->id_isar3 = 0x11112131;
    cpu->id_isar4 = 0x00111142;
    cpu->clidr = (1 << 27) | (2 << 24) | 3;
    cpu->ccsidr[0] = 0xe007e01a; /* 16k L1 dcache. */
    cpu->ccsidr[1] = 0x2007e01a; /* 16k L1 icache. */
    cpu->ccsidr[2] = 0xf0000000; /* No L2 icache. */
}

static void cortex_a9_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V7);
    set_feature(&cpu->env, ARM_FEATURE_VFP3);
    set_feature(&cpu->env, ARM_FEATURE_VFP_FP16);
    set_feature(&cpu->env, ARM_FEATURE_NEON);
    set_feature(&cpu->env, ARM_FEATURE_THUMB2EE);
    /* Note that A9 supports the MP extensions even for
     * A9UP and single-core A9MP (which are both different
     * and valid configurations; we don't model A9UP).
     */
    set_feature(&cpu->env, ARM_FEATURE_V7MP);
    cpu->midr = ARM_CPUID_CORTEXA9;
    cpu->reset_fpsid = 0x41033090;
    cpu->mvfr0 = 0x11110222;
    cpu->mvfr1 = 0x01111111;
    cpu->ctr = 0x80038003;
    cpu->reset_sctlr = 0x00c50078;
    cpu->id_pfr0 = 0x1031;
    cpu->id_pfr1 = 0x11;
    cpu->id_dfr0 = 0x000;
    cpu->id_afr0 = 0;
    cpu->id_mmfr0 = 0x00100103;
    cpu->id_mmfr1 = 0x20000000;
    cpu->id_mmfr2 = 0x01230000;
    cpu->id_mmfr3 = 0x00002111;
    cpu->id_isar0 = 0x00101111;
    cpu->id_isar1 = 0x13112111;
    cpu->id_isar2 = 0x21232041;
    cpu->id_isar3 = 0x11112131;
    cpu->id_isar4 = 0x00111142;
    cpu->clidr = (1 << 27) | (1 << 24) | 3;
    cpu->ccsidr[0] = 0xe00fe015; /* 16k L1 dcache. */
    cpu->ccsidr[1] = 0x200fe015; /* 16k L1 icache. */
}

static void cortex_a15_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V7);
    set_feature(&cpu->env, ARM_FEATURE_VFP4);
    set_feature(&cpu->env, ARM_FEATURE_VFP_FP16);
    set_feature(&cpu->env, ARM_FEATURE_NEON);
    set_feature(&cpu->env, ARM_FEATURE_THUMB2EE);
    set_feature(&cpu->env, ARM_FEATURE_ARM_DIV);
    set_feature(&cpu->env, ARM_FEATURE_V7MP);
    set_feature(&cpu->env, ARM_FEATURE_GENERIC_TIMER);
    cpu->midr = ARM_CPUID_CORTEXA15;
    cpu->reset_fpsid = 0x410430f0;
    cpu->mvfr0 = 0x10110222;
    cpu->mvfr1 = 0x11111111;
    cpu->ctr = 0x8444c004;
    cpu->reset_sctlr = 0x00c50078;
    cpu->id_pfr0 = 0x00001131;
    cpu->id_pfr1 = 0x00011011;
    cpu->id_dfr0 = 0x02010555;
    cpu->id_afr0 = 0x00000000;
    cpu->id_mmfr0 = 0x10201105;
    cpu->id_mmfr1 = 0x20000000;
    cpu->id_mmfr2 = 0x01240000;
    cpu->id_mmfr3 = 0x02102211;
    cpu->id_isar0 = 0x02101110;
    cpu->id_isar1 = 0x13112111;
    cpu->id_isar2 = 0x21232041;
    cpu->id_isar3 = 0x11112131;
    cpu->id_isar4 = 0x10011142;
    cpu->clidr = 0x0a200023;
    cpu->ccsidr[0] = 0x701fe00a; /* 32K L1 dcache */
    cpu->ccsidr[1] = 0x201fe00a; /* 32K L1 icache */
    cpu->ccsidr[2] = 0x711fe07a; /* 4096K L2 unified cache */
}

static void ti925t_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V4T);
    set_feature(&cpu->env, ARM_FEATURE_OMAPCP);
    cpu->midr = ARM_CPUID_TI925T;
    cpu->ctr = 0x5109149;
    cpu->reset_sctlr = 0x00000070;
}

static void sa1100_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_STRONGARM);
    cpu->midr = ARM_CPUID_SA1100;
    cpu->reset_sctlr = 0x00000070;
}

static void sa1110_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_STRONGARM);
    cpu->midr = ARM_CPUID_SA1110;
    cpu->reset_sctlr = 0x00000070;
}

static void pxa250_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_XSCALE);
    cpu->midr = ARM_CPUID_PXA250;
    cpu->ctr = 0xd172172;
    cpu->reset_sctlr = 0x00000078;
}

static void pxa255_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_XSCALE);
    cpu->midr = ARM_CPUID_PXA255;
    cpu->ctr = 0xd172172;
    cpu->reset_sctlr = 0x00000078;
}

static void pxa260_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_XSCALE);
    cpu->midr = ARM_CPUID_PXA260;
    cpu->ctr = 0xd172172;
    cpu->reset_sctlr = 0x00000078;
}

static void pxa261_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_XSCALE);
    cpu->midr = ARM_CPUID_PXA261;
    cpu->ctr = 0xd172172;
    cpu->reset_sctlr = 0x00000078;
}

static void pxa262_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_XSCALE);
    cpu->midr = ARM_CPUID_PXA262;
    cpu->ctr = 0xd172172;
    cpu->reset_sctlr = 0x00000078;
}

static void pxa270a0_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_XSCALE);
    set_feature(&cpu->env, ARM_FEATURE_IWMMXT);
    cpu->midr = ARM_CPUID_PXA270_A0;
    cpu->ctr = 0xd172172;
    cpu->reset_sctlr = 0x00000078;
}

static void pxa270a1_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_XSCALE);
    set_feature(&cpu->env, ARM_FEATURE_IWMMXT);
    cpu->midr = ARM_CPUID_PXA270_A1;
    cpu->ctr = 0xd172172;
    cpu->reset_sctlr = 0x00000078;
}

static void pxa270b0_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_XSCALE);
    set_feature(&cpu->env, ARM_FEATURE_IWMMXT);
    cpu->midr = ARM_CPUID_PXA270_B0;
    cpu->ctr = 0xd172172;
    cpu->reset_sctlr = 0x00000078;
}

static void pxa270b1_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_XSCALE);
    set_feature(&cpu->env, ARM_FEATURE_IWMMXT);
    cpu->midr = ARM_CPUID_PXA270_B1;
    cpu->ctr = 0xd172172;
    cpu->reset_sctlr = 0x00000078;
}

static void pxa270c0_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_XSCALE);
    set_feature(&cpu->env, ARM_FEATURE_IWMMXT);
    cpu->midr = ARM_CPUID_PXA270_C0;
    cpu->ctr = 0xd172172;
    cpu->reset_sctlr = 0x00000078;
}

static void pxa270c5_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V5);
    set_feature(&cpu->env, ARM_FEATURE_XSCALE);
    set_feature(&cpu->env, ARM_FEATURE_IWMMXT);
    cpu->midr = ARM_CPUID_PXA270_C5;
    cpu->ctr = 0xd172172;
    cpu->reset_sctlr = 0x00000078;
}

static void arm_any_initfn(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V7);
    set_feature(&cpu->env, ARM_FEATURE_VFP4);
    set_feature(&cpu->env, ARM_FEATURE_VFP_FP16);
    set_feature(&cpu->env, ARM_FEATURE_NEON);
    set_feature(&cpu->env, ARM_FEATURE_THUMB2EE);
    set_feature(&cpu->env, ARM_FEATURE_ARM_DIV);
    set_feature(&cpu->env, ARM_FEATURE_V7MP);
    cpu->midr = ARM_CPUID_ANY;
}

typedef struct ARMCPUInfo {
    const char *name;
    void (*initfn)(Object *obj);
} ARMCPUInfo;

static const ARMCPUInfo arm_cpus[] = {
    { .name = "arm926",      .initfn = arm926_initfn },
    { .name = "arm946",      .initfn = arm946_initfn },
    { .name = "arm1026",     .initfn = arm1026_initfn },
    /* What QEMU calls "arm1136-r2" is actually the 1136 r0p2, i.e. an
     * older core than plain "arm1136". In particular this does not
     * have the v6K features.
     */
    { .name = "arm1136-r2",  .initfn = arm1136_r2_initfn },
    { .name = "arm1136",     .initfn = arm1136_initfn },
    { .name = "arm1176",     .initfn = arm1176_initfn },
    { .name = "arm11mpcore", .initfn = arm11mpcore_initfn },
    { .name = "cortex-m3",   .initfn = cortex_m3_initfn },
    { .name = "cortex-a8",   .initfn = cortex_a8_initfn },
    { .name = "cortex-a9",   .initfn = cortex_a9_initfn },
    { .name = "cortex-a15",  .initfn = cortex_a15_initfn },
    { .name = "ti925t",      .initfn = ti925t_initfn },
    { .name = "sa1100",      .initfn = sa1100_initfn },
    { .name = "sa1110",      .initfn = sa1110_initfn },
    { .name = "pxa250",      .initfn = pxa250_initfn },
    { .name = "pxa255",      .initfn = pxa255_initfn },
    { .name = "pxa260",      .initfn = pxa260_initfn },
    { .name = "pxa261",      .initfn = pxa261_initfn },
    { .name = "pxa262",      .initfn = pxa262_initfn },
    /* "pxa270" is an alias for "pxa270-a0" */
    { .name = "pxa270",      .initfn = pxa270a0_initfn },
    { .name = "pxa270-a0",   .initfn = pxa270a0_initfn },
    { .name = "pxa270-a1",   .initfn = pxa270a1_initfn },
    { .name = "pxa270-b0",   .initfn = pxa270b0_initfn },
    { .name = "pxa270-b1",   .initfn = pxa270b1_initfn },
    { .name = "pxa270-c0",   .initfn = pxa270c0_initfn },
    { .name = "pxa270-c5",   .initfn = pxa270c5_initfn },
    { .name = "any",         .initfn = arm_any_initfn },
};

static void arm_cpu_class_init(ObjectClass *oc, void *data)
{
    ARMCPUClass *acc = ARM_CPU_CLASS(oc);
    CPUClass *cc = CPU_CLASS(acc);

    acc->parent_reset = cc->reset;
    cc->reset = arm_cpu_reset;
}

static void cpu_register(const ARMCPUInfo *info)
{
    TypeInfo type_info = {
        .name = info->name,
        .parent = TYPE_ARM_CPU,
        .instance_size = sizeof(ARMCPU),
        .instance_init = info->initfn,
        .class_size = sizeof(ARMCPUClass),
    };

    type_register_static(&type_info);
}

static const TypeInfo arm_cpu_type_info = {
    .name = TYPE_ARM_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(ARMCPU),
    .instance_init = arm_cpu_initfn,
    .abstract = true,
    .class_size = sizeof(ARMCPUClass),
    .class_init = arm_cpu_class_init,
};

static void arm_cpu_register_types(void)
{
    int i;

    type_register_static(&arm_cpu_type_info);
    for (i = 0; i < ARRAY_SIZE(arm_cpus); i++) {
        cpu_register(&arm_cpus[i]);
    }
}

type_init(arm_cpu_register_types)
