/*
 * ARM implementation of KVM hooks, 64 bit specific code
 *
 * Copyright Mian-M. Hamayun 2013, Virtual Open Systems
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/kvm.h>

#include "config-host.h"
#include "qemu-common.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "sysemu/sysemu.h"
#include "sysemu/kvm.h"
#include "kvm_arm.h"
#include "cpu.h"
#include "internals.h"
#include "hw/arm/arm.h"

static bool have_guest_debug;

/**
 * kvm_arm_init_debug()
 * @cs: CPUState
 *
 * Check for guest debug capabilities.
 *
 */
static void kvm_arm_init_debug(CPUState *cs)
{
    have_guest_debug = kvm_check_extension(cs->kvm_state,
                                           KVM_CAP_SET_GUEST_DEBUG);
    return;
}

static inline void set_feature(uint64_t *features, int feature)
{
    *features |= 1ULL << feature;
}

bool kvm_arm_get_host_cpu_features(ARMHostCPUClass *ahcc)
{
    /* Identify the feature bits corresponding to the host CPU, and
     * fill out the ARMHostCPUClass fields accordingly. To do this
     * we have to create a scratch VM, create a single CPU inside it,
     * and then query that CPU for the relevant ID registers.
     * For AArch64 we currently don't care about ID registers at
     * all; we just want to know the CPU type.
     */
    int fdarray[3];
    uint64_t features = 0;
    /* Old kernels may not know about the PREFERRED_TARGET ioctl: however
     * we know these will only support creating one kind of guest CPU,
     * which is its preferred CPU type. Fortunately these old kernels
     * support only a very limited number of CPUs.
     */
    static const uint32_t cpus_to_try[] = {
        KVM_ARM_TARGET_AEM_V8,
        KVM_ARM_TARGET_FOUNDATION_V8,
        KVM_ARM_TARGET_CORTEX_A57,
        QEMU_KVM_ARM_TARGET_NONE
    };
    struct kvm_vcpu_init init;

    if (!kvm_arm_create_scratch_host_vcpu(cpus_to_try, fdarray, &init)) {
        return false;
    }

    ahcc->target = init.target;
    ahcc->dtb_compatible = "arm,arm-v8";

    kvm_arm_destroy_scratch_host_vcpu(fdarray);

   /* We can assume any KVM supporting CPU is at least a v8
     * with VFPv4+Neon; this in turn implies most of the other
     * feature bits.
     */
    set_feature(&features, ARM_FEATURE_V8);
    set_feature(&features, ARM_FEATURE_VFP4);
    set_feature(&features, ARM_FEATURE_NEON);
    set_feature(&features, ARM_FEATURE_AARCH64);

    ahcc->features = features;

    return true;
}

#define ARM_CPU_ID_MPIDR       3, 0, 0, 0, 5

int kvm_arch_init_vcpu(CPUState *cs)
{
    int ret;
    uint64_t mpidr;
    ARMCPU *cpu = ARM_CPU(cs);

    if (cpu->kvm_target == QEMU_KVM_ARM_TARGET_NONE ||
        !object_dynamic_cast(OBJECT(cpu), TYPE_AARCH64_CPU)) {
        fprintf(stderr, "KVM is not supported for this guest CPU type\n");
        return -EINVAL;
    }

    /* Determine init features for this CPU */
    memset(cpu->kvm_init_features, 0, sizeof(cpu->kvm_init_features));
    if (cpu->start_powered_off) {
        cpu->kvm_init_features[0] |= 1 << KVM_ARM_VCPU_POWER_OFF;
    }
    if (kvm_check_extension(cs->kvm_state, KVM_CAP_ARM_PSCI_0_2)) {
        cpu->psci_version = 2;
        cpu->kvm_init_features[0] |= 1 << KVM_ARM_VCPU_PSCI_0_2;
    }
    if (!arm_feature(&cpu->env, ARM_FEATURE_AARCH64)) {
        cpu->kvm_init_features[0] |= 1 << KVM_ARM_VCPU_EL1_32BIT;
    }

    /* Do KVM_ARM_VCPU_INIT ioctl */
    ret = kvm_arm_vcpu_init(cs);
    if (ret) {
        return ret;
    }

    /*
     * When KVM is in use, PSCI is emulated in-kernel and not by qemu.
     * Currently KVM has its own idea about MPIDR assignment, so we
     * override our defaults with what we get from KVM.
     */
    ret = kvm_get_one_reg(cs, ARM64_SYS_REG(ARM_CPU_ID_MPIDR), &mpidr);
    if (ret) {
        return ret;
    }
    cpu->mp_affinity = mpidr & ARM64_AFFINITY_MASK;

    kvm_arm_init_debug(cs);

    return kvm_arm_init_cpreg_list(cpu);
}

bool kvm_arm_reg_syncs_via_cpreg_list(uint64_t regidx)
{
    /* Return true if the regidx is a register we should synchronize
     * via the cpreg_tuples array (ie is not a core reg we sync by
     * hand in kvm_arch_get/put_registers())
     */
    switch (regidx & KVM_REG_ARM_COPROC_MASK) {
    case KVM_REG_ARM_CORE:
        return false;
    default:
        return true;
    }
}

typedef struct CPRegStateLevel {
    uint64_t regidx;
    int level;
} CPRegStateLevel;

/* All system registers not listed in the following table are assumed to be
 * of the level KVM_PUT_RUNTIME_STATE. If a register should be written less
 * often, you must add it to this table with a state of either
 * KVM_PUT_RESET_STATE or KVM_PUT_FULL_STATE.
 */
static const CPRegStateLevel non_runtime_cpregs[] = {
    { KVM_REG_ARM_TIMER_CNT, KVM_PUT_FULL_STATE },
};

int kvm_arm_cpreg_level(uint64_t regidx)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(non_runtime_cpregs); i++) {
        const CPRegStateLevel *l = &non_runtime_cpregs[i];
        if (l->regidx == regidx) {
            return l->level;
        }
    }

    return KVM_PUT_RUNTIME_STATE;
}

#define AARCH64_CORE_REG(x)   (KVM_REG_ARM64 | KVM_REG_SIZE_U64 | \
                 KVM_REG_ARM_CORE | KVM_REG_ARM_CORE_REG(x))

#define AARCH64_SIMD_CORE_REG(x)   (KVM_REG_ARM64 | KVM_REG_SIZE_U128 | \
                 KVM_REG_ARM_CORE | KVM_REG_ARM_CORE_REG(x))

#define AARCH64_SIMD_CTRL_REG(x)   (KVM_REG_ARM64 | KVM_REG_SIZE_U32 | \
                 KVM_REG_ARM_CORE | KVM_REG_ARM_CORE_REG(x))

int kvm_arch_put_registers(CPUState *cs, int level)
{
    struct kvm_one_reg reg;
    uint32_t fpr;
    uint64_t val;
    int i;
    int ret;
    unsigned int el;

    ARMCPU *cpu = ARM_CPU(cs);
    CPUARMState *env = &cpu->env;

    /* If we are in AArch32 mode then we need to copy the AArch32 regs to the
     * AArch64 registers before pushing them out to 64-bit KVM.
     */
    if (!is_a64(env)) {
        aarch64_sync_32_to_64(env);
    }

    for (i = 0; i < 31; i++) {
        reg.id = AARCH64_CORE_REG(regs.regs[i]);
        reg.addr = (uintptr_t) &env->xregs[i];
        ret = kvm_vcpu_ioctl(cs, KVM_SET_ONE_REG, &reg);
        if (ret) {
            return ret;
        }
    }

    /* KVM puts SP_EL0 in regs.sp and SP_EL1 in regs.sp_el1. On the
     * QEMU side we keep the current SP in xregs[31] as well.
     */
    aarch64_save_sp(env, 1);

    reg.id = AARCH64_CORE_REG(regs.sp);
    reg.addr = (uintptr_t) &env->sp_el[0];
    ret = kvm_vcpu_ioctl(cs, KVM_SET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }

    reg.id = AARCH64_CORE_REG(sp_el1);
    reg.addr = (uintptr_t) &env->sp_el[1];
    ret = kvm_vcpu_ioctl(cs, KVM_SET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }

    /* Note that KVM thinks pstate is 64 bit but we use a uint32_t */
    if (is_a64(env)) {
        val = pstate_read(env);
    } else {
        val = cpsr_read(env);
    }
    reg.id = AARCH64_CORE_REG(regs.pstate);
    reg.addr = (uintptr_t) &val;
    ret = kvm_vcpu_ioctl(cs, KVM_SET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }

    reg.id = AARCH64_CORE_REG(regs.pc);
    reg.addr = (uintptr_t) &env->pc;
    ret = kvm_vcpu_ioctl(cs, KVM_SET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }

    reg.id = AARCH64_CORE_REG(elr_el1);
    reg.addr = (uintptr_t) &env->elr_el[1];
    ret = kvm_vcpu_ioctl(cs, KVM_SET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }

    /* Saved Program State Registers
     *
     * Before we restore from the banked_spsr[] array we need to
     * ensure that any modifications to env->spsr are correctly
     * reflected in the banks.
     */
    el = arm_current_el(env);
    if (el > 0 && !is_a64(env)) {
        i = bank_number(env->uncached_cpsr & CPSR_M);
        env->banked_spsr[i] = env->spsr;
    }

    /* KVM 0-4 map to QEMU banks 1-5 */
    for (i = 0; i < KVM_NR_SPSR; i++) {
        reg.id = AARCH64_CORE_REG(spsr[i]);
        reg.addr = (uintptr_t) &env->banked_spsr[i + 1];
        ret = kvm_vcpu_ioctl(cs, KVM_SET_ONE_REG, &reg);
        if (ret) {
            return ret;
        }
    }

    /* Advanced SIMD and FP registers
     * We map Qn = regs[2n+1]:regs[2n]
     */
    for (i = 0; i < 32; i++) {
        int rd = i << 1;
        uint64_t fp_val[2];
#ifdef HOST_WORDS_BIGENDIAN
        fp_val[0] = env->vfp.regs[rd + 1];
        fp_val[1] = env->vfp.regs[rd];
#else
        fp_val[1] = env->vfp.regs[rd + 1];
        fp_val[0] = env->vfp.regs[rd];
#endif
        reg.id = AARCH64_SIMD_CORE_REG(fp_regs.vregs[i]);
        reg.addr = (uintptr_t)(&fp_val);
        ret = kvm_vcpu_ioctl(cs, KVM_SET_ONE_REG, &reg);
        if (ret) {
            return ret;
        }
    }

    reg.addr = (uintptr_t)(&fpr);
    fpr = vfp_get_fpsr(env);
    reg.id = AARCH64_SIMD_CTRL_REG(fp_regs.fpsr);
    ret = kvm_vcpu_ioctl(cs, KVM_SET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }

    fpr = vfp_get_fpcr(env);
    reg.id = AARCH64_SIMD_CTRL_REG(fp_regs.fpcr);
    ret = kvm_vcpu_ioctl(cs, KVM_SET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }

    if (!write_list_to_kvmstate(cpu, level)) {
        return EINVAL;
    }

    kvm_arm_sync_mpstate_to_kvm(cpu);

    return ret;
}

int kvm_arch_get_registers(CPUState *cs)
{
    struct kvm_one_reg reg;
    uint64_t val;
    uint32_t fpr;
    unsigned int el;
    int i;
    int ret;

    ARMCPU *cpu = ARM_CPU(cs);
    CPUARMState *env = &cpu->env;

    for (i = 0; i < 31; i++) {
        reg.id = AARCH64_CORE_REG(regs.regs[i]);
        reg.addr = (uintptr_t) &env->xregs[i];
        ret = kvm_vcpu_ioctl(cs, KVM_GET_ONE_REG, &reg);
        if (ret) {
            return ret;
        }
    }

    reg.id = AARCH64_CORE_REG(regs.sp);
    reg.addr = (uintptr_t) &env->sp_el[0];
    ret = kvm_vcpu_ioctl(cs, KVM_GET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }

    reg.id = AARCH64_CORE_REG(sp_el1);
    reg.addr = (uintptr_t) &env->sp_el[1];
    ret = kvm_vcpu_ioctl(cs, KVM_GET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }

    reg.id = AARCH64_CORE_REG(regs.pstate);
    reg.addr = (uintptr_t) &val;
    ret = kvm_vcpu_ioctl(cs, KVM_GET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }

    env->aarch64 = ((val & PSTATE_nRW) == 0);
    if (is_a64(env)) {
        pstate_write(env, val);
    } else {
        env->uncached_cpsr = val & CPSR_M;
        cpsr_write(env, val, 0xffffffff);
    }

    /* KVM puts SP_EL0 in regs.sp and SP_EL1 in regs.sp_el1. On the
     * QEMU side we keep the current SP in xregs[31] as well.
     */
    aarch64_restore_sp(env, 1);

    reg.id = AARCH64_CORE_REG(regs.pc);
    reg.addr = (uintptr_t) &env->pc;
    ret = kvm_vcpu_ioctl(cs, KVM_GET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }

    /* If we are in AArch32 mode then we need to sync the AArch32 regs with the
     * incoming AArch64 regs received from 64-bit KVM.
     * We must perform this after all of the registers have been acquired from
     * the kernel.
     */
    if (!is_a64(env)) {
        aarch64_sync_64_to_32(env);
    }

    reg.id = AARCH64_CORE_REG(elr_el1);
    reg.addr = (uintptr_t) &env->elr_el[1];
    ret = kvm_vcpu_ioctl(cs, KVM_GET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }

    /* Fetch the SPSR registers
     *
     * KVM SPSRs 0-4 map to QEMU banks 1-5
     */
    for (i = 0; i < KVM_NR_SPSR; i++) {
        reg.id = AARCH64_CORE_REG(spsr[i]);
        reg.addr = (uintptr_t) &env->banked_spsr[i + 1];
        ret = kvm_vcpu_ioctl(cs, KVM_GET_ONE_REG, &reg);
        if (ret) {
            return ret;
        }
    }

    el = arm_current_el(env);
    if (el > 0 && !is_a64(env)) {
        i = bank_number(env->uncached_cpsr & CPSR_M);
        env->spsr = env->banked_spsr[i];
    }

    /* Advanced SIMD and FP registers
     * We map Qn = regs[2n+1]:regs[2n]
     */
    for (i = 0; i < 32; i++) {
        uint64_t fp_val[2];
        reg.id = AARCH64_SIMD_CORE_REG(fp_regs.vregs[i]);
        reg.addr = (uintptr_t)(&fp_val);
        ret = kvm_vcpu_ioctl(cs, KVM_GET_ONE_REG, &reg);
        if (ret) {
            return ret;
        } else {
            int rd = i << 1;
#ifdef HOST_WORDS_BIGENDIAN
            env->vfp.regs[rd + 1] = fp_val[0];
            env->vfp.regs[rd] = fp_val[1];
#else
            env->vfp.regs[rd + 1] = fp_val[1];
            env->vfp.regs[rd] = fp_val[0];
#endif
        }
    }

    reg.addr = (uintptr_t)(&fpr);
    reg.id = AARCH64_SIMD_CTRL_REG(fp_regs.fpsr);
    ret = kvm_vcpu_ioctl(cs, KVM_GET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }
    vfp_set_fpsr(env, fpr);

    reg.id = AARCH64_SIMD_CTRL_REG(fp_regs.fpcr);
    ret = kvm_vcpu_ioctl(cs, KVM_GET_ONE_REG, &reg);
    if (ret) {
        return ret;
    }
    vfp_set_fpcr(env, fpr);

    if (!write_kvmstate_to_list(cpu)) {
        return EINVAL;
    }
    /* Note that it's OK to have registers which aren't in CPUState,
     * so we can ignore a failure return here.
     */
    write_list_to_cpustate(cpu);

    kvm_arm_sync_mpstate_to_qemu(cpu);

    /* TODO: other registers */
    return ret;
}

/* C6.6.29 BRK instruction */
static const uint32_t brk_insn = 0xd4200000;

int kvm_arch_insert_sw_breakpoint(CPUState *cs, struct kvm_sw_breakpoint *bp)
{
    if (have_guest_debug) {
        if (cpu_memory_rw_debug(cs, bp->pc, (uint8_t *)&bp->saved_insn, 4, 0) ||
            cpu_memory_rw_debug(cs, bp->pc, (uint8_t *)&brk_insn, 4, 1)) {
            return -EINVAL;
        }
        return 0;
    } else {
        error_report("guest debug not supported on this kernel");
        return -EINVAL;
    }
}

int kvm_arch_remove_sw_breakpoint(CPUState *cs, struct kvm_sw_breakpoint *bp)
{
    static uint32_t brk;

    if (have_guest_debug) {
        if (cpu_memory_rw_debug(cs, bp->pc, (uint8_t *)&brk, 4, 0) ||
            brk != brk_insn ||
            cpu_memory_rw_debug(cs, bp->pc, (uint8_t *)&bp->saved_insn, 4, 1)) {
            return -EINVAL;
        }
        return 0;
    } else {
        error_report("guest debug not supported on this kernel");
        return -EINVAL;
    }
}

/* See v8 ARM ARM D7.2.27 ESR_ELx, Exception Syndrome Register
 *
 * To minimise translating between kernel and user-space the kernel
 * ABI just provides user-space with the full exception syndrome
 * register value to be decoded in QEMU.
 */

bool kvm_arm_handle_debug(CPUState *cs, struct kvm_debug_exit_arch *debug_exit)
{
    int hsr_ec = debug_exit->hsr >> ARM_EL_EC_SHIFT;
    ARMCPU *cpu = ARM_CPU(cs);
    CPUARMState *env = &cpu->env;

    /* Ensure PC is synchronised */
    kvm_cpu_synchronize_state(cs);

    switch (hsr_ec) {
    case EC_AA64_BKPT:
        if (kvm_find_sw_breakpoint(cs, env->pc)) {
            return true;
        }
        break;
    default:
        error_report("%s: unhandled debug exit (%"PRIx32", %"PRIx64")\n",
                     __func__, debug_exit->hsr, env->pc);
    }

    /* If we don't handle this it could be it really is for the
       guest to handle */
    qemu_log_mask(LOG_UNIMP,
                  "%s: re-injecting exception not yet implemented"
                  " (0x%"PRIx32", %"PRIx64")\n",
                  __func__, hsr_ec, env->pc);

    return false;
}
