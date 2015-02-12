/*
 * S390x MMU related functions
 *
 * Copyright (c) 2011 Alexander Graf
 * Copyright (c) 2015 Thomas Huth, IBM Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "cpu.h"

/* #define DEBUG_S390 */
/* #define DEBUG_S390_PTE */
/* #define DEBUG_S390_STDOUT */

#ifdef DEBUG_S390
#ifdef DEBUG_S390_STDOUT
#define DPRINTF(fmt, ...) \
    do { fprintf(stderr, fmt, ## __VA_ARGS__); \
         qemu_log(fmt, ##__VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) \
    do { qemu_log(fmt, ## __VA_ARGS__); } while (0)
#endif
#else
#define DPRINTF(fmt, ...) \
    do { } while (0)
#endif

#ifdef DEBUG_S390_PTE
#define PTE_DPRINTF DPRINTF
#else
#define PTE_DPRINTF(fmt, ...) \
    do { } while (0)
#endif

static int trans_bits(CPUS390XState *env, uint64_t mode)
{
    S390CPU *cpu = s390_env_get_cpu(env);
    int bits = 0;

    switch (mode) {
    case PSW_ASC_PRIMARY:
        bits = 1;
        break;
    case PSW_ASC_SECONDARY:
        bits = 2;
        break;
    case PSW_ASC_HOME:
        bits = 3;
        break;
    default:
        cpu_abort(CPU(cpu), "unknown asc mode\n");
        break;
    }

    return bits;
}

static void trigger_prot_fault(CPUS390XState *env, target_ulong vaddr,
                               uint64_t mode)
{
    CPUState *cs = CPU(s390_env_get_cpu(env));
    int ilen = ILEN_LATER_INC;
    int bits = trans_bits(env, mode) | 4;

    DPRINTF("%s: vaddr=%016" PRIx64 " bits=%d\n", __func__, vaddr, bits);

    stq_phys(cs->as,
             env->psa + offsetof(LowCore, trans_exc_code), vaddr | bits);
    trigger_pgm_exception(env, PGM_PROTECTION, ilen);
}

static void trigger_page_fault(CPUS390XState *env, target_ulong vaddr,
                               uint32_t type, uint64_t asc, int rw)
{
    CPUState *cs = CPU(s390_env_get_cpu(env));
    int ilen = ILEN_LATER;
    int bits = trans_bits(env, asc);

    /* Code accesses have an undefined ilc.  */
    if (rw == 2) {
        ilen = 2;
    }

    DPRINTF("%s: vaddr=%016" PRIx64 " bits=%d\n", __func__, vaddr, bits);

    stq_phys(cs->as,
             env->psa + offsetof(LowCore, trans_exc_code), vaddr | bits);
    trigger_pgm_exception(env, type, ilen);
}

/**
 * Translate real address to absolute (= physical)
 * address by taking care of the prefix mapping.
 */
static target_ulong mmu_real2abs(CPUS390XState *env, target_ulong raddr)
{
    if (raddr < 0x2000) {
        return raddr + env->psa;    /* Map the lowcore. */
    } else if (raddr >= env->psa && raddr < env->psa + 0x2000) {
        return raddr - env->psa;    /* Map the 0 page. */
    }
    return raddr;
}

/* Decode page table entry (normal 4KB page) */
static int mmu_translate_pte(CPUS390XState *env, target_ulong vaddr,
                             uint64_t asc, uint64_t asce,
                             target_ulong *raddr, int *flags, int rw)
{
    if (asce & _PAGE_INVALID) {
        DPRINTF("%s: PTE=0x%" PRIx64 " invalid\n", __func__, asce);
        trigger_page_fault(env, vaddr, PGM_PAGE_TRANS, asc, rw);
        return -1;
    }

    if (asce & _PAGE_RO) {
        *flags &= ~PAGE_WRITE;
    }

    *raddr = asce & _ASCE_ORIGIN;

    PTE_DPRINTF("%s: PTE=0x%" PRIx64 "\n", __func__, asce);

    return 0;
}

/* Decode EDAT1 segment frame absolute address (1MB page) */
static int mmu_translate_sfaa(CPUS390XState *env, target_ulong vaddr,
                              uint64_t asc, uint64_t asce, target_ulong *raddr,
                              int *flags, int rw)
{
    if (asce & _SEGMENT_ENTRY_INV) {
        DPRINTF("%s: SEG=0x%" PRIx64 " invalid\n", __func__, asce);
        trigger_page_fault(env, vaddr, PGM_SEGMENT_TRANS, asc, rw);
        return -1;
    }

    if (asce & _SEGMENT_ENTRY_RO) {
        *flags &= ~PAGE_WRITE;
    }

    *raddr = (asce & 0xfffffffffff00000ULL) | (vaddr & 0xfffff);

    PTE_DPRINTF("%s: SEG=0x%" PRIx64 "\n", __func__, asce);

    return 0;
}

static int mmu_translate_asce(CPUS390XState *env, target_ulong vaddr,
                              uint64_t asc, uint64_t asce, int level,
                              target_ulong *raddr, int *flags, int rw)
{
    CPUState *cs = CPU(s390_env_get_cpu(env));
    uint64_t offs = 0;
    uint64_t origin;
    uint64_t new_asce;

    PTE_DPRINTF("%s: 0x%" PRIx64 "\n", __func__, asce);

    if (((level != _ASCE_TYPE_SEGMENT) && (asce & _REGION_ENTRY_INV)) ||
        ((level == _ASCE_TYPE_SEGMENT) && (asce & _SEGMENT_ENTRY_INV))) {
        /* XXX different regions have different faults */
        DPRINTF("%s: invalid region\n", __func__);
        trigger_page_fault(env, vaddr, PGM_SEGMENT_TRANS, asc, rw);
        return -1;
    }

    if ((level <= _ASCE_TYPE_MASK) && ((asce & _ASCE_TYPE_MASK) != level)) {
        trigger_page_fault(env, vaddr, PGM_TRANS_SPEC, asc, rw);
        return -1;
    }

    origin = asce & _ASCE_ORIGIN;

    switch (level) {
    case _ASCE_TYPE_REGION1 + 4:
        offs = (vaddr >> 50) & 0x3ff8;
        break;
    case _ASCE_TYPE_REGION1:
        offs = (vaddr >> 39) & 0x3ff8;
        break;
    case _ASCE_TYPE_REGION2:
        offs = (vaddr >> 28) & 0x3ff8;
        break;
    case _ASCE_TYPE_REGION3:
        offs = (vaddr >> 17) & 0x3ff8;
        break;
    case _ASCE_TYPE_SEGMENT:
        offs = (vaddr >> 9) & 0x07f8;
        origin = asce & _SEGMENT_ENTRY_ORIGIN;
        break;
    }

    /* XXX region protection flags */
    /* *flags &= ~PAGE_WRITE */

    new_asce = ldq_phys(cs->as, origin + offs);
    PTE_DPRINTF("%s: 0x%" PRIx64 " + 0x%" PRIx64 " => 0x%016" PRIx64 "\n",
                __func__, origin, offs, new_asce);

    if (level == _ASCE_TYPE_SEGMENT) {
        /* 4KB page */
        return mmu_translate_pte(env, vaddr, asc, new_asce, raddr, flags, rw);
    } else if (level - 4 == _ASCE_TYPE_SEGMENT &&
               (new_asce & _SEGMENT_ENTRY_FC) && (env->cregs[0] & CR0_EDAT)) {
        /* 1MB page */
        return mmu_translate_sfaa(env, vaddr, asc, new_asce, raddr, flags, rw);
    } else {
        /* yet another region */
        return mmu_translate_asce(env, vaddr, asc, new_asce, level - 4, raddr,
                                  flags, rw);
    }
}

static int mmu_translate_asc(CPUS390XState *env, target_ulong vaddr,
                             uint64_t asc, target_ulong *raddr, int *flags,
                             int rw)
{
    uint64_t asce = 0;
    int level, new_level;
    int r;

    switch (asc) {
    case PSW_ASC_PRIMARY:
        PTE_DPRINTF("%s: asc=primary\n", __func__);
        asce = env->cregs[1];
        break;
    case PSW_ASC_SECONDARY:
        PTE_DPRINTF("%s: asc=secondary\n", __func__);
        asce = env->cregs[7];
        break;
    case PSW_ASC_HOME:
        PTE_DPRINTF("%s: asc=home\n", __func__);
        asce = env->cregs[13];
        break;
    }

    if (asce & _ASCE_REAL_SPACE) {
        /* direct mapping */
        *raddr = vaddr;
        return 0;
    }

    switch (asce & _ASCE_TYPE_MASK) {
    case _ASCE_TYPE_REGION1:
        break;
    case _ASCE_TYPE_REGION2:
        if (vaddr & 0xffe0000000000000ULL) {
            DPRINTF("%s: vaddr doesn't fit 0x%16" PRIx64
                    " 0xffe0000000000000ULL\n", __func__, vaddr);
            trigger_page_fault(env, vaddr, PGM_TRANS_SPEC, asc, rw);
            return -1;
        }
        break;
    case _ASCE_TYPE_REGION3:
        if (vaddr & 0xfffffc0000000000ULL) {
            DPRINTF("%s: vaddr doesn't fit 0x%16" PRIx64
                    " 0xfffffc0000000000ULL\n", __func__, vaddr);
            trigger_page_fault(env, vaddr, PGM_TRANS_SPEC, asc, rw);
            return -1;
        }
        break;
    case _ASCE_TYPE_SEGMENT:
        if (vaddr & 0xffffffff80000000ULL) {
            DPRINTF("%s: vaddr doesn't fit 0x%16" PRIx64
                    " 0xffffffff80000000ULL\n", __func__, vaddr);
            trigger_page_fault(env, vaddr, PGM_TRANS_SPEC, asc, rw);
            return -1;
        }
        break;
    }

    /* fake level above current */
    level = asce & _ASCE_TYPE_MASK;
    new_level = level + 4;
    asce = (asce & ~_ASCE_TYPE_MASK) | (new_level & _ASCE_TYPE_MASK);

    r = mmu_translate_asce(env, vaddr, asc, asce, new_level, raddr, flags, rw);

    if ((rw == 1) && !(*flags & PAGE_WRITE)) {
        trigger_prot_fault(env, vaddr, asc);
        return -1;
    }

    return r;
}

int mmu_translate(CPUS390XState *env, target_ulong vaddr, int rw, uint64_t asc,
                  target_ulong *raddr, int *flags)
{
    int r = -1;
    uint8_t *sk;

    *flags = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
    vaddr &= TARGET_PAGE_MASK;

    if (!(env->psw.mask & PSW_MASK_DAT)) {
        *raddr = vaddr;
        r = 0;
        goto out;
    }

    switch (asc) {
    case PSW_ASC_PRIMARY:
    case PSW_ASC_HOME:
        r = mmu_translate_asc(env, vaddr, asc, raddr, flags, rw);
        break;
    case PSW_ASC_SECONDARY:
        /*
         * Instruction: Primary
         * Data: Secondary
         */
        if (rw == 2) {
            r = mmu_translate_asc(env, vaddr, PSW_ASC_PRIMARY, raddr, flags,
                                  rw);
            *flags &= ~(PAGE_READ | PAGE_WRITE);
        } else {
            r = mmu_translate_asc(env, vaddr, PSW_ASC_SECONDARY, raddr, flags,
                                  rw);
            *flags &= ~(PAGE_EXEC);
        }
        break;
    case PSW_ASC_ACCREG:
    default:
        hw_error("guest switched to unknown asc mode\n");
        break;
    }

 out:
    /* Convert real address -> absolute address */
    *raddr = mmu_real2abs(env, *raddr);

    if (*raddr <= ram_size) {
        sk = &env->storage_keys[*raddr / TARGET_PAGE_SIZE];
        if (*flags & PAGE_READ) {
            *sk |= SK_R;
        }

        if (*flags & PAGE_WRITE) {
            *sk |= SK_C;
        }
    }

    return r;
}
