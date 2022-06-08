/*
 * ARM page table walking.
 *
 * This code is licensed under the GNU GPL v2 or later.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/range.h"
#include "cpu.h"
#include "internals.h"
#include "idau.h"
#include "ptw.h"


static bool get_phys_addr_lpae(CPUARMState *env, uint64_t address,
                               MMUAccessType access_type, ARMMMUIdx mmu_idx,
                               bool s1_is_el0, hwaddr *phys_ptr,
                               MemTxAttrs *txattrs, int *prot,
                               target_ulong *page_size_ptr,
                               ARMMMUFaultInfo *fi, ARMCacheAttrs *cacheattrs)
    __attribute__((nonnull));

/* This mapping is common between ID_AA64MMFR0.PARANGE and TCR_ELx.{I}PS. */
static const uint8_t pamax_map[] = {
    [0] = 32,
    [1] = 36,
    [2] = 40,
    [3] = 42,
    [4] = 44,
    [5] = 48,
    [6] = 52,
};

/* The cpu-specific constant value of PAMax; also used by hw/arm/virt. */
unsigned int arm_pamax(ARMCPU *cpu)
{
    unsigned int parange =
        FIELD_EX64(cpu->isar.id_aa64mmfr0, ID_AA64MMFR0, PARANGE);

    /*
     * id_aa64mmfr0 is a read-only register so values outside of the
     * supported mappings can be considered an implementation error.
     */
    assert(parange < ARRAY_SIZE(pamax_map));
    return pamax_map[parange];
}

static bool regime_translation_big_endian(CPUARMState *env, ARMMMUIdx mmu_idx)
{
    return (regime_sctlr(env, mmu_idx) & SCTLR_EE) != 0;
}

static bool ptw_attrs_are_device(CPUARMState *env, ARMCacheAttrs cacheattrs)
{
    /*
     * For an S1 page table walk, the stage 1 attributes are always
     * some form of "this is Normal memory". The combined S1+S2
     * attributes are therefore only Device if stage 2 specifies Device.
     * With HCR_EL2.FWB == 0 this is when descriptor bits [5:4] are 0b00,
     * ie when cacheattrs.attrs bits [3:2] are 0b00.
     * With HCR_EL2.FWB == 1 this is when descriptor bit [4] is 0, ie
     * when cacheattrs.attrs bit [2] is 0.
     */
    assert(cacheattrs.is_s2_format);
    if (arm_hcr_el2_eff(env) & HCR_FWB) {
        return (cacheattrs.attrs & 0x4) == 0;
    } else {
        return (cacheattrs.attrs & 0xc) == 0;
    }
}

/* Translate a S1 pagetable walk through S2 if needed.  */
static hwaddr S1_ptw_translate(CPUARMState *env, ARMMMUIdx mmu_idx,
                               hwaddr addr, bool *is_secure,
                               ARMMMUFaultInfo *fi)
{
    if (arm_mmu_idx_is_stage1_of_2(mmu_idx) &&
        !regime_translation_disabled(env, ARMMMUIdx_Stage2)) {
        target_ulong s2size;
        hwaddr s2pa;
        int s2prot;
        int ret;
        ARMMMUIdx s2_mmu_idx = *is_secure ? ARMMMUIdx_Stage2_S
                                          : ARMMMUIdx_Stage2;
        ARMCacheAttrs cacheattrs = {};
        MemTxAttrs txattrs = {};

        ret = get_phys_addr_lpae(env, addr, MMU_DATA_LOAD, s2_mmu_idx, false,
                                 &s2pa, &txattrs, &s2prot, &s2size, fi,
                                 &cacheattrs);
        if (ret) {
            assert(fi->type != ARMFault_None);
            fi->s2addr = addr;
            fi->stage2 = true;
            fi->s1ptw = true;
            fi->s1ns = !*is_secure;
            return ~0;
        }
        if ((arm_hcr_el2_eff(env) & HCR_PTW) &&
            ptw_attrs_are_device(env, cacheattrs)) {
            /*
             * PTW set and S1 walk touched S2 Device memory:
             * generate Permission fault.
             */
            fi->type = ARMFault_Permission;
            fi->s2addr = addr;
            fi->stage2 = true;
            fi->s1ptw = true;
            fi->s1ns = !*is_secure;
            return ~0;
        }

        if (arm_is_secure_below_el3(env)) {
            /* Check if page table walk is to secure or non-secure PA space. */
            if (*is_secure) {
                *is_secure = !(env->cp15.vstcr_el2.raw_tcr & VSTCR_SW);
            } else {
                *is_secure = !(env->cp15.vtcr_el2.raw_tcr & VTCR_NSW);
            }
        } else {
            assert(!*is_secure);
        }

        addr = s2pa;
    }
    return addr;
}

/* All loads done in the course of a page table walk go through here. */
static uint32_t arm_ldl_ptw(CPUState *cs, hwaddr addr, bool is_secure,
                            ARMMMUIdx mmu_idx, ARMMMUFaultInfo *fi)
{
    ARMCPU *cpu = ARM_CPU(cs);
    CPUARMState *env = &cpu->env;
    MemTxAttrs attrs = {};
    MemTxResult result = MEMTX_OK;
    AddressSpace *as;
    uint32_t data;

    addr = S1_ptw_translate(env, mmu_idx, addr, &is_secure, fi);
    attrs.secure = is_secure;
    as = arm_addressspace(cs, attrs);
    if (fi->s1ptw) {
        return 0;
    }
    if (regime_translation_big_endian(env, mmu_idx)) {
        data = address_space_ldl_be(as, addr, attrs, &result);
    } else {
        data = address_space_ldl_le(as, addr, attrs, &result);
    }
    if (result == MEMTX_OK) {
        return data;
    }
    fi->type = ARMFault_SyncExternalOnWalk;
    fi->ea = arm_extabort_type(result);
    return 0;
}

static uint64_t arm_ldq_ptw(CPUState *cs, hwaddr addr, bool is_secure,
                            ARMMMUIdx mmu_idx, ARMMMUFaultInfo *fi)
{
    ARMCPU *cpu = ARM_CPU(cs);
    CPUARMState *env = &cpu->env;
    MemTxAttrs attrs = {};
    MemTxResult result = MEMTX_OK;
    AddressSpace *as;
    uint64_t data;

    addr = S1_ptw_translate(env, mmu_idx, addr, &is_secure, fi);
    attrs.secure = is_secure;
    as = arm_addressspace(cs, attrs);
    if (fi->s1ptw) {
        return 0;
    }
    if (regime_translation_big_endian(env, mmu_idx)) {
        data = address_space_ldq_be(as, addr, attrs, &result);
    } else {
        data = address_space_ldq_le(as, addr, attrs, &result);
    }
    if (result == MEMTX_OK) {
        return data;
    }
    fi->type = ARMFault_SyncExternalOnWalk;
    fi->ea = arm_extabort_type(result);
    return 0;
}

static bool get_level1_table_address(CPUARMState *env, ARMMMUIdx mmu_idx,
                                     uint32_t *table, uint32_t address)
{
    /* Note that we can only get here for an AArch32 PL0/PL1 lookup */
    TCR *tcr = regime_tcr(env, mmu_idx);

    if (address & tcr->mask) {
        if (tcr->raw_tcr & TTBCR_PD1) {
            /* Translation table walk disabled for TTBR1 */
            return false;
        }
        *table = regime_ttbr(env, mmu_idx, 1) & 0xffffc000;
    } else {
        if (tcr->raw_tcr & TTBCR_PD0) {
            /* Translation table walk disabled for TTBR0 */
            return false;
        }
        *table = regime_ttbr(env, mmu_idx, 0) & tcr->base_mask;
    }
    *table |= (address >> 18) & 0x3ffc;
    return true;
}

static bool get_phys_addr_v5(CPUARMState *env, uint32_t address,
                             MMUAccessType access_type, ARMMMUIdx mmu_idx,
                             hwaddr *phys_ptr, int *prot,
                             target_ulong *page_size,
                             ARMMMUFaultInfo *fi)
{
    CPUState *cs = env_cpu(env);
    int level = 1;
    uint32_t table;
    uint32_t desc;
    int type;
    int ap;
    int domain = 0;
    int domain_prot;
    hwaddr phys_addr;
    uint32_t dacr;

    /* Pagetable walk.  */
    /* Lookup l1 descriptor.  */
    if (!get_level1_table_address(env, mmu_idx, &table, address)) {
        /* Section translation fault if page walk is disabled by PD0 or PD1 */
        fi->type = ARMFault_Translation;
        goto do_fault;
    }
    desc = arm_ldl_ptw(cs, table, regime_is_secure(env, mmu_idx),
                       mmu_idx, fi);
    if (fi->type != ARMFault_None) {
        goto do_fault;
    }
    type = (desc & 3);
    domain = (desc >> 5) & 0x0f;
    if (regime_el(env, mmu_idx) == 1) {
        dacr = env->cp15.dacr_ns;
    } else {
        dacr = env->cp15.dacr_s;
    }
    domain_prot = (dacr >> (domain * 2)) & 3;
    if (type == 0) {
        /* Section translation fault.  */
        fi->type = ARMFault_Translation;
        goto do_fault;
    }
    if (type != 2) {
        level = 2;
    }
    if (domain_prot == 0 || domain_prot == 2) {
        fi->type = ARMFault_Domain;
        goto do_fault;
    }
    if (type == 2) {
        /* 1Mb section.  */
        phys_addr = (desc & 0xfff00000) | (address & 0x000fffff);
        ap = (desc >> 10) & 3;
        *page_size = 1024 * 1024;
    } else {
        /* Lookup l2 entry.  */
        if (type == 1) {
            /* Coarse pagetable.  */
            table = (desc & 0xfffffc00) | ((address >> 10) & 0x3fc);
        } else {
            /* Fine pagetable.  */
            table = (desc & 0xfffff000) | ((address >> 8) & 0xffc);
        }
        desc = arm_ldl_ptw(cs, table, regime_is_secure(env, mmu_idx),
                           mmu_idx, fi);
        if (fi->type != ARMFault_None) {
            goto do_fault;
        }
        switch (desc & 3) {
        case 0: /* Page translation fault.  */
            fi->type = ARMFault_Translation;
            goto do_fault;
        case 1: /* 64k page.  */
            phys_addr = (desc & 0xffff0000) | (address & 0xffff);
            ap = (desc >> (4 + ((address >> 13) & 6))) & 3;
            *page_size = 0x10000;
            break;
        case 2: /* 4k page.  */
            phys_addr = (desc & 0xfffff000) | (address & 0xfff);
            ap = (desc >> (4 + ((address >> 9) & 6))) & 3;
            *page_size = 0x1000;
            break;
        case 3: /* 1k page, or ARMv6/XScale "extended small (4k) page" */
            if (type == 1) {
                /* ARMv6/XScale extended small page format */
                if (arm_feature(env, ARM_FEATURE_XSCALE)
                    || arm_feature(env, ARM_FEATURE_V6)) {
                    phys_addr = (desc & 0xfffff000) | (address & 0xfff);
                    *page_size = 0x1000;
                } else {
                    /*
                     * UNPREDICTABLE in ARMv5; we choose to take a
                     * page translation fault.
                     */
                    fi->type = ARMFault_Translation;
                    goto do_fault;
                }
            } else {
                phys_addr = (desc & 0xfffffc00) | (address & 0x3ff);
                *page_size = 0x400;
            }
            ap = (desc >> 4) & 3;
            break;
        default:
            /* Never happens, but compiler isn't smart enough to tell.  */
            g_assert_not_reached();
        }
    }
    *prot = ap_to_rw_prot(env, mmu_idx, ap, domain_prot);
    *prot |= *prot ? PAGE_EXEC : 0;
    if (!(*prot & (1 << access_type))) {
        /* Access permission fault.  */
        fi->type = ARMFault_Permission;
        goto do_fault;
    }
    *phys_ptr = phys_addr;
    return false;
do_fault:
    fi->domain = domain;
    fi->level = level;
    return true;
}

static bool get_phys_addr_v6(CPUARMState *env, uint32_t address,
                             MMUAccessType access_type, ARMMMUIdx mmu_idx,
                             hwaddr *phys_ptr, MemTxAttrs *attrs, int *prot,
                             target_ulong *page_size, ARMMMUFaultInfo *fi)
{
    CPUState *cs = env_cpu(env);
    ARMCPU *cpu = env_archcpu(env);
    int level = 1;
    uint32_t table;
    uint32_t desc;
    uint32_t xn;
    uint32_t pxn = 0;
    int type;
    int ap;
    int domain = 0;
    int domain_prot;
    hwaddr phys_addr;
    uint32_t dacr;
    bool ns;

    /* Pagetable walk.  */
    /* Lookup l1 descriptor.  */
    if (!get_level1_table_address(env, mmu_idx, &table, address)) {
        /* Section translation fault if page walk is disabled by PD0 or PD1 */
        fi->type = ARMFault_Translation;
        goto do_fault;
    }
    desc = arm_ldl_ptw(cs, table, regime_is_secure(env, mmu_idx),
                       mmu_idx, fi);
    if (fi->type != ARMFault_None) {
        goto do_fault;
    }
    type = (desc & 3);
    if (type == 0 || (type == 3 && !cpu_isar_feature(aa32_pxn, cpu))) {
        /* Section translation fault, or attempt to use the encoding
         * which is Reserved on implementations without PXN.
         */
        fi->type = ARMFault_Translation;
        goto do_fault;
    }
    if ((type == 1) || !(desc & (1 << 18))) {
        /* Page or Section.  */
        domain = (desc >> 5) & 0x0f;
    }
    if (regime_el(env, mmu_idx) == 1) {
        dacr = env->cp15.dacr_ns;
    } else {
        dacr = env->cp15.dacr_s;
    }
    if (type == 1) {
        level = 2;
    }
    domain_prot = (dacr >> (domain * 2)) & 3;
    if (domain_prot == 0 || domain_prot == 2) {
        /* Section or Page domain fault */
        fi->type = ARMFault_Domain;
        goto do_fault;
    }
    if (type != 1) {
        if (desc & (1 << 18)) {
            /* Supersection.  */
            phys_addr = (desc & 0xff000000) | (address & 0x00ffffff);
            phys_addr |= (uint64_t)extract32(desc, 20, 4) << 32;
            phys_addr |= (uint64_t)extract32(desc, 5, 4) << 36;
            *page_size = 0x1000000;
        } else {
            /* Section.  */
            phys_addr = (desc & 0xfff00000) | (address & 0x000fffff);
            *page_size = 0x100000;
        }
        ap = ((desc >> 10) & 3) | ((desc >> 13) & 4);
        xn = desc & (1 << 4);
        pxn = desc & 1;
        ns = extract32(desc, 19, 1);
    } else {
        if (cpu_isar_feature(aa32_pxn, cpu)) {
            pxn = (desc >> 2) & 1;
        }
        ns = extract32(desc, 3, 1);
        /* Lookup l2 entry.  */
        table = (desc & 0xfffffc00) | ((address >> 10) & 0x3fc);
        desc = arm_ldl_ptw(cs, table, regime_is_secure(env, mmu_idx),
                           mmu_idx, fi);
        if (fi->type != ARMFault_None) {
            goto do_fault;
        }
        ap = ((desc >> 4) & 3) | ((desc >> 7) & 4);
        switch (desc & 3) {
        case 0: /* Page translation fault.  */
            fi->type = ARMFault_Translation;
            goto do_fault;
        case 1: /* 64k page.  */
            phys_addr = (desc & 0xffff0000) | (address & 0xffff);
            xn = desc & (1 << 15);
            *page_size = 0x10000;
            break;
        case 2: case 3: /* 4k page.  */
            phys_addr = (desc & 0xfffff000) | (address & 0xfff);
            xn = desc & 1;
            *page_size = 0x1000;
            break;
        default:
            /* Never happens, but compiler isn't smart enough to tell.  */
            g_assert_not_reached();
        }
    }
    if (domain_prot == 3) {
        *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
    } else {
        if (pxn && !regime_is_user(env, mmu_idx)) {
            xn = 1;
        }
        if (xn && access_type == MMU_INST_FETCH) {
            fi->type = ARMFault_Permission;
            goto do_fault;
        }

        if (arm_feature(env, ARM_FEATURE_V6K) &&
                (regime_sctlr(env, mmu_idx) & SCTLR_AFE)) {
            /* The simplified model uses AP[0] as an access control bit.  */
            if ((ap & 1) == 0) {
                /* Access flag fault.  */
                fi->type = ARMFault_AccessFlag;
                goto do_fault;
            }
            *prot = simple_ap_to_rw_prot(env, mmu_idx, ap >> 1);
        } else {
            *prot = ap_to_rw_prot(env, mmu_idx, ap, domain_prot);
        }
        if (*prot && !xn) {
            *prot |= PAGE_EXEC;
        }
        if (!(*prot & (1 << access_type))) {
            /* Access permission fault.  */
            fi->type = ARMFault_Permission;
            goto do_fault;
        }
    }
    if (ns) {
        /* The NS bit will (as required by the architecture) have no effect if
         * the CPU doesn't support TZ or this is a non-secure translation
         * regime, because the attribute will already be non-secure.
         */
        attrs->secure = false;
    }
    *phys_ptr = phys_addr;
    return false;
do_fault:
    fi->domain = domain;
    fi->level = level;
    return true;
}

/**
 * get_phys_addr_lpae: perform one stage of page table walk, LPAE format
 *
 * Returns false if the translation was successful. Otherwise, phys_ptr,
 * attrs, prot and page_size may not be filled in, and the populated fsr
 * value provides information on why the translation aborted, in the format
 * of a long-format DFSR/IFSR fault register, with the following caveat:
 * the WnR bit is never set (the caller must do this).
 *
 * @env: CPUARMState
 * @address: virtual address to get physical address for
 * @access_type: MMU_DATA_LOAD, MMU_DATA_STORE or MMU_INST_FETCH
 * @mmu_idx: MMU index indicating required translation regime
 * @s1_is_el0: if @mmu_idx is ARMMMUIdx_Stage2 (so this is a stage 2 page
 *             table walk), must be true if this is stage 2 of a stage 1+2
 *             walk for an EL0 access. If @mmu_idx is anything else,
 *             @s1_is_el0 is ignored.
 * @phys_ptr: set to the physical address corresponding to the virtual address
 * @attrs: set to the memory transaction attributes to use
 * @prot: set to the permissions for the page containing phys_ptr
 * @page_size_ptr: set to the size of the page containing phys_ptr
 * @fi: set to fault info if the translation fails
 * @cacheattrs: (if non-NULL) set to the cacheability/shareability attributes
 */
static bool get_phys_addr_lpae(CPUARMState *env, uint64_t address,
                               MMUAccessType access_type, ARMMMUIdx mmu_idx,
                               bool s1_is_el0, hwaddr *phys_ptr,
                               MemTxAttrs *txattrs, int *prot,
                               target_ulong *page_size_ptr,
                               ARMMMUFaultInfo *fi, ARMCacheAttrs *cacheattrs)
{
    ARMCPU *cpu = env_archcpu(env);
    CPUState *cs = CPU(cpu);
    /* Read an LPAE long-descriptor translation table. */
    ARMFaultType fault_type = ARMFault_Translation;
    uint32_t level;
    ARMVAParameters param;
    uint64_t ttbr;
    hwaddr descaddr, indexmask, indexmask_grainsize;
    uint32_t tableattrs;
    target_ulong page_size;
    uint32_t attrs;
    int32_t stride;
    int addrsize, inputsize, outputsize;
    TCR *tcr = regime_tcr(env, mmu_idx);
    int ap, ns, xn, pxn;
    uint32_t el = regime_el(env, mmu_idx);
    uint64_t descaddrmask;
    bool aarch64 = arm_el_is_aa64(env, el);
    bool guarded = false;

    /* TODO: This code does not support shareability levels. */
    if (aarch64) {
        int ps;

        param = aa64_va_parameters(env, address, mmu_idx,
                                   access_type != MMU_INST_FETCH);
        level = 0;

        /*
         * If TxSZ is programmed to a value larger than the maximum,
         * or smaller than the effective minimum, it is IMPLEMENTATION
         * DEFINED whether we behave as if the field were programmed
         * within bounds, or if a level 0 Translation fault is generated.
         *
         * With FEAT_LVA, fault on less than minimum becomes required,
         * so our choice is to always raise the fault.
         */
        if (param.tsz_oob) {
            fault_type = ARMFault_Translation;
            goto do_fault;
        }

        addrsize = 64 - 8 * param.tbi;
        inputsize = 64 - param.tsz;

        /*
         * Bound PS by PARANGE to find the effective output address size.
         * ID_AA64MMFR0 is a read-only register so values outside of the
         * supported mappings can be considered an implementation error.
         */
        ps = FIELD_EX64(cpu->isar.id_aa64mmfr0, ID_AA64MMFR0, PARANGE);
        ps = MIN(ps, param.ps);
        assert(ps < ARRAY_SIZE(pamax_map));
        outputsize = pamax_map[ps];
    } else {
        param = aa32_va_parameters(env, address, mmu_idx);
        level = 1;
        addrsize = (mmu_idx == ARMMMUIdx_Stage2 ? 40 : 32);
        inputsize = addrsize - param.tsz;
        outputsize = 40;
    }

    /*
     * We determined the region when collecting the parameters, but we
     * have not yet validated that the address is valid for the region.
     * Extract the top bits and verify that they all match select.
     *
     * For aa32, if inputsize == addrsize, then we have selected the
     * region by exclusion in aa32_va_parameters and there is no more
     * validation to do here.
     */
    if (inputsize < addrsize) {
        target_ulong top_bits = sextract64(address, inputsize,
                                           addrsize - inputsize);
        if (-top_bits != param.select) {
            /* The gap between the two regions is a Translation fault */
            fault_type = ARMFault_Translation;
            goto do_fault;
        }
    }

    if (param.using64k) {
        stride = 13;
    } else if (param.using16k) {
        stride = 11;
    } else {
        stride = 9;
    }

    /*
     * Note that QEMU ignores shareability and cacheability attributes,
     * so we don't need to do anything with the SH, ORGN, IRGN fields
     * in the TTBCR.  Similarly, TTBCR:A1 selects whether we get the
     * ASID from TTBR0 or TTBR1, but QEMU's TLB doesn't currently
     * implement any ASID-like capability so we can ignore it (instead
     * we will always flush the TLB any time the ASID is changed).
     */
    ttbr = regime_ttbr(env, mmu_idx, param.select);

    /*
     * Here we should have set up all the parameters for the translation:
     * inputsize, ttbr, epd, stride, tbi
     */

    if (param.epd) {
        /*
         * Translation table walk disabled => Translation fault on TLB miss
         * Note: This is always 0 on 64-bit EL2 and EL3.
         */
        goto do_fault;
    }

    if (mmu_idx != ARMMMUIdx_Stage2 && mmu_idx != ARMMMUIdx_Stage2_S) {
        /*
         * The starting level depends on the virtual address size (which can
         * be up to 48 bits) and the translation granule size. It indicates
         * the number of strides (stride bits at a time) needed to
         * consume the bits of the input address. In the pseudocode this is:
         *  level = 4 - RoundUp((inputsize - grainsize) / stride)
         * where their 'inputsize' is our 'inputsize', 'grainsize' is
         * our 'stride + 3' and 'stride' is our 'stride'.
         * Applying the usual "rounded up m/n is (m+n-1)/n" and simplifying:
         * = 4 - (inputsize - stride - 3 + stride - 1) / stride
         * = 4 - (inputsize - 4) / stride;
         */
        level = 4 - (inputsize - 4) / stride;
    } else {
        /*
         * For stage 2 translations the starting level is specified by the
         * VTCR_EL2.SL0 field (whose interpretation depends on the page size)
         */
        uint32_t sl0 = extract32(tcr->raw_tcr, 6, 2);
        uint32_t sl2 = extract64(tcr->raw_tcr, 33, 1);
        uint32_t startlevel;
        bool ok;

        /* SL2 is RES0 unless DS=1 & 4kb granule. */
        if (param.ds && stride == 9 && sl2) {
            if (sl0 != 0) {
                level = 0;
                fault_type = ARMFault_Translation;
                goto do_fault;
            }
            startlevel = -1;
        } else if (!aarch64 || stride == 9) {
            /* AArch32 or 4KB pages */
            startlevel = 2 - sl0;

            if (cpu_isar_feature(aa64_st, cpu)) {
                startlevel &= 3;
            }
        } else {
            /* 16KB or 64KB pages */
            startlevel = 3 - sl0;
        }

        /* Check that the starting level is valid. */
        ok = check_s2_mmu_setup(cpu, aarch64, startlevel,
                                inputsize, stride, outputsize);
        if (!ok) {
            fault_type = ARMFault_Translation;
            goto do_fault;
        }
        level = startlevel;
    }

    indexmask_grainsize = MAKE_64BIT_MASK(0, stride + 3);
    indexmask = MAKE_64BIT_MASK(0, inputsize - (stride * (4 - level)));

    /* Now we can extract the actual base address from the TTBR */
    descaddr = extract64(ttbr, 0, 48);

    /*
     * For FEAT_LPA and PS=6, bits [51:48] of descaddr are in [5:2] of TTBR.
     *
     * Otherwise, if the base address is out of range, raise AddressSizeFault.
     * In the pseudocode, this is !IsZero(baseregister<47:outputsize>),
     * but we've just cleared the bits above 47, so simplify the test.
     */
    if (outputsize > 48) {
        descaddr |= extract64(ttbr, 2, 4) << 48;
    } else if (descaddr >> outputsize) {
        level = 0;
        fault_type = ARMFault_AddressSize;
        goto do_fault;
    }

    /*
     * We rely on this masking to clear the RES0 bits at the bottom of the TTBR
     * and also to mask out CnP (bit 0) which could validly be non-zero.
     */
    descaddr &= ~indexmask;

    /*
     * For AArch32, the address field in the descriptor goes up to bit 39
     * for both v7 and v8.  However, for v8 the SBZ bits [47:40] must be 0
     * or an AddressSize fault is raised.  So for v8 we extract those SBZ
     * bits as part of the address, which will be checked via outputsize.
     * For AArch64, the address field goes up to bit 47, or 49 with FEAT_LPA2;
     * the highest bits of a 52-bit output are placed elsewhere.
     */
    if (param.ds) {
        descaddrmask = MAKE_64BIT_MASK(0, 50);
    } else if (arm_feature(env, ARM_FEATURE_V8)) {
        descaddrmask = MAKE_64BIT_MASK(0, 48);
    } else {
        descaddrmask = MAKE_64BIT_MASK(0, 40);
    }
    descaddrmask &= ~indexmask_grainsize;

    /*
     * Secure accesses start with the page table in secure memory and
     * can be downgraded to non-secure at any step. Non-secure accesses
     * remain non-secure. We implement this by just ORing in the NSTable/NS
     * bits at each step.
     */
    tableattrs = regime_is_secure(env, mmu_idx) ? 0 : (1 << 4);
    for (;;) {
        uint64_t descriptor;
        bool nstable;

        descaddr |= (address >> (stride * (4 - level))) & indexmask;
        descaddr &= ~7ULL;
        nstable = extract32(tableattrs, 4, 1);
        descriptor = arm_ldq_ptw(cs, descaddr, !nstable, mmu_idx, fi);
        if (fi->type != ARMFault_None) {
            goto do_fault;
        }

        if (!(descriptor & 1) ||
            (!(descriptor & 2) && (level == 3))) {
            /* Invalid, or the Reserved level 3 encoding */
            goto do_fault;
        }

        descaddr = descriptor & descaddrmask;

        /*
         * For FEAT_LPA and PS=6, bits [51:48] of descaddr are in [15:12]
         * of descriptor.  For FEAT_LPA2 and effective DS, bits [51:50] of
         * descaddr are in [9:8].  Otherwise, if descaddr is out of range,
         * raise AddressSizeFault.
         */
        if (outputsize > 48) {
            if (param.ds) {
                descaddr |= extract64(descriptor, 8, 2) << 50;
            } else {
                descaddr |= extract64(descriptor, 12, 4) << 48;
            }
        } else if (descaddr >> outputsize) {
            fault_type = ARMFault_AddressSize;
            goto do_fault;
        }

        if ((descriptor & 2) && (level < 3)) {
            /*
             * Table entry. The top five bits are attributes which may
             * propagate down through lower levels of the table (and
             * which are all arranged so that 0 means "no effect", so
             * we can gather them up by ORing in the bits at each level).
             */
            tableattrs |= extract64(descriptor, 59, 5);
            level++;
            indexmask = indexmask_grainsize;
            continue;
        }
        /*
         * Block entry at level 1 or 2, or page entry at level 3.
         * These are basically the same thing, although the number
         * of bits we pull in from the vaddr varies. Note that although
         * descaddrmask masks enough of the low bits of the descriptor
         * to give a correct page or table address, the address field
         * in a block descriptor is smaller; so we need to explicitly
         * clear the lower bits here before ORing in the low vaddr bits.
         */
        page_size = (1ULL << ((stride * (4 - level)) + 3));
        descaddr &= ~(page_size - 1);
        descaddr |= (address & (page_size - 1));
        /* Extract attributes from the descriptor */
        attrs = extract64(descriptor, 2, 10)
            | (extract64(descriptor, 52, 12) << 10);

        if (mmu_idx == ARMMMUIdx_Stage2 || mmu_idx == ARMMMUIdx_Stage2_S) {
            /* Stage 2 table descriptors do not include any attribute fields */
            break;
        }
        /* Merge in attributes from table descriptors */
        attrs |= nstable << 3; /* NS */
        guarded = extract64(descriptor, 50, 1);  /* GP */
        if (param.hpd) {
            /* HPD disables all the table attributes except NSTable.  */
            break;
        }
        attrs |= extract32(tableattrs, 0, 2) << 11;     /* XN, PXN */
        /*
         * The sense of AP[1] vs APTable[0] is reversed, as APTable[0] == 1
         * means "force PL1 access only", which means forcing AP[1] to 0.
         */
        attrs &= ~(extract32(tableattrs, 2, 1) << 4);   /* !APT[0] => AP[1] */
        attrs |= extract32(tableattrs, 3, 1) << 5;      /* APT[1] => AP[2] */
        break;
    }
    /*
     * Here descaddr is the final physical address, and attributes
     * are all in attrs.
     */
    fault_type = ARMFault_AccessFlag;
    if ((attrs & (1 << 8)) == 0) {
        /* Access flag */
        goto do_fault;
    }

    ap = extract32(attrs, 4, 2);

    if (mmu_idx == ARMMMUIdx_Stage2 || mmu_idx == ARMMMUIdx_Stage2_S) {
        ns = mmu_idx == ARMMMUIdx_Stage2;
        xn = extract32(attrs, 11, 2);
        *prot = get_S2prot(env, ap, xn, s1_is_el0);
    } else {
        ns = extract32(attrs, 3, 1);
        xn = extract32(attrs, 12, 1);
        pxn = extract32(attrs, 11, 1);
        *prot = get_S1prot(env, mmu_idx, aarch64, ap, ns, xn, pxn);
    }

    fault_type = ARMFault_Permission;
    if (!(*prot & (1 << access_type))) {
        goto do_fault;
    }

    if (ns) {
        /*
         * The NS bit will (as required by the architecture) have no effect if
         * the CPU doesn't support TZ or this is a non-secure translation
         * regime, because the attribute will already be non-secure.
         */
        txattrs->secure = false;
    }
    /* When in aarch64 mode, and BTI is enabled, remember GP in the IOTLB.  */
    if (aarch64 && guarded && cpu_isar_feature(aa64_bti, cpu)) {
        arm_tlb_bti_gp(txattrs) = true;
    }

    if (mmu_idx == ARMMMUIdx_Stage2 || mmu_idx == ARMMMUIdx_Stage2_S) {
        cacheattrs->is_s2_format = true;
        cacheattrs->attrs = extract32(attrs, 0, 4);
    } else {
        /* Index into MAIR registers for cache attributes */
        uint8_t attrindx = extract32(attrs, 0, 3);
        uint64_t mair = env->cp15.mair_el[regime_el(env, mmu_idx)];
        assert(attrindx <= 7);
        cacheattrs->is_s2_format = false;
        cacheattrs->attrs = extract64(mair, attrindx * 8, 8);
    }

    /*
     * For FEAT_LPA2 and effective DS, the SH field in the attributes
     * was re-purposed for output address bits.  The SH attribute in
     * that case comes from TCR_ELx, which we extracted earlier.
     */
    if (param.ds) {
        cacheattrs->shareability = param.sh;
    } else {
        cacheattrs->shareability = extract32(attrs, 6, 2);
    }

    *phys_ptr = descaddr;
    *page_size_ptr = page_size;
    return false;

do_fault:
    fi->type = fault_type;
    fi->level = level;
    /* Tag the error as S2 for failed S1 PTW at S2 or ordinary S2.  */
    fi->stage2 = fi->s1ptw || (mmu_idx == ARMMMUIdx_Stage2 ||
                               mmu_idx == ARMMMUIdx_Stage2_S);
    fi->s1ns = mmu_idx == ARMMMUIdx_Stage2;
    return true;
}

static bool get_phys_addr_pmsav5(CPUARMState *env, uint32_t address,
                                 MMUAccessType access_type, ARMMMUIdx mmu_idx,
                                 hwaddr *phys_ptr, int *prot,
                                 ARMMMUFaultInfo *fi)
{
    int n;
    uint32_t mask;
    uint32_t base;
    bool is_user = regime_is_user(env, mmu_idx);

    if (regime_translation_disabled(env, mmu_idx)) {
        /* MPU disabled.  */
        *phys_ptr = address;
        *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
        return false;
    }

    *phys_ptr = address;
    for (n = 7; n >= 0; n--) {
        base = env->cp15.c6_region[n];
        if ((base & 1) == 0) {
            continue;
        }
        mask = 1 << ((base >> 1) & 0x1f);
        /* Keep this shift separate from the above to avoid an
           (undefined) << 32.  */
        mask = (mask << 1) - 1;
        if (((base ^ address) & ~mask) == 0) {
            break;
        }
    }
    if (n < 0) {
        fi->type = ARMFault_Background;
        return true;
    }

    if (access_type == MMU_INST_FETCH) {
        mask = env->cp15.pmsav5_insn_ap;
    } else {
        mask = env->cp15.pmsav5_data_ap;
    }
    mask = (mask >> (n * 4)) & 0xf;
    switch (mask) {
    case 0:
        fi->type = ARMFault_Permission;
        fi->level = 1;
        return true;
    case 1:
        if (is_user) {
            fi->type = ARMFault_Permission;
            fi->level = 1;
            return true;
        }
        *prot = PAGE_READ | PAGE_WRITE;
        break;
    case 2:
        *prot = PAGE_READ;
        if (!is_user) {
            *prot |= PAGE_WRITE;
        }
        break;
    case 3:
        *prot = PAGE_READ | PAGE_WRITE;
        break;
    case 5:
        if (is_user) {
            fi->type = ARMFault_Permission;
            fi->level = 1;
            return true;
        }
        *prot = PAGE_READ;
        break;
    case 6:
        *prot = PAGE_READ;
        break;
    default:
        /* Bad permission.  */
        fi->type = ARMFault_Permission;
        fi->level = 1;
        return true;
    }
    *prot |= PAGE_EXEC;
    return false;
}

static void get_phys_addr_pmsav7_default(CPUARMState *env, ARMMMUIdx mmu_idx,
                                         int32_t address, int *prot)
{
    if (!arm_feature(env, ARM_FEATURE_M)) {
        *prot = PAGE_READ | PAGE_WRITE;
        switch (address) {
        case 0xF0000000 ... 0xFFFFFFFF:
            if (regime_sctlr(env, mmu_idx) & SCTLR_V) {
                /* hivecs execing is ok */
                *prot |= PAGE_EXEC;
            }
            break;
        case 0x00000000 ... 0x7FFFFFFF:
            *prot |= PAGE_EXEC;
            break;
        }
    } else {
        /* Default system address map for M profile cores.
         * The architecture specifies which regions are execute-never;
         * at the MPU level no other checks are defined.
         */
        switch (address) {
        case 0x00000000 ... 0x1fffffff: /* ROM */
        case 0x20000000 ... 0x3fffffff: /* SRAM */
        case 0x60000000 ... 0x7fffffff: /* RAM */
        case 0x80000000 ... 0x9fffffff: /* RAM */
            *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
            break;
        case 0x40000000 ... 0x5fffffff: /* Peripheral */
        case 0xa0000000 ... 0xbfffffff: /* Device */
        case 0xc0000000 ... 0xdfffffff: /* Device */
        case 0xe0000000 ... 0xffffffff: /* System */
            *prot = PAGE_READ | PAGE_WRITE;
            break;
        default:
            g_assert_not_reached();
        }
    }
}

static bool m_is_ppb_region(CPUARMState *env, uint32_t address)
{
    /* True if address is in the M profile PPB region 0xe0000000 - 0xe00fffff */
    return arm_feature(env, ARM_FEATURE_M) &&
        extract32(address, 20, 12) == 0xe00;
}

static bool m_is_system_region(CPUARMState *env, uint32_t address)
{
    /*
     * True if address is in the M profile system region
     * 0xe0000000 - 0xffffffff
     */
    return arm_feature(env, ARM_FEATURE_M) && extract32(address, 29, 3) == 0x7;
}

static bool pmsav7_use_background_region(ARMCPU *cpu, ARMMMUIdx mmu_idx,
                                         bool is_user)
{
    /*
     * Return true if we should use the default memory map as a
     * "background" region if there are no hits against any MPU regions.
     */
    CPUARMState *env = &cpu->env;

    if (is_user) {
        return false;
    }

    if (arm_feature(env, ARM_FEATURE_M)) {
        return env->v7m.mpu_ctrl[regime_is_secure(env, mmu_idx)]
            & R_V7M_MPU_CTRL_PRIVDEFENA_MASK;
    } else {
        return regime_sctlr(env, mmu_idx) & SCTLR_BR;
    }
}

static bool get_phys_addr_pmsav7(CPUARMState *env, uint32_t address,
                                 MMUAccessType access_type, ARMMMUIdx mmu_idx,
                                 hwaddr *phys_ptr, int *prot,
                                 target_ulong *page_size,
                                 ARMMMUFaultInfo *fi)
{
    ARMCPU *cpu = env_archcpu(env);
    int n;
    bool is_user = regime_is_user(env, mmu_idx);

    *phys_ptr = address;
    *page_size = TARGET_PAGE_SIZE;
    *prot = 0;

    if (regime_translation_disabled(env, mmu_idx) ||
        m_is_ppb_region(env, address)) {
        /*
         * MPU disabled or M profile PPB access: use default memory map.
         * The other case which uses the default memory map in the
         * v7M ARM ARM pseudocode is exception vector reads from the vector
         * table. In QEMU those accesses are done in arm_v7m_load_vector(),
         * which always does a direct read using address_space_ldl(), rather
         * than going via this function, so we don't need to check that here.
         */
        get_phys_addr_pmsav7_default(env, mmu_idx, address, prot);
    } else { /* MPU enabled */
        for (n = (int)cpu->pmsav7_dregion - 1; n >= 0; n--) {
            /* region search */
            uint32_t base = env->pmsav7.drbar[n];
            uint32_t rsize = extract32(env->pmsav7.drsr[n], 1, 5);
            uint32_t rmask;
            bool srdis = false;

            if (!(env->pmsav7.drsr[n] & 0x1)) {
                continue;
            }

            if (!rsize) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "DRSR[%d]: Rsize field cannot be 0\n", n);
                continue;
            }
            rsize++;
            rmask = (1ull << rsize) - 1;

            if (base & rmask) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "DRBAR[%d]: 0x%" PRIx32 " misaligned "
                              "to DRSR region size, mask = 0x%" PRIx32 "\n",
                              n, base, rmask);
                continue;
            }

            if (address < base || address > base + rmask) {
                /*
                 * Address not in this region. We must check whether the
                 * region covers addresses in the same page as our address.
                 * In that case we must not report a size that covers the
                 * whole page for a subsequent hit against a different MPU
                 * region or the background region, because it would result in
                 * incorrect TLB hits for subsequent accesses to addresses that
                 * are in this MPU region.
                 */
                if (ranges_overlap(base, rmask,
                                   address & TARGET_PAGE_MASK,
                                   TARGET_PAGE_SIZE)) {
                    *page_size = 1;
                }
                continue;
            }

            /* Region matched */

            if (rsize >= 8) { /* no subregions for regions < 256 bytes */
                int i, snd;
                uint32_t srdis_mask;

                rsize -= 3; /* sub region size (power of 2) */
                snd = ((address - base) >> rsize) & 0x7;
                srdis = extract32(env->pmsav7.drsr[n], snd + 8, 1);

                srdis_mask = srdis ? 0x3 : 0x0;
                for (i = 2; i <= 8 && rsize < TARGET_PAGE_BITS; i *= 2) {
                    /*
                     * This will check in groups of 2, 4 and then 8, whether
                     * the subregion bits are consistent. rsize is incremented
                     * back up to give the region size, considering consistent
                     * adjacent subregions as one region. Stop testing if rsize
                     * is already big enough for an entire QEMU page.
                     */
                    int snd_rounded = snd & ~(i - 1);
                    uint32_t srdis_multi = extract32(env->pmsav7.drsr[n],
                                                     snd_rounded + 8, i);
                    if (srdis_mask ^ srdis_multi) {
                        break;
                    }
                    srdis_mask = (srdis_mask << i) | srdis_mask;
                    rsize++;
                }
            }
            if (srdis) {
                continue;
            }
            if (rsize < TARGET_PAGE_BITS) {
                *page_size = 1 << rsize;
            }
            break;
        }

        if (n == -1) { /* no hits */
            if (!pmsav7_use_background_region(cpu, mmu_idx, is_user)) {
                /* background fault */
                fi->type = ARMFault_Background;
                return true;
            }
            get_phys_addr_pmsav7_default(env, mmu_idx, address, prot);
        } else { /* a MPU hit! */
            uint32_t ap = extract32(env->pmsav7.dracr[n], 8, 3);
            uint32_t xn = extract32(env->pmsav7.dracr[n], 12, 1);

            if (m_is_system_region(env, address)) {
                /* System space is always execute never */
                xn = 1;
            }

            if (is_user) { /* User mode AP bit decoding */
                switch (ap) {
                case 0:
                case 1:
                case 5:
                    break; /* no access */
                case 3:
                    *prot |= PAGE_WRITE;
                    /* fall through */
                case 2:
                case 6:
                    *prot |= PAGE_READ | PAGE_EXEC;
                    break;
                case 7:
                    /* for v7M, same as 6; for R profile a reserved value */
                    if (arm_feature(env, ARM_FEATURE_M)) {
                        *prot |= PAGE_READ | PAGE_EXEC;
                        break;
                    }
                    /* fall through */
                default:
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "DRACR[%d]: Bad value for AP bits: 0x%"
                                  PRIx32 "\n", n, ap);
                }
            } else { /* Priv. mode AP bits decoding */
                switch (ap) {
                case 0:
                    break; /* no access */
                case 1:
                case 2:
                case 3:
                    *prot |= PAGE_WRITE;
                    /* fall through */
                case 5:
                case 6:
                    *prot |= PAGE_READ | PAGE_EXEC;
                    break;
                case 7:
                    /* for v7M, same as 6; for R profile a reserved value */
                    if (arm_feature(env, ARM_FEATURE_M)) {
                        *prot |= PAGE_READ | PAGE_EXEC;
                        break;
                    }
                    /* fall through */
                default:
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "DRACR[%d]: Bad value for AP bits: 0x%"
                                  PRIx32 "\n", n, ap);
                }
            }

            /* execute never */
            if (xn) {
                *prot &= ~PAGE_EXEC;
            }
        }
    }

    fi->type = ARMFault_Permission;
    fi->level = 1;
    return !(*prot & (1 << access_type));
}

bool pmsav8_mpu_lookup(CPUARMState *env, uint32_t address,
                       MMUAccessType access_type, ARMMMUIdx mmu_idx,
                       hwaddr *phys_ptr, MemTxAttrs *txattrs,
                       int *prot, bool *is_subpage,
                       ARMMMUFaultInfo *fi, uint32_t *mregion)
{
    /*
     * Perform a PMSAv8 MPU lookup (without also doing the SAU check
     * that a full phys-to-virt translation does).
     * mregion is (if not NULL) set to the region number which matched,
     * or -1 if no region number is returned (MPU off, address did not
     * hit a region, address hit in multiple regions).
     * We set is_subpage to true if the region hit doesn't cover the
     * entire TARGET_PAGE the address is within.
     */
    ARMCPU *cpu = env_archcpu(env);
    bool is_user = regime_is_user(env, mmu_idx);
    uint32_t secure = regime_is_secure(env, mmu_idx);
    int n;
    int matchregion = -1;
    bool hit = false;
    uint32_t addr_page_base = address & TARGET_PAGE_MASK;
    uint32_t addr_page_limit = addr_page_base + (TARGET_PAGE_SIZE - 1);

    *is_subpage = false;
    *phys_ptr = address;
    *prot = 0;
    if (mregion) {
        *mregion = -1;
    }

    /*
     * Unlike the ARM ARM pseudocode, we don't need to check whether this
     * was an exception vector read from the vector table (which is always
     * done using the default system address map), because those accesses
     * are done in arm_v7m_load_vector(), which always does a direct
     * read using address_space_ldl(), rather than going via this function.
     */
    if (regime_translation_disabled(env, mmu_idx)) { /* MPU disabled */
        hit = true;
    } else if (m_is_ppb_region(env, address)) {
        hit = true;
    } else {
        if (pmsav7_use_background_region(cpu, mmu_idx, is_user)) {
            hit = true;
        }

        for (n = (int)cpu->pmsav7_dregion - 1; n >= 0; n--) {
            /* region search */
            /*
             * Note that the base address is bits [31:5] from the register
             * with bits [4:0] all zeroes, but the limit address is bits
             * [31:5] from the register with bits [4:0] all ones.
             */
            uint32_t base = env->pmsav8.rbar[secure][n] & ~0x1f;
            uint32_t limit = env->pmsav8.rlar[secure][n] | 0x1f;

            if (!(env->pmsav8.rlar[secure][n] & 0x1)) {
                /* Region disabled */
                continue;
            }

            if (address < base || address > limit) {
                /*
                 * Address not in this region. We must check whether the
                 * region covers addresses in the same page as our address.
                 * In that case we must not report a size that covers the
                 * whole page for a subsequent hit against a different MPU
                 * region or the background region, because it would result in
                 * incorrect TLB hits for subsequent accesses to addresses that
                 * are in this MPU region.
                 */
                if (limit >= base &&
                    ranges_overlap(base, limit - base + 1,
                                   addr_page_base,
                                   TARGET_PAGE_SIZE)) {
                    *is_subpage = true;
                }
                continue;
            }

            if (base > addr_page_base || limit < addr_page_limit) {
                *is_subpage = true;
            }

            if (matchregion != -1) {
                /*
                 * Multiple regions match -- always a failure (unlike
                 * PMSAv7 where highest-numbered-region wins)
                 */
                fi->type = ARMFault_Permission;
                fi->level = 1;
                return true;
            }

            matchregion = n;
            hit = true;
        }
    }

    if (!hit) {
        /* background fault */
        fi->type = ARMFault_Background;
        return true;
    }

    if (matchregion == -1) {
        /* hit using the background region */
        get_phys_addr_pmsav7_default(env, mmu_idx, address, prot);
    } else {
        uint32_t ap = extract32(env->pmsav8.rbar[secure][matchregion], 1, 2);
        uint32_t xn = extract32(env->pmsav8.rbar[secure][matchregion], 0, 1);
        bool pxn = false;

        if (arm_feature(env, ARM_FEATURE_V8_1M)) {
            pxn = extract32(env->pmsav8.rlar[secure][matchregion], 4, 1);
        }

        if (m_is_system_region(env, address)) {
            /* System space is always execute never */
            xn = 1;
        }

        *prot = simple_ap_to_rw_prot(env, mmu_idx, ap);
        if (*prot && !xn && !(pxn && !is_user)) {
            *prot |= PAGE_EXEC;
        }
        /*
         * We don't need to look the attribute up in the MAIR0/MAIR1
         * registers because that only tells us about cacheability.
         */
        if (mregion) {
            *mregion = matchregion;
        }
    }

    fi->type = ARMFault_Permission;
    fi->level = 1;
    return !(*prot & (1 << access_type));
}

static bool v8m_is_sau_exempt(CPUARMState *env,
                              uint32_t address, MMUAccessType access_type)
{
    /*
     * The architecture specifies that certain address ranges are
     * exempt from v8M SAU/IDAU checks.
     */
    return
        (access_type == MMU_INST_FETCH && m_is_system_region(env, address)) ||
        (address >= 0xe0000000 && address <= 0xe0002fff) ||
        (address >= 0xe000e000 && address <= 0xe000efff) ||
        (address >= 0xe002e000 && address <= 0xe002efff) ||
        (address >= 0xe0040000 && address <= 0xe0041fff) ||
        (address >= 0xe00ff000 && address <= 0xe00fffff);
}

void v8m_security_lookup(CPUARMState *env, uint32_t address,
                                MMUAccessType access_type, ARMMMUIdx mmu_idx,
                                V8M_SAttributes *sattrs)
{
    /*
     * Look up the security attributes for this address. Compare the
     * pseudocode SecurityCheck() function.
     * We assume the caller has zero-initialized *sattrs.
     */
    ARMCPU *cpu = env_archcpu(env);
    int r;
    bool idau_exempt = false, idau_ns = true, idau_nsc = true;
    int idau_region = IREGION_NOTVALID;
    uint32_t addr_page_base = address & TARGET_PAGE_MASK;
    uint32_t addr_page_limit = addr_page_base + (TARGET_PAGE_SIZE - 1);

    if (cpu->idau) {
        IDAUInterfaceClass *iic = IDAU_INTERFACE_GET_CLASS(cpu->idau);
        IDAUInterface *ii = IDAU_INTERFACE(cpu->idau);

        iic->check(ii, address, &idau_region, &idau_exempt, &idau_ns,
                   &idau_nsc);
    }

    if (access_type == MMU_INST_FETCH && extract32(address, 28, 4) == 0xf) {
        /* 0xf0000000..0xffffffff is always S for insn fetches */
        return;
    }

    if (idau_exempt || v8m_is_sau_exempt(env, address, access_type)) {
        sattrs->ns = !regime_is_secure(env, mmu_idx);
        return;
    }

    if (idau_region != IREGION_NOTVALID) {
        sattrs->irvalid = true;
        sattrs->iregion = idau_region;
    }

    switch (env->sau.ctrl & 3) {
    case 0: /* SAU.ENABLE == 0, SAU.ALLNS == 0 */
        break;
    case 2: /* SAU.ENABLE == 0, SAU.ALLNS == 1 */
        sattrs->ns = true;
        break;
    default: /* SAU.ENABLE == 1 */
        for (r = 0; r < cpu->sau_sregion; r++) {
            if (env->sau.rlar[r] & 1) {
                uint32_t base = env->sau.rbar[r] & ~0x1f;
                uint32_t limit = env->sau.rlar[r] | 0x1f;

                if (base <= address && limit >= address) {
                    if (base > addr_page_base || limit < addr_page_limit) {
                        sattrs->subpage = true;
                    }
                    if (sattrs->srvalid) {
                        /*
                         * If we hit in more than one region then we must report
                         * as Secure, not NS-Callable, with no valid region
                         * number info.
                         */
                        sattrs->ns = false;
                        sattrs->nsc = false;
                        sattrs->sregion = 0;
                        sattrs->srvalid = false;
                        break;
                    } else {
                        if (env->sau.rlar[r] & 2) {
                            sattrs->nsc = true;
                        } else {
                            sattrs->ns = true;
                        }
                        sattrs->srvalid = true;
                        sattrs->sregion = r;
                    }
                } else {
                    /*
                     * Address not in this region. We must check whether the
                     * region covers addresses in the same page as our address.
                     * In that case we must not report a size that covers the
                     * whole page for a subsequent hit against a different MPU
                     * region or the background region, because it would result
                     * in incorrect TLB hits for subsequent accesses to
                     * addresses that are in this MPU region.
                     */
                    if (limit >= base &&
                        ranges_overlap(base, limit - base + 1,
                                       addr_page_base,
                                       TARGET_PAGE_SIZE)) {
                        sattrs->subpage = true;
                    }
                }
            }
        }
        break;
    }

    /*
     * The IDAU will override the SAU lookup results if it specifies
     * higher security than the SAU does.
     */
    if (!idau_ns) {
        if (sattrs->ns || (!idau_nsc && sattrs->nsc)) {
            sattrs->ns = false;
            sattrs->nsc = idau_nsc;
        }
    }
}

static bool get_phys_addr_pmsav8(CPUARMState *env, uint32_t address,
                                 MMUAccessType access_type, ARMMMUIdx mmu_idx,
                                 hwaddr *phys_ptr, MemTxAttrs *txattrs,
                                 int *prot, target_ulong *page_size,
                                 ARMMMUFaultInfo *fi)
{
    uint32_t secure = regime_is_secure(env, mmu_idx);
    V8M_SAttributes sattrs = {};
    bool ret;
    bool mpu_is_subpage;

    if (arm_feature(env, ARM_FEATURE_M_SECURITY)) {
        v8m_security_lookup(env, address, access_type, mmu_idx, &sattrs);
        if (access_type == MMU_INST_FETCH) {
            /*
             * Instruction fetches always use the MMU bank and the
             * transaction attribute determined by the fetch address,
             * regardless of CPU state. This is painful for QEMU
             * to handle, because it would mean we need to encode
             * into the mmu_idx not just the (user, negpri) information
             * for the current security state but also that for the
             * other security state, which would balloon the number
             * of mmu_idx values needed alarmingly.
             * Fortunately we can avoid this because it's not actually
             * possible to arbitrarily execute code from memory with
             * the wrong security attribute: it will always generate
             * an exception of some kind or another, apart from the
             * special case of an NS CPU executing an SG instruction
             * in S&NSC memory. So we always just fail the translation
             * here and sort things out in the exception handler
             * (including possibly emulating an SG instruction).
             */
            if (sattrs.ns != !secure) {
                if (sattrs.nsc) {
                    fi->type = ARMFault_QEMU_NSCExec;
                } else {
                    fi->type = ARMFault_QEMU_SFault;
                }
                *page_size = sattrs.subpage ? 1 : TARGET_PAGE_SIZE;
                *phys_ptr = address;
                *prot = 0;
                return true;
            }
        } else {
            /*
             * For data accesses we always use the MMU bank indicated
             * by the current CPU state, but the security attributes
             * might downgrade a secure access to nonsecure.
             */
            if (sattrs.ns) {
                txattrs->secure = false;
            } else if (!secure) {
                /*
                 * NS access to S memory must fault.
                 * Architecturally we should first check whether the
                 * MPU information for this address indicates that we
                 * are doing an unaligned access to Device memory, which
                 * should generate a UsageFault instead. QEMU does not
                 * currently check for that kind of unaligned access though.
                 * If we added it we would need to do so as a special case
                 * for M_FAKE_FSR_SFAULT in arm_v7m_cpu_do_interrupt().
                 */
                fi->type = ARMFault_QEMU_SFault;
                *page_size = sattrs.subpage ? 1 : TARGET_PAGE_SIZE;
                *phys_ptr = address;
                *prot = 0;
                return true;
            }
        }
    }

    ret = pmsav8_mpu_lookup(env, address, access_type, mmu_idx, phys_ptr,
                            txattrs, prot, &mpu_is_subpage, fi, NULL);
    *page_size = sattrs.subpage || mpu_is_subpage ? 1 : TARGET_PAGE_SIZE;
    return ret;
}

/*
 * Translate from the 4-bit stage 2 representation of
 * memory attributes (without cache-allocation hints) to
 * the 8-bit representation of the stage 1 MAIR registers
 * (which includes allocation hints).
 *
 * ref: shared/translation/attrs/S2AttrDecode()
 *      .../S2ConvertAttrsHints()
 */
static uint8_t convert_stage2_attrs(CPUARMState *env, uint8_t s2attrs)
{
    uint8_t hiattr = extract32(s2attrs, 2, 2);
    uint8_t loattr = extract32(s2attrs, 0, 2);
    uint8_t hihint = 0, lohint = 0;

    if (hiattr != 0) { /* normal memory */
        if (arm_hcr_el2_eff(env) & HCR_CD) { /* cache disabled */
            hiattr = loattr = 1; /* non-cacheable */
        } else {
            if (hiattr != 1) { /* Write-through or write-back */
                hihint = 3; /* RW allocate */
            }
            if (loattr != 1) { /* Write-through or write-back */
                lohint = 3; /* RW allocate */
            }
        }
    }

    return (hiattr << 6) | (hihint << 4) | (loattr << 2) | lohint;
}

/*
 * Combine either inner or outer cacheability attributes for normal
 * memory, according to table D4-42 and pseudocode procedure
 * CombineS1S2AttrHints() of ARM DDI 0487B.b (the ARMv8 ARM).
 *
 * NB: only stage 1 includes allocation hints (RW bits), leading to
 * some asymmetry.
 */
static uint8_t combine_cacheattr_nibble(uint8_t s1, uint8_t s2)
{
    if (s1 == 4 || s2 == 4) {
        /* non-cacheable has precedence */
        return 4;
    } else if (extract32(s1, 2, 2) == 0 || extract32(s1, 2, 2) == 2) {
        /* stage 1 write-through takes precedence */
        return s1;
    } else if (extract32(s2, 2, 2) == 2) {
        /* stage 2 write-through takes precedence, but the allocation hint
         * is still taken from stage 1
         */
        return (2 << 2) | extract32(s1, 0, 2);
    } else { /* write-back */
        return s1;
    }
}

/*
 * Combine the memory type and cacheability attributes of
 * s1 and s2 for the HCR_EL2.FWB == 0 case, returning the
 * combined attributes in MAIR_EL1 format.
 */
static uint8_t combined_attrs_nofwb(CPUARMState *env,
                                    ARMCacheAttrs s1, ARMCacheAttrs s2)
{
    uint8_t s1lo, s2lo, s1hi, s2hi, s2_mair_attrs, ret_attrs;

    s2_mair_attrs = convert_stage2_attrs(env, s2.attrs);

    s1lo = extract32(s1.attrs, 0, 4);
    s2lo = extract32(s2_mair_attrs, 0, 4);
    s1hi = extract32(s1.attrs, 4, 4);
    s2hi = extract32(s2_mair_attrs, 4, 4);

    /* Combine memory type and cacheability attributes */
    if (s1hi == 0 || s2hi == 0) {
        /* Device has precedence over normal */
        if (s1lo == 0 || s2lo == 0) {
            /* nGnRnE has precedence over anything */
            ret_attrs = 0;
        } else if (s1lo == 4 || s2lo == 4) {
            /* non-Reordering has precedence over Reordering */
            ret_attrs = 4;  /* nGnRE */
        } else if (s1lo == 8 || s2lo == 8) {
            /* non-Gathering has precedence over Gathering */
            ret_attrs = 8;  /* nGRE */
        } else {
            ret_attrs = 0xc; /* GRE */
        }
    } else { /* Normal memory */
        /* Outer/inner cacheability combine independently */
        ret_attrs = combine_cacheattr_nibble(s1hi, s2hi) << 4
                  | combine_cacheattr_nibble(s1lo, s2lo);
    }
    return ret_attrs;
}

static uint8_t force_cacheattr_nibble_wb(uint8_t attr)
{
    /*
     * Given the 4 bits specifying the outer or inner cacheability
     * in MAIR format, return a value specifying Normal Write-Back,
     * with the allocation and transient hints taken from the input
     * if the input specified some kind of cacheable attribute.
     */
    if (attr == 0 || attr == 4) {
        /*
         * 0 == an UNPREDICTABLE encoding
         * 4 == Non-cacheable
         * Either way, force Write-Back RW allocate non-transient
         */
        return 0xf;
    }
    /* Change WriteThrough to WriteBack, keep allocation and transient hints */
    return attr | 4;
}

/*
 * Combine the memory type and cacheability attributes of
 * s1 and s2 for the HCR_EL2.FWB == 1 case, returning the
 * combined attributes in MAIR_EL1 format.
 */
static uint8_t combined_attrs_fwb(CPUARMState *env,
                                  ARMCacheAttrs s1, ARMCacheAttrs s2)
{
    switch (s2.attrs) {
    case 7:
        /* Use stage 1 attributes */
        return s1.attrs;
    case 6:
        /*
         * Force Normal Write-Back. Note that if S1 is Normal cacheable
         * then we take the allocation hints from it; otherwise it is
         * RW allocate, non-transient.
         */
        if ((s1.attrs & 0xf0) == 0) {
            /* S1 is Device */
            return 0xff;
        }
        /* Need to check the Inner and Outer nibbles separately */
        return force_cacheattr_nibble_wb(s1.attrs & 0xf) |
            force_cacheattr_nibble_wb(s1.attrs >> 4) << 4;
    case 5:
        /* If S1 attrs are Device, use them; otherwise Normal Non-cacheable */
        if ((s1.attrs & 0xf0) == 0) {
            return s1.attrs;
        }
        return 0x44;
    case 0 ... 3:
        /* Force Device, of subtype specified by S2 */
        return s2.attrs << 2;
    default:
        /*
         * RESERVED values (including RES0 descriptor bit [5] being nonzero);
         * arbitrarily force Device.
         */
        return 0;
    }
}

/*
 * Combine S1 and S2 cacheability/shareability attributes, per D4.5.4
 * and CombineS1S2Desc()
 *
 * @env:     CPUARMState
 * @s1:      Attributes from stage 1 walk
 * @s2:      Attributes from stage 2 walk
 */
static ARMCacheAttrs combine_cacheattrs(CPUARMState *env,
                                        ARMCacheAttrs s1, ARMCacheAttrs s2)
{
    ARMCacheAttrs ret;
    bool tagged = false;

    assert(s2.is_s2_format && !s1.is_s2_format);
    ret.is_s2_format = false;

    if (s1.attrs == 0xf0) {
        tagged = true;
        s1.attrs = 0xff;
    }

    /* Combine shareability attributes (table D4-43) */
    if (s1.shareability == 2 || s2.shareability == 2) {
        /* if either are outer-shareable, the result is outer-shareable */
        ret.shareability = 2;
    } else if (s1.shareability == 3 || s2.shareability == 3) {
        /* if either are inner-shareable, the result is inner-shareable */
        ret.shareability = 3;
    } else {
        /* both non-shareable */
        ret.shareability = 0;
    }

    /* Combine memory type and cacheability attributes */
    if (arm_hcr_el2_eff(env) & HCR_FWB) {
        ret.attrs = combined_attrs_fwb(env, s1, s2);
    } else {
        ret.attrs = combined_attrs_nofwb(env, s1, s2);
    }

    /*
     * Any location for which the resultant memory type is any
     * type of Device memory is always treated as Outer Shareable.
     * Any location for which the resultant memory type is Normal
     * Inner Non-cacheable, Outer Non-cacheable is always treated
     * as Outer Shareable.
     * TODO: FEAT_XS adds another value (0x40) also meaning iNCoNC
     */
    if ((ret.attrs & 0xf0) == 0 || ret.attrs == 0x44) {
        ret.shareability = 2;
    }

    /* TODO: CombineS1S2Desc does not consider transient, only WB, RWA. */
    if (tagged && ret.attrs == 0xff) {
        ret.attrs = 0xf0;
    }

    return ret;
}

/**
 * get_phys_addr - get the physical address for this virtual address
 *
 * Find the physical address corresponding to the given virtual address,
 * by doing a translation table walk on MMU based systems or using the
 * MPU state on MPU based systems.
 *
 * Returns false if the translation was successful. Otherwise, phys_ptr, attrs,
 * prot and page_size may not be filled in, and the populated fsr value provides
 * information on why the translation aborted, in the format of a
 * DFSR/IFSR fault register, with the following caveats:
 *  * we honour the short vs long DFSR format differences.
 *  * the WnR bit is never set (the caller must do this).
 *  * for PSMAv5 based systems we don't bother to return a full FSR format
 *    value.
 *
 * @env: CPUARMState
 * @address: virtual address to get physical address for
 * @access_type: 0 for read, 1 for write, 2 for execute
 * @mmu_idx: MMU index indicating required translation regime
 * @phys_ptr: set to the physical address corresponding to the virtual address
 * @attrs: set to the memory transaction attributes to use
 * @prot: set to the permissions for the page containing phys_ptr
 * @page_size: set to the size of the page containing phys_ptr
 * @fi: set to fault info if the translation fails
 * @cacheattrs: (if non-NULL) set to the cacheability/shareability attributes
 */
bool get_phys_addr(CPUARMState *env, target_ulong address,
                   MMUAccessType access_type, ARMMMUIdx mmu_idx,
                   hwaddr *phys_ptr, MemTxAttrs *attrs, int *prot,
                   target_ulong *page_size,
                   ARMMMUFaultInfo *fi, ARMCacheAttrs *cacheattrs)
{
    ARMMMUIdx s1_mmu_idx = stage_1_mmu_idx(mmu_idx);

    if (mmu_idx != s1_mmu_idx) {
        /*
         * Call ourselves recursively to do the stage 1 and then stage 2
         * translations if mmu_idx is a two-stage regime.
         */
        if (arm_feature(env, ARM_FEATURE_EL2)) {
            hwaddr ipa;
            int s2_prot;
            int ret;
            bool ipa_secure;
            ARMCacheAttrs cacheattrs2 = {};
            ARMMMUIdx s2_mmu_idx;
            bool is_el0;

            ret = get_phys_addr(env, address, access_type, s1_mmu_idx, &ipa,
                                attrs, prot, page_size, fi, cacheattrs);

            /* If S1 fails or S2 is disabled, return early.  */
            if (ret || regime_translation_disabled(env, ARMMMUIdx_Stage2)) {
                *phys_ptr = ipa;
                return ret;
            }

            ipa_secure = attrs->secure;
            if (arm_is_secure_below_el3(env)) {
                if (ipa_secure) {
                    attrs->secure = !(env->cp15.vstcr_el2.raw_tcr & VSTCR_SW);
                } else {
                    attrs->secure = !(env->cp15.vtcr_el2.raw_tcr & VTCR_NSW);
                }
            } else {
                assert(!ipa_secure);
            }

            s2_mmu_idx = attrs->secure ? ARMMMUIdx_Stage2_S : ARMMMUIdx_Stage2;
            is_el0 = mmu_idx == ARMMMUIdx_E10_0 || mmu_idx == ARMMMUIdx_SE10_0;

            /* S1 is done. Now do S2 translation.  */
            ret = get_phys_addr_lpae(env, ipa, access_type, s2_mmu_idx, is_el0,
                                     phys_ptr, attrs, &s2_prot,
                                     page_size, fi, &cacheattrs2);
            fi->s2addr = ipa;
            /* Combine the S1 and S2 perms.  */
            *prot &= s2_prot;

            /* If S2 fails, return early.  */
            if (ret) {
                return ret;
            }

            /* Combine the S1 and S2 cache attributes. */
            if (arm_hcr_el2_eff(env) & HCR_DC) {
                /*
                 * HCR.DC forces the first stage attributes to
                 *  Normal Non-Shareable,
                 *  Inner Write-Back Read-Allocate Write-Allocate,
                 *  Outer Write-Back Read-Allocate Write-Allocate.
                 * Do not overwrite Tagged within attrs.
                 */
                if (cacheattrs->attrs != 0xf0) {
                    cacheattrs->attrs = 0xff;
                }
                cacheattrs->shareability = 0;
            }
            *cacheattrs = combine_cacheattrs(env, *cacheattrs, cacheattrs2);

            /* Check if IPA translates to secure or non-secure PA space. */
            if (arm_is_secure_below_el3(env)) {
                if (ipa_secure) {
                    attrs->secure =
                        !(env->cp15.vstcr_el2.raw_tcr & (VSTCR_SA | VSTCR_SW));
                } else {
                    attrs->secure =
                        !((env->cp15.vtcr_el2.raw_tcr & (VTCR_NSA | VTCR_NSW))
                        || (env->cp15.vstcr_el2.raw_tcr & (VSTCR_SA | VSTCR_SW)));
                }
            }
            return 0;
        } else {
            /*
             * For non-EL2 CPUs a stage1+stage2 translation is just stage 1.
             */
            mmu_idx = stage_1_mmu_idx(mmu_idx);
        }
    }

    /*
     * The page table entries may downgrade secure to non-secure, but
     * cannot upgrade an non-secure translation regime's attributes
     * to secure.
     */
    attrs->secure = regime_is_secure(env, mmu_idx);
    attrs->user = regime_is_user(env, mmu_idx);

    /*
     * Fast Context Switch Extension. This doesn't exist at all in v8.
     * In v7 and earlier it affects all stage 1 translations.
     */
    if (address < 0x02000000 && mmu_idx != ARMMMUIdx_Stage2
        && !arm_feature(env, ARM_FEATURE_V8)) {
        if (regime_el(env, mmu_idx) == 3) {
            address += env->cp15.fcseidr_s;
        } else {
            address += env->cp15.fcseidr_ns;
        }
    }

    if (arm_feature(env, ARM_FEATURE_PMSA)) {
        bool ret;
        *page_size = TARGET_PAGE_SIZE;

        if (arm_feature(env, ARM_FEATURE_V8)) {
            /* PMSAv8 */
            ret = get_phys_addr_pmsav8(env, address, access_type, mmu_idx,
                                       phys_ptr, attrs, prot, page_size, fi);
        } else if (arm_feature(env, ARM_FEATURE_V7)) {
            /* PMSAv7 */
            ret = get_phys_addr_pmsav7(env, address, access_type, mmu_idx,
                                       phys_ptr, prot, page_size, fi);
        } else {
            /* Pre-v7 MPU */
            ret = get_phys_addr_pmsav5(env, address, access_type, mmu_idx,
                                       phys_ptr, prot, fi);
        }
        qemu_log_mask(CPU_LOG_MMU, "PMSA MPU lookup for %s at 0x%08" PRIx32
                      " mmu_idx %u -> %s (prot %c%c%c)\n",
                      access_type == MMU_DATA_LOAD ? "reading" :
                      (access_type == MMU_DATA_STORE ? "writing" : "execute"),
                      (uint32_t)address, mmu_idx,
                      ret ? "Miss" : "Hit",
                      *prot & PAGE_READ ? 'r' : '-',
                      *prot & PAGE_WRITE ? 'w' : '-',
                      *prot & PAGE_EXEC ? 'x' : '-');

        return ret;
    }

    /* Definitely a real MMU, not an MPU */

    if (regime_translation_disabled(env, mmu_idx)) {
        uint64_t hcr;
        uint8_t memattr;

        /*
         * MMU disabled.  S1 addresses within aa64 translation regimes are
         * still checked for bounds -- see AArch64.TranslateAddressS1Off.
         */
        if (mmu_idx != ARMMMUIdx_Stage2 && mmu_idx != ARMMMUIdx_Stage2_S) {
            int r_el = regime_el(env, mmu_idx);
            if (arm_el_is_aa64(env, r_el)) {
                int pamax = arm_pamax(env_archcpu(env));
                uint64_t tcr = env->cp15.tcr_el[r_el].raw_tcr;
                int addrtop, tbi;

                tbi = aa64_va_parameter_tbi(tcr, mmu_idx);
                if (access_type == MMU_INST_FETCH) {
                    tbi &= ~aa64_va_parameter_tbid(tcr, mmu_idx);
                }
                tbi = (tbi >> extract64(address, 55, 1)) & 1;
                addrtop = (tbi ? 55 : 63);

                if (extract64(address, pamax, addrtop - pamax + 1) != 0) {
                    fi->type = ARMFault_AddressSize;
                    fi->level = 0;
                    fi->stage2 = false;
                    return 1;
                }

                /*
                 * When TBI is disabled, we've just validated that all of the
                 * bits above PAMax are zero, so logically we only need to
                 * clear the top byte for TBI.  But it's clearer to follow
                 * the pseudocode set of addrdesc.paddress.
                 */
                address = extract64(address, 0, 52);
            }
        }
        *phys_ptr = address;
        *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
        *page_size = TARGET_PAGE_SIZE;

        /* Fill in cacheattr a-la AArch64.TranslateAddressS1Off. */
        hcr = arm_hcr_el2_eff(env);
        cacheattrs->shareability = 0;
        cacheattrs->is_s2_format = false;
        if (hcr & HCR_DC) {
            if (hcr & HCR_DCT) {
                memattr = 0xf0;  /* Tagged, Normal, WB, RWA */
            } else {
                memattr = 0xff;  /* Normal, WB, RWA */
            }
        } else if (access_type == MMU_INST_FETCH) {
            if (regime_sctlr(env, mmu_idx) & SCTLR_I) {
                memattr = 0xee;  /* Normal, WT, RA, NT */
            } else {
                memattr = 0x44;  /* Normal, NC, No */
            }
            cacheattrs->shareability = 2; /* outer sharable */
        } else {
            memattr = 0x00;      /* Device, nGnRnE */
        }
        cacheattrs->attrs = memattr;
        return 0;
    }

    if (regime_using_lpae_format(env, mmu_idx)) {
        return get_phys_addr_lpae(env, address, access_type, mmu_idx, false,
                                  phys_ptr, attrs, prot, page_size,
                                  fi, cacheattrs);
    } else if (regime_sctlr(env, mmu_idx) & SCTLR_XP) {
        return get_phys_addr_v6(env, address, access_type, mmu_idx,
                                phys_ptr, attrs, prot, page_size, fi);
    } else {
        return get_phys_addr_v5(env, address, access_type, mmu_idx,
                                    phys_ptr, prot, page_size, fi);
    }
}
