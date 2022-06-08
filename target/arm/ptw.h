/*
 * ARM page table walking.
 *
 * This code is licensed under the GNU GPL v2 or later.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef TARGET_ARM_PTW_H
#define TARGET_ARM_PTW_H

#ifndef CONFIG_USER_ONLY

extern const uint8_t pamax_map[7];

uint32_t arm_ldl_ptw(CPUState *cs, hwaddr addr, bool is_secure,
                     ARMMMUIdx mmu_idx, ARMMMUFaultInfo *fi);
uint64_t arm_ldq_ptw(CPUState *cs, hwaddr addr, bool is_secure,
                     ARMMMUIdx mmu_idx, ARMMMUFaultInfo *fi);

bool regime_is_user(CPUARMState *env, ARMMMUIdx mmu_idx);
bool regime_translation_disabled(CPUARMState *env, ARMMMUIdx mmu_idx);
uint64_t regime_ttbr(CPUARMState *env, ARMMMUIdx mmu_idx, int ttbrn);

int ap_to_rw_prot(CPUARMState *env, ARMMMUIdx mmu_idx,
                  int ap, int domain_prot);
int simple_ap_to_rw_prot_is_user(int ap, bool is_user);

static inline int
simple_ap_to_rw_prot(CPUARMState *env, ARMMMUIdx mmu_idx, int ap)
{
    return simple_ap_to_rw_prot_is_user(ap, regime_is_user(env, mmu_idx));
}

ARMVAParameters aa32_va_parameters(CPUARMState *env, uint32_t va,
                                   ARMMMUIdx mmu_idx);
bool check_s2_mmu_setup(ARMCPU *cpu, bool is_aa64, int level,
                        int inputsize, int stride, int outputsize);
int get_S2prot(CPUARMState *env, int s2ap, int xn, bool s1_is_el0);
int get_S1prot(CPUARMState *env, ARMMMUIdx mmu_idx, bool is_aa64,
               int ap, int ns, int xn, int pxn);

bool get_phys_addr_lpae(CPUARMState *env, uint64_t address,
                        MMUAccessType access_type, ARMMMUIdx mmu_idx,
                        bool s1_is_el0,
                        hwaddr *phys_ptr, MemTxAttrs *txattrs, int *prot,
                        target_ulong *page_size_ptr,
                        ARMMMUFaultInfo *fi, ARMCacheAttrs *cacheattrs)
    __attribute__((nonnull));

#endif /* !CONFIG_USER_ONLY */
#endif /* TARGET_ARM_PTW_H */
