#if !defined (__MMU_HASH32_H__)
#define __MMU_HASH32_H__

#ifndef CONFIG_USER_ONLY

hwaddr get_pteg_offset32(CPUPPCState *env, hwaddr hash);
hwaddr ppc_hash32_get_phys_page_debug(CPUPPCState *env, target_ulong addr);
int ppc_hash32_handle_mmu_fault(CPUPPCState *env, target_ulong address, int rw,
                                int mmu_idx);

#endif /* CONFIG_USER_ONLY */

#endif /* __MMU_HASH32_H__ */
