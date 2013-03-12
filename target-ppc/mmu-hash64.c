/*
 *  PowerPC MMU, TLB, SLB and BAT emulation helpers for QEMU.
 *
 *  Copyright (c) 2003-2007 Jocelyn Mayer
 *  Copyright (c) 2013 David Gibson, IBM Corporation
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#include "cpu.h"
#include "helper.h"
#include "sysemu/kvm.h"
#include "kvm_ppc.h"
#include "mmu-hash64.h"

//#define DEBUG_MMU
//#define DEBUG_SLB

#ifdef DEBUG_MMU
#  define LOG_MMU(...) qemu_log(__VA_ARGS__)
#  define LOG_MMU_STATE(env) log_cpu_state((env), 0)
#else
#  define LOG_MMU(...) do { } while (0)
#  define LOG_MMU_STATE(...) do { } while (0)
#endif

#ifdef DEBUG_SLB
#  define LOG_SLB(...) qemu_log(__VA_ARGS__)
#else
#  define LOG_SLB(...) do { } while (0)
#endif

struct mmu_ctx_hash64 {
    hwaddr raddr;      /* Real address              */
    int prot;                      /* Protection bits           */
    hwaddr hash[2];    /* Pagetable hash values     */
    target_ulong ptem;             /* Virtual segment ID | API  */
    int key;                       /* Access key                */
};

/*
 * SLB handling
 */

static ppc_slb_t *slb_lookup(CPUPPCState *env, target_ulong eaddr)
{
    uint64_t esid_256M, esid_1T;
    int n;

    LOG_SLB("%s: eaddr " TARGET_FMT_lx "\n", __func__, eaddr);

    esid_256M = (eaddr & SEGMENT_MASK_256M) | SLB_ESID_V;
    esid_1T = (eaddr & SEGMENT_MASK_1T) | SLB_ESID_V;

    for (n = 0; n < env->slb_nr; n++) {
        ppc_slb_t *slb = &env->slb[n];

        LOG_SLB("%s: slot %d %016" PRIx64 " %016"
                    PRIx64 "\n", __func__, n, slb->esid, slb->vsid);
        /* We check for 1T matches on all MMUs here - if the MMU
         * doesn't have 1T segment support, we will have prevented 1T
         * entries from being inserted in the slbmte code. */
        if (((slb->esid == esid_256M) &&
             ((slb->vsid & SLB_VSID_B) == SLB_VSID_B_256M))
            || ((slb->esid == esid_1T) &&
                ((slb->vsid & SLB_VSID_B) == SLB_VSID_B_1T))) {
            return slb;
        }
    }

    return NULL;
}

void dump_slb(FILE *f, fprintf_function cpu_fprintf, CPUPPCState *env)
{
    int i;
    uint64_t slbe, slbv;

    cpu_synchronize_state(env);

    cpu_fprintf(f, "SLB\tESID\t\t\tVSID\n");
    for (i = 0; i < env->slb_nr; i++) {
        slbe = env->slb[i].esid;
        slbv = env->slb[i].vsid;
        if (slbe == 0 && slbv == 0) {
            continue;
        }
        cpu_fprintf(f, "%d\t0x%016" PRIx64 "\t0x%016" PRIx64 "\n",
                    i, slbe, slbv);
    }
}

void helper_slbia(CPUPPCState *env)
{
    int n, do_invalidate;

    do_invalidate = 0;
    /* XXX: Warning: slbia never invalidates the first segment */
    for (n = 1; n < env->slb_nr; n++) {
        ppc_slb_t *slb = &env->slb[n];

        if (slb->esid & SLB_ESID_V) {
            slb->esid &= ~SLB_ESID_V;
            /* XXX: given the fact that segment size is 256 MB or 1TB,
             *      and we still don't have a tlb_flush_mask(env, n, mask)
             *      in QEMU, we just invalidate all TLBs
             */
            do_invalidate = 1;
        }
    }
    if (do_invalidate) {
        tlb_flush(env, 1);
    }
}

void helper_slbie(CPUPPCState *env, target_ulong addr)
{
    ppc_slb_t *slb;

    slb = slb_lookup(env, addr);
    if (!slb) {
        return;
    }

    if (slb->esid & SLB_ESID_V) {
        slb->esid &= ~SLB_ESID_V;

        /* XXX: given the fact that segment size is 256 MB or 1TB,
         *      and we still don't have a tlb_flush_mask(env, n, mask)
         *      in QEMU, we just invalidate all TLBs
         */
        tlb_flush(env, 1);
    }
}

int ppc_store_slb(CPUPPCState *env, target_ulong rb, target_ulong rs)
{
    int slot = rb & 0xfff;
    ppc_slb_t *slb = &env->slb[slot];

    if (rb & (0x1000 - env->slb_nr)) {
        return -1; /* Reserved bits set or slot too high */
    }
    if (rs & (SLB_VSID_B & ~SLB_VSID_B_1T)) {
        return -1; /* Bad segment size */
    }
    if ((rs & SLB_VSID_B) && !(env->mmu_model & POWERPC_MMU_1TSEG)) {
        return -1; /* 1T segment on MMU that doesn't support it */
    }

    /* Mask out the slot number as we store the entry */
    slb->esid = rb & (SLB_ESID_ESID | SLB_ESID_V);
    slb->vsid = rs;

    LOG_SLB("%s: %d " TARGET_FMT_lx " - " TARGET_FMT_lx " => %016" PRIx64
            " %016" PRIx64 "\n", __func__, slot, rb, rs,
            slb->esid, slb->vsid);

    return 0;
}

static int ppc_load_slb_esid(CPUPPCState *env, target_ulong rb,
                             target_ulong *rt)
{
    int slot = rb & 0xfff;
    ppc_slb_t *slb = &env->slb[slot];

    if (slot >= env->slb_nr) {
        return -1;
    }

    *rt = slb->esid;
    return 0;
}

static int ppc_load_slb_vsid(CPUPPCState *env, target_ulong rb,
                             target_ulong *rt)
{
    int slot = rb & 0xfff;
    ppc_slb_t *slb = &env->slb[slot];

    if (slot >= env->slb_nr) {
        return -1;
    }

    *rt = slb->vsid;
    return 0;
}

void helper_store_slb(CPUPPCState *env, target_ulong rb, target_ulong rs)
{
    if (ppc_store_slb(env, rb, rs) < 0) {
        helper_raise_exception_err(env, POWERPC_EXCP_PROGRAM,
                                   POWERPC_EXCP_INVAL);
    }
}

target_ulong helper_load_slb_esid(CPUPPCState *env, target_ulong rb)
{
    target_ulong rt = 0;

    if (ppc_load_slb_esid(env, rb, &rt) < 0) {
        helper_raise_exception_err(env, POWERPC_EXCP_PROGRAM,
                                   POWERPC_EXCP_INVAL);
    }
    return rt;
}

target_ulong helper_load_slb_vsid(CPUPPCState *env, target_ulong rb)
{
    target_ulong rt = 0;

    if (ppc_load_slb_vsid(env, rb, &rt) < 0) {
        helper_raise_exception_err(env, POWERPC_EXCP_PROGRAM,
                                   POWERPC_EXCP_INVAL);
    }
    return rt;
}

/*
 * 64-bit hash table MMU handling
 */

static int ppc_hash64_pp_check(int key, int pp, bool nx)
{
    int access;

    /* Compute access rights */
    /* When pp is 4, 5 or 7, the result is undefined. Set it to noaccess */
    access = 0;
    if (key == 0) {
        switch (pp) {
        case 0x0:
        case 0x1:
        case 0x2:
            access |= PAGE_WRITE;
            /* No break here */
        case 0x3:
        case 0x6:
            access |= PAGE_READ;
            break;
        }
    } else {
        switch (pp) {
        case 0x0:
        case 0x6:
            access = 0;
            break;
        case 0x1:
        case 0x3:
            access = PAGE_READ;
            break;
        case 0x2:
            access = PAGE_READ | PAGE_WRITE;
            break;
        }
    }
    if (!nx) {
        access |= PAGE_EXEC;
    }

    return access;
}

static int ppc_hash64_check_prot(int prot, int rwx)
{
    int ret;

    if (rwx == 2) {
        if (prot & PAGE_EXEC) {
            ret = 0;
        } else {
            ret = -2;
        }
    } else if (rwx == 1) {
        if (prot & PAGE_WRITE) {
            ret = 0;
        } else {
            ret = -2;
        }
    } else {
        if (prot & PAGE_READ) {
            ret = 0;
        } else {
            ret = -2;
        }
    }

    return ret;
}

static bool pte64_match(target_ulong pte0, target_ulong pte1,
                        bool secondary, target_ulong ptem)
{
    return (pte0 & HPTE64_V_VALID)
        && (secondary == !!(pte0 & HPTE64_V_SECONDARY))
        && HPTE64_V_COMPARE(pte0, ptem);
}

static int pte64_check(struct mmu_ctx_hash64 *ctx, target_ulong pte0,
                       target_ulong pte1, int rwx)
{
    int access, ret, pp;
    bool nx;

    pp = (pte1 & HPTE64_R_PP) | ((pte1 & HPTE64_R_PP0) >> 61);
    /* No execute if either noexec or guarded bits set */
    nx = (pte1 & HPTE64_R_N) || (pte1 & HPTE64_R_G);
    /* Compute access rights */
    access = ppc_hash64_pp_check(ctx->key, pp, nx);
    /* Keep the matching PTE informations */
    ctx->raddr = pte1;
    ctx->prot = access;
    ret = ppc_hash64_check_prot(ctx->prot, rwx);
    if (ret == 0) {
        /* Access granted */
        LOG_MMU("PTE access granted !\n");
    } else {
        /* Access right violation */
        LOG_MMU("PTE access rejected\n");
    }

    return ret;
}

static int ppc_hash64_pte_update_flags(struct mmu_ctx_hash64 *ctx,
                                       target_ulong *pte1p,
                                       int ret, int rw)
{
    int store = 0;

    /* Update page flags */
    if (!(*pte1p & HPTE64_R_R)) {
        /* Update accessed flag */
        *pte1p |= HPTE64_R_R;
        store = 1;
    }
    if (!(*pte1p & HPTE64_R_C)) {
        if (rw == 1 && ret == 0) {
            /* Update changed flag */
            *pte1p |= HPTE64_R_C;
            store = 1;
        } else {
            /* Force page fault for first write access */
            ctx->prot &= ~PAGE_WRITE;
        }
    }

    return store;
}

/* PTE table lookup */
static int find_pte64(CPUPPCState *env, struct mmu_ctx_hash64 *ctx,
                      target_ulong eaddr, int h, int rwx, int target_page_bits)
{
    hwaddr pteg_off;
    target_ulong pte0, pte1;
    int i, good = -1;
    int ret;

    ret = -1; /* No entry found */
    pteg_off = (ctx->hash[h] * HASH_PTEG_SIZE_64) & env->htab_mask;
    for (i = 0; i < HPTES_PER_GROUP; i++) {
        pte0 = ppc_hash64_load_hpte0(env, pteg_off + i*HASH_PTE_SIZE_64);
        pte1 = ppc_hash64_load_hpte1(env, pteg_off + i*HASH_PTE_SIZE_64);

        LOG_MMU("Load pte from %016" HWADDR_PRIx " => " TARGET_FMT_lx " "
                TARGET_FMT_lx " %d %d %d " TARGET_FMT_lx "\n",
                pteg_off + (i * 16), pte0, pte1, !!(pte0 & HPTE64_V_VALID),
                h, !!(pte0 & HPTE64_V_SECONDARY), ctx->ptem);

        if (pte64_match(pte0, pte1, h, ctx->ptem)) {
            good = i;
            break;
        }
    }
    if (good != -1) {
        ret = pte64_check(ctx, pte0, pte1, rwx);
        LOG_MMU("found PTE at addr %08" HWADDR_PRIx " prot=%01x ret=%d\n",
                ctx->raddr, ctx->prot, ret);
        /* Update page flags */
        pte1 = ctx->raddr;
        if (ppc_hash64_pte_update_flags(ctx, &pte1, ret, rwx) == 1) {
            ppc_hash64_store_hpte1(env, pteg_off + good * HASH_PTE_SIZE_64, pte1);
        }
    }

    /* We have a TLB that saves 4K pages, so let's
     * split a huge page to 4k chunks */
    if (target_page_bits != TARGET_PAGE_BITS) {
        ctx->raddr |= (eaddr & ((1 << target_page_bits) - 1))
                      & TARGET_PAGE_MASK;
    }
    return ret;
}

static int ppc_hash64_translate(CPUPPCState *env, struct mmu_ctx_hash64 *ctx,
                                target_ulong eaddr, int rwx)
{
    hwaddr hash;
    target_ulong vsid;
    int pr, target_page_bits;
    int ret, ret2;
    ppc_slb_t *slb;
    target_ulong pageaddr;
    int segment_bits;

    /* 1. Handle real mode accesses */
    if (((rwx == 2) && (msr_ir == 0)) || ((rwx != 2) && (msr_dr == 0))) {
        /* Translation is off */
        /* In real mode the top 4 effective address bits are ignored */
        ctx->raddr = eaddr & 0x0FFFFFFFFFFFFFFFULL;
        ctx->prot = PAGE_READ | PAGE_EXEC | PAGE_WRITE;
        return 0;
    }

    /* 2. Translation is on, so look up the SLB */
    slb = slb_lookup(env, eaddr);

    if (!slb) {
        return -5;
    }

    /* 3. Check for segment level no-execute violation */
    if ((rwx == 2) && (slb->vsid & SLB_VSID_N)) {
        return -3;
    }

    pr = msr_pr;

    if (slb->vsid & SLB_VSID_B) {
        vsid = (slb->vsid & SLB_VSID_VSID) >> SLB_VSID_SHIFT_1T;
        segment_bits = 40;
    } else {
        vsid = (slb->vsid & SLB_VSID_VSID) >> SLB_VSID_SHIFT;
        segment_bits = 28;
    }

    target_page_bits = (slb->vsid & SLB_VSID_L)
        ? TARGET_PAGE_BITS_16M : TARGET_PAGE_BITS;
    ctx->key = !!(pr ? (slb->vsid & SLB_VSID_KP)
                  : (slb->vsid & SLB_VSID_KS));

    pageaddr = eaddr & ((1ULL << segment_bits)
                            - (1ULL << target_page_bits));
    if (slb->vsid & SLB_VSID_B) {
        hash = vsid ^ (vsid << 25) ^ (pageaddr >> target_page_bits);
    } else {
        hash = vsid ^ (pageaddr >> target_page_bits);
    }
    /* Only 5 bits of the page index are used in the AVPN */
    ctx->ptem = (slb->vsid & SLB_VSID_PTEM) |
        ((pageaddr >> 16) & ((1ULL << segment_bits) - 0x80));

    LOG_MMU("pte segment: key=%d nx %d vsid " TARGET_FMT_lx "\n",
            ctx->key, !!(slb->vsid & SLB_VSID_N), vsid);
    ret = -1;

    /* Page address translation */
    LOG_MMU("htab_base " TARGET_FMT_plx " htab_mask " TARGET_FMT_plx
            " hash " TARGET_FMT_plx "\n",
            env->htab_base, env->htab_mask, hash);
    ctx->hash[0] = hash;
    ctx->hash[1] = ~hash;

    LOG_MMU("0 htab=" TARGET_FMT_plx "/" TARGET_FMT_plx
            " vsid=" TARGET_FMT_lx " ptem=" TARGET_FMT_lx
            " hash=" TARGET_FMT_plx "\n",
            env->htab_base, env->htab_mask, vsid, ctx->ptem,
            ctx->hash[0]);
    /* Primary table lookup */
    ret = find_pte64(env, ctx, eaddr, 0, rwx, target_page_bits);
    if (ret == -1) {
        /* Secondary table lookup */
        LOG_MMU("1 htab=" TARGET_FMT_plx "/" TARGET_FMT_plx
                " vsid=" TARGET_FMT_lx " api=" TARGET_FMT_lx
                " hash=" TARGET_FMT_plx "\n", env->htab_base,
                env->htab_mask, vsid, ctx->ptem, ctx->hash[1]);
        ret2 = find_pte64(env, ctx, eaddr, 1, rwx, target_page_bits);
        if (ret2 != -1) {
            ret = ret2;
        }
    }

    return ret;
}

hwaddr ppc_hash64_get_phys_page_debug(CPUPPCState *env, target_ulong addr)
{
    struct mmu_ctx_hash64 ctx;

    if (unlikely(ppc_hash64_translate(env, &ctx, addr, 0) != 0)) {
        return -1;
    }

    return ctx.raddr & TARGET_PAGE_MASK;
}

int ppc_hash64_handle_mmu_fault(CPUPPCState *env, target_ulong address, int rwx,
                                int mmu_idx)
{
    struct mmu_ctx_hash64 ctx;
    int ret = 0;

    ret = ppc_hash64_translate(env, &ctx, address, rwx);
    if (ret == 0) {
        tlb_set_page(env, address & TARGET_PAGE_MASK,
                     ctx.raddr & TARGET_PAGE_MASK, ctx.prot,
                     mmu_idx, TARGET_PAGE_SIZE);
        ret = 0;
    } else if (ret < 0) {
        LOG_MMU_STATE(env);
        if (rwx == 2) {
            switch (ret) {
            case -1:
                env->exception_index = POWERPC_EXCP_ISI;
                env->error_code = 0x40000000;
                break;
            case -2:
                /* Access rights violation */
                env->exception_index = POWERPC_EXCP_ISI;
                env->error_code = 0x08000000;
                break;
            case -3:
                /* No execute protection violation */
                env->exception_index = POWERPC_EXCP_ISI;
                env->error_code = 0x10000000;
                break;
            case -5:
                /* No match in segment table */
                env->exception_index = POWERPC_EXCP_ISEG;
                env->error_code = 0;
                break;
            }
        } else {
            switch (ret) {
            case -1:
                /* No matches in page tables or TLB */
                env->exception_index = POWERPC_EXCP_DSI;
                env->error_code = 0;
                env->spr[SPR_DAR] = address;
                if (rwx == 1) {
                    env->spr[SPR_DSISR] = 0x42000000;
                } else {
                    env->spr[SPR_DSISR] = 0x40000000;
                }
                break;
            case -2:
                /* Access rights violation */
                env->exception_index = POWERPC_EXCP_DSI;
                env->error_code = 0;
                env->spr[SPR_DAR] = address;
                if (rwx == 1) {
                    env->spr[SPR_DSISR] = 0x0A000000;
                } else {
                    env->spr[SPR_DSISR] = 0x08000000;
                }
                break;
            case -5:
                /* No match in segment table */
                env->exception_index = POWERPC_EXCP_DSEG;
                env->error_code = 0;
                env->spr[SPR_DAR] = address;
                break;
            }
        }
#if 0
        printf("%s: set exception to %d %02x\n", __func__,
               env->exception, env->error_code);
#endif
        ret = 1;
    }

    return ret;
}
