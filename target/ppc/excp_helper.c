/*
 *  PowerPC exception emulation helpers for QEMU.
 *
 *  Copyright (c) 2003-2007 Jocelyn Mayer
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
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "internal.h"
#include "helper_regs.h"

#include "trace.h"

#ifdef CONFIG_TCG
#include "exec/helper-proto.h"
#include "exec/cpu_ldst.h"
#endif

/*****************************************************************************/
/* Exception processing */
#if !defined(CONFIG_USER_ONLY)

static const char *powerpc_excp_name(int excp)
{
    switch (excp) {
    case POWERPC_EXCP_CRITICAL: return "CRITICAL";
    case POWERPC_EXCP_MCHECK:   return "MCHECK";
    case POWERPC_EXCP_DSI:      return "DSI";
    case POWERPC_EXCP_ISI:      return "ISI";
    case POWERPC_EXCP_EXTERNAL: return "EXTERNAL";
    case POWERPC_EXCP_ALIGN:    return "ALIGN";
    case POWERPC_EXCP_PROGRAM:  return "PROGRAM";
    case POWERPC_EXCP_FPU:      return "FPU";
    case POWERPC_EXCP_SYSCALL:  return "SYSCALL";
    case POWERPC_EXCP_APU:      return "APU";
    case POWERPC_EXCP_DECR:     return "DECR";
    case POWERPC_EXCP_FIT:      return "FIT";
    case POWERPC_EXCP_WDT:      return "WDT";
    case POWERPC_EXCP_DTLB:     return "DTLB";
    case POWERPC_EXCP_ITLB:     return "ITLB";
    case POWERPC_EXCP_DEBUG:    return "DEBUG";
    case POWERPC_EXCP_SPEU:     return "SPEU";
    case POWERPC_EXCP_EFPDI:    return "EFPDI";
    case POWERPC_EXCP_EFPRI:    return "EFPRI";
    case POWERPC_EXCP_EPERFM:   return "EPERFM";
    case POWERPC_EXCP_DOORI:    return "DOORI";
    case POWERPC_EXCP_DOORCI:   return "DOORCI";
    case POWERPC_EXCP_GDOORI:   return "GDOORI";
    case POWERPC_EXCP_GDOORCI:  return "GDOORCI";
    case POWERPC_EXCP_HYPPRIV:  return "HYPPRIV";
    case POWERPC_EXCP_RESET:    return "RESET";
    case POWERPC_EXCP_DSEG:     return "DSEG";
    case POWERPC_EXCP_ISEG:     return "ISEG";
    case POWERPC_EXCP_HDECR:    return "HDECR";
    case POWERPC_EXCP_TRACE:    return "TRACE";
    case POWERPC_EXCP_HDSI:     return "HDSI";
    case POWERPC_EXCP_HISI:     return "HISI";
    case POWERPC_EXCP_HDSEG:    return "HDSEG";
    case POWERPC_EXCP_HISEG:    return "HISEG";
    case POWERPC_EXCP_VPU:      return "VPU";
    case POWERPC_EXCP_PIT:      return "PIT";
    case POWERPC_EXCP_EMUL:     return "EMUL";
    case POWERPC_EXCP_IFTLB:    return "IFTLB";
    case POWERPC_EXCP_DLTLB:    return "DLTLB";
    case POWERPC_EXCP_DSTLB:    return "DSTLB";
    case POWERPC_EXCP_FPA:      return "FPA";
    case POWERPC_EXCP_DABR:     return "DABR";
    case POWERPC_EXCP_IABR:     return "IABR";
    case POWERPC_EXCP_SMI:      return "SMI";
    case POWERPC_EXCP_PERFM:    return "PERFM";
    case POWERPC_EXCP_THERM:    return "THERM";
    case POWERPC_EXCP_VPUA:     return "VPUA";
    case POWERPC_EXCP_SOFTP:    return "SOFTP";
    case POWERPC_EXCP_MAINT:    return "MAINT";
    case POWERPC_EXCP_MEXTBR:   return "MEXTBR";
    case POWERPC_EXCP_NMEXTBR:  return "NMEXTBR";
    case POWERPC_EXCP_ITLBE:    return "ITLBE";
    case POWERPC_EXCP_DTLBE:    return "DTLBE";
    case POWERPC_EXCP_VSXU:     return "VSXU";
    case POWERPC_EXCP_FU:       return "FU";
    case POWERPC_EXCP_HV_EMU:   return "HV_EMU";
    case POWERPC_EXCP_HV_MAINT: return "HV_MAINT";
    case POWERPC_EXCP_HV_FU:    return "HV_FU";
    case POWERPC_EXCP_SDOOR:    return "SDOOR";
    case POWERPC_EXCP_SDOOR_HV: return "SDOOR_HV";
    case POWERPC_EXCP_HVIRT:    return "HVIRT";
    case POWERPC_EXCP_SYSCALL_VECTORED: return "SYSCALL_VECTORED";
    default:
        g_assert_not_reached();
    }
}

static void dump_syscall(CPUPPCState *env)
{
    qemu_log_mask(CPU_LOG_INT, "syscall r0=%016" PRIx64
                  " r3=%016" PRIx64 " r4=%016" PRIx64 " r5=%016" PRIx64
                  " r6=%016" PRIx64 " r7=%016" PRIx64 " r8=%016" PRIx64
                  " nip=" TARGET_FMT_lx "\n",
                  ppc_dump_gpr(env, 0), ppc_dump_gpr(env, 3),
                  ppc_dump_gpr(env, 4), ppc_dump_gpr(env, 5),
                  ppc_dump_gpr(env, 6), ppc_dump_gpr(env, 7),
                  ppc_dump_gpr(env, 8), env->nip);
}

static void dump_hcall(CPUPPCState *env)
{
    qemu_log_mask(CPU_LOG_INT, "hypercall r3=%016" PRIx64
                  " r4=%016" PRIx64 " r5=%016" PRIx64 " r6=%016" PRIx64
                  " r7=%016" PRIx64 " r8=%016" PRIx64 " r9=%016" PRIx64
                  " r10=%016" PRIx64 " r11=%016" PRIx64 " r12=%016" PRIx64
                  " nip=" TARGET_FMT_lx "\n",
                  ppc_dump_gpr(env, 3), ppc_dump_gpr(env, 4),
                  ppc_dump_gpr(env, 5), ppc_dump_gpr(env, 6),
                  ppc_dump_gpr(env, 7), ppc_dump_gpr(env, 8),
                  ppc_dump_gpr(env, 9), ppc_dump_gpr(env, 10),
                  ppc_dump_gpr(env, 11), ppc_dump_gpr(env, 12),
                  env->nip);
}

static void ppc_excp_debug_sw_tlb(CPUPPCState *env, int excp)
{
    const char *es;
    target_ulong *miss, *cmp;
    int en;

    if (!qemu_loglevel_mask(CPU_LOG_MMU)) {
        return;
    }

    if (excp == POWERPC_EXCP_IFTLB) {
        es = "I";
        en = 'I';
        miss = &env->spr[SPR_IMISS];
        cmp = &env->spr[SPR_ICMP];
    } else {
        if (excp == POWERPC_EXCP_DLTLB) {
            es = "DL";
        } else {
            es = "DS";
        }
        en = 'D';
        miss = &env->spr[SPR_DMISS];
        cmp = &env->spr[SPR_DCMP];
    }
    qemu_log("6xx %sTLB miss: %cM " TARGET_FMT_lx " %cC "
             TARGET_FMT_lx " H1 " TARGET_FMT_lx " H2 "
             TARGET_FMT_lx " %08x\n", es, en, *miss, en, *cmp,
             env->spr[SPR_HASH1], env->spr[SPR_HASH2],
             env->error_code);
}


static int powerpc_reset_wakeup(CPUState *cs, CPUPPCState *env, int excp,
                                target_ulong *msr)
{
    /* We no longer are in a PM state */
    env->resume_as_sreset = false;

    /* Pretend to be returning from doze always as we don't lose state */
    *msr |= SRR1_WS_NOLOSS;

    /* Machine checks are sent normally */
    if (excp == POWERPC_EXCP_MCHECK) {
        return excp;
    }
    switch (excp) {
    case POWERPC_EXCP_RESET:
        *msr |= SRR1_WAKERESET;
        break;
    case POWERPC_EXCP_EXTERNAL:
        *msr |= SRR1_WAKEEE;
        break;
    case POWERPC_EXCP_DECR:
        *msr |= SRR1_WAKEDEC;
        break;
    case POWERPC_EXCP_SDOOR:
        *msr |= SRR1_WAKEDBELL;
        break;
    case POWERPC_EXCP_SDOOR_HV:
        *msr |= SRR1_WAKEHDBELL;
        break;
    case POWERPC_EXCP_HV_MAINT:
        *msr |= SRR1_WAKEHMI;
        break;
    case POWERPC_EXCP_HVIRT:
        *msr |= SRR1_WAKEHVI;
        break;
    default:
        cpu_abort(cs, "Unsupported exception %d in Power Save mode\n",
                  excp);
    }
    return POWERPC_EXCP_RESET;
}

/*
 * AIL - Alternate Interrupt Location, a mode that allows interrupts to be
 * taken with the MMU on, and which uses an alternate location (e.g., so the
 * kernel/hv can map the vectors there with an effective address).
 *
 * An interrupt is considered to be taken "with AIL" or "AIL applies" if they
 * are delivered in this way. AIL requires the LPCR to be set to enable this
 * mode, and then a number of conditions have to be true for AIL to apply.
 *
 * First of all, SRESET, MCE, and HMI are always delivered without AIL, because
 * they specifically want to be in real mode (e.g., the MCE might be signaling
 * a SLB multi-hit which requires SLB flush before the MMU can be enabled).
 *
 * After that, behaviour depends on the current MSR[IR], MSR[DR], MSR[HV],
 * whether or not the interrupt changes MSR[HV] from 0 to 1, and the current
 * radix mode (LPCR[HR]).
 *
 * POWER8, POWER9 with LPCR[HR]=0
 * | LPCR[AIL] | MSR[IR||DR] | MSR[HV] | new MSR[HV] | AIL |
 * +-----------+-------------+---------+-------------+-----+
 * | a         | 00/01/10    | x       | x           | 0   |
 * | a         | 11          | 0       | 1           | 0   |
 * | a         | 11          | 1       | 1           | a   |
 * | a         | 11          | 0       | 0           | a   |
 * +-------------------------------------------------------+
 *
 * POWER9 with LPCR[HR]=1
 * | LPCR[AIL] | MSR[IR||DR] | MSR[HV] | new MSR[HV] | AIL |
 * +-----------+-------------+---------+-------------+-----+
 * | a         | 00/01/10    | x       | x           | 0   |
 * | a         | 11          | x       | x           | a   |
 * +-------------------------------------------------------+
 *
 * The difference with POWER9 being that MSR[HV] 0->1 interrupts can be sent to
 * the hypervisor in AIL mode if the guest is radix. This is good for
 * performance but allows the guest to influence the AIL of hypervisor
 * interrupts using its MSR, and also the hypervisor must disallow guest
 * interrupts (MSR[HV] 0->0) from using AIL if the hypervisor does not want to
 * use AIL for its MSR[HV] 0->1 interrupts.
 *
 * POWER10 addresses those issues with a new LPCR[HAIL] bit that is applied to
 * interrupts that begin execution with MSR[HV]=1 (so both MSR[HV] 0->1 and
 * MSR[HV] 1->1).
 *
 * HAIL=1 is equivalent to AIL=3, for interrupts delivered with MSR[HV]=1.
 *
 * POWER10 behaviour is
 * | LPCR[AIL] | LPCR[HAIL] | MSR[IR||DR] | MSR[HV] | new MSR[HV] | AIL |
 * +-----------+------------+-------------+---------+-------------+-----+
 * | a         | h          | 00/01/10    | 0       | 0           | 0   |
 * | a         | h          | 11          | 0       | 0           | a   |
 * | a         | h          | x           | 0       | 1           | h   |
 * | a         | h          | 00/01/10    | 1       | 1           | 0   |
 * | a         | h          | 11          | 1       | 1           | h   |
 * +--------------------------------------------------------------------+
 */
static void ppc_excp_apply_ail(PowerPCCPU *cpu, int excp_model, int excp,
                                      target_ulong msr,
                                      target_ulong *new_msr,
                                      target_ulong *vector)
{
#if defined(TARGET_PPC64)
    CPUPPCState *env = &cpu->env;
    bool mmu_all_on = ((msr >> MSR_IR) & 1) && ((msr >> MSR_DR) & 1);
    bool hv_escalation = !(msr & MSR_HVB) && (*new_msr & MSR_HVB);
    int ail = 0;

    if (excp == POWERPC_EXCP_MCHECK ||
        excp == POWERPC_EXCP_RESET ||
        excp == POWERPC_EXCP_HV_MAINT) {
        /* SRESET, MCE, HMI never apply AIL */
        return;
    }

    if (excp_model == POWERPC_EXCP_POWER8 ||
        excp_model == POWERPC_EXCP_POWER9) {
        if (!mmu_all_on) {
            /* AIL only works if MSR[IR] and MSR[DR] are both enabled. */
            return;
        }
        if (hv_escalation && !(env->spr[SPR_LPCR] & LPCR_HR)) {
            /*
             * AIL does not work if there is a MSR[HV] 0->1 transition and the
             * partition is in HPT mode. For radix guests, such interrupts are
             * allowed to be delivered to the hypervisor in ail mode.
             */
            return;
        }

        ail = (env->spr[SPR_LPCR] & LPCR_AIL) >> LPCR_AIL_SHIFT;
        if (ail == 0) {
            return;
        }
        if (ail == 1) {
            /* AIL=1 is reserved, treat it like AIL=0 */
            return;
        }

    } else if (excp_model == POWERPC_EXCP_POWER10) {
        if (!mmu_all_on && !hv_escalation) {
            /*
             * AIL works for HV interrupts even with guest MSR[IR/DR] disabled.
             * Guest->guest and HV->HV interrupts do require MMU on.
             */
            return;
        }

        if (*new_msr & MSR_HVB) {
            if (!(env->spr[SPR_LPCR] & LPCR_HAIL)) {
                /* HV interrupts depend on LPCR[HAIL] */
                return;
            }
            ail = 3; /* HAIL=1 gives AIL=3 behaviour for HV interrupts */
        } else {
            ail = (env->spr[SPR_LPCR] & LPCR_AIL) >> LPCR_AIL_SHIFT;
        }
        if (ail == 0) {
            return;
        }
        if (ail == 1 || ail == 2) {
            /* AIL=1 and AIL=2 are reserved, treat them like AIL=0 */
            return;
        }
    } else {
        /* Other processors do not support AIL */
        return;
    }

    /*
     * AIL applies, so the new MSR gets IR and DR set, and an offset applied
     * to the new IP.
     */
    *new_msr |= (1 << MSR_IR) | (1 << MSR_DR);

    if (excp != POWERPC_EXCP_SYSCALL_VECTORED) {
        if (ail == 2) {
            *vector |= 0x0000000000018000ull;
        } else if (ail == 3) {
            *vector |= 0xc000000000004000ull;
        }
    } else {
        /*
         * scv AIL is a little different. AIL=2 does not change the address,
         * only the MSR. AIL=3 replaces the 0x17000 base with 0xc...3000.
         */
        if (ail == 3) {
            *vector &= ~0x0000000000017000ull; /* Un-apply the base offset */
            *vector |= 0xc000000000003000ull; /* Apply scv's AIL=3 offset */
        }
    }
#endif
}

static void powerpc_set_excp_state(PowerPCCPU *cpu,
                                          target_ulong vector, target_ulong msr)
{
    CPUState *cs = CPU(cpu);
    CPUPPCState *env = &cpu->env;

    /*
     * We don't use hreg_store_msr here as already have treated any
     * special case that could occur. Just store MSR and update hflags
     *
     * Note: We *MUST* not use hreg_store_msr() as-is anyway because it
     * will prevent setting of the HV bit which some exceptions might need
     * to do.
     */
    env->msr = msr & env->msr_mask;
    hreg_compute_hflags(env);
    env->nip = vector;
    /* Reset exception state */
    cs->exception_index = POWERPC_EXCP_NONE;
    env->error_code = 0;

    /* Reset the reservation */
    env->reserve_addr = -1;

    /*
     * Any interrupt is context synchronizing, check if TCG TLB needs
     * a delayed flush on ppc64
     */
    check_tlb_flush(env, false);
}

static void powerpc_excp_40x(PowerPCCPU *cpu, int excp)
{
    CPUState *cs = CPU(cpu);
    CPUPPCState *env = &cpu->env;
    target_ulong msr, new_msr, vector;
    int srr0, srr1;

    if (excp <= POWERPC_EXCP_NONE || excp >= POWERPC_EXCP_NB) {
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
    }

    qemu_log_mask(CPU_LOG_INT, "Raise exception at " TARGET_FMT_lx
                  " => %s (%d) error=%02x\n", env->nip, powerpc_excp_name(excp),
                  excp, env->error_code);

    /* new srr1 value excluding must-be-zero bits */
    msr = env->msr & ~0x783f0000ULL;

    /*
     * new interrupt handler msr preserves existing ME unless
     * explicitly overriden.
     */
    new_msr = env->msr & (((target_ulong)1 << MSR_ME));

    /* target registers */
    srr0 = SPR_SRR0;
    srr1 = SPR_SRR1;

    /*
     * Hypervisor emulation assistance interrupt only exists on server
     * arch 2.05 server or later.
     */
    if (excp == POWERPC_EXCP_HV_EMU) {
        excp = POWERPC_EXCP_PROGRAM;
    }

    vector = env->excp_vectors[excp];
    if (vector == (target_ulong)-1ULL) {
        cpu_abort(cs, "Raised an exception without defined vector %d\n",
                  excp);
    }

    vector |= env->excp_prefix;

    switch (excp) {
    case POWERPC_EXCP_CRITICAL:    /* Critical input                         */
        srr0 = SPR_40x_SRR2;
        srr1 = SPR_40x_SRR3;
        break;
    case POWERPC_EXCP_MCHECK:    /* Machine check exception                  */
        if (msr_me == 0) {
            /*
             * Machine check exception is not enabled.  Enter
             * checkstop state.
             */
            fprintf(stderr, "Machine check while not allowed. "
                    "Entering checkstop state\n");
            if (qemu_log_separate()) {
                qemu_log("Machine check while not allowed. "
                        "Entering checkstop state\n");
            }
            cs->halted = 1;
            cpu_interrupt_exittb(cs);
        }

        /* machine check exceptions don't have ME set */
        new_msr &= ~((target_ulong)1 << MSR_ME);

        srr0 = SPR_40x_SRR2;
        srr1 = SPR_40x_SRR3;
        break;
    case POWERPC_EXCP_DSI:       /* Data storage exception                   */
        trace_ppc_excp_dsi(env->spr[SPR_40x_ESR], env->spr[SPR_40x_DEAR]);
        break;
    case POWERPC_EXCP_ISI:       /* Instruction storage exception            */
        trace_ppc_excp_isi(msr, env->nip);
        break;
    case POWERPC_EXCP_EXTERNAL:  /* External input                           */
        break;
    case POWERPC_EXCP_ALIGN:     /* Alignment exception                      */
        break;
    case POWERPC_EXCP_PROGRAM:   /* Program exception                        */
        switch (env->error_code & ~0xF) {
        case POWERPC_EXCP_FP:
            if ((msr_fe0 == 0 && msr_fe1 == 0) || msr_fp == 0) {
                trace_ppc_excp_fp_ignore();
                cs->exception_index = POWERPC_EXCP_NONE;
                env->error_code = 0;
                return;
            }
            env->spr[SPR_40x_ESR] = ESR_FP;
            break;
        case POWERPC_EXCP_INVAL:
            trace_ppc_excp_inval(env->nip);
            env->spr[SPR_40x_ESR] = ESR_PIL;
            break;
        case POWERPC_EXCP_PRIV:
            env->spr[SPR_40x_ESR] = ESR_PPR;
            break;
        case POWERPC_EXCP_TRAP:
            env->spr[SPR_40x_ESR] = ESR_PTR;
            break;
        default:
            cpu_abort(cs, "Invalid program exception %d. Aborting\n",
                      env->error_code);
            break;
        }
        break;
    case POWERPC_EXCP_SYSCALL:   /* System call exception                    */
        dump_syscall(env);

        /*
         * We need to correct the NIP which in this case is supposed
         * to point to the next instruction
         */
        env->nip += 4;
        break;
    case POWERPC_EXCP_FIT:       /* Fixed-interval timer interrupt           */
        trace_ppc_excp_print("FIT");
        break;
    case POWERPC_EXCP_WDT:       /* Watchdog timer interrupt                 */
        trace_ppc_excp_print("WDT");
        break;
    case POWERPC_EXCP_DTLB:      /* Data TLB error                           */
    case POWERPC_EXCP_ITLB:      /* Instruction TLB error                    */
        break;
    case POWERPC_EXCP_PIT:       /* Programmable interval timer interrupt    */
        trace_ppc_excp_print("PIT");
        break;
    case POWERPC_EXCP_DEBUG:     /* Debug interrupt                          */
        cpu_abort(cs, "%s exception not implemented\n",
                  powerpc_excp_name(excp));
        break;
    default:
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
        break;
    }

    /* Sanity check */
    if (!(env->msr_mask & MSR_HVB)) {
        if (new_msr & MSR_HVB) {
            cpu_abort(cs, "Trying to deliver HV exception (MSR) %d with "
                      "no HV support\n", excp);
        }
        if (srr0 == SPR_HSRR0) {
            cpu_abort(cs, "Trying to deliver HV exception (HSRR) %d with "
                      "no HV support\n", excp);
        }
    }

    /* Save PC */
    env->spr[srr0] = env->nip;

    /* Save MSR */
    env->spr[srr1] = msr;

    powerpc_set_excp_state(cpu, vector, new_msr);
}

static void powerpc_excp_6xx(PowerPCCPU *cpu, int excp)
{
    CPUState *cs = CPU(cpu);
    CPUPPCState *env = &cpu->env;
    target_ulong msr, new_msr, vector;

    if (excp <= POWERPC_EXCP_NONE || excp >= POWERPC_EXCP_NB) {
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
    }

    qemu_log_mask(CPU_LOG_INT, "Raise exception at " TARGET_FMT_lx
                  " => %s (%d) error=%02x\n", env->nip, powerpc_excp_name(excp),
                  excp, env->error_code);

    /* new srr1 value excluding must-be-zero bits */
    msr = env->msr & ~0x783f0000ULL;

    /*
     * new interrupt handler msr preserves existing ME unless
     * explicitly overriden
     */
    new_msr = env->msr & ((target_ulong)1 << MSR_ME);

    /*
     * Hypervisor emulation assistance interrupt only exists on server
     * arch 2.05 server or later.
     */
    if (excp == POWERPC_EXCP_HV_EMU) {
        excp = POWERPC_EXCP_PROGRAM;
    }

    vector = env->excp_vectors[excp];
    if (vector == (target_ulong)-1ULL) {
        cpu_abort(cs, "Raised an exception without defined vector %d\n",
                  excp);
    }

    vector |= env->excp_prefix;

    switch (excp) {
    case POWERPC_EXCP_CRITICAL:    /* Critical input                         */
        break;
    case POWERPC_EXCP_MCHECK:    /* Machine check exception                  */
        if (msr_me == 0) {
            /*
             * Machine check exception is not enabled.  Enter
             * checkstop state.
             */
            fprintf(stderr, "Machine check while not allowed. "
                    "Entering checkstop state\n");
            if (qemu_log_separate()) {
                qemu_log("Machine check while not allowed. "
                        "Entering checkstop state\n");
            }
            cs->halted = 1;
            cpu_interrupt_exittb(cs);
        }

        /* machine check exceptions don't have ME set */
        new_msr &= ~((target_ulong)1 << MSR_ME);

        break;
    case POWERPC_EXCP_DSI:       /* Data storage exception                   */
        trace_ppc_excp_dsi(env->spr[SPR_DSISR], env->spr[SPR_DAR]);
        break;
    case POWERPC_EXCP_ISI:       /* Instruction storage exception            */
        trace_ppc_excp_isi(msr, env->nip);
        msr |= env->error_code;
        break;
    case POWERPC_EXCP_EXTERNAL:  /* External input                           */
        break;
    case POWERPC_EXCP_ALIGN:     /* Alignment exception                      */
        /* Get rS/rD and rA from faulting opcode */
        /*
         * Note: the opcode fields will not be set properly for a
         * direct store load/store, but nobody cares as nobody
         * actually uses direct store segments.
         */
        env->spr[SPR_DSISR] |= (env->error_code & 0x03FF0000) >> 16;
        break;
    case POWERPC_EXCP_PROGRAM:   /* Program exception                        */
        switch (env->error_code & ~0xF) {
        case POWERPC_EXCP_FP:
            if ((msr_fe0 == 0 && msr_fe1 == 0) || msr_fp == 0) {
                trace_ppc_excp_fp_ignore();
                cs->exception_index = POWERPC_EXCP_NONE;
                env->error_code = 0;
                return;
            }

            /*
             * FP exceptions always have NIP pointing to the faulting
             * instruction, so always use store_next and claim we are
             * precise in the MSR.
             */
            msr |= 0x00100000;
            break;
        case POWERPC_EXCP_INVAL:
            trace_ppc_excp_inval(env->nip);
            msr |= 0x00080000;
            break;
        case POWERPC_EXCP_PRIV:
            msr |= 0x00040000;
            break;
        case POWERPC_EXCP_TRAP:
            msr |= 0x00020000;
            break;
        default:
            /* Should never occur */
            cpu_abort(cs, "Invalid program exception %d. Aborting\n",
                      env->error_code);
            break;
        }
        break;
    case POWERPC_EXCP_SYSCALL:   /* System call exception                    */
        dump_syscall(env);

        /*
         * We need to correct the NIP which in this case is supposed
         * to point to the next instruction
         */
        env->nip += 4;
        break;
    case POWERPC_EXCP_FPU:       /* Floating-point unavailable exception     */
    case POWERPC_EXCP_DECR:      /* Decrementer exception                    */
        break;
    case POWERPC_EXCP_DTLB:      /* Data TLB error                           */
    case POWERPC_EXCP_ITLB:      /* Instruction TLB error                    */
        break;
    case POWERPC_EXCP_RESET:     /* System reset exception                   */
        if (msr_pow) {
            cpu_abort(cs, "Trying to deliver power-saving system reset "
                      "exception %d with no HV support\n", excp);
        }
        break;
    case POWERPC_EXCP_TRACE:     /* Trace exception                          */
        break;
    case POWERPC_EXCP_IFTLB:     /* Instruction fetch TLB error              */
    case POWERPC_EXCP_DLTLB:     /* Data load TLB miss                       */
    case POWERPC_EXCP_DSTLB:     /* Data store TLB miss                      */
        /* Swap temporary saved registers with GPRs */
        if (!(new_msr & ((target_ulong)1 << MSR_TGPR))) {
            new_msr |= (target_ulong)1 << MSR_TGPR;
            hreg_swap_gpr_tgpr(env);
        }

        ppc_excp_debug_sw_tlb(env, excp);

        msr |= env->crf[0] << 28;
        msr |= env->error_code; /* key, D/I, S/L bits */
        /* Set way using a LRU mechanism */
        msr |= ((env->last_way + 1) & (env->nb_ways - 1)) << 17;
        break;
    case POWERPC_EXCP_FPA:       /* Floating-point assist exception          */
    case POWERPC_EXCP_DABR:      /* Data address breakpoint                  */
    case POWERPC_EXCP_IABR:      /* Instruction address breakpoint           */
    case POWERPC_EXCP_SMI:       /* System management interrupt              */
    case POWERPC_EXCP_MEXTBR:    /* Maskable external breakpoint             */
    case POWERPC_EXCP_NMEXTBR:   /* Non maskable external breakpoint         */
        cpu_abort(cs, "%s exception not implemented\n",
                  powerpc_excp_name(excp));
        break;
    default:
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
        break;
    }

    /* Sanity check */
    if (!(env->msr_mask & MSR_HVB)) {
        if (new_msr & MSR_HVB) {
            cpu_abort(cs, "Trying to deliver HV exception (MSR) %d with "
                      "no HV support\n", excp);
        }
    }

    /*
     * Sort out endianness of interrupt, this differs depending on the
     * CPU, the HV mode, etc...
     */
    if (ppc_interrupts_little_endian(cpu, !!(new_msr & MSR_HVB))) {
        new_msr |= (target_ulong)1 << MSR_LE;
    }

    /* Save PC */
    env->spr[SPR_SRR0] = env->nip;

    /* Save MSR */
    env->spr[SPR_SRR1] = msr;

    powerpc_set_excp_state(cpu, vector, new_msr);
}

static void powerpc_excp_7xx(PowerPCCPU *cpu, int excp)
{
    CPUState *cs = CPU(cpu);
    CPUPPCState *env = &cpu->env;
    int excp_model = env->excp_model;
    target_ulong msr, new_msr, vector;
    int srr0, srr1, lev = -1;

    if (excp <= POWERPC_EXCP_NONE || excp >= POWERPC_EXCP_NB) {
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
    }

    qemu_log_mask(CPU_LOG_INT, "Raise exception at " TARGET_FMT_lx
                  " => %s (%d) error=%02x\n", env->nip, powerpc_excp_name(excp),
                  excp, env->error_code);

    /* new srr1 value excluding must-be-zero bits */
    if (excp_model == POWERPC_EXCP_BOOKE) {
        msr = env->msr;
    } else {
        msr = env->msr & ~0x783f0000ULL;
    }

    /*
     * new interrupt handler msr preserves existing HV and ME unless
     * explicitly overriden
     */
    new_msr = env->msr & (((target_ulong)1 << MSR_ME) | MSR_HVB);

    /* target registers */
    srr0 = SPR_SRR0;
    srr1 = SPR_SRR1;

    /*
     * check for special resume at 0x100 from doze/nap/sleep/winkle on
     * P7/P8/P9
     */
    if (env->resume_as_sreset) {
        excp = powerpc_reset_wakeup(cs, env, excp, &msr);
    }

    /*
     * Hypervisor emulation assistance interrupt only exists on server
     * arch 2.05 server or later. We also don't want to generate it if
     * we don't have HVB in msr_mask (PAPR mode).
     */
    if (excp == POWERPC_EXCP_HV_EMU
#if defined(TARGET_PPC64)
        && !(mmu_is_64bit(env->mmu_model) && (env->msr_mask & MSR_HVB))
#endif /* defined(TARGET_PPC64) */

    ) {
        excp = POWERPC_EXCP_PROGRAM;
    }

#ifdef TARGET_PPC64
    /*
     * SPEU and VPU share the same IVOR but they exist in different
     * processors. SPEU is e500v1/2 only and VPU is e6500 only.
     */
    if (excp_model == POWERPC_EXCP_BOOKE && excp == POWERPC_EXCP_VPU) {
        excp = POWERPC_EXCP_SPEU;
    }
#endif

    vector = env->excp_vectors[excp];
    if (vector == (target_ulong)-1ULL) {
        cpu_abort(cs, "Raised an exception without defined vector %d\n",
                  excp);
    }

    vector |= env->excp_prefix;

    switch (excp) {
    case POWERPC_EXCP_CRITICAL:    /* Critical input                         */
        switch (excp_model) {
        case POWERPC_EXCP_40x:
            srr0 = SPR_40x_SRR2;
            srr1 = SPR_40x_SRR3;
            break;
        case POWERPC_EXCP_BOOKE:
            srr0 = SPR_BOOKE_CSRR0;
            srr1 = SPR_BOOKE_CSRR1;
            break;
        case POWERPC_EXCP_6xx:
            break;
        default:
            goto excp_invalid;
        }
        break;
    case POWERPC_EXCP_MCHECK:    /* Machine check exception                  */
        if (msr_me == 0) {
            /*
             * Machine check exception is not enabled.  Enter
             * checkstop state.
             */
            fprintf(stderr, "Machine check while not allowed. "
                    "Entering checkstop state\n");
            if (qemu_log_separate()) {
                qemu_log("Machine check while not allowed. "
                        "Entering checkstop state\n");
            }
            cs->halted = 1;
            cpu_interrupt_exittb(cs);
        }
        if (env->msr_mask & MSR_HVB) {
            /*
             * ISA specifies HV, but can be delivered to guest with HV
             * clear (e.g., see FWNMI in PAPR).
             */
            new_msr |= (target_ulong)MSR_HVB;
        }

        /* machine check exceptions don't have ME set */
        new_msr &= ~((target_ulong)1 << MSR_ME);

        /* XXX: should also have something loaded in DAR / DSISR */
        switch (excp_model) {
        case POWERPC_EXCP_40x:
            srr0 = SPR_40x_SRR2;
            srr1 = SPR_40x_SRR3;
            break;
        case POWERPC_EXCP_BOOKE:
            /* FIXME: choose one or the other based on CPU type */
            srr0 = SPR_BOOKE_MCSRR0;
            srr1 = SPR_BOOKE_MCSRR1;

            env->spr[SPR_BOOKE_CSRR0] = env->nip;
            env->spr[SPR_BOOKE_CSRR1] = msr;
            break;
        default:
            break;
        }
        break;
    case POWERPC_EXCP_DSI:       /* Data storage exception                   */
        trace_ppc_excp_dsi(env->spr[SPR_DSISR], env->spr[SPR_DAR]);
        break;
    case POWERPC_EXCP_ISI:       /* Instruction storage exception            */
        trace_ppc_excp_isi(msr, env->nip);
        msr |= env->error_code;
        break;
    case POWERPC_EXCP_EXTERNAL:  /* External input                           */
    {
        bool lpes0;

        cs = CPU(cpu);

        /*
         * Exception targeting modifiers
         *
         * LPES0 is supported on POWER7/8/9
         * LPES1 is not supported (old iSeries mode)
         *
         * On anything else, we behave as if LPES0 is 1
         * (externals don't alter MSR:HV)
         */
#if defined(TARGET_PPC64)
        if (excp_model == POWERPC_EXCP_POWER7 ||
            excp_model == POWERPC_EXCP_POWER8 ||
            excp_model == POWERPC_EXCP_POWER9 ||
            excp_model == POWERPC_EXCP_POWER10) {
            lpes0 = !!(env->spr[SPR_LPCR] & LPCR_LPES0);
        } else
#endif /* defined(TARGET_PPC64) */
        {
            lpes0 = true;
        }

        if (!lpes0) {
            new_msr |= (target_ulong)MSR_HVB;
            new_msr |= env->msr & ((target_ulong)1 << MSR_RI);
            srr0 = SPR_HSRR0;
            srr1 = SPR_HSRR1;
        }
        if (env->mpic_proxy) {
            /* IACK the IRQ on delivery */
            env->spr[SPR_BOOKE_EPR] = ldl_phys(cs->as, env->mpic_iack);
        }
        break;
    }
    case POWERPC_EXCP_ALIGN:     /* Alignment exception                      */
        /* Get rS/rD and rA from faulting opcode */
        /*
         * Note: the opcode fields will not be set properly for a
         * direct store load/store, but nobody cares as nobody
         * actually uses direct store segments.
         */
        env->spr[SPR_DSISR] |= (env->error_code & 0x03FF0000) >> 16;
        break;
    case POWERPC_EXCP_PROGRAM:   /* Program exception                        */
        switch (env->error_code & ~0xF) {
        case POWERPC_EXCP_FP:
            if ((msr_fe0 == 0 && msr_fe1 == 0) || msr_fp == 0) {
                trace_ppc_excp_fp_ignore();
                cs->exception_index = POWERPC_EXCP_NONE;
                env->error_code = 0;
                return;
            }

            /*
             * FP exceptions always have NIP pointing to the faulting
             * instruction, so always use store_next and claim we are
             * precise in the MSR.
             */
            msr |= 0x00100000;
            env->spr[SPR_BOOKE_ESR] = ESR_FP;
            break;
        case POWERPC_EXCP_INVAL:
            trace_ppc_excp_inval(env->nip);
            msr |= 0x00080000;
            env->spr[SPR_BOOKE_ESR] = ESR_PIL;
            break;
        case POWERPC_EXCP_PRIV:
            msr |= 0x00040000;
            env->spr[SPR_BOOKE_ESR] = ESR_PPR;
            break;
        case POWERPC_EXCP_TRAP:
            msr |= 0x00020000;
            env->spr[SPR_BOOKE_ESR] = ESR_PTR;
            break;
        default:
            /* Should never occur */
            cpu_abort(cs, "Invalid program exception %d. Aborting\n",
                      env->error_code);
            break;
        }
        break;
    case POWERPC_EXCP_SYSCALL:   /* System call exception                    */
        lev = env->error_code;

        if ((lev == 1) && cpu->vhyp) {
            dump_hcall(env);
        } else {
            dump_syscall(env);
        }

        /*
         * We need to correct the NIP which in this case is supposed
         * to point to the next instruction
         */
        env->nip += 4;

        /* "PAPR mode" built-in hypercall emulation */
        if ((lev == 1) && cpu->vhyp) {
            PPCVirtualHypervisorClass *vhc =
                PPC_VIRTUAL_HYPERVISOR_GET_CLASS(cpu->vhyp);
            vhc->hypercall(cpu->vhyp, cpu);
            return;
        }
        if (lev == 1) {
            new_msr |= (target_ulong)MSR_HVB;
        }
        break;
    case POWERPC_EXCP_SYSCALL_VECTORED: /* scv exception                     */
        lev = env->error_code;
        dump_syscall(env);
        env->nip += 4;
        new_msr |= env->msr & ((target_ulong)1 << MSR_EE);
        new_msr |= env->msr & ((target_ulong)1 << MSR_RI);

        vector += lev * 0x20;

        env->lr = env->nip;
        env->ctr = msr;
        break;
    case POWERPC_EXCP_FPU:       /* Floating-point unavailable exception     */
    case POWERPC_EXCP_APU:       /* Auxiliary processor unavailable          */
    case POWERPC_EXCP_DECR:      /* Decrementer exception                    */
        break;
    case POWERPC_EXCP_FIT:       /* Fixed-interval timer interrupt           */
        /* FIT on 4xx */
        trace_ppc_excp_print("FIT");
        break;
    case POWERPC_EXCP_WDT:       /* Watchdog timer interrupt                 */
        trace_ppc_excp_print("WDT");
        switch (excp_model) {
        case POWERPC_EXCP_BOOKE:
            srr0 = SPR_BOOKE_CSRR0;
            srr1 = SPR_BOOKE_CSRR1;
            break;
        default:
            break;
        }
        break;
    case POWERPC_EXCP_DTLB:      /* Data TLB error                           */
    case POWERPC_EXCP_ITLB:      /* Instruction TLB error                    */
        break;
    case POWERPC_EXCP_DEBUG:     /* Debug interrupt                          */
        if (env->flags & POWERPC_FLAG_DE) {
            /* FIXME: choose one or the other based on CPU type */
            srr0 = SPR_BOOKE_DSRR0;
            srr1 = SPR_BOOKE_DSRR1;

            env->spr[SPR_BOOKE_CSRR0] = env->nip;
            env->spr[SPR_BOOKE_CSRR1] = msr;

            /* DBSR already modified by caller */
        } else {
            cpu_abort(cs, "Debug exception triggered on unsupported model\n");
        }
        break;
    case POWERPC_EXCP_SPEU:   /* SPE/embedded floating-point unavailable/VPU  */
        env->spr[SPR_BOOKE_ESR] = ESR_SPV;
        break;
    case POWERPC_EXCP_DOORI:     /* Embedded doorbell interrupt              */
        break;
    case POWERPC_EXCP_DOORCI:    /* Embedded doorbell critical interrupt     */
        srr0 = SPR_BOOKE_CSRR0;
        srr1 = SPR_BOOKE_CSRR1;
        break;
    case POWERPC_EXCP_RESET:     /* System reset exception                   */
        /* A power-saving exception sets ME, otherwise it is unchanged */
        if (msr_pow) {
            /* indicate that we resumed from power save mode */
            msr |= 0x10000;
            new_msr |= ((target_ulong)1 << MSR_ME);
        }
        if (env->msr_mask & MSR_HVB) {
            /*
             * ISA specifies HV, but can be delivered to guest with HV
             * clear (e.g., see FWNMI in PAPR, NMI injection in QEMU).
             */
            new_msr |= (target_ulong)MSR_HVB;
        } else {
            if (msr_pow) {
                cpu_abort(cs, "Trying to deliver power-saving system reset "
                          "exception %d with no HV support\n", excp);
            }
        }
        break;
    case POWERPC_EXCP_DSEG:      /* Data segment exception                   */
    case POWERPC_EXCP_ISEG:      /* Instruction segment exception            */
    case POWERPC_EXCP_TRACE:     /* Trace exception                          */
        break;
    case POWERPC_EXCP_HISI:      /* Hypervisor instruction storage exception */
        msr |= env->error_code;
        /* fall through */
    case POWERPC_EXCP_HDECR:     /* Hypervisor decrementer exception         */
    case POWERPC_EXCP_HDSI:      /* Hypervisor data storage exception        */
    case POWERPC_EXCP_HDSEG:     /* Hypervisor data segment exception        */
    case POWERPC_EXCP_HISEG:     /* Hypervisor instruction segment exception */
    case POWERPC_EXCP_SDOOR_HV:  /* Hypervisor Doorbell interrupt            */
    case POWERPC_EXCP_HV_EMU:
    case POWERPC_EXCP_HVIRT:     /* Hypervisor virtualization                */
        srr0 = SPR_HSRR0;
        srr1 = SPR_HSRR1;
        new_msr |= (target_ulong)MSR_HVB;
        new_msr |= env->msr & ((target_ulong)1 << MSR_RI);
        break;
    case POWERPC_EXCP_VPU:       /* Vector unavailable exception             */
    case POWERPC_EXCP_VSXU:       /* VSX unavailable exception               */
    case POWERPC_EXCP_FU:         /* Facility unavailable exception          */
#ifdef TARGET_PPC64
        env->spr[SPR_FSCR] |= ((target_ulong)env->error_code << 56);
#endif
        break;
    case POWERPC_EXCP_HV_FU:     /* Hypervisor Facility Unavailable Exception */
#ifdef TARGET_PPC64
        env->spr[SPR_HFSCR] |= ((target_ulong)env->error_code << FSCR_IC_POS);
        srr0 = SPR_HSRR0;
        srr1 = SPR_HSRR1;
        new_msr |= (target_ulong)MSR_HVB;
        new_msr |= env->msr & ((target_ulong)1 << MSR_RI);
#endif
        break;
    case POWERPC_EXCP_PIT:       /* Programmable interval timer interrupt    */
        trace_ppc_excp_print("PIT");
        break;
    case POWERPC_EXCP_IFTLB:     /* Instruction fetch TLB error              */
    case POWERPC_EXCP_DLTLB:     /* Data load TLB miss                       */
    case POWERPC_EXCP_DSTLB:     /* Data store TLB miss                      */
        switch (excp_model) {
        case POWERPC_EXCP_6xx:
            /* Swap temporary saved registers with GPRs */
            if (!(new_msr & ((target_ulong)1 << MSR_TGPR))) {
                new_msr |= (target_ulong)1 << MSR_TGPR;
                hreg_swap_gpr_tgpr(env);
            }
            /* fall through */
        case POWERPC_EXCP_7xx:
            ppc_excp_debug_sw_tlb(env, excp);

            msr |= env->crf[0] << 28;
            msr |= env->error_code; /* key, D/I, S/L bits */
            /* Set way using a LRU mechanism */
            msr |= ((env->last_way + 1) & (env->nb_ways - 1)) << 17;
            break;
        default:
            cpu_abort(cs, "Invalid TLB miss exception\n");
            break;
        }
        break;
    case POWERPC_EXCP_EFPDI:     /* Embedded floating-point data interrupt   */
    case POWERPC_EXCP_EFPRI:     /* Embedded floating-point round interrupt  */
    case POWERPC_EXCP_EPERFM:    /* Embedded performance monitor interrupt   */
    case POWERPC_EXCP_FPA:       /* Floating-point assist exception          */
    case POWERPC_EXCP_DABR:      /* Data address breakpoint                  */
    case POWERPC_EXCP_IABR:      /* Instruction address breakpoint           */
    case POWERPC_EXCP_SMI:       /* System management interrupt              */
    case POWERPC_EXCP_THERM:     /* Thermal interrupt                        */
    case POWERPC_EXCP_PERFM:     /* Embedded performance monitor interrupt   */
    case POWERPC_EXCP_VPUA:      /* Vector assist exception                  */
    case POWERPC_EXCP_SOFTP:     /* Soft patch exception                     */
    case POWERPC_EXCP_MAINT:     /* Maintenance exception                    */
    case POWERPC_EXCP_MEXTBR:    /* Maskable external breakpoint             */
    case POWERPC_EXCP_NMEXTBR:   /* Non maskable external breakpoint         */
        cpu_abort(cs, "%s exception not implemented\n",
                  powerpc_excp_name(excp));
        break;
    default:
    excp_invalid:
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
        break;
    }

    /* Sanity check */
    if (!(env->msr_mask & MSR_HVB)) {
        if (new_msr & MSR_HVB) {
            cpu_abort(cs, "Trying to deliver HV exception (MSR) %d with "
                      "no HV support\n", excp);
        }
        if (srr0 == SPR_HSRR0) {
            cpu_abort(cs, "Trying to deliver HV exception (HSRR) %d with "
                      "no HV support\n", excp);
        }
    }

    /*
     * Sort out endianness of interrupt, this differs depending on the
     * CPU, the HV mode, etc...
     */
    if (ppc_interrupts_little_endian(cpu, !!(new_msr & MSR_HVB))) {
        new_msr |= (target_ulong)1 << MSR_LE;
    }

#if defined(TARGET_PPC64)
    if (excp_model == POWERPC_EXCP_BOOKE) {
        if (env->spr[SPR_BOOKE_EPCR] & EPCR_ICM) {
            /* Cat.64-bit: EPCR.ICM is copied to MSR.CM */
            new_msr |= (target_ulong)1 << MSR_CM;
        } else {
            vector = (uint32_t)vector;
        }
    } else {
        if (!msr_isf && !mmu_is_64bit(env->mmu_model)) {
            vector = (uint32_t)vector;
        } else {
            new_msr |= (target_ulong)1 << MSR_SF;
        }
    }
#endif

    if (excp != POWERPC_EXCP_SYSCALL_VECTORED) {
        /* Save PC */
        env->spr[srr0] = env->nip;

        /* Save MSR */
        env->spr[srr1] = msr;
    }

    /* This can update new_msr and vector if AIL applies */
    ppc_excp_apply_ail(cpu, excp_model, excp, msr, &new_msr, &vector);

    powerpc_set_excp_state(cpu, vector, new_msr);
}

static void powerpc_excp_74xx(PowerPCCPU *cpu, int excp)
{
    CPUState *cs = CPU(cpu);
    CPUPPCState *env = &cpu->env;
    target_ulong msr, new_msr, vector;

    if (excp <= POWERPC_EXCP_NONE || excp >= POWERPC_EXCP_NB) {
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
    }

    qemu_log_mask(CPU_LOG_INT, "Raise exception at " TARGET_FMT_lx
                  " => %s (%d) error=%02x\n", env->nip, powerpc_excp_name(excp),
                  excp, env->error_code);

    /* new srr1 value excluding must-be-zero bits */
    msr = env->msr & ~0x783f0000ULL;

    /*
     * new interrupt handler msr preserves existing ME unless
     * explicitly overriden
     */
    new_msr = env->msr & ((target_ulong)1 << MSR_ME);

    /*
     * Hypervisor emulation assistance interrupt only exists on server
     * arch 2.05 server or later.
     */
    if (excp == POWERPC_EXCP_HV_EMU) {
        excp = POWERPC_EXCP_PROGRAM;
    }

    vector = env->excp_vectors[excp];
    if (vector == (target_ulong)-1ULL) {
        cpu_abort(cs, "Raised an exception without defined vector %d\n",
                  excp);
    }

    vector |= env->excp_prefix;

    switch (excp) {
    case POWERPC_EXCP_MCHECK:    /* Machine check exception                  */
        if (msr_me == 0) {
            /*
             * Machine check exception is not enabled.  Enter
             * checkstop state.
             */
            fprintf(stderr, "Machine check while not allowed. "
                    "Entering checkstop state\n");
            if (qemu_log_separate()) {
                qemu_log("Machine check while not allowed. "
                        "Entering checkstop state\n");
            }
            cs->halted = 1;
            cpu_interrupt_exittb(cs);
        }

        /* machine check exceptions don't have ME set */
        new_msr &= ~((target_ulong)1 << MSR_ME);

        break;
    case POWERPC_EXCP_DSI:       /* Data storage exception                   */
        trace_ppc_excp_dsi(env->spr[SPR_DSISR], env->spr[SPR_DAR]);
        break;
    case POWERPC_EXCP_ISI:       /* Instruction storage exception            */
        trace_ppc_excp_isi(msr, env->nip);
        msr |= env->error_code;
        break;
    case POWERPC_EXCP_EXTERNAL:  /* External input                           */
        break;
    case POWERPC_EXCP_ALIGN:     /* Alignment exception                      */
        /* Get rS/rD and rA from faulting opcode */
        /*
         * Note: the opcode fields will not be set properly for a
         * direct store load/store, but nobody cares as nobody
         * actually uses direct store segments.
         */
        env->spr[SPR_DSISR] |= (env->error_code & 0x03FF0000) >> 16;
        break;
    case POWERPC_EXCP_PROGRAM:   /* Program exception                        */
        switch (env->error_code & ~0xF) {
        case POWERPC_EXCP_FP:
            if ((msr_fe0 == 0 && msr_fe1 == 0) || msr_fp == 0) {
                trace_ppc_excp_fp_ignore();
                cs->exception_index = POWERPC_EXCP_NONE;
                env->error_code = 0;
                return;
            }

            /*
             * FP exceptions always have NIP pointing to the faulting
             * instruction, so always use store_next and claim we are
             * precise in the MSR.
             */
            msr |= 0x00100000;
            break;
        case POWERPC_EXCP_INVAL:
            trace_ppc_excp_inval(env->nip);
            msr |= 0x00080000;
            break;
        case POWERPC_EXCP_PRIV:
            msr |= 0x00040000;
            break;
        case POWERPC_EXCP_TRAP:
            msr |= 0x00020000;
            break;
        default:
            /* Should never occur */
            cpu_abort(cs, "Invalid program exception %d. Aborting\n",
                      env->error_code);
            break;
        }
        break;
    case POWERPC_EXCP_SYSCALL:   /* System call exception                    */
    {
        int lev = env->error_code;

        if ((lev == 1) && cpu->vhyp) {
            dump_hcall(env);
        } else {
            dump_syscall(env);
        }

        /*
         * We need to correct the NIP which in this case is supposed
         * to point to the next instruction
         */
        env->nip += 4;

        /*
         * The Virtual Open Firmware (VOF) relies on the 'sc 1'
         * instruction to communicate with QEMU. The pegasos2 machine
         * uses VOF and the 74xx CPUs, so although the 74xx don't have
         * HV mode, we need to keep hypercall support.
         */
        if ((lev == 1) && cpu->vhyp) {
            PPCVirtualHypervisorClass *vhc =
                PPC_VIRTUAL_HYPERVISOR_GET_CLASS(cpu->vhyp);
            vhc->hypercall(cpu->vhyp, cpu);
            return;
        }

        break;
    }
    case POWERPC_EXCP_FPU:       /* Floating-point unavailable exception     */
    case POWERPC_EXCP_DECR:      /* Decrementer exception                    */
        break;
    case POWERPC_EXCP_RESET:     /* System reset exception                   */
        if (msr_pow) {
            cpu_abort(cs, "Trying to deliver power-saving system reset "
                      "exception %d with no HV support\n", excp);
        }
        break;
    case POWERPC_EXCP_TRACE:     /* Trace exception                          */
        break;
    case POWERPC_EXCP_VPU:       /* Vector unavailable exception             */
        break;
    case POWERPC_EXCP_IABR:      /* Instruction address breakpoint           */
    case POWERPC_EXCP_SMI:       /* System management interrupt              */
    case POWERPC_EXCP_THERM:     /* Thermal interrupt                        */
    case POWERPC_EXCP_PERFM:     /* Embedded performance monitor interrupt   */
    case POWERPC_EXCP_VPUA:      /* Vector assist exception                  */
        cpu_abort(cs, "%s exception not implemented\n",
                  powerpc_excp_name(excp));
        break;
    default:
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
        break;
    }

    /* Sanity check */
    if (!(env->msr_mask & MSR_HVB)) {
        if (new_msr & MSR_HVB) {
            cpu_abort(cs, "Trying to deliver HV exception (MSR) %d with "
                      "no HV support\n", excp);
        }
    }

    /*
     * Sort out endianness of interrupt, this differs depending on the
     * CPU, the HV mode, etc...
     */
    if (ppc_interrupts_little_endian(cpu, !!(new_msr & MSR_HVB))) {
        new_msr |= (target_ulong)1 << MSR_LE;
    }

    /* Save PC */
    env->spr[SPR_SRR0] = env->nip;

    /* Save MSR */
    env->spr[SPR_SRR1] = msr;

    powerpc_set_excp_state(cpu, vector, new_msr);
}

static void powerpc_excp_booke(PowerPCCPU *cpu, int excp)
{
    CPUState *cs = CPU(cpu);
    CPUPPCState *env = &cpu->env;
    target_ulong msr, new_msr, vector;
    int srr0, srr1;

    if (excp <= POWERPC_EXCP_NONE || excp >= POWERPC_EXCP_NB) {
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
    }

    qemu_log_mask(CPU_LOG_INT, "Raise exception at " TARGET_FMT_lx
                  " => %s (%d) error=%02x\n", env->nip, powerpc_excp_name(excp),
                  excp, env->error_code);

    msr = env->msr;

    /*
     * new interrupt handler msr preserves existing ME unless
     * explicitly overriden
     */
    new_msr = env->msr & ((target_ulong)1 << MSR_ME);

    /* target registers */
    srr0 = SPR_SRR0;
    srr1 = SPR_SRR1;

    /*
     * Hypervisor emulation assistance interrupt only exists on server
     * arch 2.05 server or later.
     */
    if (excp == POWERPC_EXCP_HV_EMU) {
        excp = POWERPC_EXCP_PROGRAM;
    }

#ifdef TARGET_PPC64
    /*
     * SPEU and VPU share the same IVOR but they exist in different
     * processors. SPEU is e500v1/2 only and VPU is e6500 only.
     */
    if (excp == POWERPC_EXCP_VPU) {
        excp = POWERPC_EXCP_SPEU;
    }
#endif

    vector = env->excp_vectors[excp];
    if (vector == (target_ulong)-1ULL) {
        cpu_abort(cs, "Raised an exception without defined vector %d\n",
                  excp);
    }

    vector |= env->excp_prefix;

    switch (excp) {
    case POWERPC_EXCP_CRITICAL:    /* Critical input                         */
        srr0 = SPR_BOOKE_CSRR0;
        srr1 = SPR_BOOKE_CSRR1;
        break;
    case POWERPC_EXCP_MCHECK:    /* Machine check exception                  */
        if (msr_me == 0) {
            /*
             * Machine check exception is not enabled.  Enter
             * checkstop state.
             */
            fprintf(stderr, "Machine check while not allowed. "
                    "Entering checkstop state\n");
            if (qemu_log_separate()) {
                qemu_log("Machine check while not allowed. "
                        "Entering checkstop state\n");
            }
            cs->halted = 1;
            cpu_interrupt_exittb(cs);
        }

        /* machine check exceptions don't have ME set */
        new_msr &= ~((target_ulong)1 << MSR_ME);

        /* FIXME: choose one or the other based on CPU type */
        srr0 = SPR_BOOKE_MCSRR0;
        srr1 = SPR_BOOKE_MCSRR1;

        env->spr[SPR_BOOKE_CSRR0] = env->nip;
        env->spr[SPR_BOOKE_CSRR1] = msr;

        break;
    case POWERPC_EXCP_DSI:       /* Data storage exception                   */
        trace_ppc_excp_dsi(env->spr[SPR_BOOKE_ESR], env->spr[SPR_BOOKE_DEAR]);
        break;
    case POWERPC_EXCP_ISI:       /* Instruction storage exception            */
        trace_ppc_excp_isi(msr, env->nip);
        break;
    case POWERPC_EXCP_EXTERNAL:  /* External input                           */
        if (env->mpic_proxy) {
            /* IACK the IRQ on delivery */
            env->spr[SPR_BOOKE_EPR] = ldl_phys(cs->as, env->mpic_iack);
        }
        break;
    case POWERPC_EXCP_ALIGN:     /* Alignment exception                      */
        break;
    case POWERPC_EXCP_PROGRAM:   /* Program exception                        */
        switch (env->error_code & ~0xF) {
        case POWERPC_EXCP_FP:
            if ((msr_fe0 == 0 && msr_fe1 == 0) || msr_fp == 0) {
                trace_ppc_excp_fp_ignore();
                cs->exception_index = POWERPC_EXCP_NONE;
                env->error_code = 0;
                return;
            }

            /*
             * FP exceptions always have NIP pointing to the faulting
             * instruction, so always use store_next and claim we are
             * precise in the MSR.
             */
            msr |= 0x00100000;
            env->spr[SPR_BOOKE_ESR] = ESR_FP;
            break;
        case POWERPC_EXCP_INVAL:
            trace_ppc_excp_inval(env->nip);
            msr |= 0x00080000;
            env->spr[SPR_BOOKE_ESR] = ESR_PIL;
            break;
        case POWERPC_EXCP_PRIV:
            msr |= 0x00040000;
            env->spr[SPR_BOOKE_ESR] = ESR_PPR;
            break;
        case POWERPC_EXCP_TRAP:
            msr |= 0x00020000;
            env->spr[SPR_BOOKE_ESR] = ESR_PTR;
            break;
        default:
            /* Should never occur */
            cpu_abort(cs, "Invalid program exception %d. Aborting\n",
                      env->error_code);
            break;
        }
        break;
    case POWERPC_EXCP_SYSCALL:   /* System call exception                    */
        dump_syscall(env);

        /*
         * We need to correct the NIP which in this case is supposed
         * to point to the next instruction
         */
        env->nip += 4;
        break;
    case POWERPC_EXCP_FPU:       /* Floating-point unavailable exception     */
    case POWERPC_EXCP_APU:       /* Auxiliary processor unavailable          */
    case POWERPC_EXCP_DECR:      /* Decrementer exception                    */
        break;
    case POWERPC_EXCP_FIT:       /* Fixed-interval timer interrupt           */
        /* FIT on 4xx */
        trace_ppc_excp_print("FIT");
        break;
    case POWERPC_EXCP_WDT:       /* Watchdog timer interrupt                 */
        trace_ppc_excp_print("WDT");
        srr0 = SPR_BOOKE_CSRR0;
        srr1 = SPR_BOOKE_CSRR1;
        break;
    case POWERPC_EXCP_DTLB:      /* Data TLB error                           */
    case POWERPC_EXCP_ITLB:      /* Instruction TLB error                    */
        break;
    case POWERPC_EXCP_DEBUG:     /* Debug interrupt                          */
        if (env->flags & POWERPC_FLAG_DE) {
            /* FIXME: choose one or the other based on CPU type */
            srr0 = SPR_BOOKE_DSRR0;
            srr1 = SPR_BOOKE_DSRR1;

            env->spr[SPR_BOOKE_CSRR0] = env->nip;
            env->spr[SPR_BOOKE_CSRR1] = msr;

            /* DBSR already modified by caller */
        } else {
            cpu_abort(cs, "Debug exception triggered on unsupported model\n");
        }
        break;
    case POWERPC_EXCP_SPEU:   /* SPE/embedded floating-point unavailable/VPU  */
        env->spr[SPR_BOOKE_ESR] = ESR_SPV;
        break;
    case POWERPC_EXCP_RESET:     /* System reset exception                   */
        if (msr_pow) {
            cpu_abort(cs, "Trying to deliver power-saving system reset "
                      "exception %d with no HV support\n", excp);
        }
        break;
    case POWERPC_EXCP_EFPDI:     /* Embedded floating-point data interrupt   */
    case POWERPC_EXCP_EFPRI:     /* Embedded floating-point round interrupt  */
        cpu_abort(cs, "%s exception not implemented\n",
                  powerpc_excp_name(excp));
        break;
    default:
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
        break;
    }

    /* Sanity check */
    if (!(env->msr_mask & MSR_HVB)) {
        if (new_msr & MSR_HVB) {
            cpu_abort(cs, "Trying to deliver HV exception (MSR) %d with "
                      "no HV support\n", excp);
        }
        if (srr0 == SPR_HSRR0) {
            cpu_abort(cs, "Trying to deliver HV exception (HSRR) %d with "
                      "no HV support\n", excp);
        }
    }

#if defined(TARGET_PPC64)
    if (env->spr[SPR_BOOKE_EPCR] & EPCR_ICM) {
        /* Cat.64-bit: EPCR.ICM is copied to MSR.CM */
        new_msr |= (target_ulong)1 << MSR_CM;
    } else {
        vector = (uint32_t)vector;
    }
#endif

    /* Save PC */
    env->spr[srr0] = env->nip;

    /* Save MSR */
    env->spr[srr1] = msr;

    powerpc_set_excp_state(cpu, vector, new_msr);
}

#ifdef TARGET_PPC64
static void powerpc_excp_books(PowerPCCPU *cpu, int excp)
{
    CPUState *cs = CPU(cpu);
    CPUPPCState *env = &cpu->env;
    int excp_model = env->excp_model;
    target_ulong msr, new_msr, vector;
    int srr0, srr1, lev = -1;

    if (excp <= POWERPC_EXCP_NONE || excp >= POWERPC_EXCP_NB) {
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
    }

    qemu_log_mask(CPU_LOG_INT, "Raise exception at " TARGET_FMT_lx
                  " => %s (%d) error=%02x\n", env->nip, powerpc_excp_name(excp),
                  excp, env->error_code);

    /* new srr1 value excluding must-be-zero bits */
    msr = env->msr & ~0x783f0000ULL;

    /*
     * new interrupt handler msr preserves existing HV and ME unless
     * explicitly overriden
     */
    new_msr = env->msr & (((target_ulong)1 << MSR_ME) | MSR_HVB);

    /* target registers */
    srr0 = SPR_SRR0;
    srr1 = SPR_SRR1;

    /*
     * check for special resume at 0x100 from doze/nap/sleep/winkle on
     * P7/P8/P9
     */
    if (env->resume_as_sreset) {
        excp = powerpc_reset_wakeup(cs, env, excp, &msr);
    }

    /*
     * We don't want to generate a Hypervisor Emulation Assistance
     * Interrupt if we don't have HVB in msr_mask (PAPR mode).
     */
    if (excp == POWERPC_EXCP_HV_EMU && !(env->msr_mask & MSR_HVB)) {
        excp = POWERPC_EXCP_PROGRAM;
    }

    vector = env->excp_vectors[excp];
    if (vector == (target_ulong)-1ULL) {
        cpu_abort(cs, "Raised an exception without defined vector %d\n",
                  excp);
    }

    vector |= env->excp_prefix;

    switch (excp) {
    case POWERPC_EXCP_MCHECK:    /* Machine check exception                  */
        if (msr_me == 0) {
            /*
             * Machine check exception is not enabled.  Enter
             * checkstop state.
             */
            fprintf(stderr, "Machine check while not allowed. "
                    "Entering checkstop state\n");
            if (qemu_log_separate()) {
                qemu_log("Machine check while not allowed. "
                        "Entering checkstop state\n");
            }
            cs->halted = 1;
            cpu_interrupt_exittb(cs);
        }
        if (env->msr_mask & MSR_HVB) {
            /*
             * ISA specifies HV, but can be delivered to guest with HV
             * clear (e.g., see FWNMI in PAPR).
             */
            new_msr |= (target_ulong)MSR_HVB;
        }

        /* machine check exceptions don't have ME set */
        new_msr &= ~((target_ulong)1 << MSR_ME);

        break;
    case POWERPC_EXCP_DSI:       /* Data storage exception                   */
        trace_ppc_excp_dsi(env->spr[SPR_DSISR], env->spr[SPR_DAR]);
        break;
    case POWERPC_EXCP_ISI:       /* Instruction storage exception            */
        trace_ppc_excp_isi(msr, env->nip);
        msr |= env->error_code;
        break;
    case POWERPC_EXCP_EXTERNAL:  /* External input                           */
    {
        bool lpes0;

        /*
         * LPES0 is only taken into consideration if we support HV
         * mode for this CPU.
         */
        if (!env->has_hv_mode) {
            break;
        }

        lpes0 = !!(env->spr[SPR_LPCR] & LPCR_LPES0);

        if (!lpes0) {
            new_msr |= (target_ulong)MSR_HVB;
            new_msr |= env->msr & ((target_ulong)1 << MSR_RI);
            srr0 = SPR_HSRR0;
            srr1 = SPR_HSRR1;
        }

        break;
    }
    case POWERPC_EXCP_ALIGN:     /* Alignment exception                      */
        /* Get rS/rD and rA from faulting opcode */
        /*
         * Note: the opcode fields will not be set properly for a
         * direct store load/store, but nobody cares as nobody
         * actually uses direct store segments.
         */
        env->spr[SPR_DSISR] |= (env->error_code & 0x03FF0000) >> 16;
        break;
    case POWERPC_EXCP_PROGRAM:   /* Program exception                        */
        switch (env->error_code & ~0xF) {
        case POWERPC_EXCP_FP:
            if ((msr_fe0 == 0 && msr_fe1 == 0) || msr_fp == 0) {
                trace_ppc_excp_fp_ignore();
                cs->exception_index = POWERPC_EXCP_NONE;
                env->error_code = 0;
                return;
            }

            /*
             * FP exceptions always have NIP pointing to the faulting
             * instruction, so always use store_next and claim we are
             * precise in the MSR.
             */
            msr |= 0x00100000;
            break;
        case POWERPC_EXCP_INVAL:
            trace_ppc_excp_inval(env->nip);
            msr |= 0x00080000;
            break;
        case POWERPC_EXCP_PRIV:
            msr |= 0x00040000;
            break;
        case POWERPC_EXCP_TRAP:
            msr |= 0x00020000;
            break;
        default:
            /* Should never occur */
            cpu_abort(cs, "Invalid program exception %d. Aborting\n",
                      env->error_code);
            break;
        }
        break;
    case POWERPC_EXCP_SYSCALL:   /* System call exception                    */
        lev = env->error_code;

        if ((lev == 1) && cpu->vhyp) {
            dump_hcall(env);
        } else {
            dump_syscall(env);
        }

        /*
         * We need to correct the NIP which in this case is supposed
         * to point to the next instruction
         */
        env->nip += 4;

        /* "PAPR mode" built-in hypercall emulation */
        if ((lev == 1) && cpu->vhyp) {
            PPCVirtualHypervisorClass *vhc =
                PPC_VIRTUAL_HYPERVISOR_GET_CLASS(cpu->vhyp);
            vhc->hypercall(cpu->vhyp, cpu);
            return;
        }
        if (lev == 1) {
            new_msr |= (target_ulong)MSR_HVB;
        }
        break;
    case POWERPC_EXCP_SYSCALL_VECTORED: /* scv exception                     */
        lev = env->error_code;
        dump_syscall(env);
        env->nip += 4;
        new_msr |= env->msr & ((target_ulong)1 << MSR_EE);
        new_msr |= env->msr & ((target_ulong)1 << MSR_RI);

        vector += lev * 0x20;

        env->lr = env->nip;
        env->ctr = msr;
        break;
    case POWERPC_EXCP_FPU:       /* Floating-point unavailable exception     */
    case POWERPC_EXCP_DECR:      /* Decrementer exception                    */
        break;
    case POWERPC_EXCP_RESET:     /* System reset exception                   */
        /* A power-saving exception sets ME, otherwise it is unchanged */
        if (msr_pow) {
            /* indicate that we resumed from power save mode */
            msr |= 0x10000;
            new_msr |= ((target_ulong)1 << MSR_ME);
        }
        if (env->msr_mask & MSR_HVB) {
            /*
             * ISA specifies HV, but can be delivered to guest with HV
             * clear (e.g., see FWNMI in PAPR, NMI injection in QEMU).
             */
            new_msr |= (target_ulong)MSR_HVB;
        } else {
            if (msr_pow) {
                cpu_abort(cs, "Trying to deliver power-saving system reset "
                          "exception %d with no HV support\n", excp);
            }
        }
        break;
    case POWERPC_EXCP_DSEG:      /* Data segment exception                   */
    case POWERPC_EXCP_ISEG:      /* Instruction segment exception            */
    case POWERPC_EXCP_TRACE:     /* Trace exception                          */
        break;
    case POWERPC_EXCP_HISI:      /* Hypervisor instruction storage exception */
        msr |= env->error_code;
        /* fall through */
    case POWERPC_EXCP_HDECR:     /* Hypervisor decrementer exception         */
    case POWERPC_EXCP_HDSI:      /* Hypervisor data storage exception        */
    case POWERPC_EXCP_SDOOR_HV:  /* Hypervisor Doorbell interrupt            */
    case POWERPC_EXCP_HV_EMU:
    case POWERPC_EXCP_HVIRT:     /* Hypervisor virtualization                */
        srr0 = SPR_HSRR0;
        srr1 = SPR_HSRR1;
        new_msr |= (target_ulong)MSR_HVB;
        new_msr |= env->msr & ((target_ulong)1 << MSR_RI);
        break;
    case POWERPC_EXCP_VPU:       /* Vector unavailable exception             */
    case POWERPC_EXCP_VSXU:       /* VSX unavailable exception               */
    case POWERPC_EXCP_FU:         /* Facility unavailable exception          */
        env->spr[SPR_FSCR] |= ((target_ulong)env->error_code << 56);
        break;
    case POWERPC_EXCP_HV_FU:     /* Hypervisor Facility Unavailable Exception */
        env->spr[SPR_HFSCR] |= ((target_ulong)env->error_code << FSCR_IC_POS);
        srr0 = SPR_HSRR0;
        srr1 = SPR_HSRR1;
        new_msr |= (target_ulong)MSR_HVB;
        new_msr |= env->msr & ((target_ulong)1 << MSR_RI);
        break;
    case POWERPC_EXCP_THERM:     /* Thermal interrupt                        */
    case POWERPC_EXCP_PERFM:     /* Embedded performance monitor interrupt   */
    case POWERPC_EXCP_VPUA:      /* Vector assist exception                  */
    case POWERPC_EXCP_MAINT:     /* Maintenance exception                    */
    case POWERPC_EXCP_SDOOR:     /* Doorbell interrupt                       */
    case POWERPC_EXCP_HV_MAINT:  /* Hypervisor Maintenance exception         */
        cpu_abort(cs, "%s exception not implemented\n",
                  powerpc_excp_name(excp));
        break;
    default:
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
        break;
    }

    /* Sanity check */
    if (!(env->msr_mask & MSR_HVB)) {
        if (new_msr & MSR_HVB) {
            cpu_abort(cs, "Trying to deliver HV exception (MSR) %d with "
                      "no HV support\n", excp);
        }
        if (srr0 == SPR_HSRR0) {
            cpu_abort(cs, "Trying to deliver HV exception (HSRR) %d with "
                      "no HV support\n", excp);
        }
    }

    /*
     * Sort out endianness of interrupt, this differs depending on the
     * CPU, the HV mode, etc...
     */
    if (ppc_interrupts_little_endian(cpu, !!(new_msr & MSR_HVB))) {
        new_msr |= (target_ulong)1 << MSR_LE;
    }

    new_msr |= (target_ulong)1 << MSR_SF;

    if (excp != POWERPC_EXCP_SYSCALL_VECTORED) {
        /* Save PC */
        env->spr[srr0] = env->nip;

        /* Save MSR */
        env->spr[srr1] = msr;
    }

    /* This can update new_msr and vector if AIL applies */
    ppc_excp_apply_ail(cpu, excp_model, excp, msr, &new_msr, &vector);

    powerpc_set_excp_state(cpu, vector, new_msr);
}
#else
static inline void powerpc_excp_books(PowerPCCPU *cpu, int excp)
{
    g_assert_not_reached();
}
#endif

/*
 * Note that this function should be greatly optimized when called
 * with a constant excp, from ppc_hw_interrupt
 */
static inline void powerpc_excp_legacy(PowerPCCPU *cpu, int excp)
{
    CPUState *cs = CPU(cpu);
    CPUPPCState *env = &cpu->env;
    int excp_model = env->excp_model;
    target_ulong msr, new_msr, vector;
    int srr0, srr1, lev = -1;

    if (excp <= POWERPC_EXCP_NONE || excp >= POWERPC_EXCP_NB) {
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
    }

    qemu_log_mask(CPU_LOG_INT, "Raise exception at " TARGET_FMT_lx
                  " => %s (%d) error=%02x\n", env->nip, powerpc_excp_name(excp),
                  excp, env->error_code);

    /* new srr1 value excluding must-be-zero bits */
    if (excp_model == POWERPC_EXCP_BOOKE) {
        msr = env->msr;
    } else {
        msr = env->msr & ~0x783f0000ULL;
    }

    /*
     * new interrupt handler msr preserves existing HV and ME unless
     * explicitly overriden
     */
    new_msr = env->msr & (((target_ulong)1 << MSR_ME) | MSR_HVB);

    /* target registers */
    srr0 = SPR_SRR0;
    srr1 = SPR_SRR1;

    /*
     * check for special resume at 0x100 from doze/nap/sleep/winkle on
     * P7/P8/P9
     */
    if (env->resume_as_sreset) {
        excp = powerpc_reset_wakeup(cs, env, excp, &msr);
    }

    /*
     * Hypervisor emulation assistance interrupt only exists on server
     * arch 2.05 server or later. We also don't want to generate it if
     * we don't have HVB in msr_mask (PAPR mode).
     */
    if (excp == POWERPC_EXCP_HV_EMU
#if defined(TARGET_PPC64)
        && !(mmu_is_64bit(env->mmu_model) && (env->msr_mask & MSR_HVB))
#endif /* defined(TARGET_PPC64) */

    ) {
        excp = POWERPC_EXCP_PROGRAM;
    }

#ifdef TARGET_PPC64
    /*
     * SPEU and VPU share the same IVOR but they exist in different
     * processors. SPEU is e500v1/2 only and VPU is e6500 only.
     */
    if (excp_model == POWERPC_EXCP_BOOKE && excp == POWERPC_EXCP_VPU) {
        excp = POWERPC_EXCP_SPEU;
    }
#endif

    vector = env->excp_vectors[excp];
    if (vector == (target_ulong)-1ULL) {
        cpu_abort(cs, "Raised an exception without defined vector %d\n",
                  excp);
    }

    vector |= env->excp_prefix;

    switch (excp) {
    case POWERPC_EXCP_CRITICAL:    /* Critical input                         */
        switch (excp_model) {
        case POWERPC_EXCP_40x:
            srr0 = SPR_40x_SRR2;
            srr1 = SPR_40x_SRR3;
            break;
        case POWERPC_EXCP_BOOKE:
            srr0 = SPR_BOOKE_CSRR0;
            srr1 = SPR_BOOKE_CSRR1;
            break;
        case POWERPC_EXCP_6xx:
            break;
        default:
            goto excp_invalid;
        }
        break;
    case POWERPC_EXCP_MCHECK:    /* Machine check exception                  */
        if (msr_me == 0) {
            /*
             * Machine check exception is not enabled.  Enter
             * checkstop state.
             */
            fprintf(stderr, "Machine check while not allowed. "
                    "Entering checkstop state\n");
            if (qemu_log_separate()) {
                qemu_log("Machine check while not allowed. "
                        "Entering checkstop state\n");
            }
            cs->halted = 1;
            cpu_interrupt_exittb(cs);
        }
        if (env->msr_mask & MSR_HVB) {
            /*
             * ISA specifies HV, but can be delivered to guest with HV
             * clear (e.g., see FWNMI in PAPR).
             */
            new_msr |= (target_ulong)MSR_HVB;
        }

        /* machine check exceptions don't have ME set */
        new_msr &= ~((target_ulong)1 << MSR_ME);

        /* XXX: should also have something loaded in DAR / DSISR */
        switch (excp_model) {
        case POWERPC_EXCP_40x:
            srr0 = SPR_40x_SRR2;
            srr1 = SPR_40x_SRR3;
            break;
        case POWERPC_EXCP_BOOKE:
            /* FIXME: choose one or the other based on CPU type */
            srr0 = SPR_BOOKE_MCSRR0;
            srr1 = SPR_BOOKE_MCSRR1;

            env->spr[SPR_BOOKE_CSRR0] = env->nip;
            env->spr[SPR_BOOKE_CSRR1] = msr;
            break;
        default:
            break;
        }
        break;
    case POWERPC_EXCP_DSI:       /* Data storage exception                   */
        trace_ppc_excp_dsi(env->spr[SPR_DSISR], env->spr[SPR_DAR]);
        break;
    case POWERPC_EXCP_ISI:       /* Instruction storage exception            */
        trace_ppc_excp_isi(msr, env->nip);
        msr |= env->error_code;
        break;
    case POWERPC_EXCP_EXTERNAL:  /* External input                           */
    {
        bool lpes0;

        cs = CPU(cpu);

        /*
         * Exception targeting modifiers
         *
         * LPES0 is supported on POWER7/8/9
         * LPES1 is not supported (old iSeries mode)
         *
         * On anything else, we behave as if LPES0 is 1
         * (externals don't alter MSR:HV)
         */
#if defined(TARGET_PPC64)
        if (excp_model == POWERPC_EXCP_POWER7 ||
            excp_model == POWERPC_EXCP_POWER8 ||
            excp_model == POWERPC_EXCP_POWER9 ||
            excp_model == POWERPC_EXCP_POWER10) {
            lpes0 = !!(env->spr[SPR_LPCR] & LPCR_LPES0);
        } else
#endif /* defined(TARGET_PPC64) */
        {
            lpes0 = true;
        }

        if (!lpes0) {
            new_msr |= (target_ulong)MSR_HVB;
            new_msr |= env->msr & ((target_ulong)1 << MSR_RI);
            srr0 = SPR_HSRR0;
            srr1 = SPR_HSRR1;
        }
        if (env->mpic_proxy) {
            /* IACK the IRQ on delivery */
            env->spr[SPR_BOOKE_EPR] = ldl_phys(cs->as, env->mpic_iack);
        }
        break;
    }
    case POWERPC_EXCP_ALIGN:     /* Alignment exception                      */
        /* Get rS/rD and rA from faulting opcode */
        /*
         * Note: the opcode fields will not be set properly for a
         * direct store load/store, but nobody cares as nobody
         * actually uses direct store segments.
         */
        env->spr[SPR_DSISR] |= (env->error_code & 0x03FF0000) >> 16;
        break;
    case POWERPC_EXCP_PROGRAM:   /* Program exception                        */
        switch (env->error_code & ~0xF) {
        case POWERPC_EXCP_FP:
            if ((msr_fe0 == 0 && msr_fe1 == 0) || msr_fp == 0) {
                trace_ppc_excp_fp_ignore();
                cs->exception_index = POWERPC_EXCP_NONE;
                env->error_code = 0;
                return;
            }

            /*
             * FP exceptions always have NIP pointing to the faulting
             * instruction, so always use store_next and claim we are
             * precise in the MSR.
             */
            msr |= 0x00100000;
            env->spr[SPR_BOOKE_ESR] = ESR_FP;
            break;
        case POWERPC_EXCP_INVAL:
            trace_ppc_excp_inval(env->nip);
            msr |= 0x00080000;
            env->spr[SPR_BOOKE_ESR] = ESR_PIL;
            break;
        case POWERPC_EXCP_PRIV:
            msr |= 0x00040000;
            env->spr[SPR_BOOKE_ESR] = ESR_PPR;
            break;
        case POWERPC_EXCP_TRAP:
            msr |= 0x00020000;
            env->spr[SPR_BOOKE_ESR] = ESR_PTR;
            break;
        default:
            /* Should never occur */
            cpu_abort(cs, "Invalid program exception %d. Aborting\n",
                      env->error_code);
            break;
        }
        break;
    case POWERPC_EXCP_SYSCALL:   /* System call exception                    */
        lev = env->error_code;

        if ((lev == 1) && cpu->vhyp) {
            dump_hcall(env);
        } else {
            dump_syscall(env);
        }

        /*
         * We need to correct the NIP which in this case is supposed
         * to point to the next instruction
         */
        env->nip += 4;

        /* "PAPR mode" built-in hypercall emulation */
        if ((lev == 1) && cpu->vhyp) {
            PPCVirtualHypervisorClass *vhc =
                PPC_VIRTUAL_HYPERVISOR_GET_CLASS(cpu->vhyp);
            vhc->hypercall(cpu->vhyp, cpu);
            return;
        }
        if (lev == 1) {
            new_msr |= (target_ulong)MSR_HVB;
        }
        break;
    case POWERPC_EXCP_SYSCALL_VECTORED: /* scv exception                     */
        lev = env->error_code;
        dump_syscall(env);
        env->nip += 4;
        new_msr |= env->msr & ((target_ulong)1 << MSR_EE);
        new_msr |= env->msr & ((target_ulong)1 << MSR_RI);

        vector += lev * 0x20;

        env->lr = env->nip;
        env->ctr = msr;
        break;
    case POWERPC_EXCP_FPU:       /* Floating-point unavailable exception     */
    case POWERPC_EXCP_APU:       /* Auxiliary processor unavailable          */
    case POWERPC_EXCP_DECR:      /* Decrementer exception                    */
        break;
    case POWERPC_EXCP_FIT:       /* Fixed-interval timer interrupt           */
        /* FIT on 4xx */
        trace_ppc_excp_print("FIT");
        break;
    case POWERPC_EXCP_WDT:       /* Watchdog timer interrupt                 */
        trace_ppc_excp_print("WDT");
        switch (excp_model) {
        case POWERPC_EXCP_BOOKE:
            srr0 = SPR_BOOKE_CSRR0;
            srr1 = SPR_BOOKE_CSRR1;
            break;
        default:
            break;
        }
        break;
    case POWERPC_EXCP_DTLB:      /* Data TLB error                           */
    case POWERPC_EXCP_ITLB:      /* Instruction TLB error                    */
        break;
    case POWERPC_EXCP_DEBUG:     /* Debug interrupt                          */
        if (env->flags & POWERPC_FLAG_DE) {
            /* FIXME: choose one or the other based on CPU type */
            srr0 = SPR_BOOKE_DSRR0;
            srr1 = SPR_BOOKE_DSRR1;

            env->spr[SPR_BOOKE_CSRR0] = env->nip;
            env->spr[SPR_BOOKE_CSRR1] = msr;

            /* DBSR already modified by caller */
        } else {
            cpu_abort(cs, "Debug exception triggered on unsupported model\n");
        }
        break;
    case POWERPC_EXCP_SPEU:   /* SPE/embedded floating-point unavailable/VPU  */
        env->spr[SPR_BOOKE_ESR] = ESR_SPV;
        break;
    case POWERPC_EXCP_DOORI:     /* Embedded doorbell interrupt              */
        break;
    case POWERPC_EXCP_DOORCI:    /* Embedded doorbell critical interrupt     */
        srr0 = SPR_BOOKE_CSRR0;
        srr1 = SPR_BOOKE_CSRR1;
        break;
    case POWERPC_EXCP_RESET:     /* System reset exception                   */
        /* A power-saving exception sets ME, otherwise it is unchanged */
        if (msr_pow) {
            /* indicate that we resumed from power save mode */
            msr |= 0x10000;
            new_msr |= ((target_ulong)1 << MSR_ME);
        }
        if (env->msr_mask & MSR_HVB) {
            /*
             * ISA specifies HV, but can be delivered to guest with HV
             * clear (e.g., see FWNMI in PAPR, NMI injection in QEMU).
             */
            new_msr |= (target_ulong)MSR_HVB;
        } else {
            if (msr_pow) {
                cpu_abort(cs, "Trying to deliver power-saving system reset "
                          "exception %d with no HV support\n", excp);
            }
        }
        break;
    case POWERPC_EXCP_DSEG:      /* Data segment exception                   */
    case POWERPC_EXCP_ISEG:      /* Instruction segment exception            */
    case POWERPC_EXCP_TRACE:     /* Trace exception                          */
        break;
    case POWERPC_EXCP_HISI:      /* Hypervisor instruction storage exception */
        msr |= env->error_code;
        /* fall through */
    case POWERPC_EXCP_HDECR:     /* Hypervisor decrementer exception         */
    case POWERPC_EXCP_HDSI:      /* Hypervisor data storage exception        */
    case POWERPC_EXCP_HDSEG:     /* Hypervisor data segment exception        */
    case POWERPC_EXCP_HISEG:     /* Hypervisor instruction segment exception */
    case POWERPC_EXCP_SDOOR_HV:  /* Hypervisor Doorbell interrupt            */
    case POWERPC_EXCP_HV_EMU:
    case POWERPC_EXCP_HVIRT:     /* Hypervisor virtualization                */
        srr0 = SPR_HSRR0;
        srr1 = SPR_HSRR1;
        new_msr |= (target_ulong)MSR_HVB;
        new_msr |= env->msr & ((target_ulong)1 << MSR_RI);
        break;
    case POWERPC_EXCP_VPU:       /* Vector unavailable exception             */
    case POWERPC_EXCP_VSXU:       /* VSX unavailable exception               */
    case POWERPC_EXCP_FU:         /* Facility unavailable exception          */
#ifdef TARGET_PPC64
        env->spr[SPR_FSCR] |= ((target_ulong)env->error_code << 56);
#endif
        break;
    case POWERPC_EXCP_HV_FU:     /* Hypervisor Facility Unavailable Exception */
#ifdef TARGET_PPC64
        env->spr[SPR_HFSCR] |= ((target_ulong)env->error_code << FSCR_IC_POS);
        srr0 = SPR_HSRR0;
        srr1 = SPR_HSRR1;
        new_msr |= (target_ulong)MSR_HVB;
        new_msr |= env->msr & ((target_ulong)1 << MSR_RI);
#endif
        break;
    case POWERPC_EXCP_PIT:       /* Programmable interval timer interrupt    */
        trace_ppc_excp_print("PIT");
        break;
    case POWERPC_EXCP_IFTLB:     /* Instruction fetch TLB error              */
    case POWERPC_EXCP_DLTLB:     /* Data load TLB miss                       */
    case POWERPC_EXCP_DSTLB:     /* Data store TLB miss                      */
        switch (excp_model) {
        case POWERPC_EXCP_6xx:
            /* Swap temporary saved registers with GPRs */
            if (!(new_msr & ((target_ulong)1 << MSR_TGPR))) {
                new_msr |= (target_ulong)1 << MSR_TGPR;
                hreg_swap_gpr_tgpr(env);
            }
            /* fall through */
        case POWERPC_EXCP_7xx:
            ppc_excp_debug_sw_tlb(env, excp);

            msr |= env->crf[0] << 28;
            msr |= env->error_code; /* key, D/I, S/L bits */
            /* Set way using a LRU mechanism */
            msr |= ((env->last_way + 1) & (env->nb_ways - 1)) << 17;
            break;
        default:
            cpu_abort(cs, "Invalid TLB miss exception\n");
            break;
        }
        break;
    case POWERPC_EXCP_EFPDI:     /* Embedded floating-point data interrupt   */
    case POWERPC_EXCP_EFPRI:     /* Embedded floating-point round interrupt  */
    case POWERPC_EXCP_EPERFM:    /* Embedded performance monitor interrupt   */
    case POWERPC_EXCP_FPA:       /* Floating-point assist exception          */
    case POWERPC_EXCP_DABR:      /* Data address breakpoint                  */
    case POWERPC_EXCP_IABR:      /* Instruction address breakpoint           */
    case POWERPC_EXCP_SMI:       /* System management interrupt              */
    case POWERPC_EXCP_THERM:     /* Thermal interrupt                        */
    case POWERPC_EXCP_PERFM:     /* Embedded performance monitor interrupt   */
    case POWERPC_EXCP_VPUA:      /* Vector assist exception                  */
    case POWERPC_EXCP_SOFTP:     /* Soft patch exception                     */
    case POWERPC_EXCP_MAINT:     /* Maintenance exception                    */
    case POWERPC_EXCP_MEXTBR:    /* Maskable external breakpoint             */
    case POWERPC_EXCP_NMEXTBR:   /* Non maskable external breakpoint         */
        cpu_abort(cs, "%s exception not implemented\n",
                  powerpc_excp_name(excp));
        break;
    default:
    excp_invalid:
        cpu_abort(cs, "Invalid PowerPC exception %d. Aborting\n", excp);
        break;
    }

    /* Sanity check */
    if (!(env->msr_mask & MSR_HVB)) {
        if (new_msr & MSR_HVB) {
            cpu_abort(cs, "Trying to deliver HV exception (MSR) %d with "
                      "no HV support\n", excp);
        }
        if (srr0 == SPR_HSRR0) {
            cpu_abort(cs, "Trying to deliver HV exception (HSRR) %d with "
                      "no HV support\n", excp);
        }
    }

    /*
     * Sort out endianness of interrupt, this differs depending on the
     * CPU, the HV mode, etc...
     */
    if (ppc_interrupts_little_endian(cpu, !!(new_msr & MSR_HVB))) {
        new_msr |= (target_ulong)1 << MSR_LE;
    }

#if defined(TARGET_PPC64)
    if (excp_model == POWERPC_EXCP_BOOKE) {
        if (env->spr[SPR_BOOKE_EPCR] & EPCR_ICM) {
            /* Cat.64-bit: EPCR.ICM is copied to MSR.CM */
            new_msr |= (target_ulong)1 << MSR_CM;
        } else {
            vector = (uint32_t)vector;
        }
    } else {
        if (!msr_isf && !mmu_is_64bit(env->mmu_model)) {
            vector = (uint32_t)vector;
        } else {
            new_msr |= (target_ulong)1 << MSR_SF;
        }
    }
#endif

    if (excp != POWERPC_EXCP_SYSCALL_VECTORED) {
        /* Save PC */
        env->spr[srr0] = env->nip;

        /* Save MSR */
        env->spr[srr1] = msr;
    }

    /* This can update new_msr and vector if AIL applies */
    ppc_excp_apply_ail(cpu, excp_model, excp, msr, &new_msr, &vector);

    powerpc_set_excp_state(cpu, vector, new_msr);
}

static void powerpc_excp(PowerPCCPU *cpu, int excp)
{
    CPUPPCState *env = &cpu->env;

    switch (env->excp_model) {
    case POWERPC_EXCP_40x:
        powerpc_excp_40x(cpu, excp);
        break;
    case POWERPC_EXCP_6xx:
        powerpc_excp_6xx(cpu, excp);
        break;
    case POWERPC_EXCP_7xx:
        powerpc_excp_7xx(cpu, excp);
        break;
    case POWERPC_EXCP_74xx:
        powerpc_excp_74xx(cpu, excp);
        break;
    case POWERPC_EXCP_BOOKE:
        powerpc_excp_booke(cpu, excp);
        break;
    case POWERPC_EXCP_970:
    case POWERPC_EXCP_POWER7:
    case POWERPC_EXCP_POWER8:
    case POWERPC_EXCP_POWER9:
    case POWERPC_EXCP_POWER10:
        powerpc_excp_books(cpu, excp);
        break;
    default:
        powerpc_excp_legacy(cpu, excp);
    }
}

void ppc_cpu_do_interrupt(CPUState *cs)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);

    powerpc_excp(cpu, cs->exception_index);
}

static void ppc_hw_interrupt(CPUPPCState *env)
{
    PowerPCCPU *cpu = env_archcpu(env);
    bool async_deliver;

    /* External reset */
    if (env->pending_interrupts & (1 << PPC_INTERRUPT_RESET)) {
        env->pending_interrupts &= ~(1 << PPC_INTERRUPT_RESET);
        powerpc_excp(cpu, POWERPC_EXCP_RESET);
        return;
    }
    /* Machine check exception */
    if (env->pending_interrupts & (1 << PPC_INTERRUPT_MCK)) {
        env->pending_interrupts &= ~(1 << PPC_INTERRUPT_MCK);
        powerpc_excp(cpu, POWERPC_EXCP_MCHECK);
        return;
    }
#if 0 /* TODO */
    /* External debug exception */
    if (env->pending_interrupts & (1 << PPC_INTERRUPT_DEBUG)) {
        env->pending_interrupts &= ~(1 << PPC_INTERRUPT_DEBUG);
        powerpc_excp(cpu, POWERPC_EXCP_DEBUG);
        return;
    }
#endif

    /*
     * For interrupts that gate on MSR:EE, we need to do something a
     * bit more subtle, as we need to let them through even when EE is
     * clear when coming out of some power management states (in order
     * for them to become a 0x100).
     */
    async_deliver = (msr_ee != 0) || env->resume_as_sreset;

    /* Hypervisor decrementer exception */
    if (env->pending_interrupts & (1 << PPC_INTERRUPT_HDECR)) {
        /* LPCR will be clear when not supported so this will work */
        bool hdice = !!(env->spr[SPR_LPCR] & LPCR_HDICE);
        if ((async_deliver || msr_hv == 0) && hdice) {
            /* HDEC clears on delivery */
            env->pending_interrupts &= ~(1 << PPC_INTERRUPT_HDECR);
            powerpc_excp(cpu, POWERPC_EXCP_HDECR);
            return;
        }
    }

    /* Hypervisor virtualization interrupt */
    if (env->pending_interrupts & (1 << PPC_INTERRUPT_HVIRT)) {
        /* LPCR will be clear when not supported so this will work */
        bool hvice = !!(env->spr[SPR_LPCR] & LPCR_HVICE);
        if ((async_deliver || msr_hv == 0) && hvice) {
            powerpc_excp(cpu, POWERPC_EXCP_HVIRT);
            return;
        }
    }

    /* External interrupt can ignore MSR:EE under some circumstances */
    if (env->pending_interrupts & (1 << PPC_INTERRUPT_EXT)) {
        bool lpes0 = !!(env->spr[SPR_LPCR] & LPCR_LPES0);
        bool heic = !!(env->spr[SPR_LPCR] & LPCR_HEIC);
        /* HEIC blocks delivery to the hypervisor */
        if ((async_deliver && !(heic && msr_hv && !msr_pr)) ||
            (env->has_hv_mode && msr_hv == 0 && !lpes0)) {
            powerpc_excp(cpu, POWERPC_EXCP_EXTERNAL);
            return;
        }
    }
    if (msr_ce != 0) {
        /* External critical interrupt */
        if (env->pending_interrupts & (1 << PPC_INTERRUPT_CEXT)) {
            powerpc_excp(cpu, POWERPC_EXCP_CRITICAL);
            return;
        }
    }
    if (async_deliver != 0) {
        /* Watchdog timer on embedded PowerPC */
        if (env->pending_interrupts & (1 << PPC_INTERRUPT_WDT)) {
            env->pending_interrupts &= ~(1 << PPC_INTERRUPT_WDT);
            powerpc_excp(cpu, POWERPC_EXCP_WDT);
            return;
        }
        if (env->pending_interrupts & (1 << PPC_INTERRUPT_CDOORBELL)) {
            env->pending_interrupts &= ~(1 << PPC_INTERRUPT_CDOORBELL);
            powerpc_excp(cpu, POWERPC_EXCP_DOORCI);
            return;
        }
        /* Fixed interval timer on embedded PowerPC */
        if (env->pending_interrupts & (1 << PPC_INTERRUPT_FIT)) {
            env->pending_interrupts &= ~(1 << PPC_INTERRUPT_FIT);
            powerpc_excp(cpu, POWERPC_EXCP_FIT);
            return;
        }
        /* Programmable interval timer on embedded PowerPC */
        if (env->pending_interrupts & (1 << PPC_INTERRUPT_PIT)) {
            env->pending_interrupts &= ~(1 << PPC_INTERRUPT_PIT);
            powerpc_excp(cpu, POWERPC_EXCP_PIT);
            return;
        }
        /* Decrementer exception */
        if (env->pending_interrupts & (1 << PPC_INTERRUPT_DECR)) {
            if (ppc_decr_clear_on_delivery(env)) {
                env->pending_interrupts &= ~(1 << PPC_INTERRUPT_DECR);
            }
            powerpc_excp(cpu, POWERPC_EXCP_DECR);
            return;
        }
        if (env->pending_interrupts & (1 << PPC_INTERRUPT_DOORBELL)) {
            env->pending_interrupts &= ~(1 << PPC_INTERRUPT_DOORBELL);
            if (is_book3s_arch2x(env)) {
                powerpc_excp(cpu, POWERPC_EXCP_SDOOR);
            } else {
                powerpc_excp(cpu, POWERPC_EXCP_DOORI);
            }
            return;
        }
        if (env->pending_interrupts & (1 << PPC_INTERRUPT_HDOORBELL)) {
            env->pending_interrupts &= ~(1 << PPC_INTERRUPT_HDOORBELL);
            powerpc_excp(cpu, POWERPC_EXCP_SDOOR_HV);
            return;
        }
        if (env->pending_interrupts & (1 << PPC_INTERRUPT_PERFM)) {
            env->pending_interrupts &= ~(1 << PPC_INTERRUPT_PERFM);
            powerpc_excp(cpu, POWERPC_EXCP_PERFM);
            return;
        }
        /* Thermal interrupt */
        if (env->pending_interrupts & (1 << PPC_INTERRUPT_THERM)) {
            env->pending_interrupts &= ~(1 << PPC_INTERRUPT_THERM);
            powerpc_excp(cpu, POWERPC_EXCP_THERM);
            return;
        }
    }

    if (env->resume_as_sreset) {
        /*
         * This is a bug ! It means that has_work took us out of halt without
         * anything to deliver while in a PM state that requires getting
         * out via a 0x100
         *
         * This means we will incorrectly execute past the power management
         * instruction instead of triggering a reset.
         *
         * It generally means a discrepancy between the wakeup conditions in the
         * processor has_work implementation and the logic in this function.
         */
        cpu_abort(env_cpu(env),
                  "Wakeup from PM state but interrupt Undelivered");
    }
}

void ppc_cpu_do_system_reset(CPUState *cs)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);

    powerpc_excp(cpu, POWERPC_EXCP_RESET);
}

void ppc_cpu_do_fwnmi_machine_check(CPUState *cs, target_ulong vector)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);
    CPUPPCState *env = &cpu->env;
    target_ulong msr = 0;

    /*
     * Set MSR and NIP for the handler, SRR0/1, DAR and DSISR have already
     * been set by KVM.
     */
    msr = (1ULL << MSR_ME);
    msr |= env->msr & (1ULL << MSR_SF);
    if (ppc_interrupts_little_endian(cpu, false)) {
        msr |= (1ULL << MSR_LE);
    }

    powerpc_set_excp_state(cpu, vector, msr);
}

bool ppc_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);
    CPUPPCState *env = &cpu->env;

    if (interrupt_request & CPU_INTERRUPT_HARD) {
        ppc_hw_interrupt(env);
        if (env->pending_interrupts == 0) {
            cs->interrupt_request &= ~CPU_INTERRUPT_HARD;
        }
        return true;
    }
    return false;
}

#endif /* !CONFIG_USER_ONLY */

/*****************************************************************************/
/* Exceptions processing helpers */

void raise_exception_err_ra(CPUPPCState *env, uint32_t exception,
                            uint32_t error_code, uintptr_t raddr)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = exception;
    env->error_code = error_code;
    cpu_loop_exit_restore(cs, raddr);
}

void raise_exception_err(CPUPPCState *env, uint32_t exception,
                         uint32_t error_code)
{
    raise_exception_err_ra(env, exception, error_code, 0);
}

void raise_exception(CPUPPCState *env, uint32_t exception)
{
    raise_exception_err_ra(env, exception, 0, 0);
}

void raise_exception_ra(CPUPPCState *env, uint32_t exception,
                        uintptr_t raddr)
{
    raise_exception_err_ra(env, exception, 0, raddr);
}

#ifdef CONFIG_TCG
void helper_raise_exception_err(CPUPPCState *env, uint32_t exception,
                                uint32_t error_code)
{
    raise_exception_err_ra(env, exception, error_code, 0);
}

void helper_raise_exception(CPUPPCState *env, uint32_t exception)
{
    raise_exception_err_ra(env, exception, 0, 0);
}
#endif

#if !defined(CONFIG_USER_ONLY)
#ifdef CONFIG_TCG
void helper_store_msr(CPUPPCState *env, target_ulong val)
{
    uint32_t excp = hreg_store_msr(env, val, 0);

    if (excp != 0) {
        CPUState *cs = env_cpu(env);
        cpu_interrupt_exittb(cs);
        raise_exception(env, excp);
    }
}

#if defined(TARGET_PPC64)
void helper_scv(CPUPPCState *env, uint32_t lev)
{
    if (env->spr[SPR_FSCR] & (1ull << FSCR_SCV)) {
        raise_exception_err(env, POWERPC_EXCP_SYSCALL_VECTORED, lev);
    } else {
        raise_exception_err(env, POWERPC_EXCP_FU, FSCR_IC_SCV);
    }
}

void helper_pminsn(CPUPPCState *env, powerpc_pm_insn_t insn)
{
    CPUState *cs;

    cs = env_cpu(env);
    cs->halted = 1;

    /* Condition for waking up at 0x100 */
    env->resume_as_sreset = (insn != PPC_PM_STOP) ||
        (env->spr[SPR_PSSCR] & PSSCR_EC);
}
#endif /* defined(TARGET_PPC64) */

static void do_rfi(CPUPPCState *env, target_ulong nip, target_ulong msr)
{
    CPUState *cs = env_cpu(env);

    /* MSR:POW cannot be set by any form of rfi */
    msr &= ~(1ULL << MSR_POW);

    /* MSR:TGPR cannot be set by any form of rfi */
    if (env->flags & POWERPC_FLAG_TGPR)
        msr &= ~(1ULL << MSR_TGPR);

#if defined(TARGET_PPC64)
    /* Switching to 32-bit ? Crop the nip */
    if (!msr_is_64bit(env, msr)) {
        nip = (uint32_t)nip;
    }
#else
    nip = (uint32_t)nip;
#endif
    /* XXX: beware: this is false if VLE is supported */
    env->nip = nip & ~((target_ulong)0x00000003);
    hreg_store_msr(env, msr, 1);
    trace_ppc_excp_rfi(env->nip, env->msr);
    /*
     * No need to raise an exception here, as rfi is always the last
     * insn of a TB
     */
    cpu_interrupt_exittb(cs);
    /* Reset the reservation */
    env->reserve_addr = -1;

    /* Context synchronizing: check if TCG TLB needs flush */
    check_tlb_flush(env, false);
}

void helper_rfi(CPUPPCState *env)
{
    do_rfi(env, env->spr[SPR_SRR0], env->spr[SPR_SRR1] & 0xfffffffful);
}

#define MSR_BOOK3S_MASK
#if defined(TARGET_PPC64)
void helper_rfid(CPUPPCState *env)
{
    /*
     * The architecture defines a number of rules for which bits can
     * change but in practice, we handle this in hreg_store_msr()
     * which will be called by do_rfi(), so there is no need to filter
     * here
     */
    do_rfi(env, env->spr[SPR_SRR0], env->spr[SPR_SRR1]);
}

void helper_rfscv(CPUPPCState *env)
{
    do_rfi(env, env->lr, env->ctr);
}

void helper_hrfid(CPUPPCState *env)
{
    do_rfi(env, env->spr[SPR_HSRR0], env->spr[SPR_HSRR1]);
}
#endif

#if defined(TARGET_PPC64) && !defined(CONFIG_USER_ONLY)
void helper_rfebb(CPUPPCState *env, target_ulong s)
{
    target_ulong msr = env->msr;

    /*
     * Handling of BESCR bits 32:33 according to PowerISA v3.1:
     *
     * "If BESCR 32:33 != 0b00 the instruction is treated as if
     *  the instruction form were invalid."
     */
    if (env->spr[SPR_BESCR] & BESCR_INVALID) {
        raise_exception_err(env, POWERPC_EXCP_PROGRAM,
                            POWERPC_EXCP_INVAL | POWERPC_EXCP_INVAL_INVAL);
    }

    env->nip = env->spr[SPR_EBBRR];

    /* Switching to 32-bit ? Crop the nip */
    if (!msr_is_64bit(env, msr)) {
        env->nip = (uint32_t)env->spr[SPR_EBBRR];
    }

    if (s) {
        env->spr[SPR_BESCR] |= BESCR_GE;
    } else {
        env->spr[SPR_BESCR] &= ~BESCR_GE;
    }
}
#endif

/*****************************************************************************/
/* Embedded PowerPC specific helpers */
void helper_40x_rfci(CPUPPCState *env)
{
    do_rfi(env, env->spr[SPR_40x_SRR2], env->spr[SPR_40x_SRR3]);
}

void helper_rfci(CPUPPCState *env)
{
    do_rfi(env, env->spr[SPR_BOOKE_CSRR0], env->spr[SPR_BOOKE_CSRR1]);
}

void helper_rfdi(CPUPPCState *env)
{
    /* FIXME: choose CSRR1 or DSRR1 based on cpu type */
    do_rfi(env, env->spr[SPR_BOOKE_DSRR0], env->spr[SPR_BOOKE_DSRR1]);
}

void helper_rfmci(CPUPPCState *env)
{
    /* FIXME: choose CSRR1 or MCSRR1 based on cpu type */
    do_rfi(env, env->spr[SPR_BOOKE_MCSRR0], env->spr[SPR_BOOKE_MCSRR1]);
}
#endif /* CONFIG_TCG */
#endif /* !defined(CONFIG_USER_ONLY) */

#ifdef CONFIG_TCG
void helper_tw(CPUPPCState *env, target_ulong arg1, target_ulong arg2,
               uint32_t flags)
{
    if (!likely(!(((int32_t)arg1 < (int32_t)arg2 && (flags & 0x10)) ||
                  ((int32_t)arg1 > (int32_t)arg2 && (flags & 0x08)) ||
                  ((int32_t)arg1 == (int32_t)arg2 && (flags & 0x04)) ||
                  ((uint32_t)arg1 < (uint32_t)arg2 && (flags & 0x02)) ||
                  ((uint32_t)arg1 > (uint32_t)arg2 && (flags & 0x01))))) {
        raise_exception_err_ra(env, POWERPC_EXCP_PROGRAM,
                               POWERPC_EXCP_TRAP, GETPC());
    }
}

#if defined(TARGET_PPC64)
void helper_td(CPUPPCState *env, target_ulong arg1, target_ulong arg2,
               uint32_t flags)
{
    if (!likely(!(((int64_t)arg1 < (int64_t)arg2 && (flags & 0x10)) ||
                  ((int64_t)arg1 > (int64_t)arg2 && (flags & 0x08)) ||
                  ((int64_t)arg1 == (int64_t)arg2 && (flags & 0x04)) ||
                  ((uint64_t)arg1 < (uint64_t)arg2 && (flags & 0x02)) ||
                  ((uint64_t)arg1 > (uint64_t)arg2 && (flags & 0x01))))) {
        raise_exception_err_ra(env, POWERPC_EXCP_PROGRAM,
                               POWERPC_EXCP_TRAP, GETPC());
    }
}
#endif
#endif

#if !defined(CONFIG_USER_ONLY)

#ifdef CONFIG_TCG

/* Embedded.Processor Control */
static int dbell2irq(target_ulong rb)
{
    int msg = rb & DBELL_TYPE_MASK;
    int irq = -1;

    switch (msg) {
    case DBELL_TYPE_DBELL:
        irq = PPC_INTERRUPT_DOORBELL;
        break;
    case DBELL_TYPE_DBELL_CRIT:
        irq = PPC_INTERRUPT_CDOORBELL;
        break;
    case DBELL_TYPE_G_DBELL:
    case DBELL_TYPE_G_DBELL_CRIT:
    case DBELL_TYPE_G_DBELL_MC:
        /* XXX implement */
    default:
        break;
    }

    return irq;
}

void helper_msgclr(CPUPPCState *env, target_ulong rb)
{
    int irq = dbell2irq(rb);

    if (irq < 0) {
        return;
    }

    env->pending_interrupts &= ~(1 << irq);
}

void helper_msgsnd(target_ulong rb)
{
    int irq = dbell2irq(rb);
    int pir = rb & DBELL_PIRTAG_MASK;
    CPUState *cs;

    if (irq < 0) {
        return;
    }

    qemu_mutex_lock_iothread();
    CPU_FOREACH(cs) {
        PowerPCCPU *cpu = POWERPC_CPU(cs);
        CPUPPCState *cenv = &cpu->env;

        if ((rb & DBELL_BRDCAST) || (cenv->spr[SPR_BOOKE_PIR] == pir)) {
            cenv->pending_interrupts |= 1 << irq;
            cpu_interrupt(cs, CPU_INTERRUPT_HARD);
        }
    }
    qemu_mutex_unlock_iothread();
}

/* Server Processor Control */

static bool dbell_type_server(target_ulong rb)
{
    /*
     * A Directed Hypervisor Doorbell message is sent only if the
     * message type is 5. All other types are reserved and the
     * instruction is a no-op
     */
    return (rb & DBELL_TYPE_MASK) == DBELL_TYPE_DBELL_SERVER;
}

void helper_book3s_msgclr(CPUPPCState *env, target_ulong rb)
{
    if (!dbell_type_server(rb)) {
        return;
    }

    env->pending_interrupts &= ~(1 << PPC_INTERRUPT_HDOORBELL);
}

static void book3s_msgsnd_common(int pir, int irq)
{
    CPUState *cs;

    qemu_mutex_lock_iothread();
    CPU_FOREACH(cs) {
        PowerPCCPU *cpu = POWERPC_CPU(cs);
        CPUPPCState *cenv = &cpu->env;

        /* TODO: broadcast message to all threads of the same  processor */
        if (cenv->spr_cb[SPR_PIR].default_value == pir) {
            cenv->pending_interrupts |= 1 << irq;
            cpu_interrupt(cs, CPU_INTERRUPT_HARD);
        }
    }
    qemu_mutex_unlock_iothread();
}

void helper_book3s_msgsnd(target_ulong rb)
{
    int pir = rb & DBELL_PROCIDTAG_MASK;

    if (!dbell_type_server(rb)) {
        return;
    }

    book3s_msgsnd_common(pir, PPC_INTERRUPT_HDOORBELL);
}

#if defined(TARGET_PPC64)
void helper_book3s_msgclrp(CPUPPCState *env, target_ulong rb)
{
    helper_hfscr_facility_check(env, HFSCR_MSGP, "msgclrp", HFSCR_IC_MSGP);

    if (!dbell_type_server(rb)) {
        return;
    }

    env->pending_interrupts &= ~(1 << PPC_INTERRUPT_DOORBELL);
}

/*
 * sends a message to other threads that are on the same
 * multi-threaded processor
 */
void helper_book3s_msgsndp(CPUPPCState *env, target_ulong rb)
{
    int pir = env->spr_cb[SPR_PIR].default_value;

    helper_hfscr_facility_check(env, HFSCR_MSGP, "msgsndp", HFSCR_IC_MSGP);

    if (!dbell_type_server(rb)) {
        return;
    }

    /* TODO: TCG supports only one thread */

    book3s_msgsnd_common(pir, PPC_INTERRUPT_DOORBELL);
}
#endif /* TARGET_PPC64 */

void ppc_cpu_do_unaligned_access(CPUState *cs, vaddr vaddr,
                                 MMUAccessType access_type,
                                 int mmu_idx, uintptr_t retaddr)
{
    CPUPPCState *env = cs->env_ptr;
    uint32_t insn;

    /* Restore state and reload the insn we executed, for filling in DSISR.  */
    cpu_restore_state(cs, retaddr, true);
    insn = cpu_ldl_code(env, env->nip);

    switch (env->mmu_model) {
    case POWERPC_MMU_SOFT_4xx:
        env->spr[SPR_40x_DEAR] = vaddr;
        break;
    case POWERPC_MMU_BOOKE:
    case POWERPC_MMU_BOOKE206:
        env->spr[SPR_BOOKE_DEAR] = vaddr;
        break;
    default:
        env->spr[SPR_DAR] = vaddr;
        break;
    }

    cs->exception_index = POWERPC_EXCP_ALIGN;
    env->error_code = insn & 0x03FF0000;
    cpu_loop_exit(cs);
}
#endif /* CONFIG_TCG */
#endif /* !CONFIG_USER_ONLY */
