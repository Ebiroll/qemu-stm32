/*
 * m68k virtual CPU header
 * 
 *  Copyright (c) 2005-2007 CodeSourcery
 *  Written by Paul Brook
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef CPU_M68K_H
#define CPU_M68K_H

#define TARGET_LONG_BITS 32

#include "cpu-defs.h"

#include "softfloat.h"

#define MAX_QREGS 32

#define TARGET_HAS_ICE 1

#define ELF_MACHINE	EM_68K

#define EXCP_ACCESS         2   /* Access (MMU) error.  */
#define EXCP_ADDRESS        3   /* Address error.  */
#define EXCP_ILLEGAL        4   /* Illegal instruction.  */
#define EXCP_DIV0           5   /* Divide by zero */
#define EXCP_PRIVILEGE      8   /* Privilege violation.  */
#define EXCP_TRACE          9
#define EXCP_LINEA          10  /* Unimplemented line-A (MAC) opcode.  */
#define EXCP_LINEF          11  /* Unimplemented line-F (FPU) opcode.  */
#define EXCP_DEBUGNBP       12  /* Non-breakpoint debug interrupt.  */
#define EXCP_DEBEGBP        13  /* Breakpoint debug interrupt.  */
#define EXCP_FORMAT         14  /* RTE format error.  */
#define EXCP_UNINITIALIZED  15
#define EXCP_TRAP0          32   /* User trap #0.  */
#define EXCP_TRAP15         47   /* User trap #15.  */
#define EXCP_UNSUPPORTED    61
#define EXCP_ICE            13

#define EXCP_RTE            0x100
#define EXCP_HALT_INSN      0x101

typedef struct CPUM68KState {
    uint32_t dregs[8];
    uint32_t aregs[8];
    uint32_t pc;
    uint32_t sr;

    /* Condition flags.  */
    uint32_t cc_op;
    uint32_t cc_dest;
    uint32_t cc_src;
    uint32_t cc_x;

    float64 fregs[8];
    float64 fp_result;
    uint32_t fpcr;
    uint32_t fpsr;
    float_status fp_status;

    /* Temporary storage for DIV helpers.  */
    uint32_t div1;
    uint32_t div2;
    
    /* MMU status.  */
    struct {
        uint32_t ar;
    } mmu;

    /* Control registers.  */
    uint32_t vbr;
    uint32_t mbar;
    uint32_t rambar0;

    uint32_t features;

    /* ??? remove this.  */
    uint32_t t1;

    /* exception/interrupt handling */
    jmp_buf jmp_env;
    int exception_index;
    int interrupt_request;
    int user_mode_only;
    int halted;

    int pending_vector;
    int pending_level;

    uint32_t qregs[MAX_QREGS];

    CPU_COMMON
} CPUM68KState;

CPUM68KState *cpu_m68k_init(void);
int cpu_m68k_exec(CPUM68KState *s);
void cpu_m68k_close(CPUM68KState *s);
void do_interrupt(int is_hw);
/* you can call this signal handler from your SIGBUS and SIGSEGV
   signal handlers to inform the virtual CPU of exceptions. non zero
   is returned if the signal was handled by the virtual CPU.  */
int cpu_m68k_signal_handler(int host_signum, void *pinfo, 
                           void *puc);
void cpu_m68k_flush_flags(CPUM68KState *, int);

enum {
    CC_OP_DYNAMIC, /* Use env->cc_op  */
    CC_OP_FLAGS, /* CC_DEST = CVZN, CC_SRC = unused */
    CC_OP_LOGIC, /* CC_DEST = result, CC_SRC = unused */
    CC_OP_ADD,   /* CC_DEST = result, CC_SRC = source */
    CC_OP_SUB,   /* CC_DEST = result, CC_SRC = source */
    CC_OP_CMPB,  /* CC_DEST = result, CC_SRC = source */
    CC_OP_CMPW,  /* CC_DEST = result, CC_SRC = source */
    CC_OP_ADDX,  /* CC_DEST = result, CC_SRC = source */
    CC_OP_SUBX,  /* CC_DEST = result, CC_SRC = source */
    CC_OP_SHL,   /* CC_DEST = source, CC_SRC = shift */
    CC_OP_SHR,   /* CC_DEST = source, CC_SRC = shift */
    CC_OP_SAR,   /* CC_DEST = source, CC_SRC = shift */
};

#define CCF_C 0x01
#define CCF_V 0x02
#define CCF_Z 0x04
#define CCF_N 0x08
#define CCF_X 0x10

#define SR_I_SHIFT 8
#define SR_I  0x0700
#define SR_M  0x1000
#define SR_S  0x2000
#define SR_T  0x8000

typedef struct m68k_def_t m68k_def_t;

int cpu_m68k_set_model(CPUM68KState *env, const char * name);

void m68k_set_irq_level(CPUM68KState *env, int level, uint8_t vector);

#define M68K_FPCR_PREC (1 << 6)

void do_m68k_semihosting(CPUM68KState *env, int nr);

enum m68k_features {
    M68K_FEATURE_CF_ISA_A,
    M68K_FEATURE_CF_ISA_B,
    M68K_FEATURE_CF_ISA_C,
    M68K_FEATURE_CF_FPU,
    M68K_FEATURE_CF_MAC,
    M68K_FEATURE_CF_EMAC,
    M68K_FEATURE_EXT_FULL, /* 68020+ full extension word.  */
    M68K_FEATURE_WORD_INDEX /* word sized address index registers.  */
};

static inline int m68k_feature(CPUM68KState *env, int feature)
{
    return (env->features & (1u << feature)) != 0;
}

void register_m68k_insns (CPUM68KState *env);

#ifdef CONFIG_USER_ONLY
/* Linux uses 8k pages.  */
#define TARGET_PAGE_BITS 13
#else
/* Smallest TLB entry size is 1k.  */ 
#define TARGET_PAGE_BITS 10
#endif
#include "cpu-all.h"

#endif
