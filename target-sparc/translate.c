/*
   SPARC translation

   Copyright (C) 2003 Thomas M. Ogrisegg <tom@fnord.at>
   Copyright (C) 2003 Fabrice Bellard

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
   TODO-list:

   NPC/PC static optimisations (use JUMP_TB when possible)
   FPU-Instructions
   Privileged instructions
   Coprocessor-Instructions
   Optimize synthetic instructions
   Optional alignment and privileged instruction check
*/

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "cpu.h"
#include "exec-all.h"
#include "disas.h"

#define DEBUG_DISAS

#define DYNAMIC_PC  1 /* dynamic pc value */
#define JUMP_PC     2 /* dynamic pc value which takes only two values
                         according to jump_pc[T2] */

typedef struct DisasContext {
    target_ulong pc;	/* current Program Counter: integer or DYNAMIC_PC */
    target_ulong npc;	/* next PC: integer or DYNAMIC_PC or JUMP_PC */
    target_ulong jump_pc[2]; /* used when JUMP_PC pc value is used */
    int is_br;
    int mem_idx;
    struct TranslationBlock *tb;
} DisasContext;

static uint16_t *gen_opc_ptr;
static uint32_t *gen_opparam_ptr;
extern FILE *logfile;
extern int loglevel;

enum {
#define DEF(s,n,copy_size) INDEX_op_ ## s,
#include "opc.h"
#undef DEF
    NB_OPS
};

#include "gen-op.h"

#define GET_FIELD(X, FROM, TO) \
  ((X) >> (31 - (TO)) & ((1 << ((TO) - (FROM) + 1)) - 1))

#define IS_IMM (insn & (1<<13))

static void disas_sparc_insn(DisasContext * dc);

static GenOpFunc *gen_op_movl_TN_reg[2][32] = {
    {
     gen_op_movl_g0_T0,
     gen_op_movl_g1_T0,
     gen_op_movl_g2_T0,
     gen_op_movl_g3_T0,
     gen_op_movl_g4_T0,
     gen_op_movl_g5_T0,
     gen_op_movl_g6_T0,
     gen_op_movl_g7_T0,
     gen_op_movl_o0_T0,
     gen_op_movl_o1_T0,
     gen_op_movl_o2_T0,
     gen_op_movl_o3_T0,
     gen_op_movl_o4_T0,
     gen_op_movl_o5_T0,
     gen_op_movl_o6_T0,
     gen_op_movl_o7_T0,
     gen_op_movl_l0_T0,
     gen_op_movl_l1_T0,
     gen_op_movl_l2_T0,
     gen_op_movl_l3_T0,
     gen_op_movl_l4_T0,
     gen_op_movl_l5_T0,
     gen_op_movl_l6_T0,
     gen_op_movl_l7_T0,
     gen_op_movl_i0_T0,
     gen_op_movl_i1_T0,
     gen_op_movl_i2_T0,
     gen_op_movl_i3_T0,
     gen_op_movl_i4_T0,
     gen_op_movl_i5_T0,
     gen_op_movl_i6_T0,
     gen_op_movl_i7_T0,
     },
    {
     gen_op_movl_g0_T1,
     gen_op_movl_g1_T1,
     gen_op_movl_g2_T1,
     gen_op_movl_g3_T1,
     gen_op_movl_g4_T1,
     gen_op_movl_g5_T1,
     gen_op_movl_g6_T1,
     gen_op_movl_g7_T1,
     gen_op_movl_o0_T1,
     gen_op_movl_o1_T1,
     gen_op_movl_o2_T1,
     gen_op_movl_o3_T1,
     gen_op_movl_o4_T1,
     gen_op_movl_o5_T1,
     gen_op_movl_o6_T1,
     gen_op_movl_o7_T1,
     gen_op_movl_l0_T1,
     gen_op_movl_l1_T1,
     gen_op_movl_l2_T1,
     gen_op_movl_l3_T1,
     gen_op_movl_l4_T1,
     gen_op_movl_l5_T1,
     gen_op_movl_l6_T1,
     gen_op_movl_l7_T1,
     gen_op_movl_i0_T1,
     gen_op_movl_i1_T1,
     gen_op_movl_i2_T1,
     gen_op_movl_i3_T1,
     gen_op_movl_i4_T1,
     gen_op_movl_i5_T1,
     gen_op_movl_i6_T1,
     gen_op_movl_i7_T1,
     }
};

static GenOpFunc *gen_op_movl_reg_TN[3][32] = {
    {
     gen_op_movl_T0_g0,
     gen_op_movl_T0_g1,
     gen_op_movl_T0_g2,
     gen_op_movl_T0_g3,
     gen_op_movl_T0_g4,
     gen_op_movl_T0_g5,
     gen_op_movl_T0_g6,
     gen_op_movl_T0_g7,
     gen_op_movl_T0_o0,
     gen_op_movl_T0_o1,
     gen_op_movl_T0_o2,
     gen_op_movl_T0_o3,
     gen_op_movl_T0_o4,
     gen_op_movl_T0_o5,
     gen_op_movl_T0_o6,
     gen_op_movl_T0_o7,
     gen_op_movl_T0_l0,
     gen_op_movl_T0_l1,
     gen_op_movl_T0_l2,
     gen_op_movl_T0_l3,
     gen_op_movl_T0_l4,
     gen_op_movl_T0_l5,
     gen_op_movl_T0_l6,
     gen_op_movl_T0_l7,
     gen_op_movl_T0_i0,
     gen_op_movl_T0_i1,
     gen_op_movl_T0_i2,
     gen_op_movl_T0_i3,
     gen_op_movl_T0_i4,
     gen_op_movl_T0_i5,
     gen_op_movl_T0_i6,
     gen_op_movl_T0_i7,
     },
    {
     gen_op_movl_T1_g0,
     gen_op_movl_T1_g1,
     gen_op_movl_T1_g2,
     gen_op_movl_T1_g3,
     gen_op_movl_T1_g4,
     gen_op_movl_T1_g5,
     gen_op_movl_T1_g6,
     gen_op_movl_T1_g7,
     gen_op_movl_T1_o0,
     gen_op_movl_T1_o1,
     gen_op_movl_T1_o2,
     gen_op_movl_T1_o3,
     gen_op_movl_T1_o4,
     gen_op_movl_T1_o5,
     gen_op_movl_T1_o6,
     gen_op_movl_T1_o7,
     gen_op_movl_T1_l0,
     gen_op_movl_T1_l1,
     gen_op_movl_T1_l2,
     gen_op_movl_T1_l3,
     gen_op_movl_T1_l4,
     gen_op_movl_T1_l5,
     gen_op_movl_T1_l6,
     gen_op_movl_T1_l7,
     gen_op_movl_T1_i0,
     gen_op_movl_T1_i1,
     gen_op_movl_T1_i2,
     gen_op_movl_T1_i3,
     gen_op_movl_T1_i4,
     gen_op_movl_T1_i5,
     gen_op_movl_T1_i6,
     gen_op_movl_T1_i7,
     },
    {
     gen_op_movl_T2_g0,
     gen_op_movl_T2_g1,
     gen_op_movl_T2_g2,
     gen_op_movl_T2_g3,
     gen_op_movl_T2_g4,
     gen_op_movl_T2_g5,
     gen_op_movl_T2_g6,
     gen_op_movl_T2_g7,
     gen_op_movl_T2_o0,
     gen_op_movl_T2_o1,
     gen_op_movl_T2_o2,
     gen_op_movl_T2_o3,
     gen_op_movl_T2_o4,
     gen_op_movl_T2_o5,
     gen_op_movl_T2_o6,
     gen_op_movl_T2_o7,
     gen_op_movl_T2_l0,
     gen_op_movl_T2_l1,
     gen_op_movl_T2_l2,
     gen_op_movl_T2_l3,
     gen_op_movl_T2_l4,
     gen_op_movl_T2_l5,
     gen_op_movl_T2_l6,
     gen_op_movl_T2_l7,
     gen_op_movl_T2_i0,
     gen_op_movl_T2_i1,
     gen_op_movl_T2_i2,
     gen_op_movl_T2_i3,
     gen_op_movl_T2_i4,
     gen_op_movl_T2_i5,
     gen_op_movl_T2_i6,
     gen_op_movl_T2_i7,
     }
};

static GenOpFunc1 *gen_op_movl_TN_im[3] = {
    gen_op_movl_T0_im,
    gen_op_movl_T1_im,
    gen_op_movl_T2_im
};

#define GEN32(func, NAME) \
static GenOpFunc *NAME ## _table [32] = {                                     \
NAME ## 0, NAME ## 1, NAME ## 2, NAME ## 3,                                   \
NAME ## 4, NAME ## 5, NAME ## 6, NAME ## 7,                                   \
NAME ## 8, NAME ## 9, NAME ## 10, NAME ## 11,                                 \
NAME ## 12, NAME ## 13, NAME ## 14, NAME ## 15,                               \
NAME ## 16, NAME ## 17, NAME ## 18, NAME ## 19,                               \
NAME ## 20, NAME ## 21, NAME ## 22, NAME ## 23,                               \
NAME ## 24, NAME ## 25, NAME ## 26, NAME ## 27,                               \
NAME ## 28, NAME ## 29, NAME ## 30, NAME ## 31,                               \
};                                                                            \
static inline void func(int n)                                                \
{                                                                             \
    NAME ## _table[n]();                                                      \
}

/* floating point registers moves */
GEN32(gen_op_load_fpr_FT0, gen_op_load_fpr_FT0_fprf);
GEN32(gen_op_load_fpr_FT1, gen_op_load_fpr_FT1_fprf);
GEN32(gen_op_load_fpr_FT2, gen_op_load_fpr_FT2_fprf);
GEN32(gen_op_store_FT0_fpr, gen_op_store_FT0_fpr_fprf);
GEN32(gen_op_store_FT1_fpr, gen_op_store_FT1_fpr_fprf);
GEN32(gen_op_store_FT2_fpr, gen_op_store_FT2_fpr_fprf);

GEN32(gen_op_load_fpr_DT0, gen_op_load_fpr_DT0_fprf);
GEN32(gen_op_load_fpr_DT1, gen_op_load_fpr_DT1_fprf);
GEN32(gen_op_load_fpr_DT2, gen_op_load_fpr_DT2_fprf);
GEN32(gen_op_store_DT0_fpr, gen_op_store_DT0_fpr_fprf);
GEN32(gen_op_store_DT1_fpr, gen_op_store_DT1_fpr_fprf);
GEN32(gen_op_store_DT2_fpr, gen_op_store_DT2_fpr_fprf);

#if defined(CONFIG_USER_ONLY)
#define gen_op_ldst(name)        gen_op_##name##_raw()
#define OP_LD_TABLE(width)
#define supervisor(dc) 0
#else
#define gen_op_ldst(name)        (*gen_op_##name[dc->mem_idx])()
#define OP_LD_TABLE(width)						      \
static GenOpFunc *gen_op_##width[] = {                                        \
    &gen_op_##width##_user,                                                   \
    &gen_op_##width##_kernel,                                                 \
};                                                                            \
                                                                              \
static void gen_op_##width##a(int insn, int is_ld, int size, int sign)        \
{                                                                             \
    int asi;                                                                  \
                                                                              \
    asi = GET_FIELD(insn, 19, 26);                                            \
    switch (asi) {                                                            \
	case 10: /* User data access */                                       \
	    gen_op_##width##_user();                                          \
	    break;                                                            \
	case 11: /* Supervisor data access */                                 \
	    gen_op_##width##_kernel();                                        \
	    break;                                                            \
        case 0x20 ... 0x2f: /* MMU passthrough */			      \
	    if (is_ld)                                                        \
		gen_op_ld_asi(asi, size, sign);				      \
	    else                                                              \
		gen_op_st_asi(asi, size, sign);				      \
	    break;                                                            \
	default:                                                              \
	    if (is_ld)                                                        \
		gen_op_ld_asi(asi, size, sign);			              \
	    else                                                              \
		gen_op_st_asi(asi, size, sign);				      \
            break;                                                            \
    }                                                                         \
}

#define supervisor(dc) (dc->mem_idx == 1)
#endif

OP_LD_TABLE(ld);
OP_LD_TABLE(st);
OP_LD_TABLE(ldub);
OP_LD_TABLE(lduh);
OP_LD_TABLE(ldsb);
OP_LD_TABLE(ldsh);
OP_LD_TABLE(stb);
OP_LD_TABLE(sth);
OP_LD_TABLE(std);
OP_LD_TABLE(ldstub);
OP_LD_TABLE(swap);
OP_LD_TABLE(ldd);
OP_LD_TABLE(stf);
OP_LD_TABLE(stdf);
OP_LD_TABLE(ldf);
OP_LD_TABLE(lddf);

static inline void gen_movl_imm_TN(int reg, int imm)
{
    gen_op_movl_TN_im[reg] (imm);
}

static inline void gen_movl_imm_T1(int val)
{
    gen_movl_imm_TN(1, val);
}

static inline void gen_movl_imm_T0(int val)
{
    gen_movl_imm_TN(0, val);
}

static inline void gen_movl_reg_TN(int reg, int t)
{
    if (reg)
	gen_op_movl_reg_TN[t][reg] ();
    else
	gen_movl_imm_TN(t, 0);
}

static inline void gen_movl_reg_T0(int reg)
{
    gen_movl_reg_TN(reg, 0);
}

static inline void gen_movl_reg_T1(int reg)
{
    gen_movl_reg_TN(reg, 1);
}

static inline void gen_movl_reg_T2(int reg)
{
    gen_movl_reg_TN(reg, 2);
}

static inline void gen_movl_TN_reg(int reg, int t)
{
    if (reg)
	gen_op_movl_TN_reg[t][reg] ();
}

static inline void gen_movl_T0_reg(int reg)
{
    gen_movl_TN_reg(reg, 0);
}

static inline void gen_movl_T1_reg(int reg)
{
    gen_movl_TN_reg(reg, 1);
}

/* call this function before using T2 as it may have been set for a jump */
static inline void flush_T2(DisasContext * dc)
{
    if (dc->npc == JUMP_PC) {
        gen_op_generic_branch(dc->jump_pc[0], dc->jump_pc[1]);
        dc->npc = DYNAMIC_PC;
    }
}

static inline void save_npc(DisasContext * dc)
{
    if (dc->npc == JUMP_PC) {
        gen_op_generic_branch(dc->jump_pc[0], dc->jump_pc[1]);
        dc->npc = DYNAMIC_PC;
    } else if (dc->npc != DYNAMIC_PC) {
        gen_op_movl_npc_im(dc->npc);
    }
}

static inline void save_state(DisasContext * dc)
{
    gen_op_jmp_im((uint32_t)dc->pc);
    save_npc(dc);
}

static void gen_cond(int cond)
{
	switch (cond) {
        case 0x0:
            gen_op_movl_T2_0();
            break;
	case 0x1:
	    gen_op_eval_be();
	    break;
	case 0x2:
	    gen_op_eval_ble();
	    break;
	case 0x3:
	    gen_op_eval_bl();
	    break;
	case 0x4:
	    gen_op_eval_bleu();
	    break;
	case 0x5:
	    gen_op_eval_bcs();
	    break;
	case 0x6:
	    gen_op_eval_bneg();
	    break;
	case 0x7:
	    gen_op_eval_bvs();
	    break;
        case 0x8:
            gen_op_movl_T2_1();
            break;
	case 0x9:
	    gen_op_eval_bne();
	    break;
	case 0xa:
	    gen_op_eval_bg();
	    break;
	case 0xb:
	    gen_op_eval_bge();
	    break;
	case 0xc:
	    gen_op_eval_bgu();
	    break;
	case 0xd:
	    gen_op_eval_bcc();
	    break;
	case 0xe:
	    gen_op_eval_bpos();
	    break;
        default:
	case 0xf:
	    gen_op_eval_bvc();
	    break;
	}
}

static void gen_fcond(int cond)
{
	switch (cond) {
        case 0x0:
            gen_op_movl_T2_0();
            break;
	case 0x1:
	    gen_op_eval_fbne();
	    break;
	case 0x2:
	    gen_op_eval_fblg();
	    break;
	case 0x3:
	    gen_op_eval_fbul();
	    break;
	case 0x4:
	    gen_op_eval_fbl();
	    break;
	case 0x5:
	    gen_op_eval_fbug();
	    break;
	case 0x6:
	    gen_op_eval_fbg();
	    break;
	case 0x7:
	    gen_op_eval_fbu();
	    break;
        case 0x8:
            gen_op_movl_T2_1();
            break;
	case 0x9:
	    gen_op_eval_fbe();
	    break;
	case 0xa:
	    gen_op_eval_fbue();
	    break;
	case 0xb:
	    gen_op_eval_fbge();
	    break;
	case 0xc:
	    gen_op_eval_fbuge();
	    break;
	case 0xd:
	    gen_op_eval_fble();
	    break;
	case 0xe:
	    gen_op_eval_fbule();
	    break;
        default:
	case 0xf:
	    gen_op_eval_fbo();
	    break;
	}
}

static void do_branch(DisasContext * dc, uint32_t target, uint32_t insn)
{
    unsigned int cond = GET_FIELD(insn, 3, 6), a = (insn & (1 << 29));
    target += (uint32_t) dc->pc;
    if (cond == 0x0) {
	/* unconditional not taken */
	if (a) {
	    dc->pc = dc->npc + 4;
	    dc->npc = dc->pc + 4;
	} else {
	    dc->pc = dc->npc;
	    dc->npc = dc->pc + 4;
	}
    } else if (cond == 0x8) {
	/* unconditional taken */
	if (a) {
	    dc->pc = target;
	    dc->npc = dc->pc + 4;
	} else {
	    dc->pc = dc->npc;
	    dc->npc = target;
	}
    } else {
        flush_T2(dc);
        gen_cond(cond);
	if (a) {
	    gen_op_branch_a((long)dc->tb, target, dc->npc);
            dc->is_br = 1;
	} else {
            dc->pc = dc->npc;
            dc->jump_pc[0] = target;
            dc->jump_pc[1] = dc->npc + 4;
            dc->npc = JUMP_PC;
	}
    }
}

static void do_fbranch(DisasContext * dc, uint32_t target, uint32_t insn)
{
    unsigned int cond = GET_FIELD(insn, 3, 6), a = (insn & (1 << 29));
    target += (uint32_t) dc->pc;
    if (cond == 0x0) {
	/* unconditional not taken */
	if (a) {
	    dc->pc = dc->npc + 4;
	    dc->npc = dc->pc + 4;
	} else {
	    dc->pc = dc->npc;
	    dc->npc = dc->pc + 4;
	}
    } else if (cond == 0x8) {
	/* unconditional taken */
	if (a) {
	    dc->pc = target;
	    dc->npc = dc->pc + 4;
	} else {
	    dc->pc = dc->npc;
	    dc->npc = target;
	}
    } else {
        flush_T2(dc);
        gen_fcond(cond);
	if (a) {
	    gen_op_branch_a((long)dc->tb, target, dc->npc);
            dc->is_br = 1;
	} else {
            dc->pc = dc->npc;
            dc->jump_pc[0] = target;
            dc->jump_pc[1] = dc->npc + 4;
            dc->npc = JUMP_PC;
	}
    }
}

static void gen_debug(DisasContext *s, uint32_t pc)
{
    gen_op_jmp_im(pc);
    gen_op_debug();
    s->is_br = 1;
}

#define GET_FIELDs(x,a,b) sign_extend (GET_FIELD(x,a,b), (b) - (a) + 1)

static int sign_extend(int x, int len)
{
    len = 32 - len;
    return (x << len) >> len;
}

static void disas_sparc_insn(DisasContext * dc)
{
    unsigned int insn, opc, rs1, rs2, rd;

    insn = ldl_code((uint8_t *)dc->pc);
    opc = GET_FIELD(insn, 0, 1);

    rd = GET_FIELD(insn, 2, 6);
    switch (opc) {
    case 0:			/* branches/sethi */
	{
	    unsigned int xop = GET_FIELD(insn, 7, 9);
	    int target;
	    target = GET_FIELD(insn, 10, 31);
	    switch (xop) {
	    case 0x0:
	    case 0x1:		/* UNIMPL */
	    default:
                goto illegal_insn;
	    case 0x2:		/* BN+x */
		{
		    target <<= 2;
		    target = sign_extend(target, 22);
		    do_branch(dc, target, insn);
		    goto jmp_insn;
		}
	    case 0x6:		/* FBN+x */
		{
		    target <<= 2;
		    target = sign_extend(target, 22);
		    do_fbranch(dc, target, insn);
		    goto jmp_insn;
		}
	    case 0x4:		/* SETHI */
		gen_movl_imm_T0(target << 10);
		gen_movl_T0_reg(rd);
		break;
	    case 0x5:		/*CBN+x */
		break;
	    }
	    break;
	}
    case 1:
	/*CALL*/ {
	    unsigned int target = GET_FIELDs(insn, 2, 31) << 2;

	    gen_op_movl_T0_im((long) (dc->pc));
	    gen_movl_T0_reg(15);
	    target = dc->pc + target;
	    dc->pc = dc->npc;
	    dc->npc = target;
	}
	goto jmp_insn;
    case 2:			/* FPU & Logical Operations */
	{
	    unsigned int xop = GET_FIELD(insn, 7, 12);
	    if (xop == 0x3a) {	/* generate trap */
                int cond;
                rs1 = GET_FIELD(insn, 13, 17);
                gen_movl_reg_T0(rs1);
		if (IS_IMM) {
		    rs2 = GET_FIELD(insn, 25, 31);
		    if (rs2 != 0) {
			 gen_movl_imm_T1(rs2);
			 gen_op_add_T1_T0();
		    }
                } else {
                    rs2 = GET_FIELD(insn, 27, 31);
                    gen_movl_reg_T1(rs2);
                    gen_op_add_T1_T0();
                }
                save_state(dc);
                cond = GET_FIELD(insn, 3, 6);
                if (cond == 0x8) {
                    gen_op_trap_T0();
                    dc->is_br = 1;
                    goto jmp_insn;
                } else {
                    gen_op_trapcc_T0();
                }
            } else if (xop == 0x28) {
                rs1 = GET_FIELD(insn, 13, 17);
                switch(rs1) {
                case 0: /* rdy */
                    gen_op_rdy();
                    gen_movl_T0_reg(rd);
                    break;
                case 15: /* stbar */
		    break; /* no effect? */
                default:
                    goto illegal_insn;
                }
#if !defined(CONFIG_USER_ONLY)
            } else if (xop == 0x29) {
		if (!supervisor(dc))
		    goto priv_insn;
                gen_op_rdpsr();
                gen_movl_T0_reg(rd);
                break;
            } else if (xop == 0x2a) {
		if (!supervisor(dc))
		    goto priv_insn;
                gen_op_rdwim();
                gen_movl_T0_reg(rd);
                break;
            } else if (xop == 0x2b) {
		if (!supervisor(dc))
		    goto priv_insn;
                gen_op_rdtbr();
                gen_movl_T0_reg(rd);
                break;
#endif
	    } else if (xop == 0x34 || xop == 0x35) {	/* FPU Operations */
                rs1 = GET_FIELD(insn, 13, 17);
	        rs2 = GET_FIELD(insn, 27, 31);
	        xop = GET_FIELD(insn, 18, 26);
		switch (xop) {
		    case 0x1: /* fmovs */
                	gen_op_load_fpr_FT0(rs2);
			gen_op_store_FT0_fpr(rd);
			break;
		    case 0x5: /* fnegs */
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fnegs();
			gen_op_store_FT0_fpr(rd);
			break;
		    case 0x9: /* fabss */
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fabss();
			gen_op_store_FT0_fpr(rd);
			break;
		    case 0x29: /* fsqrts */
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fsqrts();
			gen_op_store_FT0_fpr(rd);
			break;
		    case 0x2a: /* fsqrtd */
                	gen_op_load_fpr_DT1(rs2);
			gen_op_fsqrtd();
			gen_op_store_DT0_fpr(rd);
			break;
		    case 0x41:
                	gen_op_load_fpr_FT0(rs1);
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fadds();
			gen_op_store_FT0_fpr(rd);
			break;
		    case 0x42:
                	gen_op_load_fpr_DT0(rs1);
                	gen_op_load_fpr_DT1(rs2);
			gen_op_faddd();
			gen_op_store_DT0_fpr(rd);
			break;
		    case 0x45:
                	gen_op_load_fpr_FT0(rs1);
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fsubs();
			gen_op_store_FT0_fpr(rd);
			break;
		    case 0x46:
                	gen_op_load_fpr_DT0(rs1);
                	gen_op_load_fpr_DT1(rs2);
			gen_op_fsubd();
			gen_op_store_DT0_fpr(rd);
			break;
		    case 0x49:
                	gen_op_load_fpr_FT0(rs1);
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fmuls();
			gen_op_store_FT0_fpr(rd);
			break;
		    case 0x4a:
                	gen_op_load_fpr_DT0(rs1);
                	gen_op_load_fpr_DT1(rs2);
			gen_op_fmuld();
			gen_op_store_DT0_fpr(rd);
			break;
		    case 0x4d:
                	gen_op_load_fpr_FT0(rs1);
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fdivs();
			gen_op_store_FT0_fpr(rd);
			break;
		    case 0x4e:
                	gen_op_load_fpr_DT0(rs1);
                	gen_op_load_fpr_DT1(rs2);
			gen_op_fdivd();
			gen_op_store_DT0_fpr(rd);
			break;
		    case 0x51:
                	gen_op_load_fpr_FT0(rs1);
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fcmps();
			break;
		    case 0x52:
                	gen_op_load_fpr_DT0(rs1);
                	gen_op_load_fpr_DT1(rs2);
			gen_op_fcmpd();
			break;
		    case 0x55: /* fcmpes */
                	gen_op_load_fpr_FT0(rs1);
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fcmps(); /* XXX */
			break;
		    case 0x56: /* fcmped */
                	gen_op_load_fpr_DT0(rs1);
                	gen_op_load_fpr_DT1(rs2);
			gen_op_fcmpd(); /* XXX */
			break;
		    case 0x69:
                	gen_op_load_fpr_FT0(rs1);
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fsmuld();
			gen_op_store_DT0_fpr(rd);
			break;
		    case 0xc4:
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fitos();
			gen_op_store_FT0_fpr(rd);
			break;
		    case 0xc6:
                	gen_op_load_fpr_DT1(rs2);
			gen_op_fdtos();
			gen_op_store_FT0_fpr(rd);
			break;
		    case 0xc8:
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fitod();
			gen_op_store_DT0_fpr(rd);
			break;
		    case 0xc9:
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fstod();
			gen_op_store_DT0_fpr(rd);
			break;
		    case 0xd1:
                	gen_op_load_fpr_FT1(rs2);
			gen_op_fstoi();
			gen_op_store_FT0_fpr(rd);
			break;
		    case 0xd2:
                	gen_op_load_fpr_DT1(rs2);
			gen_op_fdtoi();
			gen_op_store_FT0_fpr(rd);
			break;
		    default:
                	goto illegal_insn;
		}
	    } else {
                rs1 = GET_FIELD(insn, 13, 17);
                gen_movl_reg_T0(rs1);
                if (IS_IMM) {	/* immediate */
                    rs2 = GET_FIELDs(insn, 19, 31);
                    gen_movl_imm_T1(rs2);
                } else {		/* register */
                    rs2 = GET_FIELD(insn, 27, 31);
                    gen_movl_reg_T1(rs2);
                }
                if (xop < 0x20) {
                    switch (xop & ~0x10) {
                    case 0x0:
                        if (xop & 0x10)
                            gen_op_add_T1_T0_cc();
                        else
                            gen_op_add_T1_T0();
                        break;
                    case 0x1:
                        gen_op_and_T1_T0();
                        if (xop & 0x10)
                            gen_op_logic_T0_cc();
                        break;
                    case 0x2:
                        gen_op_or_T1_T0();
                        if (xop & 0x10)
                            gen_op_logic_T0_cc();
                        break;
                    case 0x3:
                        gen_op_xor_T1_T0();
                        if (xop & 0x10)
                            gen_op_logic_T0_cc();
                        break;
                    case 0x4:
                        if (xop & 0x10)
                            gen_op_sub_T1_T0_cc();
                        else
                            gen_op_sub_T1_T0();
                        break;
                    case 0x5:
                        gen_op_andn_T1_T0();
                        if (xop & 0x10)
                            gen_op_logic_T0_cc();
                        break;
                    case 0x6:
                        gen_op_orn_T1_T0();
                        if (xop & 0x10)
                            gen_op_logic_T0_cc();
                        break;
                    case 0x7:
                        gen_op_xnor_T1_T0();
                        if (xop & 0x10)
                            gen_op_logic_T0_cc();
                        break;
                    case 0x8:
                        gen_op_addx_T1_T0();
                        if (xop & 0x10)
                            gen_op_set_flags();
                        break;
                    case 0xa:
                        gen_op_umul_T1_T0();
                        if (xop & 0x10)
                            gen_op_logic_T0_cc();
                        break;
                    case 0xb:
                        gen_op_smul_T1_T0();
                        if (xop & 0x10)
                            gen_op_logic_T0_cc();
                        break;
                    case 0xc:
                        gen_op_subx_T1_T0();
                        if (xop & 0x10)
                            gen_op_set_flags();
                        break;
                    case 0xe:
                        gen_op_udiv_T1_T0();
                        if (xop & 0x10)
                            gen_op_div_cc();
                        break;
                    case 0xf:
                        gen_op_sdiv_T1_T0();
                        if (xop & 0x10)
                            gen_op_div_cc();
                        break;
                    default:
                        goto illegal_insn;
                    }
                    gen_movl_T0_reg(rd);
                } else {
                    switch (xop) {
                    case 0x24: /* mulscc */
                        gen_op_mulscc_T1_T0();
                        gen_movl_T0_reg(rd);
                        break;
                    case 0x25:	/* SLL */
                        gen_op_sll();
                        gen_movl_T0_reg(rd);
                        break;
                    case 0x26:
                        gen_op_srl();
                        gen_movl_T0_reg(rd);
                        break;
                    case 0x27:
                        gen_op_sra();
                        gen_movl_T0_reg(rd);
                        break;
                    case 0x30:
                        {
                            gen_op_xor_T1_T0();
                            switch(rd) {
                            case 0:
                                gen_op_wry();
                                break;
                            default:
                                goto illegal_insn;
                            }
                        }
                        break;
#if !defined(CONFIG_USER_ONLY)
                    case 0x31:
                        {
			    if (!supervisor(dc))
				goto priv_insn;
                            gen_op_xor_T1_T0();
                            gen_op_wrpsr();
                        }
                        break;
                    case 0x32:
                        {
			    if (!supervisor(dc))
				goto priv_insn;
                            gen_op_xor_T1_T0();
                            gen_op_wrwim();
                        }
                        break;
                    case 0x33:
                        {
			    if (!supervisor(dc))
				goto priv_insn;
                            gen_op_xor_T1_T0();
                            gen_op_wrtbr();
                        }
                        break;
#endif
                    case 0x38:	/* jmpl */
                        {
                            gen_op_add_T1_T0();
                            gen_op_movl_npc_T0();
                            if (rd != 0) {
                                gen_op_movl_T0_im((long) (dc->pc));
                                gen_movl_T0_reg(rd);
                            }
                            dc->pc = dc->npc;
                            dc->npc = DYNAMIC_PC;
                        }
                        goto jmp_insn;
#if !defined(CONFIG_USER_ONLY)
                    case 0x39:	/* rett */
                        {
			    if (!supervisor(dc))
				goto priv_insn;
                            gen_op_add_T1_T0();
                            gen_op_movl_npc_T0();
                            gen_op_rett();
#if 0
			    dc->pc = dc->npc;
			    dc->npc = DYNAMIC_PC;
#endif
                        }
#if 0
                        goto jmp_insn;
#endif
			break;
#endif
                    case 0x3b: /* flush */
                        gen_op_add_T1_T0();
                        gen_op_flush_T0();
                        break;
                    case 0x3c:	/* save */
                        save_state(dc);
                        gen_op_add_T1_T0();
                        gen_op_save();
                        gen_movl_T0_reg(rd);
                        break;
                    case 0x3d:	/* restore */
                        save_state(dc);
                        gen_op_add_T1_T0();
                        gen_op_restore();
                        gen_movl_T0_reg(rd);
                        break;
                    default:
                        goto illegal_insn;
                    }
                }
            }
	    break;
	}
    case 3:			/* load/store instructions */
	{
	    unsigned int xop = GET_FIELD(insn, 7, 12);
	    rs1 = GET_FIELD(insn, 13, 17);
	    gen_movl_reg_T0(rs1);
	    if (IS_IMM) {	/* immediate */
		rs2 = GET_FIELDs(insn, 19, 31);
		if (rs2 != 0) {
		    gen_movl_imm_T1(rs2);
		    gen_op_add_T1_T0();
		}
	    } else {		/* register */
		rs2 = GET_FIELD(insn, 27, 31);
		gen_movl_reg_T1(rs2);
	        gen_op_add_T1_T0();
	    }
	    if (xop < 4 || (xop > 7 && xop < 0x14) || \
		    (xop > 0x17 && xop < 0x20)) {
		switch (xop) {
		case 0x0:	/* load word */
		    gen_op_ldst(ld);
		    break;
		case 0x1:	/* load unsigned byte */
		    gen_op_ldst(ldub);
		    break;
		case 0x2:	/* load unsigned halfword */
		    gen_op_ldst(lduh);
		    break;
		case 0x3:	/* load double word */
		    gen_op_ldst(ldd);
		    gen_movl_T0_reg(rd + 1);
		    break;
		case 0x9:	/* load signed byte */
		    gen_op_ldst(ldsb);
		    break;
		case 0xa:	/* load signed halfword */
		    gen_op_ldst(ldsh);
		    break;
		case 0xd:	/* ldstub -- XXX: should be atomically */
		    gen_op_ldst(ldstub);
		    break;
		case 0x0f:	/* swap register with memory. Also atomically */
		    gen_op_ldst(swap);
		    break;
		case 0x10:	/* load word alternate */
		    if (!supervisor(dc))
			goto priv_insn;
		    gen_op_lda(insn, 1, 4, 0);
		    break;
		case 0x11:	/* load unsigned byte alternate */
		    if (!supervisor(dc))
			goto priv_insn;
		    gen_op_lduba(insn, 1, 1, 0);
		    break;
		case 0x12:	/* load unsigned halfword alternate */
		    if (!supervisor(dc))
			goto priv_insn;
		    gen_op_lduha(insn, 1, 2, 0);
		    break;
		case 0x13:	/* load double word alternate */
		    if (!supervisor(dc))
			goto priv_insn;
		    gen_op_ldda(insn, 1, 8, 0);
		    gen_movl_T0_reg(rd + 1);
		    break;
		case 0x19:	/* load signed byte alternate */
		    if (!supervisor(dc))
			goto priv_insn;
		    gen_op_ldsba(insn, 1, 1, 1);
		    break;
		case 0x1a:	/* load signed halfword alternate */
		    if (!supervisor(dc))
			goto priv_insn;
		    gen_op_ldsha(insn, 1, 2 ,1);
		    break;
		case 0x1d:	/* ldstuba -- XXX: should be atomically */
		    if (!supervisor(dc))
			goto priv_insn;
		    gen_op_ldstuba(insn, 1, 1, 0);
		    break;
		case 0x1f:	/* swap reg with alt. memory. Also atomically */
		    if (!supervisor(dc))
			goto priv_insn;
		    gen_op_swapa(insn, 1, 4, 0);
		    break;
		}
		gen_movl_T1_reg(rd);
	    } else if (xop >= 0x20 && xop < 0x24) {
		switch (xop) {
		case 0x20:	/* load fpreg */
		    gen_op_ldst(ldf);
		    gen_op_store_FT0_fpr(rd);
		    break;
		case 0x21:	/* load fsr */
		    gen_op_ldfsr();
		    break;
		case 0x23:	/* load double fpreg */
		    gen_op_ldst(lddf);
		    gen_op_store_DT0_fpr(rd);
		    break;
		}
	    } else if (xop < 8 || (xop >= 0x14 && xop < 0x18)) {
		gen_movl_reg_T1(rd);
		switch (xop) {
		case 0x4:
		    gen_op_ldst(st);
		    break;
		case 0x5:
		    gen_op_ldst(stb);
		    break;
		case 0x6:
		    gen_op_ldst(sth);
		    break;
		case 0x7:
                    flush_T2(dc);
		    gen_movl_reg_T2(rd + 1);
		    gen_op_ldst(std);
		    break;
		case 0x14:
		    if (!supervisor(dc))
			goto priv_insn;
		    gen_op_sta(insn, 0, 4, 0);
		    break;
		case 0x15:
		    if (!supervisor(dc))
			goto priv_insn;
		    gen_op_stba(insn, 0, 1, 0);
		    break;
		case 0x16:
		    if (!supervisor(dc))
			goto priv_insn;
		    gen_op_stha(insn, 0, 2, 0);
		    break;
		case 0x17:
		    if (!supervisor(dc))
			goto priv_insn;
                    flush_T2(dc);
		    gen_movl_reg_T2(rd + 1);
		    gen_op_stda(insn, 0, 8, 0);
		    break;
		}
	    } else if (xop > 0x23 && xop < 0x28) {
		switch (xop) {
		case 0x24:
                    gen_op_load_fpr_FT0(rd);
		    gen_op_ldst(stf);
		    break;
		case 0x25:
		    gen_op_stfsr();
		    break;
		case 0x27:
                    gen_op_load_fpr_DT0(rd);
		    gen_op_ldst(stdf);
		    break;
		}
	    } else if (xop > 0x33 && xop < 0x38) {
		/* Co-processor */
            }
	}
    }
    /* default case for non jump instructions */
    if (dc->npc == DYNAMIC_PC) {
	dc->pc = DYNAMIC_PC;
	gen_op_next_insn();
    } else if (dc->npc == JUMP_PC) {
        /* we can do a static jump */
        gen_op_branch2((long)dc->tb, dc->jump_pc[0], dc->jump_pc[1]);
        dc->is_br = 1;
    } else {
	dc->pc = dc->npc;
	dc->npc = dc->npc + 4;
    }
  jmp_insn:;
    return;
 illegal_insn:
    save_state(dc);
    gen_op_exception(TT_ILL_INSN);
    dc->is_br = 1;
    return;
 priv_insn:
    save_state(dc);
    gen_op_exception(TT_PRIV_INSN);
    dc->is_br = 1;
}

static inline int gen_intermediate_code_internal(TranslationBlock * tb,
						 int spc, CPUSPARCState *env)
{
    target_ulong pc_start, last_pc;
    uint16_t *gen_opc_end;
    DisasContext dc1, *dc = &dc1;
    int j, lj = -1;

    memset(dc, 0, sizeof(DisasContext));
    dc->tb = tb;
    pc_start = tb->pc;
    dc->pc = pc_start;
    dc->npc = (target_ulong) tb->cs_base;
#if defined(CONFIG_USER_ONLY)
    dc->mem_idx = 0;
#else
    dc->mem_idx = ((env->psrs) != 0);
#endif
    gen_opc_ptr = gen_opc_buf;
    gen_opc_end = gen_opc_buf + OPC_MAX_SIZE;
    gen_opparam_ptr = gen_opparam_buf;

    do {
        if (env->nb_breakpoints > 0) {
            for(j = 0; j < env->nb_breakpoints; j++) {
                if (env->breakpoints[j] == dc->pc) {
                    gen_debug(dc, dc->pc);
                    break;
                }
            }
        }
        if (spc) {
            if (loglevel > 0)
                fprintf(logfile, "Search PC...\n");
            j = gen_opc_ptr - gen_opc_buf;
            if (lj < j) {
                lj++;
                while (lj < j)
                    gen_opc_instr_start[lj++] = 0;
                gen_opc_pc[lj] = dc->pc;
                gen_opc_npc[lj] = dc->npc;
                gen_opc_instr_start[lj] = 1;
            }
        }
	last_pc = dc->pc;
	disas_sparc_insn(dc);
	if (dc->is_br)
	    break;
	/* if the next PC is different, we abort now */
	if (dc->pc != (last_pc + 4))
	    break;
    } while ((gen_opc_ptr < gen_opc_end) &&
	     (dc->pc - pc_start) < (TARGET_PAGE_SIZE - 32));
    if (!dc->is_br) {
        if (dc->pc != DYNAMIC_PC && 
            (dc->npc != DYNAMIC_PC && dc->npc != JUMP_PC)) {
            /* static PC and NPC: we can use direct chaining */
            gen_op_branch((long)tb, dc->pc, dc->npc);
        } else {
            if (dc->pc != DYNAMIC_PC)
                gen_op_jmp_im(dc->pc);
            save_npc(dc);
            gen_op_movl_T0_0();
            gen_op_exit_tb();
        }
    }
    *gen_opc_ptr = INDEX_op_end;
    if (spc) {
        j = gen_opc_ptr - gen_opc_buf;
        lj++;
        while (lj <= j)
            gen_opc_instr_start[lj++] = 0;
        tb->size = 0;
#if 0
        if (loglevel > 0) {
            page_dump(logfile);
        }
#endif
    } else {
        tb->size = dc->npc - pc_start;
    }
#ifdef DEBUG_DISAS
    if (loglevel & CPU_LOG_TB_IN_ASM) {
	fprintf(logfile, "--------------\n");
	fprintf(logfile, "IN: %s\n", lookup_symbol((uint8_t *)pc_start));
	disas(logfile, (uint8_t *)pc_start, last_pc + 4 - pc_start, 0, 0);
	fprintf(logfile, "\n");
        if (loglevel & CPU_LOG_TB_OP) {
            fprintf(logfile, "OP:\n");
            dump_ops(gen_opc_buf, gen_opparam_buf);
            fprintf(logfile, "\n");
        }
    }
#endif
    return 0;
}

int gen_intermediate_code(CPUSPARCState * env, TranslationBlock * tb)
{
    return gen_intermediate_code_internal(tb, 0, env);
}

int gen_intermediate_code_pc(CPUSPARCState * env, TranslationBlock * tb)
{
    return gen_intermediate_code_internal(tb, 1, env);
}

CPUSPARCState *cpu_sparc_init(void)
{
    CPUSPARCState *env;

    cpu_exec_init();

    if (!(env = malloc(sizeof(CPUSPARCState))))
	return (NULL);
    memset(env, 0, sizeof(*env));
    env->cwp = 0;
    env->wim = 1;
    env->regwptr = env->regbase + (env->cwp * 16);
#if defined(CONFIG_USER_ONLY)
    env->user_mode_only = 1;
#else
    /* Emulate Prom */
    env->psrs = 1;
    env->pc = 0x4000;
    env->npc = env->pc + 4;
    env->mmuregs[0] = (0x10<<24) | MMU_E; /* Impl 1, ver 0, MMU Enabled */
    env->mmuregs[1] = 0x3000 >> 4; /* MMU Context table */
#endif
    cpu_single_env = env;
    return (env);
}

#define GET_FLAG(a,b) ((env->psr & a)?b:'-')

void cpu_dump_state(CPUState *env, FILE *f, 
                    int (*cpu_fprintf)(FILE *f, const char *fmt, ...),
                    int flags)
{
    int i, x;

    cpu_fprintf(f, "pc: 0x%08x  npc: 0x%08x\n", (int) env->pc, (int) env->npc);
    cpu_fprintf(f, "General Registers:\n");
    for (i = 0; i < 4; i++)
	cpu_fprintf(f, "%%g%c: 0x%08x\t", i + '0', env->gregs[i]);
    cpu_fprintf(f, "\n");
    for (; i < 8; i++)
	cpu_fprintf(f, "%%g%c: 0x%08x\t", i + '0', env->gregs[i]);
    cpu_fprintf(f, "\nCurrent Register Window:\n");
    for (x = 0; x < 3; x++) {
	for (i = 0; i < 4; i++)
	    cpu_fprintf(f, "%%%c%d: 0x%08x\t",
		    (x == 0 ? 'o' : (x == 1 ? 'l' : 'i')), i,
		    env->regwptr[i + x * 8]);
	cpu_fprintf(f, "\n");
	for (; i < 8; i++)
	    cpu_fprintf(f, "%%%c%d: 0x%08x\t",
		    (x == 0 ? 'o' : x == 1 ? 'l' : 'i'), i,
		    env->regwptr[i + x * 8]);
	cpu_fprintf(f, "\n");
    }
    cpu_fprintf(f, "\nFloating Point Registers:\n");
    for (i = 0; i < 32; i++) {
        if ((i & 3) == 0)
            cpu_fprintf(f, "%%f%02d:", i);
        cpu_fprintf(f, " %016lf", env->fpr[i]);
        if ((i & 3) == 3)
            cpu_fprintf(f, "\n");
    }
    cpu_fprintf(f, "psr: 0x%08x -> %c%c%c%c %c%c%c wim: 0x%08x\n", GET_PSR(env),
	    GET_FLAG(PSR_ZERO, 'Z'), GET_FLAG(PSR_OVF, 'V'),
	    GET_FLAG(PSR_NEG, 'N'), GET_FLAG(PSR_CARRY, 'C'),
	    env->psrs?'S':'-', env->psrps?'P':'-', 
	    env->psret?'E':'-', env->wim);
    cpu_fprintf(f, "fsr: 0x%08x\n", env->fsr);
}

target_ulong cpu_get_phys_page_debug(CPUState *env, target_ulong addr)
{
    return addr;
}

void helper_flush(target_ulong addr)
{
    addr &= ~7;
    tb_invalidate_page_range(addr, addr + 8);
}
