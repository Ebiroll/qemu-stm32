/*
 *  TriCore emulation for qemu: main translation routines.
 *
 *  Copyright (c) 2013-2014 Bastian Koppelmann C-Lab/University Paderborn
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
#include "disas/disas.h"
#include "tcg-op.h"
#include "exec/cpu_ldst.h"

#include "exec/helper-proto.h"
#include "exec/helper-gen.h"

#include "tricore-opcodes.h"

/*
 * TCG registers
 */
static TCGv cpu_PC;
static TCGv cpu_PCXI;
static TCGv cpu_PSW;
static TCGv cpu_ICR;
/* GPR registers */
static TCGv cpu_gpr_a[16];
static TCGv cpu_gpr_d[16];
/* PSW Flag cache */
static TCGv cpu_PSW_C;
static TCGv cpu_PSW_V;
static TCGv cpu_PSW_SV;
static TCGv cpu_PSW_AV;
static TCGv cpu_PSW_SAV;
/* CPU env */
static TCGv_ptr cpu_env;

#include "exec/gen-icount.h"

static const char *regnames_a[] = {
      "a0"  , "a1"  , "a2"  , "a3" , "a4"  , "a5" ,
      "a6"  , "a7"  , "a8"  , "a9" , "sp" , "a11" ,
      "a12" , "a13" , "a14" , "a15",
    };

static const char *regnames_d[] = {
      "d0"  , "d1"  , "d2"  , "d3" , "d4"  , "d5"  ,
      "d6"  , "d7"  , "d8"  , "d9" , "d10" , "d11" ,
      "d12" , "d13" , "d14" , "d15",
    };

typedef struct DisasContext {
    struct TranslationBlock *tb;
    target_ulong pc, saved_pc, next_pc;
    uint32_t opcode;
    int singlestep_enabled;
    /* Routine used to access memory */
    int mem_idx;
    uint32_t hflags, saved_hflags;
    int bstate;
} DisasContext;

enum {

    BS_NONE   = 0,
    BS_STOP   = 1,
    BS_BRANCH = 2,
    BS_EXCP   = 3,
};

enum {
    MODE_LL = 0,
    MODE_LU = 1,
    MODE_UL = 2,
    MODE_UU = 3,
};

void tricore_cpu_dump_state(CPUState *cs, FILE *f,
                            fprintf_function cpu_fprintf, int flags)
{
    TriCoreCPU *cpu = TRICORE_CPU(cs);
    CPUTriCoreState *env = &cpu->env;
    uint32_t psw;
    int i;

    psw = psw_read(env);

    cpu_fprintf(f, "PC: " TARGET_FMT_lx, env->PC);
    cpu_fprintf(f, " PSW: " TARGET_FMT_lx, psw);
    cpu_fprintf(f, " ICR: " TARGET_FMT_lx, env->ICR);
    cpu_fprintf(f, "\nPCXI: " TARGET_FMT_lx, env->PCXI);
    cpu_fprintf(f, " FCX: " TARGET_FMT_lx, env->FCX);
    cpu_fprintf(f, " LCX: " TARGET_FMT_lx, env->LCX);

    for (i = 0; i < 16; ++i) {
        if ((i & 3) == 0) {
            cpu_fprintf(f, "\nGPR A%02d:", i);
        }
        cpu_fprintf(f, " " TARGET_FMT_lx, env->gpr_a[i]);
    }
    for (i = 0; i < 16; ++i) {
        if ((i & 3) == 0) {
            cpu_fprintf(f, "\nGPR D%02d:", i);
        }
        cpu_fprintf(f, " " TARGET_FMT_lx, env->gpr_d[i]);
    }
    cpu_fprintf(f, "\n");
}

/*
 * Functions to generate micro-ops
 */

/* Makros for generating helpers */

#define gen_helper_1arg(name, arg) do {                           \
    TCGv_i32 helper_tmp = tcg_const_i32(arg);                     \
    gen_helper_##name(cpu_env, helper_tmp);                       \
    tcg_temp_free_i32(helper_tmp);                                \
    } while (0)

#define GEN_HELPER_LL(name, ret, arg0, arg1, n) do {         \
    TCGv arg00 = tcg_temp_new();                             \
    TCGv arg01 = tcg_temp_new();                             \
    TCGv arg11 = tcg_temp_new();                             \
    tcg_gen_sari_tl(arg00, arg0, 16);                        \
    tcg_gen_ext16s_tl(arg01, arg0);                          \
    tcg_gen_ext16s_tl(arg11, arg1);                          \
    gen_helper_##name(ret, arg00, arg01, arg11, arg11, n);   \
    tcg_temp_free(arg00);                                    \
    tcg_temp_free(arg01);                                    \
    tcg_temp_free(arg11);                                    \
} while (0)

#define GEN_HELPER_LU(name, ret, arg0, arg1, n) do {         \
    TCGv arg00 = tcg_temp_new();                             \
    TCGv arg01 = tcg_temp_new();                             \
    TCGv arg10 = tcg_temp_new();                             \
    TCGv arg11 = tcg_temp_new();                             \
    tcg_gen_sari_tl(arg00, arg0, 16);                        \
    tcg_gen_ext16s_tl(arg01, arg0);                          \
    tcg_gen_sari_tl(arg11, arg1, 16);                        \
    tcg_gen_ext16s_tl(arg10, arg1);                          \
    gen_helper_##name(ret, arg00, arg01, arg10, arg11, n);   \
    tcg_temp_free(arg00);                                    \
    tcg_temp_free(arg01);                                    \
    tcg_temp_free(arg10);                                    \
    tcg_temp_free(arg11);                                    \
} while (0)

#define GEN_HELPER_UL(name, ret, arg0, arg1, n) do {         \
    TCGv arg00 = tcg_temp_new();                             \
    TCGv arg01 = tcg_temp_new();                             \
    TCGv arg10 = tcg_temp_new();                             \
    TCGv arg11 = tcg_temp_new();                             \
    tcg_gen_sari_tl(arg00, arg0, 16);                        \
    tcg_gen_ext16s_tl(arg01, arg0);                          \
    tcg_gen_sari_tl(arg10, arg1, 16);                        \
    tcg_gen_ext16s_tl(arg11, arg1);                          \
    gen_helper_##name(ret, arg00, arg01, arg10, arg11, n);   \
    tcg_temp_free(arg00);                                    \
    tcg_temp_free(arg01);                                    \
    tcg_temp_free(arg10);                                    \
    tcg_temp_free(arg11);                                    \
} while (0)

#define GEN_HELPER_UU(name, ret, arg0, arg1, n) do {         \
    TCGv arg00 = tcg_temp_new();                             \
    TCGv arg01 = tcg_temp_new();                             \
    TCGv arg11 = tcg_temp_new();                             \
    tcg_gen_sari_tl(arg01, arg0, 16);                        \
    tcg_gen_ext16s_tl(arg00, arg0);                          \
    tcg_gen_sari_tl(arg11, arg1, 16);                        \
    gen_helper_##name(ret, arg00, arg01, arg11, arg11, n);   \
    tcg_temp_free(arg00);                                    \
    tcg_temp_free(arg01);                                    \
    tcg_temp_free(arg11);                                    \
} while (0)

#define GEN_HELPER_RRR(name, rl, rh, al1, ah1, arg2) do {    \
    TCGv_i64 ret = tcg_temp_new_i64();                       \
    TCGv_i64 arg1 = tcg_temp_new_i64();                      \
                                                             \
    tcg_gen_concat_i32_i64(arg1, al1, ah1);                  \
    gen_helper_##name(ret, arg1, arg2);                      \
    tcg_gen_extr_i64_i32(rl, rh, ret);                       \
                                                             \
    tcg_temp_free_i64(ret);                                  \
    tcg_temp_free_i64(arg1);                                 \
} while (0)

#define EA_ABS_FORMAT(con) (((con & 0x3C000) << 14) + (con & 0x3FFF))
#define EA_B_ABSOLUT(con) (((offset & 0xf00000) << 8) | \
                           ((offset & 0x0fffff) << 1))

/* Functions for load/save to/from memory */

static inline void gen_offset_ld(DisasContext *ctx, TCGv r1, TCGv r2,
                                 int16_t con, TCGMemOp mop)
{
    TCGv temp = tcg_temp_new();
    tcg_gen_addi_tl(temp, r2, con);
    tcg_gen_qemu_ld_tl(r1, temp, ctx->mem_idx, mop);
    tcg_temp_free(temp);
}

static inline void gen_offset_st(DisasContext *ctx, TCGv r1, TCGv r2,
                                 int16_t con, TCGMemOp mop)
{
    TCGv temp = tcg_temp_new();
    tcg_gen_addi_tl(temp, r2, con);
    tcg_gen_qemu_st_tl(r1, temp, ctx->mem_idx, mop);
    tcg_temp_free(temp);
}

static void gen_st_2regs_64(TCGv rh, TCGv rl, TCGv address, DisasContext *ctx)
{
    TCGv_i64 temp = tcg_temp_new_i64();

    tcg_gen_concat_i32_i64(temp, rl, rh);
    tcg_gen_qemu_st_i64(temp, address, ctx->mem_idx, MO_LEQ);

    tcg_temp_free_i64(temp);
}

static void gen_offset_st_2regs(TCGv rh, TCGv rl, TCGv base, int16_t con,
                                DisasContext *ctx)
{
    TCGv temp = tcg_temp_new();
    tcg_gen_addi_tl(temp, base, con);
    gen_st_2regs_64(rh, rl, temp, ctx);
    tcg_temp_free(temp);
}

static void gen_ld_2regs_64(TCGv rh, TCGv rl, TCGv address, DisasContext *ctx)
{
    TCGv_i64 temp = tcg_temp_new_i64();

    tcg_gen_qemu_ld_i64(temp, address, ctx->mem_idx, MO_LEQ);
    /* write back to two 32 bit regs */
    tcg_gen_extr_i64_i32(rl, rh, temp);

    tcg_temp_free_i64(temp);
}

static void gen_offset_ld_2regs(TCGv rh, TCGv rl, TCGv base, int16_t con,
                                DisasContext *ctx)
{
    TCGv temp = tcg_temp_new();
    tcg_gen_addi_tl(temp, base, con);
    gen_ld_2regs_64(rh, rl, temp, ctx);
    tcg_temp_free(temp);
}

static void gen_st_preincr(DisasContext *ctx, TCGv r1, TCGv r2, int16_t off,
                           TCGMemOp mop)
{
    TCGv temp = tcg_temp_new();
    tcg_gen_addi_tl(temp, r2, off);
    tcg_gen_qemu_st_tl(r1, temp, ctx->mem_idx, mop);
    tcg_gen_mov_tl(r2, temp);
    tcg_temp_free(temp);
}

static void gen_ld_preincr(DisasContext *ctx, TCGv r1, TCGv r2, int16_t off,
                           TCGMemOp mop)
{
    TCGv temp = tcg_temp_new();
    tcg_gen_addi_tl(temp, r2, off);
    tcg_gen_qemu_ld_tl(r1, temp, ctx->mem_idx, mop);
    tcg_gen_mov_tl(r2, temp);
    tcg_temp_free(temp);
}

/* M(EA, word) = (M(EA, word) & ~E[a][63:32]) | (E[a][31:0] & E[a][63:32]); */
static void gen_ldmst(DisasContext *ctx, int ereg, TCGv ea)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();

    /* temp = (M(EA, word) */
    tcg_gen_qemu_ld_tl(temp, ea, ctx->mem_idx, MO_LEUL);
    /* temp = temp & ~E[a][63:32]) */
    tcg_gen_andc_tl(temp, temp, cpu_gpr_d[ereg+1]);
    /* temp2 = (E[a][31:0] & E[a][63:32]); */
    tcg_gen_and_tl(temp2, cpu_gpr_d[ereg], cpu_gpr_d[ereg+1]);
    /* temp = temp | temp2; */
    tcg_gen_or_tl(temp, temp, temp2);
    /* M(EA, word) = temp; */
    tcg_gen_qemu_st_tl(temp, ea, ctx->mem_idx, MO_LEUL);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

/* tmp = M(EA, word);
   M(EA, word) = D[a];
   D[a] = tmp[31:0];*/
static void gen_swap(DisasContext *ctx, int reg, TCGv ea)
{
    TCGv temp = tcg_temp_new();

    tcg_gen_qemu_ld_tl(temp, ea, ctx->mem_idx, MO_LEUL);
    tcg_gen_qemu_st_tl(cpu_gpr_d[reg], ea, ctx->mem_idx, MO_LEUL);
    tcg_gen_mov_tl(cpu_gpr_d[reg], temp);

    tcg_temp_free(temp);
}

static void gen_cmpswap(DisasContext *ctx, int reg, TCGv ea)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    tcg_gen_qemu_ld_tl(temp, ea, ctx->mem_idx, MO_LEUL);
    tcg_gen_movcond_tl(TCG_COND_EQ, temp2, cpu_gpr_d[reg+1], temp,
                       cpu_gpr_d[reg], temp);
    tcg_gen_qemu_st_tl(temp2, ea, ctx->mem_idx, MO_LEUL);
    tcg_gen_mov_tl(cpu_gpr_d[reg], temp);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static void gen_swapmsk(DisasContext *ctx, int reg, TCGv ea)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    TCGv temp3 = tcg_temp_new();

    tcg_gen_qemu_ld_tl(temp, ea, ctx->mem_idx, MO_LEUL);
    tcg_gen_and_tl(temp2, cpu_gpr_d[reg], cpu_gpr_d[reg+1]);
    tcg_gen_andc_tl(temp3, temp, cpu_gpr_d[reg+1]);
    tcg_gen_or_tl(temp2, temp2, temp3);
    tcg_gen_qemu_st_tl(temp2, ea, ctx->mem_idx, MO_LEUL);
    tcg_gen_mov_tl(cpu_gpr_d[reg], temp);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(temp3);
}


/* We generate loads and store to core special function register (csfr) through
   the function gen_mfcr and gen_mtcr. To handle access permissions, we use 3
   makros R, A and E, which allow read-only, all and endinit protected access.
   These makros also specify in which ISA version the csfr was introduced. */
#define R(ADDRESS, REG, FEATURE)                                         \
    case ADDRESS:                                                        \
        if (tricore_feature(env, FEATURE)) {                             \
            tcg_gen_ld_tl(ret, cpu_env, offsetof(CPUTriCoreState, REG)); \
        }                                                                \
        break;
#define A(ADDRESS, REG, FEATURE) R(ADDRESS, REG, FEATURE)
#define E(ADDRESS, REG, FEATURE) R(ADDRESS, REG, FEATURE)
static inline void gen_mfcr(CPUTriCoreState *env, TCGv ret, int32_t offset)
{
    /* since we're caching PSW make this a special case */
    if (offset == 0xfe04) {
        gen_helper_psw_read(ret, cpu_env);
    } else {
        switch (offset) {
#include "csfr.def"
        }
    }
}
#undef R
#undef A
#undef E

#define R(ADDRESS, REG, FEATURE) /* don't gen writes to read-only reg,
                                    since no execption occurs */
#define A(ADDRESS, REG, FEATURE) R(ADDRESS, REG, FEATURE)                \
    case ADDRESS:                                                        \
        if (tricore_feature(env, FEATURE)) {                             \
            tcg_gen_st_tl(r1, cpu_env, offsetof(CPUTriCoreState, REG));  \
        }                                                                \
        break;
/* Endinit protected registers
   TODO: Since the endinit bit is in a register of a not yet implemented
         watchdog device, we handle endinit protected registers like
         all-access registers for now. */
#define E(ADDRESS, REG, FEATURE) A(ADDRESS, REG, FEATURE)
static inline void gen_mtcr(CPUTriCoreState *env, DisasContext *ctx, TCGv r1,
                            int32_t offset)
{
    if ((ctx->hflags & TRICORE_HFLAG_KUU) == TRICORE_HFLAG_SM) {
        /* since we're caching PSW make this a special case */
        if (offset == 0xfe04) {
            gen_helper_psw_write(cpu_env, r1);
        } else {
            switch (offset) {
#include "csfr.def"
            }
        }
    } else {
        /* generate privilege trap */
    }
}

/* Functions for arithmetic instructions  */

static inline void gen_add_d(TCGv ret, TCGv r1, TCGv r2)
{
    TCGv t0 = tcg_temp_new_i32();
    TCGv result = tcg_temp_new_i32();
    /* Addition and set V/SV bits */
    tcg_gen_add_tl(result, r1, r2);
    /* calc V bit */
    tcg_gen_xor_tl(cpu_PSW_V, result, r1);
    tcg_gen_xor_tl(t0, r1, r2);
    tcg_gen_andc_tl(cpu_PSW_V, cpu_PSW_V, t0);
    /* Calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV/SAV bits */
    tcg_gen_add_tl(cpu_PSW_AV, result, result);
    tcg_gen_xor_tl(cpu_PSW_AV, result, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* write back result */
    tcg_gen_mov_tl(ret, result);

    tcg_temp_free(result);
    tcg_temp_free(t0);
}

static inline void
gen_add64_d(TCGv_i64 ret, TCGv_i64 r1, TCGv_i64 r2)
{
    TCGv temp = tcg_temp_new();
    TCGv_i64 t0 = tcg_temp_new_i64();
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 result = tcg_temp_new_i64();

    tcg_gen_add_i64(result, r1, r2);
    /* calc v bit */
    tcg_gen_xor_i64(t1, result, r1);
    tcg_gen_xor_i64(t0, r1, r2);
    tcg_gen_andc_i64(t1, t1, t0);
    tcg_gen_trunc_shr_i64_i32(cpu_PSW_V, t1, 32);
    /* calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* calc AV/SAV bits */
    tcg_gen_trunc_shr_i64_i32(temp, result, 32);
    tcg_gen_add_tl(cpu_PSW_AV, temp, temp);
    tcg_gen_xor_tl(cpu_PSW_AV, temp, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* write back result */
    tcg_gen_mov_i64(ret, result);

    tcg_temp_free(temp);
    tcg_temp_free_i64(result);
    tcg_temp_free_i64(t0);
    tcg_temp_free_i64(t1);
}

static inline void
gen_addsub64_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
               TCGv r3, void(*op1)(TCGv, TCGv, TCGv),
               void(*op2)(TCGv, TCGv, TCGv))
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    TCGv temp3 = tcg_temp_new();
    TCGv temp4 = tcg_temp_new();

    (*op1)(temp, r1_low, r2);
    /* calc V0 bit */
    tcg_gen_xor_tl(temp2, temp, r1_low);
    tcg_gen_xor_tl(temp3, r1_low, r2);
    if (op1 == tcg_gen_add_tl) {
        tcg_gen_andc_tl(temp2, temp2, temp3);
    } else {
        tcg_gen_and_tl(temp2, temp2, temp3);
    }

    (*op2)(temp3, r1_high, r3);
    /* calc V1 bit */
    tcg_gen_xor_tl(cpu_PSW_V, temp3, r1_high);
    tcg_gen_xor_tl(temp4, r1_high, r3);
    if (op2 == tcg_gen_add_tl) {
        tcg_gen_andc_tl(cpu_PSW_V, cpu_PSW_V, temp4);
    } else {
        tcg_gen_and_tl(cpu_PSW_V, cpu_PSW_V, temp4);
    }
    /* combine V0/V1 bits */
    tcg_gen_or_tl(cpu_PSW_V, cpu_PSW_V, temp2);
    /* calc sv bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* write result */
    tcg_gen_mov_tl(ret_low, temp);
    tcg_gen_mov_tl(ret_high, temp3);
    /* calc AV bit */
    tcg_gen_add_tl(temp, ret_low, ret_low);
    tcg_gen_xor_tl(temp, temp, ret_low);
    tcg_gen_add_tl(cpu_PSW_AV, ret_high, ret_high);
    tcg_gen_xor_tl(cpu_PSW_AV, cpu_PSW_AV, ret_high);
    tcg_gen_or_tl(cpu_PSW_AV, cpu_PSW_AV, temp);
    /* calc SAV bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(temp3);
    tcg_temp_free(temp4);
}

/* ret = r2 + (r1 * r3); */
static inline void gen_madd32_d(TCGv ret, TCGv r1, TCGv r2, TCGv r3)
{
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();

    tcg_gen_ext_i32_i64(t1, r1);
    tcg_gen_ext_i32_i64(t2, r2);
    tcg_gen_ext_i32_i64(t3, r3);

    tcg_gen_mul_i64(t1, t1, t3);
    tcg_gen_add_i64(t1, t2, t1);

    tcg_gen_trunc_i64_i32(ret, t1);
    /* calc V
       t1 > 0x7fffffff */
    tcg_gen_setcondi_i64(TCG_COND_GT, t3, t1, 0x7fffffffLL);
    /* t1 < -0x80000000 */
    tcg_gen_setcondi_i64(TCG_COND_LT, t2, t1, -0x80000000LL);
    tcg_gen_or_i64(t2, t2, t3);
    tcg_gen_trunc_i64_i32(cpu_PSW_V, t2);
    tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
    /* Calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV/SAV bits */
    tcg_gen_add_tl(cpu_PSW_AV, ret, ret);
    tcg_gen_xor_tl(cpu_PSW_AV, ret, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
}

static inline void gen_maddi32_d(TCGv ret, TCGv r1, TCGv r2, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_madd32_d(ret, r1, r2, temp);
    tcg_temp_free(temp);
}

static inline void
gen_madd64_d(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
             TCGv r3)
{
    TCGv t1 = tcg_temp_new();
    TCGv t2 = tcg_temp_new();
    TCGv t3 = tcg_temp_new();
    TCGv t4 = tcg_temp_new();

    tcg_gen_muls2_tl(t1, t2, r1, r3);
    /* only the add can overflow */
    tcg_gen_add2_tl(t3, t4, r2_low, r2_high, t1, t2);
    /* calc V bit */
    tcg_gen_xor_tl(cpu_PSW_V, t4, r2_high);
    tcg_gen_xor_tl(t1, r2_high, t2);
    tcg_gen_andc_tl(cpu_PSW_V, cpu_PSW_V, t1);
    /* Calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV/SAV bits */
    tcg_gen_add_tl(cpu_PSW_AV, t4, t4);
    tcg_gen_xor_tl(cpu_PSW_AV, t4, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* write back the result */
    tcg_gen_mov_tl(ret_low, t3);
    tcg_gen_mov_tl(ret_high, t4);

    tcg_temp_free(t1);
    tcg_temp_free(t2);
    tcg_temp_free(t3);
    tcg_temp_free(t4);
}

static inline void
gen_maddu64_d(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
              TCGv r3)
{
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();

    tcg_gen_extu_i32_i64(t1, r1);
    tcg_gen_concat_i32_i64(t2, r2_low, r2_high);
    tcg_gen_extu_i32_i64(t3, r3);

    tcg_gen_mul_i64(t1, t1, t3);
    tcg_gen_add_i64(t2, t2, t1);
    /* write back result */
    tcg_gen_extr_i64_i32(ret_low, ret_high, t2);
    /* only the add overflows, if t2 < t1
       calc V bit */
    tcg_gen_setcond_i64(TCG_COND_LTU, t2, t2, t1);
    tcg_gen_trunc_i64_i32(cpu_PSW_V, t2);
    tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
    /* Calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV/SAV bits */
    tcg_gen_add_tl(cpu_PSW_AV, ret_high, ret_high);
    tcg_gen_xor_tl(cpu_PSW_AV, ret_high, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
}

static inline void
gen_maddi64_d(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
              int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_madd64_d(ret_low, ret_high, r1, r2_low, r2_high, temp);
    tcg_temp_free(temp);
}

static inline void
gen_maddui64_d(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
               int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_maddu64_d(ret_low, ret_high, r1, r2_low, r2_high, temp);
    tcg_temp_free(temp);
}

static inline void
gen_madd_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
           TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv temp2 = tcg_temp_new();
    TCGv_i64 temp64 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_extr_i64_i32(temp, temp2, temp64);
    gen_addsub64_h(ret_low, ret_high, r1_low, r1_high, temp, temp2,
                   tcg_gen_add_tl, tcg_gen_add_tl);
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_maddsu_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
             TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv temp2 = tcg_temp_new();
    TCGv_i64 temp64 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_extr_i64_i32(temp, temp2, temp64);
    gen_addsub64_h(ret_low, ret_high, r1_low, r1_high, temp, temp2,
                   tcg_gen_sub_tl, tcg_gen_add_tl);
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_maddsum_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
              TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv_i64 temp64 = tcg_temp_new_i64();
    TCGv_i64 temp64_2 = tcg_temp_new_i64();
    TCGv_i64 temp64_3 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_concat_i32_i64(temp64_3, r1_low, r1_high);
    tcg_gen_sari_i64(temp64_2, temp64, 32); /* high */
    tcg_gen_ext32s_i64(temp64, temp64); /* low */
    tcg_gen_sub_i64(temp64, temp64_2, temp64);
    tcg_gen_shli_i64(temp64, temp64, 16);

    gen_add64_d(temp64_2, temp64_3, temp64);
    /* write back result */
    tcg_gen_extr_i64_i32(ret_low, ret_high, temp64_2);

    tcg_temp_free(temp);
    tcg_temp_free_i64(temp64);
    tcg_temp_free_i64(temp64_2);
    tcg_temp_free_i64(temp64_3);
}

static inline void gen_adds(TCGv ret, TCGv r1, TCGv r2);

static inline void
gen_madds_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
           TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv temp2 = tcg_temp_new();
    TCGv temp3 = tcg_temp_new();
    TCGv_i64 temp64 = tcg_temp_new_i64();

    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_extr_i64_i32(temp, temp2, temp64);
    gen_adds(ret_low, r1_low, temp);
    tcg_gen_mov_tl(temp, cpu_PSW_V);
    tcg_gen_mov_tl(temp3, cpu_PSW_AV);
    gen_adds(ret_high, r1_high, temp2);
    /* combine v bits */
    tcg_gen_or_tl(cpu_PSW_V, cpu_PSW_V, temp);
    /* combine av bits */
    tcg_gen_or_tl(cpu_PSW_AV, cpu_PSW_AV, temp3);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(temp3);
    tcg_temp_free_i64(temp64);

}

static inline void gen_subs(TCGv ret, TCGv r1, TCGv r2);

static inline void
gen_maddsus_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
              TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv temp2 = tcg_temp_new();
    TCGv temp3 = tcg_temp_new();
    TCGv_i64 temp64 = tcg_temp_new_i64();

    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_extr_i64_i32(temp, temp2, temp64);
    gen_subs(ret_low, r1_low, temp);
    tcg_gen_mov_tl(temp, cpu_PSW_V);
    tcg_gen_mov_tl(temp3, cpu_PSW_AV);
    gen_adds(ret_high, r1_high, temp2);
    /* combine v bits */
    tcg_gen_or_tl(cpu_PSW_V, cpu_PSW_V, temp);
    /* combine av bits */
    tcg_gen_or_tl(cpu_PSW_AV, cpu_PSW_AV, temp3);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(temp3);
    tcg_temp_free_i64(temp64);

}

static inline void
gen_maddsums_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
               TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv_i64 temp64 = tcg_temp_new_i64();
    TCGv_i64 temp64_2 = tcg_temp_new_i64();

    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_sari_i64(temp64_2, temp64, 32); /* high */
    tcg_gen_ext32s_i64(temp64, temp64); /* low */
    tcg_gen_sub_i64(temp64, temp64_2, temp64);
    tcg_gen_shli_i64(temp64, temp64, 16);
    tcg_gen_concat_i32_i64(temp64_2, r1_low, r1_high);

    gen_helper_add64_ssov(temp64, cpu_env, temp64_2, temp64);
    tcg_gen_extr_i64_i32(ret_low, ret_high, temp64);

    tcg_temp_free(temp);
    tcg_temp_free_i64(temp64);
    tcg_temp_free_i64(temp64_2);
}


static inline void
gen_maddm_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
           TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv_i64 temp64 = tcg_temp_new_i64();
    TCGv_i64 temp64_2 = tcg_temp_new_i64();
    TCGv_i64 temp64_3 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mulm_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mulm_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mulm_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mulm_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_concat_i32_i64(temp64_2, r1_low, r1_high);
    gen_add64_d(temp64_3, temp64_2, temp64);
    /* write back result */
    tcg_gen_extr_i64_i32(ret_low, ret_high, temp64_3);

    tcg_temp_free(temp);
    tcg_temp_free_i64(temp64);
    tcg_temp_free_i64(temp64_2);
    tcg_temp_free_i64(temp64_3);
}

static inline void
gen_maddms_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
           TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv_i64 temp64 = tcg_temp_new_i64();
    TCGv_i64 temp64_2 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mulm_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mulm_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mulm_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mulm_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_concat_i32_i64(temp64_2, r1_low, r1_high);
    gen_helper_add64_ssov(temp64, cpu_env, temp64_2, temp64);
    tcg_gen_extr_i64_i32(ret_low, ret_high, temp64);

    tcg_temp_free(temp);
    tcg_temp_free_i64(temp64);
    tcg_temp_free_i64(temp64_2);
}

static inline void
gen_maddr64_h(TCGv ret, TCGv r1_low, TCGv r1_high, TCGv r2, TCGv r3, uint32_t n,
              uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv_i64 temp64 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    gen_helper_addr_h(ret, cpu_env, temp64, r1_low, r1_high);

    tcg_temp_free(temp);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_maddr32_h(TCGv ret, TCGv r1, TCGv r2, TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();

    tcg_gen_andi_tl(temp2, r1, 0xffff0000);
    tcg_gen_shli_tl(temp, r1, 16);
    gen_maddr64_h(ret, temp, temp2, r2, r3, n, mode);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static inline void
gen_maddsur32_h(TCGv ret, TCGv r1, TCGv r2, TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv temp2 = tcg_temp_new();
    TCGv_i64 temp64 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_andi_tl(temp2, r1, 0xffff0000);
    tcg_gen_shli_tl(temp, r1, 16);
    gen_helper_addsur_h(ret, cpu_env, temp64, temp, temp2);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free_i64(temp64);
}


static inline void
gen_maddr64s_h(TCGv ret, TCGv r1_low, TCGv r1_high, TCGv r2, TCGv r3,
               uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv_i64 temp64 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    gen_helper_addr_h_ssov(ret, cpu_env, temp64, r1_low, r1_high);

    tcg_temp_free(temp);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_maddr32s_h(TCGv ret, TCGv r1, TCGv r2, TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();

    tcg_gen_andi_tl(temp2, r1, 0xffff0000);
    tcg_gen_shli_tl(temp, r1, 16);
    gen_maddr64s_h(ret, temp, temp2, r2, r3, n, mode);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static inline void
gen_maddsur32s_h(TCGv ret, TCGv r1, TCGv r2, TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv temp2 = tcg_temp_new();
    TCGv_i64 temp64 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_andi_tl(temp2, r1, 0xffff0000);
    tcg_gen_shli_tl(temp, r1, 16);
    gen_helper_addsur_h_ssov(ret, cpu_env, temp64, temp, temp2);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_maddr_q(TCGv ret, TCGv r1, TCGv r2, TCGv r3, uint32_t n)
{
    TCGv temp = tcg_const_i32(n);
    gen_helper_maddr_q(ret, cpu_env, r1, r2, r3, temp);
    tcg_temp_free(temp);
}

static inline void
gen_maddrs_q(TCGv ret, TCGv r1, TCGv r2, TCGv r3, uint32_t n)
{
    TCGv temp = tcg_const_i32(n);
    gen_helper_maddr_q_ssov(ret, cpu_env, r1, r2, r3, temp);
    tcg_temp_free(temp);
}

static inline void
gen_madd32_q(TCGv ret, TCGv arg1, TCGv arg2, TCGv arg3, uint32_t n,
             uint32_t up_shift, CPUTriCoreState *env)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    TCGv temp3 = tcg_temp_new();
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();

    tcg_gen_ext_i32_i64(t2, arg2);
    tcg_gen_ext_i32_i64(t3, arg3);

    tcg_gen_mul_i64(t2, t2, t3);
    tcg_gen_shli_i64(t2, t2, n);

    tcg_gen_ext_i32_i64(t1, arg1);
    tcg_gen_sari_i64(t2, t2, up_shift);

    tcg_gen_add_i64(t3, t1, t2);
    tcg_gen_trunc_i64_i32(temp3, t3);
    /* calc v bit */
    tcg_gen_setcondi_i64(TCG_COND_GT, t1, t3, 0x7fffffffLL);
    tcg_gen_setcondi_i64(TCG_COND_LT, t2, t3, -0x80000000LL);
    tcg_gen_or_i64(t1, t1, t2);
    tcg_gen_trunc_i64_i32(cpu_PSW_V, t1);
    tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
    /* We produce an overflow on the host if the mul before was
       (0x80000000 * 0x80000000) << 1). If this is the
       case, we negate the ovf. */
    if (n == 1) {
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp, arg2, 0x80000000);
        tcg_gen_setcond_tl(TCG_COND_EQ, temp2, arg2, arg3);
        tcg_gen_and_tl(temp, temp, temp2);
        tcg_gen_shli_tl(temp, temp, 31);
        /* negate v bit, if special condition */
        tcg_gen_xor_tl(cpu_PSW_V, cpu_PSW_V, temp);
    }
    /* Calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV/SAV bits */
    tcg_gen_add_tl(cpu_PSW_AV, temp3, temp3);
    tcg_gen_xor_tl(cpu_PSW_AV, temp3, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* write back result */
    tcg_gen_mov_tl(ret, temp3);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(temp3);
    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
}

static inline void
gen_m16add32_q(TCGv ret, TCGv arg1, TCGv arg2, TCGv arg3, uint32_t n)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    if (n == 0) {
        tcg_gen_mul_tl(temp, arg2, arg3);
    } else { /* n is expected to be 1 */
        tcg_gen_mul_tl(temp, arg2, arg3);
        tcg_gen_shli_tl(temp, temp, 1);
        /* catch special case r1 = r2 = 0x8000 */
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp2, temp, 0x80000000);
        tcg_gen_sub_tl(temp, temp, temp2);
    }
    gen_add_d(ret, arg1, temp);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static inline void
gen_m16adds32_q(TCGv ret, TCGv arg1, TCGv arg2, TCGv arg3, uint32_t n)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    if (n == 0) {
        tcg_gen_mul_tl(temp, arg2, arg3);
    } else { /* n is expected to be 1 */
        tcg_gen_mul_tl(temp, arg2, arg3);
        tcg_gen_shli_tl(temp, temp, 1);
        /* catch special case r1 = r2 = 0x8000 */
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp2, temp, 0x80000000);
        tcg_gen_sub_tl(temp, temp, temp2);
    }
    gen_adds(ret, arg1, temp);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static inline void
gen_m16add64_q(TCGv rl, TCGv rh, TCGv arg1_low, TCGv arg1_high, TCGv arg2,
               TCGv arg3, uint32_t n)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();

    if (n == 0) {
        tcg_gen_mul_tl(temp, arg2, arg3);
    } else { /* n is expected to be 1 */
        tcg_gen_mul_tl(temp, arg2, arg3);
        tcg_gen_shli_tl(temp, temp, 1);
        /* catch special case r1 = r2 = 0x8000 */
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp2, temp, 0x80000000);
        tcg_gen_sub_tl(temp, temp, temp2);
    }
    tcg_gen_ext_i32_i64(t2, temp);
    tcg_gen_shli_i64(t2, t2, 16);
    tcg_gen_concat_i32_i64(t1, arg1_low, arg1_high);
    gen_add64_d(t3, t1, t2);
    /* write back result */
    tcg_gen_extr_i64_i32(rl, rh, t3);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static inline void
gen_m16adds64_q(TCGv rl, TCGv rh, TCGv arg1_low, TCGv arg1_high, TCGv arg2,
               TCGv arg3, uint32_t n)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();

    if (n == 0) {
        tcg_gen_mul_tl(temp, arg2, arg3);
    } else { /* n is expected to be 1 */
        tcg_gen_mul_tl(temp, arg2, arg3);
        tcg_gen_shli_tl(temp, temp, 1);
        /* catch special case r1 = r2 = 0x8000 */
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp2, temp, 0x80000000);
        tcg_gen_sub_tl(temp, temp, temp2);
    }
    tcg_gen_ext_i32_i64(t2, temp);
    tcg_gen_shli_i64(t2, t2, 16);
    tcg_gen_concat_i32_i64(t1, arg1_low, arg1_high);

    gen_helper_add64_ssov(t1, cpu_env, t1, t2);
    tcg_gen_extr_i64_i32(rl, rh, t1);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
}

static inline void
gen_madd64_q(TCGv rl, TCGv rh, TCGv arg1_low, TCGv arg1_high, TCGv arg2,
             TCGv arg3, uint32_t n, CPUTriCoreState *env)
{
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();
    TCGv_i64 t4 = tcg_temp_new_i64();
    TCGv temp, temp2;

    tcg_gen_concat_i32_i64(t1, arg1_low, arg1_high);
    tcg_gen_ext_i32_i64(t2, arg2);
    tcg_gen_ext_i32_i64(t3, arg3);

    tcg_gen_mul_i64(t2, t2, t3);
    if (n != 0) {
        tcg_gen_shli_i64(t2, t2, 1);
    }
    tcg_gen_add_i64(t4, t1, t2);
    /* calc v bit */
    tcg_gen_xor_i64(t3, t4, t1);
    tcg_gen_xor_i64(t2, t1, t2);
    tcg_gen_andc_i64(t3, t3, t2);
    tcg_gen_trunc_shr_i64_i32(cpu_PSW_V, t3, 32);
    /* We produce an overflow on the host if the mul before was
       (0x80000000 * 0x80000000) << 1). If this is the
       case, we negate the ovf. */
    if (n == 1) {
        temp = tcg_temp_new();
        temp2 = tcg_temp_new();
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp, arg2, 0x80000000);
        tcg_gen_setcond_tl(TCG_COND_EQ, temp2, arg2, arg3);
        tcg_gen_and_tl(temp, temp, temp2);
        tcg_gen_shli_tl(temp, temp, 31);
        /* negate v bit, if special condition */
        tcg_gen_xor_tl(cpu_PSW_V, cpu_PSW_V, temp);

        tcg_temp_free(temp);
        tcg_temp_free(temp2);
    }
    /* write back result */
    tcg_gen_extr_i64_i32(rl, rh, t4);
    /* Calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV/SAV bits */
    tcg_gen_add_tl(cpu_PSW_AV, rh, rh);
    tcg_gen_xor_tl(cpu_PSW_AV, rh, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
    tcg_temp_free_i64(t4);
}

static inline void
gen_madds32_q(TCGv ret, TCGv arg1, TCGv arg2, TCGv arg3, uint32_t n,
              uint32_t up_shift)
{
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();

    tcg_gen_ext_i32_i64(t1, arg1);
    tcg_gen_ext_i32_i64(t2, arg2);
    tcg_gen_ext_i32_i64(t3, arg3);

    tcg_gen_mul_i64(t2, t2, t3);
    tcg_gen_sari_i64(t2, t2, up_shift - n);

    gen_helper_madd32_q_add_ssov(ret, cpu_env, t1, t2);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
}

static inline void
gen_madds64_q(TCGv rl, TCGv rh, TCGv arg1_low, TCGv arg1_high, TCGv arg2,
             TCGv arg3, uint32_t n)
{
    TCGv_i64 r1 = tcg_temp_new_i64();
    TCGv temp = tcg_const_i32(n);

    tcg_gen_concat_i32_i64(r1, arg1_low, arg1_high);
    gen_helper_madd64_q_ssov(r1, cpu_env, r1, arg2, arg3, temp);
    tcg_gen_extr_i64_i32(rl, rh, r1);

    tcg_temp_free_i64(r1);
    tcg_temp_free(temp);
}
/* ret = r2 - (r1 * r3); */
static inline void gen_msub32_d(TCGv ret, TCGv r1, TCGv r2, TCGv r3)
{
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();

    tcg_gen_ext_i32_i64(t1, r1);
    tcg_gen_ext_i32_i64(t2, r2);
    tcg_gen_ext_i32_i64(t3, r3);

    tcg_gen_mul_i64(t1, t1, t3);
    tcg_gen_sub_i64(t1, t2, t1);

    tcg_gen_trunc_i64_i32(ret, t1);
    /* calc V
       t2 > 0x7fffffff */
    tcg_gen_setcondi_i64(TCG_COND_GT, t3, t1, 0x7fffffffLL);
    /* result < -0x80000000 */
    tcg_gen_setcondi_i64(TCG_COND_LT, t2, t1, -0x80000000LL);
    tcg_gen_or_i64(t2, t2, t3);
    tcg_gen_trunc_i64_i32(cpu_PSW_V, t2);
    tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);

    /* Calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV/SAV bits */
    tcg_gen_add_tl(cpu_PSW_AV, ret, ret);
    tcg_gen_xor_tl(cpu_PSW_AV, ret, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
}

static inline void gen_msubi32_d(TCGv ret, TCGv r1, TCGv r2, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_msub32_d(ret, r1, r2, temp);
    tcg_temp_free(temp);
}

static inline void
gen_msub64_d(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
             TCGv r3)
{
    TCGv t1 = tcg_temp_new();
    TCGv t2 = tcg_temp_new();
    TCGv t3 = tcg_temp_new();
    TCGv t4 = tcg_temp_new();

    tcg_gen_muls2_tl(t1, t2, r1, r3);
    /* only the sub can overflow */
    tcg_gen_sub2_tl(t3, t4, r2_low, r2_high, t1, t2);
    /* calc V bit */
    tcg_gen_xor_tl(cpu_PSW_V, t4, r2_high);
    tcg_gen_xor_tl(t1, r2_high, t2);
    tcg_gen_and_tl(cpu_PSW_V, cpu_PSW_V, t1);
    /* Calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV/SAV bits */
    tcg_gen_add_tl(cpu_PSW_AV, t4, t4);
    tcg_gen_xor_tl(cpu_PSW_AV, t4, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* write back the result */
    tcg_gen_mov_tl(ret_low, t3);
    tcg_gen_mov_tl(ret_high, t4);

    tcg_temp_free(t1);
    tcg_temp_free(t2);
    tcg_temp_free(t3);
    tcg_temp_free(t4);
}

static inline void
gen_msubi64_d(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
              int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_msub64_d(ret_low, ret_high, r1, r2_low, r2_high, temp);
    tcg_temp_free(temp);
}

static inline void
gen_msubu64_d(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
              TCGv r3)
{
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();

    tcg_gen_extu_i32_i64(t1, r1);
    tcg_gen_concat_i32_i64(t2, r2_low, r2_high);
    tcg_gen_extu_i32_i64(t3, r3);

    tcg_gen_mul_i64(t1, t1, t3);
    tcg_gen_sub_i64(t3, t2, t1);
    tcg_gen_extr_i64_i32(ret_low, ret_high, t3);
    /* calc V bit, only the sub can overflow, if t1 > t2 */
    tcg_gen_setcond_i64(TCG_COND_GTU, t1, t1, t2);
    tcg_gen_trunc_i64_i32(cpu_PSW_V, t1);
    tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
    /* Calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV/SAV bits */
    tcg_gen_add_tl(cpu_PSW_AV, ret_high, ret_high);
    tcg_gen_xor_tl(cpu_PSW_AV, ret_high, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
}

static inline void
gen_msubui64_d(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
               int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_msubu64_d(ret_low, ret_high, r1, r2_low, r2_high, temp);
    tcg_temp_free(temp);
}

static inline void gen_addi_d(TCGv ret, TCGv r1, target_ulong r2)
{
    TCGv temp = tcg_const_i32(r2);
    gen_add_d(ret, r1, temp);
    tcg_temp_free(temp);
}
/* calculate the carry bit too */
static inline void gen_add_CC(TCGv ret, TCGv r1, TCGv r2)
{
    TCGv t0    = tcg_temp_new_i32();
    TCGv result = tcg_temp_new_i32();

    tcg_gen_movi_tl(t0, 0);
    /* Addition and set C/V/SV bits */
    tcg_gen_add2_i32(result, cpu_PSW_C, r1, t0, r2, t0);
    /* calc V bit */
    tcg_gen_xor_tl(cpu_PSW_V, result, r1);
    tcg_gen_xor_tl(t0, r1, r2);
    tcg_gen_andc_tl(cpu_PSW_V, cpu_PSW_V, t0);
    /* Calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV/SAV bits */
    tcg_gen_add_tl(cpu_PSW_AV, result, result);
    tcg_gen_xor_tl(cpu_PSW_AV, result, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* write back result */
    tcg_gen_mov_tl(ret, result);

    tcg_temp_free(result);
    tcg_temp_free(t0);
}

static inline void gen_addi_CC(TCGv ret, TCGv r1, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_add_CC(ret, r1, temp);
    tcg_temp_free(temp);
}

static inline void gen_addc_CC(TCGv ret, TCGv r1, TCGv r2)
{
    TCGv carry = tcg_temp_new_i32();
    TCGv t0    = tcg_temp_new_i32();
    TCGv result = tcg_temp_new_i32();

    tcg_gen_movi_tl(t0, 0);
    tcg_gen_setcondi_tl(TCG_COND_NE, carry, cpu_PSW_C, 0);
    /* Addition, carry and set C/V/SV bits */
    tcg_gen_add2_i32(result, cpu_PSW_C, r1, t0, carry, t0);
    tcg_gen_add2_i32(result, cpu_PSW_C, result, cpu_PSW_C, r2, t0);
    /* calc V bit */
    tcg_gen_xor_tl(cpu_PSW_V, result, r1);
    tcg_gen_xor_tl(t0, r1, r2);
    tcg_gen_andc_tl(cpu_PSW_V, cpu_PSW_V, t0);
    /* Calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV/SAV bits */
    tcg_gen_add_tl(cpu_PSW_AV, result, result);
    tcg_gen_xor_tl(cpu_PSW_AV, result, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* write back result */
    tcg_gen_mov_tl(ret, result);

    tcg_temp_free(result);
    tcg_temp_free(t0);
    tcg_temp_free(carry);
}

static inline void gen_addci_CC(TCGv ret, TCGv r1, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_addc_CC(ret, r1, temp);
    tcg_temp_free(temp);
}

static inline void gen_cond_add(TCGCond cond, TCGv r1, TCGv r2, TCGv r3,
                                TCGv r4)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    TCGv result = tcg_temp_new();
    TCGv mask = tcg_temp_new();
    TCGv t0 = tcg_const_i32(0);

    /* create mask for sticky bits */
    tcg_gen_setcond_tl(cond, mask, r4, t0);
    tcg_gen_shli_tl(mask, mask, 31);

    tcg_gen_add_tl(result, r1, r2);
    /* Calc PSW_V */
    tcg_gen_xor_tl(temp, result, r1);
    tcg_gen_xor_tl(temp2, r1, r2);
    tcg_gen_andc_tl(temp, temp, temp2);
    tcg_gen_movcond_tl(cond, cpu_PSW_V, r4, t0, temp, cpu_PSW_V);
    /* Set PSW_SV */
    tcg_gen_and_tl(temp, temp, mask);
    tcg_gen_or_tl(cpu_PSW_SV, temp, cpu_PSW_SV);
    /* calc AV bit */
    tcg_gen_add_tl(temp, result, result);
    tcg_gen_xor_tl(temp, temp, result);
    tcg_gen_movcond_tl(cond, cpu_PSW_AV, r4, t0, temp, cpu_PSW_AV);
    /* calc SAV bit */
    tcg_gen_and_tl(temp, temp, mask);
    tcg_gen_or_tl(cpu_PSW_SAV, temp, cpu_PSW_SAV);
    /* write back result */
    tcg_gen_movcond_tl(cond, r3, r4, t0, result, r1);

    tcg_temp_free(t0);
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(result);
    tcg_temp_free(mask);
}

static inline void gen_condi_add(TCGCond cond, TCGv r1, int32_t r2,
                                 TCGv r3, TCGv r4)
{
    TCGv temp = tcg_const_i32(r2);
    gen_cond_add(cond, r1, temp, r3, r4);
    tcg_temp_free(temp);
}

static inline void gen_sub_d(TCGv ret, TCGv r1, TCGv r2)
{
    TCGv temp = tcg_temp_new_i32();
    TCGv result = tcg_temp_new_i32();

    tcg_gen_sub_tl(result, r1, r2);
    /* calc V bit */
    tcg_gen_xor_tl(cpu_PSW_V, result, r1);
    tcg_gen_xor_tl(temp, r1, r2);
    tcg_gen_and_tl(cpu_PSW_V, cpu_PSW_V, temp);
    /* calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV bit */
    tcg_gen_add_tl(cpu_PSW_AV, result, result);
    tcg_gen_xor_tl(cpu_PSW_AV, result, cpu_PSW_AV);
    /* calc SAV bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* write back result */
    tcg_gen_mov_tl(ret, result);

    tcg_temp_free(temp);
    tcg_temp_free(result);
}

static inline void
gen_sub64_d(TCGv_i64 ret, TCGv_i64 r1, TCGv_i64 r2)
{
    TCGv temp = tcg_temp_new();
    TCGv_i64 t0 = tcg_temp_new_i64();
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 result = tcg_temp_new_i64();

    tcg_gen_sub_i64(result, r1, r2);
    /* calc v bit */
    tcg_gen_xor_i64(t1, result, r1);
    tcg_gen_xor_i64(t0, r1, r2);
    tcg_gen_and_i64(t1, t1, t0);
    tcg_gen_trunc_shr_i64_i32(cpu_PSW_V, t1, 32);
    /* calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* calc AV/SAV bits */
    tcg_gen_trunc_shr_i64_i32(temp, result, 32);
    tcg_gen_add_tl(cpu_PSW_AV, temp, temp);
    tcg_gen_xor_tl(cpu_PSW_AV, temp, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* write back result */
    tcg_gen_mov_i64(ret, result);

    tcg_temp_free(temp);
    tcg_temp_free_i64(result);
    tcg_temp_free_i64(t0);
    tcg_temp_free_i64(t1);
}

static inline void gen_sub_CC(TCGv ret, TCGv r1, TCGv r2)
{
    TCGv result = tcg_temp_new();
    TCGv temp = tcg_temp_new();

    tcg_gen_sub_tl(result, r1, r2);
    /* calc C bit */
    tcg_gen_setcond_tl(TCG_COND_GEU, cpu_PSW_C, r1, r2);
    /* calc V bit */
    tcg_gen_xor_tl(cpu_PSW_V, result, r1);
    tcg_gen_xor_tl(temp, r1, r2);
    tcg_gen_and_tl(cpu_PSW_V, cpu_PSW_V, temp);
    /* calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV bit */
    tcg_gen_add_tl(cpu_PSW_AV, result, result);
    tcg_gen_xor_tl(cpu_PSW_AV, result, cpu_PSW_AV);
    /* calc SAV bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* write back result */
    tcg_gen_mov_tl(ret, result);

    tcg_temp_free(result);
    tcg_temp_free(temp);
}

static inline void gen_subc_CC(TCGv ret, TCGv r1, TCGv r2)
{
    TCGv temp = tcg_temp_new();
    tcg_gen_not_tl(temp, r2);
    gen_addc_CC(ret, r1, temp);
    tcg_temp_free(temp);
}

static inline void gen_cond_sub(TCGCond cond, TCGv r1, TCGv r2, TCGv r3,
                                TCGv r4)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    TCGv result = tcg_temp_new();
    TCGv mask = tcg_temp_new();
    TCGv t0 = tcg_const_i32(0);

    /* create mask for sticky bits */
    tcg_gen_setcond_tl(cond, mask, r4, t0);
    tcg_gen_shli_tl(mask, mask, 31);

    tcg_gen_sub_tl(result, r1, r2);
    /* Calc PSW_V */
    tcg_gen_xor_tl(temp, result, r1);
    tcg_gen_xor_tl(temp2, r1, r2);
    tcg_gen_and_tl(temp, temp, temp2);
    tcg_gen_movcond_tl(cond, cpu_PSW_V, r4, t0, temp, cpu_PSW_V);
    /* Set PSW_SV */
    tcg_gen_and_tl(temp, temp, mask);
    tcg_gen_or_tl(cpu_PSW_SV, temp, cpu_PSW_SV);
    /* calc AV bit */
    tcg_gen_add_tl(temp, result, result);
    tcg_gen_xor_tl(temp, temp, result);
    tcg_gen_movcond_tl(cond, cpu_PSW_AV, r4, t0, temp, cpu_PSW_AV);
    /* calc SAV bit */
    tcg_gen_and_tl(temp, temp, mask);
    tcg_gen_or_tl(cpu_PSW_SAV, temp, cpu_PSW_SAV);
    /* write back result */
    tcg_gen_movcond_tl(cond, r3, r4, t0, result, r1);

    tcg_temp_free(t0);
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(result);
    tcg_temp_free(mask);
}

static inline void
gen_msub_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
           TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv temp2 = tcg_temp_new();
    TCGv_i64 temp64 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_extr_i64_i32(temp, temp2, temp64);
    gen_addsub64_h(ret_low, ret_high, r1_low, r1_high, temp, temp2,
                   tcg_gen_sub_tl, tcg_gen_sub_tl);
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_msubs_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
            TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv temp2 = tcg_temp_new();
    TCGv temp3 = tcg_temp_new();
    TCGv_i64 temp64 = tcg_temp_new_i64();

    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_extr_i64_i32(temp, temp2, temp64);
    gen_subs(ret_low, r1_low, temp);
    tcg_gen_mov_tl(temp, cpu_PSW_V);
    tcg_gen_mov_tl(temp3, cpu_PSW_AV);
    gen_subs(ret_high, r1_high, temp2);
    /* combine v bits */
    tcg_gen_or_tl(cpu_PSW_V, cpu_PSW_V, temp);
    /* combine av bits */
    tcg_gen_or_tl(cpu_PSW_AV, cpu_PSW_AV, temp3);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(temp3);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_msubm_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
            TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv_i64 temp64 = tcg_temp_new_i64();
    TCGv_i64 temp64_2 = tcg_temp_new_i64();
    TCGv_i64 temp64_3 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mulm_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mulm_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mulm_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mulm_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_concat_i32_i64(temp64_2, r1_low, r1_high);
    gen_sub64_d(temp64_3, temp64_2, temp64);
    /* write back result */
    tcg_gen_extr_i64_i32(ret_low, ret_high, temp64_3);

    tcg_temp_free(temp);
    tcg_temp_free_i64(temp64);
    tcg_temp_free_i64(temp64_2);
    tcg_temp_free_i64(temp64_3);
}

static inline void
gen_msubms_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
             TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv_i64 temp64 = tcg_temp_new_i64();
    TCGv_i64 temp64_2 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mulm_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mulm_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mulm_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mulm_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_concat_i32_i64(temp64_2, r1_low, r1_high);
    gen_helper_sub64_ssov(temp64, cpu_env, temp64_2, temp64);
    tcg_gen_extr_i64_i32(ret_low, ret_high, temp64);

    tcg_temp_free(temp);
    tcg_temp_free_i64(temp64);
    tcg_temp_free_i64(temp64_2);
}

static inline void
gen_msubr64_h(TCGv ret, TCGv r1_low, TCGv r1_high, TCGv r2, TCGv r3, uint32_t n,
              uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv_i64 temp64 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    gen_helper_subr_h(ret, cpu_env, temp64, r1_low, r1_high);

    tcg_temp_free(temp);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_msubr32_h(TCGv ret, TCGv r1, TCGv r2, TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();

    tcg_gen_andi_tl(temp2, r1, 0xffff0000);
    tcg_gen_shli_tl(temp, r1, 16);
    gen_msubr64_h(ret, temp, temp2, r2, r3, n, mode);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static inline void
gen_msubr64s_h(TCGv ret, TCGv r1_low, TCGv r1_high, TCGv r2, TCGv r3,
               uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv_i64 temp64 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    gen_helper_subr_h_ssov(ret, cpu_env, temp64, r1_low, r1_high);

    tcg_temp_free(temp);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_msubr32s_h(TCGv ret, TCGv r1, TCGv r2, TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();

    tcg_gen_andi_tl(temp2, r1, 0xffff0000);
    tcg_gen_shli_tl(temp, r1, 16);
    gen_msubr64s_h(ret, temp, temp2, r2, r3, n, mode);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static inline void
gen_msubr_q(TCGv ret, TCGv r1, TCGv r2, TCGv r3, uint32_t n)
{
    TCGv temp = tcg_const_i32(n);
    gen_helper_msubr_q(ret, cpu_env, r1, r2, r3, temp);
    tcg_temp_free(temp);
}

static inline void
gen_msubrs_q(TCGv ret, TCGv r1, TCGv r2, TCGv r3, uint32_t n)
{
    TCGv temp = tcg_const_i32(n);
    gen_helper_msubr_q_ssov(ret, cpu_env, r1, r2, r3, temp);
    tcg_temp_free(temp);
}

static inline void
gen_msub32_q(TCGv ret, TCGv arg1, TCGv arg2, TCGv arg3, uint32_t n,
             uint32_t up_shift, CPUTriCoreState *env)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    TCGv temp3 = tcg_temp_new();
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();
    TCGv_i64 t4 = tcg_temp_new_i64();

    tcg_gen_ext_i32_i64(t2, arg2);
    tcg_gen_ext_i32_i64(t3, arg3);

    tcg_gen_mul_i64(t2, t2, t3);

    tcg_gen_ext_i32_i64(t1, arg1);
    /* if we shift part of the fraction out, we need to round up */
    tcg_gen_andi_i64(t4, t2, (1ll << (up_shift - n)) - 1);
    tcg_gen_setcondi_i64(TCG_COND_NE, t4, t4, 0);
    tcg_gen_sari_i64(t2, t2, up_shift - n);
    tcg_gen_add_i64(t2, t2, t4);

    tcg_gen_sub_i64(t3, t1, t2);
    tcg_gen_trunc_i64_i32(temp3, t3);
    /* calc v bit */
    tcg_gen_setcondi_i64(TCG_COND_GT, t1, t3, 0x7fffffffLL);
    tcg_gen_setcondi_i64(TCG_COND_LT, t2, t3, -0x80000000LL);
    tcg_gen_or_i64(t1, t1, t2);
    tcg_gen_trunc_i64_i32(cpu_PSW_V, t1);
    tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
    /* We produce an overflow on the host if the mul before was
       (0x80000000 * 0x80000000) << 1). If this is the
       case, we negate the ovf. */
    if (n == 1) {
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp, arg2, 0x80000000);
        tcg_gen_setcond_tl(TCG_COND_EQ, temp2, arg2, arg3);
        tcg_gen_and_tl(temp, temp, temp2);
        tcg_gen_shli_tl(temp, temp, 31);
        /* negate v bit, if special condition */
        tcg_gen_xor_tl(cpu_PSW_V, cpu_PSW_V, temp);
    }
    /* Calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV/SAV bits */
    tcg_gen_add_tl(cpu_PSW_AV, temp3, temp3);
    tcg_gen_xor_tl(cpu_PSW_AV, temp3, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* write back result */
    tcg_gen_mov_tl(ret, temp3);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(temp3);
    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
    tcg_temp_free_i64(t4);
}

static inline void
gen_m16sub32_q(TCGv ret, TCGv arg1, TCGv arg2, TCGv arg3, uint32_t n)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    if (n == 0) {
        tcg_gen_mul_tl(temp, arg2, arg3);
    } else { /* n is expected to be 1 */
        tcg_gen_mul_tl(temp, arg2, arg3);
        tcg_gen_shli_tl(temp, temp, 1);
        /* catch special case r1 = r2 = 0x8000 */
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp2, temp, 0x80000000);
        tcg_gen_sub_tl(temp, temp, temp2);
    }
    gen_sub_d(ret, arg1, temp);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static inline void
gen_m16subs32_q(TCGv ret, TCGv arg1, TCGv arg2, TCGv arg3, uint32_t n)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    if (n == 0) {
        tcg_gen_mul_tl(temp, arg2, arg3);
    } else { /* n is expected to be 1 */
        tcg_gen_mul_tl(temp, arg2, arg3);
        tcg_gen_shli_tl(temp, temp, 1);
        /* catch special case r1 = r2 = 0x8000 */
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp2, temp, 0x80000000);
        tcg_gen_sub_tl(temp, temp, temp2);
    }
    gen_subs(ret, arg1, temp);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static inline void
gen_m16sub64_q(TCGv rl, TCGv rh, TCGv arg1_low, TCGv arg1_high, TCGv arg2,
               TCGv arg3, uint32_t n)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();

    if (n == 0) {
        tcg_gen_mul_tl(temp, arg2, arg3);
    } else { /* n is expected to be 1 */
        tcg_gen_mul_tl(temp, arg2, arg3);
        tcg_gen_shli_tl(temp, temp, 1);
        /* catch special case r1 = r2 = 0x8000 */
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp2, temp, 0x80000000);
        tcg_gen_sub_tl(temp, temp, temp2);
    }
    tcg_gen_ext_i32_i64(t2, temp);
    tcg_gen_shli_i64(t2, t2, 16);
    tcg_gen_concat_i32_i64(t1, arg1_low, arg1_high);
    gen_sub64_d(t3, t1, t2);
    /* write back result */
    tcg_gen_extr_i64_i32(rl, rh, t3);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static inline void
gen_m16subs64_q(TCGv rl, TCGv rh, TCGv arg1_low, TCGv arg1_high, TCGv arg2,
               TCGv arg3, uint32_t n)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();

    if (n == 0) {
        tcg_gen_mul_tl(temp, arg2, arg3);
    } else { /* n is expected to be 1 */
        tcg_gen_mul_tl(temp, arg2, arg3);
        tcg_gen_shli_tl(temp, temp, 1);
        /* catch special case r1 = r2 = 0x8000 */
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp2, temp, 0x80000000);
        tcg_gen_sub_tl(temp, temp, temp2);
    }
    tcg_gen_ext_i32_i64(t2, temp);
    tcg_gen_shli_i64(t2, t2, 16);
    tcg_gen_concat_i32_i64(t1, arg1_low, arg1_high);

    gen_helper_sub64_ssov(t1, cpu_env, t1, t2);
    tcg_gen_extr_i64_i32(rl, rh, t1);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
}

static inline void
gen_msub64_q(TCGv rl, TCGv rh, TCGv arg1_low, TCGv arg1_high, TCGv arg2,
             TCGv arg3, uint32_t n, CPUTriCoreState *env)
{
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();
    TCGv_i64 t4 = tcg_temp_new_i64();
    TCGv temp, temp2;

    tcg_gen_concat_i32_i64(t1, arg1_low, arg1_high);
    tcg_gen_ext_i32_i64(t2, arg2);
    tcg_gen_ext_i32_i64(t3, arg3);

    tcg_gen_mul_i64(t2, t2, t3);
    if (n != 0) {
        tcg_gen_shli_i64(t2, t2, 1);
    }
    tcg_gen_sub_i64(t4, t1, t2);
    /* calc v bit */
    tcg_gen_xor_i64(t3, t4, t1);
    tcg_gen_xor_i64(t2, t1, t2);
    tcg_gen_and_i64(t3, t3, t2);
    tcg_gen_trunc_shr_i64_i32(cpu_PSW_V, t3, 32);
    /* We produce an overflow on the host if the mul before was
       (0x80000000 * 0x80000000) << 1). If this is the
       case, we negate the ovf. */
    if (n == 1) {
        temp = tcg_temp_new();
        temp2 = tcg_temp_new();
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp, arg2, 0x80000000);
        tcg_gen_setcond_tl(TCG_COND_EQ, temp2, arg2, arg3);
        tcg_gen_and_tl(temp, temp, temp2);
        tcg_gen_shli_tl(temp, temp, 31);
        /* negate v bit, if special condition */
        tcg_gen_xor_tl(cpu_PSW_V, cpu_PSW_V, temp);

        tcg_temp_free(temp);
        tcg_temp_free(temp2);
    }
    /* write back result */
    tcg_gen_extr_i64_i32(rl, rh, t4);
    /* Calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV/SAV bits */
    tcg_gen_add_tl(cpu_PSW_AV, rh, rh);
    tcg_gen_xor_tl(cpu_PSW_AV, rh, cpu_PSW_AV);
    /* calc SAV */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
    tcg_temp_free_i64(t4);
}

static inline void
gen_msubs32_q(TCGv ret, TCGv arg1, TCGv arg2, TCGv arg3, uint32_t n,
              uint32_t up_shift)
{
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();
    TCGv_i64 t4 = tcg_temp_new_i64();

    tcg_gen_ext_i32_i64(t1, arg1);
    tcg_gen_ext_i32_i64(t2, arg2);
    tcg_gen_ext_i32_i64(t3, arg3);

    tcg_gen_mul_i64(t2, t2, t3);
    /* if we shift part of the fraction out, we need to round up */
    tcg_gen_andi_i64(t4, t2, (1ll << (up_shift - n)) - 1);
    tcg_gen_setcondi_i64(TCG_COND_NE, t4, t4, 0);
    tcg_gen_sari_i64(t3, t2, up_shift - n);
    tcg_gen_add_i64(t3, t3, t4);

    gen_helper_msub32_q_sub_ssov(ret, cpu_env, t1, t3);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
    tcg_temp_free_i64(t4);
}

static inline void
gen_msubs64_q(TCGv rl, TCGv rh, TCGv arg1_low, TCGv arg1_high, TCGv arg2,
             TCGv arg3, uint32_t n)
{
    TCGv_i64 r1 = tcg_temp_new_i64();
    TCGv temp = tcg_const_i32(n);

    tcg_gen_concat_i32_i64(r1, arg1_low, arg1_high);
    gen_helper_msub64_q_ssov(r1, cpu_env, r1, arg2, arg3, temp);
    tcg_gen_extr_i64_i32(rl, rh, r1);

    tcg_temp_free_i64(r1);
    tcg_temp_free(temp);
}

static inline void
gen_msubad_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
             TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv temp2 = tcg_temp_new();
    TCGv_i64 temp64 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_extr_i64_i32(temp, temp2, temp64);
    gen_addsub64_h(ret_low, ret_high, r1_low, r1_high, temp, temp2,
                   tcg_gen_add_tl, tcg_gen_sub_tl);
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_msubadm_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
              TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv_i64 temp64 = tcg_temp_new_i64();
    TCGv_i64 temp64_2 = tcg_temp_new_i64();
    TCGv_i64 temp64_3 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_concat_i32_i64(temp64_3, r1_low, r1_high);
    tcg_gen_sari_i64(temp64_2, temp64, 32); /* high */
    tcg_gen_ext32s_i64(temp64, temp64); /* low */
    tcg_gen_sub_i64(temp64, temp64_2, temp64);
    tcg_gen_shli_i64(temp64, temp64, 16);

    gen_sub64_d(temp64_2, temp64_3, temp64);
    /* write back result */
    tcg_gen_extr_i64_i32(ret_low, ret_high, temp64_2);

    tcg_temp_free(temp);
    tcg_temp_free_i64(temp64);
    tcg_temp_free_i64(temp64_2);
    tcg_temp_free_i64(temp64_3);
}

static inline void
gen_msubadr32_h(TCGv ret, TCGv r1, TCGv r2, TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv temp2 = tcg_temp_new();
    TCGv_i64 temp64 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_andi_tl(temp2, r1, 0xffff0000);
    tcg_gen_shli_tl(temp, r1, 16);
    gen_helper_subadr_h(ret, cpu_env, temp64, temp, temp2);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_msubads_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
              TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv temp2 = tcg_temp_new();
    TCGv temp3 = tcg_temp_new();
    TCGv_i64 temp64 = tcg_temp_new_i64();

    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_extr_i64_i32(temp, temp2, temp64);
    gen_adds(ret_low, r1_low, temp);
    tcg_gen_mov_tl(temp, cpu_PSW_V);
    tcg_gen_mov_tl(temp3, cpu_PSW_AV);
    gen_subs(ret_high, r1_high, temp2);
    /* combine v bits */
    tcg_gen_or_tl(cpu_PSW_V, cpu_PSW_V, temp);
    /* combine av bits */
    tcg_gen_or_tl(cpu_PSW_AV, cpu_PSW_AV, temp3);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(temp3);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_msubadms_h(TCGv ret_low, TCGv ret_high, TCGv r1_low, TCGv r1_high, TCGv r2,
               TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv_i64 temp64 = tcg_temp_new_i64();
    TCGv_i64 temp64_2 = tcg_temp_new_i64();

    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_sari_i64(temp64_2, temp64, 32); /* high */
    tcg_gen_ext32s_i64(temp64, temp64); /* low */
    tcg_gen_sub_i64(temp64, temp64_2, temp64);
    tcg_gen_shli_i64(temp64, temp64, 16);
    tcg_gen_concat_i32_i64(temp64_2, r1_low, r1_high);

    gen_helper_sub64_ssov(temp64, cpu_env, temp64_2, temp64);
    tcg_gen_extr_i64_i32(ret_low, ret_high, temp64);

    tcg_temp_free(temp);
    tcg_temp_free_i64(temp64);
    tcg_temp_free_i64(temp64_2);
}

static inline void
gen_msubadr32s_h(TCGv ret, TCGv r1, TCGv r2, TCGv r3, uint32_t n, uint32_t mode)
{
    TCGv temp = tcg_const_i32(n);
    TCGv temp2 = tcg_temp_new();
    TCGv_i64 temp64 = tcg_temp_new_i64();
    switch (mode) {
    case MODE_LL:
        GEN_HELPER_LL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_LU:
        GEN_HELPER_LU(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UL:
        GEN_HELPER_UL(mul_h, temp64, r2, r3, temp);
        break;
    case MODE_UU:
        GEN_HELPER_UU(mul_h, temp64, r2, r3, temp);
        break;
    }
    tcg_gen_andi_tl(temp2, r1, 0xffff0000);
    tcg_gen_shli_tl(temp, r1, 16);
    gen_helper_subadr_h_ssov(ret, cpu_env, temp64, temp, temp2);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free_i64(temp64);
}

static inline void gen_abs(TCGv ret, TCGv r1)
{
    TCGv temp = tcg_temp_new();
    TCGv t0 = tcg_const_i32(0);

    tcg_gen_neg_tl(temp, r1);
    tcg_gen_movcond_tl(TCG_COND_GE, ret, r1, t0, r1, temp);
    /* overflow can only happen, if r1 = 0x80000000 */
    tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_PSW_V, r1, 0x80000000);
    tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
    /* calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV bit */
    tcg_gen_add_tl(cpu_PSW_AV, ret, ret);
    tcg_gen_xor_tl(cpu_PSW_AV, ret, cpu_PSW_AV);
    /* calc SAV bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);

    tcg_temp_free(temp);
    tcg_temp_free(t0);
}

static inline void gen_absdif(TCGv ret, TCGv r1, TCGv r2)
{
    TCGv temp = tcg_temp_new_i32();
    TCGv result = tcg_temp_new_i32();

    tcg_gen_sub_tl(result, r1, r2);
    tcg_gen_sub_tl(temp, r2, r1);
    tcg_gen_movcond_tl(TCG_COND_GT, result, r1, r2, result, temp);

    /* calc V bit */
    tcg_gen_xor_tl(cpu_PSW_V, result, r1);
    tcg_gen_xor_tl(temp, result, r2);
    tcg_gen_movcond_tl(TCG_COND_GT, cpu_PSW_V, r1, r2, cpu_PSW_V, temp);
    tcg_gen_xor_tl(temp, r1, r2);
    tcg_gen_and_tl(cpu_PSW_V, cpu_PSW_V, temp);
    /* calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV bit */
    tcg_gen_add_tl(cpu_PSW_AV, result, result);
    tcg_gen_xor_tl(cpu_PSW_AV, result, cpu_PSW_AV);
    /* calc SAV bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* write back result */
    tcg_gen_mov_tl(ret, result);

    tcg_temp_free(temp);
    tcg_temp_free(result);
}

static inline void gen_absdifi(TCGv ret, TCGv r1, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_absdif(ret, r1, temp);
    tcg_temp_free(temp);
}

static inline void gen_absdifsi(TCGv ret, TCGv r1, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_helper_absdif_ssov(ret, cpu_env, r1, temp);
    tcg_temp_free(temp);
}

static inline void gen_mul_i32s(TCGv ret, TCGv r1, TCGv r2)
{
    TCGv high = tcg_temp_new();
    TCGv low = tcg_temp_new();

    tcg_gen_muls2_tl(low, high, r1, r2);
    tcg_gen_mov_tl(ret, low);
    /* calc V bit */
    tcg_gen_sari_tl(low, low, 31);
    tcg_gen_setcond_tl(TCG_COND_NE, cpu_PSW_V, high, low);
    tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
    /* calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV bit */
    tcg_gen_add_tl(cpu_PSW_AV, ret, ret);
    tcg_gen_xor_tl(cpu_PSW_AV, ret, cpu_PSW_AV);
    /* calc SAV bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);

    tcg_temp_free(high);
    tcg_temp_free(low);
}

static inline void gen_muli_i32s(TCGv ret, TCGv r1, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_mul_i32s(ret, r1, temp);
    tcg_temp_free(temp);
}

static inline void gen_mul_i64s(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2)
{
    tcg_gen_muls2_tl(ret_low, ret_high, r1, r2);
    /* clear V bit */
    tcg_gen_movi_tl(cpu_PSW_V, 0);
    /* calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV bit */
    tcg_gen_add_tl(cpu_PSW_AV, ret_high, ret_high);
    tcg_gen_xor_tl(cpu_PSW_AV, ret_high, cpu_PSW_AV);
    /* calc SAV bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
}

static inline void gen_muli_i64s(TCGv ret_low, TCGv ret_high, TCGv r1,
                                int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_mul_i64s(ret_low, ret_high, r1, temp);
    tcg_temp_free(temp);
}

static inline void gen_mul_i64u(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2)
{
    tcg_gen_mulu2_tl(ret_low, ret_high, r1, r2);
    /* clear V bit */
    tcg_gen_movi_tl(cpu_PSW_V, 0);
    /* calc SV bit */
    tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    /* Calc AV bit */
    tcg_gen_add_tl(cpu_PSW_AV, ret_high, ret_high);
    tcg_gen_xor_tl(cpu_PSW_AV, ret_high, cpu_PSW_AV);
    /* calc SAV bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
}

static inline void gen_muli_i64u(TCGv ret_low, TCGv ret_high, TCGv r1,
                                int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_mul_i64u(ret_low, ret_high, r1, temp);
    tcg_temp_free(temp);
}

static inline void gen_mulsi_i32(TCGv ret, TCGv r1, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_helper_mul_ssov(ret, cpu_env, r1, temp);
    tcg_temp_free(temp);
}

static inline void gen_mulsui_i32(TCGv ret, TCGv r1, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_helper_mul_suov(ret, cpu_env, r1, temp);
    tcg_temp_free(temp);
}
/* gen_maddsi_32(cpu_gpr_d[r4], cpu_gpr_d[r1], cpu_gpr_d[r3], const9); */
static inline void gen_maddsi_32(TCGv ret, TCGv r1, TCGv r2, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_helper_madd32_ssov(ret, cpu_env, r1, r2, temp);
    tcg_temp_free(temp);
}

static inline void gen_maddsui_32(TCGv ret, TCGv r1, TCGv r2, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_helper_madd32_suov(ret, cpu_env, r1, r2, temp);
    tcg_temp_free(temp);
}

static void
gen_mul_q(TCGv rl, TCGv rh, TCGv arg1, TCGv arg2, uint32_t n, uint32_t up_shift)
{
    TCGv temp = tcg_temp_new();
    TCGv_i64 temp_64 = tcg_temp_new_i64();
    TCGv_i64 temp2_64 = tcg_temp_new_i64();

    if (n == 0) {
        if (up_shift == 32) {
            tcg_gen_muls2_tl(rh, rl, arg1, arg2);
        } else if (up_shift == 16) {
            tcg_gen_ext_i32_i64(temp_64, arg1);
            tcg_gen_ext_i32_i64(temp2_64, arg2);

            tcg_gen_mul_i64(temp_64, temp_64, temp2_64);
            tcg_gen_shri_i64(temp_64, temp_64, up_shift);
            tcg_gen_extr_i64_i32(rl, rh, temp_64);
        } else {
            tcg_gen_muls2_tl(rl, rh, arg1, arg2);
        }
        /* reset v bit */
        tcg_gen_movi_tl(cpu_PSW_V, 0);
    } else { /* n is expected to be 1 */
        tcg_gen_ext_i32_i64(temp_64, arg1);
        tcg_gen_ext_i32_i64(temp2_64, arg2);

        tcg_gen_mul_i64(temp_64, temp_64, temp2_64);

        if (up_shift == 0) {
            tcg_gen_shli_i64(temp_64, temp_64, 1);
        } else {
            tcg_gen_shri_i64(temp_64, temp_64, up_shift - 1);
        }
        tcg_gen_extr_i64_i32(rl, rh, temp_64);
        /* overflow only occurs if r1 = r2 = 0x8000 */
        if (up_shift == 0) {/* result is 64 bit */
            tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_PSW_V, rh,
                                0x80000000);
        } else { /* result is 32 bit */
            tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_PSW_V, rl,
                                0x80000000);
        }
        tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
        /* calc sv overflow bit */
        tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
    }
    /* calc av overflow bit */
    if (up_shift == 0) {
        tcg_gen_add_tl(cpu_PSW_AV, rh, rh);
        tcg_gen_xor_tl(cpu_PSW_AV, rh, cpu_PSW_AV);
    } else {
        tcg_gen_add_tl(cpu_PSW_AV, rl, rl);
        tcg_gen_xor_tl(cpu_PSW_AV, rl, cpu_PSW_AV);
    }
    /* calc sav overflow bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    tcg_temp_free(temp);
    tcg_temp_free_i64(temp_64);
    tcg_temp_free_i64(temp2_64);
}

static void
gen_mul_q_16(TCGv ret, TCGv arg1, TCGv arg2, uint32_t n)
{
    TCGv temp = tcg_temp_new();
    if (n == 0) {
        tcg_gen_mul_tl(ret, arg1, arg2);
    } else { /* n is expected to be 1 */
        tcg_gen_mul_tl(ret, arg1, arg2);
        tcg_gen_shli_tl(ret, ret, 1);
        /* catch special case r1 = r2 = 0x8000 */
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp, ret, 0x80000000);
        tcg_gen_sub_tl(ret, ret, temp);
    }
    /* reset v bit */
    tcg_gen_movi_tl(cpu_PSW_V, 0);
    /* calc av overflow bit */
    tcg_gen_add_tl(cpu_PSW_AV, ret, ret);
    tcg_gen_xor_tl(cpu_PSW_AV, ret, cpu_PSW_AV);
    /* calc sav overflow bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);

    tcg_temp_free(temp);
}

static void gen_mulr_q(TCGv ret, TCGv arg1, TCGv arg2, uint32_t n)
{
    TCGv temp = tcg_temp_new();
    if (n == 0) {
        tcg_gen_mul_tl(ret, arg1, arg2);
        tcg_gen_addi_tl(ret, ret, 0x8000);
    } else {
        tcg_gen_mul_tl(ret, arg1, arg2);
        tcg_gen_shli_tl(ret, ret, 1);
        tcg_gen_addi_tl(ret, ret, 0x8000);
        /* catch special case r1 = r2 = 0x8000 */
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp, ret, 0x80008000);
        tcg_gen_muli_tl(temp, temp, 0x8001);
        tcg_gen_sub_tl(ret, ret, temp);
    }
    /* reset v bit */
    tcg_gen_movi_tl(cpu_PSW_V, 0);
    /* calc av overflow bit */
    tcg_gen_add_tl(cpu_PSW_AV, ret, ret);
    tcg_gen_xor_tl(cpu_PSW_AV, ret, cpu_PSW_AV);
    /* calc sav overflow bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* cut halfword off */
    tcg_gen_andi_tl(ret, ret, 0xffff0000);

    tcg_temp_free(temp);
}

static inline void
gen_madds_64(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
             TCGv r3)
{
    TCGv_i64 temp64 = tcg_temp_new_i64();
    tcg_gen_concat_i32_i64(temp64, r2_low, r2_high);
    gen_helper_madd64_ssov(temp64, cpu_env, r1, temp64, r3);
    tcg_gen_extr_i64_i32(ret_low, ret_high, temp64);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_maddsi_64(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
              int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_madds_64(ret_low, ret_high, r1, r2_low, r2_high, temp);
    tcg_temp_free(temp);
}

static inline void
gen_maddsu_64(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
             TCGv r3)
{
    TCGv_i64 temp64 = tcg_temp_new_i64();
    tcg_gen_concat_i32_i64(temp64, r2_low, r2_high);
    gen_helper_madd64_suov(temp64, cpu_env, r1, temp64, r3);
    tcg_gen_extr_i64_i32(ret_low, ret_high, temp64);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_maddsui_64(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
               int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_maddsu_64(ret_low, ret_high, r1, r2_low, r2_high, temp);
    tcg_temp_free(temp);
}

static inline void gen_msubsi_32(TCGv ret, TCGv r1, TCGv r2, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_helper_msub32_ssov(ret, cpu_env, r1, r2, temp);
    tcg_temp_free(temp);
}

static inline void gen_msubsui_32(TCGv ret, TCGv r1, TCGv r2, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_helper_msub32_suov(ret, cpu_env, r1, r2, temp);
    tcg_temp_free(temp);
}

static inline void
gen_msubs_64(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
             TCGv r3)
{
    TCGv_i64 temp64 = tcg_temp_new_i64();
    tcg_gen_concat_i32_i64(temp64, r2_low, r2_high);
    gen_helper_msub64_ssov(temp64, cpu_env, r1, temp64, r3);
    tcg_gen_extr_i64_i32(ret_low, ret_high, temp64);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_msubsi_64(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
              int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_msubs_64(ret_low, ret_high, r1, r2_low, r2_high, temp);
    tcg_temp_free(temp);
}

static inline void
gen_msubsu_64(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
             TCGv r3)
{
    TCGv_i64 temp64 = tcg_temp_new_i64();
    tcg_gen_concat_i32_i64(temp64, r2_low, r2_high);
    gen_helper_msub64_suov(temp64, cpu_env, r1, temp64, r3);
    tcg_gen_extr_i64_i32(ret_low, ret_high, temp64);
    tcg_temp_free_i64(temp64);
}

static inline void
gen_msubsui_64(TCGv ret_low, TCGv ret_high, TCGv r1, TCGv r2_low, TCGv r2_high,
               int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_msubsu_64(ret_low, ret_high, r1, r2_low, r2_high, temp);
    tcg_temp_free(temp);
}

static void gen_saturate(TCGv ret, TCGv arg, int32_t up, int32_t low)
{
    TCGv sat_neg = tcg_const_i32(low);
    TCGv temp = tcg_const_i32(up);

    /* sat_neg = (arg < low ) ? low : arg; */
    tcg_gen_movcond_tl(TCG_COND_LT, sat_neg, arg, sat_neg, sat_neg, arg);

    /* ret = (sat_neg > up ) ? up  : sat_neg; */
    tcg_gen_movcond_tl(TCG_COND_GT, ret, sat_neg, temp, temp, sat_neg);

    tcg_temp_free(sat_neg);
    tcg_temp_free(temp);
}

static void gen_saturate_u(TCGv ret, TCGv arg, int32_t up)
{
    TCGv temp = tcg_const_i32(up);
    /* sat_neg = (arg > up ) ? up : arg; */
    tcg_gen_movcond_tl(TCG_COND_GTU, ret, arg, temp, temp, arg);
    tcg_temp_free(temp);
}

static void gen_shi(TCGv ret, TCGv r1, int32_t shift_count)
{
    if (shift_count == -32) {
        tcg_gen_movi_tl(ret, 0);
    } else if (shift_count >= 0) {
        tcg_gen_shli_tl(ret, r1, shift_count);
    } else {
        tcg_gen_shri_tl(ret, r1, -shift_count);
    }
}

static void gen_sh_hi(TCGv ret, TCGv r1, int32_t shiftcount)
{
    TCGv temp_low, temp_high;

    if (shiftcount == -16) {
        tcg_gen_movi_tl(ret, 0);
    } else {
        temp_high = tcg_temp_new();
        temp_low = tcg_temp_new();

        tcg_gen_andi_tl(temp_low, r1, 0xffff);
        tcg_gen_andi_tl(temp_high, r1, 0xffff0000);
        gen_shi(temp_low, temp_low, shiftcount);
        gen_shi(ret, temp_high, shiftcount);
        tcg_gen_deposit_tl(ret, ret, temp_low, 0, 16);

        tcg_temp_free(temp_low);
        tcg_temp_free(temp_high);
    }
}

static void gen_shaci(TCGv ret, TCGv r1, int32_t shift_count)
{
    uint32_t msk, msk_start;
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    TCGv t_0 = tcg_const_i32(0);

    if (shift_count == 0) {
        /* Clear PSW.C and PSW.V */
        tcg_gen_movi_tl(cpu_PSW_C, 0);
        tcg_gen_mov_tl(cpu_PSW_V, cpu_PSW_C);
        tcg_gen_mov_tl(ret, r1);
    } else if (shift_count == -32) {
        /* set PSW.C */
        tcg_gen_mov_tl(cpu_PSW_C, r1);
        /* fill ret completly with sign bit */
        tcg_gen_sari_tl(ret, r1, 31);
        /* clear PSW.V */
        tcg_gen_movi_tl(cpu_PSW_V, 0);
    } else if (shift_count > 0) {
        TCGv t_max = tcg_const_i32(0x7FFFFFFF >> shift_count);
        TCGv t_min = tcg_const_i32(((int32_t) -0x80000000) >> shift_count);

        /* calc carry */
        msk_start = 32 - shift_count;
        msk = ((1 << shift_count) - 1) << msk_start;
        tcg_gen_andi_tl(cpu_PSW_C, r1, msk);
        /* calc v/sv bits */
        tcg_gen_setcond_tl(TCG_COND_GT, temp, r1, t_max);
        tcg_gen_setcond_tl(TCG_COND_LT, temp2, r1, t_min);
        tcg_gen_or_tl(cpu_PSW_V, temp, temp2);
        tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
        /* calc sv */
        tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_V, cpu_PSW_SV);
        /* do shift */
        tcg_gen_shli_tl(ret, r1, shift_count);

        tcg_temp_free(t_max);
        tcg_temp_free(t_min);
    } else {
        /* clear PSW.V */
        tcg_gen_movi_tl(cpu_PSW_V, 0);
        /* calc carry */
        msk = (1 << -shift_count) - 1;
        tcg_gen_andi_tl(cpu_PSW_C, r1, msk);
        /* do shift */
        tcg_gen_sari_tl(ret, r1, -shift_count);
    }
    /* calc av overflow bit */
    tcg_gen_add_tl(cpu_PSW_AV, ret, ret);
    tcg_gen_xor_tl(cpu_PSW_AV, ret, cpu_PSW_AV);
    /* calc sav overflow bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(t_0);
}

static void gen_shas(TCGv ret, TCGv r1, TCGv r2)
{
    gen_helper_sha_ssov(ret, cpu_env, r1, r2);
}

static void gen_shasi(TCGv ret, TCGv r1, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_shas(ret, r1, temp);
    tcg_temp_free(temp);
}

static void gen_sha_hi(TCGv ret, TCGv r1, int32_t shift_count)
{
    TCGv low, high;

    if (shift_count == 0) {
        tcg_gen_mov_tl(ret, r1);
    } else if (shift_count > 0) {
        low = tcg_temp_new();
        high = tcg_temp_new();

        tcg_gen_andi_tl(high, r1, 0xffff0000);
        tcg_gen_shli_tl(low, r1, shift_count);
        tcg_gen_shli_tl(ret, high, shift_count);
        tcg_gen_deposit_tl(ret, ret, low, 0, 16);

        tcg_temp_free(low);
        tcg_temp_free(high);
    } else {
        low = tcg_temp_new();
        high = tcg_temp_new();

        tcg_gen_ext16s_tl(low, r1);
        tcg_gen_sari_tl(low, low, -shift_count);
        tcg_gen_sari_tl(ret, r1, -shift_count);
        tcg_gen_deposit_tl(ret, ret, low, 0, 16);

        tcg_temp_free(low);
        tcg_temp_free(high);
    }

}

/* ret = {ret[30:0], (r1 cond r2)}; */
static void gen_sh_cond(int cond, TCGv ret, TCGv r1, TCGv r2)
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();

    tcg_gen_shli_tl(temp, ret, 1);
    tcg_gen_setcond_tl(cond, temp2, r1, r2);
    tcg_gen_or_tl(ret, temp, temp2);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static void gen_sh_condi(int cond, TCGv ret, TCGv r1, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_sh_cond(cond, ret, r1, temp);
    tcg_temp_free(temp);
}

static inline void gen_adds(TCGv ret, TCGv r1, TCGv r2)
{
    gen_helper_add_ssov(ret, cpu_env, r1, r2);
}

static inline void gen_addsi(TCGv ret, TCGv r1, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_helper_add_ssov(ret, cpu_env, r1, temp);
    tcg_temp_free(temp);
}

static inline void gen_addsui(TCGv ret, TCGv r1, int32_t con)
{
    TCGv temp = tcg_const_i32(con);
    gen_helper_add_suov(ret, cpu_env, r1, temp);
    tcg_temp_free(temp);
}

static inline void gen_subs(TCGv ret, TCGv r1, TCGv r2)
{
    gen_helper_sub_ssov(ret, cpu_env, r1, r2);
}

static inline void gen_subsu(TCGv ret, TCGv r1, TCGv r2)
{
    gen_helper_sub_suov(ret, cpu_env, r1, r2);
}

static inline void gen_bit_2op(TCGv ret, TCGv r1, TCGv r2,
                               int pos1, int pos2,
                               void(*op1)(TCGv, TCGv, TCGv),
                               void(*op2)(TCGv, TCGv, TCGv))
{
    TCGv temp1, temp2;

    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();

    tcg_gen_shri_tl(temp2, r2, pos2);
    tcg_gen_shri_tl(temp1, r1, pos1);

    (*op1)(temp1, temp1, temp2);
    (*op2)(temp1 , ret, temp1);

    tcg_gen_deposit_tl(ret, ret, temp1, 0, 1);

    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
}

/* ret = r1[pos1] op1 r2[pos2]; */
static inline void gen_bit_1op(TCGv ret, TCGv r1, TCGv r2,
                               int pos1, int pos2,
                               void(*op1)(TCGv, TCGv, TCGv))
{
    TCGv temp1, temp2;

    temp1 = tcg_temp_new();
    temp2 = tcg_temp_new();

    tcg_gen_shri_tl(temp2, r2, pos2);
    tcg_gen_shri_tl(temp1, r1, pos1);

    (*op1)(ret, temp1, temp2);

    tcg_gen_andi_tl(ret, ret, 0x1);

    tcg_temp_free(temp1);
    tcg_temp_free(temp2);
}

static inline void gen_accumulating_cond(int cond, TCGv ret, TCGv r1, TCGv r2,
                                         void(*op)(TCGv, TCGv, TCGv))
{
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();
    /* temp = (arg1 cond arg2 )*/
    tcg_gen_setcond_tl(cond, temp, r1, r2);
    /* temp2 = ret[0]*/
    tcg_gen_andi_tl(temp2, ret, 0x1);
    /* temp = temp insn temp2 */
    (*op)(temp, temp, temp2);
    /* ret = {ret[31:1], temp} */
    tcg_gen_deposit_tl(ret, ret, temp, 0, 1);

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static inline void
gen_accumulating_condi(int cond, TCGv ret, TCGv r1, int32_t con,
                       void(*op)(TCGv, TCGv, TCGv))
{
    TCGv temp = tcg_const_i32(con);
    gen_accumulating_cond(cond, ret, r1, temp, op);
    tcg_temp_free(temp);
}

/* ret = (r1 cond r2) ? 0xFFFFFFFF ? 0x00000000;*/
static inline void gen_cond_w(TCGCond cond, TCGv ret, TCGv r1, TCGv r2)
{
    tcg_gen_setcond_tl(cond, ret, r1, r2);
    tcg_gen_neg_tl(ret, ret);
}

static inline void gen_eqany_bi(TCGv ret, TCGv r1, int32_t con)
{
    TCGv b0 = tcg_temp_new();
    TCGv b1 = tcg_temp_new();
    TCGv b2 = tcg_temp_new();
    TCGv b3 = tcg_temp_new();

    /* byte 0 */
    tcg_gen_andi_tl(b0, r1, 0xff);
    tcg_gen_setcondi_tl(TCG_COND_EQ, b0, b0, con & 0xff);

    /* byte 1 */
    tcg_gen_andi_tl(b1, r1, 0xff00);
    tcg_gen_setcondi_tl(TCG_COND_EQ, b1, b1, con & 0xff00);

    /* byte 2 */
    tcg_gen_andi_tl(b2, r1, 0xff0000);
    tcg_gen_setcondi_tl(TCG_COND_EQ, b2, b2, con & 0xff0000);

    /* byte 3 */
    tcg_gen_andi_tl(b3, r1, 0xff000000);
    tcg_gen_setcondi_tl(TCG_COND_EQ, b3, b3, con & 0xff000000);

    /* combine them */
    tcg_gen_or_tl(ret, b0, b1);
    tcg_gen_or_tl(ret, ret, b2);
    tcg_gen_or_tl(ret, ret, b3);

    tcg_temp_free(b0);
    tcg_temp_free(b1);
    tcg_temp_free(b2);
    tcg_temp_free(b3);
}

static inline void gen_eqany_hi(TCGv ret, TCGv r1, int32_t con)
{
    TCGv h0 = tcg_temp_new();
    TCGv h1 = tcg_temp_new();

    /* halfword 0 */
    tcg_gen_andi_tl(h0, r1, 0xffff);
    tcg_gen_setcondi_tl(TCG_COND_EQ, h0, h0, con & 0xffff);

    /* halfword 1 */
    tcg_gen_andi_tl(h1, r1, 0xffff0000);
    tcg_gen_setcondi_tl(TCG_COND_EQ, h1, h1, con & 0xffff0000);

    /* combine them */
    tcg_gen_or_tl(ret, h0, h1);

    tcg_temp_free(h0);
    tcg_temp_free(h1);
}
/* mask = ((1 << width) -1) << pos;
   ret = (r1 & ~mask) | (r2 << pos) & mask); */
static inline void gen_insert(TCGv ret, TCGv r1, TCGv r2, TCGv width, TCGv pos)
{
    TCGv mask = tcg_temp_new();
    TCGv temp = tcg_temp_new();
    TCGv temp2 = tcg_temp_new();

    tcg_gen_movi_tl(mask, 1);
    tcg_gen_shl_tl(mask, mask, width);
    tcg_gen_subi_tl(mask, mask, 1);
    tcg_gen_shl_tl(mask, mask, pos);

    tcg_gen_shl_tl(temp, r2, pos);
    tcg_gen_and_tl(temp, temp, mask);
    tcg_gen_andc_tl(temp2, r1, mask);
    tcg_gen_or_tl(ret, temp, temp2);

    tcg_temp_free(mask);
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static inline void gen_bsplit(TCGv rl, TCGv rh, TCGv r1)
{
    TCGv_i64 temp = tcg_temp_new_i64();

    gen_helper_bsplit(temp, r1);
    tcg_gen_extr_i64_i32(rl, rh, temp);

    tcg_temp_free_i64(temp);
}

static inline void gen_unpack(TCGv rl, TCGv rh, TCGv r1)
{
    TCGv_i64 temp = tcg_temp_new_i64();

    gen_helper_unpack(temp, r1);
    tcg_gen_extr_i64_i32(rl, rh, temp);

    tcg_temp_free_i64(temp);
}

static inline void
gen_dvinit_b(CPUTriCoreState *env, TCGv rl, TCGv rh, TCGv r1, TCGv r2)
{
    TCGv_i64 ret = tcg_temp_new_i64();

    if (!tricore_feature(env, TRICORE_FEATURE_131)) {
        gen_helper_dvinit_b_13(ret, cpu_env, r1, r2);
    } else {
        gen_helper_dvinit_b_131(ret, cpu_env, r1, r2);
    }
    tcg_gen_extr_i64_i32(rl, rh, ret);

    tcg_temp_free_i64(ret);
}

static inline void
gen_dvinit_h(CPUTriCoreState *env, TCGv rl, TCGv rh, TCGv r1, TCGv r2)
{
    TCGv_i64 ret = tcg_temp_new_i64();

    if (!tricore_feature(env, TRICORE_FEATURE_131)) {
        gen_helper_dvinit_h_13(ret, cpu_env, r1, r2);
    } else {
        gen_helper_dvinit_h_131(ret, cpu_env, r1, r2);
    }
    tcg_gen_extr_i64_i32(rl, rh, ret);

    tcg_temp_free_i64(ret);
}

static void gen_calc_usb_mul_h(TCGv arg_low, TCGv arg_high)
{
    TCGv temp = tcg_temp_new();
    /* calc AV bit */
    tcg_gen_add_tl(temp, arg_low, arg_low);
    tcg_gen_xor_tl(temp, temp, arg_low);
    tcg_gen_add_tl(cpu_PSW_AV, arg_high, arg_high);
    tcg_gen_xor_tl(cpu_PSW_AV, cpu_PSW_AV, arg_high);
    tcg_gen_or_tl(cpu_PSW_AV, cpu_PSW_AV, temp);
    /* calc SAV bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    tcg_gen_movi_tl(cpu_PSW_V, 0);
    tcg_temp_free(temp);
}

static void gen_calc_usb_mulr_h(TCGv arg)
{
    TCGv temp = tcg_temp_new();
    /* calc AV bit */
    tcg_gen_add_tl(temp, arg, arg);
    tcg_gen_xor_tl(temp, temp, arg);
    tcg_gen_shli_tl(cpu_PSW_AV, temp, 16);
    tcg_gen_or_tl(cpu_PSW_AV, cpu_PSW_AV, temp);
    /* calc SAV bit */
    tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
    /* clear V bit */
    tcg_gen_movi_tl(cpu_PSW_V, 0);
    tcg_temp_free(temp);
}

/* helpers for generating program flow micro-ops */

static inline void gen_save_pc(target_ulong pc)
{
    tcg_gen_movi_tl(cpu_PC, pc);
}

static inline void gen_goto_tb(DisasContext *ctx, int n, target_ulong dest)
{
    TranslationBlock *tb;
    tb = ctx->tb;
    if ((tb->pc & TARGET_PAGE_MASK) == (dest & TARGET_PAGE_MASK) &&
        likely(!ctx->singlestep_enabled)) {
        tcg_gen_goto_tb(n);
        gen_save_pc(dest);
        tcg_gen_exit_tb((uintptr_t)tb + n);
    } else {
        gen_save_pc(dest);
        if (ctx->singlestep_enabled) {
            /* raise exception debug */
        }
        tcg_gen_exit_tb(0);
    }
}

static inline void gen_branch_cond(DisasContext *ctx, TCGCond cond, TCGv r1,
                                   TCGv r2, int16_t address)
{
    TCGLabel *jumpLabel = gen_new_label();
    tcg_gen_brcond_tl(cond, r1, r2, jumpLabel);

    gen_goto_tb(ctx, 1, ctx->next_pc);

    gen_set_label(jumpLabel);
    gen_goto_tb(ctx, 0, ctx->pc + address * 2);
}

static inline void gen_branch_condi(DisasContext *ctx, TCGCond cond, TCGv r1,
                                    int r2, int16_t address)
{
    TCGv temp = tcg_const_i32(r2);
    gen_branch_cond(ctx, cond, r1, temp, address);
    tcg_temp_free(temp);
}

static void gen_loop(DisasContext *ctx, int r1, int32_t offset)
{
    TCGLabel *l1 = gen_new_label();

    tcg_gen_subi_tl(cpu_gpr_a[r1], cpu_gpr_a[r1], 1);
    tcg_gen_brcondi_tl(TCG_COND_EQ, cpu_gpr_a[r1], -1, l1);
    gen_goto_tb(ctx, 1, ctx->pc + offset);
    gen_set_label(l1);
    gen_goto_tb(ctx, 0, ctx->next_pc);
}

static void gen_compute_branch(DisasContext *ctx, uint32_t opc, int r1,
                               int r2 , int32_t constant , int32_t offset)
{
    TCGv temp, temp2;
    int n;

    switch (opc) {
/* SB-format jumps */
    case OPC1_16_SB_J:
    case OPC1_32_B_J:
        gen_goto_tb(ctx, 0, ctx->pc + offset * 2);
        break;
    case OPC1_32_B_CALL:
    case OPC1_16_SB_CALL:
        gen_helper_1arg(call, ctx->next_pc);
        gen_goto_tb(ctx, 0, ctx->pc + offset * 2);
        break;
    case OPC1_16_SB_JZ:
        gen_branch_condi(ctx, TCG_COND_EQ, cpu_gpr_d[15], 0, offset);
        break;
    case OPC1_16_SB_JNZ:
        gen_branch_condi(ctx, TCG_COND_NE, cpu_gpr_d[15], 0, offset);
        break;
/* SBC-format jumps */
    case OPC1_16_SBC_JEQ:
        gen_branch_condi(ctx, TCG_COND_EQ, cpu_gpr_d[15], constant, offset);
        break;
    case OPC1_16_SBC_JNE:
        gen_branch_condi(ctx, TCG_COND_NE, cpu_gpr_d[15], constant, offset);
        break;
/* SBRN-format jumps */
    case OPC1_16_SBRN_JZ_T:
        temp = tcg_temp_new();
        tcg_gen_andi_tl(temp, cpu_gpr_d[15], 0x1u << constant);
        gen_branch_condi(ctx, TCG_COND_EQ, temp, 0, offset);
        tcg_temp_free(temp);
        break;
    case OPC1_16_SBRN_JNZ_T:
        temp = tcg_temp_new();
        tcg_gen_andi_tl(temp, cpu_gpr_d[15], 0x1u << constant);
        gen_branch_condi(ctx, TCG_COND_NE, temp, 0, offset);
        tcg_temp_free(temp);
        break;
/* SBR-format jumps */
    case OPC1_16_SBR_JEQ:
        gen_branch_cond(ctx, TCG_COND_EQ, cpu_gpr_d[r1], cpu_gpr_d[15],
                        offset);
        break;
    case OPC1_16_SBR_JNE:
        gen_branch_cond(ctx, TCG_COND_NE, cpu_gpr_d[r1], cpu_gpr_d[15],
                        offset);
        break;
    case OPC1_16_SBR_JNZ:
        gen_branch_condi(ctx, TCG_COND_NE, cpu_gpr_d[r1], 0, offset);
        break;
    case OPC1_16_SBR_JNZ_A:
        gen_branch_condi(ctx, TCG_COND_NE, cpu_gpr_a[r1], 0, offset);
        break;
    case OPC1_16_SBR_JGEZ:
        gen_branch_condi(ctx, TCG_COND_GE, cpu_gpr_d[r1], 0, offset);
        break;
    case OPC1_16_SBR_JGTZ:
        gen_branch_condi(ctx, TCG_COND_GT, cpu_gpr_d[r1], 0, offset);
        break;
    case OPC1_16_SBR_JLEZ:
        gen_branch_condi(ctx, TCG_COND_LE, cpu_gpr_d[r1], 0, offset);
        break;
    case OPC1_16_SBR_JLTZ:
        gen_branch_condi(ctx, TCG_COND_LT, cpu_gpr_d[r1], 0, offset);
        break;
    case OPC1_16_SBR_JZ:
        gen_branch_condi(ctx, TCG_COND_EQ, cpu_gpr_d[r1], 0, offset);
        break;
    case OPC1_16_SBR_JZ_A:
        gen_branch_condi(ctx, TCG_COND_EQ, cpu_gpr_a[r1], 0, offset);
        break;
    case OPC1_16_SBR_LOOP:
        gen_loop(ctx, r1, offset * 2 - 32);
        break;
/* SR-format jumps */
    case OPC1_16_SR_JI:
        tcg_gen_andi_tl(cpu_PC, cpu_gpr_a[r1], 0xfffffffe);
        tcg_gen_exit_tb(0);
        break;
    case OPC2_32_SYS_RET:
    case OPC2_16_SR_RET:
        gen_helper_ret(cpu_env);
        tcg_gen_exit_tb(0);
        break;
/* B-format */
    case OPC1_32_B_CALLA:
        gen_helper_1arg(call, ctx->next_pc);
        gen_goto_tb(ctx, 0, EA_B_ABSOLUT(offset));
        break;
    case OPC1_32_B_JLA:
        tcg_gen_movi_tl(cpu_gpr_a[11], ctx->next_pc);
        /* fall through */
    case OPC1_32_B_JA:
        gen_goto_tb(ctx, 0, EA_B_ABSOLUT(offset));
        break;
    case OPC1_32_B_JL:
        tcg_gen_movi_tl(cpu_gpr_a[11], ctx->next_pc);
        gen_goto_tb(ctx, 0, ctx->pc + offset * 2);
        break;
/* BOL format */
    case OPCM_32_BRC_EQ_NEQ:
         if (MASK_OP_BRC_OP2(ctx->opcode) == OPC2_32_BRC_JEQ) {
            gen_branch_condi(ctx, TCG_COND_EQ, cpu_gpr_d[r1], constant, offset);
         } else {
            gen_branch_condi(ctx, TCG_COND_NE, cpu_gpr_d[r1], constant, offset);
         }
         break;
    case OPCM_32_BRC_GE:
         if (MASK_OP_BRC_OP2(ctx->opcode) == OP2_32_BRC_JGE) {
            gen_branch_condi(ctx, TCG_COND_GE, cpu_gpr_d[r1], constant, offset);
         } else {
            constant = MASK_OP_BRC_CONST4(ctx->opcode);
            gen_branch_condi(ctx, TCG_COND_GEU, cpu_gpr_d[r1], constant,
                             offset);
         }
         break;
    case OPCM_32_BRC_JLT:
         if (MASK_OP_BRC_OP2(ctx->opcode) == OPC2_32_BRC_JLT) {
            gen_branch_condi(ctx, TCG_COND_LT, cpu_gpr_d[r1], constant, offset);
         } else {
            constant = MASK_OP_BRC_CONST4(ctx->opcode);
            gen_branch_condi(ctx, TCG_COND_LTU, cpu_gpr_d[r1], constant,
                             offset);
         }
         break;
    case OPCM_32_BRC_JNE:
        temp = tcg_temp_new();
        if (MASK_OP_BRC_OP2(ctx->opcode) == OPC2_32_BRC_JNED) {
            tcg_gen_mov_tl(temp, cpu_gpr_d[r1]);
            /* subi is unconditional */
            tcg_gen_subi_tl(cpu_gpr_d[r1], cpu_gpr_d[r1], 1);
            gen_branch_condi(ctx, TCG_COND_NE, temp, constant, offset);
        } else {
            tcg_gen_mov_tl(temp, cpu_gpr_d[r1]);
            /* addi is unconditional */
            tcg_gen_addi_tl(cpu_gpr_d[r1], cpu_gpr_d[r1], 1);
            gen_branch_condi(ctx, TCG_COND_NE, temp, constant, offset);
        }
        tcg_temp_free(temp);
        break;
/* BRN format */
    case OPCM_32_BRN_JTT:
        n = MASK_OP_BRN_N(ctx->opcode);

        temp = tcg_temp_new();
        tcg_gen_andi_tl(temp, cpu_gpr_d[r1], (1 << n));

        if (MASK_OP_BRN_OP2(ctx->opcode) == OPC2_32_BRN_JNZ_T) {
            gen_branch_condi(ctx, TCG_COND_NE, temp, 0, offset);
        } else {
            gen_branch_condi(ctx, TCG_COND_EQ, temp, 0, offset);
        }
        tcg_temp_free(temp);
        break;
/* BRR Format */
    case OPCM_32_BRR_EQ_NEQ:
        if (MASK_OP_BRR_OP2(ctx->opcode) == OPC2_32_BRR_JEQ) {
            gen_branch_cond(ctx, TCG_COND_EQ, cpu_gpr_d[r1], cpu_gpr_d[r2],
                            offset);
        } else {
            gen_branch_cond(ctx, TCG_COND_NE, cpu_gpr_d[r1], cpu_gpr_d[r2],
                            offset);
        }
        break;
    case OPCM_32_BRR_ADDR_EQ_NEQ:
        if (MASK_OP_BRR_OP2(ctx->opcode) == OPC2_32_BRR_JEQ_A) {
            gen_branch_cond(ctx, TCG_COND_EQ, cpu_gpr_a[r1], cpu_gpr_a[r2],
                            offset);
        } else {
            gen_branch_cond(ctx, TCG_COND_NE, cpu_gpr_a[r1], cpu_gpr_a[r2],
                            offset);
        }
        break;
    case OPCM_32_BRR_GE:
        if (MASK_OP_BRR_OP2(ctx->opcode) == OPC2_32_BRR_JGE) {
            gen_branch_cond(ctx, TCG_COND_GE, cpu_gpr_d[r1], cpu_gpr_d[r2],
                            offset);
        } else {
            gen_branch_cond(ctx, TCG_COND_GEU, cpu_gpr_d[r1], cpu_gpr_d[r2],
                            offset);
        }
        break;
    case OPCM_32_BRR_JLT:
        if (MASK_OP_BRR_OP2(ctx->opcode) == OPC2_32_BRR_JLT) {
            gen_branch_cond(ctx, TCG_COND_LT, cpu_gpr_d[r1], cpu_gpr_d[r2],
                            offset);
        } else {
            gen_branch_cond(ctx, TCG_COND_LTU, cpu_gpr_d[r1], cpu_gpr_d[r2],
                            offset);
        }
        break;
    case OPCM_32_BRR_LOOP:
        if (MASK_OP_BRR_OP2(ctx->opcode) == OPC2_32_BRR_LOOP) {
            gen_loop(ctx, r2, offset * 2);
        } else {
            /* OPC2_32_BRR_LOOPU */
            gen_goto_tb(ctx, 0, ctx->pc + offset * 2);
        }
        break;
    case OPCM_32_BRR_JNE:
        temp = tcg_temp_new();
        temp2 = tcg_temp_new();
        if (MASK_OP_BRC_OP2(ctx->opcode) == OPC2_32_BRR_JNED) {
            tcg_gen_mov_tl(temp, cpu_gpr_d[r1]);
            /* also save r2, in case of r1 == r2, so r2 is not decremented */
            tcg_gen_mov_tl(temp2, cpu_gpr_d[r2]);
            /* subi is unconditional */
            tcg_gen_subi_tl(cpu_gpr_d[r1], cpu_gpr_d[r1], 1);
            gen_branch_cond(ctx, TCG_COND_NE, temp, temp2, offset);
        } else {
            tcg_gen_mov_tl(temp, cpu_gpr_d[r1]);
            /* also save r2, in case of r1 == r2, so r2 is not decremented */
            tcg_gen_mov_tl(temp2, cpu_gpr_d[r2]);
            /* addi is unconditional */
            tcg_gen_addi_tl(cpu_gpr_d[r1], cpu_gpr_d[r1], 1);
            gen_branch_cond(ctx, TCG_COND_NE, temp, temp2, offset);
        }
        tcg_temp_free(temp);
        tcg_temp_free(temp2);
        break;
    case OPCM_32_BRR_JNZ:
        if (MASK_OP_BRR_OP2(ctx->opcode) == OPC2_32_BRR_JNZ_A) {
            gen_branch_condi(ctx, TCG_COND_NE, cpu_gpr_a[r1], 0, offset);
        } else {
            gen_branch_condi(ctx, TCG_COND_EQ, cpu_gpr_a[r1], 0, offset);
        }
        break;
    default:
        printf("Branch Error at %x\n", ctx->pc);
    }
    ctx->bstate = BS_BRANCH;
}


/*
 * Functions for decoding instructions
 */

static void decode_src_opc(CPUTriCoreState *env, DisasContext *ctx, int op1)
{
    int r1;
    int32_t const4;
    TCGv temp, temp2;

    r1 = MASK_OP_SRC_S1D(ctx->opcode);
    const4 = MASK_OP_SRC_CONST4_SEXT(ctx->opcode);

    switch (op1) {
    case OPC1_16_SRC_ADD:
        gen_addi_d(cpu_gpr_d[r1], cpu_gpr_d[r1], const4);
        break;
    case OPC1_16_SRC_ADD_A15:
        gen_addi_d(cpu_gpr_d[r1], cpu_gpr_d[15], const4);
        break;
    case OPC1_16_SRC_ADD_15A:
        gen_addi_d(cpu_gpr_d[15], cpu_gpr_d[r1], const4);
        break;
    case OPC1_16_SRC_ADD_A:
        tcg_gen_addi_tl(cpu_gpr_a[r1], cpu_gpr_a[r1], const4);
        break;
    case OPC1_16_SRC_CADD:
        gen_condi_add(TCG_COND_NE, cpu_gpr_d[r1], const4, cpu_gpr_d[r1],
                      cpu_gpr_d[15]);
        break;
    case OPC1_16_SRC_CADDN:
        gen_condi_add(TCG_COND_EQ, cpu_gpr_d[r1], const4, cpu_gpr_d[r1],
                      cpu_gpr_d[15]);
        break;
    case OPC1_16_SRC_CMOV:
        temp = tcg_const_tl(0);
        temp2 = tcg_const_tl(const4);
        tcg_gen_movcond_tl(TCG_COND_NE, cpu_gpr_d[r1], cpu_gpr_d[15], temp,
                           temp2, cpu_gpr_d[r1]);
        tcg_temp_free(temp);
        tcg_temp_free(temp2);
        break;
    case OPC1_16_SRC_CMOVN:
        temp = tcg_const_tl(0);
        temp2 = tcg_const_tl(const4);
        tcg_gen_movcond_tl(TCG_COND_EQ, cpu_gpr_d[r1], cpu_gpr_d[15], temp,
                           temp2, cpu_gpr_d[r1]);
        tcg_temp_free(temp);
        tcg_temp_free(temp2);
        break;
    case OPC1_16_SRC_EQ:
        tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_gpr_d[15], cpu_gpr_d[r1],
                            const4);
        break;
    case OPC1_16_SRC_LT:
        tcg_gen_setcondi_tl(TCG_COND_LT, cpu_gpr_d[15], cpu_gpr_d[r1],
                            const4);
        break;
    case OPC1_16_SRC_MOV:
        tcg_gen_movi_tl(cpu_gpr_d[r1], const4);
        break;
    case OPC1_16_SRC_MOV_A:
        const4 = MASK_OP_SRC_CONST4(ctx->opcode);
        tcg_gen_movi_tl(cpu_gpr_a[r1], const4);
        break;
    case OPC1_16_SRC_MOV_E:
        if (tricore_feature(env, TRICORE_FEATURE_16)) {
            tcg_gen_movi_tl(cpu_gpr_d[r1], const4);
            tcg_gen_sari_tl(cpu_gpr_d[r1+1], cpu_gpr_d[r1], 31);
        } /* TODO: else raise illegal opcode trap */
        break;
    case OPC1_16_SRC_SH:
        gen_shi(cpu_gpr_d[r1], cpu_gpr_d[r1], const4);
        break;
    case OPC1_16_SRC_SHA:
        gen_shaci(cpu_gpr_d[r1], cpu_gpr_d[r1], const4);
        break;
    }
}

static void decode_srr_opc(DisasContext *ctx, int op1)
{
    int r1, r2;
    TCGv temp;

    r1 = MASK_OP_SRR_S1D(ctx->opcode);
    r2 = MASK_OP_SRR_S2(ctx->opcode);

    switch (op1) {
    case OPC1_16_SRR_ADD:
        gen_add_d(cpu_gpr_d[r1], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_ADD_A15:
        gen_add_d(cpu_gpr_d[r1], cpu_gpr_d[15], cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_ADD_15A:
        gen_add_d(cpu_gpr_d[15], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_ADD_A:
        tcg_gen_add_tl(cpu_gpr_a[r1], cpu_gpr_a[r1], cpu_gpr_a[r2]);
        break;
    case OPC1_16_SRR_ADDS:
        gen_adds(cpu_gpr_d[r1], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_AND:
        tcg_gen_and_tl(cpu_gpr_d[r1], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_CMOV:
        temp = tcg_const_tl(0);
        tcg_gen_movcond_tl(TCG_COND_NE, cpu_gpr_d[r1], cpu_gpr_d[15], temp,
                           cpu_gpr_d[r2], cpu_gpr_d[r1]);
        tcg_temp_free(temp);
        break;
    case OPC1_16_SRR_CMOVN:
        temp = tcg_const_tl(0);
        tcg_gen_movcond_tl(TCG_COND_EQ, cpu_gpr_d[r1], cpu_gpr_d[15], temp,
                           cpu_gpr_d[r2], cpu_gpr_d[r1]);
        tcg_temp_free(temp);
        break;
    case OPC1_16_SRR_EQ:
        tcg_gen_setcond_tl(TCG_COND_EQ, cpu_gpr_d[15], cpu_gpr_d[r1],
                           cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_LT:
        tcg_gen_setcond_tl(TCG_COND_LT, cpu_gpr_d[15], cpu_gpr_d[r1],
                           cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_MOV:
        tcg_gen_mov_tl(cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_MOV_A:
        tcg_gen_mov_tl(cpu_gpr_a[r1], cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_MOV_AA:
        tcg_gen_mov_tl(cpu_gpr_a[r1], cpu_gpr_a[r2]);
        break;
    case OPC1_16_SRR_MOV_D:
        tcg_gen_mov_tl(cpu_gpr_d[r1], cpu_gpr_a[r2]);
        break;
    case OPC1_16_SRR_MUL:
        gen_mul_i32s(cpu_gpr_d[r1], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_OR:
        tcg_gen_or_tl(cpu_gpr_d[r1], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_SUB:
        gen_sub_d(cpu_gpr_d[r1], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_SUB_A15B:
        gen_sub_d(cpu_gpr_d[r1], cpu_gpr_d[15], cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_SUB_15AB:
        gen_sub_d(cpu_gpr_d[15], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_SUBS:
        gen_subs(cpu_gpr_d[r1], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC1_16_SRR_XOR:
        tcg_gen_xor_tl(cpu_gpr_d[r1], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    }
}

static void decode_ssr_opc(DisasContext *ctx, int op1)
{
    int r1, r2;

    r1 = MASK_OP_SSR_S1(ctx->opcode);
    r2 = MASK_OP_SSR_S2(ctx->opcode);

    switch (op1) {
    case OPC1_16_SSR_ST_A:
        tcg_gen_qemu_st_tl(cpu_gpr_a[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_LEUL);
        break;
    case OPC1_16_SSR_ST_A_POSTINC:
        tcg_gen_qemu_st_tl(cpu_gpr_a[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_LEUL);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], 4);
        break;
    case OPC1_16_SSR_ST_B:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_UB);
        break;
    case OPC1_16_SSR_ST_B_POSTINC:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_UB);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], 1);
        break;
    case OPC1_16_SSR_ST_H:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_LEUW);
        break;
    case OPC1_16_SSR_ST_H_POSTINC:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_LEUW);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], 2);
        break;
    case OPC1_16_SSR_ST_W:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_LEUL);
        break;
    case OPC1_16_SSR_ST_W_POSTINC:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_LEUL);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], 4);
        break;
    }
}

static void decode_sc_opc(DisasContext *ctx, int op1)
{
    int32_t const16;

    const16 = MASK_OP_SC_CONST8(ctx->opcode);

    switch (op1) {
    case OPC1_16_SC_AND:
        tcg_gen_andi_tl(cpu_gpr_d[15], cpu_gpr_d[15], const16);
        break;
    case OPC1_16_SC_BISR:
        gen_helper_1arg(bisr, const16 & 0xff);
        break;
    case OPC1_16_SC_LD_A:
        gen_offset_ld(ctx, cpu_gpr_a[15], cpu_gpr_a[10], const16 * 4, MO_LESL);
        break;
    case OPC1_16_SC_LD_W:
        gen_offset_ld(ctx, cpu_gpr_d[15], cpu_gpr_a[10], const16 * 4, MO_LESL);
        break;
    case OPC1_16_SC_MOV:
        tcg_gen_movi_tl(cpu_gpr_d[15], const16);
        break;
    case OPC1_16_SC_OR:
        tcg_gen_ori_tl(cpu_gpr_d[15], cpu_gpr_d[15], const16);
        break;
    case OPC1_16_SC_ST_A:
        gen_offset_st(ctx, cpu_gpr_a[15], cpu_gpr_a[10], const16 * 4, MO_LESL);
        break;
    case OPC1_16_SC_ST_W:
        gen_offset_st(ctx, cpu_gpr_d[15], cpu_gpr_a[10], const16 * 4, MO_LESL);
        break;
    case OPC1_16_SC_SUB_A:
        tcg_gen_subi_tl(cpu_gpr_a[10], cpu_gpr_a[10], const16);
        break;
    }
}

static void decode_slr_opc(DisasContext *ctx, int op1)
{
    int r1, r2;

    r1 = MASK_OP_SLR_D(ctx->opcode);
    r2 = MASK_OP_SLR_S2(ctx->opcode);

    switch (op1) {
/* SLR-format */
    case OPC1_16_SLR_LD_A:
        tcg_gen_qemu_ld_tl(cpu_gpr_a[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_LESL);
        break;
    case OPC1_16_SLR_LD_A_POSTINC:
        tcg_gen_qemu_ld_tl(cpu_gpr_a[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_LESL);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], 4);
        break;
    case OPC1_16_SLR_LD_BU:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_UB);
        break;
    case OPC1_16_SLR_LD_BU_POSTINC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_UB);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], 1);
        break;
    case OPC1_16_SLR_LD_H:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_LESW);
        break;
    case OPC1_16_SLR_LD_H_POSTINC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_LESW);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], 2);
        break;
    case OPC1_16_SLR_LD_W:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_LESL);
        break;
    case OPC1_16_SLR_LD_W_POSTINC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx, MO_LESL);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], 4);
        break;
    }
}

static void decode_sro_opc(DisasContext *ctx, int op1)
{
    int r2;
    int32_t address;

    r2 = MASK_OP_SRO_S2(ctx->opcode);
    address = MASK_OP_SRO_OFF4(ctx->opcode);

/* SRO-format */
    switch (op1) {
    case OPC1_16_SRO_LD_A:
        gen_offset_ld(ctx, cpu_gpr_a[15], cpu_gpr_a[r2], address * 4, MO_LESL);
        break;
    case OPC1_16_SRO_LD_BU:
        gen_offset_ld(ctx, cpu_gpr_d[15], cpu_gpr_a[r2], address, MO_UB);
        break;
    case OPC1_16_SRO_LD_H:
        gen_offset_ld(ctx, cpu_gpr_d[15], cpu_gpr_a[r2], address, MO_LESW);
        break;
    case OPC1_16_SRO_LD_W:
        gen_offset_ld(ctx, cpu_gpr_d[15], cpu_gpr_a[r2], address * 4, MO_LESL);
        break;
    case OPC1_16_SRO_ST_A:
        gen_offset_st(ctx, cpu_gpr_a[15], cpu_gpr_a[r2], address * 4, MO_LESL);
        break;
    case OPC1_16_SRO_ST_B:
        gen_offset_st(ctx, cpu_gpr_d[15], cpu_gpr_a[r2], address, MO_UB);
        break;
    case OPC1_16_SRO_ST_H:
        gen_offset_st(ctx, cpu_gpr_d[15], cpu_gpr_a[r2], address * 2, MO_LESW);
        break;
    case OPC1_16_SRO_ST_W:
        gen_offset_st(ctx, cpu_gpr_d[15], cpu_gpr_a[r2], address * 4, MO_LESL);
        break;
    }
}

static void decode_sr_system(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    op2 = MASK_OP_SR_OP2(ctx->opcode);

    switch (op2) {
    case OPC2_16_SR_NOP:
        break;
    case OPC2_16_SR_RET:
        gen_compute_branch(ctx, op2, 0, 0, 0, 0);
        break;
    case OPC2_16_SR_RFE:
        gen_helper_rfe(cpu_env);
        tcg_gen_exit_tb(0);
        ctx->bstate = BS_BRANCH;
        break;
    case OPC2_16_SR_DEBUG:
        /* raise EXCP_DEBUG */
        break;
    }
}

static void decode_sr_accu(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    uint32_t r1;
    TCGv temp;

    r1 = MASK_OP_SR_S1D(ctx->opcode);
    op2 = MASK_OP_SR_OP2(ctx->opcode);

    switch (op2) {
    case OPC2_16_SR_RSUB:
        /* overflow only if r1 = -0x80000000 */
        temp = tcg_const_i32(-0x80000000);
        /* calc V bit */
        tcg_gen_setcond_tl(TCG_COND_EQ, cpu_PSW_V, cpu_gpr_d[r1], temp);
        tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
        /* calc SV bit */
        tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
        /* sub */
        tcg_gen_neg_tl(cpu_gpr_d[r1], cpu_gpr_d[r1]);
        /* calc av */
        tcg_gen_add_tl(cpu_PSW_AV, cpu_gpr_d[r1], cpu_gpr_d[r1]);
        tcg_gen_xor_tl(cpu_PSW_AV, cpu_gpr_d[r1], cpu_PSW_AV);
        /* calc sav */
        tcg_gen_or_tl(cpu_PSW_SAV, cpu_PSW_SAV, cpu_PSW_AV);
        tcg_temp_free(temp);
        break;
    case OPC2_16_SR_SAT_B:
        gen_saturate(cpu_gpr_d[r1], cpu_gpr_d[r1], 0x7f, -0x80);
        break;
    case OPC2_16_SR_SAT_BU:
        gen_saturate_u(cpu_gpr_d[r1], cpu_gpr_d[r1], 0xff);
        break;
    case OPC2_16_SR_SAT_H:
        gen_saturate(cpu_gpr_d[r1], cpu_gpr_d[r1], 0x7fff, -0x8000);
        break;
    case OPC2_16_SR_SAT_HU:
        gen_saturate_u(cpu_gpr_d[r1], cpu_gpr_d[r1], 0xffff);
        break;
    }
}

static void decode_16Bit_opc(CPUTriCoreState *env, DisasContext *ctx)
{
    int op1;
    int r1, r2;
    int32_t const16;
    int32_t address;
    TCGv temp;

    op1 = MASK_OP_MAJOR(ctx->opcode);

    /* handle ADDSC.A opcode only being 6 bit long */
    if (unlikely((op1 & 0x3f) == OPC1_16_SRRS_ADDSC_A)) {
        op1 = OPC1_16_SRRS_ADDSC_A;
    }

    switch (op1) {
    case OPC1_16_SRC_ADD:
    case OPC1_16_SRC_ADD_A15:
    case OPC1_16_SRC_ADD_15A:
    case OPC1_16_SRC_ADD_A:
    case OPC1_16_SRC_CADD:
    case OPC1_16_SRC_CADDN:
    case OPC1_16_SRC_CMOV:
    case OPC1_16_SRC_CMOVN:
    case OPC1_16_SRC_EQ:
    case OPC1_16_SRC_LT:
    case OPC1_16_SRC_MOV:
    case OPC1_16_SRC_MOV_A:
    case OPC1_16_SRC_MOV_E:
    case OPC1_16_SRC_SH:
    case OPC1_16_SRC_SHA:
        decode_src_opc(env, ctx, op1);
        break;
/* SRR-format */
    case OPC1_16_SRR_ADD:
    case OPC1_16_SRR_ADD_A15:
    case OPC1_16_SRR_ADD_15A:
    case OPC1_16_SRR_ADD_A:
    case OPC1_16_SRR_ADDS:
    case OPC1_16_SRR_AND:
    case OPC1_16_SRR_CMOV:
    case OPC1_16_SRR_CMOVN:
    case OPC1_16_SRR_EQ:
    case OPC1_16_SRR_LT:
    case OPC1_16_SRR_MOV:
    case OPC1_16_SRR_MOV_A:
    case OPC1_16_SRR_MOV_AA:
    case OPC1_16_SRR_MOV_D:
    case OPC1_16_SRR_MUL:
    case OPC1_16_SRR_OR:
    case OPC1_16_SRR_SUB:
    case OPC1_16_SRR_SUB_A15B:
    case OPC1_16_SRR_SUB_15AB:
    case OPC1_16_SRR_SUBS:
    case OPC1_16_SRR_XOR:
        decode_srr_opc(ctx, op1);
        break;
/* SSR-format */
    case OPC1_16_SSR_ST_A:
    case OPC1_16_SSR_ST_A_POSTINC:
    case OPC1_16_SSR_ST_B:
    case OPC1_16_SSR_ST_B_POSTINC:
    case OPC1_16_SSR_ST_H:
    case OPC1_16_SSR_ST_H_POSTINC:
    case OPC1_16_SSR_ST_W:
    case OPC1_16_SSR_ST_W_POSTINC:
        decode_ssr_opc(ctx, op1);
        break;
/* SRRS-format */
    case OPC1_16_SRRS_ADDSC_A:
        r2 = MASK_OP_SRRS_S2(ctx->opcode);
        r1 = MASK_OP_SRRS_S1D(ctx->opcode);
        const16 = MASK_OP_SRRS_N(ctx->opcode);
        temp = tcg_temp_new();
        tcg_gen_shli_tl(temp, cpu_gpr_d[15], const16);
        tcg_gen_add_tl(cpu_gpr_a[r1], cpu_gpr_a[r2], temp);
        tcg_temp_free(temp);
        break;
/* SLRO-format */
    case OPC1_16_SLRO_LD_A:
        r1 = MASK_OP_SLRO_D(ctx->opcode);
        const16 = MASK_OP_SLRO_OFF4(ctx->opcode);
        gen_offset_ld(ctx, cpu_gpr_a[r1], cpu_gpr_a[15], const16 * 4, MO_LESL);
        break;
    case OPC1_16_SLRO_LD_BU:
        r1 = MASK_OP_SLRO_D(ctx->opcode);
        const16 = MASK_OP_SLRO_OFF4(ctx->opcode);
        gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[15], const16, MO_UB);
        break;
    case OPC1_16_SLRO_LD_H:
        r1 = MASK_OP_SLRO_D(ctx->opcode);
        const16 = MASK_OP_SLRO_OFF4(ctx->opcode);
        gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[15], const16 * 2, MO_LESW);
        break;
    case OPC1_16_SLRO_LD_W:
        r1 = MASK_OP_SLRO_D(ctx->opcode);
        const16 = MASK_OP_SLRO_OFF4(ctx->opcode);
        gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[15], const16 * 4, MO_LESL);
        break;
/* SB-format */
    case OPC1_16_SB_CALL:
    case OPC1_16_SB_J:
    case OPC1_16_SB_JNZ:
    case OPC1_16_SB_JZ:
        address = MASK_OP_SB_DISP8_SEXT(ctx->opcode);
        gen_compute_branch(ctx, op1, 0, 0, 0, address);
        break;
/* SBC-format */
    case OPC1_16_SBC_JEQ:
    case OPC1_16_SBC_JNE:
        address = MASK_OP_SBC_DISP4(ctx->opcode);
        const16 = MASK_OP_SBC_CONST4_SEXT(ctx->opcode);
        gen_compute_branch(ctx, op1, 0, 0, const16, address);
        break;
/* SBRN-format */
    case OPC1_16_SBRN_JNZ_T:
    case OPC1_16_SBRN_JZ_T:
        address = MASK_OP_SBRN_DISP4(ctx->opcode);
        const16 = MASK_OP_SBRN_N(ctx->opcode);
        gen_compute_branch(ctx, op1, 0, 0, const16, address);
        break;
/* SBR-format */
    case OPC1_16_SBR_JEQ:
    case OPC1_16_SBR_JGEZ:
    case OPC1_16_SBR_JGTZ:
    case OPC1_16_SBR_JLEZ:
    case OPC1_16_SBR_JLTZ:
    case OPC1_16_SBR_JNE:
    case OPC1_16_SBR_JNZ:
    case OPC1_16_SBR_JNZ_A:
    case OPC1_16_SBR_JZ:
    case OPC1_16_SBR_JZ_A:
    case OPC1_16_SBR_LOOP:
        r1 = MASK_OP_SBR_S2(ctx->opcode);
        address = MASK_OP_SBR_DISP4(ctx->opcode);
        gen_compute_branch(ctx, op1, r1, 0, 0, address);
        break;
/* SC-format */
    case OPC1_16_SC_AND:
    case OPC1_16_SC_BISR:
    case OPC1_16_SC_LD_A:
    case OPC1_16_SC_LD_W:
    case OPC1_16_SC_MOV:
    case OPC1_16_SC_OR:
    case OPC1_16_SC_ST_A:
    case OPC1_16_SC_ST_W:
    case OPC1_16_SC_SUB_A:
        decode_sc_opc(ctx, op1);
        break;
/* SLR-format */
    case OPC1_16_SLR_LD_A:
    case OPC1_16_SLR_LD_A_POSTINC:
    case OPC1_16_SLR_LD_BU:
    case OPC1_16_SLR_LD_BU_POSTINC:
    case OPC1_16_SLR_LD_H:
    case OPC1_16_SLR_LD_H_POSTINC:
    case OPC1_16_SLR_LD_W:
    case OPC1_16_SLR_LD_W_POSTINC:
        decode_slr_opc(ctx, op1);
        break;
/* SRO-format */
    case OPC1_16_SRO_LD_A:
    case OPC1_16_SRO_LD_BU:
    case OPC1_16_SRO_LD_H:
    case OPC1_16_SRO_LD_W:
    case OPC1_16_SRO_ST_A:
    case OPC1_16_SRO_ST_B:
    case OPC1_16_SRO_ST_H:
    case OPC1_16_SRO_ST_W:
        decode_sro_opc(ctx, op1);
        break;
/* SSRO-format */
    case OPC1_16_SSRO_ST_A:
        r1 = MASK_OP_SSRO_S1(ctx->opcode);
        const16 = MASK_OP_SSRO_OFF4(ctx->opcode);
        gen_offset_st(ctx, cpu_gpr_a[r1], cpu_gpr_a[15], const16 * 4, MO_LESL);
        break;
    case OPC1_16_SSRO_ST_B:
        r1 = MASK_OP_SSRO_S1(ctx->opcode);
        const16 = MASK_OP_SSRO_OFF4(ctx->opcode);
        gen_offset_st(ctx, cpu_gpr_d[r1], cpu_gpr_a[15], const16, MO_UB);
        break;
    case OPC1_16_SSRO_ST_H:
        r1 = MASK_OP_SSRO_S1(ctx->opcode);
        const16 = MASK_OP_SSRO_OFF4(ctx->opcode);
        gen_offset_st(ctx, cpu_gpr_d[r1], cpu_gpr_a[15], const16 * 2, MO_LESW);
        break;
    case OPC1_16_SSRO_ST_W:
        r1 = MASK_OP_SSRO_S1(ctx->opcode);
        const16 = MASK_OP_SSRO_OFF4(ctx->opcode);
        gen_offset_st(ctx, cpu_gpr_d[r1], cpu_gpr_a[15], const16 * 4, MO_LESL);
        break;
/* SR-format */
    case OPCM_16_SR_SYSTEM:
        decode_sr_system(env, ctx);
        break;
    case OPCM_16_SR_ACCU:
        decode_sr_accu(env, ctx);
        break;
    case OPC1_16_SR_JI:
        r1 = MASK_OP_SR_S1D(ctx->opcode);
        gen_compute_branch(ctx, op1, r1, 0, 0, 0);
        break;
    case OPC1_16_SR_NOT:
        r1 = MASK_OP_SR_S1D(ctx->opcode);
        tcg_gen_not_tl(cpu_gpr_d[r1], cpu_gpr_d[r1]);
        break;
    }
}

/*
 * 32 bit instructions
 */

/* ABS-format */
static void decode_abs_ldw(CPUTriCoreState *env, DisasContext *ctx)
{
    int32_t op2;
    int32_t r1;
    uint32_t address;
    TCGv temp;

    r1 = MASK_OP_ABS_S1D(ctx->opcode);
    address = MASK_OP_ABS_OFF18(ctx->opcode);
    op2 = MASK_OP_ABS_OP2(ctx->opcode);

    temp = tcg_const_i32(EA_ABS_FORMAT(address));

    switch (op2) {
    case OPC2_32_ABS_LD_A:
        tcg_gen_qemu_ld_tl(cpu_gpr_a[r1], temp, ctx->mem_idx, MO_LESL);
        break;
    case OPC2_32_ABS_LD_D:
        gen_ld_2regs_64(cpu_gpr_d[r1+1], cpu_gpr_d[r1], temp, ctx);
        break;
    case OPC2_32_ABS_LD_DA:
        gen_ld_2regs_64(cpu_gpr_a[r1+1], cpu_gpr_a[r1], temp, ctx);
        break;
    case OPC2_32_ABS_LD_W:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp, ctx->mem_idx, MO_LESL);
        break;
    }

    tcg_temp_free(temp);
}

static void decode_abs_ldb(CPUTriCoreState *env, DisasContext *ctx)
{
    int32_t op2;
    int32_t r1;
    uint32_t address;
    TCGv temp;

    r1 = MASK_OP_ABS_S1D(ctx->opcode);
    address = MASK_OP_ABS_OFF18(ctx->opcode);
    op2 = MASK_OP_ABS_OP2(ctx->opcode);

    temp = tcg_const_i32(EA_ABS_FORMAT(address));

    switch (op2) {
    case OPC2_32_ABS_LD_B:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp, ctx->mem_idx, MO_SB);
        break;
    case OPC2_32_ABS_LD_BU:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp, ctx->mem_idx, MO_UB);
        break;
    case OPC2_32_ABS_LD_H:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp, ctx->mem_idx, MO_LESW);
        break;
    case OPC2_32_ABS_LD_HU:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp, ctx->mem_idx, MO_LEUW);
        break;
    }

    tcg_temp_free(temp);
}

static void decode_abs_ldst_swap(CPUTriCoreState *env, DisasContext *ctx)
{
    int32_t op2;
    int32_t r1;
    uint32_t address;
    TCGv temp;

    r1 = MASK_OP_ABS_S1D(ctx->opcode);
    address = MASK_OP_ABS_OFF18(ctx->opcode);
    op2 = MASK_OP_ABS_OP2(ctx->opcode);

    temp = tcg_const_i32(EA_ABS_FORMAT(address));

    switch (op2) {
    case OPC2_32_ABS_LDMST:
        gen_ldmst(ctx, r1, temp);
        break;
    case OPC2_32_ABS_SWAP_W:
        gen_swap(ctx, r1, temp);
        break;
    }

    tcg_temp_free(temp);
}

static void decode_abs_ldst_context(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int32_t off18;

    off18 = MASK_OP_ABS_OFF18(ctx->opcode);
    op2   = MASK_OP_ABS_OP2(ctx->opcode);

    switch (op2) {
    case OPC2_32_ABS_LDLCX:
        gen_helper_1arg(ldlcx, EA_ABS_FORMAT(off18));
        break;
    case OPC2_32_ABS_LDUCX:
        gen_helper_1arg(lducx, EA_ABS_FORMAT(off18));
        break;
    case OPC2_32_ABS_STLCX:
        gen_helper_1arg(stlcx, EA_ABS_FORMAT(off18));
        break;
    case OPC2_32_ABS_STUCX:
        gen_helper_1arg(stucx, EA_ABS_FORMAT(off18));
        break;
    }
}

static void decode_abs_store(CPUTriCoreState *env, DisasContext *ctx)
{
    int32_t op2;
    int32_t r1;
    uint32_t address;
    TCGv temp;

    r1 = MASK_OP_ABS_S1D(ctx->opcode);
    address = MASK_OP_ABS_OFF18(ctx->opcode);
    op2 = MASK_OP_ABS_OP2(ctx->opcode);

    temp = tcg_const_i32(EA_ABS_FORMAT(address));

    switch (op2) {
    case OPC2_32_ABS_ST_A:
        tcg_gen_qemu_st_tl(cpu_gpr_a[r1], temp, ctx->mem_idx, MO_LESL);
        break;
    case OPC2_32_ABS_ST_D:
        gen_st_2regs_64(cpu_gpr_d[r1+1], cpu_gpr_d[r1], temp, ctx);
        break;
    case OPC2_32_ABS_ST_DA:
        gen_st_2regs_64(cpu_gpr_a[r1+1], cpu_gpr_a[r1], temp, ctx);
        break;
    case OPC2_32_ABS_ST_W:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], temp, ctx->mem_idx, MO_LESL);
        break;

    }
    tcg_temp_free(temp);
}

static void decode_abs_storeb_h(CPUTriCoreState *env, DisasContext *ctx)
{
    int32_t op2;
    int32_t r1;
    uint32_t address;
    TCGv temp;

    r1 = MASK_OP_ABS_S1D(ctx->opcode);
    address = MASK_OP_ABS_OFF18(ctx->opcode);
    op2 = MASK_OP_ABS_OP2(ctx->opcode);

    temp = tcg_const_i32(EA_ABS_FORMAT(address));

    switch (op2) {
    case OPC2_32_ABS_ST_B:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], temp, ctx->mem_idx, MO_UB);
        break;
    case OPC2_32_ABS_ST_H:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], temp, ctx->mem_idx, MO_LEUW);
        break;
    }
    tcg_temp_free(temp);
}

/* Bit-format */

static void decode_bit_andacc(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2, r3;
    int pos1, pos2;

    r1 = MASK_OP_BIT_S1(ctx->opcode);
    r2 = MASK_OP_BIT_S2(ctx->opcode);
    r3 = MASK_OP_BIT_D(ctx->opcode);
    pos1 = MASK_OP_BIT_POS1(ctx->opcode);
    pos2 = MASK_OP_BIT_POS2(ctx->opcode);
    op2 = MASK_OP_BIT_OP2(ctx->opcode);


    switch (op2) {
    case OPC2_32_BIT_AND_AND_T:
        gen_bit_2op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_and_tl, &tcg_gen_and_tl);
        break;
    case OPC2_32_BIT_AND_ANDN_T:
        gen_bit_2op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_andc_tl, &tcg_gen_and_tl);
        break;
    case OPC2_32_BIT_AND_NOR_T:
        if (TCG_TARGET_HAS_andc_i32) {
            gen_bit_2op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                        pos1, pos2, &tcg_gen_or_tl, &tcg_gen_andc_tl);
        } else {
            gen_bit_2op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                        pos1, pos2, &tcg_gen_nor_tl, &tcg_gen_and_tl);
        }
        break;
    case OPC2_32_BIT_AND_OR_T:
        gen_bit_2op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_or_tl, &tcg_gen_and_tl);
        break;
    }
}

static void decode_bit_logical_t(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2, r3;
    int pos1, pos2;
    r1 = MASK_OP_BIT_S1(ctx->opcode);
    r2 = MASK_OP_BIT_S2(ctx->opcode);
    r3 = MASK_OP_BIT_D(ctx->opcode);
    pos1 = MASK_OP_BIT_POS1(ctx->opcode);
    pos2 = MASK_OP_BIT_POS2(ctx->opcode);
    op2 = MASK_OP_BIT_OP2(ctx->opcode);

    switch (op2) {
    case OPC2_32_BIT_AND_T:
        gen_bit_1op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_and_tl);
        break;
    case OPC2_32_BIT_ANDN_T:
        gen_bit_1op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_andc_tl);
        break;
    case OPC2_32_BIT_NOR_T:
        gen_bit_1op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_nor_tl);
        break;
    case OPC2_32_BIT_OR_T:
        gen_bit_1op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_or_tl);
        break;
    }
}

static void decode_bit_insert(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2, r3;
    int pos1, pos2;
    TCGv temp;
    op2 = MASK_OP_BIT_OP2(ctx->opcode);
    r1 = MASK_OP_BIT_S1(ctx->opcode);
    r2 = MASK_OP_BIT_S2(ctx->opcode);
    r3 = MASK_OP_BIT_D(ctx->opcode);
    pos1 = MASK_OP_BIT_POS1(ctx->opcode);
    pos2 = MASK_OP_BIT_POS2(ctx->opcode);

    temp = tcg_temp_new();

    tcg_gen_shri_tl(temp, cpu_gpr_d[r2], pos2);
    if (op2 == OPC2_32_BIT_INSN_T) {
        tcg_gen_not_tl(temp, temp);
    }
    tcg_gen_deposit_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], temp, pos1, 1);
    tcg_temp_free(temp);
}

static void decode_bit_logical_t2(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;

    int r1, r2, r3;
    int pos1, pos2;

    op2 = MASK_OP_BIT_OP2(ctx->opcode);
    r1 = MASK_OP_BIT_S1(ctx->opcode);
    r2 = MASK_OP_BIT_S2(ctx->opcode);
    r3 = MASK_OP_BIT_D(ctx->opcode);
    pos1 = MASK_OP_BIT_POS1(ctx->opcode);
    pos2 = MASK_OP_BIT_POS2(ctx->opcode);

    switch (op2) {
    case OPC2_32_BIT_NAND_T:
        gen_bit_1op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_nand_tl);
        break;
    case OPC2_32_BIT_ORN_T:
        gen_bit_1op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_orc_tl);
        break;
    case OPC2_32_BIT_XNOR_T:
        gen_bit_1op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_eqv_tl);
        break;
    case OPC2_32_BIT_XOR_T:
        gen_bit_1op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_xor_tl);
        break;
    }
}

static void decode_bit_orand(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;

    int r1, r2, r3;
    int pos1, pos2;

    op2 = MASK_OP_BIT_OP2(ctx->opcode);
    r1 = MASK_OP_BIT_S1(ctx->opcode);
    r2 = MASK_OP_BIT_S2(ctx->opcode);
    r3 = MASK_OP_BIT_D(ctx->opcode);
    pos1 = MASK_OP_BIT_POS1(ctx->opcode);
    pos2 = MASK_OP_BIT_POS2(ctx->opcode);

    switch (op2) {
    case OPC2_32_BIT_OR_AND_T:
        gen_bit_2op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_and_tl, &tcg_gen_or_tl);
        break;
    case OPC2_32_BIT_OR_ANDN_T:
        gen_bit_2op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_andc_tl, &tcg_gen_or_tl);
        break;
    case OPC2_32_BIT_OR_NOR_T:
        if (TCG_TARGET_HAS_orc_i32) {
            gen_bit_2op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                        pos1, pos2, &tcg_gen_or_tl, &tcg_gen_orc_tl);
        } else {
            gen_bit_2op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                        pos1, pos2, &tcg_gen_nor_tl, &tcg_gen_or_tl);
        }
        break;
    case OPC2_32_BIT_OR_OR_T:
        gen_bit_2op(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_or_tl, &tcg_gen_or_tl);
        break;
    }
}

static void decode_bit_sh_logic1(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2, r3;
    int pos1, pos2;
    TCGv temp;

    op2 = MASK_OP_BIT_OP2(ctx->opcode);
    r1 = MASK_OP_BIT_S1(ctx->opcode);
    r2 = MASK_OP_BIT_S2(ctx->opcode);
    r3 = MASK_OP_BIT_D(ctx->opcode);
    pos1 = MASK_OP_BIT_POS1(ctx->opcode);
    pos2 = MASK_OP_BIT_POS2(ctx->opcode);

    temp = tcg_temp_new();

    switch (op2) {
    case OPC2_32_BIT_SH_AND_T:
        gen_bit_1op(temp, cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_and_tl);
        break;
    case OPC2_32_BIT_SH_ANDN_T:
        gen_bit_1op(temp, cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_andc_tl);
        break;
    case OPC2_32_BIT_SH_NOR_T:
        gen_bit_1op(temp, cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_nor_tl);
        break;
    case OPC2_32_BIT_SH_OR_T:
        gen_bit_1op(temp, cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_or_tl);
        break;
    }
    tcg_gen_shli_tl(cpu_gpr_d[r3], cpu_gpr_d[r3], 1);
    tcg_gen_add_tl(cpu_gpr_d[r3], cpu_gpr_d[r3], temp);
    tcg_temp_free(temp);
}

static void decode_bit_sh_logic2(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2, r3;
    int pos1, pos2;
    TCGv temp;

    op2 = MASK_OP_BIT_OP2(ctx->opcode);
    r1 = MASK_OP_BIT_S1(ctx->opcode);
    r2 = MASK_OP_BIT_S2(ctx->opcode);
    r3 = MASK_OP_BIT_D(ctx->opcode);
    pos1 = MASK_OP_BIT_POS1(ctx->opcode);
    pos2 = MASK_OP_BIT_POS2(ctx->opcode);

    temp = tcg_temp_new();

    switch (op2) {
    case OPC2_32_BIT_SH_NAND_T:
        gen_bit_1op(temp, cpu_gpr_d[r1] , cpu_gpr_d[r2] ,
                    pos1, pos2, &tcg_gen_nand_tl);
        break;
    case OPC2_32_BIT_SH_ORN_T:
        gen_bit_1op(temp, cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_orc_tl);
        break;
    case OPC2_32_BIT_SH_XNOR_T:
        gen_bit_1op(temp, cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_eqv_tl);
        break;
    case OPC2_32_BIT_SH_XOR_T:
        gen_bit_1op(temp, cpu_gpr_d[r1], cpu_gpr_d[r2],
                    pos1, pos2, &tcg_gen_xor_tl);
        break;
    }
    tcg_gen_shli_tl(cpu_gpr_d[r3], cpu_gpr_d[r3], 1);
    tcg_gen_add_tl(cpu_gpr_d[r3], cpu_gpr_d[r3], temp);
    tcg_temp_free(temp);
}

/* BO-format */


static void decode_bo_addrmode_post_pre_base(CPUTriCoreState *env,
                                             DisasContext *ctx)
{
    uint32_t op2;
    uint32_t off10;
    int32_t r1, r2;
    TCGv temp;

    r1 = MASK_OP_BO_S1D(ctx->opcode);
    r2  = MASK_OP_BO_S2(ctx->opcode);
    off10 = MASK_OP_BO_OFF10_SEXT(ctx->opcode);
    op2 = MASK_OP_BO_OP2(ctx->opcode);

    switch (op2) {
    case OPC2_32_BO_CACHEA_WI_SHORTOFF:
    case OPC2_32_BO_CACHEA_W_SHORTOFF:
    case OPC2_32_BO_CACHEA_I_SHORTOFF:
        /* instruction to access the cache */
        break;
    case OPC2_32_BO_CACHEA_WI_POSTINC:
    case OPC2_32_BO_CACHEA_W_POSTINC:
    case OPC2_32_BO_CACHEA_I_POSTINC:
        /* instruction to access the cache, but we still need to handle
           the addressing mode */
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_CACHEA_WI_PREINC:
    case OPC2_32_BO_CACHEA_W_PREINC:
    case OPC2_32_BO_CACHEA_I_PREINC:
        /* instruction to access the cache, but we still need to handle
           the addressing mode */
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_CACHEI_WI_SHORTOFF:
    case OPC2_32_BO_CACHEI_W_SHORTOFF:
        /* TODO: Raise illegal opcode trap,
                 if !tricore_feature(TRICORE_FEATURE_131) */
        break;
    case OPC2_32_BO_CACHEI_W_POSTINC:
    case OPC2_32_BO_CACHEI_WI_POSTINC:
        if (tricore_feature(env, TRICORE_FEATURE_131)) {
            tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        } /* TODO: else raise illegal opcode trap */
        break;
    case OPC2_32_BO_CACHEI_W_PREINC:
    case OPC2_32_BO_CACHEI_WI_PREINC:
        if (tricore_feature(env, TRICORE_FEATURE_131)) {
            tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        } /* TODO: else raise illegal opcode trap */
        break;
    case OPC2_32_BO_ST_A_SHORTOFF:
        gen_offset_st(ctx, cpu_gpr_a[r1], cpu_gpr_a[r2], off10, MO_LESL);
        break;
    case OPC2_32_BO_ST_A_POSTINC:
        tcg_gen_qemu_st_tl(cpu_gpr_a[r1], cpu_gpr_a[r2], ctx->mem_idx,
                           MO_LESL);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_ST_A_PREINC:
        gen_st_preincr(ctx, cpu_gpr_a[r1], cpu_gpr_a[r2], off10, MO_LESL);
        break;
    case OPC2_32_BO_ST_B_SHORTOFF:
        gen_offset_st(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_UB);
        break;
    case OPC2_32_BO_ST_B_POSTINC:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx,
                           MO_UB);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_ST_B_PREINC:
        gen_st_preincr(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_UB);
        break;
    case OPC2_32_BO_ST_D_SHORTOFF:
        gen_offset_st_2regs(cpu_gpr_d[r1+1], cpu_gpr_d[r1], cpu_gpr_a[r2],
                            off10, ctx);
        break;
    case OPC2_32_BO_ST_D_POSTINC:
        gen_st_2regs_64(cpu_gpr_d[r1+1], cpu_gpr_d[r1], cpu_gpr_a[r2], ctx);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_ST_D_PREINC:
        temp = tcg_temp_new();
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], off10);
        gen_st_2regs_64(cpu_gpr_d[r1+1], cpu_gpr_d[r1], temp, ctx);
        tcg_gen_mov_tl(cpu_gpr_a[r2], temp);
        tcg_temp_free(temp);
        break;
    case OPC2_32_BO_ST_DA_SHORTOFF:
        gen_offset_st_2regs(cpu_gpr_a[r1+1], cpu_gpr_a[r1], cpu_gpr_a[r2],
                            off10, ctx);
        break;
    case OPC2_32_BO_ST_DA_POSTINC:
        gen_st_2regs_64(cpu_gpr_a[r1+1], cpu_gpr_a[r1], cpu_gpr_a[r2], ctx);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_ST_DA_PREINC:
        temp = tcg_temp_new();
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], off10);
        gen_st_2regs_64(cpu_gpr_a[r1+1], cpu_gpr_a[r1], temp, ctx);
        tcg_gen_mov_tl(cpu_gpr_a[r2], temp);
        tcg_temp_free(temp);
        break;
    case OPC2_32_BO_ST_H_SHORTOFF:
        gen_offset_st(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_LEUW);
        break;
    case OPC2_32_BO_ST_H_POSTINC:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx,
                           MO_LEUW);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_ST_H_PREINC:
        gen_st_preincr(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_LEUW);
        break;
    case OPC2_32_BO_ST_Q_SHORTOFF:
        temp = tcg_temp_new();
        tcg_gen_shri_tl(temp, cpu_gpr_d[r1], 16);
        gen_offset_st(ctx, temp, cpu_gpr_a[r2], off10, MO_LEUW);
        tcg_temp_free(temp);
        break;
    case OPC2_32_BO_ST_Q_POSTINC:
        temp = tcg_temp_new();
        tcg_gen_shri_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_qemu_st_tl(temp, cpu_gpr_a[r2], ctx->mem_idx,
                           MO_LEUW);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        tcg_temp_free(temp);
        break;
    case OPC2_32_BO_ST_Q_PREINC:
        temp = tcg_temp_new();
        tcg_gen_shri_tl(temp, cpu_gpr_d[r1], 16);
        gen_st_preincr(ctx, temp, cpu_gpr_a[r2], off10, MO_LEUW);
        tcg_temp_free(temp);
        break;
    case OPC2_32_BO_ST_W_SHORTOFF:
        gen_offset_st(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_LEUL);
        break;
    case OPC2_32_BO_ST_W_POSTINC:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx,
                           MO_LEUL);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_ST_W_PREINC:
        gen_st_preincr(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_LEUL);
        break;
    }
}

static void decode_bo_addrmode_bitreverse_circular(CPUTriCoreState *env,
                                                   DisasContext *ctx)
{
    uint32_t op2;
    uint32_t off10;
    int32_t r1, r2;
    TCGv temp, temp2, temp3;

    r1 = MASK_OP_BO_S1D(ctx->opcode);
    r2  = MASK_OP_BO_S2(ctx->opcode);
    off10 = MASK_OP_BO_OFF10_SEXT(ctx->opcode);
    op2 = MASK_OP_BO_OP2(ctx->opcode);

    temp = tcg_temp_new();
    temp2 = tcg_temp_new();
    temp3 = tcg_const_i32(off10);

    tcg_gen_ext16u_tl(temp, cpu_gpr_a[r2+1]);
    tcg_gen_add_tl(temp2, cpu_gpr_a[r2], temp);

    switch (op2) {
    case OPC2_32_BO_CACHEA_WI_BR:
    case OPC2_32_BO_CACHEA_W_BR:
    case OPC2_32_BO_CACHEA_I_BR:
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_CACHEA_WI_CIRC:
    case OPC2_32_BO_CACHEA_W_CIRC:
    case OPC2_32_BO_CACHEA_I_CIRC:
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_ST_A_BR:
        tcg_gen_qemu_st_tl(cpu_gpr_a[r1], temp2, ctx->mem_idx, MO_LEUL);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_ST_A_CIRC:
        tcg_gen_qemu_st_tl(cpu_gpr_a[r1], temp2, ctx->mem_idx, MO_LEUL);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_ST_B_BR:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_UB);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_ST_B_CIRC:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_UB);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_ST_D_BR:
        gen_st_2regs_64(cpu_gpr_d[r1+1], cpu_gpr_d[r1], temp2, ctx);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_ST_D_CIRC:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LEUL);
        tcg_gen_shri_tl(temp2, cpu_gpr_a[r2+1], 16);
        tcg_gen_addi_tl(temp, temp, 4);
        tcg_gen_rem_tl(temp, temp, temp2);
        tcg_gen_add_tl(temp2, cpu_gpr_a[r2], temp);
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1+1], temp2, ctx->mem_idx, MO_LEUL);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_ST_DA_BR:
        gen_st_2regs_64(cpu_gpr_a[r1+1], cpu_gpr_a[r1], temp2, ctx);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_ST_DA_CIRC:
        tcg_gen_qemu_st_tl(cpu_gpr_a[r1], temp2, ctx->mem_idx, MO_LEUL);
        tcg_gen_shri_tl(temp2, cpu_gpr_a[r2+1], 16);
        tcg_gen_addi_tl(temp, temp, 4);
        tcg_gen_rem_tl(temp, temp, temp2);
        tcg_gen_add_tl(temp2, cpu_gpr_a[r2], temp);
        tcg_gen_qemu_st_tl(cpu_gpr_a[r1+1], temp2, ctx->mem_idx, MO_LEUL);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_ST_H_BR:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LEUW);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_ST_H_CIRC:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LEUW);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_ST_Q_BR:
        tcg_gen_shri_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_qemu_st_tl(temp, temp2, ctx->mem_idx, MO_LEUW);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_ST_Q_CIRC:
        tcg_gen_shri_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_qemu_st_tl(temp, temp2, ctx->mem_idx, MO_LEUW);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_ST_W_BR:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LEUL);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_ST_W_CIRC:
        tcg_gen_qemu_st_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LEUL);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    }
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(temp3);
}

static void decode_bo_addrmode_ld_post_pre_base(CPUTriCoreState *env,
                                                DisasContext *ctx)
{
    uint32_t op2;
    uint32_t off10;
    int32_t r1, r2;
    TCGv temp;

    r1 = MASK_OP_BO_S1D(ctx->opcode);
    r2  = MASK_OP_BO_S2(ctx->opcode);
    off10 = MASK_OP_BO_OFF10_SEXT(ctx->opcode);
    op2 = MASK_OP_BO_OP2(ctx->opcode);

    switch (op2) {
    case OPC2_32_BO_LD_A_SHORTOFF:
        gen_offset_ld(ctx, cpu_gpr_a[r1], cpu_gpr_a[r2], off10, MO_LEUL);
        break;
    case OPC2_32_BO_LD_A_POSTINC:
        tcg_gen_qemu_ld_tl(cpu_gpr_a[r1], cpu_gpr_a[r2], ctx->mem_idx,
                           MO_LEUL);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_LD_A_PREINC:
        gen_ld_preincr(ctx, cpu_gpr_a[r1], cpu_gpr_a[r2], off10, MO_LEUL);
        break;
    case OPC2_32_BO_LD_B_SHORTOFF:
        gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_SB);
        break;
    case OPC2_32_BO_LD_B_POSTINC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx,
                           MO_SB);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_LD_B_PREINC:
        gen_ld_preincr(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_SB);
        break;
    case OPC2_32_BO_LD_BU_SHORTOFF:
        gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_UB);
        break;
    case OPC2_32_BO_LD_BU_POSTINC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx,
                           MO_UB);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_LD_BU_PREINC:
        gen_ld_preincr(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_SB);
        break;
    case OPC2_32_BO_LD_D_SHORTOFF:
        gen_offset_ld_2regs(cpu_gpr_d[r1+1], cpu_gpr_d[r1], cpu_gpr_a[r2],
                            off10, ctx);
        break;
    case OPC2_32_BO_LD_D_POSTINC:
        gen_ld_2regs_64(cpu_gpr_d[r1+1], cpu_gpr_d[r1], cpu_gpr_a[r2], ctx);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_LD_D_PREINC:
        temp = tcg_temp_new();
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], off10);
        gen_ld_2regs_64(cpu_gpr_d[r1+1], cpu_gpr_d[r1], temp, ctx);
        tcg_gen_mov_tl(cpu_gpr_a[r2], temp);
        tcg_temp_free(temp);
        break;
    case OPC2_32_BO_LD_DA_SHORTOFF:
        gen_offset_ld_2regs(cpu_gpr_a[r1+1], cpu_gpr_a[r1], cpu_gpr_a[r2],
                            off10, ctx);
        break;
    case OPC2_32_BO_LD_DA_POSTINC:
        gen_ld_2regs_64(cpu_gpr_a[r1+1], cpu_gpr_a[r1], cpu_gpr_a[r2], ctx);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_LD_DA_PREINC:
        temp = tcg_temp_new();
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], off10);
        gen_ld_2regs_64(cpu_gpr_a[r1+1], cpu_gpr_a[r1], temp, ctx);
        tcg_gen_mov_tl(cpu_gpr_a[r2], temp);
        tcg_temp_free(temp);
        break;
    case OPC2_32_BO_LD_H_SHORTOFF:
        gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_LESW);
        break;
    case OPC2_32_BO_LD_H_POSTINC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx,
                           MO_LESW);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_LD_H_PREINC:
        gen_ld_preincr(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_LESW);
        break;
    case OPC2_32_BO_LD_HU_SHORTOFF:
        gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_LEUW);
        break;
    case OPC2_32_BO_LD_HU_POSTINC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx,
                           MO_LEUW);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_LD_HU_PREINC:
        gen_ld_preincr(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_LEUW);
        break;
    case OPC2_32_BO_LD_Q_SHORTOFF:
        gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_LEUW);
        tcg_gen_shli_tl(cpu_gpr_d[r1], cpu_gpr_d[r1], 16);
        break;
    case OPC2_32_BO_LD_Q_POSTINC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx,
                           MO_LEUW);
        tcg_gen_shli_tl(cpu_gpr_d[r1], cpu_gpr_d[r1], 16);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_LD_Q_PREINC:
        gen_ld_preincr(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_LEUW);
        tcg_gen_shli_tl(cpu_gpr_d[r1], cpu_gpr_d[r1], 16);
        break;
    case OPC2_32_BO_LD_W_SHORTOFF:
        gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_LEUL);
        break;
    case OPC2_32_BO_LD_W_POSTINC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], cpu_gpr_a[r2], ctx->mem_idx,
                           MO_LEUL);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_LD_W_PREINC:
        gen_ld_preincr(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], off10, MO_LEUL);
        break;
    }
}

static void decode_bo_addrmode_ld_bitreverse_circular(CPUTriCoreState *env,
                                                DisasContext *ctx)
{
    uint32_t op2;
    uint32_t off10;
    int r1, r2;

    TCGv temp, temp2, temp3;

    r1 = MASK_OP_BO_S1D(ctx->opcode);
    r2 = MASK_OP_BO_S2(ctx->opcode);
    off10 = MASK_OP_BO_OFF10_SEXT(ctx->opcode);
    op2 = MASK_OP_BO_OP2(ctx->opcode);

    temp = tcg_temp_new();
    temp2 = tcg_temp_new();
    temp3 = tcg_const_i32(off10);

    tcg_gen_ext16u_tl(temp, cpu_gpr_a[r2+1]);
    tcg_gen_add_tl(temp2, cpu_gpr_a[r2], temp);


    switch (op2) {
    case OPC2_32_BO_LD_A_BR:
        tcg_gen_qemu_ld_tl(cpu_gpr_a[r1], temp2, ctx->mem_idx, MO_LEUL);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_LD_A_CIRC:
        tcg_gen_qemu_ld_tl(cpu_gpr_a[r1], temp2, ctx->mem_idx, MO_LEUL);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_LD_B_BR:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_SB);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_LD_B_CIRC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_SB);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_LD_BU_BR:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_UB);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_LD_BU_CIRC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_UB);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_LD_D_BR:
        gen_ld_2regs_64(cpu_gpr_d[r1+1], cpu_gpr_d[r1], temp2, ctx);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_LD_D_CIRC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LEUL);
        tcg_gen_shri_tl(temp2, cpu_gpr_a[r2+1], 16);
        tcg_gen_addi_tl(temp, temp, 4);
        tcg_gen_rem_tl(temp, temp, temp2);
        tcg_gen_add_tl(temp2, cpu_gpr_a[r2], temp);
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1+1], temp2, ctx->mem_idx, MO_LEUL);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_LD_DA_BR:
        gen_ld_2regs_64(cpu_gpr_a[r1+1], cpu_gpr_a[r1], temp2, ctx);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_LD_DA_CIRC:
        tcg_gen_qemu_ld_tl(cpu_gpr_a[r1], temp2, ctx->mem_idx, MO_LEUL);
        tcg_gen_shri_tl(temp2, cpu_gpr_a[r2+1], 16);
        tcg_gen_addi_tl(temp, temp, 4);
        tcg_gen_rem_tl(temp, temp, temp2);
        tcg_gen_add_tl(temp2, cpu_gpr_a[r2], temp);
        tcg_gen_qemu_ld_tl(cpu_gpr_a[r1+1], temp2, ctx->mem_idx, MO_LEUL);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_LD_H_BR:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LESW);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_LD_H_CIRC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LESW);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_LD_HU_BR:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LEUW);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_LD_HU_CIRC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LEUW);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_LD_Q_BR:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LEUW);
        tcg_gen_shli_tl(cpu_gpr_d[r1], cpu_gpr_d[r1], 16);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_LD_Q_CIRC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LEUW);
        tcg_gen_shli_tl(cpu_gpr_d[r1], cpu_gpr_d[r1], 16);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_LD_W_BR:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LEUL);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_LD_W_CIRC:
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp2, ctx->mem_idx, MO_LEUL);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    }
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(temp3);
}

static void decode_bo_addrmode_stctx_post_pre_base(CPUTriCoreState *env,
                                                   DisasContext *ctx)
{
    uint32_t op2;
    uint32_t off10;
    int r1, r2;

    TCGv temp, temp2;

    r1 = MASK_OP_BO_S1D(ctx->opcode);
    r2 = MASK_OP_BO_S2(ctx->opcode);
    off10 = MASK_OP_BO_OFF10_SEXT(ctx->opcode);
    op2 = MASK_OP_BO_OP2(ctx->opcode);


    temp = tcg_temp_new();
    temp2 = tcg_temp_new();

    switch (op2) {
    case OPC2_32_BO_LDLCX_SHORTOFF:
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], off10);
        gen_helper_ldlcx(cpu_env, temp);
        break;
    case OPC2_32_BO_LDMST_SHORTOFF:
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], off10);
        gen_ldmst(ctx, r1, temp);
        break;
    case OPC2_32_BO_LDMST_POSTINC:
        gen_ldmst(ctx, r1, cpu_gpr_a[r2]);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_LDMST_PREINC:
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        gen_ldmst(ctx, r1, cpu_gpr_a[r2]);
        break;
    case OPC2_32_BO_LDUCX_SHORTOFF:
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], off10);
        gen_helper_lducx(cpu_env, temp);
        break;
    case OPC2_32_BO_LEA_SHORTOFF:
        tcg_gen_addi_tl(cpu_gpr_a[r1], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_STLCX_SHORTOFF:
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], off10);
        gen_helper_stlcx(cpu_env, temp);
        break;
    case OPC2_32_BO_STUCX_SHORTOFF:
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], off10);
        gen_helper_stucx(cpu_env, temp);
        break;
    case OPC2_32_BO_SWAP_W_SHORTOFF:
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], off10);
        gen_swap(ctx, r1, temp);
        break;
    case OPC2_32_BO_SWAP_W_POSTINC:
        gen_swap(ctx, r1, cpu_gpr_a[r2]);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_SWAP_W_PREINC:
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        gen_swap(ctx, r1, cpu_gpr_a[r2]);
        break;
    case OPC2_32_BO_CMPSWAP_W_SHORTOFF:
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], off10);
        gen_cmpswap(ctx, r1, temp);
        break;
    case OPC2_32_BO_CMPSWAP_W_POSTINC:
        gen_cmpswap(ctx, r1, cpu_gpr_a[r2]);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_CMPSWAP_W_PREINC:
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        gen_cmpswap(ctx, r1, cpu_gpr_a[r2]);
        break;
    case OPC2_32_BO_SWAPMSK_W_SHORTOFF:
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], off10);
        gen_swapmsk(ctx, r1, temp);
        break;
    case OPC2_32_BO_SWAPMSK_W_POSTINC:
        gen_swapmsk(ctx, r1, cpu_gpr_a[r2]);
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        break;
    case OPC2_32_BO_SWAPMSK_W_PREINC:
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r2], off10);
        gen_swapmsk(ctx, r1, cpu_gpr_a[r2]);
        break;
    }
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static void decode_bo_addrmode_ldmst_bitreverse_circular(CPUTriCoreState *env,
                                                         DisasContext *ctx)
{
    uint32_t op2;
    uint32_t off10;
    int r1, r2;

    TCGv temp, temp2, temp3;

    r1 = MASK_OP_BO_S1D(ctx->opcode);
    r2 = MASK_OP_BO_S2(ctx->opcode);
    off10 = MASK_OP_BO_OFF10_SEXT(ctx->opcode);
    op2 = MASK_OP_BO_OP2(ctx->opcode);

    temp = tcg_temp_new();
    temp2 = tcg_temp_new();
    temp3 = tcg_const_i32(off10);

    tcg_gen_ext16u_tl(temp, cpu_gpr_a[r2+1]);
    tcg_gen_add_tl(temp2, cpu_gpr_a[r2], temp);

    switch (op2) {
    case OPC2_32_BO_LDMST_BR:
        gen_ldmst(ctx, r1, temp2);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_LDMST_CIRC:
        gen_ldmst(ctx, r1, temp2);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_SWAP_W_BR:
        gen_swap(ctx, r1, temp2);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_SWAP_W_CIRC:
        gen_swap(ctx, r1, temp2);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_CMPSWAP_W_BR:
        gen_cmpswap(ctx, r1, temp2);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_CMPSWAP_W_CIRC:
        gen_cmpswap(ctx, r1, temp2);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    case OPC2_32_BO_SWAPMSK_W_BR:
        gen_swapmsk(ctx, r1, temp2);
        gen_helper_br_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1]);
        break;
    case OPC2_32_BO_SWAPMSK_W_CIRC:
        gen_swapmsk(ctx, r1, temp2);
        gen_helper_circ_update(cpu_gpr_a[r2+1], cpu_gpr_a[r2+1], temp3);
        break;
    }

    tcg_temp_free(temp);
    tcg_temp_free(temp2);
    tcg_temp_free(temp3);
}

static void decode_bol_opc(CPUTriCoreState *env, DisasContext *ctx, int32_t op1)
{
    int r1, r2;
    int32_t address;
    TCGv temp;

    r1 = MASK_OP_BOL_S1D(ctx->opcode);
    r2 = MASK_OP_BOL_S2(ctx->opcode);
    address = MASK_OP_BOL_OFF16_SEXT(ctx->opcode);

    switch (op1) {
    case OPC1_32_BOL_LD_A_LONGOFF:
        temp = tcg_temp_new();
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], address);
        tcg_gen_qemu_ld_tl(cpu_gpr_a[r1], temp, ctx->mem_idx, MO_LEUL);
        tcg_temp_free(temp);
        break;
    case OPC1_32_BOL_LD_W_LONGOFF:
        temp = tcg_temp_new();
        tcg_gen_addi_tl(temp, cpu_gpr_a[r2], address);
        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp, ctx->mem_idx, MO_LEUL);
        tcg_temp_free(temp);
        break;
    case OPC1_32_BOL_LEA_LONGOFF:
        tcg_gen_addi_tl(cpu_gpr_a[r1], cpu_gpr_a[r2], address);
        break;
    case OPC1_32_BOL_ST_A_LONGOFF:
        if (tricore_feature(env, TRICORE_FEATURE_16)) {
            gen_offset_st(ctx, cpu_gpr_a[r1], cpu_gpr_a[r2], address, MO_LEUL);
        } else {
            /* raise illegal opcode trap */
        }
        break;
    case OPC1_32_BOL_ST_W_LONGOFF:
        gen_offset_st(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], address, MO_LEUL);
        break;
    case OPC1_32_BOL_LD_B_LONGOFF:
        if (tricore_feature(env, TRICORE_FEATURE_16)) {
            gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], address, MO_SB);
        } else {
            /* raise illegal opcode trap */
        }
        break;
    case OPC1_32_BOL_LD_BU_LONGOFF:
        if (tricore_feature(env, TRICORE_FEATURE_16)) {
            gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], address, MO_UB);
        } else {
            /* raise illegal opcode trap */
        }
        break;
    case OPC1_32_BOL_LD_H_LONGOFF:
        if (tricore_feature(env, TRICORE_FEATURE_16)) {
            gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], address, MO_LESW);
        } else {
            /* raise illegal opcode trap */
        }
        break;
    case OPC1_32_BOL_LD_HU_LONGOFF:
        if (tricore_feature(env, TRICORE_FEATURE_16)) {
            gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], address, MO_LEUW);
        } else {
            /* raise illegal opcode trap */
        }
        break;
    case OPC1_32_BOL_ST_B_LONGOFF:
        if (tricore_feature(env, TRICORE_FEATURE_16)) {
            gen_offset_st(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], address, MO_SB);
        } else {
            /* raise illegal opcode trap */
        }
        break;
    case OPC1_32_BOL_ST_H_LONGOFF:
        if (tricore_feature(env, TRICORE_FEATURE_16)) {
            gen_offset_ld(ctx, cpu_gpr_d[r1], cpu_gpr_a[r2], address, MO_LESW);
        } else {
            /* raise illegal opcode trap */
        }
        break;
    }
}

/* RC format */
static void decode_rc_logical_shift(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2;
    int32_t const9;
    TCGv temp;

    r2 = MASK_OP_RC_D(ctx->opcode);
    r1 = MASK_OP_RC_S1(ctx->opcode);
    const9 = MASK_OP_RC_CONST9(ctx->opcode);
    op2 = MASK_OP_RC_OP2(ctx->opcode);

    temp = tcg_temp_new();

    switch (op2) {
    case OPC2_32_RC_AND:
        tcg_gen_andi_tl(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_ANDN:
        tcg_gen_andi_tl(cpu_gpr_d[r2], cpu_gpr_d[r1], ~const9);
        break;
    case OPC2_32_RC_NAND:
        tcg_gen_movi_tl(temp, const9);
        tcg_gen_nand_tl(cpu_gpr_d[r2], cpu_gpr_d[r1], temp);
        break;
    case OPC2_32_RC_NOR:
        tcg_gen_movi_tl(temp, const9);
        tcg_gen_nor_tl(cpu_gpr_d[r2], cpu_gpr_d[r1], temp);
        break;
    case OPC2_32_RC_OR:
        tcg_gen_ori_tl(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_ORN:
        tcg_gen_ori_tl(cpu_gpr_d[r2], cpu_gpr_d[r1], ~const9);
        break;
    case OPC2_32_RC_SH:
        const9 = sextract32(const9, 0, 6);
        gen_shi(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_SH_H:
        const9 = sextract32(const9, 0, 5);
        gen_sh_hi(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_SHA:
        const9 = sextract32(const9, 0, 6);
        gen_shaci(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_SHA_H:
        const9 = sextract32(const9, 0, 5);
        gen_sha_hi(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_SHAS:
        gen_shasi(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_XNOR:
        tcg_gen_xori_tl(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        tcg_gen_not_tl(cpu_gpr_d[r2], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RC_XOR:
        tcg_gen_xori_tl(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    }
    tcg_temp_free(temp);
}

static void decode_rc_accumulator(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2;
    int16_t const9;

    TCGv temp;

    r2 = MASK_OP_RC_D(ctx->opcode);
    r1 = MASK_OP_RC_S1(ctx->opcode);
    const9 = MASK_OP_RC_CONST9_SEXT(ctx->opcode);

    op2 = MASK_OP_RC_OP2(ctx->opcode);

    temp = tcg_temp_new();

    switch (op2) {
    case OPC2_32_RC_ABSDIF:
        gen_absdifi(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_ABSDIFS:
        gen_absdifsi(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_ADD:
        gen_addi_d(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_ADDC:
        gen_addci_CC(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_ADDS:
        gen_addsi(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_ADDS_U:
        gen_addsui(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_ADDX:
        gen_addi_CC(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_AND_EQ:
        gen_accumulating_condi(TCG_COND_EQ, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_and_tl);
        break;
    case OPC2_32_RC_AND_GE:
        gen_accumulating_condi(TCG_COND_GE, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_and_tl);
        break;
    case OPC2_32_RC_AND_GE_U:
        const9 = MASK_OP_RC_CONST9(ctx->opcode);
        gen_accumulating_condi(TCG_COND_GEU, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_and_tl);
        break;
    case OPC2_32_RC_AND_LT:
        gen_accumulating_condi(TCG_COND_LT, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_and_tl);
        break;
    case OPC2_32_RC_AND_LT_U:
        const9 = MASK_OP_RC_CONST9(ctx->opcode);
        gen_accumulating_condi(TCG_COND_LTU, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_and_tl);
        break;
    case OPC2_32_RC_AND_NE:
        gen_accumulating_condi(TCG_COND_NE, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_and_tl);
        break;
    case OPC2_32_RC_EQ:
        tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_EQANY_B:
        gen_eqany_bi(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_EQANY_H:
        gen_eqany_hi(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_GE:
        tcg_gen_setcondi_tl(TCG_COND_GE, cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_GE_U:
        const9 = MASK_OP_RC_CONST9(ctx->opcode);
        tcg_gen_setcondi_tl(TCG_COND_GEU, cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_LT:
        tcg_gen_setcondi_tl(TCG_COND_LT, cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_LT_U:
        const9 = MASK_OP_RC_CONST9(ctx->opcode);
        tcg_gen_setcondi_tl(TCG_COND_LTU, cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_MAX:
        tcg_gen_movi_tl(temp, const9);
        tcg_gen_movcond_tl(TCG_COND_GT, cpu_gpr_d[r2], cpu_gpr_d[r1], temp,
                           cpu_gpr_d[r1], temp);
        break;
    case OPC2_32_RC_MAX_U:
        tcg_gen_movi_tl(temp, MASK_OP_RC_CONST9(ctx->opcode));
        tcg_gen_movcond_tl(TCG_COND_GTU, cpu_gpr_d[r2], cpu_gpr_d[r1], temp,
                           cpu_gpr_d[r1], temp);
        break;
    case OPC2_32_RC_MIN:
        tcg_gen_movi_tl(temp, const9);
        tcg_gen_movcond_tl(TCG_COND_LT, cpu_gpr_d[r2], cpu_gpr_d[r1], temp,
                           cpu_gpr_d[r1], temp);
        break;
    case OPC2_32_RC_MIN_U:
        tcg_gen_movi_tl(temp, MASK_OP_RC_CONST9(ctx->opcode));
        tcg_gen_movcond_tl(TCG_COND_LTU, cpu_gpr_d[r2], cpu_gpr_d[r1], temp,
                           cpu_gpr_d[r1], temp);
        break;
    case OPC2_32_RC_NE:
        tcg_gen_setcondi_tl(TCG_COND_NE, cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_OR_EQ:
        gen_accumulating_condi(TCG_COND_EQ, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_or_tl);
        break;
    case OPC2_32_RC_OR_GE:
        gen_accumulating_condi(TCG_COND_GE, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_or_tl);
        break;
    case OPC2_32_RC_OR_GE_U:
        const9 = MASK_OP_RC_CONST9(ctx->opcode);
        gen_accumulating_condi(TCG_COND_GEU, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_or_tl);
        break;
    case OPC2_32_RC_OR_LT:
        gen_accumulating_condi(TCG_COND_LT, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_or_tl);
        break;
    case OPC2_32_RC_OR_LT_U:
        const9 = MASK_OP_RC_CONST9(ctx->opcode);
        gen_accumulating_condi(TCG_COND_LTU, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_or_tl);
        break;
    case OPC2_32_RC_OR_NE:
        gen_accumulating_condi(TCG_COND_NE, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_or_tl);
        break;
    case OPC2_32_RC_RSUB:
        tcg_gen_movi_tl(temp, const9);
        gen_sub_d(cpu_gpr_d[r2], temp, cpu_gpr_d[r1]);
        break;
    case OPC2_32_RC_RSUBS:
        tcg_gen_movi_tl(temp, const9);
        gen_subs(cpu_gpr_d[r2], temp, cpu_gpr_d[r1]);
        break;
    case OPC2_32_RC_RSUBS_U:
        tcg_gen_movi_tl(temp, const9);
        gen_subsu(cpu_gpr_d[r2], temp, cpu_gpr_d[r1]);
        break;
    case OPC2_32_RC_SH_EQ:
        gen_sh_condi(TCG_COND_EQ, cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_SH_GE:
        gen_sh_condi(TCG_COND_GE, cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_SH_GE_U:
        const9 = MASK_OP_RC_CONST9(ctx->opcode);
        gen_sh_condi(TCG_COND_GEU, cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_SH_LT:
        gen_sh_condi(TCG_COND_LT, cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_SH_LT_U:
        const9 = MASK_OP_RC_CONST9(ctx->opcode);
        gen_sh_condi(TCG_COND_LTU, cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_SH_NE:
        gen_sh_condi(TCG_COND_NE, cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_XOR_EQ:
        gen_accumulating_condi(TCG_COND_EQ, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_xor_tl);
        break;
    case OPC2_32_RC_XOR_GE:
        gen_accumulating_condi(TCG_COND_GE, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_xor_tl);
        break;
    case OPC2_32_RC_XOR_GE_U:
        const9 = MASK_OP_RC_CONST9(ctx->opcode);
        gen_accumulating_condi(TCG_COND_GEU, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_xor_tl);
        break;
    case OPC2_32_RC_XOR_LT:
        gen_accumulating_condi(TCG_COND_LT, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_xor_tl);
        break;
    case OPC2_32_RC_XOR_LT_U:
        const9 = MASK_OP_RC_CONST9(ctx->opcode);
        gen_accumulating_condi(TCG_COND_LTU, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_xor_tl);
        break;
    case OPC2_32_RC_XOR_NE:
        gen_accumulating_condi(TCG_COND_NE, cpu_gpr_d[r2], cpu_gpr_d[r1],
                               const9, &tcg_gen_xor_tl);
        break;
    }
    tcg_temp_free(temp);
}

static void decode_rc_serviceroutine(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    uint32_t const9;

    op2 = MASK_OP_RC_OP2(ctx->opcode);
    const9 = MASK_OP_RC_CONST9(ctx->opcode);

    switch (op2) {
    case OPC2_32_RC_BISR:
        gen_helper_1arg(bisr, const9);
        break;
    case OPC2_32_RC_SYSCALL:
        /* TODO: Add exception generation */
        break;
    }
}

static void decode_rc_mul(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2;
    int16_t const9;

    r2 = MASK_OP_RC_D(ctx->opcode);
    r1 = MASK_OP_RC_S1(ctx->opcode);
    const9 = MASK_OP_RC_CONST9_SEXT(ctx->opcode);

    op2 = MASK_OP_RC_OP2(ctx->opcode);

    switch (op2) {
    case OPC2_32_RC_MUL_32:
        gen_muli_i32s(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_MUL_64:
        gen_muli_i64s(cpu_gpr_d[r2], cpu_gpr_d[r2+1], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_MULS_32:
        gen_mulsi_i32(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_MUL_U_64:
        const9 = MASK_OP_RC_CONST9(ctx->opcode);
        gen_muli_i64u(cpu_gpr_d[r2], cpu_gpr_d[r2+1], cpu_gpr_d[r1], const9);
        break;
    case OPC2_32_RC_MULS_U_32:
        const9 = MASK_OP_RC_CONST9(ctx->opcode);
        gen_mulsui_i32(cpu_gpr_d[r2], cpu_gpr_d[r1], const9);
        break;
    }
}

/* RCPW format */
static void decode_rcpw_insert(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2;
    int32_t pos, width, const4;

    TCGv temp;

    op2    = MASK_OP_RCPW_OP2(ctx->opcode);
    r1     = MASK_OP_RCPW_S1(ctx->opcode);
    r2     = MASK_OP_RCPW_D(ctx->opcode);
    const4 = MASK_OP_RCPW_CONST4(ctx->opcode);
    width  = MASK_OP_RCPW_WIDTH(ctx->opcode);
    pos    = MASK_OP_RCPW_POS(ctx->opcode);

    switch (op2) {
    case OPC2_32_RCPW_IMASK:
        /* if pos + width > 31 undefined result */
        if (pos + width <= 31) {
            tcg_gen_movi_tl(cpu_gpr_d[r2+1], ((1u << width) - 1) << pos);
            tcg_gen_movi_tl(cpu_gpr_d[r2], (const4 << pos));
        }
        break;
    case OPC2_32_RCPW_INSERT:
        /* if pos + width > 32 undefined result */
        if (pos + width <= 32) {
            temp = tcg_const_i32(const4);
            tcg_gen_deposit_tl(cpu_gpr_d[r2], cpu_gpr_d[r1], temp, pos, width);
            tcg_temp_free(temp);
        }
        break;
    }
}

/* RCRW format */

static void decode_rcrw_insert(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r3, r4;
    int32_t width, const4;

    TCGv temp, temp2, temp3;

    op2    = MASK_OP_RCRW_OP2(ctx->opcode);
    r1     = MASK_OP_RCRW_S1(ctx->opcode);
    r3     = MASK_OP_RCRW_S3(ctx->opcode);
    r4     = MASK_OP_RCRW_D(ctx->opcode);
    width  = MASK_OP_RCRW_WIDTH(ctx->opcode);
    const4 = MASK_OP_RCRW_CONST4(ctx->opcode);

    temp = tcg_temp_new();
    temp2 = tcg_temp_new();

    switch (op2) {
    case OPC2_32_RCRW_IMASK:
        tcg_gen_andi_tl(temp, cpu_gpr_d[r4], 0x1f);
        tcg_gen_movi_tl(temp2, (1 << width) - 1);
        tcg_gen_shl_tl(cpu_gpr_d[r3 + 1], temp2, temp);
        tcg_gen_movi_tl(temp2, const4);
        tcg_gen_shl_tl(cpu_gpr_d[r3], temp2, temp);
        break;
    case OPC2_32_RCRW_INSERT:
        temp3 = tcg_temp_new();

        tcg_gen_movi_tl(temp, width);
        tcg_gen_movi_tl(temp2, const4);
        tcg_gen_andi_tl(temp3, cpu_gpr_d[r4], 0x1f);
        gen_insert(cpu_gpr_d[r3], cpu_gpr_d[r1], temp2, temp, temp3);

        tcg_temp_free(temp3);
        break;
    }
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

/* RCR format */

static void decode_rcr_cond_select(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r3, r4;
    int32_t const9;

    TCGv temp, temp2;

    op2 = MASK_OP_RCR_OP2(ctx->opcode);
    r1 = MASK_OP_RCR_S1(ctx->opcode);
    const9 = MASK_OP_RCR_CONST9_SEXT(ctx->opcode);
    r3 = MASK_OP_RCR_S3(ctx->opcode);
    r4 = MASK_OP_RCR_D(ctx->opcode);

    switch (op2) {
    case OPC2_32_RCR_CADD:
        gen_condi_add(TCG_COND_NE, cpu_gpr_d[r1], const9, cpu_gpr_d[r3],
                      cpu_gpr_d[r4]);
        break;
    case OPC2_32_RCR_CADDN:
        gen_condi_add(TCG_COND_EQ, cpu_gpr_d[r1], const9, cpu_gpr_d[r3],
                      cpu_gpr_d[r4]);
        break;
    case OPC2_32_RCR_SEL:
        temp = tcg_const_i32(0);
        temp2 = tcg_const_i32(const9);
        tcg_gen_movcond_tl(TCG_COND_NE, cpu_gpr_d[r4], cpu_gpr_d[r3], temp,
                           cpu_gpr_d[r1], temp2);
        tcg_temp_free(temp);
        tcg_temp_free(temp2);
        break;
    case OPC2_32_RCR_SELN:
        temp = tcg_const_i32(0);
        temp2 = tcg_const_i32(const9);
        tcg_gen_movcond_tl(TCG_COND_EQ, cpu_gpr_d[r4], cpu_gpr_d[r3], temp,
                           cpu_gpr_d[r1], temp2);
        tcg_temp_free(temp);
        tcg_temp_free(temp2);
        break;
    }
}

static void decode_rcr_madd(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r3, r4;
    int32_t const9;


    op2 = MASK_OP_RCR_OP2(ctx->opcode);
    r1 = MASK_OP_RCR_S1(ctx->opcode);
    const9 = MASK_OP_RCR_CONST9_SEXT(ctx->opcode);
    r3 = MASK_OP_RCR_S3(ctx->opcode);
    r4 = MASK_OP_RCR_D(ctx->opcode);

    switch (op2) {
    case OPC2_32_RCR_MADD_32:
        gen_maddi32_d(cpu_gpr_d[r4], cpu_gpr_d[r1], cpu_gpr_d[r3], const9);
        break;
    case OPC2_32_RCR_MADD_64:
        gen_maddi64_d(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                      cpu_gpr_d[r3], cpu_gpr_d[r3+1], const9);
        break;
    case OPC2_32_RCR_MADDS_32:
        gen_maddsi_32(cpu_gpr_d[r4], cpu_gpr_d[r1], cpu_gpr_d[r3], const9);
        break;
    case OPC2_32_RCR_MADDS_64:
        gen_maddsi_64(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                      cpu_gpr_d[r3], cpu_gpr_d[r3+1], const9);
        break;
    case OPC2_32_RCR_MADD_U_64:
        const9 = MASK_OP_RCR_CONST9(ctx->opcode);
        gen_maddui64_d(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                       cpu_gpr_d[r3], cpu_gpr_d[r3+1], const9);
        break;
    case OPC2_32_RCR_MADDS_U_32:
        const9 = MASK_OP_RCR_CONST9(ctx->opcode);
        gen_maddsui_32(cpu_gpr_d[r4], cpu_gpr_d[r1], cpu_gpr_d[r3], const9);
        break;
    case OPC2_32_RCR_MADDS_U_64:
        const9 = MASK_OP_RCR_CONST9(ctx->opcode);
        gen_maddsui_64(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                       cpu_gpr_d[r3], cpu_gpr_d[r3+1], const9);
        break;
    }
}

static void decode_rcr_msub(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r3, r4;
    int32_t const9;


    op2 = MASK_OP_RCR_OP2(ctx->opcode);
    r1 = MASK_OP_RCR_S1(ctx->opcode);
    const9 = MASK_OP_RCR_CONST9_SEXT(ctx->opcode);
    r3 = MASK_OP_RCR_S3(ctx->opcode);
    r4 = MASK_OP_RCR_D(ctx->opcode);

    switch (op2) {
    case OPC2_32_RCR_MSUB_32:
        gen_msubi32_d(cpu_gpr_d[r4], cpu_gpr_d[r1], cpu_gpr_d[r3], const9);
        break;
    case OPC2_32_RCR_MSUB_64:
        gen_msubi64_d(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                      cpu_gpr_d[r3], cpu_gpr_d[r3+1], const9);
        break;
    case OPC2_32_RCR_MSUBS_32:
        gen_msubsi_32(cpu_gpr_d[r4], cpu_gpr_d[r1], cpu_gpr_d[r3], const9);
        break;
    case OPC2_32_RCR_MSUBS_64:
        gen_msubsi_64(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                      cpu_gpr_d[r3], cpu_gpr_d[r3+1], const9);
        break;
    case OPC2_32_RCR_MSUB_U_64:
        const9 = MASK_OP_RCR_CONST9(ctx->opcode);
        gen_msubui64_d(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                       cpu_gpr_d[r3], cpu_gpr_d[r3+1], const9);
        break;
    case OPC2_32_RCR_MSUBS_U_32:
        const9 = MASK_OP_RCR_CONST9(ctx->opcode);
        gen_msubsui_32(cpu_gpr_d[r4], cpu_gpr_d[r1], cpu_gpr_d[r3], const9);
        break;
    case OPC2_32_RCR_MSUBS_U_64:
        const9 = MASK_OP_RCR_CONST9(ctx->opcode);
        gen_msubsui_64(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                       cpu_gpr_d[r3], cpu_gpr_d[r3+1], const9);
        break;
    }
}

/* RLC format */

static void decode_rlc_opc(CPUTriCoreState *env, DisasContext *ctx,
                           uint32_t op1)
{
    int32_t const16;
    int r1, r2;

    const16 = MASK_OP_RLC_CONST16_SEXT(ctx->opcode);
    r1      = MASK_OP_RLC_S1(ctx->opcode);
    r2      = MASK_OP_RLC_D(ctx->opcode);

    switch (op1) {
    case OPC1_32_RLC_ADDI:
        gen_addi_d(cpu_gpr_d[r2], cpu_gpr_d[r1], const16);
        break;
    case OPC1_32_RLC_ADDIH:
        gen_addi_d(cpu_gpr_d[r2], cpu_gpr_d[r1], const16 << 16);
        break;
    case OPC1_32_RLC_ADDIH_A:
        tcg_gen_addi_tl(cpu_gpr_a[r2], cpu_gpr_a[r1], const16 << 16);
        break;
    case OPC1_32_RLC_MFCR:
        const16 = MASK_OP_RLC_CONST16(ctx->opcode);
        gen_mfcr(env, cpu_gpr_d[r2], const16);
        break;
    case OPC1_32_RLC_MOV:
        tcg_gen_movi_tl(cpu_gpr_d[r2], const16);
        break;
    case OPC1_32_RLC_MOV_64:
        if (tricore_feature(env, TRICORE_FEATURE_16)) {
            if ((r2 & 0x1) != 0) {
                /* TODO: raise OPD trap */
            }
            tcg_gen_movi_tl(cpu_gpr_d[r2], const16);
            tcg_gen_movi_tl(cpu_gpr_d[r2+1], const16 >> 15);
        } else {
            /* TODO: raise illegal opcode trap */
        }
        break;
    case OPC1_32_RLC_MOV_U:
        const16 = MASK_OP_RLC_CONST16(ctx->opcode);
        tcg_gen_movi_tl(cpu_gpr_d[r2], const16);
        break;
    case OPC1_32_RLC_MOV_H:
        tcg_gen_movi_tl(cpu_gpr_d[r2], const16 << 16);
        break;
    case OPC1_32_RLC_MOVH_A:
        tcg_gen_movi_tl(cpu_gpr_a[r2], const16 << 16);
        break;
    case OPC1_32_RLC_MTCR:
        const16 = MASK_OP_RLC_CONST16(ctx->opcode);
        gen_mtcr(env, ctx, cpu_gpr_d[r1], const16);
        break;
    }
}

/* RR format */
static void decode_rr_accumulator(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r3, r2, r1;

    r3 = MASK_OP_RR_D(ctx->opcode);
    r2 = MASK_OP_RR_S2(ctx->opcode);
    r1 = MASK_OP_RR_S1(ctx->opcode);
    op2 = MASK_OP_RR_OP2(ctx->opcode);

    switch (op2) {
    case OPC2_32_RR_ABS:
        gen_abs(cpu_gpr_d[r3], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ABS_B:
        gen_helper_abs_b(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ABS_H:
        gen_helper_abs_h(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ABSDIF:
        gen_absdif(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ABSDIF_B:
        gen_helper_absdif_b(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1],
                            cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ABSDIF_H:
        gen_helper_absdif_h(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1],
                            cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ABSDIFS:
        gen_helper_absdif_ssov(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1],
                               cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ABSDIFS_H:
        gen_helper_absdif_h_ssov(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1],
                                 cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ABSS:
        gen_helper_abs_ssov(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ABSS_H:
        gen_helper_abs_h_ssov(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ADD:
        gen_add_d(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ADD_B:
        gen_helper_add_b(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ADD_H:
        gen_helper_add_h(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ADDC:
        gen_addc_CC(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ADDS:
        gen_adds(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ADDS_H:
        gen_helper_add_h_ssov(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1],
                              cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ADDS_HU:
        gen_helper_add_h_suov(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1],
                              cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ADDS_U:
        gen_helper_add_suov(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1],
                            cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ADDX:
        gen_add_CC(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_AND_EQ:
        gen_accumulating_cond(TCG_COND_EQ, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_and_tl);
        break;
    case OPC2_32_RR_AND_GE:
        gen_accumulating_cond(TCG_COND_GE, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_and_tl);
        break;
    case OPC2_32_RR_AND_GE_U:
        gen_accumulating_cond(TCG_COND_GEU, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_and_tl);
        break;
    case OPC2_32_RR_AND_LT:
        gen_accumulating_cond(TCG_COND_LT, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_and_tl);
        break;
    case OPC2_32_RR_AND_LT_U:
        gen_accumulating_cond(TCG_COND_LTU, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_and_tl);
        break;
    case OPC2_32_RR_AND_NE:
        gen_accumulating_cond(TCG_COND_NE, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_and_tl);
        break;
    case OPC2_32_RR_EQ:
        tcg_gen_setcond_tl(TCG_COND_EQ, cpu_gpr_d[r3], cpu_gpr_d[r1],
                           cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_EQ_B:
        gen_helper_eq_b(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_EQ_H:
        gen_helper_eq_h(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_EQ_W:
        gen_cond_w(TCG_COND_EQ, cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_EQANY_B:
        gen_helper_eqany_b(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_EQANY_H:
        gen_helper_eqany_h(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_GE:
        tcg_gen_setcond_tl(TCG_COND_GE, cpu_gpr_d[r3], cpu_gpr_d[r1],
                           cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_GE_U:
        tcg_gen_setcond_tl(TCG_COND_GEU, cpu_gpr_d[r3], cpu_gpr_d[r1],
                           cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_LT:
        tcg_gen_setcond_tl(TCG_COND_LT, cpu_gpr_d[r3], cpu_gpr_d[r1],
                           cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_LT_U:
        tcg_gen_setcond_tl(TCG_COND_LTU, cpu_gpr_d[r3], cpu_gpr_d[r1],
                           cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_LT_B:
        gen_helper_lt_b(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_LT_BU:
        gen_helper_lt_bu(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_LT_H:
        gen_helper_lt_h(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_LT_HU:
        gen_helper_lt_hu(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_LT_W:
        gen_cond_w(TCG_COND_LT, cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_LT_WU:
        gen_cond_w(TCG_COND_LTU, cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MAX:
        tcg_gen_movcond_tl(TCG_COND_GT, cpu_gpr_d[r3], cpu_gpr_d[r1],
                           cpu_gpr_d[r2], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MAX_U:
        tcg_gen_movcond_tl(TCG_COND_GTU, cpu_gpr_d[r3], cpu_gpr_d[r1],
                           cpu_gpr_d[r2], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MAX_B:
        gen_helper_max_b(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MAX_BU:
        gen_helper_max_bu(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MAX_H:
        gen_helper_max_h(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MAX_HU:
        gen_helper_max_hu(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MIN:
        tcg_gen_movcond_tl(TCG_COND_LT, cpu_gpr_d[r3], cpu_gpr_d[r1],
                           cpu_gpr_d[r2], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MIN_U:
        tcg_gen_movcond_tl(TCG_COND_LTU, cpu_gpr_d[r3], cpu_gpr_d[r1],
                           cpu_gpr_d[r2], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MIN_B:
        gen_helper_min_b(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MIN_BU:
        gen_helper_min_bu(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MIN_H:
        gen_helper_min_h(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MIN_HU:
        gen_helper_min_hu(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MOV:
        tcg_gen_mov_tl(cpu_gpr_d[r3], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_NE:
        tcg_gen_setcond_tl(TCG_COND_NE, cpu_gpr_d[r3], cpu_gpr_d[r1],
                           cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_OR_EQ:
        gen_accumulating_cond(TCG_COND_EQ, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_or_tl);
        break;
    case OPC2_32_RR_OR_GE:
        gen_accumulating_cond(TCG_COND_GE, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_or_tl);
        break;
    case OPC2_32_RR_OR_GE_U:
        gen_accumulating_cond(TCG_COND_GEU, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_or_tl);
        break;
    case OPC2_32_RR_OR_LT:
        gen_accumulating_cond(TCG_COND_LT, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_or_tl);
        break;
    case OPC2_32_RR_OR_LT_U:
        gen_accumulating_cond(TCG_COND_LTU, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_or_tl);
        break;
    case OPC2_32_RR_OR_NE:
        gen_accumulating_cond(TCG_COND_NE, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_or_tl);
        break;
    case OPC2_32_RR_SAT_B:
        gen_saturate(cpu_gpr_d[r3], cpu_gpr_d[r1], 0x7f, -0x80);
        break;
    case OPC2_32_RR_SAT_BU:
        gen_saturate_u(cpu_gpr_d[r3], cpu_gpr_d[r1], 0xff);
        break;
    case OPC2_32_RR_SAT_H:
        gen_saturate(cpu_gpr_d[r3], cpu_gpr_d[r1], 0x7fff, -0x8000);
        break;
    case OPC2_32_RR_SAT_HU:
        gen_saturate_u(cpu_gpr_d[r3], cpu_gpr_d[r1], 0xffff);
        break;
    case OPC2_32_RR_SH_EQ:
        gen_sh_cond(TCG_COND_EQ, cpu_gpr_d[r3], cpu_gpr_d[r1],
                    cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SH_GE:
        gen_sh_cond(TCG_COND_GE, cpu_gpr_d[r3], cpu_gpr_d[r1],
                    cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SH_GE_U:
        gen_sh_cond(TCG_COND_GEU, cpu_gpr_d[r3], cpu_gpr_d[r1],
                    cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SH_LT:
        gen_sh_cond(TCG_COND_LT, cpu_gpr_d[r3], cpu_gpr_d[r1],
                    cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SH_LT_U:
        gen_sh_cond(TCG_COND_LTU, cpu_gpr_d[r3], cpu_gpr_d[r1],
                    cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SH_NE:
        gen_sh_cond(TCG_COND_NE, cpu_gpr_d[r3], cpu_gpr_d[r1],
                    cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SUB:
        gen_sub_d(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SUB_B:
        gen_helper_sub_b(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SUB_H:
        gen_helper_sub_h(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SUBC:
        gen_subc_CC(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SUBS:
        gen_subs(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SUBS_U:
        gen_subsu(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SUBS_H:
        gen_helper_sub_h_ssov(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1],
                              cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SUBS_HU:
        gen_helper_sub_h_suov(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1],
                              cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SUBX:
        gen_sub_CC(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_XOR_EQ:
        gen_accumulating_cond(TCG_COND_EQ, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_xor_tl);
        break;
    case OPC2_32_RR_XOR_GE:
        gen_accumulating_cond(TCG_COND_GE, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_xor_tl);
        break;
    case OPC2_32_RR_XOR_GE_U:
        gen_accumulating_cond(TCG_COND_GEU, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_xor_tl);
        break;
    case OPC2_32_RR_XOR_LT:
        gen_accumulating_cond(TCG_COND_LT, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_xor_tl);
        break;
    case OPC2_32_RR_XOR_LT_U:
        gen_accumulating_cond(TCG_COND_LTU, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_xor_tl);
        break;
    case OPC2_32_RR_XOR_NE:
        gen_accumulating_cond(TCG_COND_NE, cpu_gpr_d[r3], cpu_gpr_d[r1],
                              cpu_gpr_d[r2], &tcg_gen_xor_tl);
        break;
    }
}

static void decode_rr_logical_shift(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r3, r2, r1;
    TCGv temp;

    r3 = MASK_OP_RR_D(ctx->opcode);
    r2 = MASK_OP_RR_S2(ctx->opcode);
    r1 = MASK_OP_RR_S1(ctx->opcode);

    temp = tcg_temp_new();
    op2 = MASK_OP_RR_OP2(ctx->opcode);

    switch (op2) {
    case OPC2_32_RR_AND:
        tcg_gen_and_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ANDN:
        tcg_gen_andc_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_CLO:
        gen_helper_clo(cpu_gpr_d[r3], cpu_gpr_d[r1]);
        break;
    case OPC2_32_RR_CLO_H:
        gen_helper_clo_h(cpu_gpr_d[r3], cpu_gpr_d[r1]);
        break;
    case OPC2_32_RR_CLS:
        gen_helper_cls(cpu_gpr_d[r3], cpu_gpr_d[r1]);
        break;
    case OPC2_32_RR_CLS_H:
        gen_helper_cls_h(cpu_gpr_d[r3], cpu_gpr_d[r1]);
        break;
    case OPC2_32_RR_CLZ:
        gen_helper_clz(cpu_gpr_d[r3], cpu_gpr_d[r1]);
        break;
    case OPC2_32_RR_CLZ_H:
        gen_helper_clz_h(cpu_gpr_d[r3], cpu_gpr_d[r1]);
        break;
    case OPC2_32_RR_NAND:
        tcg_gen_nand_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_NOR:
        tcg_gen_nor_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_OR:
        tcg_gen_or_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_ORN:
        tcg_gen_orc_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SH:
        gen_helper_sh(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SH_H:
        gen_helper_sh_h(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SHA:
        gen_helper_sha(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SHA_H:
        gen_helper_sha_h(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_SHAS:
        gen_shas(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_XNOR:
        tcg_gen_eqv_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_XOR:
        tcg_gen_xor_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    }
    tcg_temp_free(temp);
}

static void decode_rr_address(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2, n;
    int r1, r2, r3;
    TCGv temp;

    op2 = MASK_OP_RR_OP2(ctx->opcode);
    r3 = MASK_OP_RR_D(ctx->opcode);
    r2 = MASK_OP_RR_S2(ctx->opcode);
    r1 = MASK_OP_RR_S1(ctx->opcode);
    n = MASK_OP_RR_N(ctx->opcode);

    switch (op2) {
    case OPC2_32_RR_ADD_A:
        tcg_gen_add_tl(cpu_gpr_a[r3], cpu_gpr_a[r1], cpu_gpr_a[r2]);
        break;
    case OPC2_32_RR_ADDSC_A:
        temp = tcg_temp_new();
        tcg_gen_shli_tl(temp, cpu_gpr_d[r1], n);
        tcg_gen_add_tl(cpu_gpr_a[r3], cpu_gpr_a[r2], temp);
        tcg_temp_free(temp);
        break;
    case OPC2_32_RR_ADDSC_AT:
        temp = tcg_temp_new();
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 3);
        tcg_gen_add_tl(temp, cpu_gpr_a[r2], temp);
        tcg_gen_andi_tl(cpu_gpr_a[r3], temp, 0xFFFFFFFC);
        tcg_temp_free(temp);
        break;
    case OPC2_32_RR_EQ_A:
        tcg_gen_setcond_tl(TCG_COND_EQ, cpu_gpr_d[r3], cpu_gpr_a[r1],
                           cpu_gpr_a[r2]);
        break;
    case OPC2_32_RR_EQZ:
        tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_gpr_d[r3], cpu_gpr_a[r1], 0);
        break;
    case OPC2_32_RR_GE_A:
        tcg_gen_setcond_tl(TCG_COND_GEU, cpu_gpr_d[r3], cpu_gpr_a[r1],
                           cpu_gpr_a[r2]);
        break;
    case OPC2_32_RR_LT_A:
        tcg_gen_setcond_tl(TCG_COND_LTU, cpu_gpr_d[r3], cpu_gpr_a[r1],
                           cpu_gpr_a[r2]);
        break;
    case OPC2_32_RR_MOV_A:
        tcg_gen_mov_tl(cpu_gpr_a[r3], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_MOV_AA:
        tcg_gen_mov_tl(cpu_gpr_a[r3], cpu_gpr_a[r2]);
        break;
    case OPC2_32_RR_MOV_D:
        tcg_gen_mov_tl(cpu_gpr_d[r3], cpu_gpr_a[r2]);
        break;
    case OPC2_32_RR_NE_A:
        tcg_gen_setcond_tl(TCG_COND_NE, cpu_gpr_d[r3], cpu_gpr_a[r1],
                           cpu_gpr_a[r2]);
        break;
    case OPC2_32_RR_NEZ_A:
        tcg_gen_setcondi_tl(TCG_COND_NE, cpu_gpr_d[r3], cpu_gpr_a[r1], 0);
        break;
    case OPC2_32_RR_SUB_A:
        tcg_gen_sub_tl(cpu_gpr_a[r3], cpu_gpr_a[r1], cpu_gpr_a[r2]);
        break;
    }
}

static void decode_rr_idirect(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1;

    op2 = MASK_OP_RR_OP2(ctx->opcode);
    r1 = MASK_OP_RR_S1(ctx->opcode);

    switch (op2) {
    case OPC2_32_RR_JI:
        tcg_gen_andi_tl(cpu_PC, cpu_gpr_a[r1], ~0x1);
        break;
    case OPC2_32_RR_JLI:
        tcg_gen_movi_tl(cpu_gpr_a[11], ctx->next_pc);
        tcg_gen_andi_tl(cpu_PC, cpu_gpr_a[r1], ~0x1);
        break;
    case OPC2_32_RR_CALLI:
        gen_helper_1arg(call, ctx->next_pc);
        tcg_gen_andi_tl(cpu_PC, cpu_gpr_a[r1], ~0x1);
        break;
    }
    tcg_gen_exit_tb(0);
    ctx->bstate = BS_BRANCH;
}

static void decode_rr_divide(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2, r3;

    TCGv temp, temp2, temp3;

    op2 = MASK_OP_RR_OP2(ctx->opcode);
    r3 = MASK_OP_RR_D(ctx->opcode);
    r2 = MASK_OP_RR_S2(ctx->opcode);
    r1 = MASK_OP_RR_S1(ctx->opcode);

    switch (op2) {
    case OPC2_32_RR_BMERGE:
        gen_helper_bmerge(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_BSPLIT:
        gen_bsplit(cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r1]);
        break;
    case OPC2_32_RR_DVINIT_B:
        gen_dvinit_b(env, cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r1],
                     cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_DVINIT_BU:
        temp = tcg_temp_new();
        temp2 = tcg_temp_new();
        temp3 = tcg_temp_new();

        tcg_gen_shri_tl(temp3, cpu_gpr_d[r1], 8);
        /* reset av */
        tcg_gen_movi_tl(cpu_PSW_AV, 0);
        if (!tricore_feature(env, TRICORE_FEATURE_131)) {
            /* overflow = (abs(D[r3+1]) >= abs(D[r2])) */
            tcg_gen_neg_tl(temp, temp3);
            /* use cpu_PSW_AV to compare against 0 */
            tcg_gen_movcond_tl(TCG_COND_LT, temp, temp3, cpu_PSW_AV,
                               temp, temp3);
            tcg_gen_neg_tl(temp2, cpu_gpr_d[r2]);
            tcg_gen_movcond_tl(TCG_COND_LT, temp2, cpu_gpr_d[r2], cpu_PSW_AV,
                               temp2, cpu_gpr_d[r2]);
            tcg_gen_setcond_tl(TCG_COND_GE, cpu_PSW_V, temp, temp2);
        } else {
            /* overflow = (D[b] == 0) */
            tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_PSW_V, cpu_gpr_d[r2], 0);
        }
        tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
        /* sv */
        tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
        /* write result */
        tcg_gen_shli_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], 24);
        tcg_gen_mov_tl(cpu_gpr_d[r3+1], temp3);

        tcg_temp_free(temp);
        tcg_temp_free(temp2);
        tcg_temp_free(temp3);
        break;
    case OPC2_32_RR_DVINIT_H:
        gen_dvinit_h(env, cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r1],
                     cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR_DVINIT_HU:
        temp = tcg_temp_new();
        temp2 = tcg_temp_new();
        temp3 = tcg_temp_new();

        tcg_gen_shri_tl(temp3, cpu_gpr_d[r1], 16);
        /* reset av */
        tcg_gen_movi_tl(cpu_PSW_AV, 0);
        if (!tricore_feature(env, TRICORE_FEATURE_131)) {
            /* overflow = (abs(D[r3+1]) >= abs(D[r2])) */
            tcg_gen_neg_tl(temp, temp3);
            /* use cpu_PSW_AV to compare against 0 */
            tcg_gen_movcond_tl(TCG_COND_LT, temp, temp3, cpu_PSW_AV,
                               temp, temp3);
            tcg_gen_neg_tl(temp2, cpu_gpr_d[r2]);
            tcg_gen_movcond_tl(TCG_COND_LT, temp2, cpu_gpr_d[r2], cpu_PSW_AV,
                               temp2, cpu_gpr_d[r2]);
            tcg_gen_setcond_tl(TCG_COND_GE, cpu_PSW_V, temp, temp2);
        } else {
            /* overflow = (D[b] == 0) */
            tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_PSW_V, cpu_gpr_d[r2], 0);
        }
        tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
        /* sv */
        tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
        /* write result */
        tcg_gen_mov_tl(cpu_gpr_d[r3+1], temp3);
        tcg_gen_shli_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], 16);
        tcg_temp_free(temp);
        tcg_temp_free(temp2);
        tcg_temp_free(temp3);
        break;
    case OPC2_32_RR_DVINIT:
        temp = tcg_temp_new();
        temp2 = tcg_temp_new();
        /* overflow = ((D[b] == 0) ||
                      ((D[b] == 0xFFFFFFFF) && (D[a] == 0x80000000))) */
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp, cpu_gpr_d[r2], 0xffffffff);
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp2, cpu_gpr_d[r1], 0x80000000);
        tcg_gen_and_tl(temp, temp, temp2);
        tcg_gen_setcondi_tl(TCG_COND_EQ, temp2, cpu_gpr_d[r2], 0);
        tcg_gen_or_tl(cpu_PSW_V, temp, temp2);
        tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
        /* sv */
        tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
        /* reset av */
       tcg_gen_movi_tl(cpu_PSW_AV, 0);
        /* write result */
        tcg_gen_mov_tl(cpu_gpr_d[r3], cpu_gpr_d[r1]);
        /* sign extend to high reg */
        tcg_gen_sari_tl(cpu_gpr_d[r3+1], cpu_gpr_d[r1], 31);
        tcg_temp_free(temp);
        tcg_temp_free(temp2);
        break;
    case OPC2_32_RR_DVINIT_U:
        /* overflow = (D[b] == 0) */
        tcg_gen_setcondi_tl(TCG_COND_EQ, cpu_PSW_V, cpu_gpr_d[r2], 0);
        tcg_gen_shli_tl(cpu_PSW_V, cpu_PSW_V, 31);
        /* sv */
        tcg_gen_or_tl(cpu_PSW_SV, cpu_PSW_SV, cpu_PSW_V);
        /* reset av */
        tcg_gen_movi_tl(cpu_PSW_AV, 0);
        /* write result */
        tcg_gen_mov_tl(cpu_gpr_d[r3], cpu_gpr_d[r1]);
        /* zero extend to high reg*/
        tcg_gen_movi_tl(cpu_gpr_d[r3+1], 0);
        break;
    case OPC2_32_RR_PARITY:
        gen_helper_parity(cpu_gpr_d[r3], cpu_gpr_d[r1]);
        break;
    case OPC2_32_RR_UNPACK:
        gen_unpack(cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r1]);
        break;
    case OPC2_32_RR_CRC32:
        if (tricore_feature(env, TRICORE_FEATURE_161)) {
            gen_helper_crc32(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        } /* TODO: else raise illegal opcode trap */
        break;
    }
}

/* RR1 Format */
static void decode_rr1_mul(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;

    int r1, r2, r3;
    TCGv n;
    TCGv_i64 temp64;

    r1 = MASK_OP_RR1_S1(ctx->opcode);
    r2 = MASK_OP_RR1_S2(ctx->opcode);
    r3 = MASK_OP_RR1_D(ctx->opcode);
    n  = tcg_const_i32(MASK_OP_RR1_N(ctx->opcode));
    op2 = MASK_OP_RR1_OP2(ctx->opcode);

    switch (op2) {
    case OPC2_32_RR1_MUL_H_32_LL:
        temp64 = tcg_temp_new_i64();
        GEN_HELPER_LL(mul_h, temp64, cpu_gpr_d[r1], cpu_gpr_d[r2], n);
        tcg_gen_extr_i64_i32(cpu_gpr_d[r3], cpu_gpr_d[r3+1], temp64);
        gen_calc_usb_mul_h(cpu_gpr_d[r3], cpu_gpr_d[r3+1]);
        tcg_temp_free_i64(temp64);
        break;
    case OPC2_32_RR1_MUL_H_32_LU:
        temp64 = tcg_temp_new_i64();
        GEN_HELPER_LU(mul_h, temp64, cpu_gpr_d[r1], cpu_gpr_d[r2], n);
        tcg_gen_extr_i64_i32(cpu_gpr_d[r3], cpu_gpr_d[r3+1], temp64);
        gen_calc_usb_mul_h(cpu_gpr_d[r3], cpu_gpr_d[r3+1]);
        tcg_temp_free_i64(temp64);
        break;
    case OPC2_32_RR1_MUL_H_32_UL:
        temp64 = tcg_temp_new_i64();
        GEN_HELPER_UL(mul_h, temp64, cpu_gpr_d[r1], cpu_gpr_d[r2], n);
        tcg_gen_extr_i64_i32(cpu_gpr_d[r3], cpu_gpr_d[r3+1], temp64);
        gen_calc_usb_mul_h(cpu_gpr_d[r3], cpu_gpr_d[r3+1]);
        tcg_temp_free_i64(temp64);
        break;
    case OPC2_32_RR1_MUL_H_32_UU:
        temp64 = tcg_temp_new_i64();
        GEN_HELPER_UU(mul_h, temp64, cpu_gpr_d[r1], cpu_gpr_d[r2], n);
        tcg_gen_extr_i64_i32(cpu_gpr_d[r3], cpu_gpr_d[r3+1], temp64);
        gen_calc_usb_mul_h(cpu_gpr_d[r3], cpu_gpr_d[r3+1]);
        tcg_temp_free_i64(temp64);
        break;
    case OPC2_32_RR1_MULM_H_64_LL:
        temp64 = tcg_temp_new_i64();
        GEN_HELPER_LL(mulm_h, temp64, cpu_gpr_d[r1], cpu_gpr_d[r2], n);
        tcg_gen_extr_i64_i32(cpu_gpr_d[r3], cpu_gpr_d[r3+1], temp64);
        /* reset V bit */
        tcg_gen_movi_tl(cpu_PSW_V, 0);
        /* reset AV bit */
        tcg_gen_mov_tl(cpu_PSW_AV, cpu_PSW_V);
        tcg_temp_free_i64(temp64);
        break;
    case OPC2_32_RR1_MULM_H_64_LU:
        temp64 = tcg_temp_new_i64();
        GEN_HELPER_LU(mulm_h, temp64, cpu_gpr_d[r1], cpu_gpr_d[r2], n);
        tcg_gen_extr_i64_i32(cpu_gpr_d[r3], cpu_gpr_d[r3+1], temp64);
        /* reset V bit */
        tcg_gen_movi_tl(cpu_PSW_V, 0);
        /* reset AV bit */
        tcg_gen_mov_tl(cpu_PSW_AV, cpu_PSW_V);
        tcg_temp_free_i64(temp64);
        break;
    case OPC2_32_RR1_MULM_H_64_UL:
        temp64 = tcg_temp_new_i64();
        GEN_HELPER_UL(mulm_h, temp64, cpu_gpr_d[r1], cpu_gpr_d[r2], n);
        tcg_gen_extr_i64_i32(cpu_gpr_d[r3], cpu_gpr_d[r3+1], temp64);
        /* reset V bit */
        tcg_gen_movi_tl(cpu_PSW_V, 0);
        /* reset AV bit */
        tcg_gen_mov_tl(cpu_PSW_AV, cpu_PSW_V);
        tcg_temp_free_i64(temp64);
        break;
    case OPC2_32_RR1_MULM_H_64_UU:
        temp64 = tcg_temp_new_i64();
        GEN_HELPER_UU(mulm_h, temp64, cpu_gpr_d[r1], cpu_gpr_d[r2], n);
        tcg_gen_extr_i64_i32(cpu_gpr_d[r3], cpu_gpr_d[r3+1], temp64);
        /* reset V bit */
        tcg_gen_movi_tl(cpu_PSW_V, 0);
        /* reset AV bit */
        tcg_gen_mov_tl(cpu_PSW_AV, cpu_PSW_V);
        tcg_temp_free_i64(temp64);

        break;
    case OPC2_32_RR1_MULR_H_16_LL:
        GEN_HELPER_LL(mulr_h, cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2], n);
        gen_calc_usb_mulr_h(cpu_gpr_d[r3]);
        break;
    case OPC2_32_RR1_MULR_H_16_LU:
        GEN_HELPER_LU(mulr_h, cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2], n);
        gen_calc_usb_mulr_h(cpu_gpr_d[r3]);
        break;
    case OPC2_32_RR1_MULR_H_16_UL:
        GEN_HELPER_UL(mulr_h, cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2], n);
        gen_calc_usb_mulr_h(cpu_gpr_d[r3]);
        break;
    case OPC2_32_RR1_MULR_H_16_UU:
        GEN_HELPER_UU(mulr_h, cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2], n);
        gen_calc_usb_mulr_h(cpu_gpr_d[r3]);
        break;
    }
    tcg_temp_free(n);
}

static void decode_rr1_mulq(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2, r3;
    uint32_t n;

    TCGv temp, temp2;

    r1 = MASK_OP_RR1_S1(ctx->opcode);
    r2 = MASK_OP_RR1_S2(ctx->opcode);
    r3 = MASK_OP_RR1_D(ctx->opcode);
    n  = MASK_OP_RR1_N(ctx->opcode);
    op2 = MASK_OP_RR1_OP2(ctx->opcode);

    temp = tcg_temp_new();
    temp2 = tcg_temp_new();

    switch (op2) {
    case OPC2_32_RR1_MUL_Q_32:
        gen_mul_q(cpu_gpr_d[r3], temp, cpu_gpr_d[r1], cpu_gpr_d[r2], n, 32);
        break;
    case OPC2_32_RR1_MUL_Q_64:
        gen_mul_q(cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                  n, 0);
        break;
    case OPC2_32_RR1_MUL_Q_32_L:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r2]);
        gen_mul_q(cpu_gpr_d[r3], temp, cpu_gpr_d[r1], temp, n, 16);
        break;
    case OPC2_32_RR1_MUL_Q_64_L:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r2]);
        gen_mul_q(cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r1], temp, n, 0);
        break;
    case OPC2_32_RR1_MUL_Q_32_U:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r2], 16);
        gen_mul_q(cpu_gpr_d[r3], temp, cpu_gpr_d[r1], temp, n, 16);
        break;
    case OPC2_32_RR1_MUL_Q_64_U:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r2], 16);
        gen_mul_q(cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r1], temp, n, 0);
        break;
    case OPC2_32_RR1_MUL_Q_32_LL:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_mul_q_16(cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RR1_MUL_Q_32_UU:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_mul_q_16(cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RR1_MULR_Q_32_L:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_mulr_q(cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RR1_MULR_Q_32_U:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_mulr_q(cpu_gpr_d[r3], temp, temp2, n);
        break;
    }
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

/* RR2 format */
static void decode_rr2_mul(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2, r3;

    op2 = MASK_OP_RR2_OP2(ctx->opcode);
    r1  = MASK_OP_RR2_S1(ctx->opcode);
    r2  = MASK_OP_RR2_S2(ctx->opcode);
    r3  = MASK_OP_RR2_D(ctx->opcode);
    switch (op2) {
    case OPC2_32_RR2_MUL_32:
        gen_mul_i32s(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR2_MUL_64:
        gen_mul_i64s(cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r1],
                     cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR2_MULS_32:
        gen_helper_mul_ssov(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1],
                            cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR2_MUL_U_64:
        gen_mul_i64u(cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r1],
                     cpu_gpr_d[r2]);
        break;
    case OPC2_32_RR2_MULS_U_32:
        gen_helper_mul_suov(cpu_gpr_d[r3], cpu_env, cpu_gpr_d[r1],
                            cpu_gpr_d[r2]);
        break;
    }
}

/* RRPW format */
static void decode_rrpw_extract_insert(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2, r3;
    int32_t pos, width;

    op2 = MASK_OP_RRPW_OP2(ctx->opcode);
    r1 = MASK_OP_RRPW_S1(ctx->opcode);
    r2 = MASK_OP_RRPW_S2(ctx->opcode);
    r3 = MASK_OP_RRPW_D(ctx->opcode);
    pos = MASK_OP_RRPW_POS(ctx->opcode);
    width = MASK_OP_RRPW_WIDTH(ctx->opcode);

    switch (op2) {
    case OPC2_32_RRPW_EXTR:
        if (pos + width <= 31) {
            /* optimize special cases */
            if ((pos == 0) && (width == 8)) {
                tcg_gen_ext8s_tl(cpu_gpr_d[r3], cpu_gpr_d[r1]);
            } else if ((pos == 0) && (width == 16)) {
                tcg_gen_ext16s_tl(cpu_gpr_d[r3], cpu_gpr_d[r1]);
            } else {
                tcg_gen_shli_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], 32 - pos - width);
                tcg_gen_sari_tl(cpu_gpr_d[r3], cpu_gpr_d[r3], 32 - width);
            }
        }
        break;
    case OPC2_32_RRPW_EXTR_U:
        if (width == 0) {
            tcg_gen_movi_tl(cpu_gpr_d[r3], 0);
        } else {
            tcg_gen_shri_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], pos);
            tcg_gen_andi_tl(cpu_gpr_d[r3], cpu_gpr_d[r3], ~0u >> (32-width));
        }
        break;
    case OPC2_32_RRPW_IMASK:
        if (pos + width <= 31) {
            tcg_gen_movi_tl(cpu_gpr_d[r3+1], ((1u << width) - 1) << pos);
            tcg_gen_shli_tl(cpu_gpr_d[r3], cpu_gpr_d[r2], pos);
        }
        break;
    case OPC2_32_RRPW_INSERT:
        if (pos + width <= 31) {
            tcg_gen_deposit_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], cpu_gpr_d[r2],
                               width, pos);
        }
        break;
    }
}

/* RRR format */
static void decode_rrr_cond_select(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2, r3, r4;
    TCGv temp;

    op2 = MASK_OP_RRR_OP2(ctx->opcode);
    r1  = MASK_OP_RRR_S1(ctx->opcode);
    r2  = MASK_OP_RRR_S2(ctx->opcode);
    r3  = MASK_OP_RRR_S3(ctx->opcode);
    r4  = MASK_OP_RRR_D(ctx->opcode);

    switch (op2) {
    case OPC2_32_RRR_CADD:
        gen_cond_add(TCG_COND_NE, cpu_gpr_d[r1], cpu_gpr_d[r2],
                     cpu_gpr_d[r4], cpu_gpr_d[r3]);
        break;
    case OPC2_32_RRR_CADDN:
        gen_cond_add(TCG_COND_EQ, cpu_gpr_d[r1], cpu_gpr_d[r2], cpu_gpr_d[r4],
                     cpu_gpr_d[r3]);
        break;
    case OPC2_32_RRR_CSUB:
        gen_cond_sub(TCG_COND_NE, cpu_gpr_d[r1], cpu_gpr_d[r2], cpu_gpr_d[r4],
                     cpu_gpr_d[r3]);
        break;
    case OPC2_32_RRR_CSUBN:
        gen_cond_sub(TCG_COND_EQ, cpu_gpr_d[r1], cpu_gpr_d[r2], cpu_gpr_d[r4],
                     cpu_gpr_d[r3]);
        break;
    case OPC2_32_RRR_SEL:
        temp = tcg_const_i32(0);
        tcg_gen_movcond_tl(TCG_COND_NE, cpu_gpr_d[r4], cpu_gpr_d[r3], temp,
                           cpu_gpr_d[r1], cpu_gpr_d[r2]);
        tcg_temp_free(temp);
        break;
    case OPC2_32_RRR_SELN:
        temp = tcg_const_i32(0);
        tcg_gen_movcond_tl(TCG_COND_EQ, cpu_gpr_d[r4], cpu_gpr_d[r3], temp,
                           cpu_gpr_d[r1], cpu_gpr_d[r2]);
        tcg_temp_free(temp);
        break;
    }
}

static void decode_rrr_divide(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;

    int r1, r2, r3, r4;

    op2 = MASK_OP_RRR_OP2(ctx->opcode);
    r1 = MASK_OP_RRR_S1(ctx->opcode);
    r2 = MASK_OP_RRR_S2(ctx->opcode);
    r3 = MASK_OP_RRR_S3(ctx->opcode);
    r4 = MASK_OP_RRR_D(ctx->opcode);

    switch (op2) {
    case OPC2_32_RRR_DVADJ:
        GEN_HELPER_RRR(dvadj, cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR_DVSTEP:
        GEN_HELPER_RRR(dvstep, cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR_DVSTEP_U:
        GEN_HELPER_RRR(dvstep_u, cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR_IXMAX:
        GEN_HELPER_RRR(ixmax, cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR_IXMAX_U:
        GEN_HELPER_RRR(ixmax_u, cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR_IXMIN:
        GEN_HELPER_RRR(ixmin, cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR_IXMIN_U:
        GEN_HELPER_RRR(ixmin_u, cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR_PACK:
        gen_helper_pack(cpu_gpr_d[r4], cpu_PSW_C, cpu_gpr_d[r3],
                        cpu_gpr_d[r3+1], cpu_gpr_d[r1]);
        break;
    }
}

/* RRR2 format */
static void decode_rrr2_madd(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    uint32_t r1, r2, r3, r4;

    op2 = MASK_OP_RRR2_OP2(ctx->opcode);
    r1 = MASK_OP_RRR2_S1(ctx->opcode);
    r2 = MASK_OP_RRR2_S2(ctx->opcode);
    r3 = MASK_OP_RRR2_S3(ctx->opcode);
    r4 = MASK_OP_RRR2_D(ctx->opcode);
    switch (op2) {
    case OPC2_32_RRR2_MADD_32:
        gen_madd32_d(cpu_gpr_d[r4], cpu_gpr_d[r1], cpu_gpr_d[r3],
                     cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR2_MADD_64:
        gen_madd64_d(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                     cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR2_MADDS_32:
        gen_helper_madd32_ssov(cpu_gpr_d[r4], cpu_env, cpu_gpr_d[r1],
                               cpu_gpr_d[r3], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR2_MADDS_64:
        gen_madds_64(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                     cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR2_MADD_U_64:
        gen_maddu64_d(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                      cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR2_MADDS_U_32:
        gen_helper_madd32_suov(cpu_gpr_d[r4], cpu_env, cpu_gpr_d[r1],
                               cpu_gpr_d[r3], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR2_MADDS_U_64:
        gen_maddsu_64(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                      cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    }
}

static void decode_rrr2_msub(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    uint32_t r1, r2, r3, r4;

    op2 = MASK_OP_RRR2_OP2(ctx->opcode);
    r1 = MASK_OP_RRR2_S1(ctx->opcode);
    r2 = MASK_OP_RRR2_S2(ctx->opcode);
    r3 = MASK_OP_RRR2_S3(ctx->opcode);
    r4 = MASK_OP_RRR2_D(ctx->opcode);

    switch (op2) {
    case OPC2_32_RRR2_MSUB_32:
        gen_msub32_d(cpu_gpr_d[r4], cpu_gpr_d[r1], cpu_gpr_d[r3],
                      cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR2_MSUB_64:
        gen_msub64_d(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                     cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR2_MSUBS_32:
        gen_helper_msub32_ssov(cpu_gpr_d[r4], cpu_env, cpu_gpr_d[r1],
                               cpu_gpr_d[r3], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR2_MSUBS_64:
        gen_msubs_64(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                     cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR2_MSUB_U_64:
        gen_msubu64_d(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                      cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR2_MSUBS_U_32:
        gen_helper_msub32_suov(cpu_gpr_d[r4], cpu_env, cpu_gpr_d[r1],
                               cpu_gpr_d[r3], cpu_gpr_d[r2]);
        break;
    case OPC2_32_RRR2_MSUBS_U_64:
        gen_msubsu_64(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r1],
                      cpu_gpr_d[r3], cpu_gpr_d[r3+1], cpu_gpr_d[r2]);
        break;
    }
}

/* RRR1 format */
static void decode_rrr1_madd(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    uint32_t r1, r2, r3, r4, n;

    op2 = MASK_OP_RRR1_OP2(ctx->opcode);
    r1 = MASK_OP_RRR1_S1(ctx->opcode);
    r2 = MASK_OP_RRR1_S2(ctx->opcode);
    r3 = MASK_OP_RRR1_S3(ctx->opcode);
    r4 = MASK_OP_RRR1_D(ctx->opcode);
    n = MASK_OP_RRR1_N(ctx->opcode);

    switch (op2) {
    case OPC2_32_RRR1_MADD_H_LL:
        gen_madd_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                   cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MADD_H_LU:
        gen_madd_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                   cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MADD_H_UL:
        gen_madd_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                   cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MADD_H_UU:
        gen_madd_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                   cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MADDS_H_LL:
        gen_madds_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MADDS_H_LU:
        gen_madds_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MADDS_H_UL:
        gen_madds_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MADDS_H_UU:
        gen_madds_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MADDM_H_LL:
        gen_maddm_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MADDM_H_LU:
        gen_maddm_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MADDM_H_UL:
        gen_maddm_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MADDM_H_UU:
        gen_maddm_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MADDMS_H_LL:
        gen_maddms_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MADDMS_H_LU:
        gen_maddms_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MADDMS_H_UL:
        gen_maddms_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MADDMS_H_UU:
        gen_maddms_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MADDR_H_LL:
        gen_maddr32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MADDR_H_LU:
        gen_maddr32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MADDR_H_UL:
        gen_maddr32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MADDR_H_UU:
        gen_maddr32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MADDRS_H_LL:
        gen_maddr32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                       cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MADDRS_H_LU:
        gen_maddr32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                       cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MADDRS_H_UL:
        gen_maddr32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                       cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MADDRS_H_UU:
        gen_maddr32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                       cpu_gpr_d[r2], n, MODE_UU);
        break;
    }
}

static void decode_rrr1_maddq_h(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    uint32_t r1, r2, r3, r4, n;
    TCGv temp, temp2;

    op2 = MASK_OP_RRR1_OP2(ctx->opcode);
    r1 = MASK_OP_RRR1_S1(ctx->opcode);
    r2 = MASK_OP_RRR1_S2(ctx->opcode);
    r3 = MASK_OP_RRR1_S3(ctx->opcode);
    r4 = MASK_OP_RRR1_D(ctx->opcode);
    n = MASK_OP_RRR1_N(ctx->opcode);

    temp = tcg_const_i32(n);
    temp2 = tcg_temp_new();

    switch (op2) {
    case OPC2_32_RRR1_MADD_Q_32:
        gen_madd32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                     cpu_gpr_d[r2], n, 32, env);
        break;
    case OPC2_32_RRR1_MADD_Q_64:
        gen_madd64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                     n, env);
        break;
    case OPC2_32_RRR1_MADD_Q_32_L:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r2]);
        gen_madd32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                     temp, n, 16, env);
        break;
    case OPC2_32_RRR1_MADD_Q_64_L:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r2]);
        gen_madd64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], temp,
                     n, env);
        break;
    case OPC2_32_RRR1_MADD_Q_32_U:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r2], 16);
        gen_madd32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                     temp, n, 16, env);
        break;
    case OPC2_32_RRR1_MADD_Q_64_U:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r2], 16);
        gen_madd64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], temp,
                     n, env);
        break;
    case OPC2_32_RRR1_MADD_Q_32_LL:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_m16add32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MADD_Q_64_LL:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_m16add64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MADD_Q_32_UU:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_m16add32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MADD_Q_64_UU:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_m16add64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MADDS_Q_32:
        gen_madds32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      cpu_gpr_d[r2], n, 32);
        break;
    case OPC2_32_RRR1_MADDS_Q_64:
        gen_madds64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n);
        break;
    case OPC2_32_RRR1_MADDS_Q_32_L:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r2]);
        gen_madds32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      temp, n, 16);
        break;
    case OPC2_32_RRR1_MADDS_Q_64_L:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r2]);
        gen_madds64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], temp,
                      n);
        break;
    case OPC2_32_RRR1_MADDS_Q_32_U:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r2], 16);
        gen_madds32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      temp, n, 16);
        break;
    case OPC2_32_RRR1_MADDS_Q_64_U:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r2], 16);
        gen_madds64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], temp,
                      n);
        break;
    case OPC2_32_RRR1_MADDS_Q_32_LL:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_m16adds32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MADDS_Q_64_LL:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_m16adds64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                        cpu_gpr_d[r3+1], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MADDS_Q_32_UU:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_m16adds32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MADDS_Q_64_UU:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_m16adds64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                        cpu_gpr_d[r3+1], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MADDR_H_64_UL:
        gen_maddr64_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r3+1],
                      cpu_gpr_d[r1], cpu_gpr_d[r2], n, 2);
        break;
    case OPC2_32_RRR1_MADDRS_H_64_UL:
        gen_maddr64s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r3+1],
                       cpu_gpr_d[r1], cpu_gpr_d[r2], n, 2);
        break;
    case OPC2_32_RRR1_MADDR_Q_32_LL:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_maddr_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MADDR_Q_32_UU:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_maddr_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MADDRS_Q_32_LL:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_maddrs_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MADDRS_Q_32_UU:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_maddrs_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    }
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static void decode_rrr1_maddsu_h(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    uint32_t r1, r2, r3, r4, n;

    op2 = MASK_OP_RRR1_OP2(ctx->opcode);
    r1 = MASK_OP_RRR1_S1(ctx->opcode);
    r2 = MASK_OP_RRR1_S2(ctx->opcode);
    r3 = MASK_OP_RRR1_S3(ctx->opcode);
    r4 = MASK_OP_RRR1_D(ctx->opcode);
    n = MASK_OP_RRR1_N(ctx->opcode);

    switch (op2) {
    case OPC2_32_RRR1_MADDSU_H_32_LL:
        gen_maddsu_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MADDSU_H_32_LU:
        gen_maddsu_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MADDSU_H_32_UL:
        gen_maddsu_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MADDSU_H_32_UU:
        gen_maddsu_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MADDSUS_H_32_LL:
        gen_maddsus_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_LL);
        break;
    case OPC2_32_RRR1_MADDSUS_H_32_LU:
        gen_maddsus_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_LU);
        break;
    case OPC2_32_RRR1_MADDSUS_H_32_UL:
        gen_maddsus_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_UL);
        break;
    case OPC2_32_RRR1_MADDSUS_H_32_UU:
        gen_maddsus_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_UU);
        break;
    case OPC2_32_RRR1_MADDSUM_H_64_LL:
        gen_maddsum_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_LL);
        break;
    case OPC2_32_RRR1_MADDSUM_H_64_LU:
        gen_maddsum_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_LU);
        break;
    case OPC2_32_RRR1_MADDSUM_H_64_UL:
        gen_maddsum_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_UL);
        break;
    case OPC2_32_RRR1_MADDSUM_H_64_UU:
        gen_maddsum_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_UU);
        break;
    case OPC2_32_RRR1_MADDSUMS_H_64_LL:
        gen_maddsums_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                       n, MODE_LL);
        break;
    case OPC2_32_RRR1_MADDSUMS_H_64_LU:
        gen_maddsums_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                       n, MODE_LU);
        break;
    case OPC2_32_RRR1_MADDSUMS_H_64_UL:
        gen_maddsums_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                       n, MODE_UL);
        break;
    case OPC2_32_RRR1_MADDSUMS_H_64_UU:
        gen_maddsums_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                       n, MODE_UU);
        break;
    case OPC2_32_RRR1_MADDSUR_H_16_LL:
        gen_maddsur32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                        cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MADDSUR_H_16_LU:
        gen_maddsur32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                        cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MADDSUR_H_16_UL:
        gen_maddsur32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                        cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MADDSUR_H_16_UU:
        gen_maddsur32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                        cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MADDSURS_H_16_LL:
        gen_maddsur32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                         cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MADDSURS_H_16_LU:
        gen_maddsur32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                         cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MADDSURS_H_16_UL:
        gen_maddsur32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                         cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MADDSURS_H_16_UU:
        gen_maddsur32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                         cpu_gpr_d[r2], n, MODE_UU);
        break;
    }
}

static void decode_rrr1_msub(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    uint32_t r1, r2, r3, r4, n;

    op2 = MASK_OP_RRR1_OP2(ctx->opcode);
    r1 = MASK_OP_RRR1_S1(ctx->opcode);
    r2 = MASK_OP_RRR1_S2(ctx->opcode);
    r3 = MASK_OP_RRR1_S3(ctx->opcode);
    r4 = MASK_OP_RRR1_D(ctx->opcode);
    n = MASK_OP_RRR1_N(ctx->opcode);

    switch (op2) {
    case OPC2_32_RRR1_MSUB_H_LL:
        gen_msub_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                   cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MSUB_H_LU:
        gen_msub_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                   cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MSUB_H_UL:
        gen_msub_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                   cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MSUB_H_UU:
        gen_msub_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                   cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MSUBS_H_LL:
        gen_msubs_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MSUBS_H_LU:
        gen_msubs_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MSUBS_H_UL:
        gen_msubs_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MSUBS_H_UU:
        gen_msubs_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MSUBM_H_LL:
        gen_msubm_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MSUBM_H_LU:
        gen_msubm_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MSUBM_H_UL:
        gen_msubm_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MSUBM_H_UU:
        gen_msubm_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                    cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MSUBMS_H_LL:
        gen_msubms_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MSUBMS_H_LU:
        gen_msubms_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MSUBMS_H_UL:
        gen_msubms_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MSUBMS_H_UU:
        gen_msubms_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MSUBR_H_LL:
        gen_msubr32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MSUBR_H_LU:
        gen_msubr32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MSUBR_H_UL:
        gen_msubr32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MSUBR_H_UU:
        gen_msubr32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MSUBRS_H_LL:
        gen_msubr32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                       cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MSUBRS_H_LU:
        gen_msubr32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                       cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MSUBRS_H_UL:
        gen_msubr32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                       cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MSUBRS_H_UU:
        gen_msubr32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                       cpu_gpr_d[r2], n, MODE_UU);
        break;
    }
}

static void decode_rrr1_msubq_h(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    uint32_t r1, r2, r3, r4, n;
    TCGv temp, temp2;

    op2 = MASK_OP_RRR1_OP2(ctx->opcode);
    r1 = MASK_OP_RRR1_S1(ctx->opcode);
    r2 = MASK_OP_RRR1_S2(ctx->opcode);
    r3 = MASK_OP_RRR1_S3(ctx->opcode);
    r4 = MASK_OP_RRR1_D(ctx->opcode);
    n = MASK_OP_RRR1_N(ctx->opcode);

    temp = tcg_const_i32(n);
    temp2 = tcg_temp_new();

    switch (op2) {
    case OPC2_32_RRR1_MSUB_Q_32:
        gen_msub32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                     cpu_gpr_d[r2], n, 32, env);
        break;
    case OPC2_32_RRR1_MSUB_Q_64:
        gen_msub64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                     n, env);
        break;
    case OPC2_32_RRR1_MSUB_Q_32_L:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r2]);
        gen_msub32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                     temp, n, 16, env);
        break;
    case OPC2_32_RRR1_MSUB_Q_64_L:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r2]);
        gen_msub64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], temp,
                     n, env);
        break;
    case OPC2_32_RRR1_MSUB_Q_32_U:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r2], 16);
        gen_msub32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                     temp, n, 16, env);
        break;
    case OPC2_32_RRR1_MSUB_Q_64_U:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r2], 16);
        gen_msub64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], temp,
                     n, env);
        break;
    case OPC2_32_RRR1_MSUB_Q_32_LL:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_m16sub32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MSUB_Q_64_LL:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_m16sub64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MSUB_Q_32_UU:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_m16sub32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MSUB_Q_64_UU:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_m16sub64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MSUBS_Q_32:
        gen_msubs32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      cpu_gpr_d[r2], n, 32);
        break;
    case OPC2_32_RRR1_MSUBS_Q_64:
        gen_msubs64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n);
        break;
    case OPC2_32_RRR1_MSUBS_Q_32_L:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r2]);
        gen_msubs32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      temp, n, 16);
        break;
    case OPC2_32_RRR1_MSUBS_Q_64_L:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r2]);
        gen_msubs64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], temp,
                      n);
        break;
    case OPC2_32_RRR1_MSUBS_Q_32_U:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r2], 16);
        gen_msubs32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                      temp, n, 16);
        break;
    case OPC2_32_RRR1_MSUBS_Q_64_U:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r2], 16);
        gen_msubs64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], temp,
                      n);
        break;
    case OPC2_32_RRR1_MSUBS_Q_32_LL:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_m16subs32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MSUBS_Q_64_LL:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_m16subs64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                        cpu_gpr_d[r3+1], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MSUBS_Q_32_UU:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_m16subs32_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MSUBS_Q_64_UU:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_m16subs64_q(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                        cpu_gpr_d[r3+1], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MSUBR_H_64_UL:
        gen_msubr64_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r3+1],
                      cpu_gpr_d[r1], cpu_gpr_d[r2], n, 2);
        break;
    case OPC2_32_RRR1_MSUBRS_H_64_UL:
        gen_msubr64s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r3+1],
                       cpu_gpr_d[r1], cpu_gpr_d[r2], n, 2);
        break;
    case OPC2_32_RRR1_MSUBR_Q_32_LL:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_msubr_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MSUBR_Q_32_UU:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_msubr_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MSUBRS_Q_32_LL:
        tcg_gen_ext16s_tl(temp, cpu_gpr_d[r1]);
        tcg_gen_ext16s_tl(temp2, cpu_gpr_d[r2]);
        gen_msubrs_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    case OPC2_32_RRR1_MSUBRS_Q_32_UU:
        tcg_gen_sari_tl(temp, cpu_gpr_d[r1], 16);
        tcg_gen_sari_tl(temp2, cpu_gpr_d[r2], 16);
        gen_msubrs_q(cpu_gpr_d[r4], cpu_gpr_d[r3], temp, temp2, n);
        break;
    }
    tcg_temp_free(temp);
    tcg_temp_free(temp2);
}

static void decode_rrr1_msubad_h(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    uint32_t r1, r2, r3, r4, n;

    op2 = MASK_OP_RRR1_OP2(ctx->opcode);
    r1 = MASK_OP_RRR1_S1(ctx->opcode);
    r2 = MASK_OP_RRR1_S2(ctx->opcode);
    r3 = MASK_OP_RRR1_S3(ctx->opcode);
    r4 = MASK_OP_RRR1_D(ctx->opcode);
    n = MASK_OP_RRR1_N(ctx->opcode);

    switch (op2) {
    case OPC2_32_RRR1_MSUBAD_H_32_LL:
        gen_msubad_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MSUBAD_H_32_LU:
        gen_msubad_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MSUBAD_H_32_UL:
        gen_msubad_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MSUBAD_H_32_UU:
        gen_msubad_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                     cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MSUBADS_H_32_LL:
        gen_msubads_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_LL);
        break;
    case OPC2_32_RRR1_MSUBADS_H_32_LU:
        gen_msubads_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_LU);
        break;
    case OPC2_32_RRR1_MSUBADS_H_32_UL:
        gen_msubads_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_UL);
        break;
    case OPC2_32_RRR1_MSUBADS_H_32_UU:
        gen_msubads_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_UU);
        break;
    case OPC2_32_RRR1_MSUBADM_H_64_LL:
        gen_msubadm_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_LL);
        break;
    case OPC2_32_RRR1_MSUBADM_H_64_LU:
        gen_msubadm_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_LU);
        break;
    case OPC2_32_RRR1_MSUBADM_H_64_UL:
        gen_msubadm_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_UL);
        break;
    case OPC2_32_RRR1_MSUBADM_H_64_UU:
        gen_msubadm_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                      cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                      n, MODE_UU);
        break;
    case OPC2_32_RRR1_MSUBADMS_H_64_LL:
        gen_msubadms_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                       n, MODE_LL);
        break;
    case OPC2_32_RRR1_MSUBADMS_H_64_LU:
        gen_msubadms_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                       n, MODE_LU);
        break;
    case OPC2_32_RRR1_MSUBADMS_H_64_UL:
        gen_msubadms_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                       n, MODE_UL);
        break;
    case OPC2_32_RRR1_MSUBADMS_H_64_UU:
        gen_msubadms_h(cpu_gpr_d[r4], cpu_gpr_d[r4+1], cpu_gpr_d[r3],
                       cpu_gpr_d[r3+1], cpu_gpr_d[r1], cpu_gpr_d[r2],
                       n, MODE_UU);
        break;
    case OPC2_32_RRR1_MSUBADR_H_16_LL:
        gen_msubadr32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                        cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MSUBADR_H_16_LU:
        gen_msubadr32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                        cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MSUBADR_H_16_UL:
        gen_msubadr32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                        cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MSUBADR_H_16_UU:
        gen_msubadr32_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                        cpu_gpr_d[r2], n, MODE_UU);
        break;
    case OPC2_32_RRR1_MSUBADRS_H_16_LL:
        gen_msubadr32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                         cpu_gpr_d[r2], n, MODE_LL);
        break;
    case OPC2_32_RRR1_MSUBADRS_H_16_LU:
        gen_msubadr32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                         cpu_gpr_d[r2], n, MODE_LU);
        break;
    case OPC2_32_RRR1_MSUBADRS_H_16_UL:
        gen_msubadr32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                         cpu_gpr_d[r2], n, MODE_UL);
        break;
    case OPC2_32_RRR1_MSUBADRS_H_16_UU:
        gen_msubadr32s_h(cpu_gpr_d[r4], cpu_gpr_d[r3], cpu_gpr_d[r1],
                         cpu_gpr_d[r2], n, MODE_UU);
        break;
    }
}

/* RRRR format */
static void decode_rrrr_extract_insert(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2, r3, r4;
    TCGv tmp_width, tmp_pos;

    r1 = MASK_OP_RRRR_S1(ctx->opcode);
    r2 = MASK_OP_RRRR_S2(ctx->opcode);
    r3 = MASK_OP_RRRR_S3(ctx->opcode);
    r4 = MASK_OP_RRRR_D(ctx->opcode);
    op2 = MASK_OP_RRRR_OP2(ctx->opcode);

    tmp_pos = tcg_temp_new();
    tmp_width = tcg_temp_new();

    switch (op2) {
    case OPC2_32_RRRR_DEXTR:
        tcg_gen_andi_tl(tmp_pos, cpu_gpr_d[r3], 0x1f);
        if (r1 == r2) {
            tcg_gen_rotl_tl(cpu_gpr_d[r4], cpu_gpr_d[r1], tmp_pos);
        } else {
            tcg_gen_shl_tl(tmp_width, cpu_gpr_d[r1], tmp_pos);
            tcg_gen_subfi_tl(tmp_pos, 32, tmp_pos);
            tcg_gen_shr_tl(tmp_pos, cpu_gpr_d[r2], tmp_pos);
            tcg_gen_or_tl(cpu_gpr_d[r4], tmp_width, tmp_pos);
        }
        break;
    case OPC2_32_RRRR_EXTR:
    case OPC2_32_RRRR_EXTR_U:
        tcg_gen_andi_tl(tmp_width, cpu_gpr_d[r3+1], 0x1f);
        tcg_gen_andi_tl(tmp_pos, cpu_gpr_d[r3], 0x1f);
        tcg_gen_add_tl(tmp_pos, tmp_pos, tmp_width);
        tcg_gen_subfi_tl(tmp_pos, 32, tmp_pos);
        tcg_gen_shl_tl(cpu_gpr_d[r4], cpu_gpr_d[r1], tmp_pos);
        tcg_gen_subfi_tl(tmp_width, 32, tmp_width);
        if (op2 == OPC2_32_RRRR_EXTR) {
            tcg_gen_sar_tl(cpu_gpr_d[r4], cpu_gpr_d[r4], tmp_width);
        } else {
            tcg_gen_shr_tl(cpu_gpr_d[r4], cpu_gpr_d[r4], tmp_width);
        }
        break;
    case OPC2_32_RRRR_INSERT:
        tcg_gen_andi_tl(tmp_width, cpu_gpr_d[r3+1], 0x1f);
        tcg_gen_andi_tl(tmp_pos, cpu_gpr_d[r3], 0x1f);
        gen_insert(cpu_gpr_d[r4], cpu_gpr_d[r1], cpu_gpr_d[r2], tmp_width,
                   tmp_pos);
        break;
    }
    tcg_temp_free(tmp_pos);
    tcg_temp_free(tmp_width);
}

/* RRRW format */
static void decode_rrrw_extract_insert(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    int r1, r2, r3, r4;
    int32_t width;

    TCGv temp, temp2;

    op2 = MASK_OP_RRRW_OP2(ctx->opcode);
    r1  = MASK_OP_RRRW_S1(ctx->opcode);
    r2  = MASK_OP_RRRW_S2(ctx->opcode);
    r3  = MASK_OP_RRRW_S3(ctx->opcode);
    r4  = MASK_OP_RRRW_D(ctx->opcode);
    width = MASK_OP_RRRW_WIDTH(ctx->opcode);

    temp = tcg_temp_new();

    switch (op2) {
    case OPC2_32_RRRW_EXTR:
        tcg_gen_andi_tl(temp, cpu_gpr_d[r3], 0x1f);
        tcg_gen_addi_tl(temp, temp, width);
        tcg_gen_subfi_tl(temp, 32, temp);
        tcg_gen_shl_tl(cpu_gpr_d[r4], cpu_gpr_d[r1], temp);
        tcg_gen_sari_tl(cpu_gpr_d[r4], cpu_gpr_d[r4], 32 - width);
        break;
    case OPC2_32_RRRW_EXTR_U:
        if (width == 0) {
            tcg_gen_movi_tl(cpu_gpr_d[r4], 0);
        } else {
            tcg_gen_andi_tl(temp, cpu_gpr_d[r3], 0x1f);
            tcg_gen_shr_tl(cpu_gpr_d[r4], cpu_gpr_d[r1], temp);
            tcg_gen_andi_tl(cpu_gpr_d[r4], cpu_gpr_d[r4], ~0u >> (32-width));
        }
        break;
    case OPC2_32_RRRW_IMASK:
        temp2 = tcg_temp_new();

        tcg_gen_andi_tl(temp, cpu_gpr_d[r3], 0x1f);
        tcg_gen_movi_tl(temp2, (1 << width) - 1);
        tcg_gen_shl_tl(temp2, temp2, temp);
        tcg_gen_shl_tl(cpu_gpr_d[r4], cpu_gpr_d[r2], temp);
        tcg_gen_mov_tl(cpu_gpr_d[r4+1], temp2);

        tcg_temp_free(temp2);
        break;
    case OPC2_32_RRRW_INSERT:
        temp2 = tcg_temp_new();

        tcg_gen_movi_tl(temp, width);
        tcg_gen_andi_tl(temp2, cpu_gpr_d[r3], 0x1f);
        gen_insert(cpu_gpr_d[r4], cpu_gpr_d[r1], cpu_gpr_d[r2], temp, temp2);

        tcg_temp_free(temp2);
        break;
    }
    tcg_temp_free(temp);
}

/* SYS Format*/
static void decode_sys_interrupts(CPUTriCoreState *env, DisasContext *ctx)
{
    uint32_t op2;
    uint32_t r1;
    TCGLabel *l1;
    TCGv tmp;

    op2 = MASK_OP_SYS_OP2(ctx->opcode);
    r1  = MASK_OP_SYS_S1D(ctx->opcode);

    switch (op2) {
    case OPC2_32_SYS_DEBUG:
        /* raise EXCP_DEBUG */
        break;
    case OPC2_32_SYS_DISABLE:
        tcg_gen_andi_tl(cpu_ICR, cpu_ICR, ~MASK_ICR_IE);
        break;
    case OPC2_32_SYS_DSYNC:
        break;
    case OPC2_32_SYS_ENABLE:
        tcg_gen_ori_tl(cpu_ICR, cpu_ICR, MASK_ICR_IE);
        break;
    case OPC2_32_SYS_ISYNC:
        break;
    case OPC2_32_SYS_NOP:
        break;
    case OPC2_32_SYS_RET:
        gen_compute_branch(ctx, op2, 0, 0, 0, 0);
        break;
    case OPC2_32_SYS_RFE:
        gen_helper_rfe(cpu_env);
        tcg_gen_exit_tb(0);
        ctx->bstate = BS_BRANCH;
        break;
    case OPC2_32_SYS_RFM:
        if ((ctx->hflags & TRICORE_HFLAG_KUU) == TRICORE_HFLAG_SM) {
            tmp = tcg_temp_new();
            l1 = gen_new_label();

            tcg_gen_ld32u_tl(tmp, cpu_env, offsetof(CPUTriCoreState, DBGSR));
            tcg_gen_andi_tl(tmp, tmp, MASK_DBGSR_DE);
            tcg_gen_brcondi_tl(TCG_COND_NE, tmp, 1, l1);
            gen_helper_rfm(cpu_env);
            gen_set_label(l1);
            tcg_gen_exit_tb(0);
            ctx->bstate = BS_BRANCH;
            tcg_temp_free(tmp);
        } else {
            /* generate privilege trap */
        }
        break;
    case OPC2_32_SYS_RSLCX:
        gen_helper_rslcx(cpu_env);
        break;
    case OPC2_32_SYS_SVLCX:
        gen_helper_svlcx(cpu_env);
        break;
    case OPC2_32_SYS_RESTORE:
        if (tricore_feature(env, TRICORE_FEATURE_16)) {
            if ((ctx->hflags & TRICORE_HFLAG_KUU) == TRICORE_HFLAG_SM ||
                (ctx->hflags & TRICORE_HFLAG_KUU) == TRICORE_HFLAG_UM1) {
                tcg_gen_deposit_tl(cpu_ICR, cpu_ICR, cpu_gpr_d[r1], 8, 1);
            } /* else raise privilege trap */
        } /* else raise illegal opcode trap */
        break;
    case OPC2_32_SYS_TRAPSV:
        /* TODO: raise sticky overflow trap */
        break;
    case OPC2_32_SYS_TRAPV:
        /* TODO: raise overflow trap */
        break;
    }
}

static void decode_32Bit_opc(CPUTriCoreState *env, DisasContext *ctx)
{
    int op1;
    int32_t r1, r2, r3;
    int32_t address, const16;
    int8_t b, const4;
    int32_t bpos;
    TCGv temp, temp2, temp3;

    op1 = MASK_OP_MAJOR(ctx->opcode);

    /* handle JNZ.T opcode only being 7 bit long */
    if (unlikely((op1 & 0x7f) == OPCM_32_BRN_JTT)) {
        op1 = OPCM_32_BRN_JTT;
    }

    switch (op1) {
/* ABS-format */
    case OPCM_32_ABS_LDW:
        decode_abs_ldw(env, ctx);
        break;
    case OPCM_32_ABS_LDB:
        decode_abs_ldb(env, ctx);
        break;
    case OPCM_32_ABS_LDMST_SWAP:
        decode_abs_ldst_swap(env, ctx);
        break;
    case OPCM_32_ABS_LDST_CONTEXT:
        decode_abs_ldst_context(env, ctx);
        break;
    case OPCM_32_ABS_STORE:
        decode_abs_store(env, ctx);
        break;
    case OPCM_32_ABS_STOREB_H:
        decode_abs_storeb_h(env, ctx);
        break;
    case OPC1_32_ABS_STOREQ:
        address = MASK_OP_ABS_OFF18(ctx->opcode);
        r1 = MASK_OP_ABS_S1D(ctx->opcode);
        temp = tcg_const_i32(EA_ABS_FORMAT(address));
        temp2 = tcg_temp_new();

        tcg_gen_shri_tl(temp2, cpu_gpr_d[r1], 16);
        tcg_gen_qemu_st_tl(temp2, temp, ctx->mem_idx, MO_LEUW);

        tcg_temp_free(temp2);
        tcg_temp_free(temp);
        break;
    case OPC1_32_ABS_LD_Q:
        address = MASK_OP_ABS_OFF18(ctx->opcode);
        r1 = MASK_OP_ABS_S1D(ctx->opcode);
        temp = tcg_const_i32(EA_ABS_FORMAT(address));

        tcg_gen_qemu_ld_tl(cpu_gpr_d[r1], temp, ctx->mem_idx, MO_LEUW);
        tcg_gen_shli_tl(cpu_gpr_d[r1], cpu_gpr_d[r1], 16);

        tcg_temp_free(temp);
        break;
    case OPC1_32_ABS_LEA:
        address = MASK_OP_ABS_OFF18(ctx->opcode);
        r1 = MASK_OP_ABS_S1D(ctx->opcode);
        tcg_gen_movi_tl(cpu_gpr_a[r1], EA_ABS_FORMAT(address));
        break;
/* ABSB-format */
    case OPC1_32_ABSB_ST_T:
        address = MASK_OP_ABS_OFF18(ctx->opcode);
        b = MASK_OP_ABSB_B(ctx->opcode);
        bpos = MASK_OP_ABSB_BPOS(ctx->opcode);

        temp = tcg_const_i32(EA_ABS_FORMAT(address));
        temp2 = tcg_temp_new();

        tcg_gen_qemu_ld_tl(temp2, temp, ctx->mem_idx, MO_UB);
        tcg_gen_andi_tl(temp2, temp2, ~(0x1u << bpos));
        tcg_gen_ori_tl(temp2, temp2, (b << bpos));
        tcg_gen_qemu_st_tl(temp2, temp, ctx->mem_idx, MO_UB);

        tcg_temp_free(temp);
        tcg_temp_free(temp2);
        break;
/* B-format */
    case OPC1_32_B_CALL:
    case OPC1_32_B_CALLA:
    case OPC1_32_B_J:
    case OPC1_32_B_JA:
    case OPC1_32_B_JL:
    case OPC1_32_B_JLA:
        address = MASK_OP_B_DISP24_SEXT(ctx->opcode);
        gen_compute_branch(ctx, op1, 0, 0, 0, address);
        break;
/* Bit-format */
    case OPCM_32_BIT_ANDACC:
        decode_bit_andacc(env, ctx);
        break;
    case OPCM_32_BIT_LOGICAL_T1:
        decode_bit_logical_t(env, ctx);
        break;
    case OPCM_32_BIT_INSERT:
        decode_bit_insert(env, ctx);
        break;
    case OPCM_32_BIT_LOGICAL_T2:
        decode_bit_logical_t2(env, ctx);
        break;
    case OPCM_32_BIT_ORAND:
        decode_bit_orand(env, ctx);
        break;
    case OPCM_32_BIT_SH_LOGIC1:
        decode_bit_sh_logic1(env, ctx);
        break;
    case OPCM_32_BIT_SH_LOGIC2:
        decode_bit_sh_logic2(env, ctx);
        break;
    /* BO Format */
    case OPCM_32_BO_ADDRMODE_POST_PRE_BASE:
        decode_bo_addrmode_post_pre_base(env, ctx);
        break;
    case OPCM_32_BO_ADDRMODE_BITREVERSE_CIRCULAR:
        decode_bo_addrmode_bitreverse_circular(env, ctx);
        break;
    case OPCM_32_BO_ADDRMODE_LD_POST_PRE_BASE:
        decode_bo_addrmode_ld_post_pre_base(env, ctx);
        break;
    case OPCM_32_BO_ADDRMODE_LD_BITREVERSE_CIRCULAR:
        decode_bo_addrmode_ld_bitreverse_circular(env, ctx);
        break;
    case OPCM_32_BO_ADDRMODE_STCTX_POST_PRE_BASE:
        decode_bo_addrmode_stctx_post_pre_base(env, ctx);
        break;
    case OPCM_32_BO_ADDRMODE_LDMST_BITREVERSE_CIRCULAR:
        decode_bo_addrmode_ldmst_bitreverse_circular(env, ctx);
        break;
/* BOL-format */
    case OPC1_32_BOL_LD_A_LONGOFF:
    case OPC1_32_BOL_LD_W_LONGOFF:
    case OPC1_32_BOL_LEA_LONGOFF:
    case OPC1_32_BOL_ST_W_LONGOFF:
    case OPC1_32_BOL_ST_A_LONGOFF:
    case OPC1_32_BOL_LD_B_LONGOFF:
    case OPC1_32_BOL_LD_BU_LONGOFF:
    case OPC1_32_BOL_LD_H_LONGOFF:
    case OPC1_32_BOL_LD_HU_LONGOFF:
    case OPC1_32_BOL_ST_B_LONGOFF:
    case OPC1_32_BOL_ST_H_LONGOFF:
        decode_bol_opc(env, ctx, op1);
        break;
/* BRC Format */
    case OPCM_32_BRC_EQ_NEQ:
    case OPCM_32_BRC_GE:
    case OPCM_32_BRC_JLT:
    case OPCM_32_BRC_JNE:
        const4 = MASK_OP_BRC_CONST4_SEXT(ctx->opcode);
        address = MASK_OP_BRC_DISP15_SEXT(ctx->opcode);
        r1 = MASK_OP_BRC_S1(ctx->opcode);
        gen_compute_branch(ctx, op1, r1, 0, const4, address);
        break;
/* BRN Format */
    case OPCM_32_BRN_JTT:
        address = MASK_OP_BRN_DISP15_SEXT(ctx->opcode);
        r1 = MASK_OP_BRN_S1(ctx->opcode);
        gen_compute_branch(ctx, op1, r1, 0, 0, address);
        break;
/* BRR Format */
    case OPCM_32_BRR_EQ_NEQ:
    case OPCM_32_BRR_ADDR_EQ_NEQ:
    case OPCM_32_BRR_GE:
    case OPCM_32_BRR_JLT:
    case OPCM_32_BRR_JNE:
    case OPCM_32_BRR_JNZ:
    case OPCM_32_BRR_LOOP:
        address = MASK_OP_BRR_DISP15_SEXT(ctx->opcode);
        r2 = MASK_OP_BRR_S2(ctx->opcode);
        r1 = MASK_OP_BRR_S1(ctx->opcode);
        gen_compute_branch(ctx, op1, r1, r2, 0, address);
        break;
/* RC Format */
    case OPCM_32_RC_LOGICAL_SHIFT:
        decode_rc_logical_shift(env, ctx);
        break;
    case OPCM_32_RC_ACCUMULATOR:
        decode_rc_accumulator(env, ctx);
        break;
    case OPCM_32_RC_SERVICEROUTINE:
        decode_rc_serviceroutine(env, ctx);
        break;
    case OPCM_32_RC_MUL:
        decode_rc_mul(env, ctx);
        break;
/* RCPW Format */
    case OPCM_32_RCPW_MASK_INSERT:
        decode_rcpw_insert(env, ctx);
        break;
/* RCRR Format */
    case OPC1_32_RCRR_INSERT:
        r1 = MASK_OP_RCRR_S1(ctx->opcode);
        r2 = MASK_OP_RCRR_S3(ctx->opcode);
        r3 = MASK_OP_RCRR_D(ctx->opcode);
        const16 = MASK_OP_RCRR_CONST4(ctx->opcode);
        temp = tcg_const_i32(const16);
        temp2 = tcg_temp_new(); /* width*/
        temp3 = tcg_temp_new(); /* pos */

        tcg_gen_andi_tl(temp2, cpu_gpr_d[r3+1], 0x1f);
        tcg_gen_andi_tl(temp3, cpu_gpr_d[r3], 0x1f);

        gen_insert(cpu_gpr_d[r2], cpu_gpr_d[r1], temp, temp2, temp3);

        tcg_temp_free(temp);
        tcg_temp_free(temp2);
        tcg_temp_free(temp3);
        break;
/* RCRW Format */
    case OPCM_32_RCRW_MASK_INSERT:
        decode_rcrw_insert(env, ctx);
        break;
/* RCR Format */
    case OPCM_32_RCR_COND_SELECT:
        decode_rcr_cond_select(env, ctx);
        break;
    case OPCM_32_RCR_MADD:
        decode_rcr_madd(env, ctx);
        break;
    case OPCM_32_RCR_MSUB:
        decode_rcr_msub(env, ctx);
        break;
/* RLC Format */
    case OPC1_32_RLC_ADDI:
    case OPC1_32_RLC_ADDIH:
    case OPC1_32_RLC_ADDIH_A:
    case OPC1_32_RLC_MFCR:
    case OPC1_32_RLC_MOV:
    case OPC1_32_RLC_MOV_64:
    case OPC1_32_RLC_MOV_U:
    case OPC1_32_RLC_MOV_H:
    case OPC1_32_RLC_MOVH_A:
    case OPC1_32_RLC_MTCR:
        decode_rlc_opc(env, ctx, op1);
        break;
/* RR Format */
    case OPCM_32_RR_ACCUMULATOR:
        decode_rr_accumulator(env, ctx);
        break;
    case OPCM_32_RR_LOGICAL_SHIFT:
        decode_rr_logical_shift(env, ctx);
        break;
    case OPCM_32_RR_ADDRESS:
        decode_rr_address(env, ctx);
        break;
    case OPCM_32_RR_IDIRECT:
        decode_rr_idirect(env, ctx);
        break;
    case OPCM_32_RR_DIVIDE:
        decode_rr_divide(env, ctx);
        break;
/* RR1 Format */
    case OPCM_32_RR1_MUL:
        decode_rr1_mul(env, ctx);
        break;
    case OPCM_32_RR1_MULQ:
        decode_rr1_mulq(env, ctx);
        break;
/* RR2 format */
    case OPCM_32_RR2_MUL:
        decode_rr2_mul(env, ctx);
        break;
/* RRPW format */
    case OPCM_32_RRPW_EXTRACT_INSERT:
        decode_rrpw_extract_insert(env, ctx);
        break;
    case OPC1_32_RRPW_DEXTR:
        r1 = MASK_OP_RRPW_S1(ctx->opcode);
        r2 = MASK_OP_RRPW_S2(ctx->opcode);
        r3 = MASK_OP_RRPW_D(ctx->opcode);
        const16 = MASK_OP_RRPW_POS(ctx->opcode);
        if (r1 == r2) {
            tcg_gen_rotli_tl(cpu_gpr_d[r3], cpu_gpr_d[r1], const16);
        } else {
            temp = tcg_temp_new();
            tcg_gen_shli_tl(temp, cpu_gpr_d[r1], const16);
            tcg_gen_shri_tl(cpu_gpr_d[r3], cpu_gpr_d[r2], 32 - const16);
            tcg_gen_or_tl(cpu_gpr_d[r3], cpu_gpr_d[r3], temp);
            tcg_temp_free(temp);
        }
        break;
/* RRR Format */
    case OPCM_32_RRR_COND_SELECT:
        decode_rrr_cond_select(env, ctx);
        break;
    case OPCM_32_RRR_DIVIDE:
        decode_rrr_divide(env, ctx);
/* RRR2 Format */
    case OPCM_32_RRR2_MADD:
        decode_rrr2_madd(env, ctx);
        break;
    case OPCM_32_RRR2_MSUB:
        decode_rrr2_msub(env, ctx);
        break;
/* RRR1 format */
    case OPCM_32_RRR1_MADD:
        decode_rrr1_madd(env, ctx);
        break;
    case OPCM_32_RRR1_MADDQ_H:
        decode_rrr1_maddq_h(env, ctx);
        break;
    case OPCM_32_RRR1_MADDSU_H:
        decode_rrr1_maddsu_h(env, ctx);
        break;
    case OPCM_32_RRR1_MSUB_H:
        decode_rrr1_msub(env, ctx);
        break;
    case OPCM_32_RRR1_MSUB_Q:
        decode_rrr1_msubq_h(env, ctx);
        break;
    case OPCM_32_RRR1_MSUBAD_H:
        decode_rrr1_msubad_h(env, ctx);
        break;
/* RRRR format */
    case OPCM_32_RRRR_EXTRACT_INSERT:
        decode_rrrr_extract_insert(env, ctx);
/* RRRW format */
    case OPCM_32_RRRW_EXTRACT_INSERT:
        decode_rrrw_extract_insert(env, ctx);
        break;
/* SYS format */
    case OPCM_32_SYS_INTERRUPTS:
        decode_sys_interrupts(env, ctx);
        break;
    case OPC1_32_SYS_RSTV:
        tcg_gen_movi_tl(cpu_PSW_V, 0);
        tcg_gen_mov_tl(cpu_PSW_SV, cpu_PSW_V);
        tcg_gen_mov_tl(cpu_PSW_AV, cpu_PSW_V);
        tcg_gen_mov_tl(cpu_PSW_SAV, cpu_PSW_V);
        break;
    }
}

static void decode_opc(CPUTriCoreState *env, DisasContext *ctx, int *is_branch)
{
    /* 16-Bit Instruction */
    if ((ctx->opcode & 0x1) == 0) {
        ctx->next_pc = ctx->pc + 2;
        decode_16Bit_opc(env, ctx);
    /* 32-Bit Instruction */
    } else {
        ctx->next_pc = ctx->pc + 4;
        decode_32Bit_opc(env, ctx);
    }
}

static inline void
gen_intermediate_code_internal(TriCoreCPU *cpu, struct TranslationBlock *tb,
                              int search_pc)
{
    CPUState *cs = CPU(cpu);
    CPUTriCoreState *env = &cpu->env;
    DisasContext ctx;
    target_ulong pc_start;
    int num_insns;

    if (search_pc) {
        qemu_log("search pc %d\n", search_pc);
    }

    num_insns = 0;
    pc_start = tb->pc;
    ctx.pc = pc_start;
    ctx.saved_pc = -1;
    ctx.tb = tb;
    ctx.singlestep_enabled = cs->singlestep_enabled;
    ctx.bstate = BS_NONE;
    ctx.mem_idx = cpu_mmu_index(env);

    tcg_clear_temp_count();
    gen_tb_start(tb);
    while (ctx.bstate == BS_NONE) {
        ctx.opcode = cpu_ldl_code(env, ctx.pc);
        decode_opc(env, &ctx, 0);

        num_insns++;

        if (tcg_op_buf_full()) {
            gen_save_pc(ctx.next_pc);
            tcg_gen_exit_tb(0);
            break;
        }
        if (singlestep) {
            gen_save_pc(ctx.next_pc);
            tcg_gen_exit_tb(0);
            break;
        }
        ctx.pc = ctx.next_pc;
    }

    gen_tb_end(tb, num_insns);
    if (search_pc) {
        printf("done_generating search pc\n");
    } else {
        tb->size = ctx.pc - pc_start;
        tb->icount = num_insns;
    }
    if (tcg_check_temp_count()) {
        printf("LEAK at %08x\n", env->PC);
    }

#ifdef DEBUG_DISAS
    if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)) {
        qemu_log("IN: %s\n", lookup_symbol(pc_start));
        log_target_disas(env, pc_start, ctx.pc - pc_start, 0);
        qemu_log("\n");
    }
#endif
}

void
gen_intermediate_code(CPUTriCoreState *env, struct TranslationBlock *tb)
{
    gen_intermediate_code_internal(tricore_env_get_cpu(env), tb, false);
}

void
gen_intermediate_code_pc(CPUTriCoreState *env, struct TranslationBlock *tb)
{
    gen_intermediate_code_internal(tricore_env_get_cpu(env), tb, true);
}

void
restore_state_to_opc(CPUTriCoreState *env, TranslationBlock *tb, int pc_pos)
{
    env->PC = tcg_ctx.gen_opc_pc[pc_pos];
}
/*
 *
 * Initialization
 *
 */

void cpu_state_reset(CPUTriCoreState *env)
{
    /* Reset Regs to Default Value */
    env->PSW = 0xb80;
}

static void tricore_tcg_init_csfr(void)
{
    cpu_PCXI = tcg_global_mem_new(TCG_AREG0,
                          offsetof(CPUTriCoreState, PCXI), "PCXI");
    cpu_PSW = tcg_global_mem_new(TCG_AREG0,
                          offsetof(CPUTriCoreState, PSW), "PSW");
    cpu_PC = tcg_global_mem_new(TCG_AREG0,
                          offsetof(CPUTriCoreState, PC), "PC");
    cpu_ICR = tcg_global_mem_new(TCG_AREG0,
                          offsetof(CPUTriCoreState, ICR), "ICR");
}

void tricore_tcg_init(void)
{
    int i;
    static int inited;
    if (inited) {
        return;
    }
    cpu_env = tcg_global_reg_new_ptr(TCG_AREG0, "env");
    /* reg init */
    for (i = 0 ; i < 16 ; i++) {
        cpu_gpr_a[i] = tcg_global_mem_new(TCG_AREG0,
                                          offsetof(CPUTriCoreState, gpr_a[i]),
                                          regnames_a[i]);
    }
    for (i = 0 ; i < 16 ; i++) {
        cpu_gpr_d[i] = tcg_global_mem_new(TCG_AREG0,
                                  offsetof(CPUTriCoreState, gpr_d[i]),
                                           regnames_d[i]);
    }
    tricore_tcg_init_csfr();
    /* init PSW flag cache */
    cpu_PSW_C = tcg_global_mem_new(TCG_AREG0,
                                   offsetof(CPUTriCoreState, PSW_USB_C),
                                   "PSW_C");
    cpu_PSW_V = tcg_global_mem_new(TCG_AREG0,
                                   offsetof(CPUTriCoreState, PSW_USB_V),
                                   "PSW_V");
    cpu_PSW_SV = tcg_global_mem_new(TCG_AREG0,
                                    offsetof(CPUTriCoreState, PSW_USB_SV),
                                    "PSW_SV");
    cpu_PSW_AV = tcg_global_mem_new(TCG_AREG0,
                                    offsetof(CPUTriCoreState, PSW_USB_AV),
                                    "PSW_AV");
    cpu_PSW_SAV = tcg_global_mem_new(TCG_AREG0,
                                     offsetof(CPUTriCoreState, PSW_USB_SAV),
                                     "PSW_SAV");
}
