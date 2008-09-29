/*
 *  Alpha emulation cpu translation for qemu.
 *
 *  Copyright (c) 2007 Jocelyn Mayer
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
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "cpu.h"
#include "exec-all.h"
#include "disas.h"
#include "host-utils.h"
#include "helper.h"
#include "tcg-op.h"
#include "qemu-common.h"

#define DO_SINGLE_STEP
#define ALPHA_DEBUG_DISAS
#define DO_TB_FLUSH

typedef struct DisasContext DisasContext;
struct DisasContext {
    uint64_t pc;
    int mem_idx;
#if !defined (CONFIG_USER_ONLY)
    int pal_mode;
#endif
    uint32_t amask;
};

/* global register indexes */
static TCGv cpu_env;
static TCGv cpu_ir[31];
static TCGv cpu_fir[31];
static TCGv cpu_pc;

/* dyngen register indexes */
static TCGv cpu_T[2];

/* register names */
static char cpu_reg_names[10*4+21*5 + 10*5+21*6];

#include "gen-icount.h"

static void alpha_translate_init(void)
{
    int i;
    char *p;
    static int done_init = 0;

    if (done_init)
        return;

    cpu_env = tcg_global_reg_new(TCG_TYPE_PTR, TCG_AREG0, "env");

#if TARGET_LONG_BITS > HOST_LONG_BITS
    cpu_T[0] = tcg_global_mem_new(TCG_TYPE_I64, TCG_AREG0,
                                  offsetof(CPUState, t0), "T0");
    cpu_T[1] = tcg_global_mem_new(TCG_TYPE_I64, TCG_AREG0,
                                  offsetof(CPUState, t1), "T1");
#else
    cpu_T[0] = tcg_global_reg_new(TCG_TYPE_I64, TCG_AREG1, "T0");
    cpu_T[1] = tcg_global_reg_new(TCG_TYPE_I64, TCG_AREG2, "T1");
#endif

    p = cpu_reg_names;
    for (i = 0; i < 31; i++) {
        sprintf(p, "ir%d", i);
        cpu_ir[i] = tcg_global_mem_new(TCG_TYPE_I64, TCG_AREG0,
                                       offsetof(CPUState, ir[i]), p);
        p += (i < 10) ? 4 : 5;

        sprintf(p, "fir%d", i);
        cpu_fir[i] = tcg_global_mem_new(TCG_TYPE_I64, TCG_AREG0,
                                        offsetof(CPUState, fir[i]), p);
        p += (i < 10) ? 5 : 6;
    }

    cpu_pc = tcg_global_mem_new(TCG_TYPE_I64, TCG_AREG0,
                                offsetof(CPUState, pc), "pc");

    /* register helpers */
#undef DEF_HELPER
#define DEF_HELPER(ret, name, params) tcg_register_helper(name, #name);
#include "helper.h"

    done_init = 1;
}

/* Memory moves */
#if defined(CONFIG_USER_ONLY)
#define OP_LD_TABLE(width)                                                    \
static GenOpFunc *gen_op_ld##width[] = {                                      \
    &gen_op_ld##width##_raw,                                                  \
}
#define OP_ST_TABLE(width)                                                    \
static GenOpFunc *gen_op_st##width[] = {                                      \
    &gen_op_st##width##_raw,                                                  \
}
#else
#define OP_LD_TABLE(width)                                                    \
static GenOpFunc *gen_op_ld##width[] = {                                      \
    &gen_op_ld##width##_kernel,                                               \
    &gen_op_ld##width##_executive,                                            \
    &gen_op_ld##width##_supervisor,                                           \
    &gen_op_ld##width##_user,                                                 \
}
#define OP_ST_TABLE(width)                                                    \
static GenOpFunc *gen_op_st##width[] = {                                      \
    &gen_op_st##width##_kernel,                                               \
    &gen_op_st##width##_executive,                                            \
    &gen_op_st##width##_supervisor,                                           \
    &gen_op_st##width##_user,                                                 \
}
#endif

#define GEN_LD(width)                                                         \
OP_LD_TABLE(width);                                                           \
static always_inline void gen_ld##width (DisasContext *ctx)                   \
{                                                                             \
    (*gen_op_ld##width[ctx->mem_idx])();                                      \
}

#define GEN_ST(width)                                                         \
OP_ST_TABLE(width);                                                           \
static always_inline void gen_st##width (DisasContext *ctx)                   \
{                                                                             \
    (*gen_op_st##width[ctx->mem_idx])();                                      \
}

GEN_LD(l);
GEN_ST(l);
GEN_LD(q);
GEN_ST(q);
GEN_LD(l_l);
GEN_ST(l_c);
GEN_LD(q_l);
GEN_ST(q_c);

static always_inline void gen_excp (DisasContext *ctx,
                                    int exception, int error_code)
{
    TCGv tmp1, tmp2;

    tcg_gen_movi_i64(cpu_pc, ctx->pc);
    tmp1 = tcg_const_i32(exception);
    tmp2 = tcg_const_i32(error_code);
    tcg_gen_helper_0_2(helper_excp, tmp1, tmp2);
    tcg_temp_free(tmp2);
    tcg_temp_free(tmp1);
}

static always_inline void gen_invalid (DisasContext *ctx)
{
    gen_excp(ctx, EXCP_OPCDEC, 0);
}

static always_inline void gen_load_mem_dyngen (DisasContext *ctx,
                                        void (*gen_load_op)(DisasContext *ctx),
                                        int ra, int rb, int32_t disp16,
                                        int clear)
{
    if (ra != 31 || disp16 != 0) {
        if (rb != 31)
            tcg_gen_addi_i64(cpu_T[0], cpu_ir[rb], disp16);
        else
            tcg_gen_movi_i64(cpu_T[0], disp16);
        if (clear)
            tcg_gen_andi_i64(cpu_T[0], cpu_T[0], ~0x7);
        (*gen_load_op)(ctx);
        if (ra != 31)
            tcg_gen_mov_i64(cpu_ir[ra], cpu_T[1]);
    }
}

static always_inline void gen_qemu_ldf (TCGv t0, TCGv t1, int flags)
{
    TCGv tmp = tcg_temp_new(TCG_TYPE_I32);
    tcg_gen_qemu_ld32u(tmp, t1, flags);
    tcg_gen_helper_1_1(helper_memory_to_f, t0, tmp);
    tcg_temp_free(tmp);
}

static always_inline void gen_qemu_ldg (TCGv t0, TCGv t1, int flags)
{
    TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
    tcg_gen_qemu_ld64(tmp, t1, flags);
    tcg_gen_helper_1_1(helper_memory_to_g, t0, tmp);
    tcg_temp_free(tmp);
}

static always_inline void gen_qemu_lds (TCGv t0, TCGv t1, int flags)
{
    TCGv tmp = tcg_temp_new(TCG_TYPE_I32);
    tcg_gen_qemu_ld32u(tmp, t1, flags);
    tcg_gen_helper_1_1(helper_memory_to_s, t0, tmp);
    tcg_temp_free(tmp);
}

static always_inline void gen_load_mem (DisasContext *ctx,
                                        void (*tcg_gen_qemu_load)(TCGv t0, TCGv t1, int flags),
                                        int ra, int rb, int32_t disp16,
                                        int fp, int clear)
{
    TCGv addr;

    if (unlikely(ra == 31))
        return;

    addr = tcg_temp_new(TCG_TYPE_I64);
    if (rb != 31) {
        tcg_gen_addi_i64(addr, cpu_ir[rb], disp16);
        if (clear)
            tcg_gen_andi_i64(addr, addr, ~0x7);
    } else {
        if (clear)
            disp16 &= ~0x7;
        tcg_gen_movi_i64(addr, disp16);
    }
    if (fp)
        tcg_gen_qemu_load(cpu_fir[ra], addr, ctx->mem_idx);
    else
        tcg_gen_qemu_load(cpu_ir[ra], addr, ctx->mem_idx);
    tcg_temp_free(addr);
}

static always_inline void gen_store_mem_dyngen (DisasContext *ctx,
                                         void (*gen_store_op)(DisasContext *ctx),
                                         int ra, int rb, int32_t disp16,
                                         int clear)
{
    if (rb != 31)
        tcg_gen_addi_i64(cpu_T[0], cpu_ir[rb], disp16);
    else
        tcg_gen_movi_i64(cpu_T[0], disp16);
    if (clear)
        tcg_gen_andi_i64(cpu_T[0], cpu_T[0], ~0x7);
    if (ra != 31)
        tcg_gen_mov_i64(cpu_T[1], cpu_ir[ra]);
    else
        tcg_gen_movi_i64(cpu_T[1], 0);
    (*gen_store_op)(ctx);
}

static always_inline void gen_qemu_stf (TCGv t0, TCGv t1, int flags)
{
    TCGv tmp = tcg_temp_new(TCG_TYPE_I32);
    tcg_gen_helper_1_1(helper_f_to_memory, tmp, t0);
    tcg_gen_qemu_st32(tmp, t1, flags);
    tcg_temp_free(tmp);
}

static always_inline void gen_qemu_stg (TCGv t0, TCGv t1, int flags)
{
    TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
    tcg_gen_helper_1_1(helper_g_to_memory, tmp, t0);
    tcg_gen_qemu_st64(tmp, t1, flags);
    tcg_temp_free(tmp);
}

static always_inline void gen_qemu_sts (TCGv t0, TCGv t1, int flags)
{
    TCGv tmp = tcg_temp_new(TCG_TYPE_I32);
    tcg_gen_helper_1_1(helper_s_to_memory, tmp, t0);
    tcg_gen_qemu_st32(tmp, t1, flags);
    tcg_temp_free(tmp);
}

static always_inline void gen_store_mem (DisasContext *ctx,
                                         void (*tcg_gen_qemu_store)(TCGv t0, TCGv t1, int flags),
                                         int ra, int rb, int32_t disp16,
                                         int fp, int clear)
{
    TCGv addr = tcg_temp_new(TCG_TYPE_I64);
    if (rb != 31) {
        tcg_gen_addi_i64(addr, cpu_ir[rb], disp16);
        if (clear)
            tcg_gen_andi_i64(addr, addr, ~0x7);
    } else {
        if (clear)
            disp16 &= ~0x7;
        tcg_gen_movi_i64(addr, disp16);
    }
    if (ra != 31) {
        if (fp)
            tcg_gen_qemu_store(cpu_fir[ra], addr, ctx->mem_idx);
        else
            tcg_gen_qemu_store(cpu_ir[ra], addr, ctx->mem_idx);
    } else {
        TCGv zero = tcg_const_i64(0);
        tcg_gen_qemu_store(zero, addr, ctx->mem_idx);
        tcg_temp_free(zero);
    }
    tcg_temp_free(addr);
}

static always_inline void gen_bcond (DisasContext *ctx,
                                     TCGCond cond,
                                     int ra, int32_t disp16, int mask)
{
    int l1, l2;

    l1 = gen_new_label();
    l2 = gen_new_label();
    if (likely(ra != 31)) {
        if (mask) {
            TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
            tcg_gen_andi_i64(tmp, cpu_ir[ra], 1);
            tcg_gen_brcondi_i64(cond, tmp, 0, l1);
            tcg_temp_free(tmp);
        } else
            tcg_gen_brcondi_i64(cond, cpu_ir[ra], 0, l1);
    } else {
        /* Very uncommon case - Do not bother to optimize.  */
        TCGv tmp = tcg_const_i64(0);
        tcg_gen_brcondi_i64(cond, tmp, 0, l1);
        tcg_temp_free(tmp);
    }
    tcg_gen_movi_i64(cpu_pc, ctx->pc);
    tcg_gen_br(l2);
    gen_set_label(l1);
    tcg_gen_movi_i64(cpu_pc, ctx->pc + (int64_t)(disp16 << 2));
    gen_set_label(l2);
}

static always_inline void gen_fbcond (DisasContext *ctx,
                                      void* func,
                                      int ra, int32_t disp16)
{
    int l1, l2;
    TCGv tmp;

    l1 = gen_new_label();
    l2 = gen_new_label();
    if (ra != 31) {
        tmp = tcg_temp_new(TCG_TYPE_I64);
        tcg_gen_helper_1_1(func, tmp, cpu_fir[ra]);
    } else  {
        tmp = tcg_const_i64(0);
        tcg_gen_helper_1_1(func, tmp, tmp);
    }
    tcg_gen_brcondi_i64(TCG_COND_NE, tmp, 0, l1);
    tcg_gen_movi_i64(cpu_pc, ctx->pc);
    tcg_gen_br(l2);
    gen_set_label(l1);
    tcg_gen_movi_i64(cpu_pc, ctx->pc + (int64_t)(disp16 << 2));
    gen_set_label(l2);
}

static always_inline void gen_cmov (TCGCond inv_cond,
                                    int ra, int rb, int rc,
                                    int islit, uint8_t lit, int mask)
{
    int l1;

    if (unlikely(rc == 31))
        return;

    l1 = gen_new_label();

    if (ra != 31) {
        if (mask) {
            TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
            tcg_gen_andi_i64(tmp, cpu_ir[ra], 1);
            tcg_gen_brcondi_i64(inv_cond, tmp, 0, l1);
            tcg_temp_free(tmp);
        } else
            tcg_gen_brcondi_i64(inv_cond, cpu_ir[ra], 0, l1);
    } else {
        /* Very uncommon case - Do not bother to optimize.  */
        TCGv tmp = tcg_const_i64(0);
        tcg_gen_brcondi_i64(inv_cond, tmp, 0, l1);
        tcg_temp_free(tmp);
    }

    if (islit)
        tcg_gen_movi_i64(cpu_ir[rc], lit);
    else
        tcg_gen_mov_i64(cpu_ir[rc], cpu_ir[rb]);
    gen_set_label(l1);
}

static always_inline void gen_farith2 (void *helper,
                                       int rb, int rc)
{
    if (unlikely(rc == 31))
      return;

    if (rb != 31)
        tcg_gen_helper_1_1(helper, cpu_fir[rc], cpu_fir[rb]);
    else {
        TCGv tmp = tcg_const_i64(0);
        tcg_gen_helper_1_1(helper, cpu_fir[rc], tmp);
        tcg_temp_free(tmp);
    }
}

static always_inline void gen_farith3 (void *helper,
                                       int ra, int rb, int rc)
{
    if (unlikely(rc == 31))
        return;

    if (ra != 31) {
        if (rb != 31)
            tcg_gen_helper_1_2(helper, cpu_fir[rc], cpu_fir[ra], cpu_fir[rb]);
        else {
            TCGv tmp = tcg_const_i64(0);
            tcg_gen_helper_1_2(helper, cpu_fir[rc], cpu_fir[ra], tmp);
            tcg_temp_free(tmp);
        }
    } else {
        TCGv tmp = tcg_const_i64(0);
        if (rb != 31)
            tcg_gen_helper_1_2(helper, cpu_fir[rc], tmp, cpu_fir[rb]);
        else
            tcg_gen_helper_1_2(helper, cpu_fir[rc], tmp, tmp);
        tcg_temp_free(tmp);
    }
}

static always_inline void gen_fcmov (void *func,
                                     int ra, int rb, int rc)
{
    int l1;
    TCGv tmp;

    if (unlikely(rc == 31))
        return;

    l1 = gen_new_label();
    tmp = tcg_temp_new(TCG_TYPE_I64);
    if (ra != 31) {
        tmp = tcg_temp_new(TCG_TYPE_I64);
        tcg_gen_helper_1_1(func, tmp, cpu_fir[ra]);
    } else  {
        tmp = tcg_const_i64(0);
        tcg_gen_helper_1_1(func, tmp, tmp);
    }
    tcg_gen_brcondi_i64(TCG_COND_EQ, tmp, 0, l1);
    if (rb != 31)
        tcg_gen_mov_i64(cpu_fir[rc], cpu_fir[ra]);
    else
        tcg_gen_movi_i64(cpu_fir[rc], 0);
    gen_set_label(l1);
}

/* EXTWH, EXTWH, EXTLH, EXTQH */
static always_inline void gen_ext_h(void (*tcg_gen_ext_i64)(TCGv t0, TCGv t1),
                                    int ra, int rb, int rc,
                                    int islit, uint8_t lit)
{
    if (unlikely(rc == 31))
        return;

    if (ra != 31) {
        if (islit) {
            if (lit != 0)
                tcg_gen_shli_i64(cpu_ir[rc], cpu_ir[ra], 64 - ((lit & 7) * 8));
            else
                tcg_gen_mov_i64(cpu_ir[rc], cpu_ir[ra]);
        } else {
            TCGv tmp1, tmp2;
            tmp1 = tcg_temp_new(TCG_TYPE_I64);
            tcg_gen_andi_i64(tmp1, cpu_ir[rb], 7);
            tcg_gen_shli_i64(tmp1, tmp1, 3);
            tmp2 = tcg_const_i64(64);
            tcg_gen_sub_i64(tmp1, tmp2, tmp1);
            tcg_temp_free(tmp2);
            tcg_gen_shl_i64(cpu_ir[rc], cpu_ir[ra], tmp1);
            tcg_temp_free(tmp1);
        }
        if (tcg_gen_ext_i64)
            tcg_gen_ext_i64(cpu_ir[rc], cpu_ir[rc]);
    } else
        tcg_gen_movi_i64(cpu_ir[rc], 0);
}

/* EXTBL, EXTWL, EXTWL, EXTLL, EXTQL */
static always_inline void gen_ext_l(void (*tcg_gen_ext_i64)(TCGv t0, TCGv t1),
                                    int ra, int rb, int rc,
                                    int islit, uint8_t lit)
{
    if (unlikely(rc == 31))
        return;

    if (ra != 31) {
        if (islit) {
                tcg_gen_shri_i64(cpu_ir[rc], cpu_ir[ra], (lit & 7) * 8);
        } else {
            TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
            tcg_gen_andi_i64(tmp, cpu_ir[rb], 7);
            tcg_gen_shli_i64(tmp, tmp, 3);
            tcg_gen_shr_i64(cpu_ir[rc], cpu_ir[ra], tmp);
            tcg_temp_free(tmp);
        }
        if (tcg_gen_ext_i64)
            tcg_gen_ext_i64(cpu_ir[rc], cpu_ir[rc]);
    } else
        tcg_gen_movi_i64(cpu_ir[rc], 0);
}

/* Code to call arith3 helpers */
static always_inline void gen_arith3 (void *helper,
                                      int ra, int rb, int rc,
                                      int islit, uint8_t lit)
{
    if (unlikely(rc == 31))
        return;

    if (ra != 31) {
        if (islit) {
            TCGv tmp = tcg_const_i64(lit);
            tcg_gen_helper_1_2(helper, cpu_ir[rc], cpu_ir[ra], tmp);
            tcg_temp_free(tmp);
        } else
            tcg_gen_helper_1_2(helper, cpu_ir[rc], cpu_ir[ra], cpu_ir[rb]);
    } else {
        TCGv tmp1 = tcg_const_i64(0);
        if (islit) {
            TCGv tmp2 = tcg_const_i64(lit);
            tcg_gen_helper_1_2(helper, cpu_ir[rc], tmp1, tmp2);
            tcg_temp_free(tmp2);
        } else
            tcg_gen_helper_1_2(helper, cpu_ir[rc], tmp1, cpu_ir[rb]);
        tcg_temp_free(tmp1);
    }
}

static always_inline void gen_cmp(TCGCond cond,
                                  int ra, int rb, int rc,
                                  int islit, uint8_t lit)
{
    int l1, l2;
    TCGv tmp;

    if (unlikely(rc == 31))
    return;

    l1 = gen_new_label();
    l2 = gen_new_label();

    if (ra != 31) {
        tmp = tcg_temp_new(TCG_TYPE_I64);
        tcg_gen_mov_i64(tmp, cpu_ir[ra]);
    } else
        tmp = tcg_const_i64(0);
    if (islit)
        tcg_gen_brcondi_i64(cond, tmp, lit, l1);
    else
        tcg_gen_brcond_i64(cond, tmp, cpu_ir[rb], l1);

    tcg_gen_movi_i64(cpu_ir[rc], 0);
    tcg_gen_br(l2);
    gen_set_label(l1);
    tcg_gen_movi_i64(cpu_ir[rc], 1);
    gen_set_label(l2);
}

static always_inline int translate_one (DisasContext *ctx, uint32_t insn)
{
    uint32_t palcode;
    int32_t disp21, disp16, disp12;
    uint16_t fn11, fn16;
    uint8_t opc, ra, rb, rc, sbz, fpfn, fn7, fn2, islit;
    uint8_t lit;
    int ret;

    /* Decode all instruction fields */
    opc = insn >> 26;
    ra = (insn >> 21) & 0x1F;
    rb = (insn >> 16) & 0x1F;
    rc = insn & 0x1F;
    sbz = (insn >> 13) & 0x07;
    islit = (insn >> 12) & 1;
    if (rb == 31 && !islit) {
        islit = 1;
        lit = 0;
    } else
        lit = (insn >> 13) & 0xFF;
    palcode = insn & 0x03FFFFFF;
    disp21 = ((int32_t)((insn & 0x001FFFFF) << 11)) >> 11;
    disp16 = (int16_t)(insn & 0x0000FFFF);
    disp12 = (int32_t)((insn & 0x00000FFF) << 20) >> 20;
    fn16 = insn & 0x0000FFFF;
    fn11 = (insn >> 5) & 0x000007FF;
    fpfn = fn11 & 0x3F;
    fn7 = (insn >> 5) & 0x0000007F;
    fn2 = (insn >> 5) & 0x00000003;
    ret = 0;
#if defined ALPHA_DEBUG_DISAS
    if (logfile != NULL) {
        fprintf(logfile, "opc %02x ra %d rb %d rc %d disp16 %04x\n",
                opc, ra, rb, rc, disp16);
    }
#endif
    switch (opc) {
    case 0x00:
        /* CALL_PAL */
        if (palcode >= 0x80 && palcode < 0xC0) {
            /* Unprivileged PAL call */
            gen_excp(ctx, EXCP_CALL_PAL + ((palcode & 0x1F) << 6), 0);
#if !defined (CONFIG_USER_ONLY)
        } else if (palcode < 0x40) {
            /* Privileged PAL code */
            if (ctx->mem_idx & 1)
                goto invalid_opc;
            else
                gen_excp(ctx, EXCP_CALL_PALP + ((palcode & 0x1F) << 6), 0);
#endif
        } else {
            /* Invalid PAL call */
            goto invalid_opc;
        }
        ret = 3;
        break;
    case 0x01:
        /* OPC01 */
        goto invalid_opc;
    case 0x02:
        /* OPC02 */
        goto invalid_opc;
    case 0x03:
        /* OPC03 */
        goto invalid_opc;
    case 0x04:
        /* OPC04 */
        goto invalid_opc;
    case 0x05:
        /* OPC05 */
        goto invalid_opc;
    case 0x06:
        /* OPC06 */
        goto invalid_opc;
    case 0x07:
        /* OPC07 */
        goto invalid_opc;
    case 0x08:
        /* LDA */
        if (likely(ra != 31)) {
            if (rb != 31)
                tcg_gen_addi_i64(cpu_ir[ra], cpu_ir[rb], disp16);
            else
                tcg_gen_movi_i64(cpu_ir[ra], disp16);
        }
        break;
    case 0x09:
        /* LDAH */
        if (likely(ra != 31)) {
            if (rb != 31)
                tcg_gen_addi_i64(cpu_ir[ra], cpu_ir[rb], disp16 << 16);
            else
                tcg_gen_movi_i64(cpu_ir[ra], disp16 << 16);
        }
        break;
    case 0x0A:
        /* LDBU */
        if (!(ctx->amask & AMASK_BWX))
            goto invalid_opc;
        gen_load_mem(ctx, &tcg_gen_qemu_ld8u, ra, rb, disp16, 0, 0);
        break;
    case 0x0B:
        /* LDQ_U */
        gen_load_mem(ctx, &tcg_gen_qemu_ld64, ra, rb, disp16, 0, 1);
        break;
    case 0x0C:
        /* LDWU */
        if (!(ctx->amask & AMASK_BWX))
            goto invalid_opc;
        gen_load_mem(ctx, &tcg_gen_qemu_ld16u, ra, rb, disp16, 0, 1);
        break;
    case 0x0D:
        /* STW */
        gen_store_mem(ctx, &tcg_gen_qemu_st16, ra, rb, disp16, 0, 0);
        break;
    case 0x0E:
        /* STB */
        gen_store_mem(ctx, &tcg_gen_qemu_st8, ra, rb, disp16, 0, 0);
        break;
    case 0x0F:
        /* STQ_U */
        gen_store_mem(ctx, &tcg_gen_qemu_st64, ra, rb, disp16, 0, 1);
        break;
    case 0x10:
        switch (fn7) {
        case 0x00:
            /* ADDL */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    if (islit) {
                        tcg_gen_addi_i64(cpu_ir[rc], cpu_ir[ra], lit);
                        tcg_gen_ext32s_i64(cpu_ir[rc], cpu_ir[rc]);
                    } else {
                        tcg_gen_add_i64(cpu_ir[rc], cpu_ir[ra], cpu_ir[rb]);
                        tcg_gen_ext32s_i64(cpu_ir[rc], cpu_ir[rc]);
                    }
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], lit);
                    else
                        tcg_gen_ext32s_i64(cpu_ir[rc], cpu_ir[rb]);
                }
            }
            break;
        case 0x02:
            /* S4ADDL */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
                    tcg_gen_shli_i64(tmp, cpu_ir[ra], 2);
                    if (islit)
                        tcg_gen_addi_i64(tmp, tmp, lit);
                    else
                        tcg_gen_add_i64(tmp, tmp, cpu_ir[rb]);
                    tcg_gen_ext32s_i64(cpu_ir[rc], tmp);
                    tcg_temp_free(tmp);
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], lit);
                    else
                        tcg_gen_ext32s_i64(cpu_ir[rc], cpu_ir[rb]);
                }
            }
            break;
        case 0x09:
            /* SUBL */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    if (islit)
                        tcg_gen_subi_i64(cpu_ir[rc], cpu_ir[ra], lit);
                    else
                        tcg_gen_sub_i64(cpu_ir[rc], cpu_ir[ra], cpu_ir[rb]);
                    tcg_gen_ext32s_i64(cpu_ir[rc], cpu_ir[rc]);
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], -lit);
                    else {
                        tcg_gen_neg_i64(cpu_ir[rc], cpu_ir[rb]);
                        tcg_gen_ext32s_i64(cpu_ir[rc], cpu_ir[rc]);
                }
            }
            break;
        case 0x0B:
            /* S4SUBL */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
                    tcg_gen_shli_i64(tmp, cpu_ir[ra], 2);
                    if (islit)
                        tcg_gen_subi_i64(tmp, tmp, lit);
                    else
                        tcg_gen_sub_i64(tmp, tmp, cpu_ir[rb]);
                    tcg_gen_ext32s_i64(cpu_ir[rc], tmp);
                    tcg_temp_free(tmp);
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], -lit);
                    else {
                        tcg_gen_neg_i64(cpu_ir[rc], cpu_ir[rb]);
                        tcg_gen_ext32s_i64(cpu_ir[rc], cpu_ir[rc]);
                    }
                }
            }
            break;
        case 0x0F:
            /* CMPBGE */
            gen_arith3(helper_cmpbge, ra, rb, rc, islit, lit);
            break;
        case 0x12:
            /* S8ADDL */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
                    tcg_gen_shli_i64(tmp, cpu_ir[ra], 3);
                    if (islit)
                        tcg_gen_addi_i64(tmp, tmp, lit);
                    else
                        tcg_gen_add_i64(tmp, tmp, cpu_ir[rb]);
                    tcg_gen_ext32s_i64(cpu_ir[rc], tmp);
                    tcg_temp_free(tmp);
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], lit);
                    else
                        tcg_gen_ext32s_i64(cpu_ir[rc], cpu_ir[rb]);
                }
            }
            break;
        case 0x1B:
            /* S8SUBL */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
                    tcg_gen_shli_i64(tmp, cpu_ir[ra], 3);
                    if (islit)
                        tcg_gen_subi_i64(tmp, tmp, lit);
                    else
                       tcg_gen_sub_i64(tmp, tmp, cpu_ir[rb]);
                    tcg_gen_ext32s_i64(cpu_ir[rc], tmp);
                    tcg_temp_free(tmp);
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], -lit);
                    else
                        tcg_gen_neg_i64(cpu_ir[rc], cpu_ir[rb]);
                        tcg_gen_ext32s_i64(cpu_ir[rc], cpu_ir[rc]);
                    }
                }
            }
            break;
        case 0x1D:
            /* CMPULT */
            gen_cmp(TCG_COND_LTU, ra, rb, rc, islit, lit);
            break;
        case 0x20:
            /* ADDQ */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    if (islit)
                        tcg_gen_addi_i64(cpu_ir[rc], cpu_ir[ra], lit);
                    else
                        tcg_gen_add_i64(cpu_ir[rc], cpu_ir[ra], cpu_ir[rb]);
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], lit);
                    else
                        tcg_gen_mov_i64(cpu_ir[rc], cpu_ir[rb]);
                }
            }
            break;
        case 0x22:
            /* S4ADDQ */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
                    tcg_gen_shli_i64(tmp, cpu_ir[ra], 2);
                    if (islit)
                        tcg_gen_addi_i64(cpu_ir[rc], tmp, lit);
                    else
                        tcg_gen_add_i64(cpu_ir[rc], tmp, cpu_ir[rb]);
                    tcg_temp_free(tmp);
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], lit);
                    else
                        tcg_gen_mov_i64(cpu_ir[rc], cpu_ir[rb]);
                }
            }
            break;
        case 0x29:
            /* SUBQ */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    if (islit)
                        tcg_gen_subi_i64(cpu_ir[rc], cpu_ir[ra], lit);
                    else
                        tcg_gen_sub_i64(cpu_ir[rc], cpu_ir[ra], cpu_ir[rb]);
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], -lit);
                    else
                        tcg_gen_neg_i64(cpu_ir[rc], cpu_ir[rb]);
                }
            }
            break;
        case 0x2B:
            /* S4SUBQ */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
                    tcg_gen_shli_i64(tmp, cpu_ir[ra], 2);
                    if (islit)
                        tcg_gen_subi_i64(cpu_ir[rc], tmp, lit);
                    else
                        tcg_gen_sub_i64(cpu_ir[rc], tmp, cpu_ir[rb]);
                    tcg_temp_free(tmp);
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], -lit);
                    else
                        tcg_gen_neg_i64(cpu_ir[rc], cpu_ir[rb]);
                }
            }
            break;
        case 0x2D:
            /* CMPEQ */
            gen_cmp(TCG_COND_EQ, ra, rb, rc, islit, lit);
            break;
        case 0x32:
            /* S8ADDQ */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
                    tcg_gen_shli_i64(tmp, cpu_ir[ra], 3);
                    if (islit)
                        tcg_gen_addi_i64(cpu_ir[rc], tmp, lit);
                    else
                        tcg_gen_add_i64(cpu_ir[rc], tmp, cpu_ir[rb]);
                    tcg_temp_free(tmp);
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], lit);
                    else
                        tcg_gen_mov_i64(cpu_ir[rc], cpu_ir[rb]);
                }
            }
            break;
        case 0x3B:
            /* S8SUBQ */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
                    tcg_gen_shli_i64(tmp, cpu_ir[ra], 3);
                    if (islit)
                        tcg_gen_subi_i64(cpu_ir[rc], tmp, lit);
                    else
                        tcg_gen_sub_i64(cpu_ir[rc], tmp, cpu_ir[rb]);
                    tcg_temp_free(tmp);
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], -lit);
                    else
                        tcg_gen_neg_i64(cpu_ir[rc], cpu_ir[rb]);
                }
            }
            break;
        case 0x3D:
            /* CMPULE */
            gen_cmp(TCG_COND_LEU, ra, rb, rc, islit, lit);
            break;
        case 0x40:
            /* ADDL/V */
            gen_arith3(helper_addlv, ra, rb, rc, islit, lit);
            break;
        case 0x49:
            /* SUBL/V */
            gen_arith3(helper_sublv, ra, rb, rc, islit, lit);
            break;
        case 0x4D:
            /* CMPLT */
            gen_cmp(TCG_COND_LT, ra, rb, rc, islit, lit);
            break;
        case 0x60:
            /* ADDQ/V */
            gen_arith3(helper_addqv, ra, rb, rc, islit, lit);
            break;
        case 0x69:
            /* SUBQ/V */
            gen_arith3(helper_subqv, ra, rb, rc, islit, lit);
            break;
        case 0x6D:
            /* CMPLE */
            gen_cmp(TCG_COND_LE, ra, rb, rc, islit, lit);
            break;
        default:
            goto invalid_opc;
        }
        break;
    case 0x11:
        switch (fn7) {
        case 0x00:
            /* AND */
            if (likely(rc != 31)) {
                if (ra == 31)
                    tcg_gen_movi_i64(cpu_ir[rc], 0);
                else if (islit)
                    tcg_gen_andi_i64(cpu_ir[rc], cpu_ir[ra], lit);
                else
                    tcg_gen_and_i64(cpu_ir[rc], cpu_ir[ra], cpu_ir[rb]);
            }
            break;
        case 0x08:
            /* BIC */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    if (islit)
                        tcg_gen_andi_i64(cpu_ir[rc], cpu_ir[ra], ~lit);
                    else {
                        TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
                        tcg_gen_not_i64(tmp, cpu_ir[rb]);
                        tcg_gen_and_i64(cpu_ir[rc], cpu_ir[ra], tmp);
                        tcg_temp_free(tmp);
                    }
                } else
                    tcg_gen_movi_i64(cpu_ir[rc], 0);
            }
            break;
        case 0x14:
            /* CMOVLBS */
            gen_cmov(TCG_COND_EQ, ra, rb, rc, islit, lit, 1);
            break;
        case 0x16:
            /* CMOVLBC */
            gen_cmov(TCG_COND_NE, ra, rb, rc, islit, lit, 1);
            break;
        case 0x20:
            /* BIS */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    if (islit)
                        tcg_gen_ori_i64(cpu_ir[rc], cpu_ir[ra], lit);
        	    else
                        tcg_gen_or_i64(cpu_ir[rc], cpu_ir[ra], cpu_ir[rb]);
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], lit);
                    else
                        tcg_gen_mov_i64(cpu_ir[rc], cpu_ir[rb]);
                }
            }
            break;
        case 0x24:
            /* CMOVEQ */
            gen_cmov(TCG_COND_NE, ra, rb, rc, islit, lit, 0);
            break;
        case 0x26:
            /* CMOVNE */
            gen_cmov(TCG_COND_EQ, ra, rb, rc, islit, lit, 0);
            break;
        case 0x28:
            /* ORNOT */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    if (islit)
                        tcg_gen_ori_i64(cpu_ir[rc], cpu_ir[ra], ~lit);
                    else {
                        TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
                        tcg_gen_not_i64(tmp, cpu_ir[rb]);
                        tcg_gen_or_i64(cpu_ir[rc], cpu_ir[ra], tmp);
                        tcg_temp_free(tmp);
                    }
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], ~lit);
                    else
                        tcg_gen_not_i64(cpu_ir[rc], cpu_ir[rb]);
                }
            }
            break;
        case 0x40:
            /* XOR */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    if (islit)
                        tcg_gen_xori_i64(cpu_ir[rc], cpu_ir[ra], lit);
                    else
                        tcg_gen_xor_i64(cpu_ir[rc], cpu_ir[ra], cpu_ir[rb]);
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], lit);
                    else
                        tcg_gen_mov_i64(cpu_ir[rc], cpu_ir[rb]);
                }
            }
            break;
        case 0x44:
            /* CMOVLT */
            gen_cmov(TCG_COND_GE, ra, rb, rc, islit, lit, 0);
            break;
        case 0x46:
            /* CMOVGE */
            gen_cmov(TCG_COND_LT, ra, rb, rc, islit, lit, 0);
            break;
        case 0x48:
            /* EQV */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    if (islit)
                        tcg_gen_xori_i64(cpu_ir[rc], cpu_ir[ra], ~lit);
                    else {
                        TCGv tmp = tcg_temp_new(TCG_TYPE_I64);
                        tcg_gen_not_i64(tmp, cpu_ir[rb]);
                        tcg_gen_xor_i64(cpu_ir[rc], cpu_ir[ra], tmp);
                        tcg_temp_free(tmp);
                    }
                } else {
                    if (islit)
                        tcg_gen_movi_i64(cpu_ir[rc], ~lit);
                    else
                        tcg_gen_not_i64(cpu_ir[rc], cpu_ir[rb]);
                }
            }
            break;
        case 0x61:
            /* AMASK */
            if (likely(rc != 31)) {
                if (islit)
                    tcg_gen_movi_i64(cpu_ir[rc], helper_amask(lit));
                else
                    tcg_gen_helper_1_1(helper_amask, cpu_ir[rc], cpu_ir[rb]);
            }
            break;
        case 0x64:
            /* CMOVLE */
            gen_cmov(TCG_COND_GT, ra, rb, rc, islit, lit, 0);
            break;
        case 0x66:
            /* CMOVGT */
            gen_cmov(TCG_COND_LE, ra, rb, rc, islit, lit, 0);
            break;
        case 0x6C:
            /* IMPLVER */
            if (rc != 31)
                tcg_gen_helper_1_0(helper_load_implver, cpu_ir[rc]);
            break;
        default:
            goto invalid_opc;
        }
        break;
    case 0x12:
        switch (fn7) {
        case 0x02:
            /* MSKBL */
            gen_arith3(helper_mskbl, ra, rb, rc, islit, lit);
            break;
        case 0x06:
            /* EXTBL */
            gen_ext_l(&tcg_gen_ext8u_i64, ra, rb, rc, islit, lit);
            break;
        case 0x0B:
            /* INSBL */
            gen_arith3(helper_insbl, ra, rb, rc, islit, lit);
            break;
        case 0x12:
            /* MSKWL */
            gen_arith3(helper_mskwl, ra, rb, rc, islit, lit);
            break;
        case 0x16:
            /* EXTWL */
            gen_ext_l(&tcg_gen_ext16u_i64, ra, rb, rc, islit, lit);
            break;
        case 0x1B:
            /* INSWL */
            gen_arith3(helper_inswl, ra, rb, rc, islit, lit);
            break;
        case 0x22:
            /* MSKLL */
            gen_arith3(helper_mskll, ra, rb, rc, islit, lit);
            break;
        case 0x26:
            /* EXTLL */
            gen_ext_l(&tcg_gen_ext32u_i64, ra, rb, rc, islit, lit);
            break;
        case 0x2B:
            /* INSLL */
            gen_arith3(helper_insll, ra, rb, rc, islit, lit);
            break;
        case 0x30:
            /* ZAP */
            gen_arith3(helper_zap, ra, rb, rc, islit, lit);
            break;
        case 0x31:
            /* ZAPNOT */
            gen_arith3(helper_zapnot, ra, rb, rc, islit, lit);
            break;
        case 0x32:
            /* MSKQL */
            gen_arith3(helper_mskql, ra, rb, rc, islit, lit);
            break;
        case 0x34:
            /* SRL */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    if (islit)
                        tcg_gen_shri_i64(cpu_ir[rc], cpu_ir[ra], lit & 0x3f);
                    else {
                        TCGv shift = tcg_temp_new(TCG_TYPE_I64);
                        tcg_gen_andi_i64(shift, cpu_ir[rb], 0x3f);
                        tcg_gen_shr_i64(cpu_ir[rc], cpu_ir[ra], shift);
                        tcg_temp_free(shift);
                    }
                } else
                    tcg_gen_movi_i64(cpu_ir[rc], 0);
            }
            break;
        case 0x36:
            /* EXTQL */
            gen_ext_l(NULL, ra, rb, rc, islit, lit);
            break;
        case 0x39:
            /* SLL */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    if (islit)
                        tcg_gen_shli_i64(cpu_ir[rc], cpu_ir[ra], lit & 0x3f);
                    else {
                        TCGv shift = tcg_temp_new(TCG_TYPE_I64);
                        tcg_gen_andi_i64(shift, cpu_ir[rb], 0x3f);
                        tcg_gen_shl_i64(cpu_ir[rc], cpu_ir[ra], shift);
                        tcg_temp_free(shift);
                    }
                } else
                    tcg_gen_movi_i64(cpu_ir[rc], 0);
            }
            break;
        case 0x3B:
            /* INSQL */
            gen_arith3(helper_insql, ra, rb, rc, islit, lit);
            break;
        case 0x3C:
            /* SRA */
            if (likely(rc != 31)) {
                if (ra != 31) {
                    if (islit)
                        tcg_gen_sari_i64(cpu_ir[rc], cpu_ir[ra], lit & 0x3f);
                    else {
                        TCGv shift = tcg_temp_new(TCG_TYPE_I64);
                        tcg_gen_andi_i64(shift, cpu_ir[rb], 0x3f);
                        tcg_gen_sar_i64(cpu_ir[rc], cpu_ir[ra], shift);
                        tcg_temp_free(shift);
                    }
                } else
                    tcg_gen_movi_i64(cpu_ir[rc], 0);
            }
            break;
        case 0x52:
            /* MSKWH */
            gen_arith3(helper_mskwh, ra, rb, rc, islit, lit);
            break;
        case 0x57:
            /* INSWH */
            gen_arith3(helper_inswh, ra, rb, rc, islit, lit);
            break;
        case 0x5A:
            /* EXTWH */
            gen_ext_h(&tcg_gen_ext16u_i64, ra, rb, rc, islit, lit);
            break;
        case 0x62:
            /* MSKLH */
            gen_arith3(helper_msklh, ra, rb, rc, islit, lit);
            break;
        case 0x67:
            /* INSLH */
            gen_arith3(helper_inslh, ra, rb, rc, islit, lit);
            break;
        case 0x6A:
            /* EXTLH */
            gen_ext_h(&tcg_gen_ext16u_i64, ra, rb, rc, islit, lit);
            break;
        case 0x72:
            /* MSKQH */
            gen_arith3(helper_mskqh, ra, rb, rc, islit, lit);
            break;
        case 0x77:
            /* INSQH */
            gen_arith3(helper_insqh, ra, rb, rc, islit, lit);
            break;
        case 0x7A:
            /* EXTQH */
            gen_ext_h(NULL, ra, rb, rc, islit, lit);
            break;
        default:
            goto invalid_opc;
        }
        break;
    case 0x13:
        switch (fn7) {
        case 0x00:
            /* MULL */
            if (likely(rc != 31)) {
                if (ra == 31)
                    tcg_gen_movi_i64(cpu_ir[rc], 0);
                else {
                    if (islit)
                        tcg_gen_muli_i64(cpu_ir[rc], cpu_ir[ra], lit);
                    else
                        tcg_gen_mul_i64(cpu_ir[rc], cpu_ir[ra], cpu_ir[rb]);
                    tcg_gen_ext32s_i64(cpu_ir[rc], cpu_ir[rc]);
                }
            }
            break;
        case 0x20:
            /* MULQ */
            if (likely(rc != 31)) {
                if (ra == 31)
                    tcg_gen_movi_i64(cpu_ir[rc], 0);
                else if (islit)
                    tcg_gen_muli_i64(cpu_ir[rc], cpu_ir[ra], lit);
                else
                    tcg_gen_mul_i64(cpu_ir[rc], cpu_ir[ra], cpu_ir[rb]);
            }
            break;
        case 0x30:
            /* UMULH */
            gen_arith3(helper_umulh, ra, rb, rc, islit, lit);
            break;
        case 0x40:
            /* MULL/V */
            gen_arith3(helper_mullv, ra, rb, rc, islit, lit);
            break;
        case 0x60:
            /* MULQ/V */
            gen_arith3(helper_mulqv, ra, rb, rc, islit, lit);
            break;
        default:
            goto invalid_opc;
        }
        break;
    case 0x14:
        switch (fpfn) { /* f11 & 0x3F */
        case 0x04:
            /* ITOFS */
            if (!(ctx->amask & AMASK_FIX))
                goto invalid_opc;
            if (likely(rc != 31)) {
                if (ra != 31) {
                    TCGv tmp = tcg_temp_new(TCG_TYPE_I32);
                    tcg_gen_trunc_i64_i32(tmp, cpu_ir[ra]);
                    tcg_gen_helper_1_1(helper_memory_to_s, cpu_fir[rc], tmp);
                    tcg_temp_free(tmp);
                } else
                    tcg_gen_movi_i64(cpu_fir[rc], 0);
            }
            break;
        case 0x0A:
            /* SQRTF */
            if (!(ctx->amask & AMASK_FIX))
                goto invalid_opc;
            gen_farith2(&helper_sqrtf, rb, rc);
            break;
        case 0x0B:
            /* SQRTS */
            if (!(ctx->amask & AMASK_FIX))
                goto invalid_opc;
            gen_farith2(&helper_sqrts, rb, rc);
            break;
        case 0x14:
            /* ITOFF */
            if (!(ctx->amask & AMASK_FIX))
                goto invalid_opc;
            if (likely(rc != 31)) {
                if (ra != 31) {
                    TCGv tmp = tcg_temp_new(TCG_TYPE_I32);
                    tcg_gen_trunc_i64_i32(tmp, cpu_ir[ra]);
                    tcg_gen_helper_1_1(helper_memory_to_f, cpu_fir[rc], tmp);
                    tcg_temp_free(tmp);
                } else
                    tcg_gen_movi_i64(cpu_fir[rc], 0);
            }
            break;
        case 0x24:
            /* ITOFT */
            if (!(ctx->amask & AMASK_FIX))
                goto invalid_opc;
            if (likely(rc != 31)) {
                if (ra != 31)
                    tcg_gen_mov_i64(cpu_fir[rc], cpu_ir[ra]);
                else
                    tcg_gen_movi_i64(cpu_fir[rc], 0);
            }
            break;
        case 0x2A:
            /* SQRTG */
            if (!(ctx->amask & AMASK_FIX))
                goto invalid_opc;
            gen_farith2(&helper_sqrtg, rb, rc);
            break;
        case 0x02B:
            /* SQRTT */
            if (!(ctx->amask & AMASK_FIX))
                goto invalid_opc;
            gen_farith2(&helper_sqrtt, rb, rc);
            break;
        default:
            goto invalid_opc;
        }
        break;
    case 0x15:
        /* VAX floating point */
        /* XXX: rounding mode and trap are ignored (!) */
        switch (fpfn) { /* f11 & 0x3F */
        case 0x00:
            /* ADDF */
            gen_farith3(&helper_addf, ra, rb, rc);
            break;
        case 0x01:
            /* SUBF */
            gen_farith3(&helper_subf, ra, rb, rc);
            break;
        case 0x02:
            /* MULF */
            gen_farith3(&helper_mulf, ra, rb, rc);
            break;
        case 0x03:
            /* DIVF */
            gen_farith3(&helper_divf, ra, rb, rc);
            break;
        case 0x1E:
            /* CVTDG */
#if 0 // TODO
            gen_farith2(&helper_cvtdg, rb, rc);
#else
            goto invalid_opc;
#endif
            break;
        case 0x20:
            /* ADDG */
            gen_farith3(&helper_addg, ra, rb, rc);
            break;
        case 0x21:
            /* SUBG */
            gen_farith3(&helper_subg, ra, rb, rc);
            break;
        case 0x22:
            /* MULG */
            gen_farith3(&helper_mulg, ra, rb, rc);
            break;
        case 0x23:
            /* DIVG */
            gen_farith3(&helper_divg, ra, rb, rc);
            break;
        case 0x25:
            /* CMPGEQ */
            gen_farith3(&helper_cmpgeq, ra, rb, rc);
            break;
        case 0x26:
            /* CMPGLT */
            gen_farith3(&helper_cmpglt, ra, rb, rc);
            break;
        case 0x27:
            /* CMPGLE */
            gen_farith3(&helper_cmpgle, ra, rb, rc);
            break;
        case 0x2C:
            /* CVTGF */
            gen_farith2(&helper_cvtgf, rb, rc);
            break;
        case 0x2D:
            /* CVTGD */
#if 0 // TODO
            gen_farith2(ctx, &helper_cvtgd, rb, rc);
#else
            goto invalid_opc;
#endif
            break;
        case 0x2F:
            /* CVTGQ */
            gen_farith2(&helper_cvtgq, rb, rc);
            break;
        case 0x3C:
            /* CVTQF */
            gen_farith2(&helper_cvtqf, rb, rc);
            break;
        case 0x3E:
            /* CVTQG */
            gen_farith2(&helper_cvtqg, rb, rc);
            break;
        default:
            goto invalid_opc;
        }
        break;
    case 0x16:
        /* IEEE floating-point */
        /* XXX: rounding mode and traps are ignored (!) */
        switch (fpfn) { /* f11 & 0x3F */
        case 0x00:
            /* ADDS */
            gen_farith3(&helper_adds, ra, rb, rc);
            break;
        case 0x01:
            /* SUBS */
            gen_farith3(&helper_subs, ra, rb, rc);
            break;
        case 0x02:
            /* MULS */
            gen_farith3(&helper_muls, ra, rb, rc);
            break;
        case 0x03:
            /* DIVS */
            gen_farith3(&helper_divs, ra, rb, rc);
            break;
        case 0x20:
            /* ADDT */
            gen_farith3(&helper_addt, ra, rb, rc);
            break;
        case 0x21:
            /* SUBT */
            gen_farith3(&helper_subt, ra, rb, rc);
            break;
        case 0x22:
            /* MULT */
            gen_farith3(&helper_mult, ra, rb, rc);
            break;
        case 0x23:
            /* DIVT */
            gen_farith3(&helper_divt, ra, rb, rc);
            break;
        case 0x24:
            /* CMPTUN */
            gen_farith3(&helper_cmptun, ra, rb, rc);
            break;
        case 0x25:
            /* CMPTEQ */
            gen_farith3(&helper_cmpteq, ra, rb, rc);
            break;
        case 0x26:
            /* CMPTLT */
            gen_farith3(&helper_cmptlt, ra, rb, rc);
            break;
        case 0x27:
            /* CMPTLE */
            gen_farith3(&helper_cmptle, ra, rb, rc);
            break;
        case 0x2C:
            /* XXX: incorrect */
            if (fn11 == 0x2AC) {
                /* CVTST */
                gen_farith2(&helper_cvtst, rb, rc);
            } else {
                /* CVTTS */
                gen_farith2(&helper_cvtts, rb, rc);
            }
            break;
        case 0x2F:
            /* CVTTQ */
            gen_farith2(&helper_cvttq, rb, rc);
            break;
        case 0x3C:
            /* CVTQS */
            gen_farith2(&helper_cvtqs, rb, rc);
            break;
        case 0x3E:
            /* CVTQT */
            gen_farith2(&helper_cvtqt, rb, rc);
            break;
        default:
            goto invalid_opc;
        }
        break;
    case 0x17:
        switch (fn11) {
        case 0x010:
            /* CVTLQ */
            gen_farith2(&helper_cvtlq, rb, rc);
            break;
        case 0x020:
            if (likely(rc != 31)) {
                if (ra == rb)
                    /* FMOV */
                    tcg_gen_mov_i64(cpu_fir[rc], cpu_fir[ra]);
                else
                    /* CPYS */
                    gen_farith3(&helper_cpys, ra, rb, rc);
            }
            break;
        case 0x021:
            /* CPYSN */
            gen_farith3(&helper_cpysn, ra, rb, rc);
            break;
        case 0x022:
            /* CPYSE */
            gen_farith3(&helper_cpyse, ra, rb, rc);
            break;
        case 0x024:
            /* MT_FPCR */
            if (likely(ra != 31))
                tcg_gen_helper_0_1(helper_store_fpcr, cpu_fir[ra]);
            else {
                TCGv tmp = tcg_const_i64(0);
                tcg_gen_helper_0_1(helper_store_fpcr, tmp);
                tcg_temp_free(tmp);
            }
            break;
        case 0x025:
            /* MF_FPCR */
            if (likely(ra != 31))
                tcg_gen_helper_1_0(helper_load_fpcr, cpu_fir[ra]);
            break;
        case 0x02A:
            /* FCMOVEQ */
            gen_fcmov(&helper_cmpfeq, ra, rb, rc);
            break;
        case 0x02B:
            /* FCMOVNE */
            gen_fcmov(&helper_cmpfne, ra, rb, rc);
            break;
        case 0x02C:
            /* FCMOVLT */
            gen_fcmov(&helper_cmpflt, ra, rb, rc);
            break;
        case 0x02D:
            /* FCMOVGE */
            gen_fcmov(&helper_cmpfge, ra, rb, rc);
            break;
        case 0x02E:
            /* FCMOVLE */
            gen_fcmov(&helper_cmpfle, ra, rb, rc);
            break;
        case 0x02F:
            /* FCMOVGT */
            gen_fcmov(&helper_cmpfgt, ra, rb, rc);
            break;
        case 0x030:
            /* CVTQL */
            gen_farith2(&helper_cvtql, rb, rc);
            break;
        case 0x130:
            /* CVTQL/V */
            gen_farith2(&helper_cvtqlv, rb, rc);
            break;
        case 0x530:
            /* CVTQL/SV */
            gen_farith2(&helper_cvtqlsv, rb, rc);
            break;
        default:
            goto invalid_opc;
        }
        break;
    case 0x18:
        switch ((uint16_t)disp16) {
        case 0x0000:
            /* TRAPB */
            /* No-op. Just exit from the current tb */
            ret = 2;
            break;
        case 0x0400:
            /* EXCB */
            /* No-op. Just exit from the current tb */
            ret = 2;
            break;
        case 0x4000:
            /* MB */
            /* No-op */
            break;
        case 0x4400:
            /* WMB */
            /* No-op */
            break;
        case 0x8000:
            /* FETCH */
            /* No-op */
            break;
        case 0xA000:
            /* FETCH_M */
            /* No-op */
            break;
        case 0xC000:
            /* RPCC */
            if (ra != 31)
                tcg_gen_helper_1_0(helper_load_pcc, cpu_ir[ra]);
            break;
        case 0xE000:
            /* RC */
            if (ra != 31)
                tcg_gen_helper_1_0(helper_rc, cpu_ir[ra]);
            break;
        case 0xE800:
            /* ECB */
            /* XXX: TODO: evict tb cache at address rb */
#if 0
            ret = 2;
#else
            goto invalid_opc;
#endif
            break;
        case 0xF000:
            /* RS */
            if (ra != 31)
                tcg_gen_helper_1_0(helper_rs, cpu_ir[ra]);
            break;
        case 0xF800:
            /* WH64 */
            /* No-op */
            break;
        default:
            goto invalid_opc;
        }
        break;
    case 0x19:
        /* HW_MFPR (PALcode) */
#if defined (CONFIG_USER_ONLY)
        goto invalid_opc;
#else
        if (!ctx->pal_mode)
            goto invalid_opc;
        gen_op_mfpr(insn & 0xFF);
        if (ra != 31)
            tcg_gen_mov_i64(cpu_ir[ra], cpu_T[0]);
        break;
#endif
    case 0x1A:
        if (ra != 31)
            tcg_gen_movi_i64(cpu_ir[ra], ctx->pc);
        if (rb != 31)
            tcg_gen_andi_i64(cpu_pc, cpu_ir[rb], ~3);
        else
            tcg_gen_movi_i64(cpu_pc, 0);
        /* Those four jumps only differ by the branch prediction hint */
        switch (fn2) {
        case 0x0:
            /* JMP */
            break;
        case 0x1:
            /* JSR */
            break;
        case 0x2:
            /* RET */
            break;
        case 0x3:
            /* JSR_COROUTINE */
            break;
        }
        ret = 1;
        break;
    case 0x1B:
        /* HW_LD (PALcode) */
#if defined (CONFIG_USER_ONLY)
        goto invalid_opc;
#else
        if (!ctx->pal_mode)
            goto invalid_opc;
        if (rb != 31)
            tcg_gen_mov_i64(cpu_T[0], cpu_ir[rb]);
        else
            tcg_gen_movi_i64(cpu_T[0], 0);
        tcg_gen_movi_i64(cpu_T[1], disp12);
        tcg_gen_add_i64(cpu_T[0], cpu_T[0], cpu_T[1]);
        switch ((insn >> 12) & 0xF) {
        case 0x0:
            /* Longword physical access */
            gen_op_ldl_raw();
            break;
        case 0x1:
            /* Quadword physical access */
            gen_op_ldq_raw();
            break;
        case 0x2:
            /* Longword physical access with lock */
            gen_op_ldl_l_raw();
            break;
        case 0x3:
            /* Quadword physical access with lock */
            gen_op_ldq_l_raw();
            break;
        case 0x4:
            /* Longword virtual PTE fetch */
            gen_op_ldl_kernel();
            break;
        case 0x5:
            /* Quadword virtual PTE fetch */
            gen_op_ldq_kernel();
            break;
        case 0x6:
            /* Invalid */
            goto invalid_opc;
        case 0x7:
            /* Invalid */
            goto invalid_opc;
        case 0x8:
            /* Longword virtual access */
            gen_op_ld_phys_to_virt();
            gen_op_ldl_raw();
            break;
        case 0x9:
            /* Quadword virtual access */
            gen_op_ld_phys_to_virt();
            gen_op_ldq_raw();
            break;
        case 0xA:
            /* Longword virtual access with protection check */
            gen_ldl(ctx);
            break;
        case 0xB:
            /* Quadword virtual access with protection check */
            gen_ldq(ctx);
            break;
        case 0xC:
            /* Longword virtual access with altenate access mode */
            gen_op_set_alt_mode();
            gen_op_ld_phys_to_virt();
            gen_op_ldl_raw();
            gen_op_restore_mode();
            break;
        case 0xD:
            /* Quadword virtual access with altenate access mode */
            gen_op_set_alt_mode();
            gen_op_ld_phys_to_virt();
            gen_op_ldq_raw();
            gen_op_restore_mode();
            break;
        case 0xE:
            /* Longword virtual access with alternate access mode and
             * protection checks
             */
            gen_op_set_alt_mode();
            gen_op_ldl_data();
            gen_op_restore_mode();
            break;
        case 0xF:
            /* Quadword virtual access with alternate access mode and
             * protection checks
             */
            gen_op_set_alt_mode();
            gen_op_ldq_data();
            gen_op_restore_mode();
            break;
        }
        if (ra != 31)
            tcg_gen_mov_i64(cpu_ir[ra], cpu_T[1]);
        break;
#endif
    case 0x1C:
        switch (fn7) {
        case 0x00:
            /* SEXTB */
            if (!(ctx->amask & AMASK_BWX))
                goto invalid_opc;
            if (likely(rc != 31)) {
                if (islit)
                    tcg_gen_movi_i64(cpu_ir[rc], (int64_t)((int8_t)lit));
                else
                    tcg_gen_ext8s_i64(cpu_ir[rc], cpu_ir[rb]);
            }
            break;
        case 0x01:
            /* SEXTW */
            if (!(ctx->amask & AMASK_BWX))
                goto invalid_opc;
            if (likely(rc != 31)) {
                if (islit)
                    tcg_gen_movi_i64(cpu_ir[rc], (int64_t)((int16_t)lit));
                else
                    tcg_gen_ext16s_i64(cpu_ir[rc], cpu_ir[rb]);
            }
            break;
        case 0x30:
            /* CTPOP */
            if (!(ctx->amask & AMASK_CIX))
                goto invalid_opc;
            if (likely(rc != 31)) {
                if (islit)
                    tcg_gen_movi_i64(cpu_ir[rc], ctpop64(lit));
                else
                    tcg_gen_helper_1_1(helper_ctpop, cpu_ir[rc], cpu_ir[rb]);
            }
            break;
        case 0x31:
            /* PERR */
            if (!(ctx->amask & AMASK_MVI))
                goto invalid_opc;
            /* XXX: TODO */
            goto invalid_opc;
            break;
        case 0x32:
            /* CTLZ */
            if (!(ctx->amask & AMASK_CIX))
                goto invalid_opc;
            if (likely(rc != 31)) {
                if (islit)
                    tcg_gen_movi_i64(cpu_ir[rc], clz64(lit));
                else
                    tcg_gen_helper_1_1(helper_ctlz, cpu_ir[rc], cpu_ir[rb]);
            }
            break;
        case 0x33:
            /* CTTZ */
            if (!(ctx->amask & AMASK_CIX))
                goto invalid_opc;
            if (likely(rc != 31)) {
                if (islit)
                    tcg_gen_movi_i64(cpu_ir[rc], ctz64(lit));
                else
                    tcg_gen_helper_1_1(helper_cttz, cpu_ir[rc], cpu_ir[rb]);
            }
            break;
        case 0x34:
            /* UNPKBW */
            if (!(ctx->amask & AMASK_MVI))
                goto invalid_opc;
            /* XXX: TODO */
            goto invalid_opc;
            break;
        case 0x35:
            /* UNPKWL */
            if (!(ctx->amask & AMASK_MVI))
                goto invalid_opc;
            /* XXX: TODO */
            goto invalid_opc;
            break;
        case 0x36:
            /* PKWB */
            if (!(ctx->amask & AMASK_MVI))
                goto invalid_opc;
            /* XXX: TODO */
            goto invalid_opc;
            break;
        case 0x37:
            /* PKLB */
            if (!(ctx->amask & AMASK_MVI))
                goto invalid_opc;
            /* XXX: TODO */
            goto invalid_opc;
            break;
        case 0x38:
            /* MINSB8 */
            if (!(ctx->amask & AMASK_MVI))
                goto invalid_opc;
            /* XXX: TODO */
            goto invalid_opc;
            break;
        case 0x39:
            /* MINSW4 */
            if (!(ctx->amask & AMASK_MVI))
                goto invalid_opc;
            /* XXX: TODO */
            goto invalid_opc;
            break;
        case 0x3A:
            /* MINUB8 */
            if (!(ctx->amask & AMASK_MVI))
                goto invalid_opc;
            /* XXX: TODO */
            goto invalid_opc;
            break;
        case 0x3B:
            /* MINUW4 */
            if (!(ctx->amask & AMASK_MVI))
                goto invalid_opc;
            /* XXX: TODO */
            goto invalid_opc;
            break;
        case 0x3C:
            /* MAXUB8 */
            if (!(ctx->amask & AMASK_MVI))
                goto invalid_opc;
            /* XXX: TODO */
            goto invalid_opc;
            break;
        case 0x3D:
            /* MAXUW4 */
            if (!(ctx->amask & AMASK_MVI))
                goto invalid_opc;
            /* XXX: TODO */
            goto invalid_opc;
            break;
        case 0x3E:
            /* MAXSB8 */
            if (!(ctx->amask & AMASK_MVI))
                goto invalid_opc;
            /* XXX: TODO */
            goto invalid_opc;
            break;
        case 0x3F:
            /* MAXSW4 */
            if (!(ctx->amask & AMASK_MVI))
                goto invalid_opc;
            /* XXX: TODO */
            goto invalid_opc;
            break;
        case 0x70:
            /* FTOIT */
            if (!(ctx->amask & AMASK_FIX))
                goto invalid_opc;
            if (likely(rc != 31)) {
                if (ra != 31)
                    tcg_gen_mov_i64(cpu_ir[rc], cpu_fir[ra]);
                else
                    tcg_gen_movi_i64(cpu_ir[rc], 0);
            }
            break;
        case 0x78:
            /* FTOIS */
            if (!(ctx->amask & AMASK_FIX))
                goto invalid_opc;
            if (rc != 31) {
                TCGv tmp1 = tcg_temp_new(TCG_TYPE_I32);
                if (ra != 31)
                    tcg_gen_helper_1_1(helper_s_to_memory, tmp1, cpu_fir[ra]);
                else {
                    TCGv tmp2 = tcg_const_i64(0);
                    tcg_gen_helper_1_1(helper_s_to_memory, tmp1, tmp2);
                    tcg_temp_free(tmp2);
                }
                tcg_gen_ext_i32_i64(cpu_ir[rc], tmp1);
                tcg_temp_free(tmp1);
            }
            break;
        default:
            goto invalid_opc;
        }
        break;
    case 0x1D:
        /* HW_MTPR (PALcode) */
#if defined (CONFIG_USER_ONLY)
        goto invalid_opc;
#else
        if (!ctx->pal_mode)
            goto invalid_opc;
        if (ra != 31)
            tcg_gen_mov_i64(cpu_T[0], cpu_ir[ra]);
        else
            tcg_gen_movi_i64(cpu_T[0], 0);
        gen_op_mtpr(insn & 0xFF);
        ret = 2;
        break;
#endif
    case 0x1E:
        /* HW_REI (PALcode) */
#if defined (CONFIG_USER_ONLY)
        goto invalid_opc;
#else
        if (!ctx->pal_mode)
            goto invalid_opc;
        if (rb == 31) {
            /* "Old" alpha */
            gen_op_hw_rei();
        } else {
            if (ra != 31)
                tcg_gen_mov_i64(cpu_T[0], cpu_ir[rb]);
            else
                tcg_gen_movi_i64(cpu_T[0], 0);
            tcg_gen_movi_i64(cpu_T[1], (((int64_t)insn << 51) >> 51));
            tcg_gen_add_i64(cpu_T[0], cpu_T[0], cpu_T[1]);
            gen_op_hw_ret();
        }
        ret = 2;
        break;
#endif
    case 0x1F:
        /* HW_ST (PALcode) */
#if defined (CONFIG_USER_ONLY)
        goto invalid_opc;
#else
        if (!ctx->pal_mode)
            goto invalid_opc;
        if (ra != 31)
            tcg_gen_addi_i64(cpu_T[0], cpu_ir[rb], disp12);
        else
            tcg_gen_movi_i64(cpu_T[0], disp12);
        if (ra != 31)
            tcg_gen_mov_i64(cpu_T[1], cpu_ir[ra]);
        else
            tcg_gen_movi_i64(cpu_T[1], 0);
        switch ((insn >> 12) & 0xF) {
        case 0x0:
            /* Longword physical access */
            gen_op_stl_raw();
            break;
        case 0x1:
            /* Quadword physical access */
            gen_op_stq_raw();
            break;
        case 0x2:
            /* Longword physical access with lock */
            gen_op_stl_c_raw();
            break;
        case 0x3:
            /* Quadword physical access with lock */
            gen_op_stq_c_raw();
            break;
        case 0x4:
            /* Longword virtual access */
            gen_op_st_phys_to_virt();
            gen_op_stl_raw();
            break;
        case 0x5:
            /* Quadword virtual access */
            gen_op_st_phys_to_virt();
            gen_op_stq_raw();
            break;
        case 0x6:
            /* Invalid */
            goto invalid_opc;
        case 0x7:
            /* Invalid */
            goto invalid_opc;
        case 0x8:
            /* Invalid */
            goto invalid_opc;
        case 0x9:
            /* Invalid */
            goto invalid_opc;
        case 0xA:
            /* Invalid */
            goto invalid_opc;
        case 0xB:
            /* Invalid */
            goto invalid_opc;
        case 0xC:
            /* Longword virtual access with alternate access mode */
            gen_op_set_alt_mode();
            gen_op_st_phys_to_virt();
            gen_op_ldl_raw();
            gen_op_restore_mode();
            break;
        case 0xD:
            /* Quadword virtual access with alternate access mode */
            gen_op_set_alt_mode();
            gen_op_st_phys_to_virt();
            gen_op_ldq_raw();
            gen_op_restore_mode();
            break;
        case 0xE:
            /* Invalid */
            goto invalid_opc;
        case 0xF:
            /* Invalid */
            goto invalid_opc;
        }
        ret = 2;
        break;
#endif
    case 0x20:
        /* LDF */
        gen_load_mem(ctx, &gen_qemu_ldf, ra, rb, disp16, 1, 0);
        break;
    case 0x21:
        /* LDG */
        gen_load_mem(ctx, &gen_qemu_ldg, ra, rb, disp16, 1, 0);
        break;
    case 0x22:
        /* LDS */
        gen_load_mem(ctx, &gen_qemu_lds, ra, rb, disp16, 1, 0);
        break;
    case 0x23:
        /* LDT */
        gen_load_mem(ctx, &tcg_gen_qemu_ld64, ra, rb, disp16, 1, 0);
        break;
    case 0x24:
        /* STF */
        gen_store_mem(ctx, &gen_qemu_stf, ra, rb, disp16, 1, 0);
        break;
    case 0x25:
        /* STG */
        gen_store_mem(ctx, &gen_qemu_stg, ra, rb, disp16, 1, 0);
        break;
    case 0x26:
        /* STS */
        gen_store_mem(ctx, &gen_qemu_sts, ra, rb, disp16, 1, 0);
        break;
    case 0x27:
        /* STT */
        gen_store_mem(ctx, &tcg_gen_qemu_st64, ra, rb, disp16, 1, 0);
        break;
    case 0x28:
        /* LDL */
        gen_load_mem(ctx, &tcg_gen_qemu_ld32s, ra, rb, disp16, 0, 0);
        break;
    case 0x29:
        /* LDQ */
        gen_load_mem(ctx, &tcg_gen_qemu_ld64, ra, rb, disp16, 0, 0);
        break;
    case 0x2A:
        /* LDL_L */
        gen_load_mem_dyngen(ctx, &gen_ldl_l, ra, rb, disp16, 0);
        break;
    case 0x2B:
        /* LDQ_L */
        gen_load_mem_dyngen(ctx, &gen_ldq_l, ra, rb, disp16, 0);
        break;
    case 0x2C:
        /* STL */
        gen_store_mem(ctx, &tcg_gen_qemu_st32, ra, rb, disp16, 0, 0);
        break;
    case 0x2D:
        /* STQ */
        gen_store_mem(ctx, &tcg_gen_qemu_st64, ra, rb, disp16, 0, 0);
        break;
    case 0x2E:
        /* STL_C */
        gen_store_mem_dyngen(ctx, &gen_stl_c, ra, rb, disp16, 0);
        break;
    case 0x2F:
        /* STQ_C */
        gen_store_mem_dyngen(ctx, &gen_stq_c, ra, rb, disp16, 0);
        break;
    case 0x30:
        /* BR */
        if (ra != 31)
            tcg_gen_movi_i64(cpu_ir[ra], ctx->pc);
        tcg_gen_movi_i64(cpu_pc, ctx->pc + (int64_t)(disp21 << 2));
        ret = 1;
        break;
    case 0x31:
        /* FBEQ */
        gen_fbcond(ctx, &helper_cmpfeq, ra, disp16);
        ret = 1;
        break;
    case 0x32:
        /* FBLT */
        gen_fbcond(ctx, &helper_cmpflt, ra, disp16);
        ret = 1;
        break;
    case 0x33:
        /* FBLE */
        gen_fbcond(ctx, &helper_cmpfle, ra, disp16);
        ret = 1;
        break;
    case 0x34:
        /* BSR */
        if (ra != 31)
            tcg_gen_movi_i64(cpu_ir[ra], ctx->pc);
        tcg_gen_movi_i64(cpu_pc, ctx->pc + (int64_t)(disp21 << 2));
        ret = 1;
        break;
    case 0x35:
        /* FBNE */
        gen_fbcond(ctx, &helper_cmpfne, ra, disp16);
        ret = 1;
        break;
    case 0x36:
        /* FBGE */
        gen_fbcond(ctx, &helper_cmpfge, ra, disp16);
        ret = 1;
        break;
    case 0x37:
        /* FBGT */
        gen_fbcond(ctx, &helper_cmpfgt, ra, disp16);
        ret = 1;
        break;
    case 0x38:
        /* BLBC */
        gen_bcond(ctx, TCG_COND_EQ, ra, disp16, 1);
        ret = 1;
        break;
    case 0x39:
        /* BEQ */
        gen_bcond(ctx, TCG_COND_EQ, ra, disp16, 0);
        ret = 1;
        break;
    case 0x3A:
        /* BLT */
        gen_bcond(ctx, TCG_COND_LT, ra, disp16, 0);
        ret = 1;
        break;
    case 0x3B:
        /* BLE */
        gen_bcond(ctx, TCG_COND_LE, ra, disp16, 0);
        ret = 1;
        break;
    case 0x3C:
        /* BLBS */
        gen_bcond(ctx, TCG_COND_NE, ra, disp16, 1);
        ret = 1;
        break;
    case 0x3D:
        /* BNE */
        gen_bcond(ctx, TCG_COND_NE, ra, disp16, 0);
        ret = 1;
        break;
    case 0x3E:
        /* BGE */
        gen_bcond(ctx, TCG_COND_GE, ra, disp16, 0);
        ret = 1;
        break;
    case 0x3F:
        /* BGT */
        gen_bcond(ctx, TCG_COND_GT, ra, disp16, 0);
        ret = 1;
        break;
    invalid_opc:
        gen_invalid(ctx);
        ret = 3;
        break;
    }

    return ret;
}

static always_inline void gen_intermediate_code_internal (CPUState *env,
                                                          TranslationBlock *tb,
                                                          int search_pc)
{
#if defined ALPHA_DEBUG_DISAS
    static int insn_count;
#endif
    DisasContext ctx, *ctxp = &ctx;
    target_ulong pc_start;
    uint32_t insn;
    uint16_t *gen_opc_end;
    int j, lj = -1;
    int ret;
    int num_insns;
    int max_insns;

    pc_start = tb->pc;
    gen_opc_end = gen_opc_buf + OPC_MAX_SIZE;
    ctx.pc = pc_start;
    ctx.amask = env->amask;
#if defined (CONFIG_USER_ONLY)
    ctx.mem_idx = 0;
#else
    ctx.mem_idx = ((env->ps >> 3) & 3);
    ctx.pal_mode = env->ipr[IPR_EXC_ADDR] & 1;
#endif
    num_insns = 0;
    max_insns = tb->cflags & CF_COUNT_MASK;
    if (max_insns == 0)
        max_insns = CF_COUNT_MASK;

    gen_icount_start();
    for (ret = 0; ret == 0;) {
        if (env->nb_breakpoints > 0) {
            for(j = 0; j < env->nb_breakpoints; j++) {
                if (env->breakpoints[j] == ctx.pc) {
                    gen_excp(&ctx, EXCP_DEBUG, 0);
                    break;
                }
            }
        }
        if (search_pc) {
            j = gen_opc_ptr - gen_opc_buf;
            if (lj < j) {
                lj++;
                while (lj < j)
                    gen_opc_instr_start[lj++] = 0;
                gen_opc_pc[lj] = ctx.pc;
                gen_opc_instr_start[lj] = 1;
                gen_opc_icount[lj] = num_insns;
            }
        }
        if (num_insns + 1 == max_insns && (tb->cflags & CF_LAST_IO))
            gen_io_start();
#if defined ALPHA_DEBUG_DISAS
        insn_count++;
        if (logfile != NULL) {
            fprintf(logfile, "pc " TARGET_FMT_lx " mem_idx %d\n",
                    ctx.pc, ctx.mem_idx);
        }
#endif
        insn = ldl_code(ctx.pc);
#if defined ALPHA_DEBUG_DISAS
        insn_count++;
        if (logfile != NULL) {
            fprintf(logfile, "opcode %08x %d\n", insn, insn_count);
        }
#endif
        num_insns++;
        ctx.pc += 4;
        ret = translate_one(ctxp, insn);
        if (ret != 0)
            break;
        /* if we reach a page boundary or are single stepping, stop
         * generation
         */
        if (((ctx.pc & (TARGET_PAGE_SIZE - 1)) == 0) ||
            (env->singlestep_enabled) ||
            num_insns >= max_insns) {
            break;
        }
#if defined (DO_SINGLE_STEP)
        break;
#endif
    }
    if (ret != 1 && ret != 3) {
        tcg_gen_movi_i64(cpu_pc, ctx.pc);
    }
#if defined (DO_TB_FLUSH)
    tcg_gen_helper_0_0(helper_tb_flush);
#endif
    if (tb->cflags & CF_LAST_IO)
        gen_io_end();
    /* Generate the return instruction */
    tcg_gen_exit_tb(0);
    gen_icount_end(tb, num_insns);
    *gen_opc_ptr = INDEX_op_end;
    if (search_pc) {
        j = gen_opc_ptr - gen_opc_buf;
        lj++;
        while (lj <= j)
            gen_opc_instr_start[lj++] = 0;
    } else {
        tb->size = ctx.pc - pc_start;
        tb->icount = num_insns;
    }
#if defined ALPHA_DEBUG_DISAS
    if (loglevel & CPU_LOG_TB_CPU) {
        cpu_dump_state(env, logfile, fprintf, 0);
    }
    if (loglevel & CPU_LOG_TB_IN_ASM) {
        fprintf(logfile, "IN: %s\n", lookup_symbol(pc_start));
        target_disas(logfile, pc_start, ctx.pc - pc_start, 1);
        fprintf(logfile, "\n");
    }
#endif
}

void gen_intermediate_code (CPUState *env, struct TranslationBlock *tb)
{
    gen_intermediate_code_internal(env, tb, 0);
}

void gen_intermediate_code_pc (CPUState *env, struct TranslationBlock *tb)
{
    gen_intermediate_code_internal(env, tb, 1);
}

CPUAlphaState * cpu_alpha_init (const char *cpu_model)
{
    CPUAlphaState *env;
    uint64_t hwpcb;

    env = qemu_mallocz(sizeof(CPUAlphaState));
    if (!env)
        return NULL;
    cpu_exec_init(env);
    alpha_translate_init();
    tlb_flush(env, 1);
    /* XXX: should not be hardcoded */
    env->implver = IMPLVER_2106x;
    env->ps = 0x1F00;
#if defined (CONFIG_USER_ONLY)
    env->ps |= 1 << 3;
#endif
    pal_init(env);
    /* Initialize IPR */
    hwpcb = env->ipr[IPR_PCBB];
    env->ipr[IPR_ASN] = 0;
    env->ipr[IPR_ASTEN] = 0;
    env->ipr[IPR_ASTSR] = 0;
    env->ipr[IPR_DATFX] = 0;
    /* XXX: fix this */
    //    env->ipr[IPR_ESP] = ldq_raw(hwpcb + 8);
    //    env->ipr[IPR_KSP] = ldq_raw(hwpcb + 0);
    //    env->ipr[IPR_SSP] = ldq_raw(hwpcb + 16);
    //    env->ipr[IPR_USP] = ldq_raw(hwpcb + 24);
    env->ipr[IPR_FEN] = 0;
    env->ipr[IPR_IPL] = 31;
    env->ipr[IPR_MCES] = 0;
    env->ipr[IPR_PERFMON] = 0; /* Implementation specific */
    //    env->ipr[IPR_PTBR] = ldq_raw(hwpcb + 32);
    env->ipr[IPR_SISR] = 0;
    env->ipr[IPR_VIRBND] = -1ULL;

    return env;
}

void gen_pc_load(CPUState *env, TranslationBlock *tb,
                unsigned long searched_pc, int pc_pos, void *puc)
{
    env->pc = gen_opc_pc[pc_pos];
}
