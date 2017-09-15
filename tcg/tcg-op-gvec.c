/*
 * Generic vector operation expansion
 *
 * Copyright (c) 2018 Linaro
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

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "tcg.h"
#include "tcg-op.h"
#include "tcg-op-gvec.h"
#include "tcg-gvec-desc.h"

#define MAX_UNROLL  4

/* Verify vector size and alignment rules.  OFS should be the OR of all
   of the operand offsets so that we can check them all at once.  */
static void check_size_align(uint32_t oprsz, uint32_t maxsz, uint32_t ofs)
{
    uint32_t opr_align = oprsz >= 16 ? 15 : 7;
    uint32_t max_align = maxsz >= 16 || oprsz >= 16 ? 15 : 7;
    tcg_debug_assert(oprsz > 0);
    tcg_debug_assert(oprsz <= maxsz);
    tcg_debug_assert((oprsz & opr_align) == 0);
    tcg_debug_assert((maxsz & max_align) == 0);
    tcg_debug_assert((ofs & max_align) == 0);
}

/* Verify vector overlap rules for two operands.  */
static void check_overlap_2(uint32_t d, uint32_t a, uint32_t s)
{
    tcg_debug_assert(d == a || d + s <= a || a + s <= d);
}

/* Verify vector overlap rules for three operands.  */
static void check_overlap_3(uint32_t d, uint32_t a, uint32_t b, uint32_t s)
{
    check_overlap_2(d, a, s);
    check_overlap_2(d, b, s);
    check_overlap_2(a, b, s);
}

/* Verify vector overlap rules for four operands.  */
static void check_overlap_4(uint32_t d, uint32_t a, uint32_t b,
                            uint32_t c, uint32_t s)
{
    check_overlap_2(d, a, s);
    check_overlap_2(d, b, s);
    check_overlap_2(d, c, s);
    check_overlap_2(a, b, s);
    check_overlap_2(a, c, s);
    check_overlap_2(b, c, s);
}

/* Create a descriptor from components.  */
uint32_t simd_desc(uint32_t oprsz, uint32_t maxsz, int32_t data)
{
    uint32_t desc = 0;

    assert(oprsz % 8 == 0 && oprsz <= (8 << SIMD_OPRSZ_BITS));
    assert(maxsz % 8 == 0 && maxsz <= (8 << SIMD_MAXSZ_BITS));
    assert(data == sextract32(data, 0, SIMD_DATA_BITS));

    oprsz = (oprsz / 8) - 1;
    maxsz = (maxsz / 8) - 1;
    desc = deposit32(desc, SIMD_OPRSZ_SHIFT, SIMD_OPRSZ_BITS, oprsz);
    desc = deposit32(desc, SIMD_MAXSZ_SHIFT, SIMD_MAXSZ_BITS, maxsz);
    desc = deposit32(desc, SIMD_DATA_SHIFT, SIMD_DATA_BITS, data);

    return desc;
}

/* Generate a call to a gvec-style helper with two vector operands.  */
void tcg_gen_gvec_2_ool(uint32_t dofs, uint32_t aofs,
                        uint32_t oprsz, uint32_t maxsz, int32_t data,
                        gen_helper_gvec_2 *fn)
{
    TCGv_ptr a0, a1;
    TCGv_i32 desc = tcg_const_i32(simd_desc(oprsz, maxsz, data));

    a0 = tcg_temp_new_ptr();
    a1 = tcg_temp_new_ptr();

    tcg_gen_addi_ptr(a0, cpu_env, dofs);
    tcg_gen_addi_ptr(a1, cpu_env, aofs);

    fn(a0, a1, desc);

    tcg_temp_free_ptr(a0);
    tcg_temp_free_ptr(a1);
    tcg_temp_free_i32(desc);
}

/* Generate a call to a gvec-style helper with three vector operands.  */
void tcg_gen_gvec_3_ool(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                        uint32_t oprsz, uint32_t maxsz, int32_t data,
                        gen_helper_gvec_3 *fn)
{
    TCGv_ptr a0, a1, a2;
    TCGv_i32 desc = tcg_const_i32(simd_desc(oprsz, maxsz, data));

    a0 = tcg_temp_new_ptr();
    a1 = tcg_temp_new_ptr();
    a2 = tcg_temp_new_ptr();

    tcg_gen_addi_ptr(a0, cpu_env, dofs);
    tcg_gen_addi_ptr(a1, cpu_env, aofs);
    tcg_gen_addi_ptr(a2, cpu_env, bofs);

    fn(a0, a1, a2, desc);

    tcg_temp_free_ptr(a0);
    tcg_temp_free_ptr(a1);
    tcg_temp_free_ptr(a2);
    tcg_temp_free_i32(desc);
}

/* Generate a call to a gvec-style helper with four vector operands.  */
void tcg_gen_gvec_4_ool(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                        uint32_t cofs, uint32_t oprsz, uint32_t maxsz,
                        int32_t data, gen_helper_gvec_4 *fn)
{
    TCGv_ptr a0, a1, a2, a3;
    TCGv_i32 desc = tcg_const_i32(simd_desc(oprsz, maxsz, data));

    a0 = tcg_temp_new_ptr();
    a1 = tcg_temp_new_ptr();
    a2 = tcg_temp_new_ptr();
    a3 = tcg_temp_new_ptr();

    tcg_gen_addi_ptr(a0, cpu_env, dofs);
    tcg_gen_addi_ptr(a1, cpu_env, aofs);
    tcg_gen_addi_ptr(a2, cpu_env, bofs);
    tcg_gen_addi_ptr(a3, cpu_env, cofs);

    fn(a0, a1, a2, a3, desc);

    tcg_temp_free_ptr(a0);
    tcg_temp_free_ptr(a1);
    tcg_temp_free_ptr(a2);
    tcg_temp_free_ptr(a3);
    tcg_temp_free_i32(desc);
}

/* Generate a call to a gvec-style helper with five vector operands.  */
void tcg_gen_gvec_5_ool(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                        uint32_t cofs, uint32_t xofs, uint32_t oprsz,
                        uint32_t maxsz, int32_t data, gen_helper_gvec_5 *fn)
{
    TCGv_ptr a0, a1, a2, a3, a4;
    TCGv_i32 desc = tcg_const_i32(simd_desc(oprsz, maxsz, data));

    a0 = tcg_temp_new_ptr();
    a1 = tcg_temp_new_ptr();
    a2 = tcg_temp_new_ptr();
    a3 = tcg_temp_new_ptr();
    a4 = tcg_temp_new_ptr();

    tcg_gen_addi_ptr(a0, cpu_env, dofs);
    tcg_gen_addi_ptr(a1, cpu_env, aofs);
    tcg_gen_addi_ptr(a2, cpu_env, bofs);
    tcg_gen_addi_ptr(a3, cpu_env, cofs);
    tcg_gen_addi_ptr(a4, cpu_env, xofs);

    fn(a0, a1, a2, a3, a4, desc);

    tcg_temp_free_ptr(a0);
    tcg_temp_free_ptr(a1);
    tcg_temp_free_ptr(a2);
    tcg_temp_free_ptr(a3);
    tcg_temp_free_ptr(a4);
    tcg_temp_free_i32(desc);
}

/* Generate a call to a gvec-style helper with three vector operands
   and an extra pointer operand.  */
void tcg_gen_gvec_2_ptr(uint32_t dofs, uint32_t aofs,
                        TCGv_ptr ptr, uint32_t oprsz, uint32_t maxsz,
                        int32_t data, gen_helper_gvec_2_ptr *fn)
{
    TCGv_ptr a0, a1;
    TCGv_i32 desc = tcg_const_i32(simd_desc(oprsz, maxsz, data));

    a0 = tcg_temp_new_ptr();
    a1 = tcg_temp_new_ptr();

    tcg_gen_addi_ptr(a0, cpu_env, dofs);
    tcg_gen_addi_ptr(a1, cpu_env, aofs);

    fn(a0, a1, ptr, desc);

    tcg_temp_free_ptr(a0);
    tcg_temp_free_ptr(a1);
    tcg_temp_free_i32(desc);
}

/* Generate a call to a gvec-style helper with three vector operands
   and an extra pointer operand.  */
void tcg_gen_gvec_3_ptr(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                        TCGv_ptr ptr, uint32_t oprsz, uint32_t maxsz,
                        int32_t data, gen_helper_gvec_3_ptr *fn)
{
    TCGv_ptr a0, a1, a2;
    TCGv_i32 desc = tcg_const_i32(simd_desc(oprsz, maxsz, data));

    a0 = tcg_temp_new_ptr();
    a1 = tcg_temp_new_ptr();
    a2 = tcg_temp_new_ptr();

    tcg_gen_addi_ptr(a0, cpu_env, dofs);
    tcg_gen_addi_ptr(a1, cpu_env, aofs);
    tcg_gen_addi_ptr(a2, cpu_env, bofs);

    fn(a0, a1, a2, ptr, desc);

    tcg_temp_free_ptr(a0);
    tcg_temp_free_ptr(a1);
    tcg_temp_free_ptr(a2);
    tcg_temp_free_i32(desc);
}

/* Generate a call to a gvec-style helper with four vector operands
   and an extra pointer operand.  */
void tcg_gen_gvec_4_ptr(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                        uint32_t cofs, TCGv_ptr ptr, uint32_t oprsz,
                        uint32_t maxsz, int32_t data,
                        gen_helper_gvec_4_ptr *fn)
{
    TCGv_ptr a0, a1, a2, a3;
    TCGv_i32 desc = tcg_const_i32(simd_desc(oprsz, maxsz, data));

    a0 = tcg_temp_new_ptr();
    a1 = tcg_temp_new_ptr();
    a2 = tcg_temp_new_ptr();
    a3 = tcg_temp_new_ptr();

    tcg_gen_addi_ptr(a0, cpu_env, dofs);
    tcg_gen_addi_ptr(a1, cpu_env, aofs);
    tcg_gen_addi_ptr(a2, cpu_env, bofs);
    tcg_gen_addi_ptr(a3, cpu_env, cofs);

    fn(a0, a1, a2, a3, ptr, desc);

    tcg_temp_free_ptr(a0);
    tcg_temp_free_ptr(a1);
    tcg_temp_free_ptr(a2);
    tcg_temp_free_ptr(a3);
    tcg_temp_free_i32(desc);
}

/* Return true if we want to implement something of OPRSZ bytes
   in units of LNSZ.  This limits the expansion of inline code.  */
static inline bool check_size_impl(uint32_t oprsz, uint32_t lnsz)
{
    uint32_t lnct = oprsz / lnsz;
    return lnct >= 1 && lnct <= MAX_UNROLL;
}

static void expand_clr(uint32_t dofs, uint32_t maxsz);

/* Duplicate C as per VECE.  */
uint64_t (dup_const)(unsigned vece, uint64_t c)
{
    switch (vece) {
    case MO_8:
        return 0x0101010101010101ull * (uint8_t)c;
    case MO_16:
        return 0x0001000100010001ull * (uint16_t)c;
    case MO_32:
        return 0x0000000100000001ull * (uint32_t)c;
    case MO_64:
        return c;
    default:
        g_assert_not_reached();
    }
}

/* Duplicate IN into OUT as per VECE.  */
static void gen_dup_i32(unsigned vece, TCGv_i32 out, TCGv_i32 in)
{
    switch (vece) {
    case MO_8:
        tcg_gen_ext8u_i32(out, in);
        tcg_gen_muli_i32(out, out, 0x01010101);
        break;
    case MO_16:
        tcg_gen_deposit_i32(out, in, in, 16, 16);
        break;
    case MO_32:
        tcg_gen_mov_i32(out, in);
        break;
    default:
        g_assert_not_reached();
    }
}

static void gen_dup_i64(unsigned vece, TCGv_i64 out, TCGv_i64 in)
{
    switch (vece) {
    case MO_8:
        tcg_gen_ext8u_i64(out, in);
        tcg_gen_muli_i64(out, out, 0x0101010101010101ull);
        break;
    case MO_16:
        tcg_gen_ext16u_i64(out, in);
        tcg_gen_muli_i64(out, out, 0x0001000100010001ull);
        break;
    case MO_32:
        tcg_gen_deposit_i64(out, in, in, 32, 32);
        break;
    case MO_64:
        tcg_gen_mov_i64(out, in);
        break;
    default:
        g_assert_not_reached();
    }
}

/* Set OPRSZ bytes at DOFS to replications of IN_32, IN_64 or IN_C.
 * Only one of IN_32 or IN_64 may be set;
 * IN_C is used if IN_32 and IN_64 are unset.
 */
static void do_dup(unsigned vece, uint32_t dofs, uint32_t oprsz,
                   uint32_t maxsz, TCGv_i32 in_32, TCGv_i64 in_64,
                   uint64_t in_c)
{
    TCGType type;
    TCGv_i64 t_64;
    TCGv_i32 t_32, t_desc;
    TCGv_ptr t_ptr;
    uint32_t i;

    assert(vece <= (in_32 ? MO_32 : MO_64));
    assert(in_32 == NULL || in_64 == NULL);

    /* If we're storing 0, expand oprsz to maxsz.  */
    if (in_32 == NULL && in_64 == NULL) {
        in_c = dup_const(vece, in_c);
        if (in_c == 0) {
            oprsz = maxsz;
        }
    }

    type = 0;
    if (TCG_TARGET_HAS_v256 && check_size_impl(oprsz, 32)) {
        type = TCG_TYPE_V256;
    } else if (TCG_TARGET_HAS_v128 && check_size_impl(oprsz, 16)) {
        type = TCG_TYPE_V128;
    } else if (TCG_TARGET_HAS_v64 && check_size_impl(oprsz, 8)
               /* Prefer integer when 64-bit host and no variable dup.  */
               && !(TCG_TARGET_REG_BITS == 64 && in_32 == NULL
                    && (in_64 == NULL || vece == MO_64))) {
        type = TCG_TYPE_V64;
    }

    /* Implement inline with a vector type, if possible.  */
    if (type != 0) {
        TCGv_vec t_vec = tcg_temp_new_vec(type);

        if (in_32) {
            tcg_gen_dup_i32_vec(vece, t_vec, in_32);
        } else if (in_64) {
            tcg_gen_dup_i64_vec(vece, t_vec, in_64);
        } else {
            switch (vece) {
            case MO_8:
                tcg_gen_dup8i_vec(t_vec, in_c);
                break;
            case MO_16:
                tcg_gen_dup16i_vec(t_vec, in_c);
                break;
            case MO_32:
                tcg_gen_dup32i_vec(t_vec, in_c);
                break;
            default:
                tcg_gen_dup64i_vec(t_vec, in_c);
                break;
            }
        }

        i = 0;
        if (TCG_TARGET_HAS_v256) {
            for (; i + 32 <= oprsz; i += 32) {
                tcg_gen_stl_vec(t_vec, cpu_env, dofs + i, TCG_TYPE_V256);
            }
        }
        if (TCG_TARGET_HAS_v128) {
            for (; i + 16 <= oprsz; i += 16) {
                tcg_gen_stl_vec(t_vec, cpu_env, dofs + i, TCG_TYPE_V128);
            }
        }
        if (TCG_TARGET_HAS_v64) {
            for (; i < oprsz; i += 8) {
                tcg_gen_stl_vec(t_vec, cpu_env, dofs + i, TCG_TYPE_V64);
            }
        }
        tcg_temp_free_vec(t_vec);
        goto done;
    }

    /* Otherwise, inline with an integer type, unless "large".  */
    if (check_size_impl(oprsz, TCG_TARGET_REG_BITS / 8)) {
        t_64 = NULL;
        t_32 = NULL;

        if (in_32) {
            /* We are given a 32-bit variable input.  For a 64-bit host,
               use a 64-bit operation unless the 32-bit operation would
               be simple enough.  */
            if (TCG_TARGET_REG_BITS == 64
                && (vece != MO_32 || !check_size_impl(oprsz, 4))) {
                t_64 = tcg_temp_new_i64();
                tcg_gen_extu_i32_i64(t_64, in_32);
                gen_dup_i64(vece, t_64, t_64);
            } else {
                t_32 = tcg_temp_new_i32();
                gen_dup_i32(vece, t_32, in_32);
            }
        } else if (in_64) {
            /* We are given a 64-bit variable input.  */
            t_64 = tcg_temp_new_i64();
            gen_dup_i64(vece, t_64, in_64);
        } else {
            /* We are given a constant input.  */
            /* For 64-bit hosts, use 64-bit constants for "simple" constants
               or when we'd need too many 32-bit stores, or when a 64-bit
               constant is really required.  */
            if (vece == MO_64
                || (TCG_TARGET_REG_BITS == 64
                    && (in_c == 0 || in_c == -1
                        || !check_size_impl(oprsz, 4)))) {
                t_64 = tcg_const_i64(in_c);
            } else {
                t_32 = tcg_const_i32(in_c);
            }
        }

        /* Implement inline if we picked an implementation size above.  */
        if (t_32) {
            for (i = 0; i < oprsz; i += 4) {
                tcg_gen_st_i32(t_32, cpu_env, dofs + i);
            }
            tcg_temp_free_i32(t_32);
            goto done;
        }
        if (t_64) {
            for (i = 0; i < oprsz; i += 8) {
                tcg_gen_st_i64(t_64, cpu_env, dofs + i);
            }
            tcg_temp_free_i64(t_64);
            goto done;
        } 
    }

    /* Otherwise implement out of line.  */
    t_ptr = tcg_temp_new_ptr();
    tcg_gen_addi_ptr(t_ptr, cpu_env, dofs);
    t_desc = tcg_const_i32(simd_desc(oprsz, maxsz, 0));

    if (vece == MO_64) {
        if (in_64) {
            gen_helper_gvec_dup64(t_ptr, t_desc, in_64);
        } else {
            t_64 = tcg_const_i64(in_c);
            gen_helper_gvec_dup64(t_ptr, t_desc, t_64);
            tcg_temp_free_i64(t_64);
        }
    } else {
        typedef void dup_fn(TCGv_ptr, TCGv_i32, TCGv_i32);
        static dup_fn * const fns[3] = {
            gen_helper_gvec_dup8,
            gen_helper_gvec_dup16,
            gen_helper_gvec_dup32
        };

        if (in_32) {
            fns[vece](t_ptr, t_desc, in_32);
        } else {
            t_32 = tcg_temp_new_i32();
            if (in_64) {
                tcg_gen_extrl_i64_i32(t_32, in_64);
            } else if (vece == MO_8) {
                tcg_gen_movi_i32(t_32, in_c & 0xff);
            } else if (vece == MO_16) {
                tcg_gen_movi_i32(t_32, in_c & 0xffff);
            } else {
                tcg_gen_movi_i32(t_32, in_c);
            }
            fns[vece](t_ptr, t_desc, t_32);
            tcg_temp_free_i32(t_32);
        }
    }

    tcg_temp_free_ptr(t_ptr);
    tcg_temp_free_i32(t_desc);
    return;

 done:
    if (oprsz < maxsz) {
        expand_clr(dofs + oprsz, maxsz - oprsz);
    }
}

/* Likewise, but with zero.  */
static void expand_clr(uint32_t dofs, uint32_t maxsz)
{
    do_dup(MO_8, dofs, maxsz, maxsz, NULL, NULL, 0);
}

/* Expand OPSZ bytes worth of two-operand operations using i32 elements.  */
static void expand_2_i32(uint32_t dofs, uint32_t aofs, uint32_t oprsz,
                         void (*fni)(TCGv_i32, TCGv_i32))
{
    TCGv_i32 t0 = tcg_temp_new_i32();
    uint32_t i;

    for (i = 0; i < oprsz; i += 4) {
        tcg_gen_ld_i32(t0, cpu_env, aofs + i);
        fni(t0, t0);
        tcg_gen_st_i32(t0, cpu_env, dofs + i);
    }
    tcg_temp_free_i32(t0);
}

/* Expand OPSZ bytes worth of three-operand operations using i32 elements.  */
static void expand_3_i32(uint32_t dofs, uint32_t aofs,
                         uint32_t bofs, uint32_t oprsz, bool load_dest,
                         void (*fni)(TCGv_i32, TCGv_i32, TCGv_i32))
{
    TCGv_i32 t0 = tcg_temp_new_i32();
    TCGv_i32 t1 = tcg_temp_new_i32();
    TCGv_i32 t2 = tcg_temp_new_i32();
    uint32_t i;

    for (i = 0; i < oprsz; i += 4) {
        tcg_gen_ld_i32(t0, cpu_env, aofs + i);
        tcg_gen_ld_i32(t1, cpu_env, bofs + i);
        if (load_dest) {
            tcg_gen_ld_i32(t2, cpu_env, dofs + i);
        }
        fni(t2, t0, t1);
        tcg_gen_st_i32(t2, cpu_env, dofs + i);
    }
    tcg_temp_free_i32(t2);
    tcg_temp_free_i32(t1);
    tcg_temp_free_i32(t0);
}

/* Expand OPSZ bytes worth of three-operand operations using i32 elements.  */
static void expand_4_i32(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                         uint32_t cofs, uint32_t oprsz,
                         void (*fni)(TCGv_i32, TCGv_i32, TCGv_i32, TCGv_i32))
{
    TCGv_i32 t0 = tcg_temp_new_i32();
    TCGv_i32 t1 = tcg_temp_new_i32();
    TCGv_i32 t2 = tcg_temp_new_i32();
    TCGv_i32 t3 = tcg_temp_new_i32();
    uint32_t i;

    for (i = 0; i < oprsz; i += 4) {
        tcg_gen_ld_i32(t1, cpu_env, aofs + i);
        tcg_gen_ld_i32(t2, cpu_env, bofs + i);
        tcg_gen_ld_i32(t3, cpu_env, cofs + i);
        fni(t0, t1, t2, t3);
        tcg_gen_st_i32(t0, cpu_env, dofs + i);
    }
    tcg_temp_free_i32(t3);
    tcg_temp_free_i32(t2);
    tcg_temp_free_i32(t1);
    tcg_temp_free_i32(t0);
}

/* Expand OPSZ bytes worth of two-operand operations using i64 elements.  */
static void expand_2_i64(uint32_t dofs, uint32_t aofs, uint32_t oprsz,
                         void (*fni)(TCGv_i64, TCGv_i64))
{
    TCGv_i64 t0 = tcg_temp_new_i64();
    uint32_t i;

    for (i = 0; i < oprsz; i += 8) {
        tcg_gen_ld_i64(t0, cpu_env, aofs + i);
        fni(t0, t0);
        tcg_gen_st_i64(t0, cpu_env, dofs + i);
    }
    tcg_temp_free_i64(t0);
}

/* Expand OPSZ bytes worth of three-operand operations using i64 elements.  */
static void expand_3_i64(uint32_t dofs, uint32_t aofs,
                         uint32_t bofs, uint32_t oprsz, bool load_dest,
                         void (*fni)(TCGv_i64, TCGv_i64, TCGv_i64))
{
    TCGv_i64 t0 = tcg_temp_new_i64();
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    uint32_t i;

    for (i = 0; i < oprsz; i += 8) {
        tcg_gen_ld_i64(t0, cpu_env, aofs + i);
        tcg_gen_ld_i64(t1, cpu_env, bofs + i);
        if (load_dest) {
            tcg_gen_ld_i64(t2, cpu_env, dofs + i);
        }
        fni(t2, t0, t1);
        tcg_gen_st_i64(t2, cpu_env, dofs + i);
    }
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t0);
}

/* Expand OPSZ bytes worth of three-operand operations using i64 elements.  */
static void expand_4_i64(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                         uint32_t cofs, uint32_t oprsz,
                         void (*fni)(TCGv_i64, TCGv_i64, TCGv_i64, TCGv_i64))
{
    TCGv_i64 t0 = tcg_temp_new_i64();
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();
    uint32_t i;

    for (i = 0; i < oprsz; i += 8) {
        tcg_gen_ld_i64(t1, cpu_env, aofs + i);
        tcg_gen_ld_i64(t2, cpu_env, bofs + i);
        tcg_gen_ld_i64(t3, cpu_env, cofs + i);
        fni(t0, t1, t2, t3);
        tcg_gen_st_i64(t0, cpu_env, dofs + i);
    }
    tcg_temp_free_i64(t3);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t0);
}

/* Expand OPSZ bytes worth of two-operand operations using host vectors.  */
static void expand_2_vec(unsigned vece, uint32_t dofs, uint32_t aofs,
                         uint32_t oprsz, uint32_t tysz, TCGType type,
                         void (*fni)(unsigned, TCGv_vec, TCGv_vec))
{
    TCGv_vec t0 = tcg_temp_new_vec(type);
    uint32_t i;

    for (i = 0; i < oprsz; i += tysz) {
        tcg_gen_ld_vec(t0, cpu_env, aofs + i);
        fni(vece, t0, t0);
        tcg_gen_st_vec(t0, cpu_env, dofs + i);
    }
    tcg_temp_free_vec(t0);
}

/* Expand OPSZ bytes worth of three-operand operations using host vectors.  */
static void expand_3_vec(unsigned vece, uint32_t dofs, uint32_t aofs,
                         uint32_t bofs, uint32_t oprsz,
                         uint32_t tysz, TCGType type, bool load_dest,
                         void (*fni)(unsigned, TCGv_vec, TCGv_vec, TCGv_vec))
{
    TCGv_vec t0 = tcg_temp_new_vec(type);
    TCGv_vec t1 = tcg_temp_new_vec(type);
    TCGv_vec t2 = tcg_temp_new_vec(type);
    uint32_t i;

    for (i = 0; i < oprsz; i += tysz) {
        tcg_gen_ld_vec(t0, cpu_env, aofs + i);
        tcg_gen_ld_vec(t1, cpu_env, bofs + i);
        if (load_dest) {
            tcg_gen_ld_vec(t2, cpu_env, dofs + i);
        }
        fni(vece, t2, t0, t1);
        tcg_gen_st_vec(t2, cpu_env, dofs + i);
    }
    tcg_temp_free_vec(t2);
    tcg_temp_free_vec(t1);
    tcg_temp_free_vec(t0);
}

/* Expand OPSZ bytes worth of four-operand operations using host vectors.  */
static void expand_4_vec(unsigned vece, uint32_t dofs, uint32_t aofs,
                         uint32_t bofs, uint32_t cofs, uint32_t oprsz,
                         uint32_t tysz, TCGType type,
                         void (*fni)(unsigned, TCGv_vec, TCGv_vec,
                                     TCGv_vec, TCGv_vec))
{
    TCGv_vec t0 = tcg_temp_new_vec(type);
    TCGv_vec t1 = tcg_temp_new_vec(type);
    TCGv_vec t2 = tcg_temp_new_vec(type);
    TCGv_vec t3 = tcg_temp_new_vec(type);
    uint32_t i;

    for (i = 0; i < oprsz; i += tysz) {
        tcg_gen_ld_vec(t1, cpu_env, aofs + i);
        tcg_gen_ld_vec(t2, cpu_env, bofs + i);
        tcg_gen_ld_vec(t3, cpu_env, cofs + i);
        fni(vece, t0, t1, t2, t3);
        tcg_gen_st_vec(t0, cpu_env, dofs + i);
    }
    tcg_temp_free_vec(t3);
    tcg_temp_free_vec(t2);
    tcg_temp_free_vec(t1);
    tcg_temp_free_vec(t0);
}

/* Expand a vector two-operand operation.  */
void tcg_gen_gvec_2(uint32_t dofs, uint32_t aofs,
                    uint32_t oprsz, uint32_t maxsz, const GVecGen2 *g)
{
    check_size_align(oprsz, maxsz, dofs | aofs);
    check_overlap_2(dofs, aofs, maxsz);

    /* Recall that ARM SVE allows vector sizes that are not a power of 2.
       Expand with successively smaller host vector sizes.  The intent is
       that e.g. oprsz == 80 would be expanded with 2x32 + 1x16.  */
    /* ??? For maxsz > oprsz, the host may be able to use an opr-sized
       operation, zeroing the balance of the register.  We can then
       use a max-sized store to implement the clearing without an extra
       store operation.  This is true for aarch64 and x86_64 hosts.  */

    if (TCG_TARGET_HAS_v256 && g->fniv && check_size_impl(oprsz, 32)
        && (!g->opc || tcg_can_emit_vec_op(g->opc, TCG_TYPE_V256, g->vece))) {
        uint32_t some = QEMU_ALIGN_DOWN(oprsz, 32);
        expand_2_vec(g->vece, dofs, aofs, some, 32, TCG_TYPE_V256, g->fniv);
        if (some == oprsz) {
            goto done;
        }
        dofs += some;
        aofs += some;
        oprsz -= some;
        maxsz -= some;
    }

    if (TCG_TARGET_HAS_v128 && g->fniv && check_size_impl(oprsz, 16)
        && (!g->opc || tcg_can_emit_vec_op(g->opc, TCG_TYPE_V128, g->vece))) {
        expand_2_vec(g->vece, dofs, aofs, oprsz, 16, TCG_TYPE_V128, g->fniv);
    } else if (TCG_TARGET_HAS_v64 && !g->prefer_i64
               && g->fniv && check_size_impl(oprsz, 8)
               && (!g->opc
                   || tcg_can_emit_vec_op(g->opc, TCG_TYPE_V64, g->vece))) {
        expand_2_vec(g->vece, dofs, aofs, oprsz, 8, TCG_TYPE_V64, g->fniv);
    } else if (g->fni8 && check_size_impl(oprsz, 8)) {
        expand_2_i64(dofs, aofs, oprsz, g->fni8);
    } else if (g->fni4 && check_size_impl(oprsz, 4)) {
        expand_2_i32(dofs, aofs, oprsz, g->fni4);
    } else {
        assert(g->fno != NULL);
        tcg_gen_gvec_2_ool(dofs, aofs, oprsz, maxsz, g->data, g->fno);
        return;
    }

 done:
    if (oprsz < maxsz) {
        expand_clr(dofs + oprsz, maxsz - oprsz);
    }
}

/* Expand a vector three-operand operation.  */
void tcg_gen_gvec_3(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                    uint32_t oprsz, uint32_t maxsz, const GVecGen3 *g)
{
    check_size_align(oprsz, maxsz, dofs | aofs | bofs);
    check_overlap_3(dofs, aofs, bofs, maxsz);

    /* Recall that ARM SVE allows vector sizes that are not a power of 2.
       Expand with successively smaller host vector sizes.  The intent is
       that e.g. oprsz == 80 would be expanded with 2x32 + 1x16.  */

    if (TCG_TARGET_HAS_v256 && g->fniv && check_size_impl(oprsz, 32)
        && (!g->opc || tcg_can_emit_vec_op(g->opc, TCG_TYPE_V256, g->vece))) {
        uint32_t some = QEMU_ALIGN_DOWN(oprsz, 32);
        expand_3_vec(g->vece, dofs, aofs, bofs, some, 32, TCG_TYPE_V256,
                     g->load_dest, g->fniv);
        if (some == oprsz) {
            goto done;
        }
        dofs += some;
        aofs += some;
        bofs += some;
        oprsz -= some;
        maxsz -= some;
    }

    if (TCG_TARGET_HAS_v128 && g->fniv && check_size_impl(oprsz, 16)
        && (!g->opc || tcg_can_emit_vec_op(g->opc, TCG_TYPE_V128, g->vece))) {
        expand_3_vec(g->vece, dofs, aofs, bofs, oprsz, 16, TCG_TYPE_V128,
                     g->load_dest, g->fniv);
    } else if (TCG_TARGET_HAS_v64 && !g->prefer_i64
               && g->fniv && check_size_impl(oprsz, 8)
               && (!g->opc
                   || tcg_can_emit_vec_op(g->opc, TCG_TYPE_V64, g->vece))) {
        expand_3_vec(g->vece, dofs, aofs, bofs, oprsz, 8, TCG_TYPE_V64,
                     g->load_dest, g->fniv);
    } else if (g->fni8 && check_size_impl(oprsz, 8)) {
        expand_3_i64(dofs, aofs, bofs, oprsz, g->load_dest, g->fni8);
    } else if (g->fni4 && check_size_impl(oprsz, 4)) {
        expand_3_i32(dofs, aofs, bofs, oprsz, g->load_dest, g->fni4);
    } else {
        assert(g->fno != NULL);
        tcg_gen_gvec_3_ool(dofs, aofs, bofs, oprsz, maxsz, g->data, g->fno);
    }

 done:
    if (oprsz < maxsz) {
        expand_clr(dofs + oprsz, maxsz - oprsz);
    }
}

/* Expand a vector four-operand operation.  */
void tcg_gen_gvec_4(uint32_t dofs, uint32_t aofs, uint32_t bofs, uint32_t cofs,
                    uint32_t oprsz, uint32_t maxsz, const GVecGen4 *g)
{
    check_size_align(oprsz, maxsz, dofs | aofs | bofs | cofs);
    check_overlap_4(dofs, aofs, bofs, cofs, maxsz);

    /* Recall that ARM SVE allows vector sizes that are not a power of 2.
       Expand with successively smaller host vector sizes.  The intent is
       that e.g. oprsz == 80 would be expanded with 2x32 + 1x16.  */

    if (TCG_TARGET_HAS_v256 && g->fniv && check_size_impl(oprsz, 32)
        && (!g->opc || tcg_can_emit_vec_op(g->opc, TCG_TYPE_V256, g->vece))) {
        uint32_t some = QEMU_ALIGN_DOWN(oprsz, 32);
        expand_4_vec(g->vece, dofs, aofs, bofs, cofs, some,
                     32, TCG_TYPE_V256, g->fniv);
        if (some == oprsz) {
            goto done;
        }
        dofs += some;
        aofs += some;
        bofs += some;
        cofs += some;
        oprsz -= some;
        maxsz -= some;
    }

    if (TCG_TARGET_HAS_v128 && g->fniv && check_size_impl(oprsz, 16)
        && (!g->opc || tcg_can_emit_vec_op(g->opc, TCG_TYPE_V128, g->vece))) {
        expand_4_vec(g->vece, dofs, aofs, bofs, cofs, oprsz,
                     16, TCG_TYPE_V128, g->fniv);
    } else if (TCG_TARGET_HAS_v64 && !g->prefer_i64
               && g->fniv && check_size_impl(oprsz, 8)
                && (!g->opc
                    || tcg_can_emit_vec_op(g->opc, TCG_TYPE_V64, g->vece))) {
        expand_4_vec(g->vece, dofs, aofs, bofs, cofs, oprsz,
                     8, TCG_TYPE_V64, g->fniv);
    } else if (g->fni8 && check_size_impl(oprsz, 8)) {
        expand_4_i64(dofs, aofs, bofs, cofs, oprsz, g->fni8);
    } else if (g->fni4 && check_size_impl(oprsz, 4)) {
        expand_4_i32(dofs, aofs, bofs, cofs, oprsz, g->fni4);
    } else {
        assert(g->fno != NULL);
        tcg_gen_gvec_4_ool(dofs, aofs, bofs, cofs,
                           oprsz, maxsz, g->data, g->fno);
        return;
    }

 done:
    if (oprsz < maxsz) {
        expand_clr(dofs + oprsz, maxsz - oprsz);
    }
}

/*
 * Expand specific vector operations.
 */

static void vec_mov2(unsigned vece, TCGv_vec a, TCGv_vec b)
{
    tcg_gen_mov_vec(a, b);
}

void tcg_gen_gvec_mov(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t oprsz, uint32_t maxsz)
{
    static const GVecGen2 g = {
        .fni8 = tcg_gen_mov_i64,
        .fniv = vec_mov2,
        .fno = gen_helper_gvec_mov,
        .prefer_i64 = TCG_TARGET_REG_BITS == 64,
    };
    if (dofs != aofs) {
        tcg_gen_gvec_2(dofs, aofs, oprsz, maxsz, &g);
    } else {
        check_size_align(oprsz, maxsz, dofs);
        if (oprsz < maxsz) {
            expand_clr(dofs + oprsz, maxsz - oprsz);
        }
    }
}

void tcg_gen_gvec_dup_i32(unsigned vece, uint32_t dofs, uint32_t oprsz,
                          uint32_t maxsz, TCGv_i32 in)
{
    check_size_align(oprsz, maxsz, dofs);
    tcg_debug_assert(vece <= MO_32);
    do_dup(vece, dofs, oprsz, maxsz, in, NULL, 0);
}

void tcg_gen_gvec_dup_i64(unsigned vece, uint32_t dofs, uint32_t oprsz,
                          uint32_t maxsz, TCGv_i64 in)
{
    check_size_align(oprsz, maxsz, dofs);
    tcg_debug_assert(vece <= MO_64);
    do_dup(vece, dofs, oprsz, maxsz, NULL, in, 0);
}

void tcg_gen_gvec_dup_mem(unsigned vece, uint32_t dofs, uint32_t aofs,
                          uint32_t oprsz, uint32_t maxsz)
{
    if (vece <= MO_32) {
        TCGv_i32 in = tcg_temp_new_i32();
        switch (vece) {
        case MO_8:
            tcg_gen_ld8u_i32(in, cpu_env, aofs);
            break;
        case MO_16:
            tcg_gen_ld16u_i32(in, cpu_env, aofs);
            break;
        case MO_32:
            tcg_gen_ld_i32(in, cpu_env, aofs);
            break;
        }
        tcg_gen_gvec_dup_i32(vece, dofs, oprsz, maxsz, in);
        tcg_temp_free_i32(in);
    } else if (vece == MO_64) {
        TCGv_i64 in = tcg_temp_new_i64();
        tcg_gen_ld_i64(in, cpu_env, aofs);
        tcg_gen_gvec_dup_i64(MO_64, dofs, oprsz, maxsz, in);
        tcg_temp_free_i64(in);
    } else {
        /* 128-bit duplicate.  */
        /* ??? Dup to 256-bit vector.  */
        int i;

        tcg_debug_assert(vece == 4);
        tcg_debug_assert(oprsz >= 16);
        if (TCG_TARGET_HAS_v128) {
            TCGv_vec in = tcg_temp_new_vec(TCG_TYPE_V128);

            tcg_gen_ld_vec(in, cpu_env, aofs);
            for (i = 0; i < oprsz; i += 16) {
                tcg_gen_st_vec(in, cpu_env, dofs + i);
            }
            tcg_temp_free_vec(in);
        } else {
            TCGv_i64 in0 = tcg_temp_new_i64();
            TCGv_i64 in1 = tcg_temp_new_i64();

            tcg_gen_ld_i64(in0, cpu_env, aofs);
            tcg_gen_ld_i64(in1, cpu_env, aofs + 8);
            for (i = 0; i < oprsz; i += 16) {
                tcg_gen_st_i64(in0, cpu_env, dofs + i);
                tcg_gen_st_i64(in1, cpu_env, dofs + i + 8);
            }
            tcg_temp_free_i64(in0);
            tcg_temp_free_i64(in1);
        }
    }
}

void tcg_gen_gvec_dup64i(uint32_t dofs, uint32_t oprsz,
                         uint32_t maxsz, uint64_t x)
{
    check_size_align(oprsz, maxsz, dofs);
    do_dup(MO_64, dofs, oprsz, maxsz, NULL, NULL, x);
}

void tcg_gen_gvec_dup32i(uint32_t dofs, uint32_t oprsz,
                         uint32_t maxsz, uint32_t x)
{
    check_size_align(oprsz, maxsz, dofs);
    do_dup(MO_32, dofs, oprsz, maxsz, NULL, NULL, x);
}

void tcg_gen_gvec_dup16i(uint32_t dofs, uint32_t oprsz,
                         uint32_t maxsz, uint16_t x)
{
    check_size_align(oprsz, maxsz, dofs);
    do_dup(MO_16, dofs, oprsz, maxsz, NULL, NULL, x);
}

void tcg_gen_gvec_dup8i(uint32_t dofs, uint32_t oprsz,
                         uint32_t maxsz, uint8_t x)
{
    check_size_align(oprsz, maxsz, dofs);
    do_dup(MO_8, dofs, oprsz, maxsz, NULL, NULL, x);
}

void tcg_gen_gvec_not(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t oprsz, uint32_t maxsz)
{
    static const GVecGen2 g = {
        .fni8 = tcg_gen_not_i64,
        .fniv = tcg_gen_not_vec,
        .fno = gen_helper_gvec_not,
        .prefer_i64 = TCG_TARGET_REG_BITS == 64,
    };
    tcg_gen_gvec_2(dofs, aofs, oprsz, maxsz, &g);
}

/* Perform a vector addition using normal addition and a mask.  The mask
   should be the sign bit of each lane.  This 6-operation form is more
   efficient than separate additions when there are 4 or more lanes in
   the 64-bit operation.  */
static void gen_addv_mask(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b, TCGv_i64 m)
{
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();

    tcg_gen_andc_i64(t1, a, m);
    tcg_gen_andc_i64(t2, b, m);
    tcg_gen_xor_i64(t3, a, b);
    tcg_gen_add_i64(d, t1, t2);
    tcg_gen_and_i64(t3, t3, m);
    tcg_gen_xor_i64(d, d, t3);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
}

void tcg_gen_vec_add8_i64(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b)
{
    TCGv_i64 m = tcg_const_i64(dup_const(MO_8, 0x80));
    gen_addv_mask(d, a, b, m);
    tcg_temp_free_i64(m);
}

void tcg_gen_vec_add16_i64(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b)
{
    TCGv_i64 m = tcg_const_i64(dup_const(MO_16, 0x8000));
    gen_addv_mask(d, a, b, m);
    tcg_temp_free_i64(m);
}

void tcg_gen_vec_add32_i64(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b)
{
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();

    tcg_gen_andi_i64(t1, a, ~0xffffffffull);
    tcg_gen_add_i64(t2, a, b);
    tcg_gen_add_i64(t1, t1, b);
    tcg_gen_deposit_i64(d, t1, t2, 0, 32);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
}

void tcg_gen_gvec_add(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t bofs, uint32_t oprsz, uint32_t maxsz)
{
    static const GVecGen3 g[4] = {
        { .fni8 = tcg_gen_vec_add8_i64,
          .fniv = tcg_gen_add_vec,
          .fno = gen_helper_gvec_add8,
          .opc = INDEX_op_add_vec,
          .vece = MO_8 },
        { .fni8 = tcg_gen_vec_add16_i64,
          .fniv = tcg_gen_add_vec,
          .fno = gen_helper_gvec_add16,
          .opc = INDEX_op_add_vec,
          .vece = MO_16 },
        { .fni4 = tcg_gen_add_i32,
          .fniv = tcg_gen_add_vec,
          .fno = gen_helper_gvec_add32,
          .opc = INDEX_op_add_vec,
          .vece = MO_32 },
        { .fni8 = tcg_gen_add_i64,
          .fniv = tcg_gen_add_vec,
          .fno = gen_helper_gvec_add64,
          .opc = INDEX_op_add_vec,
          .prefer_i64 = TCG_TARGET_REG_BITS == 64,
          .vece = MO_64 },
    };

    tcg_debug_assert(vece <= MO_64);
    tcg_gen_gvec_3(dofs, aofs, bofs, oprsz, maxsz, &g[vece]);
}

/* Perform a vector subtraction using normal subtraction and a mask.
   Compare gen_addv_mask above.  */
static void gen_subv_mask(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b, TCGv_i64 m)
{
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();

    tcg_gen_or_i64(t1, a, m);
    tcg_gen_andc_i64(t2, b, m);
    tcg_gen_eqv_i64(t3, a, b);
    tcg_gen_sub_i64(d, t1, t2);
    tcg_gen_and_i64(t3, t3, m);
    tcg_gen_xor_i64(d, d, t3);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
}

void tcg_gen_vec_sub8_i64(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b)
{
    TCGv_i64 m = tcg_const_i64(dup_const(MO_8, 0x80));
    gen_subv_mask(d, a, b, m);
    tcg_temp_free_i64(m);
}

void tcg_gen_vec_sub16_i64(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b)
{
    TCGv_i64 m = tcg_const_i64(dup_const(MO_16, 0x8000));
    gen_subv_mask(d, a, b, m);
    tcg_temp_free_i64(m);
}

void tcg_gen_vec_sub32_i64(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b)
{
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();

    tcg_gen_andi_i64(t1, b, ~0xffffffffull);
    tcg_gen_sub_i64(t2, a, b);
    tcg_gen_sub_i64(t1, a, t1);
    tcg_gen_deposit_i64(d, t1, t2, 0, 32);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
}

void tcg_gen_gvec_sub(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t bofs, uint32_t oprsz, uint32_t maxsz)
{
    static const GVecGen3 g[4] = {
        { .fni8 = tcg_gen_vec_sub8_i64,
          .fniv = tcg_gen_sub_vec,
          .fno = gen_helper_gvec_sub8,
          .opc = INDEX_op_sub_vec,
          .vece = MO_8 },
        { .fni8 = tcg_gen_vec_sub16_i64,
          .fniv = tcg_gen_sub_vec,
          .fno = gen_helper_gvec_sub16,
          .opc = INDEX_op_sub_vec,
          .vece = MO_16 },
        { .fni4 = tcg_gen_sub_i32,
          .fniv = tcg_gen_sub_vec,
          .fno = gen_helper_gvec_sub32,
          .opc = INDEX_op_sub_vec,
          .vece = MO_32 },
        { .fni8 = tcg_gen_sub_i64,
          .fniv = tcg_gen_sub_vec,
          .fno = gen_helper_gvec_sub64,
          .opc = INDEX_op_sub_vec,
          .prefer_i64 = TCG_TARGET_REG_BITS == 64,
          .vece = MO_64 },
    };

    tcg_debug_assert(vece <= MO_64);
    tcg_gen_gvec_3(dofs, aofs, bofs, oprsz, maxsz, &g[vece]);
}

/* Perform a vector negation using normal negation and a mask.
   Compare gen_subv_mask above.  */
static void gen_negv_mask(TCGv_i64 d, TCGv_i64 b, TCGv_i64 m)
{
    TCGv_i64 t2 = tcg_temp_new_i64();
    TCGv_i64 t3 = tcg_temp_new_i64();

    tcg_gen_andc_i64(t3, m, b);
    tcg_gen_andc_i64(t2, b, m);
    tcg_gen_sub_i64(d, m, t2);
    tcg_gen_xor_i64(d, d, t3);

    tcg_temp_free_i64(t2);
    tcg_temp_free_i64(t3);
}

void tcg_gen_vec_neg8_i64(TCGv_i64 d, TCGv_i64 b)
{
    TCGv_i64 m = tcg_const_i64(dup_const(MO_8, 0x80));
    gen_negv_mask(d, b, m);
    tcg_temp_free_i64(m);
}

void tcg_gen_vec_neg16_i64(TCGv_i64 d, TCGv_i64 b)
{
    TCGv_i64 m = tcg_const_i64(dup_const(MO_16, 0x8000));
    gen_negv_mask(d, b, m);
    tcg_temp_free_i64(m);
}

void tcg_gen_vec_neg32_i64(TCGv_i64 d, TCGv_i64 b)
{
    TCGv_i64 t1 = tcg_temp_new_i64();
    TCGv_i64 t2 = tcg_temp_new_i64();

    tcg_gen_andi_i64(t1, b, ~0xffffffffull);
    tcg_gen_neg_i64(t2, b);
    tcg_gen_neg_i64(t1, t1);
    tcg_gen_deposit_i64(d, t1, t2, 0, 32);

    tcg_temp_free_i64(t1);
    tcg_temp_free_i64(t2);
}

void tcg_gen_gvec_neg(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t oprsz, uint32_t maxsz)
{
    static const GVecGen2 g[4] = {
        { .fni8 = tcg_gen_vec_neg8_i64,
          .fniv = tcg_gen_neg_vec,
          .fno = gen_helper_gvec_neg8,
          .opc = INDEX_op_neg_vec,
          .vece = MO_8 },
        { .fni8 = tcg_gen_vec_neg16_i64,
          .fniv = tcg_gen_neg_vec,
          .fno = gen_helper_gvec_neg16,
          .opc = INDEX_op_neg_vec,
          .vece = MO_16 },
        { .fni4 = tcg_gen_neg_i32,
          .fniv = tcg_gen_neg_vec,
          .fno = gen_helper_gvec_neg32,
          .opc = INDEX_op_neg_vec,
          .vece = MO_32 },
        { .fni8 = tcg_gen_neg_i64,
          .fniv = tcg_gen_neg_vec,
          .fno = gen_helper_gvec_neg64,
          .opc = INDEX_op_neg_vec,
          .prefer_i64 = TCG_TARGET_REG_BITS == 64,
          .vece = MO_64 },
    };

    tcg_debug_assert(vece <= MO_64);
    tcg_gen_gvec_2(dofs, aofs, oprsz, maxsz, &g[vece]);
}

void tcg_gen_gvec_and(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t bofs, uint32_t oprsz, uint32_t maxsz)
{
    static const GVecGen3 g = {
        .fni8 = tcg_gen_and_i64,
        .fniv = tcg_gen_and_vec,
        .fno = gen_helper_gvec_and,
        .opc = INDEX_op_and_vec,
        .prefer_i64 = TCG_TARGET_REG_BITS == 64,
    };
    tcg_gen_gvec_3(dofs, aofs, bofs, oprsz, maxsz, &g);
}

void tcg_gen_gvec_or(unsigned vece, uint32_t dofs, uint32_t aofs,
                     uint32_t bofs, uint32_t oprsz, uint32_t maxsz)
{
    static const GVecGen3 g = {
        .fni8 = tcg_gen_or_i64,
        .fniv = tcg_gen_or_vec,
        .fno = gen_helper_gvec_or,
        .opc = INDEX_op_or_vec,
        .prefer_i64 = TCG_TARGET_REG_BITS == 64,
    };
    tcg_gen_gvec_3(dofs, aofs, bofs, oprsz, maxsz, &g);
}

void tcg_gen_gvec_xor(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t bofs, uint32_t oprsz, uint32_t maxsz)
{
    static const GVecGen3 g = {
        .fni8 = tcg_gen_xor_i64,
        .fniv = tcg_gen_xor_vec,
        .fno = gen_helper_gvec_xor,
        .opc = INDEX_op_xor_vec,
        .prefer_i64 = TCG_TARGET_REG_BITS == 64,
    };
    tcg_gen_gvec_3(dofs, aofs, bofs, oprsz, maxsz, &g);
}

void tcg_gen_gvec_andc(unsigned vece, uint32_t dofs, uint32_t aofs,
                       uint32_t bofs, uint32_t oprsz, uint32_t maxsz)
{
    static const GVecGen3 g = {
        .fni8 = tcg_gen_andc_i64,
        .fniv = tcg_gen_andc_vec,
        .fno = gen_helper_gvec_andc,
        .opc = INDEX_op_andc_vec,
        .prefer_i64 = TCG_TARGET_REG_BITS == 64,
    };
    tcg_gen_gvec_3(dofs, aofs, bofs, oprsz, maxsz, &g);
}

void tcg_gen_gvec_orc(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t bofs, uint32_t oprsz, uint32_t maxsz)
{
    static const GVecGen3 g = {
        .fni8 = tcg_gen_orc_i64,
        .fniv = tcg_gen_orc_vec,
        .fno = gen_helper_gvec_orc,
        .opc = INDEX_op_orc_vec,
        .prefer_i64 = TCG_TARGET_REG_BITS == 64,
    };
    tcg_gen_gvec_3(dofs, aofs, bofs, oprsz, maxsz, &g);
}
