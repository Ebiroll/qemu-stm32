/*
 *  S/390 FPU helper routines
 *
 *  Copyright (c) 2009 Ulrich Hecht
 *  Copyright (c) 2009 Alexander Graf
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
#include "cpu.h"
#include "internal.h"
#include "tcg_s390x.h"
#include "exec/exec-all.h"
#include "exec/cpu_ldst.h"
#include "exec/helper-proto.h"
#include "fpu/softfloat.h"

/* #define DEBUG_HELPER */
#ifdef DEBUG_HELPER
#define HELPER_LOG(x...) qemu_log(x)
#else
#define HELPER_LOG(x...)
#endif

#define RET128(F) (env->retxl = F.low, F.high)

uint8_t s390_softfloat_exc_to_ieee(unsigned int exc)
{
    uint8_t s390_exc = 0;

    s390_exc |= (exc & float_flag_invalid) ? S390_IEEE_MASK_INVALID : 0;
    s390_exc |= (exc & float_flag_divbyzero) ? S390_IEEE_MASK_DIVBYZERO : 0;
    s390_exc |= (exc & float_flag_overflow) ? S390_IEEE_MASK_OVERFLOW : 0;
    s390_exc |= (exc & float_flag_underflow) ? S390_IEEE_MASK_UNDERFLOW : 0;
    s390_exc |= (exc & float_flag_inexact) ? S390_IEEE_MASK_INEXACT : 0;

    return s390_exc;
}

/* Should be called after any operation that may raise IEEE exceptions.  */
static void handle_exceptions(CPUS390XState *env, uintptr_t retaddr)
{
    unsigned s390_exc, qemu_exc;

    /* Get the exceptions raised by the current operation.  Reset the
       fpu_status contents so that the next operation has a clean slate.  */
    qemu_exc = env->fpu_status.float_exception_flags;
    if (qemu_exc == 0) {
        return;
    }
    env->fpu_status.float_exception_flags = 0;
    s390_exc = s390_softfloat_exc_to_ieee(qemu_exc);

    /*
     * IEEE-Underflow exception recognition exists if a tininess condition
     * (underflow) exists and
     * - The mask bit in the FPC is zero and the result is inexact
     * - The mask bit in the FPC is one
     * So tininess conditions that are not inexact don't trigger any
     * underflow action in case the mask bit is not one.
     */
    if (!(s390_exc & S390_IEEE_MASK_INEXACT) &&
        !((env->fpc >> 24) & S390_IEEE_MASK_UNDERFLOW)) {
        s390_exc &= ~S390_IEEE_MASK_UNDERFLOW;
    }

    /*
     * FIXME:
     * 1. Right now, all inexact conditions are inidicated as
     *    "truncated" (0) and never as "incremented" (1) in the DXC.
     * 2. Only traps due to invalid/divbyzero are suppressing. Other traps
     *    are completing, meaning the target register has to be written!
     *    This, however will mean that we have to write the register before
     *    triggering the trap - impossible right now.
     */

    /*
     * invalid/divbyzero cannot coexist with other conditions.
     * overflow/underflow however can coexist with inexact, we have to
     * handle it separatly.
     */
    if (s390_exc & ~S390_IEEE_MASK_INEXACT) {
        if (s390_exc & ~S390_IEEE_MASK_INEXACT & env->fpc >> 24) {
            /* trap condition - inexact reported along */
            tcg_s390_data_exception(env, s390_exc, retaddr);
        }
        /* nontrap condition - inexact handled differently */
        env->fpc |= (s390_exc & ~S390_IEEE_MASK_INEXACT) << 16;
    }

    /* inexact handling */
    if (s390_exc & S390_IEEE_MASK_INEXACT) {
        /* trap condition - overflow/underflow _not_ reported along */
        if (s390_exc & S390_IEEE_MASK_INEXACT & env->fpc >> 24) {
            tcg_s390_data_exception(env, s390_exc & S390_IEEE_MASK_INEXACT,
                                    retaddr);
        }
        /* nontrap condition */
        env->fpc |= (s390_exc & S390_IEEE_MASK_INEXACT) << 16;
    }
}

static inline int float_comp_to_cc(CPUS390XState *env, int float_compare)
{
    S390CPU *cpu = s390_env_get_cpu(env);

    switch (float_compare) {
    case float_relation_equal:
        return 0;
    case float_relation_less:
        return 1;
    case float_relation_greater:
        return 2;
    case float_relation_unordered:
        return 3;
    default:
        cpu_abort(CPU(cpu), "unknown return value for float compare\n");
    }
}

/* condition codes for unary FP ops */
uint32_t set_cc_nz_f32(float32 v)
{
    if (float32_is_any_nan(v)) {
        return 3;
    } else if (float32_is_zero(v)) {
        return 0;
    } else if (float32_is_neg(v)) {
        return 1;
    } else {
        return 2;
    }
}

uint32_t set_cc_nz_f64(float64 v)
{
    if (float64_is_any_nan(v)) {
        return 3;
    } else if (float64_is_zero(v)) {
        return 0;
    } else if (float64_is_neg(v)) {
        return 1;
    } else {
        return 2;
    }
}

uint32_t set_cc_nz_f128(float128 v)
{
    if (float128_is_any_nan(v)) {
        return 3;
    } else if (float128_is_zero(v)) {
        return 0;
    } else if (float128_is_neg(v)) {
        return 1;
    } else {
        return 2;
    }
}

/* 32-bit FP addition */
uint64_t HELPER(aeb)(CPUS390XState *env, uint64_t f1, uint64_t f2)
{
    float32 ret = float32_add(f1, f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* 64-bit FP addition */
uint64_t HELPER(adb)(CPUS390XState *env, uint64_t f1, uint64_t f2)
{
    float64 ret = float64_add(f1, f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* 128-bit FP addition */
uint64_t HELPER(axb)(CPUS390XState *env, uint64_t ah, uint64_t al,
                     uint64_t bh, uint64_t bl)
{
    float128 ret = float128_add(make_float128(ah, al),
                                make_float128(bh, bl),
                                &env->fpu_status);
    handle_exceptions(env, GETPC());
    return RET128(ret);
}

/* 32-bit FP subtraction */
uint64_t HELPER(seb)(CPUS390XState *env, uint64_t f1, uint64_t f2)
{
    float32 ret = float32_sub(f1, f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* 64-bit FP subtraction */
uint64_t HELPER(sdb)(CPUS390XState *env, uint64_t f1, uint64_t f2)
{
    float64 ret = float64_sub(f1, f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* 128-bit FP subtraction */
uint64_t HELPER(sxb)(CPUS390XState *env, uint64_t ah, uint64_t al,
                     uint64_t bh, uint64_t bl)
{
    float128 ret = float128_sub(make_float128(ah, al),
                                make_float128(bh, bl),
                                &env->fpu_status);
    handle_exceptions(env, GETPC());
    return RET128(ret);
}

/* 32-bit FP division */
uint64_t HELPER(deb)(CPUS390XState *env, uint64_t f1, uint64_t f2)
{
    float32 ret = float32_div(f1, f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* 64-bit FP division */
uint64_t HELPER(ddb)(CPUS390XState *env, uint64_t f1, uint64_t f2)
{
    float64 ret = float64_div(f1, f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* 128-bit FP division */
uint64_t HELPER(dxb)(CPUS390XState *env, uint64_t ah, uint64_t al,
                     uint64_t bh, uint64_t bl)
{
    float128 ret = float128_div(make_float128(ah, al),
                                make_float128(bh, bl),
                                &env->fpu_status);
    handle_exceptions(env, GETPC());
    return RET128(ret);
}

/* 32-bit FP multiplication */
uint64_t HELPER(meeb)(CPUS390XState *env, uint64_t f1, uint64_t f2)
{
    float32 ret = float32_mul(f1, f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* 64-bit FP multiplication */
uint64_t HELPER(mdb)(CPUS390XState *env, uint64_t f1, uint64_t f2)
{
    float64 ret = float64_mul(f1, f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* 64/32-bit FP multiplication */
uint64_t HELPER(mdeb)(CPUS390XState *env, uint64_t f1, uint64_t f2)
{
    float64 ret = float32_to_float64(f2, &env->fpu_status);
    ret = float64_mul(f1, ret, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* 128-bit FP multiplication */
uint64_t HELPER(mxb)(CPUS390XState *env, uint64_t ah, uint64_t al,
                     uint64_t bh, uint64_t bl)
{
    float128 ret = float128_mul(make_float128(ah, al),
                                make_float128(bh, bl),
                                &env->fpu_status);
    handle_exceptions(env, GETPC());
    return RET128(ret);
}

/* 128/64-bit FP multiplication */
uint64_t HELPER(mxdb)(CPUS390XState *env, uint64_t ah, uint64_t al,
                      uint64_t f2)
{
    float128 ret = float64_to_float128(f2, &env->fpu_status);
    ret = float128_mul(make_float128(ah, al), ret, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return RET128(ret);
}

/* convert 32-bit float to 64-bit float */
uint64_t HELPER(ldeb)(CPUS390XState *env, uint64_t f2)
{
    float64 ret = float32_to_float64(f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 128-bit float to 64-bit float */
uint64_t HELPER(ldxb)(CPUS390XState *env, uint64_t ah, uint64_t al)
{
    float64 ret = float128_to_float64(make_float128(ah, al), &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 64-bit float to 128-bit float */
uint64_t HELPER(lxdb)(CPUS390XState *env, uint64_t f2)
{
    float128 ret = float64_to_float128(f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return RET128(ret);
}

/* convert 32-bit float to 128-bit float */
uint64_t HELPER(lxeb)(CPUS390XState *env, uint64_t f2)
{
    float128 ret = float32_to_float128(f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return RET128(ret);
}

/* convert 64-bit float to 32-bit float */
uint64_t HELPER(ledb)(CPUS390XState *env, uint64_t f2)
{
    float32 ret = float64_to_float32(f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 128-bit float to 32-bit float */
uint64_t HELPER(lexb)(CPUS390XState *env, uint64_t ah, uint64_t al)
{
    float32 ret = float128_to_float32(make_float128(ah, al), &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* 32-bit FP compare */
uint32_t HELPER(ceb)(CPUS390XState *env, uint64_t f1, uint64_t f2)
{
    int cmp = float32_compare_quiet(f1, f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return float_comp_to_cc(env, cmp);
}

/* 64-bit FP compare */
uint32_t HELPER(cdb)(CPUS390XState *env, uint64_t f1, uint64_t f2)
{
    int cmp = float64_compare_quiet(f1, f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return float_comp_to_cc(env, cmp);
}

/* 128-bit FP compare */
uint32_t HELPER(cxb)(CPUS390XState *env, uint64_t ah, uint64_t al,
                     uint64_t bh, uint64_t bl)
{
    int cmp = float128_compare_quiet(make_float128(ah, al),
                                     make_float128(bh, bl),
                                     &env->fpu_status);
    handle_exceptions(env, GETPC());
    return float_comp_to_cc(env, cmp);
}

int s390_swap_bfp_rounding_mode(CPUS390XState *env, int m3)
{
    int ret = env->fpu_status.float_rounding_mode;
    switch (m3) {
    case 0:
        /* current mode */
        break;
    case 1:
        /* biased round no nearest */
    case 4:
        /* round to nearest */
        set_float_rounding_mode(float_round_nearest_even, &env->fpu_status);
        break;
    case 5:
        /* round to zero */
        set_float_rounding_mode(float_round_to_zero, &env->fpu_status);
        break;
    case 6:
        /* round to +inf */
        set_float_rounding_mode(float_round_up, &env->fpu_status);
        break;
    case 7:
        /* round to -inf */
        set_float_rounding_mode(float_round_down, &env->fpu_status);
        break;
    }
    return ret;
}

void s390_restore_bfp_rounding_mode(CPUS390XState *env, int old_mode)
{
    set_float_rounding_mode(old_mode, &env->fpu_status);
}

/* convert 64-bit int to 32-bit float */
uint64_t HELPER(cegb)(CPUS390XState *env, int64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    float32 ret = int64_to_float32(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 64-bit int to 64-bit float */
uint64_t HELPER(cdgb)(CPUS390XState *env, int64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    float64 ret = int64_to_float64(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 64-bit int to 128-bit float */
uint64_t HELPER(cxgb)(CPUS390XState *env, int64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    float128 ret = int64_to_float128(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return RET128(ret);
}

/* convert 64-bit uint to 32-bit float */
uint64_t HELPER(celgb)(CPUS390XState *env, uint64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    float32 ret = uint64_to_float32(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 64-bit uint to 64-bit float */
uint64_t HELPER(cdlgb)(CPUS390XState *env, uint64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    float64 ret = uint64_to_float64(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 64-bit uint to 128-bit float */
uint64_t HELPER(cxlgb)(CPUS390XState *env, uint64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    float128 ret = uint64_to_float128(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return RET128(ret);
}

/* convert 32-bit float to 64-bit int */
uint64_t HELPER(cgeb)(CPUS390XState *env, uint64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    int64_t ret = float32_to_int64(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 64-bit float to 64-bit int */
uint64_t HELPER(cgdb)(CPUS390XState *env, uint64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    int64_t ret = float64_to_int64(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 128-bit float to 64-bit int */
uint64_t HELPER(cgxb)(CPUS390XState *env, uint64_t h, uint64_t l, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    float128 v2 = make_float128(h, l);
    int64_t ret = float128_to_int64(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 32-bit float to 32-bit int */
uint64_t HELPER(cfeb)(CPUS390XState *env, uint64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    int32_t ret = float32_to_int32(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 64-bit float to 32-bit int */
uint64_t HELPER(cfdb)(CPUS390XState *env, uint64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    int32_t ret = float64_to_int32(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 128-bit float to 32-bit int */
uint64_t HELPER(cfxb)(CPUS390XState *env, uint64_t h, uint64_t l, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    float128 v2 = make_float128(h, l);
    int32_t ret = float128_to_int32(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 32-bit float to 64-bit uint */
uint64_t HELPER(clgeb)(CPUS390XState *env, uint64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    uint64_t ret;

    v2 = float32_to_float64(v2, &env->fpu_status);
    ret = float64_to_uint64(v2, &env->fpu_status);
    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 64-bit float to 64-bit uint */
uint64_t HELPER(clgdb)(CPUS390XState *env, uint64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    uint64_t ret = float64_to_uint64(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 128-bit float to 64-bit uint */
uint64_t HELPER(clgxb)(CPUS390XState *env, uint64_t h, uint64_t l, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    uint64_t ret = float128_to_uint64(make_float128(h, l), &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 32-bit float to 32-bit uint */
uint64_t HELPER(clfeb)(CPUS390XState *env, uint64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    uint32_t ret = float32_to_uint32(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 64-bit float to 32-bit uint */
uint64_t HELPER(clfdb)(CPUS390XState *env, uint64_t v2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    uint32_t ret = float64_to_uint32(v2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* convert 128-bit float to 32-bit uint */
uint64_t HELPER(clfxb)(CPUS390XState *env, uint64_t h, uint64_t l, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    uint32_t ret = float128_to_uint32(make_float128(h, l), &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* round to integer 32-bit */
uint64_t HELPER(fieb)(CPUS390XState *env, uint64_t f2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    float32 ret = float32_round_to_int(f2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* round to integer 64-bit */
uint64_t HELPER(fidb)(CPUS390XState *env, uint64_t f2, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);
    float64 ret = float64_round_to_int(f2, &env->fpu_status);

    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return ret;
}

/* round to integer 128-bit */
uint64_t HELPER(fixb)(CPUS390XState *env, uint64_t ah, uint64_t al, uint32_t m3)
{
    int old_mode = s390_swap_bfp_rounding_mode(env, m3);

    float128 ret = float128_round_to_int(make_float128(ah, al),
                                         &env->fpu_status);
    s390_restore_bfp_rounding_mode(env, old_mode);
    handle_exceptions(env, GETPC());
    return RET128(ret);
}

/* 32-bit FP compare and signal */
uint32_t HELPER(keb)(CPUS390XState *env, uint64_t f1, uint64_t f2)
{
    int cmp = float32_compare(f1, f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return float_comp_to_cc(env, cmp);
}

/* 64-bit FP compare and signal */
uint32_t HELPER(kdb)(CPUS390XState *env, uint64_t f1, uint64_t f2)
{
    int cmp = float64_compare(f1, f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return float_comp_to_cc(env, cmp);
}

/* 128-bit FP compare and signal */
uint32_t HELPER(kxb)(CPUS390XState *env, uint64_t ah, uint64_t al,
                     uint64_t bh, uint64_t bl)
{
    int cmp = float128_compare(make_float128(ah, al),
                               make_float128(bh, bl),
                               &env->fpu_status);
    handle_exceptions(env, GETPC());
    return float_comp_to_cc(env, cmp);
}

/* 32-bit FP multiply and add */
uint64_t HELPER(maeb)(CPUS390XState *env, uint64_t f1,
                      uint64_t f2, uint64_t f3)
{
    float32 ret = float32_muladd(f2, f3, f1, 0, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* 64-bit FP multiply and add */
uint64_t HELPER(madb)(CPUS390XState *env, uint64_t f1,
                      uint64_t f2, uint64_t f3)
{
    float64 ret = float64_muladd(f2, f3, f1, 0, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* 32-bit FP multiply and subtract */
uint64_t HELPER(mseb)(CPUS390XState *env, uint64_t f1,
                      uint64_t f2, uint64_t f3)
{
    float32 ret = float32_muladd(f2, f3, f1, float_muladd_negate_c,
                                 &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* 64-bit FP multiply and subtract */
uint64_t HELPER(msdb)(CPUS390XState *env, uint64_t f1,
                      uint64_t f2, uint64_t f3)
{
    float64 ret = float64_muladd(f2, f3, f1, float_muladd_negate_c,
                                 &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* The rightmost bit has the number 11. */
static inline uint16_t dcmask(int bit, bool neg)
{
    return 1 << (11 - bit - neg);
}

#define DEF_FLOAT_DCMASK(_TYPE) \
static uint16_t _TYPE##_dcmask(CPUS390XState *env, _TYPE f1)       \
{                                                                  \
    const bool neg = _TYPE##_is_neg(f1);                           \
                                                                   \
    /* Sorted by most common cases - only one class is possible */ \
    if (_TYPE##_is_normal(f1)) {                                   \
        return dcmask(2, neg);                                     \
    } else if (_TYPE##_is_zero(f1)) {                              \
        return dcmask(0, neg);                                     \
    } else if (_TYPE##_is_denormal(f1)) {                          \
        return dcmask(4, neg);                                     \
    } else if (_TYPE##_is_infinity(f1)) {                          \
        return dcmask(6, neg);                                     \
    } else if (_TYPE##_is_quiet_nan(f1, &env->fpu_status)) {       \
        return dcmask(8, neg);                                     \
    }                                                              \
    /* signaling nan, as last remaining case */                    \
    return dcmask(10, neg);                                        \
}
DEF_FLOAT_DCMASK(float32)
DEF_FLOAT_DCMASK(float64)
DEF_FLOAT_DCMASK(float128)

/* test data class 32-bit */
uint32_t HELPER(tceb)(CPUS390XState *env, uint64_t f1, uint64_t m2)
{
    return (m2 & float32_dcmask(env, f1)) != 0;
}

/* test data class 64-bit */
uint32_t HELPER(tcdb)(CPUS390XState *env, uint64_t v1, uint64_t m2)
{
    return (m2 & float64_dcmask(env, v1)) != 0;
}

/* test data class 128-bit */
uint32_t HELPER(tcxb)(CPUS390XState *env, uint64_t ah, uint64_t al, uint64_t m2)
{
    return (m2 & float128_dcmask(env, make_float128(ah, al))) != 0;
}

/* square root 32-bit */
uint64_t HELPER(sqeb)(CPUS390XState *env, uint64_t f2)
{
    float32 ret = float32_sqrt(f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* square root 64-bit */
uint64_t HELPER(sqdb)(CPUS390XState *env, uint64_t f2)
{
    float64 ret = float64_sqrt(f2, &env->fpu_status);
    handle_exceptions(env, GETPC());
    return ret;
}

/* square root 128-bit */
uint64_t HELPER(sqxb)(CPUS390XState *env, uint64_t ah, uint64_t al)
{
    float128 ret = float128_sqrt(make_float128(ah, al), &env->fpu_status);
    handle_exceptions(env, GETPC());
    return RET128(ret);
}

static const int fpc_to_rnd[8] = {
    float_round_nearest_even,
    float_round_to_zero,
    float_round_up,
    float_round_down,
    -1,
    -1,
    -1,
    float_round_to_odd,
};

/* set fpc */
void HELPER(sfpc)(CPUS390XState *env, uint64_t fpc)
{
    if (fpc_to_rnd[fpc & 0x7] == -1 || fpc & 0x03030088u ||
        (!s390_has_feat(S390_FEAT_FLOATING_POINT_EXT) && fpc & 0x4)) {
        s390_program_interrupt(env, PGM_SPECIFICATION, ILEN_AUTO, GETPC());
    }

    /* Install everything in the main FPC.  */
    env->fpc = fpc;

    /* Install the rounding mode in the shadow fpu_status.  */
    set_float_rounding_mode(fpc_to_rnd[fpc & 0x7], &env->fpu_status);
}

/* set fpc and signal */
void HELPER(sfas)(CPUS390XState *env, uint64_t fpc)
{
    uint32_t signalling = env->fpc;
    uint32_t s390_exc;

    if (fpc_to_rnd[fpc & 0x7] == -1 || fpc & 0x03030088u ||
        (!s390_has_feat(S390_FEAT_FLOATING_POINT_EXT) && fpc & 0x4)) {
        s390_program_interrupt(env, PGM_SPECIFICATION, ILEN_AUTO, GETPC());
    }

    /*
     * FPC is set to the FPC operand with a bitwise OR of the signalling
     * flags.
     */
    env->fpc = fpc | (signalling & 0x00ff0000);
    set_float_rounding_mode(fpc_to_rnd[fpc & 0x7], &env->fpu_status);

    /*
     * If any signaling flag is enabled in the new FPC mask, a
     * simulated-iee-exception exception occurs.
     */
    s390_exc = (signalling >> 16) & (fpc >> 24);
    if (s390_exc) {
        if (s390_exc & S390_IEEE_MASK_INVALID) {
            s390_exc = S390_IEEE_MASK_INVALID;
        } else if (s390_exc & S390_IEEE_MASK_DIVBYZERO) {
            s390_exc = S390_IEEE_MASK_DIVBYZERO;
        } else if (s390_exc & S390_IEEE_MASK_OVERFLOW) {
            s390_exc &= (S390_IEEE_MASK_OVERFLOW | S390_IEEE_MASK_INEXACT);
        } else if (s390_exc & S390_IEEE_MASK_UNDERFLOW) {
            s390_exc &= (S390_IEEE_MASK_UNDERFLOW | S390_IEEE_MASK_INEXACT);
        } else if (s390_exc & S390_IEEE_MASK_INEXACT) {
            s390_exc = S390_IEEE_MASK_INEXACT;
        } else if (s390_exc & S390_IEEE_MASK_QUANTUM) {
            s390_exc = S390_IEEE_MASK_QUANTUM;
        }
        tcg_s390_data_exception(env, s390_exc | 3, GETPC());
    }
}

/* set bfp rounding mode */
void HELPER(srnm)(CPUS390XState *env, uint64_t rnd)
{
    if (rnd > 0x7 || fpc_to_rnd[rnd & 0x7] == -1) {
        s390_program_interrupt(env, PGM_SPECIFICATION, ILEN_AUTO, GETPC());
    }

    env->fpc = deposit32(env->fpc, 0, 3, rnd);
    set_float_rounding_mode(fpc_to_rnd[rnd & 0x7], &env->fpu_status);
}
