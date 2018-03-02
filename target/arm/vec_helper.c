/*
 * ARM AdvSIMD / SVE Vector Operations
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
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/helper-proto.h"
#include "tcg/tcg-gvec-desc.h"


#define SET_QC() env->vfp.xregs[ARM_VFP_FPSCR] |= CPSR_Q

static void clear_tail(void *vd, uintptr_t opr_sz, uintptr_t max_sz)
{
    uint64_t *d = vd + opr_sz;
    uintptr_t i;

    for (i = opr_sz; i < max_sz; i += 8) {
        *d++ = 0;
    }
}

/* Signed saturating rounding doubling multiply-accumulate high half, 16-bit */
static uint16_t inl_qrdmlah_s16(CPUARMState *env, int16_t src1,
                                int16_t src2, int16_t src3)
{
    /* Simplify:
     * = ((a3 << 16) + ((e1 * e2) << 1) + (1 << 15)) >> 16
     * = ((a3 << 15) + (e1 * e2) + (1 << 14)) >> 15
     */
    int32_t ret = (int32_t)src1 * src2;
    ret = ((int32_t)src3 << 15) + ret + (1 << 14);
    ret >>= 15;
    if (ret != (int16_t)ret) {
        SET_QC();
        ret = (ret < 0 ? -0x8000 : 0x7fff);
    }
    return ret;
}

uint32_t HELPER(neon_qrdmlah_s16)(CPUARMState *env, uint32_t src1,
                                  uint32_t src2, uint32_t src3)
{
    uint16_t e1 = inl_qrdmlah_s16(env, src1, src2, src3);
    uint16_t e2 = inl_qrdmlah_s16(env, src1 >> 16, src2 >> 16, src3 >> 16);
    return deposit32(e1, 16, 16, e2);
}

void HELPER(gvec_qrdmlah_s16)(void *vd, void *vn, void *vm,
                              void *ve, uint32_t desc)
{
    uintptr_t opr_sz = simd_oprsz(desc);
    int16_t *d = vd;
    int16_t *n = vn;
    int16_t *m = vm;
    CPUARMState *env = ve;
    uintptr_t i;

    for (i = 0; i < opr_sz / 2; ++i) {
        d[i] = inl_qrdmlah_s16(env, n[i], m[i], d[i]);
    }
    clear_tail(d, opr_sz, simd_maxsz(desc));
}

/* Signed saturating rounding doubling multiply-subtract high half, 16-bit */
static uint16_t inl_qrdmlsh_s16(CPUARMState *env, int16_t src1,
                                int16_t src2, int16_t src3)
{
    /* Similarly, using subtraction:
     * = ((a3 << 16) - ((e1 * e2) << 1) + (1 << 15)) >> 16
     * = ((a3 << 15) - (e1 * e2) + (1 << 14)) >> 15
     */
    int32_t ret = (int32_t)src1 * src2;
    ret = ((int32_t)src3 << 15) - ret + (1 << 14);
    ret >>= 15;
    if (ret != (int16_t)ret) {
        SET_QC();
        ret = (ret < 0 ? -0x8000 : 0x7fff);
    }
    return ret;
}

uint32_t HELPER(neon_qrdmlsh_s16)(CPUARMState *env, uint32_t src1,
                                  uint32_t src2, uint32_t src3)
{
    uint16_t e1 = inl_qrdmlsh_s16(env, src1, src2, src3);
    uint16_t e2 = inl_qrdmlsh_s16(env, src1 >> 16, src2 >> 16, src3 >> 16);
    return deposit32(e1, 16, 16, e2);
}

void HELPER(gvec_qrdmlsh_s16)(void *vd, void *vn, void *vm,
                              void *ve, uint32_t desc)
{
    uintptr_t opr_sz = simd_oprsz(desc);
    int16_t *d = vd;
    int16_t *n = vn;
    int16_t *m = vm;
    CPUARMState *env = ve;
    uintptr_t i;

    for (i = 0; i < opr_sz / 2; ++i) {
        d[i] = inl_qrdmlsh_s16(env, n[i], m[i], d[i]);
    }
    clear_tail(d, opr_sz, simd_maxsz(desc));
}

/* Signed saturating rounding doubling multiply-accumulate high half, 32-bit */
uint32_t HELPER(neon_qrdmlah_s32)(CPUARMState *env, int32_t src1,
                                  int32_t src2, int32_t src3)
{
    /* Simplify similarly to int_qrdmlah_s16 above.  */
    int64_t ret = (int64_t)src1 * src2;
    ret = ((int64_t)src3 << 31) + ret + (1 << 30);
    ret >>= 31;
    if (ret != (int32_t)ret) {
        SET_QC();
        ret = (ret < 0 ? INT32_MIN : INT32_MAX);
    }
    return ret;
}

void HELPER(gvec_qrdmlah_s32)(void *vd, void *vn, void *vm,
                              void *ve, uint32_t desc)
{
    uintptr_t opr_sz = simd_oprsz(desc);
    int32_t *d = vd;
    int32_t *n = vn;
    int32_t *m = vm;
    CPUARMState *env = ve;
    uintptr_t i;

    for (i = 0; i < opr_sz / 4; ++i) {
        d[i] = helper_neon_qrdmlah_s32(env, n[i], m[i], d[i]);
    }
    clear_tail(d, opr_sz, simd_maxsz(desc));
}

/* Signed saturating rounding doubling multiply-subtract high half, 32-bit */
uint32_t HELPER(neon_qrdmlsh_s32)(CPUARMState *env, int32_t src1,
                                  int32_t src2, int32_t src3)
{
    /* Simplify similarly to int_qrdmlsh_s16 above.  */
    int64_t ret = (int64_t)src1 * src2;
    ret = ((int64_t)src3 << 31) - ret + (1 << 30);
    ret >>= 31;
    if (ret != (int32_t)ret) {
        SET_QC();
        ret = (ret < 0 ? INT32_MIN : INT32_MAX);
    }
    return ret;
}

void HELPER(gvec_qrdmlsh_s32)(void *vd, void *vn, void *vm,
                              void *ve, uint32_t desc)
{
    uintptr_t opr_sz = simd_oprsz(desc);
    int32_t *d = vd;
    int32_t *n = vn;
    int32_t *m = vm;
    CPUARMState *env = ve;
    uintptr_t i;

    for (i = 0; i < opr_sz / 4; ++i) {
        d[i] = helper_neon_qrdmlsh_s32(env, n[i], m[i], d[i]);
    }
    clear_tail(d, opr_sz, simd_maxsz(desc));
}
