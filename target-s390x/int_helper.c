/*
 *  S/390 integer helper routines
 *
 *  Copyright (c) 2009 Ulrich Hecht
 *  Copyright (c) 2009 Alexander Graf
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
#include "qemu/host-utils.h"
#include "helper.h"

/* #define DEBUG_HELPER */
#ifdef DEBUG_HELPER
#define HELPER_LOG(x...) qemu_log(x)
#else
#define HELPER_LOG(x...)
#endif

/* 64/64 -> 128 unsigned multiplication */
uint64_t HELPER(mul128)(CPUS390XState *env, uint64_t v1, uint64_t v2)
{
    uint64_t reth;
    mulu64(&env->retxl, &reth, v1, v2);
    return reth;
}

/* 64/32 -> 32 signed division */
int64_t HELPER(divs32)(CPUS390XState *env, int64_t a, int64_t b)
{
    env->retxl = a % (int32_t)b;
    return a / (int32_t)b;
}

/* 64/32 -> 32 unsigned division */
uint64_t HELPER(divu32)(CPUS390XState *env, uint64_t a, uint64_t b)
{
    env->retxl = a % (uint32_t)b;
    return a / (uint32_t)b;
}

/* 64/64 -> 64 signed division */
int64_t HELPER(divs64)(CPUS390XState *env, int64_t a, int64_t b)
{
    env->retxl = a % b;
    return a / b;
}

/* 128 -> 64/64 unsigned division */
uint64_t HELPER(divu64)(CPUS390XState *env, uint64_t ah, uint64_t al,
                        uint64_t b)
{
    uint64_t ret;
    if (ah == 0) {
        /* 64 -> 64/64 case */
        env->retxl = al % b;
        ret = al / b;
    } else {
        /* ??? Move i386 idivq helper to host-utils.  */
#if HOST_LONG_BITS == 64 && defined(__GNUC__)
        /* assuming 64-bit hosts have __uint128_t */
        __uint128_t a = ((__uint128_t)ah << 64) | al;
        __uint128_t q = a / b;
        env->retxl = a % b;
        ret = q;
#else
        /* 32-bit hosts would need special wrapper functionality - just abort if
           we encounter such a case; it's very unlikely anyways. */
        cpu_abort(env, "128 -> 64/64 division not implemented\n");
#endif
    }
    return ret;
}

/* absolute value 32-bit */
uint32_t HELPER(abs_i32)(int32_t val)
{
    if (val < 0) {
        return -val;
    } else {
        return val;
    }
}

/* negative absolute value 32-bit */
int32_t HELPER(nabs_i32)(int32_t val)
{
    if (val < 0) {
        return val;
    } else {
        return -val;
    }
}

/* absolute value 64-bit */
uint64_t HELPER(abs_i64)(int64_t val)
{
    HELPER_LOG("%s: val 0x%" PRIx64 "\n", __func__, val);

    if (val < 0) {
        return -val;
    } else {
        return val;
    }
}

/* negative absolute value 64-bit */
int64_t HELPER(nabs_i64)(int64_t val)
{
    if (val < 0) {
        return val;
    } else {
        return -val;
    }
}

/* find leftmost one */
uint32_t HELPER(flogr)(CPUS390XState *env, uint32_t r1, uint64_t v2)
{
    uint64_t res = 0;
    uint64_t ov2 = v2;

    while (!(v2 & 0x8000000000000000ULL) && v2) {
        v2 <<= 1;
        res++;
    }

    if (!v2) {
        env->regs[r1] = 64;
        env->regs[r1 + 1] = 0;
        return 0;
    } else {
        env->regs[r1] = res;
        env->regs[r1 + 1] = ov2 & ~(0x8000000000000000ULL >> res);
        return 2;
    }
}

uint64_t HELPER(cvd)(int32_t bin)
{
    /* positive 0 */
    uint64_t dec = 0x0c;
    int shift = 4;

    if (bin < 0) {
        bin = -bin;
        dec = 0x0d;
    }

    for (shift = 4; (shift < 64) && bin; shift += 4) {
        int current_number = bin % 10;

        dec |= (current_number) << shift;
        bin /= 10;
    }

    return dec;
}
