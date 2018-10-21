/*
 * Test R5900-specific three-operand MULTU.
 */

#include <stdio.h>
#include <inttypes.h>
#include <assert.h>

static uint64_t multu(uint32_t rs, uint32_t rt)
{
    uint32_t rd, lo, hi;
    uint64_t r;

    __asm__ __volatile__ (
            "    multu %0, %3, %4\n"
            "    mflo %1\n"
            "    mfhi %2\n"
            : "=r" (rd), "=r" (lo), "=r" (hi)
            : "r" (rs), "r" (rt));
    r = ((uint64_t)hi << 32) | (uint32_t)lo;

    assert((uint64_t)rs * rt == r);
    assert(rd == lo);

    return r;
}

int main()
{
    assert(multu(17, 19) == 323);
    assert(multu(77773, 99991) == 7776600043);
    assert(multu(12207031, 305175781) == 3725290219116211);

    assert(multu(0x80000000U, 0x7FFFFFFF) == 0x3FFFFFFF80000000);
    assert(multu(0x80000000U, 0x80000000U) ==  0x4000000000000000);
    assert(multu(0xFFFFFFFFU, 0xFFFFFFFFU) ==  0xFFFFFFFE00000001U);

    return 0;
}
