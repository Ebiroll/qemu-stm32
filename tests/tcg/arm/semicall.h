/*
 * Semihosting Tests - ARM Helper
 *
 * Copyright (c) 2019
 * Written by Alex Bennée <alex.bennee@linaro.org>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

uintptr_t __semi_call(uintptr_t type, uintptr_t arg0)
{
    register uintptr_t t asm("r0") = type;
    register uintptr_t a0 asm("r1") = arg0;
#ifdef __thumb__
#  define SVC  "svc 0xab"
#else
#  define SVC  "svc 0x123456"
#endif
    asm(SVC : "=r" (t)
        : "r" (t), "r" (a0));
    return t;
}
