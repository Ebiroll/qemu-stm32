/*
 * QEMU AVR CPU helpers
 *
 * Copyright (c) 2016-2020 Michael Rolnik
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
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/helper-proto.h"

bool avr_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    bool ret = false;
    CPUClass *cc = CPU_GET_CLASS(cs);
    AVRCPU *cpu = AVR_CPU(cs);
    CPUAVRState *env = &cpu->env;

    if (interrupt_request & CPU_INTERRUPT_RESET) {
        if (cpu_interrupts_enabled(env)) {
            cs->exception_index = EXCP_RESET;
            cc->do_interrupt(cs);

            cs->interrupt_request &= ~CPU_INTERRUPT_RESET;

            ret = true;
        }
    }
    if (interrupt_request & CPU_INTERRUPT_HARD) {
        if (cpu_interrupts_enabled(env) && env->intsrc != 0) {
            int index = ctz32(env->intsrc);
            cs->exception_index = EXCP_INT(index);
            cc->do_interrupt(cs);

            env->intsrc &= env->intsrc - 1; /* clear the interrupt */
            cs->interrupt_request &= ~CPU_INTERRUPT_HARD;

            ret = true;
        }
    }
    return ret;
}

void avr_cpu_do_interrupt(CPUState *cs)
{
    AVRCPU *cpu = AVR_CPU(cs);
    CPUAVRState *env = &cpu->env;

    uint32_t ret = env->pc_w;
    int vector = 0;
    int size = avr_feature(env, AVR_FEATURE_JMP_CALL) ? 2 : 1;
    int base = 0;

    if (cs->exception_index == EXCP_RESET) {
        vector = 0;
    } else if (env->intsrc != 0) {
        vector = ctz32(env->intsrc) + 1;
    }

    if (avr_feature(env, AVR_FEATURE_3_BYTE_PC)) {
        cpu_stb_data(env, env->sp--, (ret & 0x0000ff));
        cpu_stb_data(env, env->sp--, (ret & 0x00ff00) >> 8);
        cpu_stb_data(env, env->sp--, (ret & 0xff0000) >> 16);
    } else if (avr_feature(env, AVR_FEATURE_2_BYTE_PC)) {
        cpu_stb_data(env, env->sp--, (ret & 0x0000ff));
        cpu_stb_data(env, env->sp--, (ret & 0x00ff00) >> 8);
    } else {
        cpu_stb_data(env, env->sp--, (ret & 0x0000ff));
    }

    env->pc_w = base + vector * size;
    env->sregI = 0; /* clear Global Interrupt Flag */

    cs->exception_index = -1;
}

int avr_cpu_memory_rw_debug(CPUState *cs, vaddr addr, uint8_t *buf,
                            int len, bool is_write)
{
    return cpu_memory_rw_debug(cs, addr, buf, len, is_write);
}

hwaddr avr_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    return addr; /* I assume 1:1 address correspondance */
}

bool avr_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                      MMUAccessType access_type, int mmu_idx,
                      bool probe, uintptr_t retaddr)
{
    int prot = 0;
    MemTxAttrs attrs = {};
    uint32_t paddr;

    address &= TARGET_PAGE_MASK;

    if (mmu_idx == MMU_CODE_IDX) {
        /* access to code in flash */
        paddr = OFFSET_CODE + address;
        prot = PAGE_READ | PAGE_EXEC;
        if (paddr + TARGET_PAGE_SIZE > OFFSET_DATA) {
            error_report("execution left flash memory");
            abort();
        }
    } else if (address < NUMBER_OF_CPU_REGISTERS + NUMBER_OF_IO_REGISTERS) {
        /*
         * access to CPU registers, exit and rebuilt this TB to use full access
         * incase it touches specially handled registers like SREG or SP
         */
        AVRCPU *cpu = AVR_CPU(cs);
        CPUAVRState *env = &cpu->env;
        env->fullacc = 1;
        cpu_loop_exit_restore(cs, retaddr);
    } else {
        /* access to memory. nothing special */
        paddr = OFFSET_DATA + address;
        prot = PAGE_READ | PAGE_WRITE;
    }

    tlb_set_page_with_attrs(cs, address, paddr, attrs, prot,
                            mmu_idx, TARGET_PAGE_SIZE);

    return true;
}
