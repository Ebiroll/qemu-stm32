/*
 * QEMU TCG Multi Threaded vCPUs implementation
 *
 * Copyright (c) 2003-2008 Fabrice Bellard
 * Copyright (c) 2014 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "sysemu/tcg.h"
#include "sysemu/replay.h"
#include "qemu/main-loop.h"
#include "qemu/guest-random.h"
#include "exec/exec-all.h"
#include "hw/boards.h"

#include "tcg-cpus.h"
#include "tcg-cpus-mttcg.h"

/*
 * In the multi-threaded case each vCPU has its own thread. The TLS
 * variable current_cpu can be used deep in the code to find the
 * current CPUState for a given thread.
 */

void *tcg_cpu_thread_fn(void *arg)
{
    CPUState *cpu = arg;

    assert(tcg_enabled());
    g_assert(!icount_enabled());

    rcu_register_thread();
    tcg_register_thread();

    qemu_mutex_lock_iothread();
    qemu_thread_get_self(cpu->thread);

    cpu->thread_id = qemu_get_thread_id();
    cpu->can_do_io = 1;
    current_cpu = cpu;
    cpu_thread_signal_created(cpu);
    qemu_guest_random_seed_thread_part2(cpu->random_seed);

    /* process any pending work */
    cpu->exit_request = 1;

    do {
        if (cpu_can_run(cpu)) {
            int r;
            qemu_mutex_unlock_iothread();
            r = tcg_cpu_exec(cpu);
            qemu_mutex_lock_iothread();
            switch (r) {
            case EXCP_DEBUG:
                cpu_handle_guest_debug(cpu);
                break;
            case EXCP_HALTED:
                /*
                 * during start-up the vCPU is reset and the thread is
                 * kicked several times. If we don't ensure we go back
                 * to sleep in the halted state we won't cleanly
                 * start-up when the vCPU is enabled.
                 *
                 * cpu->halted should ensure we sleep in wait_io_event
                 */
                g_assert(cpu->halted);
                break;
            case EXCP_ATOMIC:
                qemu_mutex_unlock_iothread();
                cpu_exec_step_atomic(cpu);
                qemu_mutex_lock_iothread();
            default:
                /* Ignore everything else? */
                break;
            }
        }

        qatomic_mb_set(&cpu->exit_request, 0);
        qemu_wait_io_event(cpu);
    } while (!cpu->unplug || cpu_can_run(cpu));

    qemu_tcg_destroy_vcpu(cpu);
    qemu_mutex_unlock_iothread();
    rcu_unregister_thread();
    return NULL;
}

static void mttcg_kick_vcpu_thread(CPUState *cpu)
{
    cpu_exit(cpu);
}

const CpusAccel tcg_cpus_mttcg = {
    .create_vcpu_thread = tcg_start_vcpu_thread,
    .kick_vcpu_thread = mttcg_kick_vcpu_thread,

    .handle_interrupt = tcg_handle_interrupt,
};
