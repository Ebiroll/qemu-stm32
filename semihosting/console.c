/*
 * Semihosting Console Support
 *
 * Copyright (c) 2015 Imagination Technologies
 * Copyright (c) 2019 Linaro Ltd
 *
 * This provides support for outputting to a semihosting console.
 *
 * While most semihosting implementations support reading and writing
 * to arbitrary file descriptors we treat the console as something
 * specifically for debugging interaction. This means messages can be
 * re-directed to gdb (if currently being used to debug) or even
 * re-directed elsewhere.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "semihosting/semihost.h"
#include "semihosting/console.h"
#include "exec/gdbstub.h"
#include "exec/exec-all.h"
#include "qemu/log.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "qemu/main-loop.h"
#include "qapi/error.h"
#include "qemu/fifo8.h"

/* Access to this structure is protected by the BQL */
typedef struct SemihostingConsole {
    CharBackend         backend;
    Chardev             *chr;
    GSList              *sleeping_cpus;
    bool                got;
    Fifo8               fifo;
} SemihostingConsole;

static SemihostingConsole console;

int qemu_semihosting_log_out(const char *s, int len)
{
    if (console.chr) {
        return qemu_chr_write_all(console.chr, (uint8_t *) s, len);
    } else {
        return write(STDERR_FILENO, s, len);
    }
}

#define FIFO_SIZE   1024

static int console_can_read(void *opaque)
{
    SemihostingConsole *c = opaque;
    int ret;
    g_assert(qemu_mutex_iothread_locked());
    ret = (int) fifo8_num_free(&c->fifo);
    return ret;
}

static void console_wake_up(gpointer data, gpointer user_data)
{
    CPUState *cs = (CPUState *) data;
    /* cpu_handle_halt won't know we have work so just unbung here */
    cs->halted = 0;
    qemu_cpu_kick(cs);
}

static void console_read(void *opaque, const uint8_t *buf, int size)
{
    SemihostingConsole *c = opaque;
    g_assert(qemu_mutex_iothread_locked());
    while (size-- && !fifo8_is_full(&c->fifo)) {
        fifo8_push(&c->fifo, *buf++);
    }
    g_slist_foreach(c->sleeping_cpus, console_wake_up, NULL);
    c->sleeping_cpus = NULL;
}

int qemu_semihosting_console_read(CPUState *cs, void *buf, int len)
{
    SemihostingConsole *c = &console;
    int ret = 0;

    g_assert(qemu_mutex_iothread_locked());

    /* Block if the fifo is completely empty. */
    if (fifo8_is_empty(&c->fifo)) {
        c->sleeping_cpus = g_slist_prepend(c->sleeping_cpus, cs);
        cs->halted = 1;
        cs->exception_index = EXCP_HALTED;
        cpu_loop_exit(cs);
        /* never returns */
    }

    /* Read until buffer full or fifo exhausted. */
    do {
        *(char *)(buf + ret) = fifo8_pop(&c->fifo);
        ret++;
    } while (ret < len && !fifo8_is_empty(&c->fifo));

    return ret;
}

int qemu_semihosting_console_write(void *buf, int len)
{
    if (console.chr) {
        return qemu_chr_write_all(console.chr, (uint8_t *)buf, len);
    } else {
        return fwrite(buf, 1, len, stderr);
    }
}

void qemu_semihosting_console_init(Chardev *chr)
{
    console.chr = chr;
    if  (chr) {
        fifo8_create(&console.fifo, FIFO_SIZE);
        qemu_chr_fe_init(&console.backend, chr, &error_abort);
        qemu_chr_fe_set_handlers(&console.backend,
                                 console_can_read,
                                 console_read,
                                 NULL, NULL, &console,
                                 NULL, true);
    }

    qemu_semihosting_guestfd_init();
}
