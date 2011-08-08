/*
 * Virtio 9p backend
 *
 * Copyright IBM, Corp. 2010
 *
 * Authors:
 *  Harsh Prateek Bora <harsh@linux.vnet.ibm.com>
 *  Venkateswararao Jujjuri(JV) <jvrao@linux.vnet.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

#ifndef _QEMU_VIRTIO_9P_COTH_H
#define _QEMU_VIRTIO_9P_COTH_H

#include "qemu-thread.h"
#include "qemu-coroutine.h"
#include "virtio-9p.h"
#include <glib.h>

typedef struct V9fsThPool {
    int rfd;
    int wfd;
    GThreadPool *pool;
    GAsyncQueue *completed;
} V9fsThPool;

/*
 * we want to use bottom half because we want to make sure the below
 * sequence of events.
 *
 *   1. Yield the coroutine in the QEMU thread.
 *   2. Submit the coroutine to a worker thread.
 *   3. Enter the coroutine in the worker thread.
 * we cannot swap step 1 and 2, because that would imply worker thread
 * can enter coroutine while step1 is still running
 */
#define v9fs_co_run_in_worker(code_block)                               \
    do {                                                                \
        QEMUBH *co_bh;                                                  \
        co_bh = qemu_bh_new(co_run_in_worker_bh,                        \
                            qemu_coroutine_self());                     \
        qemu_bh_schedule(co_bh);                                        \
        /*                                                              \
         * yeild in qemu thread and re-enter back                       \
         * in glib worker thread                                        \
         */                                                             \
        qemu_coroutine_yield();                                         \
        qemu_bh_delete(co_bh);                                          \
        code_block;                                                     \
        /* re-enter back to qemu thread */                              \
        qemu_coroutine_yield();                                         \
    } while (0)

extern void co_run_in_worker_bh(void *);
extern int v9fs_init_worker_threads(void);
extern int v9fs_co_readlink(V9fsState *, V9fsString *, V9fsString *);
#endif
