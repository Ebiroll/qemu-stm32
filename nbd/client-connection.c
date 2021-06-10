/*
 * QEMU Block driver for  NBD
 *
 * Copyright (c) 2021 Virtuozzo International GmbH.
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

#include "block/nbd.h"

#include "qapi/qapi-visit-sockets.h"
#include "qapi/clone-visitor.h"

struct NBDClientConnection {
    /* Initialization constants, never change */
    SocketAddress *saddr; /* address to connect to */
    QCryptoTLSCreds *tlscreds;
    NBDExportInfo initial_info;
    bool do_negotiation;
    bool do_retry;

    QemuMutex mutex;

    /*
     * @sioc and @err represent a connection attempt.  While running
     * is true, they are only used by the connection thread, and mutex
     * locking is not needed.  Once the thread finishes,
     * nbd_co_establish_connection then steals these pointers while
     * under the mutex.
     */
    NBDExportInfo updated_info;
    QIOChannelSocket *sioc;
    QIOChannel *ioc;
    Error *err;

    /* All further fields are accessed only under mutex */
    bool running; /* thread is running now */
    bool detached; /* thread is detached and should cleanup the state */

    /*
     * wait_co: if non-NULL, which coroutine to wake in
     * nbd_co_establish_connection() after yield()
     */
    Coroutine *wait_co;
};

/*
 * The function isn't protected by any mutex, only call it when the client
 * connection attempt has not yet started.
 */
void nbd_client_connection_enable_retry(NBDClientConnection *conn)
{
    conn->do_retry = true;
}

NBDClientConnection *nbd_client_connection_new(const SocketAddress *saddr,
                                               bool do_negotiation,
                                               const char *export_name,
                                               const char *x_dirty_bitmap,
                                               QCryptoTLSCreds *tlscreds)
{
    NBDClientConnection *conn = g_new(NBDClientConnection, 1);

    object_ref(OBJECT(tlscreds));
    *conn = (NBDClientConnection) {
        .saddr = QAPI_CLONE(SocketAddress, saddr),
        .tlscreds = tlscreds,
        .do_negotiation = do_negotiation,

        .initial_info.request_sizes = true,
        .initial_info.structured_reply = true,
        .initial_info.base_allocation = true,
        .initial_info.x_dirty_bitmap = g_strdup(x_dirty_bitmap),
        .initial_info.name = g_strdup(export_name ?: "")
    };

    qemu_mutex_init(&conn->mutex);

    return conn;
}

static void nbd_client_connection_do_free(NBDClientConnection *conn)
{
    if (conn->sioc) {
        qio_channel_close(QIO_CHANNEL(conn->sioc), NULL);
        object_unref(OBJECT(conn->sioc));
    }
    error_free(conn->err);
    qapi_free_SocketAddress(conn->saddr);
    object_unref(OBJECT(conn->tlscreds));
    g_free(conn->initial_info.x_dirty_bitmap);
    g_free(conn->initial_info.name);
    g_free(conn);
}

/*
 * Connect to @addr and do NBD negotiation if @info is not null. If @tlscreds
 * are given @outioc is returned. @outioc is provided only on success.  The call
 * may be cancelled from other thread by simply qio_channel_shutdown(sioc).
 */
static int nbd_connect(QIOChannelSocket *sioc, SocketAddress *addr,
                       NBDExportInfo *info, QCryptoTLSCreds *tlscreds,
                       QIOChannel **outioc, Error **errp)
{
    int ret;

    if (outioc) {
        *outioc = NULL;
    }

    ret = qio_channel_socket_connect_sync(sioc, addr, errp);
    if (ret < 0) {
        return ret;
    }

    qio_channel_set_delay(QIO_CHANNEL(sioc), false);

    if (!info) {
        return 0;
    }

    ret = nbd_receive_negotiate(NULL, QIO_CHANNEL(sioc), tlscreds,
                                tlscreds ? addr->u.inet.host : NULL,
                                outioc, info, errp);
    if (ret < 0) {
        /*
         * nbd_receive_negotiate() may setup tls ioc and return it even on
         * failure path. In this case we should use it instead of original
         * channel.
         */
        if (outioc && *outioc) {
            qio_channel_close(QIO_CHANNEL(*outioc), NULL);
            object_unref(OBJECT(*outioc));
            *outioc = NULL;
        } else {
            qio_channel_close(QIO_CHANNEL(sioc), NULL);
        }

        return ret;
    }

    return 0;
}

static void *connect_thread_func(void *opaque)
{
    NBDClientConnection *conn = opaque;
    int ret;
    bool do_free;
    uint64_t timeout = 1;
    uint64_t max_timeout = 16;

    while (true) {
        conn->sioc = qio_channel_socket_new();

        error_free(conn->err);
        conn->err = NULL;
        conn->updated_info = conn->initial_info;

        ret = nbd_connect(conn->sioc, conn->saddr,
                          conn->do_negotiation ? &conn->updated_info : NULL,
                          conn->tlscreds, &conn->ioc, &conn->err);

        /*
         * conn->updated_info will finally be returned to the user. Clear the
         * pointers to our internally allocated strings, which are IN parameters
         * of nbd_receive_negotiate() and therefore nbd_connect(). Caller
         * shoudn't be interested in these fields.
         */
        conn->updated_info.x_dirty_bitmap = NULL;
        conn->updated_info.name = NULL;

        if (ret < 0) {
            object_unref(OBJECT(conn->sioc));
            conn->sioc = NULL;
            if (conn->do_retry) {
                sleep(timeout);
                if (timeout < max_timeout) {
                    timeout *= 2;
                }
                continue;
            }
        }

        break;
    }

    qemu_mutex_lock(&conn->mutex);

    assert(conn->running);
    conn->running = false;
    if (conn->wait_co) {
        aio_co_wake(conn->wait_co);
        conn->wait_co = NULL;
    }
    do_free = conn->detached;

    qemu_mutex_unlock(&conn->mutex);

    if (do_free) {
        nbd_client_connection_do_free(conn);
    }

    return NULL;
}

void nbd_client_connection_release(NBDClientConnection *conn)
{
    bool do_free = false;

    if (!conn) {
        return;
    }

    WITH_QEMU_LOCK_GUARD(&conn->mutex) {
        assert(!conn->detached);
        if (conn->running) {
            conn->detached = true;
        } else {
            do_free = true;
        }
    }

    if (do_free) {
        nbd_client_connection_do_free(conn);
    }
}

/*
 * Get a new connection in context of @conn:
 *   if the thread is running, wait for completion
 *   if the thread already succeeded in the background, and user didn't get the
 *     result, just return it now
 *   otherwise the thread is not running, so start a thread and wait for
 *     completion
 *
 * If @info is not NULL, also do nbd-negotiation after successful connection.
 * In this case info is used only as out parameter, and is fully initialized by
 * nbd_co_establish_connection(). "IN" fields of info as well as related only to
 * nbd_receive_export_list() would be zero (see description of NBDExportInfo in
 * include/block/nbd.h).
 */
QIOChannelSocket *coroutine_fn
nbd_co_establish_connection(NBDClientConnection *conn, NBDExportInfo *info,
                            QIOChannel **ioc, Error **errp)
{
    QemuThread thread;

    if (conn->do_negotiation) {
        assert(info);
        assert(ioc);
    }

    WITH_QEMU_LOCK_GUARD(&conn->mutex) {
        /*
         * Don't call nbd_co_establish_connection() in several coroutines in
         * parallel. Only one call at once is supported.
         */
        assert(!conn->wait_co);

        if (!conn->running) {
            if (conn->sioc) {
                /* Previous attempt finally succeeded in background */
                if (conn->do_negotiation) {
                    *ioc = g_steal_pointer(&conn->ioc);
                    memcpy(info, &conn->updated_info, sizeof(*info));
                }
                return g_steal_pointer(&conn->sioc);
            }

            conn->running = true;
            error_free(conn->err);
            conn->err = NULL;
            qemu_thread_create(&thread, "nbd-connect",
                               connect_thread_func, conn, QEMU_THREAD_DETACHED);
        }

        conn->wait_co = qemu_coroutine_self();
    }

    /*
     * We are going to wait for connect-thread finish, but
     * nbd_co_establish_connection_cancel() can interrupt.
     */
    qemu_coroutine_yield();

    WITH_QEMU_LOCK_GUARD(&conn->mutex) {
        if (conn->running) {
            /*
             * The connection attempt was canceled and the coroutine resumed
             * before the connection thread finished its job.  Report the
             * attempt as failed, but leave the connection thread running,
             * to reuse it for the next connection attempt.
             */
            error_setg(errp, "Connection attempt cancelled by other operation");
            return NULL;
        } else {
            error_propagate(errp, conn->err);
            conn->err = NULL;
            if (conn->sioc && conn->do_negotiation) {
                *ioc = g_steal_pointer(&conn->ioc);
                memcpy(info, &conn->updated_info, sizeof(*info));
            }
            return g_steal_pointer(&conn->sioc);
        }
    }

    abort(); /* unreachable */
}

/*
 * nbd_co_establish_connection_cancel
 * Cancel nbd_co_establish_connection() asynchronously.
 *
 * Note that this function neither directly stops the thread nor closes the
 * socket, but rather safely wakes nbd_co_establish_connection() which is
 * sleeping in yield()
 */
void nbd_co_establish_connection_cancel(NBDClientConnection *conn)
{
    Coroutine *wait_co;

    WITH_QEMU_LOCK_GUARD(&conn->mutex) {
        wait_co = g_steal_pointer(&conn->wait_co);
    }

    if (wait_co) {
        aio_co_wake(wait_co);
    }
}
