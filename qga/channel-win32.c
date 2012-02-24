#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <glib.h>
#include <windows.h>
#include <errno.h>
#include <io.h>
#include "qga/guest-agent-core.h"
#include "qga/channel.h"

typedef struct GAChannelReadState {
    guint thread_id;
    uint8_t *buf;
    size_t buf_size;
    size_t cur; /* current buffer start */
    size_t pending; /* pending buffered bytes to read */
    OVERLAPPED ov;
    bool ov_pending; /* whether on async read is outstanding */
} GAChannelReadState;

struct GAChannel {
    HANDLE handle;
    GAChannelCallback cb;
    gpointer user_data;
    GAChannelReadState rstate;
    GIOCondition pending_events; /* TODO: use GAWatch.pollfd.revents */
    GSource *source;
};

typedef struct GAWatch {
    GSource source;
    GPollFD pollfd;
    GAChannel *channel;
    GIOCondition events_mask;
} GAWatch;

/*
 * Called by glib prior to polling to set up poll events if polling is needed.
 *
 */
static gboolean ga_channel_prepare(GSource *source, gint *timeout_ms)
{
    GAWatch *watch = (GAWatch *)source;
    GAChannel *c = (GAChannel *)watch->channel;
    GAChannelReadState *rs = &c->rstate;
    DWORD count_read, count_to_read = 0;
    bool success;
    GIOCondition new_events = 0;

    g_debug("prepare");
    /* go ahead and submit another read if there's room in the buffer
     * and no previous reads are outstanding
     */
    if (!rs->ov_pending) {
        if (rs->cur + rs->pending >= rs->buf_size) {
            if (rs->cur) {
                memmove(rs->buf, rs->buf + rs->cur, rs->pending);
                rs->cur = 0;
            }
        }
        count_to_read = rs->buf_size - rs->cur - rs->pending;
    }

    if (rs->ov_pending || count_to_read <= 0) {
            goto out;
    }

    /* submit the read */
    success = ReadFile(c->handle, rs->buf + rs->cur + rs->pending,
                       count_to_read, &count_read, &rs->ov);
    if (success) {
        rs->pending += count_read;
        rs->ov_pending = false;
    } else {
        if (GetLastError() == ERROR_IO_PENDING) {
            rs->ov_pending = true;
        } else {
            new_events |= G_IO_ERR;
        }
    }

out:
    /* dont block forever, iterate the main loop every once and a while */
    *timeout_ms = 500;
    /* if there's data in the read buffer, or another event is pending,
     * skip polling and issue user cb.
     */
    if (rs->pending) {
        new_events |= G_IO_IN;
    }
    c->pending_events |= new_events;
    return !!c->pending_events;
}

/*
 * Called by glib after an outstanding read request is completed.
 */
static gboolean ga_channel_check(GSource *source)
{
    GAWatch *watch = (GAWatch *)source;
    GAChannel *c = (GAChannel *)watch->channel;
    GAChannelReadState *rs = &c->rstate;
    DWORD count_read, error;
    BOOL success;

    GIOCondition new_events = 0;

    g_debug("check");

    /* failing this implies we issued a read that completed immediately,
     * yet no data was placed into the buffer (and thus we did not skip
     * polling). but since EOF is not obtainable until we retrieve an
     * overlapped result, it must be the case that there was data placed
     * into the buffer, or an error was generated by Readfile(). in either
     * case, we should've skipped the polling for this round.
     */
    g_assert(rs->ov_pending);

    success = GetOverlappedResult(c->handle, &rs->ov, &count_read, FALSE);
    if (success) {
        g_debug("thread: overlapped result, count_read: %d", (int)count_read);
        rs->pending += count_read;
        new_events |= G_IO_IN;
    } else {
        error = GetLastError();
        if (error == 0 || error == ERROR_HANDLE_EOF ||
            error == ERROR_NO_SYSTEM_RESOURCES ||
            error == ERROR_OPERATION_ABORTED) {
            /* note: On WinXP SP3 with rhel6ga virtio-win-1.1.16 vioser drivers,
             * ENSR seems to be synonymous with when we'd normally expect
             * ERROR_HANDLE_EOF. So treat it as such. Microsoft's
             * recommendation for ERROR_NO_SYSTEM_RESOURCES is to
             * retry the read, so this happens to work out anyway. On newer
             * virtio-win driver, this seems to be replaced with EOA, so
             * handle that in the same fashion.
             */
            new_events |= G_IO_HUP;
        } else if (error != ERROR_IO_INCOMPLETE) {
            g_critical("error retrieving overlapped result: %d", (int)error);
            new_events |= G_IO_ERR;
        }
    }

    if (new_events) {
        rs->ov_pending = 0;
    }
    c->pending_events |= new_events;

    return !!c->pending_events;
}

/*
 * Called by glib after either prepare or check routines signal readiness
 */
static gboolean ga_channel_dispatch(GSource *source, GSourceFunc unused,
                                    gpointer user_data)
{
    GAWatch *watch = (GAWatch *)source;
    GAChannel *c = (GAChannel *)watch->channel;
    GAChannelReadState *rs = &c->rstate;
    gboolean success;

    g_debug("dispatch");
    success = c->cb(watch->pollfd.revents, c->user_data);

    if (c->pending_events & G_IO_ERR) {
        g_critical("channel error, removing source");
        return false;
    }

    /* TODO: replace rs->pending with watch->revents */
    c->pending_events &= ~G_IO_HUP;
    if (!rs->pending) {
        c->pending_events &= ~G_IO_IN;
    } else {
        c->pending_events = 0;
    }
    return success;
}

static void ga_channel_finalize(GSource *source)
{
    g_debug("finalize");
}

GSourceFuncs ga_channel_watch_funcs = {
    ga_channel_prepare,
    ga_channel_check,
    ga_channel_dispatch,
    ga_channel_finalize
};

static GSource *ga_channel_create_watch(GAChannel *c)
{
    GSource *source = g_source_new(&ga_channel_watch_funcs, sizeof(GAWatch));
    GAWatch *watch = (GAWatch *)source;

    watch->channel = c;
    watch->pollfd.fd = (gintptr) c->rstate.ov.hEvent;
    g_source_add_poll(source, &watch->pollfd);

    return source;
}

GIOStatus ga_channel_read(GAChannel *c, char *buf, size_t size, gsize *count)
{
    GAChannelReadState *rs = &c->rstate;
    GIOStatus status;
    size_t to_read = 0;

    if (c->pending_events & G_IO_ERR) {
        return G_IO_STATUS_ERROR;
    }

    *count = to_read = MIN(size, rs->pending);
    if (to_read) {
        memcpy(buf, rs->buf + rs->cur, to_read);
        rs->cur += to_read;
        rs->pending -= to_read;
        status = G_IO_STATUS_NORMAL;
    } else {
        status = G_IO_STATUS_AGAIN;
    }

    return status;
}

static GIOStatus ga_channel_write(GAChannel *c, const char *buf, size_t size,
                                  size_t *count)
{
    GIOStatus status;
    OVERLAPPED ov = {0};
    BOOL ret;
    DWORD written;

    ov.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
    ret = WriteFile(c->handle, buf, size, &written, &ov);
    if (!ret) {
        if (GetLastError() == ERROR_IO_PENDING) {
            /* write is pending */
            ret = GetOverlappedResult(c->handle, &ov, &written, TRUE);
            if (!ret) {
                if (!GetLastError()) {
                    status = G_IO_STATUS_AGAIN;
                } else {
                    status = G_IO_STATUS_ERROR;
                }
            } else {
                /* write is complete */
                status = G_IO_STATUS_NORMAL;
                *count = written;
            }
        } else {
            status = G_IO_STATUS_ERROR;
        }
    } else {
        /* write returned immediately */
        status = G_IO_STATUS_NORMAL;
        *count = written;
    }

    return status;
}

GIOStatus ga_channel_write_all(GAChannel *c, const char *buf, size_t size)
{
    GIOStatus status = G_IO_STATUS_NORMAL;;
    size_t count;

    while (size) {
        status = ga_channel_write(c, buf, size, &count);
        if (status == G_IO_STATUS_NORMAL) {
            size -= count;
            buf += count;
        } else if (status != G_IO_STATUS_AGAIN) {
            break;
        }
    }

    return status;
}

static gboolean ga_channel_open(GAChannel *c, GAChannelMethod method,
                                const gchar *path)
{
    if (!method == GA_CHANNEL_VIRTIO_SERIAL) {
        g_critical("unsupported communication method");
        return false;
    }

    c->handle = CreateFile(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                           OPEN_EXISTING,
                           FILE_FLAG_NO_BUFFERING | FILE_FLAG_OVERLAPPED, NULL);
    if (c->handle == INVALID_HANDLE_VALUE) {
        g_critical("error opening path");
        return false;
    }

    return true;
}

GAChannel *ga_channel_new(GAChannelMethod method, const gchar *path,
                          GAChannelCallback cb, gpointer opaque)
{
    GAChannel *c = g_malloc0(sizeof(GAChannel));
    SECURITY_ATTRIBUTES sec_attrs;

    if (!ga_channel_open(c, method, path)) {
        g_critical("error opening channel");
        g_free(c);
        return NULL;
    }

    c->cb = cb;
    c->user_data = opaque;

    sec_attrs.nLength = sizeof(SECURITY_ATTRIBUTES);
    sec_attrs.lpSecurityDescriptor = NULL;
    sec_attrs.bInheritHandle = false;

    c->rstate.buf_size = QGA_READ_COUNT_DEFAULT;
    c->rstate.buf = g_malloc(QGA_READ_COUNT_DEFAULT);
    c->rstate.ov.hEvent = CreateEvent(&sec_attrs, FALSE, FALSE, NULL);

    c->source = ga_channel_create_watch(c);
    g_source_attach(c->source, NULL);
    return c;
}

void ga_channel_free(GAChannel *c)
{
    if (c->source) {
        g_source_destroy(c->source);
    }
    if (c->rstate.ov.hEvent) {
        CloseHandle(c->rstate.ov.hEvent);
    }
    g_free(c->rstate.buf);
    g_free(c);
}
