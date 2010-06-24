/*
 * Simple trace backend
 *
 * Copyright IBM, Corp. 2010
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "trace.h"

/** Trace file header event ID */
#define HEADER_EVENT_ID (~(uint64_t)0) /* avoids conflicting with TraceEventIDs */

/** Trace file magic number */
#define HEADER_MAGIC 0xf2b177cb0aa429b4ULL

/** Trace file version number, bump if format changes */
#define HEADER_VERSION 0

/** Trace buffer entry */
typedef struct {
    uint64_t event;
    uint64_t timestamp_ns;
    uint64_t x1;
    uint64_t x2;
    uint64_t x3;
    uint64_t x4;
    uint64_t x5;
    uint64_t x6;
} TraceRecord;

enum {
    TRACE_BUF_LEN = 64 * 1024 / sizeof(TraceRecord),
};

static TraceRecord trace_buf[TRACE_BUF_LEN];
static unsigned int trace_idx;
static FILE *trace_fp;

static bool write_header(FILE *fp)
{
    static const TraceRecord header = {
        .event = HEADER_EVENT_ID,
        .timestamp_ns = HEADER_MAGIC,
        .x1 = HEADER_VERSION,
    };

    return fwrite(&header, sizeof header, 1, fp) == 1;
}

static void flush_trace_buffer(void)
{
    if (!trace_fp) {
        trace_fp = fopen("trace.log", "w");
        if (trace_fp) {
            write_header(trace_fp);
        }
    }
    if (trace_fp) {
        size_t unused; /* for when fwrite(3) is declared warn_unused_result */
        unused = fwrite(trace_buf, trace_idx * sizeof(trace_buf[0]), 1, trace_fp);
    }

    /* Discard written trace records */
    trace_idx = 0;
}

void st_set_trace_file_enabled(bool enable)
{
    if (enable == trace_file_enabled) {
        return; /* no change */
    }

    /* Flush/discard trace buffer */
    st_flush_trace_buffer();

    /* To disable, close trace file */
    if (!enable) {
        fclose(trace_fp);
        trace_fp = NULL;
    }

    trace_file_enabled = enable;
}

static void trace(TraceEventID event, uint64_t x1, uint64_t x2, uint64_t x3,
                  uint64_t x4, uint64_t x5, uint64_t x6)
{
    TraceRecord *rec = &trace_buf[trace_idx];
    struct timespec ts;

    /* TODO Windows?  It would be good to use qemu-timer here but that isn't
     * linked into qemu-tools.  Also we should avoid recursion in the tracing
     * code, therefore it is useful to be self-contained.
     */
    clock_gettime(CLOCK_MONOTONIC, &ts);

    if (!trace_list[event].state) {
        return;
    }

    rec->event = event;
    rec->timestamp_ns = ts.tv_sec * 1000000000LL + ts.tv_nsec;
    rec->x1 = x1;
    rec->x2 = x2;
    rec->x3 = x3;
    rec->x4 = x4;
    rec->x5 = x5;
    rec->x6 = x6;

    if (++trace_idx == TRACE_BUF_LEN) {
        flush_trace_buffer();
    }
}

void trace0(TraceEventID event)
{
    trace(event, 0, 0, 0, 0, 0, 0);
}

void trace1(TraceEventID event, uint64_t x1)
{
    trace(event, x1, 0, 0, 0, 0, 0);
}

void trace2(TraceEventID event, uint64_t x1, uint64_t x2)
{
    trace(event, x1, x2, 0, 0, 0, 0);
}

void trace3(TraceEventID event, uint64_t x1, uint64_t x2, uint64_t x3)
{
    trace(event, x1, x2, x3, 0, 0, 0);
}

void trace4(TraceEventID event, uint64_t x1, uint64_t x2, uint64_t x3, uint64_t x4)
{
    trace(event, x1, x2, x3, x4, 0, 0);
}

void trace5(TraceEventID event, uint64_t x1, uint64_t x2, uint64_t x3, uint64_t x4, uint64_t x5)
{
    trace(event, x1, x2, x3, x4, x5, 0);
}

void trace6(TraceEventID event, uint64_t x1, uint64_t x2, uint64_t x3, uint64_t x4, uint64_t x5, uint64_t x6)
{
    trace(event, x1, x2, x3, x4, x5, x6);
}

/**
 * Flush the trace buffer on exit
 */
static void __attribute__((constructor)) st_init(void)
{
    atexit(st_flush_trace_buffer);
}

void st_print_trace(FILE *stream, int (*stream_printf)(FILE *stream, const char *fmt, ...))
{
    unsigned int i;

    for (i = 0; i < trace_idx; i++) {
        stream_printf(stream, "Event %lu : %lx %lx %lx %lx %lx\n",
                      trace_buf[i].event, trace_buf[i].x1, trace_buf[i].x2,
                      trace_buf[i].x3, trace_buf[i].x4, trace_buf[i].x5);
    }
}

void st_print_trace_events(FILE *stream, int (*stream_printf)(FILE *stream, const char *fmt, ...))
{
    unsigned int i;

    for (i = 0; i < NR_TRACE_EVENTS; i++) {
        stream_printf(stream, "%s [Event ID %u] : state %u\n",
                      trace_list[i].tp_name, i, trace_list[i].state);
    }
}

static TraceEvent* find_trace_event_by_name(const char *tname)
{
    unsigned int i;

    if (!tname) {
        return NULL;
    }

    for (i = 0; i < NR_TRACE_EVENTS; i++) {
        if (!strcmp(trace_list[i].tp_name, tname)) {
            return &trace_list[i];
        }
    }
    return NULL; /* indicates end of list reached without a match */
}

void st_change_trace_event_state(const char *tname, bool tstate)
{
    TraceEvent *tp;

    tp = find_trace_event_by_name(tname);
    if (tp) {
        tp->state = tstate;
    }
}
