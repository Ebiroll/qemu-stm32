/*
 * Error reporting
 *
 * Copyright (C) 2010 Red Hat Inc.
 *
 * Authors:
 *  Markus Armbruster <armbru@redhat.com>,
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef QEMU_ERROR_REPORT_H
#define QEMU_ERROR_REPORT_H

typedef struct Location {
    /* all members are private to qemu-error.c */
    enum { LOC_NONE, LOC_CMDLINE, LOC_FILE } kind;
    int num;
    const void *ptr;
    struct Location *prev;
} Location;

Location *loc_push_restore(Location *loc);
Location *loc_push_none(Location *loc);
Location *loc_pop(Location *loc);
Location *loc_save(Location *loc);
void loc_restore(Location *loc);
void loc_set_none(void);
void loc_set_cmdline(char **argv, int idx, int cnt);
void loc_set_file(const char *fname, int lno);

void error_vprintf(const char *fmt, va_list ap) GCC_FMT_ATTR(1, 0);
void error_printf(const char *fmt, ...) GCC_FMT_ATTR(1, 2);
void error_vprintf_unless_qmp(const char *fmt, va_list ap) GCC_FMT_ATTR(1, 0);
void error_printf_unless_qmp(const char *fmt, ...) GCC_FMT_ATTR(1, 2);
void error_set_progname(const char *argv0);

void error_vreport(const char *fmt, va_list ap) GCC_FMT_ATTR(1, 0);
void warn_vreport(const char *fmt, va_list ap) GCC_FMT_ATTR(1, 0);
void info_vreport(const char *fmt, va_list ap) GCC_FMT_ATTR(1, 0);

void error_report(const char *fmt, ...) GCC_FMT_ATTR(1, 2);
void warn_report(const char *fmt, ...) GCC_FMT_ATTR(1, 2);
void info_report(const char *fmt, ...) GCC_FMT_ATTR(1, 2);

/*
 * Similar to error_report(), except it prints the message just once.
 * Return true when it prints, false otherwise.
 */
#define error_report_once(fmt, ...)             \
    ({                                          \
        static bool print_once_;                \
        bool ret_print_once_ = !print_once_;    \
                                                \
        if (!print_once_) {                     \
            print_once_ = true;                 \
            error_report(fmt, ##__VA_ARGS__);   \
        }                                       \
        unlikely(ret_print_once_);              \
    })

/*
 * Similar to warn_report(), except it prints the message just once.
 * Return true when it prints, false otherwise.
 */
#define warn_report_once(fmt, ...)              \
    ({                                          \
        static bool print_once_;                \
        bool ret_print_once_ = !print_once_;    \
                                                \
        if (!print_once_) {                     \
            print_once_ = true;                 \
            warn_report(fmt, ##__VA_ARGS__);    \
        }                                       \
        unlikely(ret_print_once_);              \
    })

const char *error_get_progname(void);
extern bool enable_timestamp_msg;

#endif
