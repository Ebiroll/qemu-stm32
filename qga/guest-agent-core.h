/*
 * QEMU Guest Agent core declarations
 *
 * Copyright IBM Corp. 2011
 *
 * Authors:
 *  Adam Litke        <aglitke@linux.vnet.ibm.com>
 *  Michael Roth      <mdroth@linux.vnet.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */
#include "qapi/qmp/dispatch.h"
#include "qemu-common.h"

#define QGA_READ_COUNT_DEFAULT 4096

/* Mapping of whence codes used by guest-file-seek. */
enum {
    QGA_SEEK_SET = 0,
    QGA_SEEK_CUR = 1,
    QGA_SEEK_END = 2,
};

typedef struct GAState GAState;
typedef struct GACommandState GACommandState;
extern GAState *ga_state;

GList *ga_command_blacklist_init(GList *blacklist);
void ga_command_state_init(GAState *s, GACommandState *cs);
void ga_command_state_add(GACommandState *cs,
                          void (*init)(void),
                          void (*cleanup)(void));
void ga_command_state_init_all(GACommandState *cs);
void ga_command_state_cleanup_all(GACommandState *cs);
GACommandState *ga_command_state_new(void);
bool ga_logging_enabled(GAState *s);
void ga_disable_logging(GAState *s);
void ga_enable_logging(GAState *s);
void GCC_FMT_ATTR(1, 2) slog(const gchar *fmt, ...);
void ga_set_response_delimited(GAState *s);
bool ga_is_frozen(GAState *s);
void ga_set_frozen(GAState *s);
void ga_unset_frozen(GAState *s);
const char *ga_fsfreeze_hook(GAState *s);
int64_t ga_get_fd_handle(GAState *s, Error **errp);

#ifndef _WIN32
void reopen_fd_to_null(int fd);
#endif
