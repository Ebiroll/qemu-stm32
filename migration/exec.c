/*
 * QEMU live migration
 *
 * Copyright IBM, Corp. 2008
 * Copyright Dell MessageOne 2008
 * Copyright Red Hat, Inc. 2015-2016
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *  Charles Duffy     <charles_duffy@messageone.com>
 *  Daniel P. Berrange <berrange@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "channel.h"
#include "exec.h"
#include "migration.h"
#include "io/channel-command.h"
#include "trace.h"


void exec_start_outgoing_migration(MigrationState *s, const char *command, Error **errp)
{
    QIOChannel *ioc;
    const char *argv[] = { "/bin/sh", "-c", command, NULL };

    trace_migration_exec_outgoing(command);
    ioc = QIO_CHANNEL(qio_channel_command_new_spawn(argv,
                                                    O_RDWR,
                                                    errp));
    if (!ioc) {
        return;
    }

    qio_channel_set_name(ioc, "migration-exec-outgoing");
    migration_channel_connect(s, ioc, NULL, NULL);
    object_unref(OBJECT(ioc));
}

static gboolean exec_accept_incoming_migration(QIOChannel *ioc,
                                               GIOCondition condition,
                                               gpointer opaque)
{
    migration_channel_process_incoming(ioc);
    object_unref(OBJECT(ioc));
    if (!migrate_use_multifd()) {
        migration_incoming_process();
    }
    return G_SOURCE_REMOVE;
}

void exec_start_incoming_migration(const char *command, Error **errp)
{
    QIOChannel *ioc;
    const char *argv[] = { "/bin/sh", "-c", command, NULL };

    trace_migration_exec_incoming(command);
    ioc = QIO_CHANNEL(qio_channel_command_new_spawn(argv,
                                                    O_RDWR,
                                                    errp));
    if (!ioc) {
        return;
    }

    qio_channel_set_name(ioc, "migration-exec-incoming");
    qio_channel_add_watch_full(ioc, G_IO_IN,
                               exec_accept_incoming_migration,
                               NULL, NULL,
                               g_main_context_get_thread_default());
}
