/*
 * QEMU System Emulator
 *
 * Copyright (c) 2003-2008 Fabrice Bellard
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

#ifndef MIGRATION_QEMU_FILE_H
#define MIGRATION_QEMU_FILE_H

#include <zlib.h>
#include "exec/cpu-common.h"
#include "io/channel.h"

/*
 * This function provides hooks around different
 * stages of RAM migration.
 * 'data' is call specific data associated with the 'flags' value
 */
typedef int (QEMURamHookFunc)(QEMUFile *f, uint64_t flags, void *data);

/*
 * Constants used by ram_control_* hooks
 */
#define RAM_CONTROL_SETUP     0
#define RAM_CONTROL_ROUND     1
#define RAM_CONTROL_HOOK      2
#define RAM_CONTROL_FINISH    3
#define RAM_CONTROL_BLOCK_REG 4

/*
 * This function allows override of where the RAM page
 * is saved (such as RDMA, for example.)
 */
typedef int (QEMURamSaveFunc)(QEMUFile *f,
                              ram_addr_t block_offset,
                              ram_addr_t offset,
                              size_t size);

typedef struct QEMUFileHooks {
    QEMURamHookFunc *after_ram_iterate;
    QEMURamHookFunc *hook_ram_load;
    QEMURamSaveFunc *save_page;
} QEMUFileHooks;

QEMUFile *qemu_file_new_input(QIOChannel *ioc);
QEMUFile *qemu_file_new_output(QIOChannel *ioc);
void qemu_file_set_hooks(QEMUFile *f, const QEMUFileHooks *hooks);
int qemu_fclose(QEMUFile *f);

/*
 * qemu_file_transferred:
 *
 * Report the total number of bytes transferred with
 * this file.
 *
 * For writable files, any pending buffers will be
 * flushed, so the reported value will be equal to
 * the number of bytes transferred on the wire.
 *
 * For readable files, the reported value will be
 * equal to the number of bytes transferred on the
 * wire.
 *
 * Returns: the total bytes transferred
 */
uint64_t qemu_file_transferred(QEMUFile *f);

/*
 * qemu_file_transferred_noflush:
 *
 * As qemu_file_transferred except for writable files, where no flush
 * is performed and the reported amount will include the size of any
 * queued buffers, on top of the amount actually transferred.
 *
 * Returns: the total bytes transferred and queued
 */
uint64_t qemu_file_transferred_noflush(QEMUFile *f);

/*
 * put_buffer without copying the buffer.
 * The buffer should be available till it is sent asynchronously.
 */
void qemu_put_buffer_async(QEMUFile *f, const uint8_t *buf, size_t size,
                           bool may_free);

#include "migration/qemu-file-types.h"

size_t coroutine_mixed_fn qemu_peek_buffer(QEMUFile *f, uint8_t **buf, size_t size, size_t offset);
size_t coroutine_mixed_fn qemu_get_buffer_in_place(QEMUFile *f, uint8_t **buf, size_t size);
ssize_t qemu_put_compression_data(QEMUFile *f, z_stream *stream,
                                  const uint8_t *p, size_t size);
int qemu_put_qemu_file(QEMUFile *f_des, QEMUFile *f_src);
bool qemu_file_buffer_empty(QEMUFile *file);

/*
 * Note that you can only peek continuous bytes from where the current pointer
 * is; you aren't guaranteed to be able to peak to +n bytes unless you've
 * previously peeked +n-1.
 */
int coroutine_mixed_fn qemu_peek_byte(QEMUFile *f, int offset);
void qemu_file_skip(QEMUFile *f, int size);
int qemu_file_get_error_obj_any(QEMUFile *f1, QEMUFile *f2, Error **errp);
void qemu_file_set_error_obj(QEMUFile *f, int ret, Error *err);
void qemu_file_set_error(QEMUFile *f, int ret);
int qemu_file_shutdown(QEMUFile *f);
QEMUFile *qemu_file_get_return_path(QEMUFile *f);
void qemu_fflush(QEMUFile *f);
void qemu_file_set_blocking(QEMUFile *f, bool block);
int qemu_file_get_to_fd(QEMUFile *f, int fd, size_t size);

void ram_control_after_iterate(QEMUFile *f, uint64_t flags);
void ram_control_load_hook(QEMUFile *f, uint64_t flags, void *data);

/* Whenever this is found in the data stream, the flags
 * will be passed to ram_control_load_hook in the incoming-migration
 * side. This lets before_ram_iterate/after_ram_iterate add
 * transport-specific sections to the RAM migration data.
 */
#define RAM_SAVE_FLAG_HOOK     0x80

#define RAM_SAVE_CONTROL_NOT_SUPP -1000
#define RAM_SAVE_CONTROL_DELAYED  -2000

int ram_control_save_page(QEMUFile *f, ram_addr_t block_offset,
                          ram_addr_t offset, size_t size);
QIOChannel *qemu_file_get_ioc(QEMUFile *file);

#endif
