/*
 * QEMU Block backends
 *
 * Copyright (C) 2014-2016 Red Hat, Inc.
 *
 * Authors:
 *  Markus Armbruster <armbru@redhat.com>,
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1
 * or later.  See the COPYING.LIB file in the top-level directory.
 */

#ifndef BLOCK_BACKEND_IO_H
#define BLOCK_BACKEND_IO_H

#include "block-backend-common.h"

/*
 * I/O API functions. These functions are thread-safe.
 *
 * See include/block/block-io.h for more information about
 * the I/O API.
 */

const char *blk_name(const BlockBackend *blk);

BlockDriverState *blk_bs(BlockBackend *blk);

void blk_set_allow_write_beyond_eof(BlockBackend *blk, bool allow);
void blk_set_allow_aio_context_change(BlockBackend *blk, bool allow);
void blk_set_disable_request_queuing(BlockBackend *blk, bool disable);
bool blk_iostatus_is_enabled(const BlockBackend *blk);

char *blk_get_attached_dev_id(BlockBackend *blk);

BlockAIOCB *blk_aio_pwrite_zeroes(BlockBackend *blk, int64_t offset,
                                  int64_t bytes, BdrvRequestFlags flags,
                                  BlockCompletionFunc *cb, void *opaque);

BlockAIOCB *blk_aio_preadv(BlockBackend *blk, int64_t offset,
                           QEMUIOVector *qiov, BdrvRequestFlags flags,
                           BlockCompletionFunc *cb, void *opaque);
BlockAIOCB *blk_aio_pwritev(BlockBackend *blk, int64_t offset,
                            QEMUIOVector *qiov, BdrvRequestFlags flags,
                            BlockCompletionFunc *cb, void *opaque);
BlockAIOCB *blk_aio_flush(BlockBackend *blk,
                          BlockCompletionFunc *cb, void *opaque);
BlockAIOCB *blk_aio_pdiscard(BlockBackend *blk, int64_t offset, int64_t bytes,
                             BlockCompletionFunc *cb, void *opaque);
void blk_aio_cancel_async(BlockAIOCB *acb);
BlockAIOCB *blk_aio_ioctl(BlockBackend *blk, unsigned long int req, void *buf,
                          BlockCompletionFunc *cb, void *opaque);

void blk_inc_in_flight(BlockBackend *blk);
void blk_dec_in_flight(BlockBackend *blk);
bool blk_is_inserted(BlockBackend *blk);
bool blk_is_available(BlockBackend *blk);
void blk_lock_medium(BlockBackend *blk, bool locked);
void blk_eject(BlockBackend *blk, bool eject_flag);
int64_t blk_getlength(BlockBackend *blk);
void blk_get_geometry(BlockBackend *blk, uint64_t *nb_sectors_ptr);
int64_t blk_nb_sectors(BlockBackend *blk);
void *blk_try_blockalign(BlockBackend *blk, size_t size);
void *blk_blockalign(BlockBackend *blk, size_t size);
bool blk_is_writable(BlockBackend *blk);
bool blk_enable_write_cache(BlockBackend *blk);
BlockdevOnError blk_get_on_error(BlockBackend *blk, bool is_read);
BlockErrorAction blk_get_error_action(BlockBackend *blk, bool is_read,
                                      int error);
void blk_error_action(BlockBackend *blk, BlockErrorAction action,
                      bool is_read, int error);
void blk_iostatus_set_err(BlockBackend *blk, int error);
int blk_get_max_iov(BlockBackend *blk);
int blk_get_max_hw_iov(BlockBackend *blk);

void blk_io_plug(BlockBackend *blk);
void blk_io_unplug(BlockBackend *blk);
AioContext *blk_get_aio_context(BlockBackend *blk);
BlockAcctStats *blk_get_stats(BlockBackend *blk);
void *blk_aio_get(const AIOCBInfo *aiocb_info, BlockBackend *blk,
                  BlockCompletionFunc *cb, void *opaque);
BlockAIOCB *blk_abort_aio_request(BlockBackend *blk,
                                  BlockCompletionFunc *cb,
                                  void *opaque, int ret);

uint32_t blk_get_request_alignment(BlockBackend *blk);
uint32_t blk_get_max_transfer(BlockBackend *blk);
uint64_t blk_get_max_hw_transfer(BlockBackend *blk);

int coroutine_fn blk_co_copy_range(BlockBackend *blk_in, int64_t off_in,
                                   BlockBackend *blk_out, int64_t off_out,
                                   int64_t bytes, BdrvRequestFlags read_flags,
                                   BdrvRequestFlags write_flags);


/*
 * "I/O or GS" API functions. These functions can run without
 * the BQL, but only in one specific iothread/main loop.
 *
 * See include/block/block-io.h for more information about
 * the "I/O or GS" API.
 */

int blk_pread(BlockBackend *blk, int64_t offset, int bytes, void *buf,
              BdrvRequestFlags flags);
int blk_pwrite(BlockBackend *blk, int64_t offset, int bytes, const void *buf,
               BdrvRequestFlags flags);
int coroutine_fn blk_co_preadv(BlockBackend *blk, int64_t offset,
                               int64_t bytes, QEMUIOVector *qiov,
                               BdrvRequestFlags flags);
int coroutine_fn blk_co_pwritev_part(BlockBackend *blk, int64_t offset,
                                     int64_t bytes,
                                     QEMUIOVector *qiov, size_t qiov_offset,
                                     BdrvRequestFlags flags);
int coroutine_fn blk_co_pwritev(BlockBackend *blk, int64_t offset,
                                int64_t bytes, QEMUIOVector *qiov,
                                BdrvRequestFlags flags);

static inline int coroutine_fn blk_co_pread(BlockBackend *blk, int64_t offset,
                                            int64_t bytes, void *buf,
                                            BdrvRequestFlags flags)
{
    QEMUIOVector qiov = QEMU_IOVEC_INIT_BUF(qiov, buf, bytes);
    IO_OR_GS_CODE();

    assert(bytes <= SIZE_MAX);

    return blk_co_preadv(blk, offset, bytes, &qiov, flags);
}

static inline int coroutine_fn blk_co_pwrite(BlockBackend *blk, int64_t offset,
                                             int64_t bytes, void *buf,
                                             BdrvRequestFlags flags)
{
    QEMUIOVector qiov = QEMU_IOVEC_INIT_BUF(qiov, buf, bytes);
    IO_OR_GS_CODE();

    assert(bytes <= SIZE_MAX);

    return blk_co_pwritev(blk, offset, bytes, &qiov, flags);
}

int coroutine_fn blk_co_pdiscard(BlockBackend *blk, int64_t offset,
                                 int64_t bytes);

int coroutine_fn blk_co_flush(BlockBackend *blk);
int blk_flush(BlockBackend *blk);

int blk_ioctl(BlockBackend *blk, unsigned long int req, void *buf);

int blk_pwrite_compressed(BlockBackend *blk, int64_t offset, const void *buf,
                          int64_t bytes);
int blk_pdiscard(BlockBackend *blk, int64_t offset, int64_t bytes);
int blk_pwrite_zeroes(BlockBackend *blk, int64_t offset,
                      int64_t bytes, BdrvRequestFlags flags);
int coroutine_fn blk_co_pwrite_zeroes(BlockBackend *blk, int64_t offset,
                                      int64_t bytes, BdrvRequestFlags flags);
int blk_truncate(BlockBackend *blk, int64_t offset, bool exact,
                 PreallocMode prealloc, BdrvRequestFlags flags, Error **errp);

#endif /* BLOCK_BACKEND_IO_H */
