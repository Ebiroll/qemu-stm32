/*
 * vhost-user
 *
 * Copyright (c) 2013 Virtual Open Systems Sarl.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/virtio/vhost.h"
#include "hw/virtio/vhost-backend.h"
#include "hw/virtio/virtio-net.h"
#include "chardev/char-fe.h"
#include "sysemu/kvm.h"
#include "qemu/error-report.h"
#include "qemu/sockets.h"
#include "sysemu/cryptodev.h"
#include "migration/migration.h"
#include "migration/postcopy-ram.h"
#include "trace.h"

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <linux/vhost.h>

#define VHOST_MEMORY_MAX_NREGIONS    8
#define VHOST_USER_F_PROTOCOL_FEATURES 30

/*
 * Maximum size of virtio device config space
 */
#define VHOST_USER_MAX_CONFIG_SIZE 256

enum VhostUserProtocolFeature {
    VHOST_USER_PROTOCOL_F_MQ = 0,
    VHOST_USER_PROTOCOL_F_LOG_SHMFD = 1,
    VHOST_USER_PROTOCOL_F_RARP = 2,
    VHOST_USER_PROTOCOL_F_REPLY_ACK = 3,
    VHOST_USER_PROTOCOL_F_NET_MTU = 4,
    VHOST_USER_PROTOCOL_F_SLAVE_REQ = 5,
    VHOST_USER_PROTOCOL_F_CROSS_ENDIAN = 6,
    VHOST_USER_PROTOCOL_F_CRYPTO_SESSION = 7,
    VHOST_USER_PROTOCOL_F_PAGEFAULT = 8,
    VHOST_USER_PROTOCOL_F_MAX
};

#define VHOST_USER_PROTOCOL_FEATURE_MASK ((1 << VHOST_USER_PROTOCOL_F_MAX) - 1)

typedef enum VhostUserRequest {
    VHOST_USER_NONE = 0,
    VHOST_USER_GET_FEATURES = 1,
    VHOST_USER_SET_FEATURES = 2,
    VHOST_USER_SET_OWNER = 3,
    VHOST_USER_RESET_OWNER = 4,
    VHOST_USER_SET_MEM_TABLE = 5,
    VHOST_USER_SET_LOG_BASE = 6,
    VHOST_USER_SET_LOG_FD = 7,
    VHOST_USER_SET_VRING_NUM = 8,
    VHOST_USER_SET_VRING_ADDR = 9,
    VHOST_USER_SET_VRING_BASE = 10,
    VHOST_USER_GET_VRING_BASE = 11,
    VHOST_USER_SET_VRING_KICK = 12,
    VHOST_USER_SET_VRING_CALL = 13,
    VHOST_USER_SET_VRING_ERR = 14,
    VHOST_USER_GET_PROTOCOL_FEATURES = 15,
    VHOST_USER_SET_PROTOCOL_FEATURES = 16,
    VHOST_USER_GET_QUEUE_NUM = 17,
    VHOST_USER_SET_VRING_ENABLE = 18,
    VHOST_USER_SEND_RARP = 19,
    VHOST_USER_NET_SET_MTU = 20,
    VHOST_USER_SET_SLAVE_REQ_FD = 21,
    VHOST_USER_IOTLB_MSG = 22,
    VHOST_USER_SET_VRING_ENDIAN = 23,
    VHOST_USER_GET_CONFIG = 24,
    VHOST_USER_SET_CONFIG = 25,
    VHOST_USER_CREATE_CRYPTO_SESSION = 26,
    VHOST_USER_CLOSE_CRYPTO_SESSION = 27,
    VHOST_USER_POSTCOPY_ADVISE  = 28,
    VHOST_USER_POSTCOPY_LISTEN  = 29,
    VHOST_USER_MAX
} VhostUserRequest;

typedef enum VhostUserSlaveRequest {
    VHOST_USER_SLAVE_NONE = 0,
    VHOST_USER_SLAVE_IOTLB_MSG = 1,
    VHOST_USER_SLAVE_CONFIG_CHANGE_MSG = 2,
    VHOST_USER_SLAVE_MAX
}  VhostUserSlaveRequest;

typedef struct VhostUserMemoryRegion {
    uint64_t guest_phys_addr;
    uint64_t memory_size;
    uint64_t userspace_addr;
    uint64_t mmap_offset;
} VhostUserMemoryRegion;

typedef struct VhostUserMemory {
    uint32_t nregions;
    uint32_t padding;
    VhostUserMemoryRegion regions[VHOST_MEMORY_MAX_NREGIONS];
} VhostUserMemory;

typedef struct VhostUserLog {
    uint64_t mmap_size;
    uint64_t mmap_offset;
} VhostUserLog;

typedef struct VhostUserConfig {
    uint32_t offset;
    uint32_t size;
    uint32_t flags;
    uint8_t region[VHOST_USER_MAX_CONFIG_SIZE];
} VhostUserConfig;

#define VHOST_CRYPTO_SYM_HMAC_MAX_KEY_LEN    512
#define VHOST_CRYPTO_SYM_CIPHER_MAX_KEY_LEN  64

typedef struct VhostUserCryptoSession {
    /* session id for success, -1 on errors */
    int64_t session_id;
    CryptoDevBackendSymSessionInfo session_setup_data;
    uint8_t key[VHOST_CRYPTO_SYM_CIPHER_MAX_KEY_LEN];
    uint8_t auth_key[VHOST_CRYPTO_SYM_HMAC_MAX_KEY_LEN];
} VhostUserCryptoSession;

static VhostUserConfig c __attribute__ ((unused));
#define VHOST_USER_CONFIG_HDR_SIZE (sizeof(c.offset) \
                                   + sizeof(c.size) \
                                   + sizeof(c.flags))

typedef struct {
    VhostUserRequest request;

#define VHOST_USER_VERSION_MASK     (0x3)
#define VHOST_USER_REPLY_MASK       (0x1<<2)
#define VHOST_USER_NEED_REPLY_MASK  (0x1 << 3)
    uint32_t flags;
    uint32_t size; /* the following payload size */
} QEMU_PACKED VhostUserHeader;

typedef union {
#define VHOST_USER_VRING_IDX_MASK   (0xff)
#define VHOST_USER_VRING_NOFD_MASK  (0x1<<8)
        uint64_t u64;
        struct vhost_vring_state state;
        struct vhost_vring_addr addr;
        VhostUserMemory memory;
        VhostUserLog log;
        struct vhost_iotlb_msg iotlb;
        VhostUserConfig config;
        VhostUserCryptoSession session;
} VhostUserPayload;

typedef struct VhostUserMsg {
    VhostUserHeader hdr;
    VhostUserPayload payload;
} QEMU_PACKED VhostUserMsg;

static VhostUserMsg m __attribute__ ((unused));
#define VHOST_USER_HDR_SIZE (sizeof(VhostUserHeader))

#define VHOST_USER_PAYLOAD_SIZE (sizeof(VhostUserPayload))

/* The version of the protocol we support */
#define VHOST_USER_VERSION    (0x1)

struct vhost_user {
    struct vhost_dev *dev;
    CharBackend *chr;
    int slave_fd;
    NotifierWithReturn postcopy_notifier;
    struct PostCopyFD  postcopy_fd;
    uint64_t           postcopy_client_bases[VHOST_MEMORY_MAX_NREGIONS];
    /* True once we've entered postcopy_listen */
    bool               postcopy_listen;
};

static bool ioeventfd_enabled(void)
{
    return kvm_enabled() && kvm_eventfds_enabled();
}

static int vhost_user_read(struct vhost_dev *dev, VhostUserMsg *msg)
{
    struct vhost_user *u = dev->opaque;
    CharBackend *chr = u->chr;
    uint8_t *p = (uint8_t *) msg;
    int r, size = VHOST_USER_HDR_SIZE;

    r = qemu_chr_fe_read_all(chr, p, size);
    if (r != size) {
        error_report("Failed to read msg header. Read %d instead of %d."
                     " Original request %d.", r, size, msg->hdr.request);
        goto fail;
    }

    /* validate received flags */
    if (msg->hdr.flags != (VHOST_USER_REPLY_MASK | VHOST_USER_VERSION)) {
        error_report("Failed to read msg header."
                " Flags 0x%x instead of 0x%x.", msg->hdr.flags,
                VHOST_USER_REPLY_MASK | VHOST_USER_VERSION);
        goto fail;
    }

    /* validate message size is sane */
    if (msg->hdr.size > VHOST_USER_PAYLOAD_SIZE) {
        error_report("Failed to read msg header."
                " Size %d exceeds the maximum %zu.", msg->hdr.size,
                VHOST_USER_PAYLOAD_SIZE);
        goto fail;
    }

    if (msg->hdr.size) {
        p += VHOST_USER_HDR_SIZE;
        size = msg->hdr.size;
        r = qemu_chr_fe_read_all(chr, p, size);
        if (r != size) {
            error_report("Failed to read msg payload."
                         " Read %d instead of %d.", r, msg->hdr.size);
            goto fail;
        }
    }

    return 0;

fail:
    return -1;
}

static int process_message_reply(struct vhost_dev *dev,
                                 const VhostUserMsg *msg)
{
    VhostUserMsg msg_reply;

    if ((msg->hdr.flags & VHOST_USER_NEED_REPLY_MASK) == 0) {
        return 0;
    }

    if (vhost_user_read(dev, &msg_reply) < 0) {
        return -1;
    }

    if (msg_reply.hdr.request != msg->hdr.request) {
        error_report("Received unexpected msg type."
                     "Expected %d received %d",
                     msg->hdr.request, msg_reply.hdr.request);
        return -1;
    }

    return msg_reply.payload.u64 ? -1 : 0;
}

static bool vhost_user_one_time_request(VhostUserRequest request)
{
    switch (request) {
    case VHOST_USER_SET_OWNER:
    case VHOST_USER_RESET_OWNER:
    case VHOST_USER_SET_MEM_TABLE:
    case VHOST_USER_GET_QUEUE_NUM:
    case VHOST_USER_NET_SET_MTU:
        return true;
    default:
        return false;
    }
}

/* most non-init callers ignore the error */
static int vhost_user_write(struct vhost_dev *dev, VhostUserMsg *msg,
                            int *fds, int fd_num)
{
    struct vhost_user *u = dev->opaque;
    CharBackend *chr = u->chr;
    int ret, size = VHOST_USER_HDR_SIZE + msg->hdr.size;

    /*
     * For non-vring specific requests, like VHOST_USER_SET_MEM_TABLE,
     * we just need send it once in the first time. For later such
     * request, we just ignore it.
     */
    if (vhost_user_one_time_request(msg->hdr.request) && dev->vq_index != 0) {
        msg->hdr.flags &= ~VHOST_USER_NEED_REPLY_MASK;
        return 0;
    }

    if (qemu_chr_fe_set_msgfds(chr, fds, fd_num) < 0) {
        error_report("Failed to set msg fds.");
        return -1;
    }

    ret = qemu_chr_fe_write_all(chr, (const uint8_t *) msg, size);
    if (ret != size) {
        error_report("Failed to write msg."
                     " Wrote %d instead of %d.", ret, size);
        return -1;
    }

    return 0;
}

static int vhost_user_set_log_base(struct vhost_dev *dev, uint64_t base,
                                   struct vhost_log *log)
{
    int fds[VHOST_MEMORY_MAX_NREGIONS];
    size_t fd_num = 0;
    bool shmfd = virtio_has_feature(dev->protocol_features,
                                    VHOST_USER_PROTOCOL_F_LOG_SHMFD);
    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_SET_LOG_BASE,
        .hdr.flags = VHOST_USER_VERSION,
        .payload.log.mmap_size = log->size * sizeof(*(log->log)),
        .payload.log.mmap_offset = 0,
        .hdr.size = sizeof(msg.payload.log),
    };

    if (shmfd && log->fd != -1) {
        fds[fd_num++] = log->fd;
    }

    if (vhost_user_write(dev, &msg, fds, fd_num) < 0) {
        return -1;
    }

    if (shmfd) {
        msg.hdr.size = 0;
        if (vhost_user_read(dev, &msg) < 0) {
            return -1;
        }

        if (msg.hdr.request != VHOST_USER_SET_LOG_BASE) {
            error_report("Received unexpected msg type. "
                         "Expected %d received %d",
                         VHOST_USER_SET_LOG_BASE, msg.hdr.request);
            return -1;
        }
    }

    return 0;
}

static int vhost_user_set_mem_table_postcopy(struct vhost_dev *dev,
                                             struct vhost_memory *mem)
{
    struct vhost_user *u = dev->opaque;
    int fds[VHOST_MEMORY_MAX_NREGIONS];
    int i, fd;
    size_t fd_num = 0;
    bool reply_supported = virtio_has_feature(dev->protocol_features,
                                              VHOST_USER_PROTOCOL_F_REPLY_ACK);
    VhostUserMsg msg_reply;
    int region_i, msg_i;

    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_SET_MEM_TABLE,
        .hdr.flags = VHOST_USER_VERSION,
    };

    if (reply_supported) {
        msg.hdr.flags |= VHOST_USER_NEED_REPLY_MASK;
    }

    for (i = 0; i < dev->mem->nregions; ++i) {
        struct vhost_memory_region *reg = dev->mem->regions + i;
        ram_addr_t offset;
        MemoryRegion *mr;

        assert((uintptr_t)reg->userspace_addr == reg->userspace_addr);
        mr = memory_region_from_host((void *)(uintptr_t)reg->userspace_addr,
                                     &offset);
        fd = memory_region_get_fd(mr);
        if (fd > 0) {
            msg.payload.memory.regions[fd_num].userspace_addr =
                reg->userspace_addr;
            msg.payload.memory.regions[fd_num].memory_size  = reg->memory_size;
            msg.payload.memory.regions[fd_num].guest_phys_addr =
                reg->guest_phys_addr;
            msg.payload.memory.regions[fd_num].mmap_offset = offset;
            assert(fd_num < VHOST_MEMORY_MAX_NREGIONS);
            fds[fd_num++] = fd;
        }
    }

    msg.payload.memory.nregions = fd_num;

    if (!fd_num) {
        error_report("Failed initializing vhost-user memory map, "
                     "consider using -object memory-backend-file share=on");
        return -1;
    }

    msg.hdr.size = sizeof(msg.payload.memory.nregions);
    msg.hdr.size += sizeof(msg.payload.memory.padding);
    msg.hdr.size += fd_num * sizeof(VhostUserMemoryRegion);

    if (vhost_user_write(dev, &msg, fds, fd_num) < 0) {
        return -1;
    }

    if (vhost_user_read(dev, &msg_reply) < 0) {
        return -1;
    }

    if (msg_reply.hdr.request != VHOST_USER_SET_MEM_TABLE) {
        error_report("%s: Received unexpected msg type."
                     "Expected %d received %d", __func__,
                     VHOST_USER_SET_MEM_TABLE, msg_reply.hdr.request);
        return -1;
    }
    /* We're using the same structure, just reusing one of the
     * fields, so it should be the same size.
     */
    if (msg_reply.hdr.size != msg.hdr.size) {
        error_report("%s: Unexpected size for postcopy reply "
                     "%d vs %d", __func__, msg_reply.hdr.size, msg.hdr.size);
        return -1;
    }

    memset(u->postcopy_client_bases, 0,
           sizeof(uint64_t) * VHOST_MEMORY_MAX_NREGIONS);

    /* They're in the same order as the regions that were sent
     * but some of the regions were skipped (above) if they
     * didn't have fd's
    */
    for (msg_i = 0, region_i = 0;
         region_i < dev->mem->nregions;
        region_i++) {
        if (msg_i < fd_num &&
            msg_reply.payload.memory.regions[msg_i].guest_phys_addr ==
            dev->mem->regions[region_i].guest_phys_addr) {
            u->postcopy_client_bases[region_i] =
                msg_reply.payload.memory.regions[msg_i].userspace_addr;
            trace_vhost_user_set_mem_table_postcopy(
                msg_reply.payload.memory.regions[msg_i].userspace_addr,
                msg.payload.memory.regions[msg_i].userspace_addr,
                msg_i, region_i);
            msg_i++;
        }
    }
    if (msg_i != fd_num) {
        error_report("%s: postcopy reply not fully consumed "
                     "%d vs %zd",
                     __func__, msg_i, fd_num);
        return -1;
    }
    /* Now we've registered this with the postcopy code, we ack to the client,
     * because now we're in the position to be able to deal with any faults
     * it generates.
     */
    /* TODO: Use this for failure cases as well with a bad value */
    msg.hdr.size = sizeof(msg.payload.u64);
    msg.payload.u64 = 0; /* OK */
    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        return -1;
    }

    if (reply_supported) {
        return process_message_reply(dev, &msg);
    }

    return 0;
}

static int vhost_user_set_mem_table(struct vhost_dev *dev,
                                    struct vhost_memory *mem)
{
    struct vhost_user *u = dev->opaque;
    int fds[VHOST_MEMORY_MAX_NREGIONS];
    int i, fd;
    size_t fd_num = 0;
    bool do_postcopy = u->postcopy_listen && u->postcopy_fd.handler;
    bool reply_supported = virtio_has_feature(dev->protocol_features,
                                          VHOST_USER_PROTOCOL_F_REPLY_ACK) &&
                                          !do_postcopy;

    if (do_postcopy) {
        /* Postcopy has enough differences that it's best done in it's own
         * version
         */
        return vhost_user_set_mem_table_postcopy(dev, mem);
    }

    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_SET_MEM_TABLE,
        .hdr.flags = VHOST_USER_VERSION,
    };

    if (reply_supported) {
        msg.hdr.flags |= VHOST_USER_NEED_REPLY_MASK;
    }

    for (i = 0; i < dev->mem->nregions; ++i) {
        struct vhost_memory_region *reg = dev->mem->regions + i;
        ram_addr_t offset;
        MemoryRegion *mr;

        assert((uintptr_t)reg->userspace_addr == reg->userspace_addr);
        mr = memory_region_from_host((void *)(uintptr_t)reg->userspace_addr,
                                     &offset);
        fd = memory_region_get_fd(mr);
        if (fd > 0) {
            if (fd_num == VHOST_MEMORY_MAX_NREGIONS) {
                error_report("Failed preparing vhost-user memory table msg");
                return -1;
            }
            msg.payload.memory.regions[fd_num].userspace_addr =
                reg->userspace_addr;
            msg.payload.memory.regions[fd_num].memory_size  = reg->memory_size;
            msg.payload.memory.regions[fd_num].guest_phys_addr =
                reg->guest_phys_addr;
            msg.payload.memory.regions[fd_num].mmap_offset = offset;
            fds[fd_num++] = fd;
        }
    }

    msg.payload.memory.nregions = fd_num;

    if (!fd_num) {
        error_report("Failed initializing vhost-user memory map, "
                     "consider using -object memory-backend-file share=on");
        return -1;
    }

    msg.hdr.size = sizeof(msg.payload.memory.nregions);
    msg.hdr.size += sizeof(msg.payload.memory.padding);
    msg.hdr.size += fd_num * sizeof(VhostUserMemoryRegion);

    if (vhost_user_write(dev, &msg, fds, fd_num) < 0) {
        return -1;
    }

    if (reply_supported) {
        return process_message_reply(dev, &msg);
    }

    return 0;
}

static int vhost_user_set_vring_addr(struct vhost_dev *dev,
                                     struct vhost_vring_addr *addr)
{
    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_SET_VRING_ADDR,
        .hdr.flags = VHOST_USER_VERSION,
        .payload.addr = *addr,
        .hdr.size = sizeof(msg.payload.addr),
    };

    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        return -1;
    }

    return 0;
}

static int vhost_user_set_vring_endian(struct vhost_dev *dev,
                                       struct vhost_vring_state *ring)
{
    bool cross_endian = virtio_has_feature(dev->protocol_features,
                                           VHOST_USER_PROTOCOL_F_CROSS_ENDIAN);
    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_SET_VRING_ENDIAN,
        .hdr.flags = VHOST_USER_VERSION,
        .payload.state = *ring,
        .hdr.size = sizeof(msg.payload.state),
    };

    if (!cross_endian) {
        error_report("vhost-user trying to send unhandled ioctl");
        return -1;
    }

    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        return -1;
    }

    return 0;
}

static int vhost_set_vring(struct vhost_dev *dev,
                           unsigned long int request,
                           struct vhost_vring_state *ring)
{
    VhostUserMsg msg = {
        .hdr.request = request,
        .hdr.flags = VHOST_USER_VERSION,
        .payload.state = *ring,
        .hdr.size = sizeof(msg.payload.state),
    };

    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        return -1;
    }

    return 0;
}

static int vhost_user_set_vring_num(struct vhost_dev *dev,
                                    struct vhost_vring_state *ring)
{
    return vhost_set_vring(dev, VHOST_USER_SET_VRING_NUM, ring);
}

static int vhost_user_set_vring_base(struct vhost_dev *dev,
                                     struct vhost_vring_state *ring)
{
    return vhost_set_vring(dev, VHOST_USER_SET_VRING_BASE, ring);
}

static int vhost_user_set_vring_enable(struct vhost_dev *dev, int enable)
{
    int i;

    if (!virtio_has_feature(dev->features, VHOST_USER_F_PROTOCOL_FEATURES)) {
        return -1;
    }

    for (i = 0; i < dev->nvqs; ++i) {
        struct vhost_vring_state state = {
            .index = dev->vq_index + i,
            .num   = enable,
        };

        vhost_set_vring(dev, VHOST_USER_SET_VRING_ENABLE, &state);
    }

    return 0;
}

static int vhost_user_get_vring_base(struct vhost_dev *dev,
                                     struct vhost_vring_state *ring)
{
    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_GET_VRING_BASE,
        .hdr.flags = VHOST_USER_VERSION,
        .payload.state = *ring,
        .hdr.size = sizeof(msg.payload.state),
    };

    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        return -1;
    }

    if (vhost_user_read(dev, &msg) < 0) {
        return -1;
    }

    if (msg.hdr.request != VHOST_USER_GET_VRING_BASE) {
        error_report("Received unexpected msg type. Expected %d received %d",
                     VHOST_USER_GET_VRING_BASE, msg.hdr.request);
        return -1;
    }

    if (msg.hdr.size != sizeof(msg.payload.state)) {
        error_report("Received bad msg size.");
        return -1;
    }

    *ring = msg.payload.state;

    return 0;
}

static int vhost_set_vring_file(struct vhost_dev *dev,
                                VhostUserRequest request,
                                struct vhost_vring_file *file)
{
    int fds[VHOST_MEMORY_MAX_NREGIONS];
    size_t fd_num = 0;
    VhostUserMsg msg = {
        .hdr.request = request,
        .hdr.flags = VHOST_USER_VERSION,
        .payload.u64 = file->index & VHOST_USER_VRING_IDX_MASK,
        .hdr.size = sizeof(msg.payload.u64),
    };

    if (ioeventfd_enabled() && file->fd > 0) {
        fds[fd_num++] = file->fd;
    } else {
        msg.payload.u64 |= VHOST_USER_VRING_NOFD_MASK;
    }

    if (vhost_user_write(dev, &msg, fds, fd_num) < 0) {
        return -1;
    }

    return 0;
}

static int vhost_user_set_vring_kick(struct vhost_dev *dev,
                                     struct vhost_vring_file *file)
{
    return vhost_set_vring_file(dev, VHOST_USER_SET_VRING_KICK, file);
}

static int vhost_user_set_vring_call(struct vhost_dev *dev,
                                     struct vhost_vring_file *file)
{
    return vhost_set_vring_file(dev, VHOST_USER_SET_VRING_CALL, file);
}

static int vhost_user_set_u64(struct vhost_dev *dev, int request, uint64_t u64)
{
    VhostUserMsg msg = {
        .hdr.request = request,
        .hdr.flags = VHOST_USER_VERSION,
        .payload.u64 = u64,
        .hdr.size = sizeof(msg.payload.u64),
    };

    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        return -1;
    }

    return 0;
}

static int vhost_user_set_features(struct vhost_dev *dev,
                                   uint64_t features)
{
    return vhost_user_set_u64(dev, VHOST_USER_SET_FEATURES, features);
}

static int vhost_user_set_protocol_features(struct vhost_dev *dev,
                                            uint64_t features)
{
    return vhost_user_set_u64(dev, VHOST_USER_SET_PROTOCOL_FEATURES, features);
}

static int vhost_user_get_u64(struct vhost_dev *dev, int request, uint64_t *u64)
{
    VhostUserMsg msg = {
        .hdr.request = request,
        .hdr.flags = VHOST_USER_VERSION,
    };

    if (vhost_user_one_time_request(request) && dev->vq_index != 0) {
        return 0;
    }

    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        return -1;
    }

    if (vhost_user_read(dev, &msg) < 0) {
        return -1;
    }

    if (msg.hdr.request != request) {
        error_report("Received unexpected msg type. Expected %d received %d",
                     request, msg.hdr.request);
        return -1;
    }

    if (msg.hdr.size != sizeof(msg.payload.u64)) {
        error_report("Received bad msg size.");
        return -1;
    }

    *u64 = msg.payload.u64;

    return 0;
}

static int vhost_user_get_features(struct vhost_dev *dev, uint64_t *features)
{
    return vhost_user_get_u64(dev, VHOST_USER_GET_FEATURES, features);
}

static int vhost_user_set_owner(struct vhost_dev *dev)
{
    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_SET_OWNER,
        .hdr.flags = VHOST_USER_VERSION,
    };

    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        return -1;
    }

    return 0;
}

static int vhost_user_reset_device(struct vhost_dev *dev)
{
    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_RESET_OWNER,
        .hdr.flags = VHOST_USER_VERSION,
    };

    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        return -1;
    }

    return 0;
}

static int vhost_user_slave_handle_config_change(struct vhost_dev *dev)
{
    int ret = -1;

    if (!dev->config_ops) {
        return -1;
    }

    if (dev->config_ops->vhost_dev_config_notifier) {
        ret = dev->config_ops->vhost_dev_config_notifier(dev);
    }

    return ret;
}

static void slave_read(void *opaque)
{
    struct vhost_dev *dev = opaque;
    struct vhost_user *u = dev->opaque;
    VhostUserHeader hdr = { 0, };
    VhostUserPayload payload = { 0, };
    int size, ret = 0;

    /* Read header */
    size = read(u->slave_fd, &hdr, VHOST_USER_HDR_SIZE);
    if (size != VHOST_USER_HDR_SIZE) {
        error_report("Failed to read from slave.");
        goto err;
    }

    if (hdr.size > VHOST_USER_PAYLOAD_SIZE) {
        error_report("Failed to read msg header."
                " Size %d exceeds the maximum %zu.", hdr.size,
                VHOST_USER_PAYLOAD_SIZE);
        goto err;
    }

    /* Read payload */
    size = read(u->slave_fd, &payload, hdr.size);
    if (size != hdr.size) {
        error_report("Failed to read payload from slave.");
        goto err;
    }

    switch (hdr.request) {
    case VHOST_USER_SLAVE_IOTLB_MSG:
        ret = vhost_backend_handle_iotlb_msg(dev, &payload.iotlb);
        break;
    case VHOST_USER_SLAVE_CONFIG_CHANGE_MSG :
        ret = vhost_user_slave_handle_config_change(dev);
        break;
    default:
        error_report("Received unexpected msg type.");
        ret = -EINVAL;
    }

    /*
     * REPLY_ACK feature handling. Other reply types has to be managed
     * directly in their request handlers.
     */
    if (hdr.flags & VHOST_USER_NEED_REPLY_MASK) {
        struct iovec iovec[2];


        hdr.flags &= ~VHOST_USER_NEED_REPLY_MASK;
        hdr.flags |= VHOST_USER_REPLY_MASK;

        payload.u64 = !!ret;
        hdr.size = sizeof(payload.u64);

        iovec[0].iov_base = &hdr;
        iovec[0].iov_len = VHOST_USER_HDR_SIZE;
        iovec[1].iov_base = &payload;
        iovec[1].iov_len = hdr.size;

        size = writev(u->slave_fd, iovec, ARRAY_SIZE(iovec));
        if (size != VHOST_USER_HDR_SIZE + hdr.size) {
            error_report("Failed to send msg reply to slave.");
            goto err;
        }
    }

    return;

err:
    qemu_set_fd_handler(u->slave_fd, NULL, NULL, NULL);
    close(u->slave_fd);
    u->slave_fd = -1;
    return;
}

static int vhost_setup_slave_channel(struct vhost_dev *dev)
{
    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_SET_SLAVE_REQ_FD,
        .hdr.flags = VHOST_USER_VERSION,
    };
    struct vhost_user *u = dev->opaque;
    int sv[2], ret = 0;
    bool reply_supported = virtio_has_feature(dev->protocol_features,
                                              VHOST_USER_PROTOCOL_F_REPLY_ACK);

    if (!virtio_has_feature(dev->protocol_features,
                            VHOST_USER_PROTOCOL_F_SLAVE_REQ)) {
        return 0;
    }

    if (socketpair(PF_UNIX, SOCK_STREAM, 0, sv) == -1) {
        error_report("socketpair() failed");
        return -1;
    }

    u->slave_fd = sv[0];
    qemu_set_fd_handler(u->slave_fd, slave_read, NULL, dev);

    if (reply_supported) {
        msg.hdr.flags |= VHOST_USER_NEED_REPLY_MASK;
    }

    ret = vhost_user_write(dev, &msg, &sv[1], 1);
    if (ret) {
        goto out;
    }

    if (reply_supported) {
        ret = process_message_reply(dev, &msg);
    }

out:
    close(sv[1]);
    if (ret) {
        qemu_set_fd_handler(u->slave_fd, NULL, NULL, NULL);
        close(u->slave_fd);
        u->slave_fd = -1;
    }

    return ret;
}

/*
 * Called back from the postcopy fault thread when a fault is received on our
 * ufd.
 * TODO: This is Linux specific
 */
static int vhost_user_postcopy_fault_handler(struct PostCopyFD *pcfd,
                                             void *ufd)
{
    return 0;
}

/*
 * Called at the start of an inbound postcopy on reception of the
 * 'advise' command.
 */
static int vhost_user_postcopy_advise(struct vhost_dev *dev, Error **errp)
{
    struct vhost_user *u = dev->opaque;
    CharBackend *chr = u->chr;
    int ufd;
    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_POSTCOPY_ADVISE,
        .hdr.flags = VHOST_USER_VERSION,
    };

    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        error_setg(errp, "Failed to send postcopy_advise to vhost");
        return -1;
    }

    if (vhost_user_read(dev, &msg) < 0) {
        error_setg(errp, "Failed to get postcopy_advise reply from vhost");
        return -1;
    }

    if (msg.hdr.request != VHOST_USER_POSTCOPY_ADVISE) {
        error_setg(errp, "Unexpected msg type. Expected %d received %d",
                     VHOST_USER_POSTCOPY_ADVISE, msg.hdr.request);
        return -1;
    }

    if (msg.hdr.size) {
        error_setg(errp, "Received bad msg size.");
        return -1;
    }
    ufd = qemu_chr_fe_get_msgfd(chr);
    if (ufd < 0) {
        error_setg(errp, "%s: Failed to get ufd", __func__);
        return -1;
    }
    fcntl(ufd, F_SETFL, O_NONBLOCK);

    /* register ufd with userfault thread */
    u->postcopy_fd.fd = ufd;
    u->postcopy_fd.data = dev;
    u->postcopy_fd.handler = vhost_user_postcopy_fault_handler;
    u->postcopy_fd.idstr = "vhost-user"; /* Need to find unique name */
    postcopy_register_shared_ufd(&u->postcopy_fd);
    return 0;
}

/*
 * Called at the switch to postcopy on reception of the 'listen' command.
 */
static int vhost_user_postcopy_listen(struct vhost_dev *dev, Error **errp)
{
    struct vhost_user *u = dev->opaque;
    int ret;
    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_POSTCOPY_LISTEN,
        .hdr.flags = VHOST_USER_VERSION | VHOST_USER_NEED_REPLY_MASK,
    };
    u->postcopy_listen = true;
    trace_vhost_user_postcopy_listen();
    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        error_setg(errp, "Failed to send postcopy_listen to vhost");
        return -1;
    }

    ret = process_message_reply(dev, &msg);
    if (ret) {
        error_setg(errp, "Failed to receive reply to postcopy_listen");
        return ret;
    }

    return 0;
}

static int vhost_user_postcopy_notifier(NotifierWithReturn *notifier,
                                        void *opaque)
{
    struct PostcopyNotifyData *pnd = opaque;
    struct vhost_user *u = container_of(notifier, struct vhost_user,
                                         postcopy_notifier);
    struct vhost_dev *dev = u->dev;

    switch (pnd->reason) {
    case POSTCOPY_NOTIFY_PROBE:
        if (!virtio_has_feature(dev->protocol_features,
                                VHOST_USER_PROTOCOL_F_PAGEFAULT)) {
            /* TODO: Get the device name into this error somehow */
            error_setg(pnd->errp,
                       "vhost-user backend not capable of postcopy");
            return -ENOENT;
        }
        break;

    case POSTCOPY_NOTIFY_INBOUND_ADVISE:
        return vhost_user_postcopy_advise(dev, pnd->errp);

    case POSTCOPY_NOTIFY_INBOUND_LISTEN:
        return vhost_user_postcopy_listen(dev, pnd->errp);

    default:
        /* We ignore notifications we don't know */
        break;
    }

    return 0;
}

static int vhost_user_init(struct vhost_dev *dev, void *opaque)
{
    uint64_t features, protocol_features;
    struct vhost_user *u;
    int err;

    assert(dev->vhost_ops->backend_type == VHOST_BACKEND_TYPE_USER);

    u = g_new0(struct vhost_user, 1);
    u->chr = opaque;
    u->slave_fd = -1;
    u->dev = dev;
    dev->opaque = u;

    err = vhost_user_get_features(dev, &features);
    if (err < 0) {
        return err;
    }

    if (virtio_has_feature(features, VHOST_USER_F_PROTOCOL_FEATURES)) {
        dev->backend_features |= 1ULL << VHOST_USER_F_PROTOCOL_FEATURES;

        err = vhost_user_get_u64(dev, VHOST_USER_GET_PROTOCOL_FEATURES,
                                 &protocol_features);
        if (err < 0) {
            return err;
        }

        dev->protocol_features =
            protocol_features & VHOST_USER_PROTOCOL_FEATURE_MASK;
        err = vhost_user_set_protocol_features(dev, dev->protocol_features);
        if (err < 0) {
            return err;
        }

        /* query the max queues we support if backend supports Multiple Queue */
        if (dev->protocol_features & (1ULL << VHOST_USER_PROTOCOL_F_MQ)) {
            err = vhost_user_get_u64(dev, VHOST_USER_GET_QUEUE_NUM,
                                     &dev->max_queues);
            if (err < 0) {
                return err;
            }
        }

        if (virtio_has_feature(features, VIRTIO_F_IOMMU_PLATFORM) &&
                !(virtio_has_feature(dev->protocol_features,
                    VHOST_USER_PROTOCOL_F_SLAVE_REQ) &&
                 virtio_has_feature(dev->protocol_features,
                    VHOST_USER_PROTOCOL_F_REPLY_ACK))) {
            error_report("IOMMU support requires reply-ack and "
                         "slave-req protocol features.");
            return -1;
        }
    }

    if (dev->migration_blocker == NULL &&
        !virtio_has_feature(dev->protocol_features,
                            VHOST_USER_PROTOCOL_F_LOG_SHMFD)) {
        error_setg(&dev->migration_blocker,
                   "Migration disabled: vhost-user backend lacks "
                   "VHOST_USER_PROTOCOL_F_LOG_SHMFD feature.");
    }

    err = vhost_setup_slave_channel(dev);
    if (err < 0) {
        return err;
    }

    u->postcopy_notifier.notify = vhost_user_postcopy_notifier;
    postcopy_add_notifier(&u->postcopy_notifier);

    return 0;
}

static int vhost_user_cleanup(struct vhost_dev *dev)
{
    struct vhost_user *u;

    assert(dev->vhost_ops->backend_type == VHOST_BACKEND_TYPE_USER);

    u = dev->opaque;
    if (u->postcopy_notifier.notify) {
        postcopy_remove_notifier(&u->postcopy_notifier);
        u->postcopy_notifier.notify = NULL;
    }
    if (u->slave_fd >= 0) {
        qemu_set_fd_handler(u->slave_fd, NULL, NULL, NULL);
        close(u->slave_fd);
        u->slave_fd = -1;
    }
    g_free(u);
    dev->opaque = 0;

    return 0;
}

static int vhost_user_get_vq_index(struct vhost_dev *dev, int idx)
{
    assert(idx >= dev->vq_index && idx < dev->vq_index + dev->nvqs);

    return idx;
}

static int vhost_user_memslots_limit(struct vhost_dev *dev)
{
    return VHOST_MEMORY_MAX_NREGIONS;
}

static bool vhost_user_requires_shm_log(struct vhost_dev *dev)
{
    assert(dev->vhost_ops->backend_type == VHOST_BACKEND_TYPE_USER);

    return virtio_has_feature(dev->protocol_features,
                              VHOST_USER_PROTOCOL_F_LOG_SHMFD);
}

static int vhost_user_migration_done(struct vhost_dev *dev, char* mac_addr)
{
    VhostUserMsg msg = { 0 };

    assert(dev->vhost_ops->backend_type == VHOST_BACKEND_TYPE_USER);

    /* If guest supports GUEST_ANNOUNCE do nothing */
    if (virtio_has_feature(dev->acked_features, VIRTIO_NET_F_GUEST_ANNOUNCE)) {
        return 0;
    }

    /* if backend supports VHOST_USER_PROTOCOL_F_RARP ask it to send the RARP */
    if (virtio_has_feature(dev->protocol_features,
                           VHOST_USER_PROTOCOL_F_RARP)) {
        msg.hdr.request = VHOST_USER_SEND_RARP;
        msg.hdr.flags = VHOST_USER_VERSION;
        memcpy((char *)&msg.payload.u64, mac_addr, 6);
        msg.hdr.size = sizeof(msg.payload.u64);

        return vhost_user_write(dev, &msg, NULL, 0);
    }
    return -1;
}

static bool vhost_user_can_merge(struct vhost_dev *dev,
                                 uint64_t start1, uint64_t size1,
                                 uint64_t start2, uint64_t size2)
{
    ram_addr_t offset;
    int mfd, rfd;
    MemoryRegion *mr;

    mr = memory_region_from_host((void *)(uintptr_t)start1, &offset);
    mfd = memory_region_get_fd(mr);

    mr = memory_region_from_host((void *)(uintptr_t)start2, &offset);
    rfd = memory_region_get_fd(mr);

    return mfd == rfd;
}

static int vhost_user_net_set_mtu(struct vhost_dev *dev, uint16_t mtu)
{
    VhostUserMsg msg;
    bool reply_supported = virtio_has_feature(dev->protocol_features,
                                              VHOST_USER_PROTOCOL_F_REPLY_ACK);

    if (!(dev->protocol_features & (1ULL << VHOST_USER_PROTOCOL_F_NET_MTU))) {
        return 0;
    }

    msg.hdr.request = VHOST_USER_NET_SET_MTU;
    msg.payload.u64 = mtu;
    msg.hdr.size = sizeof(msg.payload.u64);
    msg.hdr.flags = VHOST_USER_VERSION;
    if (reply_supported) {
        msg.hdr.flags |= VHOST_USER_NEED_REPLY_MASK;
    }

    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        return -1;
    }

    /* If reply_ack supported, slave has to ack specified MTU is valid */
    if (reply_supported) {
        return process_message_reply(dev, &msg);
    }

    return 0;
}

static int vhost_user_send_device_iotlb_msg(struct vhost_dev *dev,
                                            struct vhost_iotlb_msg *imsg)
{
    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_IOTLB_MSG,
        .hdr.size = sizeof(msg.payload.iotlb),
        .hdr.flags = VHOST_USER_VERSION | VHOST_USER_NEED_REPLY_MASK,
        .payload.iotlb = *imsg,
    };

    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        return -EFAULT;
    }

    return process_message_reply(dev, &msg);
}


static void vhost_user_set_iotlb_callback(struct vhost_dev *dev, int enabled)
{
    /* No-op as the receive channel is not dedicated to IOTLB messages. */
}

static int vhost_user_get_config(struct vhost_dev *dev, uint8_t *config,
                                 uint32_t config_len)
{
    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_GET_CONFIG,
        .hdr.flags = VHOST_USER_VERSION,
        .hdr.size = VHOST_USER_CONFIG_HDR_SIZE + config_len,
    };

    if (config_len > VHOST_USER_MAX_CONFIG_SIZE) {
        return -1;
    }

    msg.payload.config.offset = 0;
    msg.payload.config.size = config_len;
    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        return -1;
    }

    if (vhost_user_read(dev, &msg) < 0) {
        return -1;
    }

    if (msg.hdr.request != VHOST_USER_GET_CONFIG) {
        error_report("Received unexpected msg type. Expected %d received %d",
                     VHOST_USER_GET_CONFIG, msg.hdr.request);
        return -1;
    }

    if (msg.hdr.size != VHOST_USER_CONFIG_HDR_SIZE + config_len) {
        error_report("Received bad msg size.");
        return -1;
    }

    memcpy(config, msg.payload.config.region, config_len);

    return 0;
}

static int vhost_user_set_config(struct vhost_dev *dev, const uint8_t *data,
                                 uint32_t offset, uint32_t size, uint32_t flags)
{
    uint8_t *p;
    bool reply_supported = virtio_has_feature(dev->protocol_features,
                                              VHOST_USER_PROTOCOL_F_REPLY_ACK);

    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_SET_CONFIG,
        .hdr.flags = VHOST_USER_VERSION,
        .hdr.size = VHOST_USER_CONFIG_HDR_SIZE + size,
    };

    if (reply_supported) {
        msg.hdr.flags |= VHOST_USER_NEED_REPLY_MASK;
    }

    if (size > VHOST_USER_MAX_CONFIG_SIZE) {
        return -1;
    }

    msg.payload.config.offset = offset,
    msg.payload.config.size = size,
    msg.payload.config.flags = flags,
    p = msg.payload.config.region;
    memcpy(p, data, size);

    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        return -1;
    }

    if (reply_supported) {
        return process_message_reply(dev, &msg);
    }

    return 0;
}

static int vhost_user_crypto_create_session(struct vhost_dev *dev,
                                            void *session_info,
                                            uint64_t *session_id)
{
    bool crypto_session = virtio_has_feature(dev->protocol_features,
                                       VHOST_USER_PROTOCOL_F_CRYPTO_SESSION);
    CryptoDevBackendSymSessionInfo *sess_info = session_info;
    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_CREATE_CRYPTO_SESSION,
        .hdr.flags = VHOST_USER_VERSION,
        .hdr.size = sizeof(msg.payload.session),
    };

    assert(dev->vhost_ops->backend_type == VHOST_BACKEND_TYPE_USER);

    if (!crypto_session) {
        error_report("vhost-user trying to send unhandled ioctl");
        return -1;
    }

    memcpy(&msg.payload.session.session_setup_data, sess_info,
              sizeof(CryptoDevBackendSymSessionInfo));
    if (sess_info->key_len) {
        memcpy(&msg.payload.session.key, sess_info->cipher_key,
               sess_info->key_len);
    }
    if (sess_info->auth_key_len > 0) {
        memcpy(&msg.payload.session.auth_key, sess_info->auth_key,
               sess_info->auth_key_len);
    }
    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        error_report("vhost_user_write() return -1, create session failed");
        return -1;
    }

    if (vhost_user_read(dev, &msg) < 0) {
        error_report("vhost_user_read() return -1, create session failed");
        return -1;
    }

    if (msg.hdr.request != VHOST_USER_CREATE_CRYPTO_SESSION) {
        error_report("Received unexpected msg type. Expected %d received %d",
                     VHOST_USER_CREATE_CRYPTO_SESSION, msg.hdr.request);
        return -1;
    }

    if (msg.hdr.size != sizeof(msg.payload.session)) {
        error_report("Received bad msg size.");
        return -1;
    }

    if (msg.payload.session.session_id < 0) {
        error_report("Bad session id: %" PRId64 "",
                              msg.payload.session.session_id);
        return -1;
    }
    *session_id = msg.payload.session.session_id;

    return 0;
}

static int
vhost_user_crypto_close_session(struct vhost_dev *dev, uint64_t session_id)
{
    bool crypto_session = virtio_has_feature(dev->protocol_features,
                                       VHOST_USER_PROTOCOL_F_CRYPTO_SESSION);
    VhostUserMsg msg = {
        .hdr.request = VHOST_USER_CLOSE_CRYPTO_SESSION,
        .hdr.flags = VHOST_USER_VERSION,
        .hdr.size = sizeof(msg.payload.u64),
    };
    msg.payload.u64 = session_id;

    if (!crypto_session) {
        error_report("vhost-user trying to send unhandled ioctl");
        return -1;
    }

    if (vhost_user_write(dev, &msg, NULL, 0) < 0) {
        error_report("vhost_user_write() return -1, close session failed");
        return -1;
    }

    return 0;
}

const VhostOps user_ops = {
        .backend_type = VHOST_BACKEND_TYPE_USER,
        .vhost_backend_init = vhost_user_init,
        .vhost_backend_cleanup = vhost_user_cleanup,
        .vhost_backend_memslots_limit = vhost_user_memslots_limit,
        .vhost_set_log_base = vhost_user_set_log_base,
        .vhost_set_mem_table = vhost_user_set_mem_table,
        .vhost_set_vring_addr = vhost_user_set_vring_addr,
        .vhost_set_vring_endian = vhost_user_set_vring_endian,
        .vhost_set_vring_num = vhost_user_set_vring_num,
        .vhost_set_vring_base = vhost_user_set_vring_base,
        .vhost_get_vring_base = vhost_user_get_vring_base,
        .vhost_set_vring_kick = vhost_user_set_vring_kick,
        .vhost_set_vring_call = vhost_user_set_vring_call,
        .vhost_set_features = vhost_user_set_features,
        .vhost_get_features = vhost_user_get_features,
        .vhost_set_owner = vhost_user_set_owner,
        .vhost_reset_device = vhost_user_reset_device,
        .vhost_get_vq_index = vhost_user_get_vq_index,
        .vhost_set_vring_enable = vhost_user_set_vring_enable,
        .vhost_requires_shm_log = vhost_user_requires_shm_log,
        .vhost_migration_done = vhost_user_migration_done,
        .vhost_backend_can_merge = vhost_user_can_merge,
        .vhost_net_set_mtu = vhost_user_net_set_mtu,
        .vhost_set_iotlb_callback = vhost_user_set_iotlb_callback,
        .vhost_send_device_iotlb_msg = vhost_user_send_device_iotlb_msg,
        .vhost_get_config = vhost_user_get_config,
        .vhost_set_config = vhost_user_set_config,
        .vhost_crypto_create_session = vhost_user_crypto_create_session,
        .vhost_crypto_close_session = vhost_user_crypto_close_session,
};
