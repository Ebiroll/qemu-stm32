#include "qemu/osdep.h"
#include "qapi/error.h"
#include "include/qemu-common.h"
#include "chardev/char.h"
#include "qemu/buffer.h"
#include "qemu/units.h"
#include "trace.h"

#include "qapi/qapi-types-char.h"

#include "spice/vd_agent.h"

#define VDAGENT_BUFFER_LIMIT (1 * MiB)

struct VDAgentChardev {
    Chardev parent;

    /* guest vdagent */
    uint32_t caps;
    VDIChunkHeader chunk;
    uint32_t chunksize;
    uint8_t *msgbuf;
    uint32_t msgsize;
    uint8_t *xbuf;
    uint32_t xoff, xsize;
    Buffer outbuf;
};
typedef struct VDAgentChardev VDAgentChardev;

#define TYPE_CHARDEV_QEMU_VDAGENT "chardev-qemu-vdagent"

DECLARE_INSTANCE_CHECKER(VDAgentChardev, QEMU_VDAGENT_CHARDEV,
                         TYPE_CHARDEV_QEMU_VDAGENT);

/* ------------------------------------------------------------------ */
/* names, for debug logging                                           */

static const char *cap_name[] = {
    [VD_AGENT_CAP_MOUSE_STATE]                    = "mouse-state",
    [VD_AGENT_CAP_MONITORS_CONFIG]                = "monitors-config",
    [VD_AGENT_CAP_REPLY]                          = "reply",
    [VD_AGENT_CAP_CLIPBOARD]                      = "clipboard",
    [VD_AGENT_CAP_DISPLAY_CONFIG]                 = "display-config",
    [VD_AGENT_CAP_CLIPBOARD_BY_DEMAND]            = "clipboard-by-demand",
    [VD_AGENT_CAP_CLIPBOARD_SELECTION]            = "clipboard-selection",
    [VD_AGENT_CAP_SPARSE_MONITORS_CONFIG]         = "sparse-monitors-config",
    [VD_AGENT_CAP_GUEST_LINEEND_LF]               = "guest-lineend-lf",
    [VD_AGENT_CAP_GUEST_LINEEND_CRLF]             = "guest-lineend-crlf",
    [VD_AGENT_CAP_MAX_CLIPBOARD]                  = "max-clipboard",
    [VD_AGENT_CAP_AUDIO_VOLUME_SYNC]              = "audio-volume-sync",
    [VD_AGENT_CAP_MONITORS_CONFIG_POSITION]       = "monitors-config-position",
    [VD_AGENT_CAP_FILE_XFER_DISABLED]             = "file-xfer-disabled",
    [VD_AGENT_CAP_FILE_XFER_DETAILED_ERRORS]      = "file-xfer-detailed-errors",
#if 0
    [VD_AGENT_CAP_GRAPHICS_DEVICE_INFO]           = "graphics-device-info",
    [VD_AGENT_CAP_CLIPBOARD_NO_RELEASE_ON_REGRAB] = "clipboard-no-release-on-regrab",
    [VD_AGENT_CAP_CLIPBOARD_GRAB_SERIAL]          = "clipboard-grab-serial",
#endif
};

static const char *msg_name[] = {
    [VD_AGENT_MOUSE_STATE]           = "mouse-state",
    [VD_AGENT_MONITORS_CONFIG]       = "monitors-config",
    [VD_AGENT_REPLY]                 = "reply",
    [VD_AGENT_CLIPBOARD]             = "clipboard",
    [VD_AGENT_DISPLAY_CONFIG]        = "display-config",
    [VD_AGENT_ANNOUNCE_CAPABILITIES] = "announce-capabilities",
    [VD_AGENT_CLIPBOARD_GRAB]        = "clipboard-grab",
    [VD_AGENT_CLIPBOARD_REQUEST]     = "clipboard-request",
    [VD_AGENT_CLIPBOARD_RELEASE]     = "clipboard-release",
    [VD_AGENT_FILE_XFER_START]       = "file-xfer-start",
    [VD_AGENT_FILE_XFER_STATUS]      = "file-xfer-status",
    [VD_AGENT_FILE_XFER_DATA]        = "file-xfer-data",
    [VD_AGENT_CLIENT_DISCONNECTED]   = "client-disconnected",
    [VD_AGENT_MAX_CLIPBOARD]         = "max-clipboard",
    [VD_AGENT_AUDIO_VOLUME_SYNC]     = "audio-volume-sync",
#if 0
    [VD_AGENT_GRAPHICS_DEVICE_INFO]  = "graphics-device-info",
#endif
};

#define GET_NAME(_m, _v) \
    (((_v) < ARRAY_SIZE(_m) && (_m[_v])) ? (_m[_v]) : "???")

/* ------------------------------------------------------------------ */
/* send messages                                                      */

static void vdagent_send_buf(VDAgentChardev *vd)
{
    uint32_t len;

    while (!buffer_empty(&vd->outbuf)) {
        len = qemu_chr_be_can_write(CHARDEV(vd));
        if (len == 0) {
            return;
        }
        if (len > vd->outbuf.offset) {
            len = vd->outbuf.offset;
        }
        qemu_chr_be_write(CHARDEV(vd), vd->outbuf.buffer, len);
        buffer_advance(&vd->outbuf, len);
    }
}

static void vdagent_send_msg(VDAgentChardev *vd, VDAgentMessage *msg)
{
    uint8_t *msgbuf = (void *)msg;
    uint32_t msgsize = sizeof(VDAgentMessage) + msg->size;
    uint32_t msgoff = 0;
    VDIChunkHeader chunk;

    trace_vdagent_send(GET_NAME(msg_name, msg->type));

    msg->protocol = VD_AGENT_PROTOCOL;

    if (vd->outbuf.offset + msgsize > VDAGENT_BUFFER_LIMIT) {
        error_report("buffer full, dropping message");
        return;
    }

    while (msgoff < msgsize) {
        chunk.port = VDP_CLIENT_PORT;
        chunk.size = msgsize - msgoff;
        if (chunk.size > 1024) {
            chunk.size = 1024;
        }
        buffer_reserve(&vd->outbuf, sizeof(chunk) + chunk.size);
        buffer_append(&vd->outbuf, &chunk, sizeof(chunk));
        buffer_append(&vd->outbuf, msgbuf + msgoff, chunk.size);
        msgoff += chunk.size;
    }
    vdagent_send_buf(vd);
}

static void vdagent_send_caps(VDAgentChardev *vd)
{
    g_autofree VDAgentMessage *msg = g_malloc0(sizeof(VDAgentMessage) +
                                               sizeof(VDAgentAnnounceCapabilities) +
                                               sizeof(uint32_t));

    msg->type = VD_AGENT_ANNOUNCE_CAPABILITIES;
    msg->size = sizeof(VDAgentAnnounceCapabilities) + sizeof(uint32_t);

    vdagent_send_msg(vd, msg);
}

/* ------------------------------------------------------------------ */
/* chardev backend                                                    */

static void vdagent_chr_open(Chardev *chr,
                             ChardevBackend *backend,
                             bool *be_opened,
                             Error **errp)
{
#if defined(HOST_WORDS_BIGENDIAN)
    /*
     * TODO: vdagent protocol is defined to be LE,
     * so we have to byteswap everything on BE hosts.
     */
    error_setg(errp, "vdagent is not supported on bigendian hosts");
    return;
#endif

    *be_opened = true;
}

static void vdagent_chr_recv_caps(VDAgentChardev *vd, VDAgentMessage *msg)
{
    VDAgentAnnounceCapabilities *caps = (void *)msg->data;
    int i;

    if (msg->size < (sizeof(VDAgentAnnounceCapabilities) +
                     sizeof(uint32_t))) {
        return;
    }

    for (i = 0; i < ARRAY_SIZE(cap_name); i++) {
        if (caps->caps[0] & (1 << i)) {
            trace_vdagent_peer_cap(GET_NAME(cap_name, i));
        }
    }

    vd->caps = caps->caps[0];
    if (caps->request) {
        vdagent_send_caps(vd);
    }
}

static void vdagent_chr_recv_msg(VDAgentChardev *vd, VDAgentMessage *msg)
{
    trace_vdagent_recv_msg(GET_NAME(msg_name, msg->type), msg->size);

    switch (msg->type) {
    case VD_AGENT_ANNOUNCE_CAPABILITIES:
        vdagent_chr_recv_caps(vd, msg);
        break;
    default:
        break;
    }
}

static void vdagent_reset_xbuf(VDAgentChardev *vd)
{
    g_clear_pointer(&vd->xbuf, g_free);
    vd->xoff = 0;
    vd->xsize = 0;
}

static void vdagent_chr_recv_chunk(VDAgentChardev *vd)
{
    VDAgentMessage *msg = (void *)vd->msgbuf;

    if (!vd->xsize) {
        if (vd->msgsize < sizeof(*msg)) {
            error_report("%s: message too small: %d < %zd", __func__,
                         vd->msgsize, sizeof(*msg));
            return;
        }
        if (vd->msgsize == msg->size + sizeof(*msg)) {
            vdagent_chr_recv_msg(vd, msg);
            return;
        }
    }

    if (!vd->xsize) {
        vd->xsize = msg->size + sizeof(*msg);
        vd->xbuf = g_malloc0(vd->xsize);
    }

    if (vd->xoff + vd->msgsize > vd->xsize) {
        error_report("%s: Oops: %d+%d > %d", __func__,
                     vd->xoff, vd->msgsize, vd->xsize);
        vdagent_reset_xbuf(vd);
        return;
    }

    memcpy(vd->xbuf + vd->xoff, vd->msgbuf, vd->msgsize);
    vd->xoff += vd->msgsize;
    if (vd->xoff < vd->xsize) {
        return;
    }

    msg = (void *)vd->xbuf;
    vdagent_chr_recv_msg(vd, msg);
    vdagent_reset_xbuf(vd);
}

static void vdagent_reset_bufs(VDAgentChardev *vd)
{
    memset(&vd->chunk, 0, sizeof(vd->chunk));
    vd->chunksize = 0;
    g_free(vd->msgbuf);
    vd->msgbuf = NULL;
    vd->msgsize = 0;
}

static int vdagent_chr_write(Chardev *chr, const uint8_t *buf, int len)
{
    VDAgentChardev *vd = QEMU_VDAGENT_CHARDEV(chr);
    uint32_t copy, ret = len;

    while (len) {
        if (vd->chunksize < sizeof(vd->chunk)) {
            copy = sizeof(vd->chunk) - vd->chunksize;
            if (copy > len) {
                copy = len;
            }
            memcpy((void *)(&vd->chunk) + vd->chunksize, buf, copy);
            vd->chunksize += copy;
            buf += copy;
            len -= copy;
            if (vd->chunksize < sizeof(vd->chunk)) {
                break;
            }

            assert(vd->msgbuf == NULL);
            vd->msgbuf = g_malloc0(vd->chunk.size);
        }

        copy = vd->chunk.size - vd->msgsize;
        if (copy > len) {
            copy = len;
        }
        memcpy(vd->msgbuf + vd->msgsize, buf, copy);
        vd->msgsize += copy;
        buf += copy;
        len -= copy;

        if (vd->msgsize == vd->chunk.size) {
            trace_vdagent_recv_chunk(vd->chunk.size);
            vdagent_chr_recv_chunk(vd);
            vdagent_reset_bufs(vd);
        }
    }

    return ret;
}

static void vdagent_chr_accept_input(Chardev *chr)
{
    VDAgentChardev *vd = QEMU_VDAGENT_CHARDEV(chr);

    vdagent_send_buf(vd);
}

static void vdagent_chr_set_fe_open(struct Chardev *chr, int fe_open)
{
    VDAgentChardev *vd = QEMU_VDAGENT_CHARDEV(chr);

    if (!fe_open) {
        trace_vdagent_close();
        /* reset state */
        vdagent_reset_bufs(vd);
        vd->caps = 0;
        return;
    }

    trace_vdagent_open();
}

/* ------------------------------------------------------------------ */

static void vdagent_chr_class_init(ObjectClass *oc, void *data)
{
    ChardevClass *cc = CHARDEV_CLASS(oc);

    cc->open             = vdagent_chr_open;
    cc->chr_write        = vdagent_chr_write;
    cc->chr_set_fe_open  = vdagent_chr_set_fe_open;
    cc->chr_accept_input = vdagent_chr_accept_input;
}

static void vdagent_chr_init(Object *obj)
{
    VDAgentChardev *vd = QEMU_VDAGENT_CHARDEV(obj);

    buffer_init(&vd->outbuf, "vdagent-outbuf");
}

static void vdagent_chr_fini(Object *obj)
{
    VDAgentChardev *vd = QEMU_VDAGENT_CHARDEV(obj);

    buffer_free(&vd->outbuf);
}

static const TypeInfo vdagent_chr_type_info = {
    .name = TYPE_CHARDEV_QEMU_VDAGENT,
    .parent = TYPE_CHARDEV,
    .instance_size = sizeof(VDAgentChardev),
    .instance_init = vdagent_chr_init,
    .instance_finalize = vdagent_chr_fini,
    .class_init = vdagent_chr_class_init,
};

static void register_types(void)
{
    type_register_static(&vdagent_chr_type_info);
}

type_init(register_types);
