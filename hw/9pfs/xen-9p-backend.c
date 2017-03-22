/*
 * Xen 9p backend
 *
 * Copyright Aporeto 2017
 *
 * Authors:
 *  Stefano Stabellini <stefano@aporeto.com>
 *
 */

#include "qemu/osdep.h"

#include "hw/hw.h"
#include "hw/9pfs/9p.h"
#include "hw/xen/xen_backend.h"
#include "hw/9pfs/xen-9pfs.h"
#include "qemu/config-file.h"
#include "fsdev/qemu-fsdev.h"

#define VERSIONS "1"
#define MAX_RINGS 8
#define MAX_RING_ORDER 8

typedef struct Xen9pfsRing {
    struct Xen9pfsDev *priv;

    int ref;
    xenevtchn_handle   *evtchndev;
    int evtchn;
    int local_port;
    int ring_order;
    struct xen_9pfs_data_intf *intf;
    unsigned char *data;
    struct xen_9pfs_data ring;

    struct iovec *sg;
    QEMUBH *bh;

    /* local copies, so that we can read/write PDU data directly from
     * the ring */
    RING_IDX out_cons, out_size, in_cons;
    bool inprogress;
} Xen9pfsRing;

typedef struct Xen9pfsDev {
    struct XenDevice xendev;  /* must be first */
    V9fsState state;
    char *path;
    char *security_model;
    char *tag;
    char *id;

    int num_rings;
    Xen9pfsRing *rings;
} Xen9pfsDev;

static ssize_t xen_9pfs_pdu_vmarshal(V9fsPDU *pdu,
                                     size_t offset,
                                     const char *fmt,
                                     va_list ap)
{
    return 0;
}

static ssize_t xen_9pfs_pdu_vunmarshal(V9fsPDU *pdu,
                                       size_t offset,
                                       const char *fmt,
                                       va_list ap)
{
    return 0;
}

static void xen_9pfs_init_out_iov_from_pdu(V9fsPDU *pdu,
                                           struct iovec **piov,
                                           unsigned int *pniov)
{
}

static void xen_9pfs_init_in_iov_from_pdu(V9fsPDU *pdu,
                                          struct iovec **piov,
                                          unsigned int *pniov,
                                          size_t size)
{
}

static void xen_9pfs_push_and_notify(V9fsPDU *pdu)
{
}

static const struct V9fsTransport xen_9p_transport = {
    .pdu_vmarshal = xen_9pfs_pdu_vmarshal,
    .pdu_vunmarshal = xen_9pfs_pdu_vunmarshal,
    .init_in_iov_from_pdu = xen_9pfs_init_in_iov_from_pdu,
    .init_out_iov_from_pdu = xen_9pfs_init_out_iov_from_pdu,
    .push_and_notify = xen_9pfs_push_and_notify,
};

static int xen_9pfs_init(struct XenDevice *xendev)
{
    return 0;
}

static void xen_9pfs_bh(void *opaque)
{
}

static void xen_9pfs_evtchn_event(void *opaque)
{
}

static int xen_9pfs_free(struct XenDevice *xendev)
{
    int i;
    Xen9pfsDev *xen_9pdev = container_of(xendev, Xen9pfsDev, xendev);

    g_free(xen_9pdev->id);
    g_free(xen_9pdev->tag);
    g_free(xen_9pdev->path);
    g_free(xen_9pdev->security_model);

    for (i = 0; i < xen_9pdev->num_rings; i++) {
        if (xen_9pdev->rings[i].data != NULL) {
            xengnttab_unmap(xen_9pdev->xendev.gnttabdev,
                    xen_9pdev->rings[i].data,
                    (1 << xen_9pdev->rings[i].ring_order));
        }
        if (xen_9pdev->rings[i].intf != NULL) {
            xengnttab_unmap(xen_9pdev->xendev.gnttabdev,
                    xen_9pdev->rings[i].intf,
                    1);
        }
        if (xen_9pdev->rings[i].evtchndev > 0) {
            qemu_set_fd_handler(xenevtchn_fd(xen_9pdev->rings[i].evtchndev),
                    NULL, NULL, NULL);
            xenevtchn_unbind(xen_9pdev->rings[i].evtchndev,
                             xen_9pdev->rings[i].local_port);
        }
        if (xen_9pdev->rings[i].bh != NULL) {
            qemu_bh_delete(xen_9pdev->rings[i].bh);
        }
    }
    g_free(xen_9pdev->rings);
    return 0;
}

static int xen_9pfs_connect(struct XenDevice *xendev)
{
    int i;
    Xen9pfsDev *xen_9pdev = container_of(xendev, Xen9pfsDev, xendev);
    V9fsState *s = &xen_9pdev->state;
    QemuOpts *fsdev;

    if (xenstore_read_fe_int(&xen_9pdev->xendev, "num-rings",
                             &xen_9pdev->num_rings) == -1 ||
        xen_9pdev->num_rings > MAX_RINGS || xen_9pdev->num_rings < 1) {
        return -1;
    }

    xen_9pdev->rings = g_malloc0(xen_9pdev->num_rings * sizeof(Xen9pfsRing));
    for (i = 0; i < xen_9pdev->num_rings; i++) {
        char *str;
        int ring_order;

        xen_9pdev->rings[i].priv = xen_9pdev;
        xen_9pdev->rings[i].evtchn = -1;
        xen_9pdev->rings[i].local_port = -1;

        str = g_strdup_printf("ring-ref%u", i);
        if (xenstore_read_fe_int(&xen_9pdev->xendev, str,
                                 &xen_9pdev->rings[i].ref) == -1) {
            goto out;
        }
        g_free(str);
        str = g_strdup_printf("event-channel-%u", i);
        if (xenstore_read_fe_int(&xen_9pdev->xendev, str,
                                 &xen_9pdev->rings[i].evtchn) == -1) {
            goto out;
        }
        g_free(str);

        xen_9pdev->rings[i].intf =  xengnttab_map_grant_ref(
                xen_9pdev->xendev.gnttabdev,
                xen_9pdev->xendev.dom,
                xen_9pdev->rings[i].ref,
                PROT_READ | PROT_WRITE);
        if (!xen_9pdev->rings[i].intf) {
            goto out;
        }
        ring_order = xen_9pdev->rings[i].intf->ring_order;
        if (ring_order > MAX_RING_ORDER) {
            goto out;
        }
        xen_9pdev->rings[i].ring_order = ring_order;
        xen_9pdev->rings[i].data = xengnttab_map_domain_grant_refs(
                xen_9pdev->xendev.gnttabdev,
                (1 << ring_order),
                xen_9pdev->xendev.dom,
                xen_9pdev->rings[i].intf->ref,
                PROT_READ | PROT_WRITE);
        if (!xen_9pdev->rings[i].data) {
            goto out;
        }
        xen_9pdev->rings[i].ring.in = xen_9pdev->rings[i].data;
        xen_9pdev->rings[i].ring.out = xen_9pdev->rings[i].data +
                                       XEN_FLEX_RING_SIZE(ring_order);

        xen_9pdev->rings[i].bh = qemu_bh_new(xen_9pfs_bh, &xen_9pdev->rings[i]);
        xen_9pdev->rings[i].out_cons = 0;
        xen_9pdev->rings[i].out_size = 0;
        xen_9pdev->rings[i].inprogress = false;


        xen_9pdev->rings[i].evtchndev = xenevtchn_open(NULL, 0);
        if (xen_9pdev->rings[i].evtchndev == NULL) {
            goto out;
        }
        fcntl(xenevtchn_fd(xen_9pdev->rings[i].evtchndev), F_SETFD, FD_CLOEXEC);
        xen_9pdev->rings[i].local_port = xenevtchn_bind_interdomain
                                            (xen_9pdev->rings[i].evtchndev,
                                             xendev->dom,
                                             xen_9pdev->rings[i].evtchn);
        if (xen_9pdev->rings[i].local_port == -1) {
            xen_pv_printf(xendev, 0,
                          "xenevtchn_bind_interdomain failed port=%d\n",
                          xen_9pdev->rings[i].evtchn);
            goto out;
        }
        xen_pv_printf(xendev, 2, "bind evtchn port %d\n", xendev->local_port);
        qemu_set_fd_handler(xenevtchn_fd(xen_9pdev->rings[i].evtchndev),
                xen_9pfs_evtchn_event, NULL, &xen_9pdev->rings[i]);
    }

    xen_9pdev->security_model = xenstore_read_be_str(xendev, "security_model");
    xen_9pdev->path = xenstore_read_be_str(xendev, "path");
    xen_9pdev->id = s->fsconf.fsdev_id =
        g_strdup_printf("xen9p%d", xendev->dev);
    xen_9pdev->tag = s->fsconf.tag = xenstore_read_fe_str(xendev, "tag");
    v9fs_register_transport(s, &xen_9p_transport);
    fsdev = qemu_opts_create(qemu_find_opts("fsdev"),
            s->fsconf.tag,
            1, NULL);
    qemu_opt_set(fsdev, "fsdriver", "local", NULL);
    qemu_opt_set(fsdev, "path", xen_9pdev->path, NULL);
    qemu_opt_set(fsdev, "security_model", xen_9pdev->security_model, NULL);
    qemu_opts_set_id(fsdev, s->fsconf.fsdev_id);
    qemu_fsdev_add(fsdev);
    v9fs_device_realize_common(s, NULL);

    return 0;

out:
    xen_9pfs_free(xendev);
    return -1;
}

static void xen_9pfs_alloc(struct XenDevice *xendev)
{
    xenstore_write_be_str(xendev, "versions", VERSIONS);
    xenstore_write_be_int(xendev, "max-rings", MAX_RINGS);
    xenstore_write_be_int(xendev, "max-ring-page-order", MAX_RING_ORDER);
}

static void xen_9pfs_disconnect(struct XenDevice *xendev)
{
    /* Dynamic hotplug of PV filesystems at runtime is not supported. */
}

struct XenDevOps xen_9pfs_ops = {
    .size       = sizeof(Xen9pfsDev),
    .flags      = DEVOPS_FLAG_NEED_GNTDEV,
    .alloc      = xen_9pfs_alloc,
    .init       = xen_9pfs_init,
    .initialise = xen_9pfs_connect,
    .disconnect = xen_9pfs_disconnect,
    .free       = xen_9pfs_free,
};
