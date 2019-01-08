/*
 * Copyright (c) 2018  Citrix Systems Inc.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/xen/xen.h"
#include "hw/xen/xen-bus.h"
#include "hw/xen/xen-bus-helper.h"
#include "monitor/monitor.h"
#include "qapi/error.h"
#include "sysemu/sysemu.h"
#include "trace.h"

static char *xen_device_get_backend_path(XenDevice *xendev)
{
    XenBus *xenbus = XEN_BUS(qdev_get_parent_bus(DEVICE(xendev)));
    XenDeviceClass *xendev_class = XEN_DEVICE_GET_CLASS(xendev);
    const char *type = object_get_typename(OBJECT(xendev));
    const char *backend = xendev_class->backend;

    if (!backend) {
        backend = type;
    }

    return g_strdup_printf("/local/domain/%u/backend/%s/%u/%s",
                           xenbus->backend_id, backend, xendev->frontend_id,
                           xendev->name);
}

static char *xen_device_get_frontend_path(XenDevice *xendev)
{
    XenDeviceClass *xendev_class = XEN_DEVICE_GET_CLASS(xendev);
    const char *type = object_get_typename(OBJECT(xendev));
    const char *device = xendev_class->device;

    if (!device) {
        device = type;
    }

    return g_strdup_printf("/local/domain/%u/device/%s/%s",
                           xendev->frontend_id, device, xendev->name);
}

static void xen_bus_print_dev(Monitor *mon, DeviceState *dev, int indent)
{
    XenDevice *xendev = XEN_DEVICE(dev);

    monitor_printf(mon, "%*sname = '%s' frontend_id = %u\n",
                   indent, "", xendev->name, xendev->frontend_id);
}

static char *xen_bus_get_dev_path(DeviceState *dev)
{
    return xen_device_get_backend_path(XEN_DEVICE(dev));
}

static void xen_bus_unrealize(BusState *bus, Error **errp)
{
    XenBus *xenbus = XEN_BUS(bus);

    trace_xen_bus_unrealize();

    if (!xenbus->xsh) {
        return;
    }

    xs_close(xenbus->xsh);
}

static void xen_bus_realize(BusState *bus, Error **errp)
{
    XenBus *xenbus = XEN_BUS(bus);
    unsigned int domid;

    trace_xen_bus_realize();

    xenbus->xsh = xs_open(0);
    if (!xenbus->xsh) {
        error_setg_errno(errp, errno, "failed xs_open");
        goto fail;
    }

    if (xs_node_scanf(xenbus->xsh, XBT_NULL, "", /* domain root node */
                      "domid", NULL, "%u", &domid) == 1) {
        xenbus->backend_id = domid;
    } else {
        xenbus->backend_id = 0; /* Assume lack of node means dom0 */
    }

    return;

fail:
    xen_bus_unrealize(bus, &error_abort);
}

static void xen_bus_class_init(ObjectClass *class, void *data)
{
    BusClass *bus_class = BUS_CLASS(class);

    bus_class->print_dev = xen_bus_print_dev;
    bus_class->get_dev_path = xen_bus_get_dev_path;
    bus_class->realize = xen_bus_realize;
    bus_class->unrealize = xen_bus_unrealize;
}

static const TypeInfo xen_bus_type_info = {
    .name = TYPE_XEN_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(XenBus),
    .class_size = sizeof(XenBusClass),
    .class_init = xen_bus_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_HOTPLUG_HANDLER },
        { }
    },
};

static void xen_device_backend_printf(XenDevice *xendev, const char *key,
                                      const char *fmt, ...)
{
    XenBus *xenbus = XEN_BUS(qdev_get_parent_bus(DEVICE(xendev)));
    Error *local_err = NULL;
    va_list ap;

    g_assert(xenbus->xsh);

    va_start(ap, fmt);
    xs_node_vprintf(xenbus->xsh, XBT_NULL, xendev->backend_path, key,
                    &local_err, fmt, ap);
    va_end(ap);

    if (local_err) {
        error_report_err(local_err);
    }
}

static void xen_device_backend_set_state(XenDevice *xendev,
                                         enum xenbus_state state)
{
    const char *type = object_get_typename(OBJECT(xendev));

    if (xendev->backend_state == state) {
        return;
    }

    trace_xen_device_backend_state(type, xendev->name,
                                   xs_strstate(state));

    xendev->backend_state = state;
    xen_device_backend_printf(xendev, "state", "%u", state);
}

static void xen_device_backend_create(XenDevice *xendev, Error **errp)
{
    XenBus *xenbus = XEN_BUS(qdev_get_parent_bus(DEVICE(xendev)));
    struct xs_permissions perms[2];
    Error *local_err = NULL;

    xendev->backend_path = xen_device_get_backend_path(xendev);

    perms[0].id = xenbus->backend_id;
    perms[0].perms = XS_PERM_NONE;
    perms[1].id = xendev->frontend_id;
    perms[1].perms = XS_PERM_READ;

    g_assert(xenbus->xsh);

    xs_node_create(xenbus->xsh, XBT_NULL, xendev->backend_path, perms,
                   ARRAY_SIZE(perms), &local_err);
    if (local_err) {
        error_propagate_prepend(errp, local_err,
                                "failed to create backend: ");
    }
}

static void xen_device_backend_destroy(XenDevice *xendev)
{
    XenBus *xenbus = XEN_BUS(qdev_get_parent_bus(DEVICE(xendev)));
    Error *local_err = NULL;

    if (!xendev->backend_path) {
        return;
    }

    g_assert(xenbus->xsh);

    xs_node_destroy(xenbus->xsh, XBT_NULL, xendev->backend_path,
                    &local_err);
    g_free(xendev->backend_path);
    xendev->backend_path = NULL;

    if (local_err) {
        error_report_err(local_err);
    }
}

static void xen_device_frontend_printf(XenDevice *xendev, const char *key,
                                       const char *fmt, ...)
{
    XenBus *xenbus = XEN_BUS(qdev_get_parent_bus(DEVICE(xendev)));
    Error *local_err = NULL;
    va_list ap;

    g_assert(xenbus->xsh);

    va_start(ap, fmt);
    xs_node_vprintf(xenbus->xsh, XBT_NULL, xendev->frontend_path, key,
                    &local_err, fmt, ap);
    va_end(ap);

    if (local_err) {
        error_report_err(local_err);
    }
}

static void xen_device_frontend_set_state(XenDevice *xendev,
                                          enum xenbus_state state)
{
    const char *type = object_get_typename(OBJECT(xendev));

    if (xendev->frontend_state == state) {
        return;
    }

    trace_xen_device_frontend_state(type, xendev->name,
                                    xs_strstate(state));

    xendev->frontend_state = state;
    xen_device_frontend_printf(xendev, "state", "%u", state);
}

static void xen_device_frontend_create(XenDevice *xendev, Error **errp)
{
    XenBus *xenbus = XEN_BUS(qdev_get_parent_bus(DEVICE(xendev)));
    struct xs_permissions perms[2];
    Error *local_err = NULL;

    xendev->frontend_path = xen_device_get_frontend_path(xendev);

    perms[0].id = xendev->frontend_id;
    perms[0].perms = XS_PERM_NONE;
    perms[1].id = xenbus->backend_id;
    perms[1].perms = XS_PERM_READ | XS_PERM_WRITE;

    g_assert(xenbus->xsh);

    xs_node_create(xenbus->xsh, XBT_NULL, xendev->frontend_path, perms,
                   ARRAY_SIZE(perms), &local_err);
    if (local_err) {
        error_propagate_prepend(errp, local_err,
                                "failed to create frontend: ");
    }
}

static void xen_device_frontend_destroy(XenDevice *xendev)
{
    XenBus *xenbus = XEN_BUS(qdev_get_parent_bus(DEVICE(xendev)));
    Error *local_err = NULL;

    if (!xendev->frontend_path) {
        return;
    }

    g_assert(xenbus->xsh);

    xs_node_destroy(xenbus->xsh, XBT_NULL, xendev->frontend_path,
                    &local_err);
    g_free(xendev->frontend_path);
    xendev->frontend_path = NULL;

    if (local_err) {
        error_report_err(local_err);
    }
}

static void xen_device_unrealize(DeviceState *dev, Error **errp)
{
    XenDevice *xendev = XEN_DEVICE(dev);
    XenDeviceClass *xendev_class = XEN_DEVICE_GET_CLASS(xendev);
    const char *type = object_get_typename(OBJECT(xendev));

    if (!xendev->name) {
        return;
    }

    trace_xen_device_unrealize(type, xendev->name);

    if (xendev->exit.notify) {
        qemu_remove_exit_notifier(&xendev->exit);
        xendev->exit.notify = NULL;
    }

    if (xendev_class->unrealize) {
        xendev_class->unrealize(xendev, errp);
    }

    xen_device_frontend_destroy(xendev);
    xen_device_backend_destroy(xendev);

    g_free(xendev->name);
    xendev->name = NULL;
}

static void xen_device_exit(Notifier *n, void *data)
{
    XenDevice *xendev = container_of(n, XenDevice, exit);

    xen_device_unrealize(DEVICE(xendev), &error_abort);
}

static void xen_device_realize(DeviceState *dev, Error **errp)
{
    XenDevice *xendev = XEN_DEVICE(dev);
    XenDeviceClass *xendev_class = XEN_DEVICE_GET_CLASS(xendev);
    XenBus *xenbus = XEN_BUS(qdev_get_parent_bus(DEVICE(xendev)));
    const char *type = object_get_typename(OBJECT(xendev));
    Error *local_err = NULL;

    if (xendev->frontend_id == DOMID_INVALID) {
        xendev->frontend_id = xen_domid;
    }

    if (xendev->frontend_id >= DOMID_FIRST_RESERVED) {
        error_setg(errp, "invalid frontend-id");
        goto unrealize;
    }

    if (!xendev_class->get_name) {
        error_setg(errp, "get_name method not implemented");
        goto unrealize;
    }

    xendev->name = xendev_class->get_name(xendev, &local_err);
    if (local_err) {
        error_propagate_prepend(errp, local_err,
                                "failed to get device name: ");
        goto unrealize;
    }

    trace_xen_device_realize(type, xendev->name);

    xen_device_backend_create(xendev, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        goto unrealize;
    }

    xen_device_frontend_create(xendev, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        goto unrealize;
    }

    if (xendev_class->realize) {
        xendev_class->realize(xendev, &local_err);
        if (local_err) {
            error_propagate(errp, local_err);
            goto unrealize;
        }
    }

    xen_device_backend_printf(xendev, "frontend", "%s",
                              xendev->frontend_path);
    xen_device_backend_printf(xendev, "frontend-id", "%u",
                              xendev->frontend_id);
    xen_device_backend_printf(xendev, "online", "%u", 1);
    xen_device_backend_printf(xendev, "hotplug-status", "connected");

    xen_device_backend_set_state(xendev, XenbusStateInitWait);

    xen_device_frontend_printf(xendev, "backend", "%s",
                               xendev->backend_path);
    xen_device_frontend_printf(xendev, "backend-id", "%u",
                               xenbus->backend_id);

    xen_device_frontend_set_state(xendev, XenbusStateInitialising);

    xendev->exit.notify = xen_device_exit;
    qemu_add_exit_notifier(&xendev->exit);
    return;

unrealize:
    xen_device_unrealize(dev, &error_abort);
}

static Property xen_device_props[] = {
    DEFINE_PROP_UINT16("frontend-id", XenDevice, frontend_id,
                       DOMID_INVALID),
    DEFINE_PROP_END_OF_LIST()
};

static void xen_device_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(class);

    dev_class->realize = xen_device_realize;
    dev_class->unrealize = xen_device_unrealize;
    dev_class->props = xen_device_props;
    dev_class->bus_type = TYPE_XEN_BUS;
}

static const TypeInfo xen_device_type_info = {
    .name = TYPE_XEN_DEVICE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(XenDevice),
    .abstract = true,
    .class_size = sizeof(XenDeviceClass),
    .class_init = xen_device_class_init,
};

typedef struct XenBridge {
    SysBusDevice busdev;
} XenBridge;

#define TYPE_XEN_BRIDGE "xen-bridge"

static const TypeInfo xen_bridge_type_info = {
    .name = TYPE_XEN_BRIDGE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XenBridge),
};

static void xen_register_types(void)
{
    type_register_static(&xen_bridge_type_info);
    type_register_static(&xen_bus_type_info);
    type_register_static(&xen_device_type_info);
}

type_init(xen_register_types)

void xen_bus_init(void)
{
    DeviceState *dev = qdev_create(NULL, TYPE_XEN_BRIDGE);
    BusState *bus = qbus_create(TYPE_XEN_BUS, dev, NULL);

    qdev_init_nofail(dev);
    qbus_set_bus_hotplug_handler(bus, &error_abort);
}
