/*
 * Copyright © 2020, 2021 Oracle and/or its affiliates.
 *
 * This work is licensed under the terms of the GNU GPL-v2, version 2 or later.
 *
 * See the COPYING file in the top-level directory.
 *
 */

#include "qemu/osdep.h"
#include "qemu-common.h"

#include "hw/remote/machine.h"
#include "io/channel.h"
#include "hw/remote/mpqemu-link.h"
#include "qapi/error.h"
#include "sysemu/runstate.h"
#include "hw/pci/pci.h"

static void process_config_write(QIOChannel *ioc, PCIDevice *dev,
                                 MPQemuMsg *msg, Error **errp);
static void process_config_read(QIOChannel *ioc, PCIDevice *dev,
                                MPQemuMsg *msg, Error **errp);

void coroutine_fn mpqemu_remote_msg_loop_co(void *data)
{
    g_autofree RemoteCommDev *com = (RemoteCommDev *)data;
    PCIDevice *pci_dev = NULL;
    Error *local_err = NULL;

    assert(com->ioc);

    pci_dev = com->dev;
    for (; !local_err;) {
        MPQemuMsg msg = {0};

        if (!mpqemu_msg_recv(&msg, com->ioc, &local_err)) {
            break;
        }

        if (!mpqemu_msg_valid(&msg)) {
            error_setg(&local_err, "Received invalid message from proxy"
                                   "in remote process pid="FMT_pid"",
                                   getpid());
            break;
        }

        switch (msg.cmd) {
        case MPQEMU_CMD_PCI_CFGWRITE:
            process_config_write(com->ioc, pci_dev, &msg, &local_err);
            break;
        case MPQEMU_CMD_PCI_CFGREAD:
            process_config_read(com->ioc, pci_dev, &msg, &local_err);
            break;
        default:
            error_setg(&local_err,
                       "Unknown command (%d) received for device %s"
                       " (pid="FMT_pid")",
                       msg.cmd, DEVICE(pci_dev)->id, getpid());
        }
    }

    if (local_err) {
        error_report_err(local_err);
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_HOST_ERROR);
    } else {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    }
}

static void process_config_write(QIOChannel *ioc, PCIDevice *dev,
                                 MPQemuMsg *msg, Error **errp)
{
    ERRP_GUARD();
    PciConfDataMsg *conf = (PciConfDataMsg *)&msg->data.pci_conf_data;
    MPQemuMsg ret = { 0 };

    if ((conf->addr + sizeof(conf->val)) > pci_config_size(dev)) {
        error_setg(errp, "Bad address for PCI config write, pid "FMT_pid".",
                   getpid());
        ret.data.u64 = UINT64_MAX;
    } else {
        pci_default_write_config(dev, conf->addr, conf->val, conf->len);
    }

    ret.cmd = MPQEMU_CMD_RET;
    ret.size = sizeof(ret.data.u64);

    if (!mpqemu_msg_send(&ret, ioc, NULL)) {
        error_prepend(errp, "Error returning code to proxy, pid "FMT_pid": ",
                      getpid());
    }
}

static void process_config_read(QIOChannel *ioc, PCIDevice *dev,
                                MPQemuMsg *msg, Error **errp)
{
    ERRP_GUARD();
    PciConfDataMsg *conf = (PciConfDataMsg *)&msg->data.pci_conf_data;
    MPQemuMsg ret = { 0 };

    if ((conf->addr + sizeof(conf->val)) > pci_config_size(dev)) {
        error_setg(errp, "Bad address for PCI config read, pid "FMT_pid".",
                   getpid());
        ret.data.u64 = UINT64_MAX;
    } else {
        ret.data.u64 = pci_default_read_config(dev, conf->addr, conf->len);
    }

    ret.cmd = MPQEMU_CMD_RET;
    ret.size = sizeof(ret.data.u64);

    if (!mpqemu_msg_send(&ret, ioc, NULL)) {
        error_prepend(errp, "Error returning code to proxy, pid "FMT_pid": ",
                      getpid());
    }
}
