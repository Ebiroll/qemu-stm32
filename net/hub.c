/*
 * Hub net client
 *
 * Copyright IBM, Corp. 2012
 *
 * Authors:
 *  Stefan Hajnoczi   <stefanha@linux.vnet.ibm.com>
 *  Zhi Yong Wu       <wuzhy@linux.vnet.ibm.com>
 *
 * This work is licensed under the terms of the GNU LGPL, version 2 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 */

#include "monitor.h"
#include "net.h"
#include "clients.h"
#include "hub.h"
#include "iov.h"

/*
 * A hub broadcasts incoming packets to all its ports except the source port.
 * Hubs can be used to provide independent network segments, also confusingly
 * named the QEMU 'vlan' feature.
 */

typedef struct NetHub NetHub;

typedef struct NetHubPort {
    NetClientState nc;
    QLIST_ENTRY(NetHubPort) next;
    NetHub *hub;
    int id;
} NetHubPort;

struct NetHub {
    int id;
    QLIST_ENTRY(NetHub) next;
    int num_ports;
    QLIST_HEAD(, NetHubPort) ports;
};

static QLIST_HEAD(, NetHub) hubs = QLIST_HEAD_INITIALIZER(&hubs);

static ssize_t net_hub_receive(NetHub *hub, NetHubPort *source_port,
                               const uint8_t *buf, size_t len)
{
    NetHubPort *port;

    QLIST_FOREACH(port, &hub->ports, next) {
        if (port == source_port) {
            continue;
        }

        qemu_send_packet(&port->nc, buf, len);
    }
    return len;
}

static ssize_t net_hub_receive_iov(NetHub *hub, NetHubPort *source_port,
                                   const struct iovec *iov, int iovcnt)
{
    NetHubPort *port;
    ssize_t len = iov_size(iov, iovcnt);

    QLIST_FOREACH(port, &hub->ports, next) {
        if (port == source_port) {
            continue;
        }

        qemu_sendv_packet(&port->nc, iov, iovcnt);
    }
    return len;
}

static NetHub *net_hub_new(int id)
{
    NetHub *hub;

    hub = g_malloc(sizeof(*hub));
    hub->id = id;
    hub->num_ports = 0;
    QLIST_INIT(&hub->ports);

    QLIST_INSERT_HEAD(&hubs, hub, next);

    return hub;
}

static int net_hub_port_can_receive(NetClientState *nc)
{
    NetHubPort *port;
    NetHubPort *src_port = DO_UPCAST(NetHubPort, nc, nc);
    NetHub *hub = src_port->hub;

    QLIST_FOREACH(port, &hub->ports, next) {
        if (port == src_port) {
            continue;
        }

        if (qemu_can_send_packet(&port->nc)) {
            return 1;
        }
    }

    return 0;
}

static ssize_t net_hub_port_receive(NetClientState *nc,
                                    const uint8_t *buf, size_t len)
{
    NetHubPort *port = DO_UPCAST(NetHubPort, nc, nc);

    return net_hub_receive(port->hub, port, buf, len);
}

static ssize_t net_hub_port_receive_iov(NetClientState *nc,
                                        const struct iovec *iov, int iovcnt)
{
    NetHubPort *port = DO_UPCAST(NetHubPort, nc, nc);

    return net_hub_receive_iov(port->hub, port, iov, iovcnt);
}

static void net_hub_port_cleanup(NetClientState *nc)
{
    NetHubPort *port = DO_UPCAST(NetHubPort, nc, nc);

    QLIST_REMOVE(port, next);
}

static NetClientInfo net_hub_port_info = {
    .type = NET_CLIENT_OPTIONS_KIND_HUBPORT,
    .size = sizeof(NetHubPort),
    .can_receive = net_hub_port_can_receive,
    .receive = net_hub_port_receive,
    .receive_iov = net_hub_port_receive_iov,
    .cleanup = net_hub_port_cleanup,
};

static NetHubPort *net_hub_port_new(NetHub *hub, const char *name)
{
    NetClientState *nc;
    NetHubPort *port;
    int id = hub->num_ports++;
    char default_name[128];

    if (!name) {
        snprintf(default_name, sizeof(default_name),
                 "hub%dport%d", hub->id, id);
        name = default_name;
    }

    nc = qemu_new_net_client(&net_hub_port_info, NULL, "hub", name);
    port = DO_UPCAST(NetHubPort, nc, nc);
    port->id = id;
    port->hub = hub;

    QLIST_INSERT_HEAD(&hub->ports, port, next);

    return port;
}

/**
 * Create a port on a given hub
 * @name: Net client name or NULL for default name.
 *
 * If there is no existing hub with the given id then a new hub is created.
 */
NetClientState *net_hub_add_port(int hub_id, const char *name)
{
    NetHub *hub;
    NetHubPort *port;

    QLIST_FOREACH(hub, &hubs, next) {
        if (hub->id == hub_id) {
            break;
        }
    }

    if (!hub) {
        hub = net_hub_new(hub_id);
    }

    port = net_hub_port_new(hub, name);
    return &port->nc;
}

/**
 * Find a specific client on a hub
 */
NetClientState *net_hub_find_client_by_name(int hub_id, const char *name)
{
    NetHub *hub;
    NetHubPort *port;
    NetClientState *peer;

    QLIST_FOREACH(hub, &hubs, next) {
        if (hub->id == hub_id) {
            QLIST_FOREACH(port, &hub->ports, next) {
                peer = port->nc.peer;

                if (peer && strcmp(peer->name, name) == 0) {
                    return peer;
                }
            }
        }
    }
    return NULL;
}

/**
 * Find a available port on a hub; otherwise create one new port
 */
NetClientState *net_hub_port_find(int hub_id)
{
    NetHub *hub;
    NetHubPort *port;
    NetClientState *nc;

    QLIST_FOREACH(hub, &hubs, next) {
        if (hub->id == hub_id) {
            QLIST_FOREACH(port, &hub->ports, next) {
                nc = port->nc.peer;
                if (!nc) {
                    return &(port->nc);
                }
            }
            break;
        }
    }

    nc = net_hub_add_port(hub_id, NULL);
    return nc;
}

/**
 * Print hub configuration
 */
void net_hub_info(Monitor *mon)
{
    NetHub *hub;
    NetHubPort *port;

    QLIST_FOREACH(hub, &hubs, next) {
        monitor_printf(mon, "hub %d\n", hub->id);
        QLIST_FOREACH(port, &hub->ports, next) {
            if (port->nc.peer) {
                monitor_printf(mon, " \\ ");
                print_net_client(mon, port->nc.peer);
            }
        }
    }
}

/**
 * Get the hub id that a client is connected to
 *
 * @id: Pointer for hub id output, may be NULL
 */
int net_hub_id_for_client(NetClientState *nc, int *id)
{
    NetHubPort *port;

    if (nc->info->type == NET_CLIENT_OPTIONS_KIND_HUBPORT) {
        port = DO_UPCAST(NetHubPort, nc, nc);
    } else if (nc->peer != NULL && nc->peer->info->type ==
            NET_CLIENT_OPTIONS_KIND_HUBPORT) {
        port = DO_UPCAST(NetHubPort, nc, nc->peer);
    } else {
        return -ENOENT;
    }

    if (id) {
        *id = port->hub->id;
    }
    return 0;
}

int net_init_hubport(const NetClientOptions *opts, const char *name,
                     NetClientState *peer)
{
    const NetdevHubPortOptions *hubport;

    assert(opts->kind == NET_CLIENT_OPTIONS_KIND_HUBPORT);
    hubport = opts->hubport;

    /* Treat hub port like a backend, NIC must be the one to peer */
    if (peer) {
        return -EINVAL;
    }

    net_hub_add_port(hubport->hubid, name);
    return 0;
}

/**
 * Warn if hub configurations are likely wrong
 */
void net_hub_check_clients(void)
{
    NetHub *hub;
    NetHubPort *port;
    NetClientState *peer;

    QLIST_FOREACH(hub, &hubs, next) {
        int has_nic = 0, has_host_dev = 0;

        QLIST_FOREACH(port, &hub->ports, next) {
            peer = port->nc.peer;
            if (!peer) {
                fprintf(stderr, "Warning: hub port %s has no peer\n",
                        port->nc.name);
                continue;
            }

            switch (peer->info->type) {
            case NET_CLIENT_OPTIONS_KIND_NIC:
                has_nic = 1;
                break;
            case NET_CLIENT_OPTIONS_KIND_USER:
            case NET_CLIENT_OPTIONS_KIND_TAP:
            case NET_CLIENT_OPTIONS_KIND_SOCKET:
            case NET_CLIENT_OPTIONS_KIND_VDE:
                has_host_dev = 1;
                break;
            default:
                break;
            }
        }
        if (has_host_dev && !has_nic) {
            fprintf(stderr, "Warning: vlan %d with no nics\n", hub->id);
        }
        if (has_nic && !has_host_dev) {
            fprintf(stderr,
                    "Warning: vlan %d is not connected to host network\n",
                    hub->id);
        }
    }
}
