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

#ifndef NET_HUB_H
#define NET_HUB_H

#include "qemu-common.h"

NetClientState *net_hub_add_port(int hub_id, const char *name);
NetClientState *net_hub_find_client_by_name(int hub_id, const char *name);
void net_hub_info(Monitor *mon);
int net_hub_id_for_client(NetClientState *nc, int *id);
void net_hub_check_clients(void);
NetClientState *net_hub_port_find(int hub_id);

#endif /* NET_HUB_H */
