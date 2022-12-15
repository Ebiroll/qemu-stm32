/*
 * QEMU Xen emulation: Event channel support
 *
 * Copyright © 2022 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Authors: David Woodhouse <dwmw2@infradead.org>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef QEMU_XEN_EVTCHN_H
#define QEMU_XEN_EVTCHN_H

#include "hw/sysbus.h"

void xen_evtchn_create(void);
int xen_evtchn_soft_reset(void);
int xen_evtchn_set_callback_param(uint64_t param);
void xen_evtchn_connect_gsis(qemu_irq *system_gsis);
void xen_evtchn_set_callback_level(int level);

struct evtchn_status;
struct evtchn_close;
struct evtchn_unmask;
struct evtchn_bind_virq;
struct evtchn_bind_ipi;
struct evtchn_send;
struct evtchn_alloc_unbound;
struct evtchn_bind_interdomain;
struct evtchn_bind_vcpu;
struct evtchn_reset;
int xen_evtchn_status_op(struct evtchn_status *status);
int xen_evtchn_close_op(struct evtchn_close *close);
int xen_evtchn_unmask_op(struct evtchn_unmask *unmask);
int xen_evtchn_bind_virq_op(struct evtchn_bind_virq *virq);
int xen_evtchn_bind_ipi_op(struct evtchn_bind_ipi *ipi);
int xen_evtchn_send_op(struct evtchn_send *send);
int xen_evtchn_alloc_unbound_op(struct evtchn_alloc_unbound *alloc);
int xen_evtchn_bind_interdomain_op(struct evtchn_bind_interdomain *interdomain);
int xen_evtchn_bind_vcpu_op(struct evtchn_bind_vcpu *vcpu);
int xen_evtchn_reset_op(struct evtchn_reset *reset);

#endif /* QEMU_XEN_EVTCHN_H */
