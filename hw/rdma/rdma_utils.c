/*
 * QEMU paravirtual RDMA - Generic RDMA backend
 *
 * Copyright (C) 2018 Oracle
 * Copyright (C) 2018 Red Hat Inc
 *
 * Authors:
 *     Yuval Shaia <yuval.shaia@oracle.com>
 *     Marcel Apfelbaum <marcel@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include "qemu/osdep.h"
#include "trace.h"
#include "rdma_utils.h"

void *rdma_pci_dma_map(PCIDevice *dev, dma_addr_t addr, dma_addr_t plen)
{
    void *p;
    hwaddr len = plen;

    if (!addr) {
        rdma_error_report("addr is NULL");
        return NULL;
    }

    p = pci_dma_map(dev, addr, &len, DMA_DIRECTION_TO_DEVICE);
    if (!p) {
        rdma_error_report("pci_dma_map fail, addr=0x%"PRIx64", len=%"PRId64,
                          addr, len);
        return NULL;
    }

    if (len != plen) {
        rdma_pci_dma_unmap(dev, p, len);
        return NULL;
    }

    trace_rdma_pci_dma_map(addr, p, len);

    return p;
}

void rdma_pci_dma_unmap(PCIDevice *dev, void *buffer, dma_addr_t len)
{
    trace_rdma_pci_dma_unmap(buffer);
    if (buffer) {
        pci_dma_unmap(dev, buffer, len, DMA_DIRECTION_TO_DEVICE, 0);
    }
}
