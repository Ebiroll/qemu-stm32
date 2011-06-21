/*
 * Copyright (C) 2011       Citrix Ltd.
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

#include "config.h"

#include "cpu.h"
#include "qemu-common.h"
#include "cpu-common.h"
#include "xen-mapcache.h"

void xen_map_cache_init(void)
{
}

uint8_t *xen_map_cache(target_phys_addr_t phys_addr, target_phys_addr_t size,
                       uint8_t lock)
{
    return qemu_get_ram_ptr(phys_addr);
}

ram_addr_t xen_ram_addr_from_mapcache(void *ptr)
{
    return -1;
}

void xen_invalidate_map_cache(void)
{
}

void xen_invalidate_map_cache_entry(uint8_t *buffer)
{
}
