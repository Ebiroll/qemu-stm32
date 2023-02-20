/*
 * QEMU NVM Express Virtual Namespace
 *
 * Copyright (c) 2019 CNEX Labs
 * Copyright (c) 2020 Samsung Electronics
 *
 * Authors:
 *  Klaus Jensen      <k.jensen@samsung.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2. See the
 * COPYING file in the top-level directory.
 *
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "sysemu/sysemu.h"
#include "sysemu/block-backend.h"

#include "nvme.h"
#include "trace.h"

#define MIN_DISCARD_GRANULARITY (4 * KiB)
#define NVME_DEFAULT_ZONE_SIZE   (128 * MiB)

void nvme_ns_init_format(NvmeNamespace *ns)
{
    NvmeIdNs *id_ns = &ns->id_ns;
    BlockDriverInfo bdi;
    int npdg, ret;
    int64_t nlbas;

    ns->lbaf = id_ns->lbaf[NVME_ID_NS_FLBAS_INDEX(id_ns->flbas)];
    ns->lbasz = 1 << ns->lbaf.ds;

    nlbas = ns->size / (ns->lbasz + ns->lbaf.ms);

    id_ns->nsze = cpu_to_le64(nlbas);

    /* no thin provisioning */
    id_ns->ncap = id_ns->nsze;
    id_ns->nuse = id_ns->ncap;

    ns->moff = nlbas << ns->lbaf.ds;

    npdg = ns->blkconf.discard_granularity / ns->lbasz;

    ret = bdrv_get_info(blk_bs(ns->blkconf.blk), &bdi);
    if (ret >= 0 && bdi.cluster_size > ns->blkconf.discard_granularity) {
        npdg = bdi.cluster_size / ns->lbasz;
    }

    id_ns->npda = id_ns->npdg = npdg - 1;
}

static int nvme_ns_init(NvmeNamespace *ns, Error **errp)
{
    static uint64_t ns_count;
    NvmeIdNs *id_ns = &ns->id_ns;
    NvmeIdNsNvm *id_ns_nvm = &ns->id_ns_nvm;
    uint8_t ds;
    uint16_t ms;
    int i;

    ns->csi = NVME_CSI_NVM;
    ns->status = 0x0;

    ns->id_ns.dlfeat = 0x1;

    /* support DULBE and I/O optimization fields */
    id_ns->nsfeat |= (0x4 | 0x10);

    if (ns->params.shared) {
        id_ns->nmic |= NVME_NMIC_NS_SHARED;
    }

    /* Substitute a missing EUI-64 by an autogenerated one */
    ++ns_count;
    if (!ns->params.eui64 && ns->params.eui64_default) {
        ns->params.eui64 = ns_count + NVME_EUI64_DEFAULT;
    }

    /* simple copy */
    id_ns->mssrl = cpu_to_le16(ns->params.mssrl);
    id_ns->mcl = cpu_to_le32(ns->params.mcl);
    id_ns->msrc = ns->params.msrc;
    id_ns->eui64 = cpu_to_be64(ns->params.eui64);

    ds = 31 - clz32(ns->blkconf.logical_block_size);
    ms = ns->params.ms;

    id_ns->mc = NVME_ID_NS_MC_EXTENDED | NVME_ID_NS_MC_SEPARATE;

    if (ms && ns->params.mset) {
        id_ns->flbas |= NVME_ID_NS_FLBAS_EXTENDED;
    }

    id_ns->dpc = 0x1f;
    id_ns->dps = ns->params.pi;
    if (ns->params.pi && ns->params.pil) {
        id_ns->dps |= NVME_ID_NS_DPS_FIRST_EIGHT;
    }

    ns->pif = ns->params.pif;

    static const NvmeLBAF lbaf[16] = {
        [0] = { .ds =  9           },
        [1] = { .ds =  9, .ms =  8 },
        [2] = { .ds =  9, .ms = 16 },
        [3] = { .ds =  9, .ms = 64 },
        [4] = { .ds = 12           },
        [5] = { .ds = 12, .ms =  8 },
        [6] = { .ds = 12, .ms = 16 },
        [7] = { .ds = 12, .ms = 64 },
    };

    ns->nlbaf = 8;

    memcpy(&id_ns->lbaf, &lbaf, sizeof(lbaf));

    for (i = 0; i < ns->nlbaf; i++) {
        NvmeLBAF *lbaf = &id_ns->lbaf[i];
        if (lbaf->ds == ds) {
            if (lbaf->ms == ms) {
                id_ns->flbas |= i;
                goto lbaf_found;
            }
        }
    }

    /* add non-standard lba format */
    id_ns->lbaf[ns->nlbaf].ds = ds;
    id_ns->lbaf[ns->nlbaf].ms = ms;
    ns->nlbaf++;

    id_ns->flbas |= i;


lbaf_found:
    id_ns_nvm->elbaf[i] = (ns->pif & 0x3) << 7;
    id_ns->nlbaf = ns->nlbaf - 1;
    nvme_ns_init_format(ns);

    return 0;
}

static int nvme_ns_init_blk(NvmeNamespace *ns, Error **errp)
{
    bool read_only;

    if (!blkconf_blocksizes(&ns->blkconf, errp)) {
        return -1;
    }

    read_only = !blk_supports_write_perm(ns->blkconf.blk);
    if (!blkconf_apply_backend_options(&ns->blkconf, read_only, false, errp)) {
        return -1;
    }

    if (ns->blkconf.discard_granularity == -1) {
        ns->blkconf.discard_granularity =
            MAX(ns->blkconf.logical_block_size, MIN_DISCARD_GRANULARITY);
    }

    ns->size = blk_getlength(ns->blkconf.blk);
    if (ns->size < 0) {
        error_setg_errno(errp, -ns->size, "could not get blockdev size");
        return -1;
    }

    return 0;
}

static int nvme_ns_zoned_check_calc_geometry(NvmeNamespace *ns, Error **errp)
{
    uint64_t zone_size, zone_cap;

    /* Make sure that the values of ZNS properties are sane */
    if (ns->params.zone_size_bs) {
        zone_size = ns->params.zone_size_bs;
    } else {
        zone_size = NVME_DEFAULT_ZONE_SIZE;
    }
    if (ns->params.zone_cap_bs) {
        zone_cap = ns->params.zone_cap_bs;
    } else {
        zone_cap = zone_size;
    }
    if (zone_cap > zone_size) {
        error_setg(errp, "zone capacity %"PRIu64"B exceeds "
                   "zone size %"PRIu64"B", zone_cap, zone_size);
        return -1;
    }
    if (zone_size < ns->lbasz) {
        error_setg(errp, "zone size %"PRIu64"B too small, "
                   "must be at least %zuB", zone_size, ns->lbasz);
        return -1;
    }
    if (zone_cap < ns->lbasz) {
        error_setg(errp, "zone capacity %"PRIu64"B too small, "
                   "must be at least %zuB", zone_cap, ns->lbasz);
        return -1;
    }

    /*
     * Save the main zone geometry values to avoid
     * calculating them later again.
     */
    ns->zone_size = zone_size / ns->lbasz;
    ns->zone_capacity = zone_cap / ns->lbasz;
    ns->num_zones = le64_to_cpu(ns->id_ns.nsze) / ns->zone_size;

    /* Do a few more sanity checks of ZNS properties */
    if (!ns->num_zones) {
        error_setg(errp,
                   "insufficient drive capacity, must be at least the size "
                   "of one zone (%"PRIu64"B)", zone_size);
        return -1;
    }

    return 0;
}

static void nvme_ns_zoned_init_state(NvmeNamespace *ns)
{
    uint64_t start = 0, zone_size = ns->zone_size;
    uint64_t capacity = ns->num_zones * zone_size;
    NvmeZone *zone;
    int i;

    ns->zone_array = g_new0(NvmeZone, ns->num_zones);
    if (ns->params.zd_extension_size) {
        ns->zd_extensions = g_malloc0(ns->params.zd_extension_size *
                                      ns->num_zones);
    }

    QTAILQ_INIT(&ns->exp_open_zones);
    QTAILQ_INIT(&ns->imp_open_zones);
    QTAILQ_INIT(&ns->closed_zones);
    QTAILQ_INIT(&ns->full_zones);

    zone = ns->zone_array;
    for (i = 0; i < ns->num_zones; i++, zone++) {
        if (start + zone_size > capacity) {
            zone_size = capacity - start;
        }
        zone->d.zt = NVME_ZONE_TYPE_SEQ_WRITE;
        nvme_set_zone_state(zone, NVME_ZONE_STATE_EMPTY);
        zone->d.za = 0;
        zone->d.zcap = ns->zone_capacity;
        zone->d.zslba = start;
        zone->d.wp = start;
        zone->w_ptr = start;
        start += zone_size;
    }

    ns->zone_size_log2 = 0;
    if (is_power_of_2(ns->zone_size)) {
        ns->zone_size_log2 = 63 - clz64(ns->zone_size);
    }
}

static void nvme_ns_init_zoned(NvmeNamespace *ns)
{
    NvmeIdNsZoned *id_ns_z;
    int i;

    nvme_ns_zoned_init_state(ns);

    id_ns_z = g_new0(NvmeIdNsZoned, 1);

    /* MAR/MOR are zeroes-based, FFFFFFFFFh means no limit */
    id_ns_z->mar = cpu_to_le32(ns->params.max_active_zones - 1);
    id_ns_z->mor = cpu_to_le32(ns->params.max_open_zones - 1);
    id_ns_z->zoc = 0;
    id_ns_z->ozcs = ns->params.cross_zone_read ?
        NVME_ID_NS_ZONED_OZCS_RAZB : 0x00;

    for (i = 0; i <= ns->id_ns.nlbaf; i++) {
        id_ns_z->lbafe[i].zsze = cpu_to_le64(ns->zone_size);
        id_ns_z->lbafe[i].zdes =
            ns->params.zd_extension_size >> 6; /* Units of 64B */
    }

    if (ns->params.zrwas) {
        ns->zns.numzrwa = ns->params.numzrwa ?
            ns->params.numzrwa : ns->num_zones;

        ns->zns.zrwas = ns->params.zrwas >> ns->lbaf.ds;
        ns->zns.zrwafg = ns->params.zrwafg >> ns->lbaf.ds;

        id_ns_z->ozcs |= NVME_ID_NS_ZONED_OZCS_ZRWASUP;
        id_ns_z->zrwacap = NVME_ID_NS_ZONED_ZRWACAP_EXPFLUSHSUP;

        id_ns_z->numzrwa = cpu_to_le32(ns->params.numzrwa);
        id_ns_z->zrwas = cpu_to_le16(ns->zns.zrwas);
        id_ns_z->zrwafg = cpu_to_le16(ns->zns.zrwafg);
    }

    id_ns_z->ozcs = cpu_to_le16(id_ns_z->ozcs);

    ns->csi = NVME_CSI_ZONED;
    ns->id_ns.nsze = cpu_to_le64(ns->num_zones * ns->zone_size);
    ns->id_ns.ncap = ns->id_ns.nsze;
    ns->id_ns.nuse = ns->id_ns.ncap;

    /*
     * The device uses the BDRV_BLOCK_ZERO flag to determine the "deallocated"
     * status of logical blocks. Since the spec defines that logical blocks
     * SHALL be deallocated when then zone is in the Empty or Offline states,
     * we can only support DULBE if the zone size is a multiple of the
     * calculated NPDG.
     */
    if (ns->zone_size % (ns->id_ns.npdg + 1)) {
        warn_report("the zone size (%"PRIu64" blocks) is not a multiple of "
                    "the calculated deallocation granularity (%d blocks); "
                    "DULBE support disabled",
                    ns->zone_size, ns->id_ns.npdg + 1);

        ns->id_ns.nsfeat &= ~0x4;
    }

    ns->id_ns_zoned = id_ns_z;
}

static void nvme_clear_zone(NvmeNamespace *ns, NvmeZone *zone)
{
    uint8_t state;

    zone->w_ptr = zone->d.wp;
    state = nvme_get_zone_state(zone);
    if (zone->d.wp != zone->d.zslba ||
        (zone->d.za & NVME_ZA_ZD_EXT_VALID)) {
        if (state != NVME_ZONE_STATE_CLOSED) {
            trace_pci_nvme_clear_ns_close(state, zone->d.zslba);
            nvme_set_zone_state(zone, NVME_ZONE_STATE_CLOSED);
        }
        nvme_aor_inc_active(ns);
        QTAILQ_INSERT_HEAD(&ns->closed_zones, zone, entry);
    } else {
        trace_pci_nvme_clear_ns_reset(state, zone->d.zslba);
        if (zone->d.za & NVME_ZA_ZRWA_VALID) {
            zone->d.za &= ~NVME_ZA_ZRWA_VALID;
            ns->zns.numzrwa++;
        }
        nvme_set_zone_state(zone, NVME_ZONE_STATE_EMPTY);
    }
}

/*
 * Close all the zones that are currently open.
 */
static void nvme_zoned_ns_shutdown(NvmeNamespace *ns)
{
    NvmeZone *zone, *next;

    QTAILQ_FOREACH_SAFE(zone, &ns->closed_zones, entry, next) {
        QTAILQ_REMOVE(&ns->closed_zones, zone, entry);
        nvme_aor_dec_active(ns);
        nvme_clear_zone(ns, zone);
    }
    QTAILQ_FOREACH_SAFE(zone, &ns->imp_open_zones, entry, next) {
        QTAILQ_REMOVE(&ns->imp_open_zones, zone, entry);
        nvme_aor_dec_open(ns);
        nvme_aor_dec_active(ns);
        nvme_clear_zone(ns, zone);
    }
    QTAILQ_FOREACH_SAFE(zone, &ns->exp_open_zones, entry, next) {
        QTAILQ_REMOVE(&ns->exp_open_zones, zone, entry);
        nvme_aor_dec_open(ns);
        nvme_aor_dec_active(ns);
        nvme_clear_zone(ns, zone);
    }

    assert(ns->nr_open_zones == 0);
}

static int nvme_ns_check_constraints(NvmeNamespace *ns, Error **errp)
{
    unsigned int pi_size;

    if (!ns->blkconf.blk) {
        error_setg(errp, "block backend not configured");
        return -1;
    }

    if (ns->params.pi) {
        if (ns->params.pi > NVME_ID_NS_DPS_TYPE_3) {
            error_setg(errp, "invalid 'pi' value");
            return -1;
        }

        switch (ns->params.pif) {
        case NVME_PI_GUARD_16:
            pi_size = 8;
            break;
        case NVME_PI_GUARD_64:
            pi_size = 16;
            break;
        default:
            error_setg(errp, "invalid 'pif'");
            return -1;
        }

        if (ns->params.ms < pi_size) {
            error_setg(errp, "at least %u bytes of metadata required to "
                       "enable protection information", pi_size);
            return -1;
        }
    }

    if (ns->params.nsid > NVME_MAX_NAMESPACES) {
        error_setg(errp, "invalid namespace id (must be between 0 and %d)",
                   NVME_MAX_NAMESPACES);
        return -1;
    }

    if (ns->params.zoned) {
        if (ns->params.max_active_zones) {
            if (ns->params.max_open_zones > ns->params.max_active_zones) {
                error_setg(errp, "max_open_zones (%u) exceeds "
                           "max_active_zones (%u)", ns->params.max_open_zones,
                           ns->params.max_active_zones);
                return -1;
            }

            if (!ns->params.max_open_zones) {
                ns->params.max_open_zones = ns->params.max_active_zones;
            }
        }

        if (ns->params.zd_extension_size) {
            if (ns->params.zd_extension_size & 0x3f) {
                error_setg(errp, "zone descriptor extension size must be a "
                           "multiple of 64B");
                return -1;
            }
            if ((ns->params.zd_extension_size >> 6) > 0xff) {
                error_setg(errp,
                           "zone descriptor extension size is too large");
                return -1;
            }
        }

        if (ns->params.zrwas) {
            if (ns->params.zrwas % ns->blkconf.logical_block_size) {
                error_setg(errp, "zone random write area size (zoned.zrwas "
                           "%"PRIu64") must be a multiple of the logical "
                           "block size (logical_block_size %"PRIu32")",
                           ns->params.zrwas, ns->blkconf.logical_block_size);
                return -1;
            }

            if (ns->params.zrwafg == -1) {
                ns->params.zrwafg = ns->blkconf.logical_block_size;
            }

            if (ns->params.zrwas % ns->params.zrwafg) {
                error_setg(errp, "zone random write area size (zoned.zrwas "
                           "%"PRIu64") must be a multiple of the zone random "
                           "write area flush granularity (zoned.zrwafg, "
                           "%"PRIu64")", ns->params.zrwas, ns->params.zrwafg);
                return -1;
            }

            if (ns->params.max_active_zones) {
                if (ns->params.numzrwa > ns->params.max_active_zones) {
                    error_setg(errp, "number of zone random write area "
                               "resources (zoned.numzrwa, %d) must be less "
                               "than or equal to maximum active resources "
                               "(zoned.max_active_zones, %d)",
                               ns->params.numzrwa,
                               ns->params.max_active_zones);
                    return -1;
                }
            }
        }
    }

    return 0;
}

int nvme_ns_setup(NvmeNamespace *ns, Error **errp)
{
    if (nvme_ns_check_constraints(ns, errp)) {
        return -1;
    }

    if (nvme_ns_init_blk(ns, errp)) {
        return -1;
    }

    if (nvme_ns_init(ns, errp)) {
        return -1;
    }
    if (ns->params.zoned) {
        if (nvme_ns_zoned_check_calc_geometry(ns, errp) != 0) {
            return -1;
        }
        nvme_ns_init_zoned(ns);
    }

    return 0;
}

void nvme_ns_drain(NvmeNamespace *ns)
{
    blk_drain(ns->blkconf.blk);
}

void nvme_ns_shutdown(NvmeNamespace *ns)
{
    blk_flush(ns->blkconf.blk);
    if (ns->params.zoned) {
        nvme_zoned_ns_shutdown(ns);
    }
}

void nvme_ns_cleanup(NvmeNamespace *ns)
{
    if (ns->params.zoned) {
        g_free(ns->id_ns_zoned);
        g_free(ns->zone_array);
        g_free(ns->zd_extensions);
    }
}

static void nvme_ns_unrealize(DeviceState *dev)
{
    NvmeNamespace *ns = NVME_NS(dev);

    nvme_ns_drain(ns);
    nvme_ns_shutdown(ns);
    nvme_ns_cleanup(ns);
}

static void nvme_ns_realize(DeviceState *dev, Error **errp)
{
    NvmeNamespace *ns = NVME_NS(dev);
    BusState *s = qdev_get_parent_bus(dev);
    NvmeCtrl *n = NVME(s->parent);
    NvmeSubsystem *subsys = n->subsys;
    uint32_t nsid = ns->params.nsid;
    int i;

    if (!n->subsys) {
        /* If no subsys, the ns cannot be attached to more than one ctrl. */
        ns->params.shared = false;
        if (ns->params.detached) {
            error_setg(errp, "detached requires that the nvme device is "
                       "linked to an nvme-subsys device");
            return;
        }
    } else {
        /*
         * If this namespace belongs to a subsystem (through a link on the
         * controller device), reparent the device.
         */
        if (!qdev_set_parent_bus(dev, &subsys->bus.parent_bus, errp)) {
            return;
        }
        ns->subsys = subsys;
    }

    if (nvme_ns_setup(ns, errp)) {
        return;
    }

    if (!nsid) {
        for (i = 1; i <= NVME_MAX_NAMESPACES; i++) {
            if (nvme_ns(n, i) || nvme_subsys_ns(subsys, i)) {
                continue;
            }

            nsid = ns->params.nsid = i;
            break;
        }

        if (!nsid) {
            error_setg(errp, "no free namespace id");
            return;
        }
    } else {
        if (nvme_ns(n, nsid) || nvme_subsys_ns(subsys, nsid)) {
            error_setg(errp, "namespace id '%d' already allocated", nsid);
            return;
        }
    }

    if (subsys) {
        subsys->namespaces[nsid] = ns;

        ns->id_ns.endgid = cpu_to_le16(0x1);

        if (ns->params.detached) {
            return;
        }

        if (ns->params.shared) {
            for (i = 0; i < ARRAY_SIZE(subsys->ctrls); i++) {
                NvmeCtrl *ctrl = subsys->ctrls[i];

                if (ctrl && ctrl != SUBSYS_SLOT_RSVD) {
                    nvme_attach_ns(ctrl, ns);
                }
            }

            return;
        }

    }

    nvme_attach_ns(n, ns);
}

static Property nvme_ns_props[] = {
    DEFINE_BLOCK_PROPERTIES(NvmeNamespace, blkconf),
    DEFINE_PROP_BOOL("detached", NvmeNamespace, params.detached, false),
    DEFINE_PROP_BOOL("shared", NvmeNamespace, params.shared, true),
    DEFINE_PROP_UINT32("nsid", NvmeNamespace, params.nsid, 0),
    DEFINE_PROP_UUID_NODEFAULT("uuid", NvmeNamespace, params.uuid),
    DEFINE_PROP_UINT64("eui64", NvmeNamespace, params.eui64, 0),
    DEFINE_PROP_UINT16("ms", NvmeNamespace, params.ms, 0),
    DEFINE_PROP_UINT8("mset", NvmeNamespace, params.mset, 0),
    DEFINE_PROP_UINT8("pi", NvmeNamespace, params.pi, 0),
    DEFINE_PROP_UINT8("pil", NvmeNamespace, params.pil, 0),
    DEFINE_PROP_UINT8("pif", NvmeNamespace, params.pif, 0),
    DEFINE_PROP_UINT16("mssrl", NvmeNamespace, params.mssrl, 128),
    DEFINE_PROP_UINT32("mcl", NvmeNamespace, params.mcl, 128),
    DEFINE_PROP_UINT8("msrc", NvmeNamespace, params.msrc, 127),
    DEFINE_PROP_BOOL("zoned", NvmeNamespace, params.zoned, false),
    DEFINE_PROP_SIZE("zoned.zone_size", NvmeNamespace, params.zone_size_bs,
                     NVME_DEFAULT_ZONE_SIZE),
    DEFINE_PROP_SIZE("zoned.zone_capacity", NvmeNamespace, params.zone_cap_bs,
                     0),
    DEFINE_PROP_BOOL("zoned.cross_read", NvmeNamespace,
                     params.cross_zone_read, false),
    DEFINE_PROP_UINT32("zoned.max_active", NvmeNamespace,
                       params.max_active_zones, 0),
    DEFINE_PROP_UINT32("zoned.max_open", NvmeNamespace,
                       params.max_open_zones, 0),
    DEFINE_PROP_UINT32("zoned.descr_ext_size", NvmeNamespace,
                       params.zd_extension_size, 0),
    DEFINE_PROP_UINT32("zoned.numzrwa", NvmeNamespace, params.numzrwa, 0),
    DEFINE_PROP_SIZE("zoned.zrwas", NvmeNamespace, params.zrwas, 0),
    DEFINE_PROP_SIZE("zoned.zrwafg", NvmeNamespace, params.zrwafg, -1),
    DEFINE_PROP_BOOL("eui64-default", NvmeNamespace, params.eui64_default,
                     false),
    DEFINE_PROP_END_OF_LIST(),
};

static void nvme_ns_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);

    dc->bus_type = TYPE_NVME_BUS;
    dc->realize = nvme_ns_realize;
    dc->unrealize = nvme_ns_unrealize;
    device_class_set_props(dc, nvme_ns_props);
    dc->desc = "Virtual NVMe namespace";
}

static void nvme_ns_instance_init(Object *obj)
{
    NvmeNamespace *ns = NVME_NS(obj);
    char *bootindex = g_strdup_printf("/namespace@%d,0", ns->params.nsid);

    device_add_bootindex_property(obj, &ns->bootindex, "bootindex",
                                  bootindex, DEVICE(obj));

    g_free(bootindex);
}

static const TypeInfo nvme_ns_info = {
    .name = TYPE_NVME_NS,
    .parent = TYPE_DEVICE,
    .class_init = nvme_ns_class_init,
    .instance_size = sizeof(NvmeNamespace),
    .instance_init = nvme_ns_instance_init,
};

static void nvme_ns_register_types(void)
{
    type_register_static(&nvme_ns_info);
}

type_init(nvme_ns_register_types)
