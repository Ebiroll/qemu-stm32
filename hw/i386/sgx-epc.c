/*
 * SGX EPC device
 *
 * Copyright (C) 2019 Intel Corporation
 *
 * Authors:
 *   Sean Christopherson <sean.j.christopherson@intel.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */
#include "qemu/osdep.h"
#include "hw/i386/pc.h"
#include "hw/i386/sgx-epc.h"
#include "hw/mem/memory-device.h"
#include "hw/qdev-properties.h"
#include "monitor/qdev.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/config-file.h"
#include "qemu/error-report.h"
#include "qemu/option.h"
#include "qemu/units.h"
#include "target/i386/cpu.h"
#include "exec/address-spaces.h"

static Property sgx_epc_properties[] = {
    DEFINE_PROP_UINT64(SGX_EPC_ADDR_PROP, SGXEPCDevice, addr, 0),
    DEFINE_PROP_LINK(SGX_EPC_MEMDEV_PROP, SGXEPCDevice, hostmem,
                     TYPE_MEMORY_BACKEND_EPC, HostMemoryBackendEpc *),
    DEFINE_PROP_END_OF_LIST(),
};

static void sgx_epc_get_size(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    Error *local_err = NULL;
    uint64_t value;

    value = memory_device_get_region_size(MEMORY_DEVICE(obj), &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    visit_type_uint64(v, name, &value, errp);
}

static void sgx_epc_init(Object *obj)
{
    object_property_add(obj, SGX_EPC_SIZE_PROP, "uint64", sgx_epc_get_size,
                        NULL, NULL, NULL);
}

static void sgx_epc_realize(DeviceState *dev, Error **errp)
{
    PCMachineState *pcms = PC_MACHINE(qdev_get_machine());
    X86MachineState *x86ms = X86_MACHINE(pcms);
    SGXEPCDevice *epc = SGX_EPC(dev);
    HostMemoryBackend *hostmem;
    const char *path;

    if (x86ms->boot_cpus != 0) {
        error_setg(errp, "'" TYPE_SGX_EPC "' can't be created after vCPUs,"
                         "e.g. via -device");
        return;
    }

    if (!epc->hostmem) {
        error_setg(errp, "'" SGX_EPC_MEMDEV_PROP "' property is not set");
        return;
    }
    hostmem = MEMORY_BACKEND(epc->hostmem);
    if (host_memory_backend_is_mapped(hostmem)) {
        path = object_get_canonical_path_component(OBJECT(hostmem));
        error_setg(errp, "can't use already busy memdev: %s", path);
        return;
    }

    error_setg(errp, "'" TYPE_SGX_EPC "' not supported");
}

static void sgx_epc_unrealize(DeviceState *dev)
{
    SGXEPCDevice *epc = SGX_EPC(dev);
    HostMemoryBackend *hostmem = MEMORY_BACKEND(epc->hostmem);

    host_memory_backend_set_mapped(hostmem, false);
}

static uint64_t sgx_epc_md_get_addr(const MemoryDeviceState *md)
{
    const SGXEPCDevice *epc = SGX_EPC(md);

    return epc->addr;
}

static void sgx_epc_md_set_addr(MemoryDeviceState *md, uint64_t addr,
                                Error **errp)
{
    object_property_set_uint(OBJECT(md), SGX_EPC_ADDR_PROP, addr, errp);
}

static uint64_t sgx_epc_md_get_plugged_size(const MemoryDeviceState *md,
                                            Error **errp)
{
    return 0;
}

static MemoryRegion *sgx_epc_md_get_memory_region(MemoryDeviceState *md,
                                                  Error **errp)
{
    SGXEPCDevice *epc = SGX_EPC(md);
    HostMemoryBackend *hostmem;

    if (!epc->hostmem) {
        error_setg(errp, "'" SGX_EPC_MEMDEV_PROP "' property must be set");
        return NULL;
    }

    hostmem = MEMORY_BACKEND(epc->hostmem);
    return host_memory_backend_get_memory(hostmem);
}

static void sgx_epc_md_fill_device_info(const MemoryDeviceState *md,
                                        MemoryDeviceInfo *info)
{
    /* TODO */
}

static void sgx_epc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    MemoryDeviceClass *mdc = MEMORY_DEVICE_CLASS(oc);

    dc->hotpluggable = false;
    dc->realize = sgx_epc_realize;
    dc->unrealize = sgx_epc_unrealize;
    dc->desc = "SGX EPC section";
    device_class_set_props(dc, sgx_epc_properties);

    mdc->get_addr = sgx_epc_md_get_addr;
    mdc->set_addr = sgx_epc_md_set_addr;
    mdc->get_plugged_size = sgx_epc_md_get_plugged_size;
    mdc->get_memory_region = sgx_epc_md_get_memory_region;
    mdc->fill_device_info = sgx_epc_md_fill_device_info;
}

static TypeInfo sgx_epc_info = {
    .name          = TYPE_SGX_EPC,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(SGXEPCDevice),
    .instance_init = sgx_epc_init,
    .class_init    = sgx_epc_class_init,
    .class_size    = sizeof(DeviceClass),
    .interfaces = (InterfaceInfo[]) {
        { TYPE_MEMORY_DEVICE },
        { }
    },
};

static void sgx_epc_register_types(void)
{
    type_register_static(&sgx_epc_info);
}

type_init(sgx_epc_register_types)
