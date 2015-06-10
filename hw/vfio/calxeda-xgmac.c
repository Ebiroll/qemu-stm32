/*
 * calxeda xgmac VFIO device
 *
 * Copyright Linaro Limited, 2014
 *
 * Authors:
 *  Eric Auger <eric.auger@linaro.org>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

#include "hw/vfio/vfio-calxeda-xgmac.h"

static void calxeda_xgmac_realize(DeviceState *dev, Error **errp)
{
    VFIOPlatformDevice *vdev = VFIO_PLATFORM_DEVICE(dev);
    VFIOCalxedaXgmacDeviceClass *k = VFIO_CALXEDA_XGMAC_DEVICE_GET_CLASS(dev);

    vdev->compat = g_strdup("calxeda,hb-xgmac");

    k->parent_realize(dev, errp);
}

static const VMStateDescription vfio_platform_calxeda_xgmac_vmstate = {
    .name = TYPE_VFIO_CALXEDA_XGMAC,
    .unmigratable = 1,
};

static void vfio_calxeda_xgmac_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    VFIOCalxedaXgmacDeviceClass *vcxc =
        VFIO_CALXEDA_XGMAC_DEVICE_CLASS(klass);
    vcxc->parent_realize = dc->realize;
    dc->realize = calxeda_xgmac_realize;
    dc->desc = "VFIO Calxeda XGMAC";
    dc->vmsd = &vfio_platform_calxeda_xgmac_vmstate;
}

static const TypeInfo vfio_calxeda_xgmac_dev_info = {
    .name = TYPE_VFIO_CALXEDA_XGMAC,
    .parent = TYPE_VFIO_PLATFORM,
    .instance_size = sizeof(VFIOCalxedaXgmacDevice),
    .class_init = vfio_calxeda_xgmac_class_init,
    .class_size = sizeof(VFIOCalxedaXgmacDeviceClass),
};

static void register_calxeda_xgmac_dev_type(void)
{
    type_register_static(&vfio_calxeda_xgmac_dev_info);
}

type_init(register_calxeda_xgmac_dev_type)
