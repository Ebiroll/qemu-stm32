/*
 * QEMU Synchronous Serial Interface support
 *
 * Copyright (c) 2009 CodeSourcery.
 * Copyright (c) 2012 Peter A.G. Crosthwaite (peter.crosthwaite@petalogix.com)
 * Copyright (c) 2012 PetaLogix Pty Ltd.
 * Written by Paul Brook
 *
 * This code is licensed under the GNU GPL v2.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#include "ssi.h"

struct SSIBus {
    BusState qbus;
};

#define TYPE_SSI_BUS "SSI"
#define SSI_BUS(obj) OBJECT_CHECK(SSIBus, (obj), TYPE_SSI_BUS)

static const TypeInfo ssi_bus_info = {
    .name = TYPE_SSI_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(SSIBus),
};

static int ssi_slave_init(DeviceState *dev)
{
    SSISlave *s = SSI_SLAVE(dev);
    SSISlaveClass *ssc = SSI_SLAVE_GET_CLASS(s);

    return ssc->init(s);
}

static void ssi_slave_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->init = ssi_slave_init;
    dc->bus_type = TYPE_SSI_BUS;
}

static TypeInfo ssi_slave_info = {
    .name = TYPE_SSI_SLAVE,
    .parent = TYPE_DEVICE,
    .class_init = ssi_slave_class_init,
    .class_size = sizeof(SSISlaveClass),
    .abstract = true,
};

DeviceState *ssi_create_slave(SSIBus *bus, const char *name)
{
    DeviceState *dev;
    dev = qdev_create(&bus->qbus, name);
    qdev_init_nofail(dev);
    return dev;
}

SSIBus *ssi_create_bus(DeviceState *parent, const char *name)
{
    BusState *bus;
    bus = qbus_create(TYPE_SSI_BUS, parent, name);
    return FROM_QBUS(SSIBus, bus);
}

uint32_t ssi_transfer(SSIBus *bus, uint32_t val)
{
    BusChild *kid;
    SSISlaveClass *ssc;
    uint32_t r = 0;

    QTAILQ_FOREACH(kid, &bus->qbus.children, sibling) {
        SSISlave *slave = SSI_SLAVE(kid->child);
        ssc = SSI_SLAVE_GET_CLASS(slave);
        r |= ssc->transfer(slave, val);
    }

    return r;
}

static void ssi_slave_register_types(void)
{
    type_register_static(&ssi_bus_info);
    type_register_static(&ssi_slave_info);
}

type_init(ssi_slave_register_types)
