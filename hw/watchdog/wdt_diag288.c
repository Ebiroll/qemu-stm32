/*
 * watchdog device diag288 support
 *
 * Copyright IBM, Corp. 2015
 *
 * Authors:
 *  Xu Wang <gesaint@linux.vnet.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or (at your
 * option) any later version.  See the COPYING file in the top-level directory.
 *
 */

#include "sysemu/watchdog.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "hw/watchdog/wdt_diag288.h"

static WatchdogTimerModel model = {
    .wdt_name = TYPE_WDT_DIAG288,
    .wdt_description = "diag288 device for s390x platform",
};

static void wdt_diag288_reset(DeviceState *dev)
{
    DIAG288State *diag288 = DIAG288(dev);

    diag288->enabled = false;
    timer_del(diag288->timer);
}

static void diag288_timer_expired(void *dev)
{
    qemu_log_mask(CPU_LOG_RESET, "Watchdog timer expired.\n");
    watchdog_perform_action();
    wdt_diag288_reset(dev);
}

static int wdt_diag288_handle_timer(DIAG288State *diag288,
                                     uint64_t func, uint64_t timeout)
{
    switch (func) {
    case WDT_DIAG288_INIT:
        diag288->enabled = true;
        /* fall through */
    case WDT_DIAG288_CHANGE:
        if (!diag288->enabled) {
            return -1;
        }
        timer_mod(diag288->timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  timeout * get_ticks_per_sec());
        break;
    case WDT_DIAG288_CANCEL:
        if (!diag288->enabled) {
            return -1;
        }
        diag288->enabled = false;
        timer_del(diag288->timer);
        break;
    default:
        return -1;
    }

    return 0;
}

static void wdt_diag288_realize(DeviceState *dev, Error **errp)
{
    DIAG288State *diag288 = DIAG288(dev);

    diag288->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, diag288_timer_expired,
                                  dev);
}

static void wdt_diag288_unrealize(DeviceState *dev, Error **errp)
{
    DIAG288State *diag288 = DIAG288(dev);

    timer_del(diag288->timer);
    timer_free(diag288->timer);
}

static void wdt_diag288_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    DIAG288Class *diag288 = DIAG288_CLASS(klass);

    dc->realize = wdt_diag288_realize;
    dc->unrealize = wdt_diag288_unrealize;
    dc->reset = wdt_diag288_reset;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    diag288->handle_timer = wdt_diag288_handle_timer;
}

static const TypeInfo wdt_diag288_info = {
    .class_init = wdt_diag288_class_init,
    .parent = TYPE_DEVICE,
    .name  = TYPE_WDT_DIAG288,
    .instance_size  = sizeof(DIAG288State),
    .class_size = sizeof(DIAG288Class),
};

static void wdt_diag288_register_types(void)
{
    watchdog_add_model(&model);
    type_register_static(&wdt_diag288_info);
}

type_init(wdt_diag288_register_types)
