/*
 * SiFive CLINT (Core Local Interruptor)
 *
 * Copyright (c) 2016-2017 Sagar Karandikar, sagark@eecs.berkeley.edu
 * Copyright (c) 2017 SiFive, Inc.
 *
 * This provides real-time clock, timer and interprocessor interrupts.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "target/riscv/cpu.h"
#include "hw/qdev-properties.h"
#include "hw/intc/sifive_clint.h"
#include "qemu/timer.h"
#include "hw/irq.h"

typedef struct sifive_clint_callback {
    SiFiveCLINTState *s;
    int num;
} sifive_clint_callback;

static uint64_t cpu_riscv_read_rtc(uint32_t timebase_freq)
{
    return muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
        timebase_freq, NANOSECONDS_PER_SECOND);
}

/*
 * Called when timecmp is written to update the QEMU timer or immediately
 * trigger timer interrupt if mtimecmp <= current timer value.
 */
static void sifive_clint_write_timecmp(SiFiveCLINTState *s, RISCVCPU *cpu,
                                       int hartid,
                                       uint64_t value,
                                       uint32_t timebase_freq)
{
    uint64_t next;
    uint64_t diff;

    uint64_t rtc_r = cpu_riscv_read_rtc(timebase_freq);

    cpu->env.timecmp = value;
    if (cpu->env.timecmp <= rtc_r) {
        /* if we're setting an MTIMECMP value in the "past",
           immediately raise the timer interrupt */
        qemu_irq_raise(s->timer_irqs[hartid - s->hartid_base]);
        return;
    }

    /* otherwise, set up the future timer interrupt */
    qemu_irq_lower(s->timer_irqs[hartid - s->hartid_base]);
    diff = cpu->env.timecmp - rtc_r;
    /* back to ns (note args switched in muldiv64) */
    uint64_t ns_diff = muldiv64(diff, NANOSECONDS_PER_SECOND, timebase_freq);

    /*
     * check if ns_diff overflowed and check if the addition would potentially
     * overflow
     */
    if ((NANOSECONDS_PER_SECOND > timebase_freq && ns_diff < diff) ||
        ns_diff > INT64_MAX) {
        next = INT64_MAX;
    } else {
        /*
         * as it is very unlikely qemu_clock_get_ns will return a value
         * greater than INT64_MAX, no additional check is needed for an
         * unsigned integer overflow.
         */
        next = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns_diff;
        /*
         * if ns_diff is INT64_MAX next may still be outside the range
         * of a signed integer.
         */
        next = MIN(next, INT64_MAX);
    }

    timer_mod(cpu->env.timer, next);
}

/*
 * Callback used when the timer set using timer_mod expires.
 * Should raise the timer interrupt line
 */
static void sifive_clint_timer_cb(void *opaque)
{
    sifive_clint_callback *state = opaque;

    qemu_irq_raise(state->s->timer_irqs[state->num]);
}

/* CPU wants to read rtc or timecmp register */
static uint64_t sifive_clint_read(void *opaque, hwaddr addr, unsigned size)
{
    SiFiveCLINTState *clint = opaque;
    if (addr >= clint->sip_base &&
        addr < clint->sip_base + (clint->num_harts << 2)) {
        size_t hartid = clint->hartid_base + ((addr - clint->sip_base) >> 2);
        CPUState *cpu = qemu_get_cpu(hartid);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        if (!env) {
            error_report("clint: invalid timecmp hartid: %zu", hartid);
        } else if ((addr & 0x3) == 0) {
            return (env->mip & MIP_MSIP) > 0;
        } else {
            error_report("clint: invalid read: %08x", (uint32_t)addr);
            return 0;
        }
    } else if (addr >= clint->timecmp_base &&
        addr < clint->timecmp_base + (clint->num_harts << 3)) {
        size_t hartid = clint->hartid_base +
            ((addr - clint->timecmp_base) >> 3);
        CPUState *cpu = qemu_get_cpu(hartid);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        if (!env) {
            error_report("clint: invalid timecmp hartid: %zu", hartid);
        } else if ((addr & 0x7) == 0) {
            /* timecmp_lo */
            uint64_t timecmp = env->timecmp;
            return timecmp & 0xFFFFFFFF;
        } else if ((addr & 0x7) == 4) {
            /* timecmp_hi */
            uint64_t timecmp = env->timecmp;
            return (timecmp >> 32) & 0xFFFFFFFF;
        } else {
            error_report("clint: invalid read: %08x", (uint32_t)addr);
            return 0;
        }
    } else if (addr == clint->time_base) {
        /* time_lo */
        return cpu_riscv_read_rtc(clint->timebase_freq) & 0xFFFFFFFF;
    } else if (addr == clint->time_base + 4) {
        /* time_hi */
        return (cpu_riscv_read_rtc(clint->timebase_freq) >> 32) & 0xFFFFFFFF;
    }

    error_report("clint: invalid read: %08x", (uint32_t)addr);
    return 0;
}

/* CPU wrote to rtc or timecmp register */
static void sifive_clint_write(void *opaque, hwaddr addr, uint64_t value,
        unsigned size)
{
    SiFiveCLINTState *clint = opaque;

    if (addr >= clint->sip_base &&
        addr < clint->sip_base + (clint->num_harts << 2)) {
        size_t hartid = clint->hartid_base + ((addr - clint->sip_base) >> 2);
        CPUState *cpu = qemu_get_cpu(hartid);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        if (!env) {
            error_report("clint: invalid timecmp hartid: %zu", hartid);
        } else if ((addr & 0x3) == 0) {
            qemu_set_irq(clint->soft_irqs[hartid - clint->hartid_base], value);
        } else {
            error_report("clint: invalid sip write: %08x", (uint32_t)addr);
        }
        return;
    } else if (addr >= clint->timecmp_base &&
        addr < clint->timecmp_base + (clint->num_harts << 3)) {
        size_t hartid = clint->hartid_base +
            ((addr - clint->timecmp_base) >> 3);
        CPUState *cpu = qemu_get_cpu(hartid);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        if (!env) {
            error_report("clint: invalid timecmp hartid: %zu", hartid);
        } else if ((addr & 0x7) == 0) {
            /* timecmp_lo */
            uint64_t timecmp_hi = env->timecmp >> 32;
            sifive_clint_write_timecmp(clint, RISCV_CPU(cpu), hartid,
                timecmp_hi << 32 | (value & 0xFFFFFFFF), clint->timebase_freq);
            return;
        } else if ((addr & 0x7) == 4) {
            /* timecmp_hi */
            uint64_t timecmp_lo = env->timecmp;
            sifive_clint_write_timecmp(clint, RISCV_CPU(cpu), hartid,
                value << 32 | (timecmp_lo & 0xFFFFFFFF), clint->timebase_freq);
        } else {
            error_report("clint: invalid timecmp write: %08x", (uint32_t)addr);
        }
        return;
    } else if (addr == clint->time_base) {
        /* time_lo */
        error_report("clint: time_lo write not implemented");
        return;
    } else if (addr == clint->time_base + 4) {
        /* time_hi */
        error_report("clint: time_hi write not implemented");
        return;
    }

    error_report("clint: invalid write: %08x", (uint32_t)addr);
}

static const MemoryRegionOps sifive_clint_ops = {
    .read = sifive_clint_read,
    .write = sifive_clint_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8
    }
};

static Property sifive_clint_properties[] = {
    DEFINE_PROP_UINT32("hartid-base", SiFiveCLINTState, hartid_base, 0),
    DEFINE_PROP_UINT32("num-harts", SiFiveCLINTState, num_harts, 0),
    DEFINE_PROP_UINT32("sip-base", SiFiveCLINTState, sip_base, 0),
    DEFINE_PROP_UINT32("timecmp-base", SiFiveCLINTState, timecmp_base, 0),
    DEFINE_PROP_UINT32("time-base", SiFiveCLINTState, time_base, 0),
    DEFINE_PROP_UINT32("aperture-size", SiFiveCLINTState, aperture_size, 0),
    DEFINE_PROP_UINT32("timebase-freq", SiFiveCLINTState, timebase_freq, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void sifive_clint_realize(DeviceState *dev, Error **errp)
{
    SiFiveCLINTState *s = SIFIVE_CLINT(dev);
    memory_region_init_io(&s->mmio, OBJECT(dev), &sifive_clint_ops, s,
                          TYPE_SIFIVE_CLINT, s->aperture_size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);

    s->timer_irqs = g_malloc(sizeof(qemu_irq) * s->num_harts);
    qdev_init_gpio_out(dev, s->timer_irqs, s->num_harts);

    s->soft_irqs = g_malloc(sizeof(qemu_irq) * s->num_harts);
    qdev_init_gpio_out(dev, s->soft_irqs, s->num_harts);
}

static void sifive_clint_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = sifive_clint_realize;
    device_class_set_props(dc, sifive_clint_properties);
}

static const TypeInfo sifive_clint_info = {
    .name          = TYPE_SIFIVE_CLINT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SiFiveCLINTState),
    .class_init    = sifive_clint_class_init,
};

static void sifive_clint_register_types(void)
{
    type_register_static(&sifive_clint_info);
}

type_init(sifive_clint_register_types)

/*
 * Create CLINT device.
 */
DeviceState *sifive_clint_create(hwaddr addr, hwaddr size,
    uint32_t hartid_base, uint32_t num_harts, uint32_t sip_base,
    uint32_t timecmp_base, uint32_t time_base, uint32_t timebase_freq,
    bool provide_rdtime)
{
    int i;

    DeviceState *dev = qdev_new(TYPE_SIFIVE_CLINT);
    qdev_prop_set_uint32(dev, "hartid-base", hartid_base);
    qdev_prop_set_uint32(dev, "num-harts", num_harts);
    qdev_prop_set_uint32(dev, "sip-base", sip_base);
    qdev_prop_set_uint32(dev, "timecmp-base", timecmp_base);
    qdev_prop_set_uint32(dev, "time-base", time_base);
    qdev_prop_set_uint32(dev, "aperture-size", size);
    qdev_prop_set_uint32(dev, "timebase-freq", timebase_freq);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);

    for (i = 0; i < num_harts; i++) {
        CPUState *cpu = qemu_get_cpu(hartid_base + i);
        RISCVCPU *rvcpu = RISCV_CPU(cpu);
        CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
        sifive_clint_callback *cb = g_malloc0(sizeof(sifive_clint_callback));

        if (!env) {
            g_free(cb);
            continue;
        }
        if (provide_rdtime) {
            riscv_cpu_set_rdtime_fn(env, cpu_riscv_read_rtc, timebase_freq);
        }

        cb->s = SIFIVE_CLINT(dev);
        cb->num = i;
        env->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                  &sifive_clint_timer_cb, cb);
        env->timecmp = 0;

        qdev_connect_gpio_out(dev, i,
                              qdev_get_gpio_in(DEVICE(rvcpu), IRQ_M_TIMER));
        qdev_connect_gpio_out(dev, num_harts + i,
                              qdev_get_gpio_in(DEVICE(rvcpu), IRQ_M_SOFT));
    }

    return dev;
}
