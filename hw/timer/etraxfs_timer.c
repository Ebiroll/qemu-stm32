/*
 * QEMU ETRAX Timers
 *
 * Copyright (c) 2007 Edgar E. Iglesias, Axis Communications AB.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "qemu/timer.h"
#include "hw/ptimer.h"

#define D(x)

#define RW_TMR0_DIV   0x00
#define R_TMR0_DATA   0x04
#define RW_TMR0_CTRL  0x08
#define RW_TMR1_DIV   0x10
#define R_TMR1_DATA   0x14
#define RW_TMR1_CTRL  0x18
#define R_TIME        0x38
#define RW_WD_CTRL    0x40
#define R_WD_STAT     0x44
#define RW_INTR_MASK  0x48
#define RW_ACK_INTR   0x4c
#define R_INTR        0x50
#define R_MASKED_INTR 0x54

#define TYPE_ETRAX_FS_TIMER "etraxfs,timer"
#define ETRAX_TIMER(obj) \
    OBJECT_CHECK(ETRAXTimerState, (obj), TYPE_ETRAX_FS_TIMER)

typedef struct ETRAXTimerState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    qemu_irq nmi;

    QEMUBH *bh_t0;
    QEMUBH *bh_t1;
    QEMUBH *bh_wd;
    ptimer_state *ptimer_t0;
    ptimer_state *ptimer_t1;
    ptimer_state *ptimer_wd;

    int wd_hits;

    /* Control registers.  */
    uint32_t rw_tmr0_div;
    uint32_t r_tmr0_data;
    uint32_t rw_tmr0_ctrl;

    uint32_t rw_tmr1_div;
    uint32_t r_tmr1_data;
    uint32_t rw_tmr1_ctrl;

    uint32_t rw_wd_ctrl;

    uint32_t rw_intr_mask;
    uint32_t rw_ack_intr;
    uint32_t r_intr;
    uint32_t r_masked_intr;
} ETRAXTimerState;

static uint64_t
timer_read(void *opaque, hwaddr addr, unsigned int size)
{
    ETRAXTimerState *t = opaque;
    uint32_t r = 0;

    switch (addr) {
    case R_TMR0_DATA:
        r = ptimer_get_count(t->ptimer_t0);
        break;
    case R_TMR1_DATA:
        r = ptimer_get_count(t->ptimer_t1);
        break;
    case R_TIME:
        r = qemu_get_clock_ns(vm_clock) / 10;
        break;
    case RW_INTR_MASK:
        r = t->rw_intr_mask;
        break;
    case R_MASKED_INTR:
        r = t->r_intr & t->rw_intr_mask;
        break;
    default:
        D(printf ("%s %x\n", __func__, addr));
        break;
    }
    return r;
}

static void update_ctrl(ETRAXTimerState *t, int tnum)
{
    unsigned int op;
    unsigned int freq;
    unsigned int freq_hz;
    unsigned int div;
    uint32_t ctrl;

    ptimer_state *timer;

    if (tnum == 0) {
        ctrl = t->rw_tmr0_ctrl;
        div = t->rw_tmr0_div;
        timer = t->ptimer_t0;
    } else {
        ctrl = t->rw_tmr1_ctrl;
        div = t->rw_tmr1_div;
        timer = t->ptimer_t1;
    }


    op = ctrl & 3;
    freq = ctrl >> 2;
    freq_hz = 32000000;

    switch (freq)
    {
    case 0:
    case 1:
        D(printf ("extern or disabled timer clock?\n"));
        break;
    case 4: freq_hz =  29493000; break;
    case 5: freq_hz =  32000000; break;
    case 6: freq_hz =  32768000; break;
    case 7: freq_hz = 100000000; break;
    default:
        abort();
        break;
    }

    D(printf ("freq_hz=%d div=%d\n", freq_hz, div));
    ptimer_set_freq(timer, freq_hz);
    ptimer_set_limit(timer, div, 0);

    switch (op)
    {
        case 0:
            /* Load.  */
            ptimer_set_limit(timer, div, 1);
            break;
        case 1:
            /* Hold.  */
            ptimer_stop(timer);
            break;
        case 2:
            /* Run.  */
            ptimer_run(timer, 0);
            break;
        default:
            abort();
            break;
    }
}

static void timer_update_irq(ETRAXTimerState *t)
{
    t->r_intr &= ~(t->rw_ack_intr);
    t->r_masked_intr = t->r_intr & t->rw_intr_mask;

    D(printf("%s: masked_intr=%x\n", __func__, t->r_masked_intr));
    qemu_set_irq(t->irq, !!t->r_masked_intr);
}

static void timer0_hit(void *opaque)
{
    ETRAXTimerState *t = opaque;
    t->r_intr |= 1;
    timer_update_irq(t);
}

static void timer1_hit(void *opaque)
{
    ETRAXTimerState *t = opaque;
    t->r_intr |= 2;
    timer_update_irq(t);
}

static void watchdog_hit(void *opaque)
{
    ETRAXTimerState *t = opaque;
    if (t->wd_hits == 0) {
        /* real hw gives a single tick before reseting but we are
           a bit friendlier to compensate for our slower execution.  */
        ptimer_set_count(t->ptimer_wd, 10);
        ptimer_run(t->ptimer_wd, 1);
        qemu_irq_raise(t->nmi);
    }
    else
        qemu_system_reset_request();

    t->wd_hits++;
}

static inline void timer_watchdog_update(ETRAXTimerState *t, uint32_t value)
{
    unsigned int wd_en = t->rw_wd_ctrl & (1 << 8);
    unsigned int wd_key = t->rw_wd_ctrl >> 9;
    unsigned int wd_cnt = t->rw_wd_ctrl & 511;
    unsigned int new_key = value >> 9 & ((1 << 7) - 1);
    unsigned int new_cmd = (value >> 8) & 1;

    /* If the watchdog is enabled, they written key must match the
       complement of the previous.  */
    wd_key = ~wd_key & ((1 << 7) - 1);

    if (wd_en && wd_key != new_key)
        return;

    D(printf("en=%d new_key=%x oldkey=%x cmd=%d cnt=%d\n", 
         wd_en, new_key, wd_key, new_cmd, wd_cnt));

    if (t->wd_hits)
        qemu_irq_lower(t->nmi);

    t->wd_hits = 0;

    ptimer_set_freq(t->ptimer_wd, 760);
    if (wd_cnt == 0)
        wd_cnt = 256;
    ptimer_set_count(t->ptimer_wd, wd_cnt);
    if (new_cmd)
        ptimer_run(t->ptimer_wd, 1);
    else
        ptimer_stop(t->ptimer_wd);

    t->rw_wd_ctrl = value;
}

static void
timer_write(void *opaque, hwaddr addr,
            uint64_t val64, unsigned int size)
{
    ETRAXTimerState *t = opaque;
    uint32_t value = val64;

    switch (addr)
    {
        case RW_TMR0_DIV:
            t->rw_tmr0_div = value;
            break;
        case RW_TMR0_CTRL:
            D(printf ("RW_TMR0_CTRL=%x\n", value));
            t->rw_tmr0_ctrl = value;
            update_ctrl(t, 0);
            break;
        case RW_TMR1_DIV:
            t->rw_tmr1_div = value;
            break;
        case RW_TMR1_CTRL:
            D(printf ("RW_TMR1_CTRL=%x\n", value));
            t->rw_tmr1_ctrl = value;
            update_ctrl(t, 1);
            break;
        case RW_INTR_MASK:
            D(printf ("RW_INTR_MASK=%x\n", value));
            t->rw_intr_mask = value;
            timer_update_irq(t);
            break;
        case RW_WD_CTRL:
            timer_watchdog_update(t, value);
            break;
        case RW_ACK_INTR:
            t->rw_ack_intr = value;
            timer_update_irq(t);
            t->rw_ack_intr = 0;
            break;
        default:
            printf ("%s " TARGET_FMT_plx " %x\n",
                __func__, addr, value);
            break;
    }
}

static const MemoryRegionOps timer_ops = {
    .read = timer_read,
    .write = timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void etraxfs_timer_reset(void *opaque)
{
    ETRAXTimerState *t = opaque;

    ptimer_stop(t->ptimer_t0);
    ptimer_stop(t->ptimer_t1);
    ptimer_stop(t->ptimer_wd);
    t->rw_wd_ctrl = 0;
    t->r_intr = 0;
    t->rw_intr_mask = 0;
    qemu_irq_lower(t->irq);
}

static int etraxfs_timer_init(SysBusDevice *dev)
{
    ETRAXTimerState *t = ETRAX_TIMER(dev);

    t->bh_t0 = qemu_bh_new(timer0_hit, t);
    t->bh_t1 = qemu_bh_new(timer1_hit, t);
    t->bh_wd = qemu_bh_new(watchdog_hit, t);
    t->ptimer_t0 = ptimer_init(t->bh_t0);
    t->ptimer_t1 = ptimer_init(t->bh_t1);
    t->ptimer_wd = ptimer_init(t->bh_wd);

    sysbus_init_irq(dev, &t->irq);
    sysbus_init_irq(dev, &t->nmi);

    memory_region_init_io(&t->mmio, OBJECT(t), &timer_ops, t,
                          "etraxfs-timer", 0x5c);
    sysbus_init_mmio(dev, &t->mmio);
    qemu_register_reset(etraxfs_timer_reset, t);
    return 0;
}

static void etraxfs_timer_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

    sdc->init = etraxfs_timer_init;
}

static const TypeInfo etraxfs_timer_info = {
    .name          = TYPE_ETRAX_FS_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ETRAXTimerState),
    .class_init    = etraxfs_timer_class_init,
};

static void etraxfs_timer_register_types(void)
{
    type_register_static(&etraxfs_timer_info);
}

type_init(etraxfs_timer_register_types)
