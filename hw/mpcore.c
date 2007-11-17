/*
 * ARM MPCore internal peripheral emulation.
 *
 * Copyright (c) 2006-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licenced under the GPL.
 */

#include "hw.h"
#include "qemu-timer.h"
#include "primecell.h"

#define MPCORE_PRIV_BASE  0x10100000
#define NCPU 4
/* ??? The MPCore TRM says the on-chip controller has 224 external IRQ lines
   (+ 32 internal).  However my test chip only exposes/reports 32.
   More importantly Linux falls over if more than 32 are present!  */
#define GIC_NIRQ 64

static inline int
gic_get_current_cpu(void)
{
  return cpu_single_env->cpu_index;
}

#include "arm_gic.c"

/* MPCore private memory region.  */

typedef struct {
    uint32_t count;
    uint32_t load;
    uint32_t control;
    uint32_t status;
    uint32_t old_status;
    int64_t tick;
    QEMUTimer *timer;
    struct mpcore_priv_state *mpcore;
    int id; /* Encodes both timer/watchdog and CPU.  */
} mpcore_timer_state;

typedef struct mpcore_priv_state {
    gic_state *gic;
    uint32_t scu_control;
    mpcore_timer_state timer[8];
} mpcore_priv_state;

/* Per-CPU Timers.  */

static inline void mpcore_timer_update_irq(mpcore_timer_state *s)
{
    if (s->status & ~s->old_status) {
        gic_set_pending_private(s->mpcore->gic, s->id >> 1, 29 + (s->id & 1));
    }
    s->old_status = s->status;
}

/* Return conversion factor from mpcore timer ticks to qemu timer ticks.  */
static inline uint32_t mpcore_timer_scale(mpcore_timer_state *s)
{
    return (((s->control >> 8) & 0xff) + 1) * 10;
}

static void mpcore_timer_reload(mpcore_timer_state *s, int restart)
{
    if (s->count == 0)
        return;
    if (restart)
        s->tick = qemu_get_clock(vm_clock);
    s->tick += (int64_t)s->count * mpcore_timer_scale(s);
    qemu_mod_timer(s->timer, s->tick);
}

static void mpcore_timer_tick(void *opaque)
{
    mpcore_timer_state *s = (mpcore_timer_state *)opaque;
    s->status = 1;
    if (s->control & 2) {
        s->count = s->load;
        mpcore_timer_reload(s, 0);
    } else {
        s->count = 0;
    }
    mpcore_timer_update_irq(s);
}

static uint32_t mpcore_timer_read(mpcore_timer_state *s, int offset)
{
    int64_t val;
    switch (offset) {
    case 0: /* Load */
        return s->load;
        /* Fall through.  */
    case 4: /* Counter.  */
        if (((s->control & 1) == 0) || (s->count == 0))
            return 0;
        /* Slow and ugly, but hopefully won't happen too often.  */
        val = s->tick - qemu_get_clock(vm_clock);
        val /= mpcore_timer_scale(s);
        if (val < 0)
            val = 0;
        return val;
    case 8: /* Control.  */
        return s->control;
    case 12: /* Interrupt status.  */
        return s->status;
    }
}

static void mpcore_timer_write(mpcore_timer_state *s, int offset,
                               uint32_t value)
{
    int64_t old;
    switch (offset) {
    case 0: /* Load */
        s->load = value;
        /* Fall through.  */
    case 4: /* Counter.  */
        if ((s->control & 1) && s->count) {
            /* Cancel the previous timer.  */
            qemu_del_timer(s->timer);
        }
        s->count = value;
        if (s->control & 1) {
            mpcore_timer_reload(s, 1);
        }
        break;
    case 8: /* Control.  */
        old = s->control;
        s->control = value;
        if (((old & 1) == 0) && (value & 1)) {
            if (s->count == 0 && (s->control & 2))
                s->count = s->load;
            mpcore_timer_reload(s, 1);
        }
        break;
    case 12: /* Interrupt status.  */
        s->status &= ~value;
        mpcore_timer_update_irq(s);
        break;
    }
}

static void mpcore_timer_init(mpcore_priv_state *mpcore,
                              mpcore_timer_state *s, int id)
{
    s->id = id;
    s->mpcore = mpcore;
    s->timer = qemu_new_timer(vm_clock, mpcore_timer_tick, s);
}


/* Per-CPU private memory mapped IO.  */

static uint32_t mpcore_priv_read(void *opaque, target_phys_addr_t offset)
{
    mpcore_priv_state *s = (mpcore_priv_state *)opaque;
    int id;
    offset &= 0xfff;
    if (offset < 0x100) {
        /* SCU */
        switch (offset) {
        case 0x00: /* Control.  */
            return s->scu_control;
        case 0x04: /* Configuration.  */
            return 0xf3;
        case 0x08: /* CPU status.  */
            return 0;
        case 0x0c: /* Invalidate all.  */
            return 0;
        default:
            goto bad_reg;
        }
    } else if (offset < 0x600) {
        /* Interrupt controller.  */
        if (offset < 0x200) {
            id = gic_get_current_cpu();
        } else {
            id = (offset - 0x200) >> 8;
        }
        return gic_cpu_read(s->gic, id, offset & 0xff);
    } else if (offset < 0xb00) {
        /* Timers.  */
        if (offset < 0x700) {
            id = gic_get_current_cpu();
        } else {
            id = (offset - 0x700) >> 8;
        }
        id <<= 1;
        if (offset & 0x20)
          id++;
        return mpcore_timer_read(&s->timer[id], offset & 0xf);
    }
bad_reg:
    cpu_abort(cpu_single_env, "mpcore_priv_read: Bad offset %x\n",
              (int)offset);
    return 0;
}

static void mpcore_priv_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    mpcore_priv_state *s = (mpcore_priv_state *)opaque;
    int id;
    offset &= 0xfff;
    if (offset < 0x100) {
        /* SCU */
        switch (offset) {
        case 0: /* Control register.  */
            s->scu_control = value & 1;
            break;
        case 0x0c: /* Invalidate all.  */
            /* This is a no-op as cache is not emulated.  */
            break;
        default:
            goto bad_reg;
        }
    } else if (offset < 0x600) {
        /* Interrupt controller.  */
        if (offset < 0x200) {
            id = gic_get_current_cpu();
        } else {
            id = (offset - 0x200) >> 8;
        }
        gic_cpu_write(s->gic, id, offset & 0xff, value);
    } else if (offset < 0xb00) {
        /* Timers.  */
        if (offset < 0x700) {
            id = gic_get_current_cpu();
        } else {
            id = (offset - 0x700) >> 8;
        }
        id <<= 1;
        if (offset & 0x20)
          id++;
        mpcore_timer_write(&s->timer[id], offset & 0xf, value);
        return;
    }
    return;
bad_reg:
    cpu_abort(cpu_single_env, "mpcore_priv_read: Bad offset %x\n",
              (int)offset);
}

static CPUReadMemoryFunc *mpcore_priv_readfn[] = {
   mpcore_priv_read,
   mpcore_priv_read,
   mpcore_priv_read
};

static CPUWriteMemoryFunc *mpcore_priv_writefn[] = {
   mpcore_priv_write,
   mpcore_priv_write,
   mpcore_priv_write
};


static qemu_irq *mpcore_priv_init(uint32_t base, qemu_irq *pic_irq)
{
    mpcore_priv_state *s;
    int iomemtype;
    int i;

    s = (mpcore_priv_state *)qemu_mallocz(sizeof(mpcore_priv_state));
    if (!s)
        return NULL;
    s->gic = gic_init(base, pic_irq);
    if (!s->gic)
        return NULL;
    iomemtype = cpu_register_io_memory(0, mpcore_priv_readfn,
                                       mpcore_priv_writefn, s);
    cpu_register_physical_memory(base, 0x00001000, iomemtype);
    for (i = 0; i < 8; i++) {
        mpcore_timer_init(s, &s->timer[i], i);
    }
    return s->gic->in;
}

/* Dummy PIC to route IRQ lines.  The baseboard has 4 independent IRQ
   controllers.  The output of these, plus some of the raw input lines
   are fed into a single SMP-aware interrupt controller on the CPU.  */
typedef struct {
    qemu_irq *cpuic;
    qemu_irq *rvic[4];
} mpcore_rirq_state;

/* Map baseboard IRQs onto CPU IRQ lines.  */
static const int mpcore_irq_map[32] = {
    -1, -1, -1, -1,  1,  2, -1, -1,
    -1, -1,  6, -1,  4,  5, -1, -1,
    -1, 14, 15,  0,  7,  8, -1, -1,
    -1, -1, -1, -1,  9,  3, -1, -1,
};

static void mpcore_rirq_set_irq(void *opaque, int irq, int level)
{
    mpcore_rirq_state *s = (mpcore_rirq_state *)opaque;
    int i;

    for (i = 0; i < 4; i++) {
        qemu_set_irq(s->rvic[i][irq], level);
    }
    if (irq < 32) {
        irq = mpcore_irq_map[irq];
        if (irq >= 0) {
            qemu_set_irq(s->cpuic[irq], level);
        }
    }
}

qemu_irq *mpcore_irq_init(qemu_irq *cpu_irq)
{
    mpcore_rirq_state *s;
    int n;

    /* ??? IRQ routing is hardcoded to "normal" mode.  */
    s = qemu_mallocz(sizeof(mpcore_rirq_state));
    s->cpuic = mpcore_priv_init(MPCORE_PRIV_BASE, cpu_irq);
    for (n = 0; n < 4; n++) {
        s->rvic[n] = realview_gic_init(0x10040000 + n * 0x10000,
                                       s->cpuic[10 + n]);
    }
    return qemu_allocate_irqs(mpcore_rirq_set_irq, s, 64);
}
