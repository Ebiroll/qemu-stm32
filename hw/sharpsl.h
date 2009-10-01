/*
 * Common declarations for the Zaurii.
 *
 * This file is licensed under the GNU GPL.
 */
#ifndef QEMU_SHARPSL_H
#define QEMU_SHARPSL_H

#define zaurus_printf(format, ...)	\
    fprintf(stderr, "%s: " format, __FUNCTION__, ##__VA_ARGS__)

/* zaurus.c */
typedef struct ScoopInfo ScoopInfo;
ScoopInfo *scoop_init(PXA2xxState *cpu,
                int instance, a_target_phys_addr target_base);
void scoop_gpio_set(void *opaque, int line, int level);
qemu_irq *scoop_gpio_in_get(ScoopInfo *s);
void scoop_gpio_out_set(ScoopInfo *s, int line,
                qemu_irq handler);

#define SL_PXA_PARAM_BASE	0xa0000a00
void sl_bootparam_write(a_target_phys_addr ptr);

#endif
