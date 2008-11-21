#ifndef QEMU_SH_H
#define QEMU_SH_H
/* Definitions for SH board emulation.  */

#include "sh_intc.h"

/* sh7750.c */
struct SH7750State;

struct SH7750State *sh7750_init(CPUState * cpu);

typedef struct {
    /* The callback will be triggered if any of the designated lines change */
    uint16_t portamask_trigger;
    uint16_t portbmask_trigger;
    /* Return 0 if no action was taken */
    int (*port_change_cb) (uint16_t porta, uint16_t portb,
			   uint16_t * periph_pdtra,
			   uint16_t * periph_portdira,
			   uint16_t * periph_pdtrb,
			   uint16_t * periph_portdirb);
} sh7750_io_device;

int sh7750_register_io_device(struct SH7750State *s,
			      sh7750_io_device * device);
/* sh_timer.c */
#define TMU012_FEAT_TOCR   (1 << 0)
#define TMU012_FEAT_3CHAN  (1 << 1)
#define TMU012_FEAT_EXTCLK (1 << 2)
void tmu012_init(target_phys_addr_t base, int feat, uint32_t freq,
		 qemu_irq ch0_irq, qemu_irq ch1_irq,
		 qemu_irq ch2_irq0, qemu_irq ch2_irq1);


/* sh_serial.c */
#define SH_SERIAL_FEAT_SCIF (1 << 0)
void sh_serial_init (target_phys_addr_t base, int feat,
		     uint32_t freq, CharDriverState *chr,
		     struct intc_source *eri_source,
		     struct intc_source *rxi_source,
		     struct intc_source *txi_source,
		     struct intc_source *tei_source,
		     struct intc_source *bri_source);

/* tc58128.c */
int tc58128_init(struct SH7750State *s, const char *zone1, const char *zone2);

#endif
