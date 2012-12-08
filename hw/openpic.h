#if !defined(__OPENPIC_H__)
#define __OPENPIC_H__

/* OpenPIC have 5 outputs per CPU connected and one IRQ out single output */
enum {
    OPENPIC_OUTPUT_INT = 0, /* IRQ                       */
    OPENPIC_OUTPUT_CINT,    /* critical IRQ              */
    OPENPIC_OUTPUT_MCK,     /* Machine check event       */
    OPENPIC_OUTPUT_DEBUG,   /* Inconditional debug event */
    OPENPIC_OUTPUT_RESET,   /* Core reset event          */
    OPENPIC_OUTPUT_NB,
};

/* OpenPIC capability flags */
#define OPENPIC_FLAG_IDE_CRIT    (1 << 0)

qemu_irq *openpic_init (MemoryRegion **pmem, int nb_cpus,
                        qemu_irq **irqs);
qemu_irq *mpic_init (MemoryRegion *address_space, hwaddr base,
                     int nb_cpus, qemu_irq **irqs);
#endif /* __OPENPIC_H__ */
