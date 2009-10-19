#define NE2000_PMEM_SIZE    (32*1024)
#define NE2000_PMEM_START   (16*1024)
#define NE2000_PMEM_END     (NE2000_PMEM_SIZE+NE2000_PMEM_START)
#define NE2000_MEM_SIZE     NE2000_PMEM_END

typedef struct NE2000State {
    uint8_t cmd;
    uint32_t start;
    uint32_t stop;
    uint8_t boundary;
    uint8_t tsr;
    uint8_t tpsr;
    uint16_t tcnt;
    uint16_t rcnt;
    uint32_t rsar;
    uint8_t rsr;
    uint8_t rxcr;
    uint8_t isr;
    uint8_t dcfg;
    uint8_t imr;
    uint8_t phys[6]; /* mac address */
    uint8_t curpag;
    uint8_t mult[8]; /* multicast mask array */
    qemu_irq irq;
    VLANClientState *vc;
    NICConf c;
    uint8_t mem[NE2000_MEM_SIZE];
} NE2000State;

void ne2000_ioport_write(void *opaque, uint32_t addr, uint32_t val);
uint32_t ne2000_ioport_read(void *opaque, uint32_t addr);
void ne2000_asic_ioport_write(void *opaque, uint32_t addr, uint32_t val);
uint32_t ne2000_asic_ioport_read(void *opaque, uint32_t addr);
void ne2000_reset_ioport_write(void *opaque, uint32_t addr, uint32_t val);
uint32_t ne2000_reset_ioport_read(void *opaque, uint32_t addr);
extern const VMStateDescription vmstate_ne2000;
void ne2000_reset(NE2000State *s);
int ne2000_can_receive(VLANClientState *vc);
ssize_t ne2000_receive(VLANClientState *vc, const uint8_t *buf, size_t size_);
