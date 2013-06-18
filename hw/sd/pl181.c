/*
 * Arm PrimeCell PL181 MultiMedia Card Interface
 *
 * Copyright (c) 2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

#include "sysemu/blockdev.h"
#include "hw/sysbus.h"
#include "hw/sd.h"

//#define DEBUG_PL181 1

#ifdef DEBUG_PL181
#define DPRINTF(fmt, ...) \
do { printf("pl181: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#endif

#define PL181_FIFO_LEN 16

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    SDState *card;
    uint32_t clock;
    uint32_t power;
    uint32_t cmdarg;
    uint32_t cmd;
    uint32_t datatimer;
    uint32_t datalength;
    uint32_t respcmd;
    uint32_t response[4];
    uint32_t datactrl;
    uint32_t datacnt;
    uint32_t status;
    uint32_t mask[2];
    int32_t fifo_pos;
    int32_t fifo_len;
    /* The linux 2.6.21 driver is buggy, and misbehaves if new data arrives
       while it is reading the FIFO.  We hack around this be defering
       subsequent transfers until after the driver polls the status word.
       http://www.arm.linux.org.uk/developer/patches/viewpatch.php?id=4446/1
     */
    int32_t linux_hack;
    uint32_t fifo[PL181_FIFO_LEN];
    qemu_irq irq[2];
    /* GPIO outputs for 'card is readonly' and 'card inserted' */
    qemu_irq cardstatus[2];
} pl181_state;

static const VMStateDescription vmstate_pl181 = {
    .name = "pl181",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(clock, pl181_state),
        VMSTATE_UINT32(power, pl181_state),
        VMSTATE_UINT32(cmdarg, pl181_state),
        VMSTATE_UINT32(cmd, pl181_state),
        VMSTATE_UINT32(datatimer, pl181_state),
        VMSTATE_UINT32(datalength, pl181_state),
        VMSTATE_UINT32(respcmd, pl181_state),
        VMSTATE_UINT32_ARRAY(response, pl181_state, 4),
        VMSTATE_UINT32(datactrl, pl181_state),
        VMSTATE_UINT32(datacnt, pl181_state),
        VMSTATE_UINT32(status, pl181_state),
        VMSTATE_UINT32_ARRAY(mask, pl181_state, 2),
        VMSTATE_INT32(fifo_pos, pl181_state),
        VMSTATE_INT32(fifo_len, pl181_state),
        VMSTATE_INT32(linux_hack, pl181_state),
        VMSTATE_UINT32_ARRAY(fifo, pl181_state, PL181_FIFO_LEN),
        VMSTATE_END_OF_LIST()
    }
};

#define PL181_CMD_INDEX     0x3f
#define PL181_CMD_RESPONSE  (1 << 6)
#define PL181_CMD_LONGRESP  (1 << 7)
#define PL181_CMD_INTERRUPT (1 << 8)
#define PL181_CMD_PENDING   (1 << 9)
#define PL181_CMD_ENABLE    (1 << 10)

#define PL181_DATA_ENABLE             (1 << 0)
#define PL181_DATA_DIRECTION          (1 << 1)
#define PL181_DATA_MODE               (1 << 2)
#define PL181_DATA_DMAENABLE          (1 << 3)

#define PL181_STATUS_CMDCRCFAIL       (1 << 0)
#define PL181_STATUS_DATACRCFAIL      (1 << 1)
#define PL181_STATUS_CMDTIMEOUT       (1 << 2)
#define PL181_STATUS_DATATIMEOUT      (1 << 3)
#define PL181_STATUS_TXUNDERRUN       (1 << 4)
#define PL181_STATUS_RXOVERRUN        (1 << 5)
#define PL181_STATUS_CMDRESPEND       (1 << 6)
#define PL181_STATUS_CMDSENT          (1 << 7)
#define PL181_STATUS_DATAEND          (1 << 8)
#define PL181_STATUS_DATABLOCKEND     (1 << 10)
#define PL181_STATUS_CMDACTIVE        (1 << 11)
#define PL181_STATUS_TXACTIVE         (1 << 12)
#define PL181_STATUS_RXACTIVE         (1 << 13)
#define PL181_STATUS_TXFIFOHALFEMPTY  (1 << 14)
#define PL181_STATUS_RXFIFOHALFFULL   (1 << 15)
#define PL181_STATUS_TXFIFOFULL       (1 << 16)
#define PL181_STATUS_RXFIFOFULL       (1 << 17)
#define PL181_STATUS_TXFIFOEMPTY      (1 << 18)
#define PL181_STATUS_RXFIFOEMPTY      (1 << 19)
#define PL181_STATUS_TXDATAAVLBL      (1 << 20)
#define PL181_STATUS_RXDATAAVLBL      (1 << 21)

#define PL181_STATUS_TX_FIFO (PL181_STATUS_TXACTIVE \
                             |PL181_STATUS_TXFIFOHALFEMPTY \
                             |PL181_STATUS_TXFIFOFULL \
                             |PL181_STATUS_TXFIFOEMPTY \
                             |PL181_STATUS_TXDATAAVLBL)
#define PL181_STATUS_RX_FIFO (PL181_STATUS_RXACTIVE \
                             |PL181_STATUS_RXFIFOHALFFULL \
                             |PL181_STATUS_RXFIFOFULL \
                             |PL181_STATUS_RXFIFOEMPTY \
                             |PL181_STATUS_RXDATAAVLBL)

static const unsigned char pl181_id[] =
{ 0x81, 0x11, 0x04, 0x00, 0x0d, 0xf0, 0x05, 0xb1 };

static void pl181_update(pl181_state *s)
{
    int i;
    for (i = 0; i < 2; i++) {
        qemu_set_irq(s->irq[i], (s->status & s->mask[i]) != 0);
    }
}

static void pl181_fifo_push(pl181_state *s, uint32_t value)
{
    int n;

    if (s->fifo_len == PL181_FIFO_LEN) {
        fprintf(stderr, "pl181: FIFO overflow\n");
        return;
    }
    n = (s->fifo_pos + s->fifo_len) & (PL181_FIFO_LEN - 1);
    s->fifo_len++;
    s->fifo[n] = value;
    DPRINTF("FIFO push %08x\n", (int)value);
}

static uint32_t pl181_fifo_pop(pl181_state *s)
{
    uint32_t value;

    if (s->fifo_len == 0) {
        fprintf(stderr, "pl181: FIFO underflow\n");
        return 0;
    }
    value = s->fifo[s->fifo_pos];
    s->fifo_len--;
    s->fifo_pos = (s->fifo_pos + 1) & (PL181_FIFO_LEN - 1);
    DPRINTF("FIFO pop %08x\n", (int)value);
    return value;
}

static void pl181_send_command(pl181_state *s)
{
    SDRequest request;
    uint8_t response[16];
    int rlen;

    request.cmd = s->cmd & PL181_CMD_INDEX;
    request.arg = s->cmdarg;
    DPRINTF("Command %d %08x\n", request.cmd, request.arg);
    rlen = sd_do_command(s->card, &request, response);
    if (rlen < 0)
        goto error;
    if (s->cmd & PL181_CMD_RESPONSE) {
#define RWORD(n) ((response[n] << 24) | (response[n + 1] << 16) \
                  | (response[n + 2] << 8) | response[n + 3])
        if (rlen == 0 || (rlen == 4 && (s->cmd & PL181_CMD_LONGRESP)))
            goto error;
        if (rlen != 4 && rlen != 16)
            goto error;
        s->response[0] = RWORD(0);
        if (rlen == 4) {
            s->response[1] = s->response[2] = s->response[3] = 0;
        } else {
            s->response[1] = RWORD(4);
            s->response[2] = RWORD(8);
            s->response[3] = RWORD(12) & ~1;
        }
        DPRINTF("Response received\n");
        s->status |= PL181_STATUS_CMDRESPEND;
#undef RWORD
    } else {
        DPRINTF("Command sent\n");
        s->status |= PL181_STATUS_CMDSENT;
    }
    return;

error:
    DPRINTF("Timeout\n");
    s->status |= PL181_STATUS_CMDTIMEOUT;
}

/* Transfer data between the card and the FIFO.  This is complicated by
   the FIFO holding 32-bit words and the card taking data in single byte
   chunks.  FIFO bytes are transferred in little-endian order.  */

static void pl181_fifo_run(pl181_state *s)
{
    uint32_t bits;
    uint32_t value = 0;
    int n;
    int is_read;

    is_read = (s->datactrl & PL181_DATA_DIRECTION) != 0;
    if (s->datacnt != 0 && (!is_read || sd_data_ready(s->card))
            && !s->linux_hack) {
        if (is_read) {
            n = 0;
            while (s->datacnt && s->fifo_len < PL181_FIFO_LEN) {
                value |= (uint32_t)sd_read_data(s->card) << (n * 8);
                s->datacnt--;
                n++;
                if (n == 4) {
                    pl181_fifo_push(s, value);
                    n = 0;
                    value = 0;
                }
            }
            if (n != 0) {
                pl181_fifo_push(s, value);
            }
        } else { /* write */
            n = 0;
            while (s->datacnt > 0 && (s->fifo_len > 0 || n > 0)) {
                if (n == 0) {
                    value = pl181_fifo_pop(s);
                    n = 4;
                }
                n--;
                s->datacnt--;
                sd_write_data(s->card, value & 0xff);
                value >>= 8;
            }
        }
    }
    s->status &= ~(PL181_STATUS_RX_FIFO | PL181_STATUS_TX_FIFO);
    if (s->datacnt == 0) {
        s->status |= PL181_STATUS_DATAEND;
        /* HACK: */
        s->status |= PL181_STATUS_DATABLOCKEND;
        DPRINTF("Transfer Complete\n");
    }
    if (s->datacnt == 0 && s->fifo_len == 0) {
        s->datactrl &= ~PL181_DATA_ENABLE;
        DPRINTF("Data engine idle\n");
    } else {
        /* Update FIFO bits.  */
        bits = PL181_STATUS_TXACTIVE | PL181_STATUS_RXACTIVE;
        if (s->fifo_len == 0) {
            bits |= PL181_STATUS_TXFIFOEMPTY;
            bits |= PL181_STATUS_RXFIFOEMPTY;
        } else {
            bits |= PL181_STATUS_TXDATAAVLBL;
            bits |= PL181_STATUS_RXDATAAVLBL;
        }
        if (s->fifo_len == 16) {
            bits |= PL181_STATUS_TXFIFOFULL;
            bits |= PL181_STATUS_RXFIFOFULL;
        }
        if (s->fifo_len <= 8) {
            bits |= PL181_STATUS_TXFIFOHALFEMPTY;
        }
        if (s->fifo_len >= 8) {
            bits |= PL181_STATUS_RXFIFOHALFFULL;
        }
        if (s->datactrl & PL181_DATA_DIRECTION) {
            bits &= PL181_STATUS_RX_FIFO;
        } else {
            bits &= PL181_STATUS_TX_FIFO;
        }
        s->status |= bits;
    }
}

static uint64_t pl181_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    pl181_state *s = (pl181_state *)opaque;
    uint32_t tmp;

    if (offset >= 0xfe0 && offset < 0x1000) {
        return pl181_id[(offset - 0xfe0) >> 2];
    }
    switch (offset) {
    case 0x00: /* Power */
        return s->power;
    case 0x04: /* Clock */
        return s->clock;
    case 0x08: /* Argument */
        return s->cmdarg;
    case 0x0c: /* Command */
        return s->cmd;
    case 0x10: /* RespCmd */
        return s->respcmd;
    case 0x14: /* Response0 */
        return s->response[0];
    case 0x18: /* Response1 */
        return s->response[1];
    case 0x1c: /* Response2 */
        return s->response[2];
    case 0x20: /* Response3 */
        return s->response[3];
    case 0x24: /* DataTimer */
        return s->datatimer;
    case 0x28: /* DataLength */
        return s->datalength;
    case 0x2c: /* DataCtrl */
        return s->datactrl;
    case 0x30: /* DataCnt */
        return s->datacnt;
    case 0x34: /* Status */
        tmp = s->status;
        if (s->linux_hack) {
            s->linux_hack = 0;
            pl181_fifo_run(s);
            pl181_update(s);
        }
        return tmp;
    case 0x3c: /* Mask0 */
        return s->mask[0];
    case 0x40: /* Mask1 */
        return s->mask[1];
    case 0x48: /* FifoCnt */
        /* The documentation is somewhat vague about exactly what FifoCnt
           does.  On real hardware it appears to be when decrememnted
           when a word is transferred between the FIFO and the serial
           data engine.  DataCnt is decremented after each byte is
           transferred between the serial engine and the card.
           We don't emulate this level of detail, so both can be the same.  */
        tmp = (s->datacnt + 3) >> 2;
        if (s->linux_hack) {
            s->linux_hack = 0;
            pl181_fifo_run(s);
            pl181_update(s);
        }
        return tmp;
    case 0x80: case 0x84: case 0x88: case 0x8c: /* FifoData */
    case 0x90: case 0x94: case 0x98: case 0x9c:
    case 0xa0: case 0xa4: case 0xa8: case 0xac:
    case 0xb0: case 0xb4: case 0xb8: case 0xbc:
        if (s->fifo_len == 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "pl181: Unexpected FIFO read\n");
            return 0;
        } else {
            uint32_t value;
            value = pl181_fifo_pop(s);
            s->linux_hack = 1;
            pl181_fifo_run(s);
            pl181_update(s);
            return value;
        }
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl181_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void pl181_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    pl181_state *s = (pl181_state *)opaque;

    switch (offset) {
    case 0x00: /* Power */
        s->power = value & 0xff;
        break;
    case 0x04: /* Clock */
        s->clock = value & 0xff;
        break;
    case 0x08: /* Argument */
        s->cmdarg = value;
        break;
    case 0x0c: /* Command */
        s->cmd = value;
        if (s->cmd & PL181_CMD_ENABLE) {
            if (s->cmd & PL181_CMD_INTERRUPT) {
                qemu_log_mask(LOG_UNIMP,
                              "pl181: Interrupt mode not implemented\n");
            } if (s->cmd & PL181_CMD_PENDING) {
                qemu_log_mask(LOG_UNIMP,
                              "pl181: Pending commands not implemented\n");
            } else {
                pl181_send_command(s);
                pl181_fifo_run(s);
            }
            /* The command has completed one way or the other.  */
            s->cmd &= ~PL181_CMD_ENABLE;
        }
        break;
    case 0x24: /* DataTimer */
        s->datatimer = value;
        break;
    case 0x28: /* DataLength */
        s->datalength = value & 0xffff;
        break;
    case 0x2c: /* DataCtrl */
        s->datactrl = value & 0xff;
        if (value & PL181_DATA_ENABLE) {
            s->datacnt = s->datalength;
            pl181_fifo_run(s);
        }
        break;
    case 0x38: /* Clear */
        s->status &= ~(value & 0x7ff);
        break;
    case 0x3c: /* Mask0 */
        s->mask[0] = value;
        break;
    case 0x40: /* Mask1 */
        s->mask[1] = value;
        break;
    case 0x80: case 0x84: case 0x88: case 0x8c: /* FifoData */
    case 0x90: case 0x94: case 0x98: case 0x9c:
    case 0xa0: case 0xa4: case 0xa8: case 0xac:
    case 0xb0: case 0xb4: case 0xb8: case 0xbc:
        if (s->datacnt == 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "pl181: Unexpected FIFO write\n");
        } else {
            pl181_fifo_push(s, value);
            pl181_fifo_run(s);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl181_write: Bad offset %x\n", (int)offset);
    }
    pl181_update(s);
}

static const MemoryRegionOps pl181_ops = {
    .read = pl181_read,
    .write = pl181_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void pl181_reset(DeviceState *d)
{
    pl181_state *s = DO_UPCAST(pl181_state, busdev.qdev, d);

    s->power = 0;
    s->cmdarg = 0;
    s->cmd = 0;
    s->datatimer = 0;
    s->datalength = 0;
    s->respcmd = 0;
    s->response[0] = 0;
    s->response[1] = 0;
    s->response[2] = 0;
    s->response[3] = 0;
    s->datatimer = 0;
    s->datalength = 0;
    s->datactrl = 0;
    s->datacnt = 0;
    s->status = 0;
    s->linux_hack = 0;
    s->mask[0] = 0;
    s->mask[1] = 0;

    /* We can assume our GPIO outputs have been wired up now */
    sd_set_cb(s->card, s->cardstatus[0], s->cardstatus[1]);
}

static int pl181_init(SysBusDevice *dev)
{
    pl181_state *s = FROM_SYSBUS(pl181_state, dev);
    DriveInfo *dinfo;

    memory_region_init_io(&s->iomem, &pl181_ops, s, "pl181", 0x1000);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq[0]);
    sysbus_init_irq(dev, &s->irq[1]);
    qdev_init_gpio_out(&s->busdev.qdev, s->cardstatus, 2);
    dinfo = drive_get_next(IF_SD);
    s->card = sd_init(dinfo ? dinfo->bdrv : NULL, false);
    return 0;
}

static void pl181_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *k = DEVICE_CLASS(klass);

    sdc->init = pl181_init;
    k->vmsd = &vmstate_pl181;
    k->reset = pl181_reset;
    k->no_user = 1;
}

static const TypeInfo pl181_info = {
    .name          = "pl181",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(pl181_state),
    .class_init    = pl181_class_init,
};

static void pl181_register_types(void)
{
    type_register_static(&pl181_info);
}

type_init(pl181_register_types)
