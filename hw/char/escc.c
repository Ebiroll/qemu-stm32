/*
 * QEMU ESCC (Z8030/Z8530/Z85C30/SCC/ESCC) serial port emulation
 *
 * Copyright (c) 2003-2005 Fabrice Bellard
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

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "hw/char/escc.h"
#include "ui/console.h"
#include "trace.h"

/*
 * Chipset docs:
 * "Z80C30/Z85C30/Z80230/Z85230/Z85233 SCC/ESCC User Manual",
 * http://www.zilog.com/docs/serial/scc_escc_um.pdf
 *
 * On Sparc32 this is the serial port, mouse and keyboard part of chip STP2001
 * (Slave I/O), also produced as NCR89C105. See
 * http://www.ibiblio.org/pub/historic-linux/early-ports/Sparc/NCR/NCR89C105.txt
 *
 * The serial ports implement full AMD AM8530 or Zilog Z8530 chips,
 * mouse and keyboard ports don't implement all functions and they are
 * only asynchronous. There is no DMA.
 *
 * Z85C30 is also used on PowerMacs and m68k Macs.
 *
 * There are some small differences between Sparc version (sunzilog)
 * and PowerMac (pmac):
 *  Offset between control and data registers
 *  There is some kind of lockup bug, but we can ignore it
 *  CTS is inverted
 *  DMA on pmac using DBDMA chip
 *  pmac can do IRDA and faster rates, sunzilog can only do 38400
 *  pmac baud rate generator clock is 3.6864 MHz, sunzilog 4.9152 MHz
 *
 * Linux driver for m68k Macs is the same as for PowerMac (pmac_zilog),
 * but registers are grouped by type and not by channel:
 * channel is selected by bit 0 of the address (instead of bit 1)
 * and register is selected by bit 1 of the address (instead of bit 0).
 */

/*
 * Modifications:
 *  2006-Aug-10  Igor Kovalenko :   Renamed KBDQueue to SERIOQueue, implemented
 *                                  serial mouse queue.
 *                                  Implemented serial mouse protocol.
 *
 *  2010-May-23  Artyom Tarasenko:  Reworked IUS logic
 */

#define CHN_C(s) ((s)->chn == escc_chn_b ? 'b' : 'a')

#define SERIAL_CTRL 0
#define SERIAL_DATA 1

#define W_CMD     0
#define CMD_PTR_MASK   0x07
#define CMD_CMD_MASK   0x38
#define CMD_HI         0x08
#define CMD_CLR_TXINT  0x28
#define CMD_CLR_IUS    0x38
#define W_INTR    1
#define INTR_INTALL    0x01
#define INTR_TXINT     0x02
#define INTR_RXMODEMSK 0x18
#define INTR_RXINT1ST  0x08
#define INTR_RXINTALL  0x10
#define W_IVEC    2
#define W_RXCTRL  3
#define RXCTRL_RXEN    0x01
#define W_TXCTRL1 4
#define TXCTRL1_PAREN  0x01
#define TXCTRL1_PAREV  0x02
#define TXCTRL1_1STOP  0x04
#define TXCTRL1_1HSTOP 0x08
#define TXCTRL1_2STOP  0x0c
#define TXCTRL1_STPMSK 0x0c
#define TXCTRL1_CLK1X  0x00
#define TXCTRL1_CLK16X 0x40
#define TXCTRL1_CLK32X 0x80
#define TXCTRL1_CLK64X 0xc0
#define TXCTRL1_CLKMSK 0xc0
#define W_TXCTRL2 5
#define TXCTRL2_TXEN   0x08
#define TXCTRL2_BITMSK 0x60
#define TXCTRL2_5BITS  0x00
#define TXCTRL2_7BITS  0x20
#define TXCTRL2_6BITS  0x40
#define TXCTRL2_8BITS  0x60
#define W_SYNC1   6
#define W_SYNC2   7
#define W_TXBUF   8
#define W_MINTR   9
#define MINTR_STATUSHI 0x10
#define MINTR_RST_MASK 0xc0
#define MINTR_RST_B    0x40
#define MINTR_RST_A    0x80
#define MINTR_RST_ALL  0xc0
#define W_MISC1  10
#define W_CLOCK  11
#define CLOCK_TRXC     0x08
#define W_BRGLO  12
#define W_BRGHI  13
#define W_MISC2  14
#define MISC2_PLLDIS   0x30
#define W_EXTINT 15
#define EXTINT_DCD     0x08
#define EXTINT_SYNCINT 0x10
#define EXTINT_CTSINT  0x20
#define EXTINT_TXUNDRN 0x40
#define EXTINT_BRKINT  0x80

#define R_STATUS  0
#define STATUS_RXAV    0x01
#define STATUS_ZERO    0x02
#define STATUS_TXEMPTY 0x04
#define STATUS_DCD     0x08
#define STATUS_SYNC    0x10
#define STATUS_CTS     0x20
#define STATUS_TXUNDRN 0x40
#define STATUS_BRK     0x80
#define R_SPEC    1
#define SPEC_ALLSENT   0x01
#define SPEC_BITS8     0x06
#define R_IVEC    2
#define IVEC_TXINTB    0x00
#define IVEC_LONOINT   0x06
#define IVEC_LORXINTA  0x0c
#define IVEC_LORXINTB  0x04
#define IVEC_LOTXINTA  0x08
#define IVEC_HINOINT   0x60
#define IVEC_HIRXINTA  0x30
#define IVEC_HIRXINTB  0x20
#define IVEC_HITXINTA  0x10
#define R_INTR    3
#define INTR_EXTINTB   0x01
#define INTR_TXINTB    0x02
#define INTR_RXINTB    0x04
#define INTR_EXTINTA   0x08
#define INTR_TXINTA    0x10
#define INTR_RXINTA    0x20
#define R_IPEN    4
#define R_TXCTRL1 5
#define R_TXCTRL2 6
#define R_BC      7
#define R_RXBUF   8
#define R_RXCTRL  9
#define R_MISC   10
#define R_MISC1  11
#define R_BRGLO  12
#define R_BRGHI  13
#define R_MISC1I 14
#define R_EXTINT 15

static void handle_kbd_command(ESCCChannelState *s, int val);
static int serial_can_receive(void *opaque);
static void serial_receive_byte(ESCCChannelState *s, int ch);

static int reg_shift(ESCCState *s)
{
    return s->bit_swap ? s->it_shift + 1 : s->it_shift;
}

static int chn_shift(ESCCState *s)
{
    return s->bit_swap ? s->it_shift : s->it_shift + 1;
}

static void clear_queue(void *opaque)
{
    ESCCChannelState *s = opaque;
    ESCCSERIOQueue *q = &s->queue;
    q->rptr = q->wptr = q->count = 0;
}

static void put_queue(void *opaque, int b)
{
    ESCCChannelState *s = opaque;
    ESCCSERIOQueue *q = &s->queue;

    trace_escc_put_queue(CHN_C(s), b);
    if (q->count >= ESCC_SERIO_QUEUE_SIZE) {
        return;
    }
    q->data[q->wptr] = b;
    if (++q->wptr == ESCC_SERIO_QUEUE_SIZE) {
        q->wptr = 0;
    }
    q->count++;
    serial_receive_byte(s, 0);
}

static uint32_t get_queue(void *opaque)
{
    ESCCChannelState *s = opaque;
    ESCCSERIOQueue *q = &s->queue;
    int val;

    if (q->count == 0) {
        return 0;
    } else {
        val = q->data[q->rptr];
        if (++q->rptr == ESCC_SERIO_QUEUE_SIZE) {
            q->rptr = 0;
        }
        q->count--;
    }
    trace_escc_get_queue(CHN_C(s), val);
    if (q->count > 0)
        serial_receive_byte(s, 0);
    return val;
}

static int escc_update_irq_chn(ESCCChannelState *s)
{
    if ((((s->wregs[W_INTR] & INTR_TXINT) && (s->txint == 1)) ||
         // tx ints enabled, pending
         ((((s->wregs[W_INTR] & INTR_RXMODEMSK) == INTR_RXINT1ST) ||
           ((s->wregs[W_INTR] & INTR_RXMODEMSK) == INTR_RXINTALL)) &&
          s->rxint == 1) || // rx ints enabled, pending
         ((s->wregs[W_EXTINT] & EXTINT_BRKINT) &&
          (s->rregs[R_STATUS] & STATUS_BRK)))) { // break int e&p
        return 1;
    }
    return 0;
}

static void escc_update_irq(ESCCChannelState *s)
{
    int irq;

    irq = escc_update_irq_chn(s);
    irq |= escc_update_irq_chn(s->otherchn);

    trace_escc_update_irq(irq);
    qemu_set_irq(s->irq, irq);
}

static void escc_reset_chn(ESCCChannelState *s)
{
    int i;

    s->reg = 0;
    for (i = 0; i < ESCC_SERIAL_REGS; i++) {
        s->rregs[i] = 0;
        s->wregs[i] = 0;
    }
    s->wregs[W_TXCTRL1] = TXCTRL1_1STOP; // 1X divisor, 1 stop bit, no parity
    s->wregs[W_MINTR] = MINTR_RST_ALL;
    s->wregs[W_CLOCK] = CLOCK_TRXC; // Synch mode tx clock = TRxC
    s->wregs[W_MISC2] = MISC2_PLLDIS; // PLL disabled
    s->wregs[W_EXTINT] = EXTINT_DCD | EXTINT_SYNCINT | EXTINT_CTSINT |
        EXTINT_TXUNDRN | EXTINT_BRKINT; // Enable most interrupts
    if (s->disabled)
        s->rregs[R_STATUS] = STATUS_TXEMPTY | STATUS_DCD | STATUS_SYNC |
            STATUS_CTS | STATUS_TXUNDRN;
    else
        s->rregs[R_STATUS] = STATUS_TXEMPTY | STATUS_TXUNDRN;
    s->rregs[R_SPEC] = SPEC_BITS8 | SPEC_ALLSENT;

    s->rx = s->tx = 0;
    s->rxint = s->txint = 0;
    s->rxint_under_svc = s->txint_under_svc = 0;
    s->e0_mode = s->led_mode = s->caps_lock_mode = s->num_lock_mode = 0;
    clear_queue(s);
}

static void escc_reset(DeviceState *d)
{
    ESCCState *s = ESCC(d);

    escc_reset_chn(&s->chn[0]);
    escc_reset_chn(&s->chn[1]);
}

static inline void set_rxint(ESCCChannelState *s)
{
    s->rxint = 1;
    /* XXX: missing daisy chainnig: escc_chn_b rx should have a lower priority
       than chn_a rx/tx/special_condition service*/
    s->rxint_under_svc = 1;
    if (s->chn == escc_chn_a) {
        s->rregs[R_INTR] |= INTR_RXINTA;
        if (s->wregs[W_MINTR] & MINTR_STATUSHI)
            s->otherchn->rregs[R_IVEC] = IVEC_HIRXINTA;
        else
            s->otherchn->rregs[R_IVEC] = IVEC_LORXINTA;
    } else {
        s->otherchn->rregs[R_INTR] |= INTR_RXINTB;
        if (s->wregs[W_MINTR] & MINTR_STATUSHI)
            s->rregs[R_IVEC] = IVEC_HIRXINTB;
        else
            s->rregs[R_IVEC] = IVEC_LORXINTB;
    }
    escc_update_irq(s);
}

static inline void set_txint(ESCCChannelState *s)
{
    s->txint = 1;
    if (!s->rxint_under_svc) {
        s->txint_under_svc = 1;
        if (s->chn == escc_chn_a) {
            if (s->wregs[W_INTR] & INTR_TXINT) {
                s->rregs[R_INTR] |= INTR_TXINTA;
            }
            if (s->wregs[W_MINTR] & MINTR_STATUSHI)
                s->otherchn->rregs[R_IVEC] = IVEC_HITXINTA;
            else
                s->otherchn->rregs[R_IVEC] = IVEC_LOTXINTA;
        } else {
            s->rregs[R_IVEC] = IVEC_TXINTB;
            if (s->wregs[W_INTR] & INTR_TXINT) {
                s->otherchn->rregs[R_INTR] |= INTR_TXINTB;
            }
        }
    escc_update_irq(s);
    }
}

static inline void clr_rxint(ESCCChannelState *s)
{
    s->rxint = 0;
    s->rxint_under_svc = 0;
    if (s->chn == escc_chn_a) {
        if (s->wregs[W_MINTR] & MINTR_STATUSHI)
            s->otherchn->rregs[R_IVEC] = IVEC_HINOINT;
        else
            s->otherchn->rregs[R_IVEC] = IVEC_LONOINT;
        s->rregs[R_INTR] &= ~INTR_RXINTA;
    } else {
        if (s->wregs[W_MINTR] & MINTR_STATUSHI)
            s->rregs[R_IVEC] = IVEC_HINOINT;
        else
            s->rregs[R_IVEC] = IVEC_LONOINT;
        s->otherchn->rregs[R_INTR] &= ~INTR_RXINTB;
    }
    if (s->txint)
        set_txint(s);
    escc_update_irq(s);
}

static inline void clr_txint(ESCCChannelState *s)
{
    s->txint = 0;
    s->txint_under_svc = 0;
    if (s->chn == escc_chn_a) {
        if (s->wregs[W_MINTR] & MINTR_STATUSHI)
            s->otherchn->rregs[R_IVEC] = IVEC_HINOINT;
        else
            s->otherchn->rregs[R_IVEC] = IVEC_LONOINT;
        s->rregs[R_INTR] &= ~INTR_TXINTA;
    } else {
        s->otherchn->rregs[R_INTR] &= ~INTR_TXINTB;
        if (s->wregs[W_MINTR] & MINTR_STATUSHI)
            s->rregs[R_IVEC] = IVEC_HINOINT;
        else
            s->rregs[R_IVEC] = IVEC_LONOINT;
        s->otherchn->rregs[R_INTR] &= ~INTR_TXINTB;
    }
    if (s->rxint)
        set_rxint(s);
    escc_update_irq(s);
}

static void escc_update_parameters(ESCCChannelState *s)
{
    int speed, parity, data_bits, stop_bits;
    QEMUSerialSetParams ssp;

    if (!qemu_chr_fe_backend_connected(&s->chr) || s->type != escc_serial)
        return;

    if (s->wregs[W_TXCTRL1] & TXCTRL1_PAREN) {
        if (s->wregs[W_TXCTRL1] & TXCTRL1_PAREV)
            parity = 'E';
        else
            parity = 'O';
    } else {
        parity = 'N';
    }
    if ((s->wregs[W_TXCTRL1] & TXCTRL1_STPMSK) == TXCTRL1_2STOP)
        stop_bits = 2;
    else
        stop_bits = 1;
    switch (s->wregs[W_TXCTRL2] & TXCTRL2_BITMSK) {
    case TXCTRL2_5BITS:
        data_bits = 5;
        break;
    case TXCTRL2_7BITS:
        data_bits = 7;
        break;
    case TXCTRL2_6BITS:
        data_bits = 6;
        break;
    default:
    case TXCTRL2_8BITS:
        data_bits = 8;
        break;
    }
    speed = s->clock / ((s->wregs[W_BRGLO] | (s->wregs[W_BRGHI] << 8)) + 2);
    switch (s->wregs[W_TXCTRL1] & TXCTRL1_CLKMSK) {
    case TXCTRL1_CLK1X:
        break;
    case TXCTRL1_CLK16X:
        speed /= 16;
        break;
    case TXCTRL1_CLK32X:
        speed /= 32;
        break;
    default:
    case TXCTRL1_CLK64X:
        speed /= 64;
        break;
    }
    ssp.speed = speed;
    ssp.parity = parity;
    ssp.data_bits = data_bits;
    ssp.stop_bits = stop_bits;
    trace_escc_update_parameters(CHN_C(s), speed, parity, data_bits, stop_bits);
    qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_PARAMS, &ssp);
}

static void escc_mem_write(void *opaque, hwaddr addr,
                           uint64_t val, unsigned size)
{
    ESCCState *serial = opaque;
    ESCCChannelState *s;
    uint32_t saddr;
    int newreg, channel;

    val &= 0xff;
    saddr = (addr >> reg_shift(serial)) & 1;
    channel = (addr >> chn_shift(serial)) & 1;
    s = &serial->chn[channel];
    switch (saddr) {
    case SERIAL_CTRL:
        trace_escc_mem_writeb_ctrl(CHN_C(s), s->reg, val & 0xff);
        newreg = 0;
        switch (s->reg) {
        case W_CMD:
            newreg = val & CMD_PTR_MASK;
            val &= CMD_CMD_MASK;
            switch (val) {
            case CMD_HI:
                newreg |= CMD_HI;
                break;
            case CMD_CLR_TXINT:
                clr_txint(s);
                break;
            case CMD_CLR_IUS:
                if (s->rxint_under_svc) {
                    s->rxint_under_svc = 0;
                    if (s->txint) {
                        set_txint(s);
                    }
                } else if (s->txint_under_svc) {
                    s->txint_under_svc = 0;
                }
                escc_update_irq(s);
                break;
            default:
                break;
            }
            break;
        case W_INTR ... W_RXCTRL:
        case W_SYNC1 ... W_TXBUF:
        case W_MISC1 ... W_CLOCK:
        case W_MISC2 ... W_EXTINT:
            s->wregs[s->reg] = val;
            break;
        case W_TXCTRL1:
        case W_TXCTRL2:
            s->wregs[s->reg] = val;
            escc_update_parameters(s);
            break;
        case W_BRGLO:
        case W_BRGHI:
            s->wregs[s->reg] = val;
            s->rregs[s->reg] = val;
            escc_update_parameters(s);
            break;
        case W_MINTR:
            switch (val & MINTR_RST_MASK) {
            case 0:
            default:
                break;
            case MINTR_RST_B:
                escc_reset_chn(&serial->chn[0]);
                return;
            case MINTR_RST_A:
                escc_reset_chn(&serial->chn[1]);
                return;
            case MINTR_RST_ALL:
                escc_reset(DEVICE(serial));
                return;
            }
            break;
        default:
            break;
        }
        if (s->reg == 0)
            s->reg = newreg;
        else
            s->reg = 0;
        break;
    case SERIAL_DATA:
        trace_escc_mem_writeb_data(CHN_C(s), val);
        /*
         * Lower the irq when data is written to the Tx buffer and no other
         * interrupts are currently pending. The irq will be raised again once
         * the Tx buffer becomes empty below.
         */
        s->txint = 0;
        escc_update_irq(s);
        s->tx = val;
        if (s->wregs[W_TXCTRL2] & TXCTRL2_TXEN) { // tx enabled
            if (qemu_chr_fe_backend_connected(&s->chr)) {
                /* XXX this blocks entire thread. Rewrite to use
                 * qemu_chr_fe_write and background I/O callbacks */
                qemu_chr_fe_write_all(&s->chr, &s->tx, 1);
            } else if (s->type == escc_kbd && !s->disabled) {
                handle_kbd_command(s, val);
            }
        }
        s->rregs[R_STATUS] |= STATUS_TXEMPTY; // Tx buffer empty
        s->rregs[R_SPEC] |= SPEC_ALLSENT; // All sent
        set_txint(s);
        break;
    default:
        break;
    }
}

static uint64_t escc_mem_read(void *opaque, hwaddr addr,
                              unsigned size)
{
    ESCCState *serial = opaque;
    ESCCChannelState *s;
    uint32_t saddr;
    uint32_t ret;
    int channel;

    saddr = (addr >> reg_shift(serial)) & 1;
    channel = (addr >> chn_shift(serial)) & 1;
    s = &serial->chn[channel];
    switch (saddr) {
    case SERIAL_CTRL:
        trace_escc_mem_readb_ctrl(CHN_C(s), s->reg, s->rregs[s->reg]);
        ret = s->rregs[s->reg];
        s->reg = 0;
        return ret;
    case SERIAL_DATA:
        s->rregs[R_STATUS] &= ~STATUS_RXAV;
        clr_rxint(s);
        if (s->type == escc_kbd || s->type == escc_mouse) {
            ret = get_queue(s);
        } else {
            ret = s->rx;
        }
        trace_escc_mem_readb_data(CHN_C(s), ret);
        qemu_chr_fe_accept_input(&s->chr);
        return ret;
    default:
        break;
    }
    return 0;
}

static const MemoryRegionOps escc_mem_ops = {
    .read = escc_mem_read,
    .write = escc_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static int serial_can_receive(void *opaque)
{
    ESCCChannelState *s = opaque;
    int ret;

    if (((s->wregs[W_RXCTRL] & RXCTRL_RXEN) == 0) // Rx not enabled
        || ((s->rregs[R_STATUS] & STATUS_RXAV) == STATUS_RXAV))
        // char already available
        ret = 0;
    else
        ret = 1;
    return ret;
}

static void serial_receive_byte(ESCCChannelState *s, int ch)
{
    trace_escc_serial_receive_byte(CHN_C(s), ch);
    s->rregs[R_STATUS] |= STATUS_RXAV;
    s->rx = ch;
    set_rxint(s);
}

static void serial_receive_break(ESCCChannelState *s)
{
    s->rregs[R_STATUS] |= STATUS_BRK;
    escc_update_irq(s);
}

static void serial_receive1(void *opaque, const uint8_t *buf, int size)
{
    ESCCChannelState *s = opaque;
    serial_receive_byte(s, buf[0]);
}

static void serial_event(void *opaque, QEMUChrEvent event)
{
    ESCCChannelState *s = opaque;
    if (event == CHR_EVENT_BREAK)
        serial_receive_break(s);
}

static const VMStateDescription vmstate_escc_chn = {
    .name ="escc_chn",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(vmstate_dummy, ESCCChannelState),
        VMSTATE_UINT32(reg, ESCCChannelState),
        VMSTATE_UINT32(rxint, ESCCChannelState),
        VMSTATE_UINT32(txint, ESCCChannelState),
        VMSTATE_UINT32(rxint_under_svc, ESCCChannelState),
        VMSTATE_UINT32(txint_under_svc, ESCCChannelState),
        VMSTATE_UINT8(rx, ESCCChannelState),
        VMSTATE_UINT8(tx, ESCCChannelState),
        VMSTATE_BUFFER(wregs, ESCCChannelState),
        VMSTATE_BUFFER(rregs, ESCCChannelState),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_escc = {
    .name ="escc",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(chn, ESCCState, 2, 2, vmstate_escc_chn,
                             ESCCChannelState),
        VMSTATE_END_OF_LIST()
    }
};

static void sunkbd_handle_event(DeviceState *dev, QemuConsole *src,
                                InputEvent *evt)
{
    ESCCChannelState *s = (ESCCChannelState *)dev;
    int qcode, keycode;
    InputKeyEvent *key;

    assert(evt->type == INPUT_EVENT_KIND_KEY);
    key = evt->u.key.data;
    qcode = qemu_input_key_value_to_qcode(key->key);
    trace_escc_sunkbd_event_in(qcode, QKeyCode_str(qcode),
                               key->down);

    if (qcode == Q_KEY_CODE_CAPS_LOCK) {
        if (key->down) {
            s->caps_lock_mode ^= 1;
            if (s->caps_lock_mode == 2) {
                return; /* Drop second press */
            }
        } else {
            s->caps_lock_mode ^= 2;
            if (s->caps_lock_mode == 3) {
                return; /* Drop first release */
            }
        }
    }

    if (qcode == Q_KEY_CODE_NUM_LOCK) {
        if (key->down) {
            s->num_lock_mode ^= 1;
            if (s->num_lock_mode == 2) {
                return; /* Drop second press */
            }
        } else {
            s->num_lock_mode ^= 2;
            if (s->num_lock_mode == 3) {
                return; /* Drop first release */
            }
        }
    }

    if (qcode > qemu_input_map_qcode_to_sun_len) {
        return;
    }

    keycode = qemu_input_map_qcode_to_sun[qcode];
    if (!key->down) {
        keycode |= 0x80;
    }
    trace_escc_sunkbd_event_out(keycode);
    put_queue(s, keycode);
}

static QemuInputHandler sunkbd_handler = {
    .name  = "sun keyboard",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = sunkbd_handle_event,
};

static void handle_kbd_command(ESCCChannelState *s, int val)
{
    trace_escc_kbd_command(val);
    if (s->led_mode) { // Ignore led byte
        s->led_mode = 0;
        return;
    }
    switch (val) {
    case 1: // Reset, return type code
        clear_queue(s);
        put_queue(s, 0xff);
        put_queue(s, 4); // Type 4
        put_queue(s, 0x7f);
        break;
    case 0xe: // Set leds
        s->led_mode = 1;
        break;
    case 7: // Query layout
    case 0xf:
        clear_queue(s);
        put_queue(s, 0xfe);
        put_queue(s, 0x21); /*  en-us layout */
        break;
    default:
        break;
    }
}

static void sunmouse_event(void *opaque,
                               int dx, int dy, int dz, int buttons_state)
{
    ESCCChannelState *s = opaque;
    int ch;

    trace_escc_sunmouse_event(dx, dy, buttons_state);
    ch = 0x80 | 0x7; /* protocol start byte, no buttons pressed */

    if (buttons_state & MOUSE_EVENT_LBUTTON)
        ch ^= 0x4;
    if (buttons_state & MOUSE_EVENT_MBUTTON)
        ch ^= 0x2;
    if (buttons_state & MOUSE_EVENT_RBUTTON)
        ch ^= 0x1;

    put_queue(s, ch);

    ch = dx;

    if (ch > 127)
        ch = 127;
    else if (ch < -127)
        ch = -127;

    put_queue(s, ch & 0xff);

    ch = -dy;

    if (ch > 127)
        ch = 127;
    else if (ch < -127)
        ch = -127;

    put_queue(s, ch & 0xff);

    // MSC protocol specify two extra motion bytes

    put_queue(s, 0);
    put_queue(s, 0);
}

static void escc_init1(Object *obj)
{
    ESCCState *s = ESCC(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);
    unsigned int i;

    for (i = 0; i < 2; i++) {
        sysbus_init_irq(dev, &s->chn[i].irq);
        s->chn[i].chn = 1 - i;
    }
    s->chn[0].otherchn = &s->chn[1];
    s->chn[1].otherchn = &s->chn[0];

    sysbus_init_mmio(dev, &s->mmio);
}

static void escc_realize(DeviceState *dev, Error **errp)
{
    ESCCState *s = ESCC(dev);
    unsigned int i;

    s->chn[0].disabled = s->disabled;
    s->chn[1].disabled = s->disabled;

    memory_region_init_io(&s->mmio, OBJECT(dev), &escc_mem_ops, s, "escc",
                          ESCC_SIZE << s->it_shift);

    for (i = 0; i < 2; i++) {
        if (qemu_chr_fe_backend_connected(&s->chn[i].chr)) {
            s->chn[i].clock = s->frequency / 2;
            qemu_chr_fe_set_handlers(&s->chn[i].chr, serial_can_receive,
                                     serial_receive1, serial_event, NULL,
                                     &s->chn[i], NULL, true);
        }
    }

    if (s->chn[0].type == escc_mouse) {
        qemu_add_mouse_event_handler(sunmouse_event, &s->chn[0], 0,
                                     "QEMU Sun Mouse");
    }
    if (s->chn[1].type == escc_kbd) {
        s->chn[1].hs = qemu_input_handler_register((DeviceState *)(&s->chn[1]),
                                                   &sunkbd_handler);
    }
}

static Property escc_properties[] = {
    DEFINE_PROP_UINT32("frequency", ESCCState, frequency,   0),
    DEFINE_PROP_UINT32("it_shift",  ESCCState, it_shift,    0),
    DEFINE_PROP_BOOL("bit_swap",    ESCCState, bit_swap,    false),
    DEFINE_PROP_UINT32("disabled",  ESCCState, disabled,    0),
    DEFINE_PROP_UINT32("chnBtype",  ESCCState, chn[0].type, 0),
    DEFINE_PROP_UINT32("chnAtype",  ESCCState, chn[1].type, 0),
    DEFINE_PROP_CHR("chrB", ESCCState, chn[0].chr),
    DEFINE_PROP_CHR("chrA", ESCCState, chn[1].chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void escc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = escc_reset;
    dc->realize = escc_realize;
    dc->vmsd = &vmstate_escc;
    device_class_set_props(dc, escc_properties);
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo escc_info = {
    .name          = TYPE_ESCC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESCCState),
    .instance_init = escc_init1,
    .class_init    = escc_class_init,
};

static void escc_register_types(void)
{
    type_register_static(&escc_info);
}

type_init(escc_register_types)
