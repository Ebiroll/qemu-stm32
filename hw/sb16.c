/*
 * QEMU Soundblaster 16 emulation
 * 
 * Copyright (c) 2003-2004 Vassili Karpov (malc)
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
#include "vl.h"

#define LENOFA(a) ((int) (sizeof(a)/sizeof(a[0])))

#define dolog(...) AUD_log ("sb16", __VA_ARGS__)
#ifdef DEBUG
#define ldebug(...) dolog (__VA_ARGS__)
#else
#define ldebug(...)
#endif

/* #define DEBUG */
/* #define DEBUG_SB16_MOST */

#define IO_READ_PROTO(name)                             \
    uint32_t name (void *opaque, uint32_t nport)
#define IO_WRITE_PROTO(name)                                    \
    void name (void *opaque, uint32_t nport, uint32_t val)

static const char e3[] = "COPYRIGHT (C) CREATIVE TECHNOLOGY LTD, 1992.";

static struct {
    int ver_lo;
    int ver_hi;
    int irq;
    int dma;
    int hdma;
    int port;
} conf = {5, 4, 5, 1, 5, 0x220};

typedef struct SB16State {
    int irq;
    int dma;
    int hdma;
    int port;
    int ver;

    int in_index;
    int out_data_len;
    int fmt_stereo;
    int fmt_signed;
    int fmt_bits;
    audfmt_e fmt;
    int dma_auto;
    int block_size;
    int fifo;
    int freq;
    int time_const;
    int speaker;
    int needed_bytes;
    int cmd;
    int use_hdma;
    int highspeed;
    int can_write;

    int v2x6;

    uint8_t csp_param;
    uint8_t csp_value;
    uint8_t csp_mode;
    uint8_t csp_regs[256];
    uint8_t csp_index;
    uint8_t csp_reg83[4];
    int csp_reg83r;
    int csp_reg83w;

    uint8_t in2_data[10];
    uint8_t out_data[50];
    uint8_t test_reg;
    uint8_t last_read_byte;
    int nzero;

    int left_till_irq;

    int dma_running;
    int bytes_per_second;
    int align;
    SWVoice *voice;

    QEMUTimer *ts, *aux_ts;
    /* mixer state */
    int mixer_nreg;
    uint8_t mixer_regs[256];
} SB16State;

/* XXX: suppress that and use a context */
static struct SB16State dsp;

static int magic_of_irq (int irq)
{
    switch (irq) {
    case 5:
        return 2;
    case 7:
        return 4;
    case 9:
        return 1;
    case 10:
        return 8;
    default:
        dolog ("bad irq %d\n", irq);
        return 2;
    }
}

static int irq_of_magic (int magic)
{
    switch (magic) {
    case 1:
        return 9;
    case 2:
        return 5;
    case 4:
        return 7;
    case 8:
        return 10;
    default:
        dolog ("bad irq magic %d\n", magic);
        return -1;
    }
}

#if 0
static void log_dsp (SB16State *dsp)
{
    ldebug ("%s:%s:%d:%s:dmasize=%d:freq=%d:const=%d:speaker=%d\n",
            dsp->fmt_stereo ? "Stereo" : "Mono",
            dsp->fmt_signed ? "Signed" : "Unsigned",
            dsp->fmt_bits,
            dsp->dma_auto ? "Auto" : "Single",
            dsp->block_size,
            dsp->freq,
            dsp->time_const,
            dsp->speaker);
}
#endif

static void speaker (SB16State *s, int on)
{
    s->speaker = on;
    /* AUD_enable (s->voice, on); */
}

static void control (SB16State *s, int hold)
{
    int dma = s->use_hdma ? s->hdma : s->dma;
    s->dma_running = hold;

    ldebug ("hold %d high %d dma %d\n", hold, s->use_hdma, dma);

    if (hold) {
        DMA_hold_DREQ (dma);
        AUD_enable (s->voice, 1);
    }
    else {
        DMA_release_DREQ (dma);
        AUD_enable (s->voice, 0);
    }
}

static void aux_timer (void *opaque)
{
    SB16State *s = opaque;
    s->can_write = 1;
    pic_set_irq (s->irq, 1);
}

#define DMA8_AUTO 1
#define DMA8_HIGH 2

static void dma_cmd8 (SB16State *s, int mask, int dma_len)
{
    s->fmt = AUD_FMT_U8;
    s->use_hdma = 0;
    s->fmt_bits = 8;
    s->fmt_signed = 0;
    s->fmt_stereo = (s->mixer_regs[0x0e] & 2) != 0;
    if (-1 == s->time_const) {
        s->freq = 11025;
    }
    else {
        int tmp = (256 - s->time_const);
        s->freq = (1000000 + (tmp / 2)) / tmp;
    }

    if (-1 != dma_len)
        s->block_size = dma_len + 1;

    s->freq >>= s->fmt_stereo;
    s->left_till_irq = s->block_size;
    s->bytes_per_second = (s->freq << s->fmt_stereo);
    /* s->highspeed = (mask & DMA8_HIGH) != 0; */
    s->dma_auto = (mask & DMA8_AUTO) != 0;
    s->align = (1 << s->fmt_stereo) - 1;

    ldebug ("freq %d, stereo %d, sign %d, bits %d, "
            "dma %d, auto %d, fifo %d, high %d\n",
            s->freq, s->fmt_stereo, s->fmt_signed, s->fmt_bits,
            s->block_size, s->dma_auto, s->fifo, s->highspeed);

    if (s->freq)
        s->voice = AUD_open (s->voice, "sb16", s->freq,
                             1 << s->fmt_stereo, s->fmt);

    control (s, 1);
    speaker (s, 1);
}

static void dma_cmd (SB16State *s, uint8_t cmd, uint8_t d0, int dma_len)
{
    s->use_hdma = cmd < 0xc0;
    s->fifo = (cmd >> 1) & 1;
    s->dma_auto = (cmd >> 2) & 1;
    s->fmt_signed = (d0 >> 4) & 1;
    s->fmt_stereo = (d0 >> 5) & 1;

    switch (cmd >> 4) {
    case 11:
        s->fmt_bits = 16;
        break;

    case 12:
        s->fmt_bits = 8;
        break;
    }

    if (-1 != s->time_const) {
#if 1
        int tmp = 256 - s->time_const;
        s->freq = (1000000 + (tmp / 2)) / tmp;
#else
        /* s->freq = 1000000 / ((255 - s->time_const) << s->fmt_stereo); */
        s->freq = 1000000 / ((255 - s->time_const));
#endif
        s->time_const = -1;
    }

    s->block_size = dma_len + 1;
    s->block_size <<= (s->fmt_bits == 16);
    if (!s->dma_auto)           /* Miles Sound System ? */
        s->block_size <<= s->fmt_stereo;

    ldebug ("freq %d, stereo %d, sign %d, bits %d, "
            "dma %d, auto %d, fifo %d, high %d\n",
            s->freq, s->fmt_stereo, s->fmt_signed, s->fmt_bits,
            s->block_size, s->dma_auto, s->fifo, s->highspeed);

    if (16 == s->fmt_bits) {
        if (s->fmt_signed) {
            s->fmt = AUD_FMT_S16;
        }
        else {
            s->fmt = AUD_FMT_U16;
        }
    }
    else {
        if (s->fmt_signed) {
            s->fmt = AUD_FMT_S8;
        }
        else {
            s->fmt = AUD_FMT_U8;
        }
    }

    s->left_till_irq = s->block_size;

    s->bytes_per_second = (s->freq << s->fmt_stereo) << (s->fmt_bits == 16);
    s->highspeed = 0;
    s->align = (1 << (s->fmt_stereo + (s->fmt_bits == 16))) - 1;

    if (s->freq)
        s->voice = AUD_open (s->voice, "sb16", s->freq,
                             1 << s->fmt_stereo, s->fmt);

    control (s, 1);
    speaker (s, 1);
}

static inline void dsp_out_data (SB16State *s, uint8_t val)
{
    ldebug ("outdata %#x\n", val);
    if (s->out_data_len < sizeof (s->out_data))
        s->out_data[s->out_data_len++] = val;
}

static inline uint8_t dsp_get_data (SB16State *s)
{
    if (s->in_index)
        return s->in2_data[--s->in_index];
    else {
        dolog ("buffer underflow\n");
        return 0;
    }
}

static void command (SB16State *s, uint8_t cmd)
{
    ldebug ("command %#x\n", cmd);

    if (cmd > 0xaf && cmd < 0xd0) {
        if (cmd & 8) {
            dolog ("ADC not yet supported (command %#x)\n", cmd);
        }

        switch (cmd >> 4) {
        case 11:
        case 12:
            break;
        default:
            dolog ("%#x wrong bits\n", cmd);
        }
        s->needed_bytes = 3;
    }
    else {
        switch (cmd) {
        case 0x03:
            dsp_out_data (s, 0x10); /* s->csp_param); */
            goto warn;

        case 0x04:
            s->needed_bytes = 1;
            goto warn;

        case 0x05:
            s->needed_bytes = 2;
            goto warn;

        case 0x08:
            /* __asm__ ("int3"); */
            goto warn;

        case 0x0e:
            s->needed_bytes = 2;
            goto warn;

        case 0x09:
            dsp_out_data (s, 0xf8);
            goto warn;

        case 0x0f:
            s->needed_bytes = 1;
            goto warn;

        case 0x10:
            s->needed_bytes = 1;
            goto warn;

        case 0x14:
            s->needed_bytes = 2;
            s->block_size = 0;
            break;

        case 0x20:              /* Direct ADC, Juice/PL */
            dsp_out_data (s, 0xff);
            goto warn;

        case 0x35:
            dolog ("MIDI command(0x35) not implemented\n");
            break;

        case 0x40:
            s->freq = -1;
            s->time_const = -1;
            s->needed_bytes = 1;
            break;

        case 0x41:
            s->freq = -1;
            s->time_const = -1;
            s->needed_bytes = 2;
            break;

        case 0x42:
            s->freq = -1;
            s->time_const = -1;
            s->needed_bytes = 2;
            goto warn;

        case 0x45:
            dsp_out_data (s, 0xaa);
            goto warn;

        case 0x47:                /* Continue Auto-Initialize DMA 16bit */
            break;

        case 0x48:
            s->needed_bytes = 2;
            break;

        case 0x80:
            s->needed_bytes = 2;
            break;

        case 0x90:
        case 0x91:
            dma_cmd8 (s, ((cmd & 1) == 0) | DMA8_HIGH, -1);
            break;

        case 0xd0:              /* halt DMA operation. 8bit */
            control (s, 0);
            break;

        case 0xd1:              /* speaker on */
            speaker (s, 1);
            break;

        case 0xd3:              /* speaker off */
            speaker (s, 0);
            break;

        case 0xd4:              /* continue DMA operation. 8bit */
            control (s, 1);
            break;

        case 0xd5:              /* halt DMA operation. 16bit */
            control (s, 0);
            break;

        case 0xd6:              /* continue DMA operation. 16bit */
            control (s, 1);
            break;

        case 0xd9:              /* exit auto-init DMA after this block. 16bit */
            s->dma_auto = 0;
            break;

        case 0xda:              /* exit auto-init DMA after this block. 8bit */
            s->dma_auto = 0;
            break;

        case 0xe0:
            s->needed_bytes = 1;
            goto warn;

        case 0xe1:
            dsp_out_data (s, s->ver & 0xff);
            dsp_out_data (s, s->ver >> 8);
            break;

        case 0xe2:
            s->needed_bytes = 1;
            goto warn;

        case 0xe3:
            {
                int i;
                for (i = sizeof (e3) - 1; i >= 0; --i)
                    dsp_out_data (s, e3[i]);
            }
            break;

        case 0xe4:              /* write test reg */
            s->needed_bytes = 1;
            break;

        case 0xe7:
            dolog ("Attempt to probe for ESS (0xe7)?\n");
            return;

        case 0xe8:              /* read test reg */
            dsp_out_data (s, s->test_reg);
            break;

        case 0xf2:
        case 0xf3:
            dsp_out_data (s, 0xaa);
            s->mixer_regs[0x82] |= (cmd == 0xf2) ? 1 : 2;
            pic_set_irq (s->irq, 1);
            break;

        case 0xf9:
            s->needed_bytes = 1;
            goto warn;

        case 0xfa:
            dsp_out_data (s, 0);
            goto warn;

        case 0xfc:              /* FIXME */
            dsp_out_data (s, 0);
            goto warn;

        default:
            dolog ("unrecognized command %#x\n", cmd);
            return;
        }
    }

    s->cmd = cmd;
    if (!s->needed_bytes)
        ldebug ("\n");
    return;

 warn:
    dolog ("warning: command %#x,%d is not trully understood yet\n",
           cmd, s->needed_bytes);
    s->cmd = cmd;
    return;
}

static uint16_t dsp_get_lohi (SB16State *s)
{
    uint8_t hi = dsp_get_data (s);
    uint8_t lo = dsp_get_data (s);
    return (hi << 8) | lo;
}

static uint16_t dsp_get_hilo (SB16State *s)
{
    uint8_t lo = dsp_get_data (s);
    uint8_t hi = dsp_get_data (s);
    return (hi << 8) | lo;
}

static void complete (SB16State *s)
{
    int d0, d1, d2;
    ldebug ("complete command %#x, in_index %d, needed_bytes %d\n",
            s->cmd, s->in_index, s->needed_bytes);

    if (s->cmd > 0xaf && s->cmd < 0xd0) {
        d2 = dsp_get_data (s);
        d1 = dsp_get_data (s);
        d0 = dsp_get_data (s);

        if (s->cmd & 8) {
            dolog ("ADC params cmd = %#x d0 = %d, d1 = %d, d2 = %d\n",
                   s->cmd, d0, d1, d2);
        }
        else {
            ldebug ("cmd = %#x d0 = %d, d1 = %d, d2 = %d\n",
                    s->cmd, d0, d1, d2);
            dma_cmd (s, s->cmd, d0, d1 + (d2 << 8));
        }
    }
    else {
        switch (s->cmd) {
        case 0x04:
            s->csp_mode = dsp_get_data (s);
            s->csp_reg83r = 0;
            s->csp_reg83w = 0;
            ldebug ("CSP command 0x04: mode=%#x\n", s->csp_mode);
            break;

        case 0x05:
            s->csp_param = dsp_get_data (s);
            s->csp_value = dsp_get_data (s);
            ldebug ("CSP command 0x05: param=%#x value=%#x\n",
                    s->csp_param,
                    s->csp_value);
            break;

        case 0x0e:
            d0 = dsp_get_data (s);
            d1 = dsp_get_data (s);
            ldebug ("write CSP register %d <- %#x\n", d1, d0);
            if (d1 == 0x83) {
                ldebug ("0x83[%d] <- %#x\n", s->csp_reg83r, d0);
                s->csp_reg83[s->csp_reg83r % 4] = d0;
                s->csp_reg83r += 1;
            }
            else
                s->csp_regs[d1] = d0;
            break;

        case 0x0f:
            d0 = dsp_get_data (s);
            ldebug ("read CSP register %#x -> %#x, mode=%#x\n",
                    d0, s->csp_regs[d0], s->csp_mode);
            if (d0 == 0x83) {
                ldebug ("0x83[%d] -> %#x\n",
                        s->csp_reg83w,
                        s->csp_reg83[s->csp_reg83w % 4]);
                dsp_out_data (s, s->csp_reg83[s->csp_reg83w % 4]);
                s->csp_reg83w += 1;
            }
            else
                dsp_out_data (s, s->csp_regs[d0]);
            break;

        case 0x10:
            d0 = dsp_get_data (s);
            dolog ("cmd 0x10 d0=%#x\n", d0);
            break;

        case 0x14:
            dma_cmd8 (s, 0, dsp_get_lohi (s));
            /* s->can_write = 0; */
            /* qemu_mod_timer (s->aux_ts, qemu_get_clock (vm_clock) + (ticks_per_sec * 320) / 1000000); */
            break;

        case 0x40:
            s->time_const = dsp_get_data (s);
            ldebug ("set time const %d\n", s->time_const);
            break;

        case 0x42:              /* FT2 sets output freq with this, go figure */
            dolog ("cmd 0x42 might not do what it think it should\n");

        case 0x41:
            s->freq = dsp_get_hilo (s);
            ldebug ("set freq %d\n", s->freq);
            break;

        case 0x48:
            s->block_size = dsp_get_lohi (s);
            /* s->highspeed = 1; */
            ldebug ("set dma block len %d\n", s->block_size);
            break;

        case 0x80:
            {
                int samples, bytes;
                int64_t ticks;

                if (-1 == s->freq)
                    s->freq = 11025;
                samples = dsp_get_lohi (s);
                bytes = samples << s->fmt_stereo << (s->fmt_bits == 16);
                ticks = ticks_per_sec / (s->freq / bytes);
                if (ticks < ticks_per_sec / 1024)
                    pic_set_irq (s->irq, 1);
                else
                    qemu_mod_timer (s->aux_ts, qemu_get_clock (vm_clock) + ticks);
                ldebug ("mix silence %d %d %lld\n", samples, bytes, ticks);
            }
            break;

        case 0xe0:
            d0 = dsp_get_data (s);
            s->out_data_len = 0;
            ldebug ("E0 data = %#x\n", d0);
            dsp_out_data(s, ~d0);
            break;

        case 0xe2:
            d0 = dsp_get_data (s);
            dolog ("E2 = %#x\n", d0);
            break;

        case 0xe4:
            s->test_reg = dsp_get_data (s);
            break;

        case 0xf9:
            d0 = dsp_get_data (s);
            ldebug ("command 0xf9 with %#x\n", d0);
            switch (d0) {
            case 0x0e:
                dsp_out_data (s, 0xff);
                break;

            case 0x0f:
                dsp_out_data (s, 0x07);
                break;

            case 0x37:
                dsp_out_data (s, 0x38);
                break;

            default:
                dsp_out_data (s, 0x00);
                break;
            }
            break;

        default:
            dolog ("complete: unrecognized command %#x\n", s->cmd);
            return;
        }
    }

    ldebug ("\n");
    s->cmd = -1;
    return;
}

static void reset (SB16State *s)
{
    pic_set_irq (s->irq, 0);
    if (s->dma_auto) {
        pic_set_irq (s->irq, 1);
        pic_set_irq (s->irq, 0);
    }

    s->mixer_regs[0x82] = 0;
    s->dma_auto = 0;
    s->in_index = 0;
    s->out_data_len = 0;
    s->left_till_irq = 0;
    s->needed_bytes = 0;
    s->block_size = -1;
    s->nzero = 0;
    s->highspeed = 0;
    s->v2x6 = 0;

    dsp_out_data(s, 0xaa);
    speaker (s, 0);
    control (s, 0);
}

static IO_WRITE_PROTO (dsp_write)
{
    SB16State *s = opaque;
    int iport;

    iport = nport - s->port;

    ldebug ("write %#x <- %#x\n", nport, val);
    switch (iport) {
    case 0x06:
        switch (val) {
        case 0x00:
            if (s->v2x6 == 1) {
                if (0 && s->highspeed) {
                    s->highspeed = 0;
                    pic_set_irq (s->irq, 0);
                    control (s, 0);
                }
                else
                    reset (s);
            }
            s->v2x6 = 0;
            break;

        case 0x01:
        case 0x03:              /* FreeBSD kludge */
            s->v2x6 = 1;
            break;

        case 0xc6:
            s->v2x6 = 0;        /* Prince of Persia, csp.sys, diagnose.exe */
            break;

        case 0xb8:              /* Panic */
            reset (s);
            break;

        case 0x39:
            dsp_out_data (s, 0x38);
            reset (s);
            s->v2x6 = 0x39;
            break;

        default:
            s->v2x6 = val;
            break;
        }
        break;

    case 0x0c:                  /* write data or command | write status */
/*         if (s->highspeed) */
/*             break; */

        if (0 == s->needed_bytes) {
            command (s, val);
#if 0
            if (0 == s->needed_bytes) {
                log_dsp (s);
            }
#endif
        }
        else {
            if (s->in_index == sizeof (s->in2_data)) {
                dolog ("in data overrun\n");
            }
            else {
                s->in2_data[s->in_index++] = val;
                if (s->in_index == s->needed_bytes) {
                    s->needed_bytes = 0;
                    complete (s);
#if 0
                    log_dsp (s);
#endif
                }
            }
        }
        break;

    default:
        ldebug ("(nport=%#x, val=%#x)\n", nport, val);
        break;
    }
}

static IO_READ_PROTO (dsp_read)
{
    SB16State *s = opaque;
    int iport, retval, ack = 0;

    iport = nport - s->port;

    switch (iport) {
    case 0x06:                  /* reset */
        retval = 0xff;
        break;

    case 0x0a:                  /* read data */
        if (s->out_data_len) {
            retval = s->out_data[--s->out_data_len];
            s->last_read_byte = retval;
        }
        else {
            dolog ("empty output buffer\n");
            retval = s->last_read_byte;
            /* goto error; */
        }
        break;

    case 0x0c:                  /* 0 can write */
        retval = s->can_write ? 0 : 0x80;
        break;

    case 0x0d:                  /* timer interrupt clear */
        /* dolog ("timer interrupt clear\n"); */
        retval = 0;
        break;

    case 0x0e:                  /* data available status | irq 8 ack */
        retval = (!s->out_data_len || s->highspeed) ? 0 : 0x80;
        if (s->mixer_regs[0x82] & 1) {
            ack = 1;
            s->mixer_regs[0x82] &= 1;
            pic_set_irq (s->irq, 0);
        }
        break;

    case 0x0f:                  /* irq 16 ack */
        retval = 0xff;
        if (s->mixer_regs[0x82] & 2) {
            ack = 1;
            s->mixer_regs[0x82] &= 2;
            pic_set_irq (s->irq, 0);
        }
        break;

    default:
        goto error;
    }

    if (!ack)
        ldebug ("read %#x -> %#x\n", nport, retval);

    return retval;

 error:
    dolog ("WARNING dsp_read %#x error\n", nport);
    return 0xff;
}

static void reset_mixer (SB16State *s)
{
    int i;

    memset (s->mixer_regs, 0xff, 0x7f);
    memset (s->mixer_regs + 0x83, 0xff, sizeof (s->mixer_regs) - 0x83);

    s->mixer_regs[0x02] = 4;    /* master volume 3bits */
    s->mixer_regs[0x06] = 4;    /* MIDI volume 3bits */
    s->mixer_regs[0x08] = 0;    /* CD volume 3bits */
    s->mixer_regs[0x0a] = 0;    /* voice volume 2bits */

    /* d5=input filt, d3=lowpass filt, d1,d2=input source */
    s->mixer_regs[0x0c] = 0;

    /* d5=output filt, d1=stereo switch */
    s->mixer_regs[0x0e] = 0;

    /* voice volume L d5,d7, R d1,d3 */
    s->mixer_regs[0x04] = (4 << 5) | (4 << 1);
    /* master ... */
    s->mixer_regs[0x22] = (4 << 5) | (4 << 1);
    /* MIDI ... */
    s->mixer_regs[0x26] = (4 << 5) | (4 << 1);

    for (i = 0x30; i < 0x48; i++) {
        s->mixer_regs[i] = 0x20;
    }
}

static IO_WRITE_PROTO(mixer_write_indexb)
{
    SB16State *s = opaque;
    s->mixer_nreg = val;
}

static IO_WRITE_PROTO(mixer_write_datab)
{
    SB16State *s = opaque;

    ldebug ("mixer_write [%#x] <- %#x\n", s->mixer_nreg, val);
    if (s->mixer_nreg > sizeof (s->mixer_regs))
        return;

    switch (s->mixer_nreg) {
    case 0x00:
        reset_mixer (s);
        break;

    case 0x80:
        {
            int irq = irq_of_magic (val);
            ldebug ("setting irq to %d (val=%#x)\n", irq, val);
            if (irq > 0)
                s->irq = irq;
        }
        break;

    case 0x81:
        {
            int dma, hdma;

            dma = lsbindex (val & 0xf);
            hdma = lsbindex (val & 0xf0);
            dolog ("attempt to set DMA register 8bit %d, 16bit %d (val=%#x)\n",
                   dma, hdma, val);
#if 0
            s->dma = dma;
            s->hdma = hdma;
#endif
        }
        break;

    case 0x82:
        dolog ("attempt to write into IRQ status register (val=%#x)\n",
               val);
        return;

    default:
        if (s->mixer_nreg >= 0x80)
            dolog ("attempt to write mixer[%#x] <- %#x\n", s->mixer_nreg, val);
        break;
    }

    s->mixer_regs[s->mixer_nreg] = val;
}

static IO_WRITE_PROTO(mixer_write_indexw)
{
    mixer_write_indexb (opaque, nport, val & 0xff);
    mixer_write_datab (opaque, nport, (val >> 8) & 0xff);
}

static IO_READ_PROTO(mixer_read)
{
    SB16State *s = opaque;
    ldebug ("mixer_read[%#x] -> %#x\n",
            s->mixer_nreg, s->mixer_regs[s->mixer_nreg]);
    return s->mixer_regs[s->mixer_nreg];
}

static int write_audio (SB16State *s, int nchan, int dma_pos,
                        int dma_len, int len)
{
    int temp, net;
    uint8_t tmpbuf[4096];

    temp = len;
    net = 0;

    while (temp) {
        int left = dma_len - dma_pos;
        int to_copy, copied;

        to_copy = audio_MIN (temp, left);
        if (to_copy > sizeof(tmpbuf))
            to_copy = sizeof(tmpbuf);

        copied = DMA_read_memory (nchan, tmpbuf, dma_pos, to_copy);
        copied = AUD_write (s->voice, tmpbuf, copied);

        temp -= copied;
        dma_pos = (dma_pos + copied) % dma_len;
        net += copied;

        if (!copied)
            break;
    }

    return net;
}

static int SB_read_DMA (void *opaque, int nchan, int dma_pos, int dma_len)
{
    SB16State *s = opaque;
    int free, rfree, till, copy, written, elapsed;

    if (s->left_till_irq < 0) {
        s->left_till_irq = s->block_size;
    }

    elapsed = AUD_calc_elapsed (s->voice);
    free = elapsed;/* AUD_get_free (s->voice); */
    rfree = free;
    free = audio_MIN (free, elapsed) & ~s->align;

    if ((free <= 0) || !dma_len) {
        return dma_pos;
    }

    copy = free;
    till = s->left_till_irq;

#ifdef DEBUG_SB16_MOST
    dolog ("pos:%06d free:%d,%d till:%d len:%d\n",
           dma_pos, free, AUD_get_free (s->voice), till, dma_len);
#endif

    if (till <= copy) {
        if (0 == s->dma_auto) {
            copy = till;
        }
    }

    written = write_audio (s, nchan, dma_pos, dma_len, copy);
    dma_pos = (dma_pos + written) % dma_len;
    s->left_till_irq -= written;

    if (s->left_till_irq <= 0) {
        s->mixer_regs[0x82] |= (nchan & 4) ? 2 : 1;
        pic_set_irq (s->irq, 1);
        if (0 == s->dma_auto) {
            control (s, 0);
            speaker (s, 0);
        }
    }

#ifdef DEBUG_SB16_MOST
    ldebug ("pos %5d free %5d size %5d till % 5d copy %5d dma size %5d\n",
            dma_pos, free, dma_len, s->left_till_irq, copy, s->block_size);
#endif

    while (s->left_till_irq <= 0) {
        s->left_till_irq = s->block_size + s->left_till_irq;
    }

    AUD_adjust (s->voice, written);
    return dma_pos;
}

void SB_timer (void *opaque)
{
    SB16State *s = opaque;
    AUD_run ();
    qemu_mod_timer (s->ts, qemu_get_clock (vm_clock) + 1);
}

static void SB_save (QEMUFile *f, void *opaque)
{
    SB16State *s = opaque;

    qemu_put_be32s (f, &s->irq);
    qemu_put_be32s (f, &s->dma);
    qemu_put_be32s (f, &s->hdma);
    qemu_put_be32s (f, &s->port);
    qemu_put_be32s (f, &s->ver);
    qemu_put_be32s (f, &s->in_index);
    qemu_put_be32s (f, &s->out_data_len);
    qemu_put_be32s (f, &s->fmt_stereo);
    qemu_put_be32s (f, &s->fmt_signed);
    qemu_put_be32s (f, &s->fmt_bits);
    qemu_put_be32s (f, &s->fmt);
    qemu_put_be32s (f, &s->dma_auto);
    qemu_put_be32s (f, &s->block_size);
    qemu_put_be32s (f, &s->fifo);
    qemu_put_be32s (f, &s->freq);
    qemu_put_be32s (f, &s->time_const);
    qemu_put_be32s (f, &s->speaker);
    qemu_put_be32s (f, &s->needed_bytes);
    qemu_put_be32s (f, &s->cmd);
    qemu_put_be32s (f, &s->use_hdma);
    qemu_put_be32s (f, &s->highspeed);
    qemu_put_be32s (f, &s->can_write);
    qemu_put_be32s (f, &s->v2x6);

    qemu_put_8s (f, &s->csp_param);
    qemu_put_8s (f, &s->csp_value);
    qemu_put_8s (f, &s->csp_mode);
    qemu_put_8s (f, &s->csp_param);
    qemu_put_buffer (f, s->csp_regs, 256);
    qemu_put_8s (f, &s->csp_index);
    qemu_put_buffer (f, s->csp_reg83, 4);
    qemu_put_be32s (f, &s->csp_reg83r);
    qemu_put_be32s (f, &s->csp_reg83w);

    qemu_put_buffer (f, s->in2_data, sizeof (s->in2_data));
    qemu_put_buffer (f, s->out_data, sizeof (s->out_data));
    qemu_put_8s (f, &s->test_reg);
    qemu_put_8s (f, &s->last_read_byte);

    qemu_put_be32s (f, &s->nzero);
    qemu_put_be32s (f, &s->left_till_irq);
    qemu_put_be32s (f, &s->dma_running);
    qemu_put_be32s (f, &s->bytes_per_second);
    qemu_put_be32s (f, &s->align);

    qemu_put_be32s (f, &s->mixer_nreg);
    qemu_put_buffer (f, s->mixer_regs, 256);
}

static int SB_load (QEMUFile *f, void *opaque, int version_id)
{
    SB16State *s = opaque;

    if (version_id != 1)
        return -EINVAL;

    qemu_get_be32s (f, &s->irq);
    qemu_get_be32s (f, &s->dma);
    qemu_get_be32s (f, &s->hdma);
    qemu_get_be32s (f, &s->port);
    qemu_get_be32s (f, &s->ver);
    qemu_get_be32s (f, &s->in_index);
    qemu_get_be32s (f, &s->out_data_len);
    qemu_get_be32s (f, &s->fmt_stereo);
    qemu_get_be32s (f, &s->fmt_signed);
    qemu_get_be32s (f, &s->fmt_bits);
    qemu_get_be32s (f, &s->fmt);
    qemu_get_be32s (f, &s->dma_auto);
    qemu_get_be32s (f, &s->block_size);
    qemu_get_be32s (f, &s->fifo);
    qemu_get_be32s (f, &s->freq);
    qemu_get_be32s (f, &s->time_const);
    qemu_get_be32s (f, &s->speaker);
    qemu_get_be32s (f, &s->needed_bytes);
    qemu_get_be32s (f, &s->cmd);
    qemu_get_be32s (f, &s->use_hdma);
    qemu_get_be32s (f, &s->highspeed);
    qemu_get_be32s (f, &s->can_write);
    qemu_get_be32s (f, &s->v2x6);

    qemu_get_8s (f, &s->csp_param);
    qemu_get_8s (f, &s->csp_value);
    qemu_get_8s (f, &s->csp_mode);
    qemu_get_8s (f, &s->csp_param);
    qemu_get_buffer (f, s->csp_regs, 256);
    qemu_get_8s (f, &s->csp_index);
    qemu_get_buffer (f, s->csp_reg83, 4);
    qemu_get_be32s (f, &s->csp_reg83r);
    qemu_get_be32s (f, &s->csp_reg83w);

    qemu_get_buffer (f, s->in2_data, sizeof (s->in2_data));
    qemu_get_buffer (f, s->out_data, sizeof (s->out_data));
    qemu_get_8s (f, &s->test_reg);
    qemu_get_8s (f, &s->last_read_byte);

    qemu_get_be32s (f, &s->nzero);
    qemu_get_be32s (f, &s->left_till_irq);
    qemu_get_be32s (f, &s->dma_running);
    qemu_get_be32s (f, &s->bytes_per_second);
    qemu_get_be32s (f, &s->align);

    qemu_get_be32s (f, &s->mixer_nreg);
    qemu_get_buffer (f, s->mixer_regs, 256);

    if (s->voice) {
        AUD_close (s->voice);
        s->voice = NULL;
    }

    if (s->dma_running) {
        if (s->freq)
            s->voice = AUD_open (s->voice, "sb16", s->freq,
                                 1 << s->fmt_stereo, s->fmt);

        control (s, 1);
        speaker (s, s->speaker);
    }
    return 0;
}

void SB16_init (void)
{
    SB16State *s = &dsp;
    int i;
    static const uint8_t dsp_write_ports[] = {0x6, 0xc};
    static const uint8_t dsp_read_ports[] = {0x6, 0xa, 0xc, 0xd, 0xe, 0xf};

    s->ts = qemu_new_timer (vm_clock, SB_timer, s);
    if (!s->ts)
        return;

    s->irq = conf.irq;
    s->dma = conf.dma;
    s->hdma = conf.hdma;
    s->port = conf.port;
    s->ver = conf.ver_lo | (conf.ver_hi << 8);

    s->mixer_regs[0x80] = magic_of_irq (s->irq);
    s->mixer_regs[0x81] = (1 << s->dma) | (1 << s->hdma);
    s->mixer_regs[0x82] = 2 << 5;

    s->csp_regs[5] = 1;
    s->csp_regs[9] = 0xf8;

    reset_mixer (s);
    s->aux_ts = qemu_new_timer (vm_clock, aux_timer, s);
    if (!s->aux_ts)
        return;

    for (i = 0; i < LENOFA (dsp_write_ports); i++) {
        register_ioport_write (s->port + dsp_write_ports[i], 1, 1, dsp_write, s);
    }

    for (i = 0; i < LENOFA (dsp_read_ports); i++) {
        register_ioport_read (s->port + dsp_read_ports[i], 1, 1, dsp_read, s);
    }

    register_ioport_write (s->port + 0x4, 1, 1, mixer_write_indexb, s);
    register_ioport_write (s->port + 0x4, 1, 2, mixer_write_indexw, s);
    register_ioport_read (s->port + 0x5, 1, 1, mixer_read, s);
    register_ioport_write (s->port + 0x5, 1, 1, mixer_write_datab, s);

    DMA_register_channel (s->hdma, SB_read_DMA, s);
    DMA_register_channel (s->dma, SB_read_DMA, s);
    s->can_write = 1;

    qemu_mod_timer (s->ts, qemu_get_clock (vm_clock) + 1);
    register_savevm ("sb16", 0, 1, SB_save, SB_load, s);
}
