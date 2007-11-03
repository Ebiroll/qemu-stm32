/*
 * TI TSC2102 (touchscreen/sensors/audio controller) emulator.
 *
 * Copyright (c) 2006 Andrzej Zaborowski  <balrog@zabor.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include "vl.h"

#define TSC_DATA_REGISTERS_PAGE		0x0
#define TSC_CONTROL_REGISTERS_PAGE	0x1
#define TSC_AUDIO_REGISTERS_PAGE	0x2

#define TSC_VERBOSE

#define TSC_CUT_RESOLUTION(value, p)	((value) >> (16 - resolution[p]))

struct tsc210x_state_s {
    qemu_irq pint;
    QEMUTimer *timer;
    struct uwire_slave_s chip;

    int x, y;
    int pressure;

    int state, page, offset, irq;
    uint16_t command, dav;

    int busy;
    int enabled;
    int host_mode;
    int function;
    int nextfunction;
    int precision;
    int nextprecision;
    int filter;
    int pin_func;
    int ref;
    int timing;
    int noise;

    uint16_t audio_ctrl1;
    uint16_t audio_ctrl2;
    uint16_t audio_ctrl3;
    uint16_t pll[2];
    uint16_t volume;
    int64_t volume_change;
    int softstep;
    uint16_t dac_power;
    int64_t powerdown;
    uint16_t filter_data[0x14];
};

static const int resolution[4] = { 12, 8, 10, 12 };

#define TSC_MODE_NO_SCAN	0x0
#define TSC_MODE_XY_SCAN	0x1
#define TSC_MODE_XYZ_SCAN	0x2
#define TSC_MODE_X		0x3
#define TSC_MODE_Y		0x4
#define TSC_MODE_Z		0x5
#define TSC_MODE_BAT1		0x6
#define TSC_MODE_BAT2		0x7
#define TSC_MODE_AUX		0x8
#define TSC_MODE_AUX_SCAN	0x9
#define TSC_MODE_TEMP1		0xa
#define TSC_MODE_PORT_SCAN	0xb
#define TSC_MODE_TEMP2		0xc
#define TSC_MODE_XX_DRV		0xd
#define TSC_MODE_YY_DRV		0xe
#define TSC_MODE_YX_DRV		0xf

static const uint16_t mode_regs[16] = {
    0x0000,	/* No scan */
    0x0600,	/* X, Y scan */
    0x0780,	/* X, Y, Z scan */
    0x0400,	/* X */
    0x0200,	/* Y */
    0x0180,	/* Z */
    0x0040,	/* BAT1 */
    0x0030,	/* BAT2 */
    0x0010,	/* AUX */
    0x0010,	/* AUX scan */
    0x0004,	/* TEMP1 */
    0x0070,	/* Port scan */
    0x0002,	/* TEMP2 */
    0x0000,	/* X+, X- drivers */
    0x0000,	/* Y+, Y- drivers */
    0x0000,	/* Y+, X- drivers */
};

/*
 * Convert screen coordinates to arbitrary values that the
 * touchscreen in my Palm Tungsten E device returns.
 * This shouldn't really matter (because the guest system
 * should calibrate the touchscreen anyway), but let's
 * imitate some real hardware.
 */
#define X_TRANSFORM(value)		\
    ((3850 - ((int) (value) * (3850 - 250) / 32768)) << 4)
#define Y_TRANSFORM(value)		\
    ((150 + ((int) (value) * (3037 - 150) / 32768)) << 4)
#define Z1_TRANSFORM(s)			\
    ((400 - (s)->x + ((s)->pressure << 9)) << 4)
#define Z2_TRANSFORM(s)			\
    ((4000 + (s)->y - ((s)->pressure << 10)) << 4)
#define BAT1_VAL			0x8660
#define BAT2_VAL			0x0000
#define AUX1_VAL			0x35c0
#define AUX2_VAL			0xffff
#define TEMP1_VAL			0x8c70
#define TEMP2_VAL			0xa5b0

#define TSC_POWEROFF_DELAY		50
#define TSC_SOFTSTEP_DELAY		50

static void tsc210x_reset(struct tsc210x_state_s *s)
{
    s->state = 0;
    s->pin_func = 2;
    s->enabled = 0;
    s->busy = 0;
    s->nextfunction = 0;
    s->ref = 0;
    s->timing = 0;
    s->irq = 0;
    s->dav = 0;

    s->audio_ctrl1 = 0x0000;
    s->audio_ctrl2 = 0x4410;
    s->audio_ctrl3 = 0x0000;
    s->pll[0] = 0x1004;
    s->pll[1] = 0x0000;
    s->volume = 0xffff;
    s->dac_power = 0x8540;
    s->softstep = 1;
    s->volume_change = 0;
    s->powerdown = 0;
    s->filter_data[0x00] = 0x6be3;
    s->filter_data[0x01] = 0x9666;
    s->filter_data[0x02] = 0x675d;
    s->filter_data[0x03] = 0x6be3;
    s->filter_data[0x04] = 0x9666;
    s->filter_data[0x05] = 0x675d;
    s->filter_data[0x06] = 0x7d83;
    s->filter_data[0x07] = 0x84ee;
    s->filter_data[0x08] = 0x7d83;
    s->filter_data[0x09] = 0x84ee;
    s->filter_data[0x0a] = 0x6be3;
    s->filter_data[0x0b] = 0x9666;
    s->filter_data[0x0c] = 0x675d;
    s->filter_data[0x0d] = 0x6be3;
    s->filter_data[0x0e] = 0x9666;
    s->filter_data[0x0f] = 0x675d;
    s->filter_data[0x10] = 0x7d83;
    s->filter_data[0x11] = 0x84ee;
    s->filter_data[0x12] = 0x7d83;
    s->filter_data[0x13] = 0x84ee;

    qemu_set_irq(s->pint, !s->irq);
}

static uint16_t tsc2102_data_register_read(struct tsc210x_state_s *s, int reg)
{
    switch (reg) {
    case 0x00:	/* X */
        s->dav &= 0xfbff;
        return TSC_CUT_RESOLUTION(X_TRANSFORM(s->x), s->precision) +
                (s->noise & 3);

    case 0x01:	/* Y */
        s->noise ++;
        s->dav &= 0xfdff;
        return TSC_CUT_RESOLUTION(Y_TRANSFORM(s->y), s->precision) ^
                (s->noise & 3);

    case 0x02:	/* Z1 */
        s->dav &= 0xfeff;
        return TSC_CUT_RESOLUTION(Z1_TRANSFORM(s), s->precision) -
                (s->noise & 3);

    case 0x03:	/* Z2 */
        s->dav &= 0xff7f;
        return TSC_CUT_RESOLUTION(Z2_TRANSFORM(s), s->precision) |
                (s->noise & 3);

    case 0x04:	/* KPData */
        return 0xffff;

    case 0x05:	/* BAT1 */
        s->dav &= 0xffbf;
        return TSC_CUT_RESOLUTION(BAT1_VAL, s->precision);

    case 0x06:	/* BAT2 */
        s->dav &= 0xffdf;
        return TSC_CUT_RESOLUTION(BAT2_VAL, s->precision);

    case 0x07:	/* AUX1 */
        s->dav &= 0xffef;
        return TSC_CUT_RESOLUTION(AUX1_VAL, s->precision);

    case 0x08:	/* AUX2 */
        s->dav &= 0xfff7;
        return 0xffff;

    case 0x09:	/* TEMP1 */
        s->dav &= 0xfffb;
        return TSC_CUT_RESOLUTION(TEMP1_VAL, s->precision);

    case 0x0a:	/* TEMP2 */
        s->dav &= 0xfffd;
        return TSC_CUT_RESOLUTION(TEMP2_VAL, s->precision);

    case 0x0b:	/* DAC */
        s->dav &= 0xfffe;
        return 0xffff;

    default:
#ifdef TSC_VERBOSE
        fprintf(stderr, "tsc2102_data_register_read: "
                        "no such register: 0x%02x\n", reg);
#endif
        return 0xffff;
    }
}

static uint16_t tsc2102_control_register_read(
                struct tsc210x_state_s *s, int reg)
{
    switch (reg) {
    case 0x00:	/* TSC ADC */
        return (s->pressure << 15) | ((!s->busy) << 14) |
                (s->nextfunction << 10) | (s->nextprecision << 8) | s->filter; 

    case 0x01:	/* Status */
        return (s->pin_func << 14) | ((!s->enabled) << 13) |
                (s->host_mode << 12) | ((!!s->dav) << 11) | s->dav;

    case 0x03:	/* Reference */
        return s->ref;

    case 0x04:	/* Reset */
        return 0xffff;

    case 0x05:	/* Configuration */
        return s->timing;

    default:
#ifdef TSC_VERBOSE
        fprintf(stderr, "tsc2102_control_register_read: "
                        "no such register: 0x%02x\n", reg);
#endif
        return 0xffff;
    }
}

static uint16_t tsc2102_audio_register_read(struct tsc210x_state_s *s, int reg)
{
    int l_ch, r_ch;
    uint16_t val;

    switch (reg) {
    case 0x00:	/* Audio Control 1 */
        return s->audio_ctrl1;

    case 0x01:
        return 0xff00;

    case 0x02:	/* DAC Volume Control */
        return s->volume;

    case 0x03:
        return 0x8b00;

    case 0x04:	/* Audio Control 2 */
        l_ch = 1;
        r_ch = 1;
        if (s->softstep && !(s->dac_power & (1 << 10))) {
            l_ch = (qemu_get_clock(vm_clock) >
                            s->volume_change + TSC_SOFTSTEP_DELAY);
            r_ch = (qemu_get_clock(vm_clock) >
                            s->volume_change + TSC_SOFTSTEP_DELAY);
        }

        return s->audio_ctrl2 | (l_ch << 3) | (r_ch << 2);

    case 0x05:	/* Stereo DAC Power Control */
        return 0x2aa0 | s->dac_power |
                (((s->dac_power & (1 << 10)) &&
                  (qemu_get_clock(vm_clock) >
                   s->powerdown + TSC_POWEROFF_DELAY)) << 6);

    case 0x06:	/* Audio Control 3 */
        val = s->audio_ctrl3 | 0x0001;
        s->audio_ctrl3 &= 0xff3f;
        return val;

    case 0x07:	/* LCH_BASS_BOOST_N0 */
    case 0x08:	/* LCH_BASS_BOOST_N1 */
    case 0x09:	/* LCH_BASS_BOOST_N2 */
    case 0x0a:	/* LCH_BASS_BOOST_N3 */
    case 0x0b:	/* LCH_BASS_BOOST_N4 */
    case 0x0c:	/* LCH_BASS_BOOST_N5 */
    case 0x0d:	/* LCH_BASS_BOOST_D1 */
    case 0x0e:	/* LCH_BASS_BOOST_D2 */
    case 0x0f:	/* LCH_BASS_BOOST_D4 */
    case 0x10:	/* LCH_BASS_BOOST_D5 */
    case 0x11:	/* RCH_BASS_BOOST_N0 */
    case 0x12:	/* RCH_BASS_BOOST_N1 */
    case 0x13:	/* RCH_BASS_BOOST_N2 */
    case 0x14:	/* RCH_BASS_BOOST_N3 */
    case 0x15:	/* RCH_BASS_BOOST_N4 */
    case 0x16:	/* RCH_BASS_BOOST_N5 */
    case 0x17:	/* RCH_BASS_BOOST_D1 */
    case 0x18:	/* RCH_BASS_BOOST_D2 */
    case 0x19:	/* RCH_BASS_BOOST_D4 */
    case 0x1a:	/* RCH_BASS_BOOST_D5 */
        return s->filter_data[reg - 0x07];

    case 0x1b:	/* PLL Programmability 1 */
        return s->pll[0];

    case 0x1c:	/* PLL Programmability 2 */
        return s->pll[1];

    case 0x1d:	/* Audio Control 4 */
        return (!s->softstep) << 14;

    default:
#ifdef TSC_VERBOSE
        fprintf(stderr, "tsc2102_audio_register_read: "
                        "no such register: 0x%02x\n", reg);
#endif
        return 0xffff;
    }
}

static void tsc2102_data_register_write(
                struct tsc210x_state_s *s, int reg, uint16_t value)
{
    switch (reg) {
    case 0x00:	/* X */
    case 0x01:	/* Y */
    case 0x02:	/* Z1 */
    case 0x03:	/* Z2 */
    case 0x05:	/* BAT1 */
    case 0x06:	/* BAT2 */
    case 0x07:	/* AUX1 */
    case 0x08:	/* AUX2 */
    case 0x09:	/* TEMP1 */
    case 0x0a:	/* TEMP2 */
        return;

    default:
#ifdef TSC_VERBOSE
        fprintf(stderr, "tsc2102_data_register_write: "
                        "no such register: 0x%02x\n", reg);
#endif
    }
}

static void tsc2102_control_register_write(
                struct tsc210x_state_s *s, int reg, uint16_t value)
{
    switch (reg) {
    case 0x00:	/* TSC ADC */
        s->host_mode = value >> 15;
        s->enabled = !(value & 0x4000);
        if (s->busy && !s->enabled)
            qemu_del_timer(s->timer);
        s->busy &= s->enabled;
        s->nextfunction = (value >> 10) & 0xf;
        s->nextprecision = (value >> 8) & 3;
        s->filter = value & 0xff;
        return;

    case 0x01:	/* Status */
        s->pin_func = value >> 14;
        return;

    case 0x03:	/* Reference */
        s->ref = value & 0x1f;
        return;

    case 0x04:	/* Reset */
        if (value == 0xbb00) {
            if (s->busy)
                qemu_del_timer(s->timer);
            tsc210x_reset(s);
#ifdef TSC_VERBOSE
        } else {
            fprintf(stderr, "tsc2102_control_register_write: "
                            "wrong value written into RESET\n");
#endif
        }
        return;

    case 0x05:	/* Configuration */
        s->timing = value & 0x3f;
#ifdef TSC_VERBOSE
        if (value & ~0x3f)
            fprintf(stderr, "tsc2102_control_register_write: "
                            "wrong value written into CONFIG\n");
#endif
        return;

    default:
#ifdef TSC_VERBOSE
        fprintf(stderr, "tsc2102_control_register_write: "
                        "no such register: 0x%02x\n", reg);
#endif
    }
}

static void tsc2102_audio_register_write(
                struct tsc210x_state_s *s, int reg, uint16_t value)
{
    switch (reg) {
    case 0x00:	/* Audio Control 1 */
        s->audio_ctrl1 = value & 0x0f3f;
#ifdef TSC_VERBOSE
        if ((value & ~0x0f3f) || ((value & 7) != ((value >> 3) & 7)))
            fprintf(stderr, "tsc2102_audio_register_write: "
                            "wrong value written into Audio 1\n");
#endif
        return;

    case 0x01:
#ifdef TSC_VERBOSE
        if (value != 0xff00)
            fprintf(stderr, "tsc2102_audio_register_write: "
                            "wrong value written into reg 0x01\n");
#endif
        return;

    case 0x02:	/* DAC Volume Control */
        s->volume = value;
        s->volume_change = qemu_get_clock(vm_clock);
        return;

    case 0x03:
#ifdef TSC_VERBOSE
        if (value != 0x8b00)
            fprintf(stderr, "tsc2102_audio_register_write: "
                            "wrong value written into reg 0x03\n");
#endif
        return;

    case 0x04:	/* Audio Control 2 */
        s->audio_ctrl2 = value & 0xf7f2;
#ifdef TSC_VERBOSE
        if (value & ~0xf7fd)
            fprintf(stderr, "tsc2102_audio_register_write: "
                            "wrong value written into Audio 2\n");
#endif
        return;

    case 0x05:	/* Stereo DAC Power Control */
        if ((value & ~s->dac_power) & (1 << 10))
            s->powerdown = qemu_get_clock(vm_clock);

        s->dac_power = value & 0x9543;
#ifdef TSC_VERBOSE
        if ((value & ~0x9543) != 0x2aa0)
            fprintf(stderr, "tsc2102_audio_register_write: "
                            "wrong value written into Power\n");
#endif
        return;

    case 0x06:	/* Audio Control 3 */
        s->audio_ctrl3 &= 0x00c0;
        s->audio_ctrl3 |= value & 0xf800;
#ifdef TSC_VERBOSE
        if (value & ~0xf8c7)
            fprintf(stderr, "tsc2102_audio_register_write: "
                            "wrong value written into Audio 3\n");
#endif
        return;

    case 0x07:	/* LCH_BASS_BOOST_N0 */
    case 0x08:	/* LCH_BASS_BOOST_N1 */
    case 0x09:	/* LCH_BASS_BOOST_N2 */
    case 0x0a:	/* LCH_BASS_BOOST_N3 */
    case 0x0b:	/* LCH_BASS_BOOST_N4 */
    case 0x0c:	/* LCH_BASS_BOOST_N5 */
    case 0x0d:	/* LCH_BASS_BOOST_D1 */
    case 0x0e:	/* LCH_BASS_BOOST_D2 */
    case 0x0f:	/* LCH_BASS_BOOST_D4 */
    case 0x10:	/* LCH_BASS_BOOST_D5 */
    case 0x11:	/* RCH_BASS_BOOST_N0 */
    case 0x12:	/* RCH_BASS_BOOST_N1 */
    case 0x13:	/* RCH_BASS_BOOST_N2 */
    case 0x14:	/* RCH_BASS_BOOST_N3 */
    case 0x15:	/* RCH_BASS_BOOST_N4 */
    case 0x16:	/* RCH_BASS_BOOST_N5 */
    case 0x17:	/* RCH_BASS_BOOST_D1 */
    case 0x18:	/* RCH_BASS_BOOST_D2 */
    case 0x19:	/* RCH_BASS_BOOST_D4 */
    case 0x1a:	/* RCH_BASS_BOOST_D5 */
        s->filter_data[reg - 0x07] = value;
        return;

    case 0x1b:	/* PLL Programmability 1 */
        s->pll[0] = value & 0xfffc;
#ifdef TSC_VERBOSE
        if (value & ~0xfffc)
            fprintf(stderr, "tsc2102_audio_register_write: "
                            "wrong value written into PLL 1\n");
#endif
        return;

    case 0x1c:	/* PLL Programmability 2 */
        s->pll[1] = value & 0xfffc;
#ifdef TSC_VERBOSE
        if (value & ~0xfffc)
            fprintf(stderr, "tsc2102_audio_register_write: "
                            "wrong value written into PLL 2\n");
#endif
        return;

    case 0x1d:	/* Audio Control 4 */
        s->softstep = !(value & 0x4000);
#ifdef TSC_VERBOSE
        if (value & ~0x4000)
            fprintf(stderr, "tsc2102_audio_register_write: "
                            "wrong value written into Audio 4\n");
#endif
        return;

    default:
#ifdef TSC_VERBOSE
        fprintf(stderr, "tsc2102_audio_register_write: "
                        "no such register: 0x%02x\n", reg);
#endif
    }
}

/* This handles most of the chip logic.  */
static void tsc210x_pin_update(struct tsc210x_state_s *s)
{
    int64_t expires;
    int pin_state;

    switch (s->pin_func) {
    case 0:
        pin_state = s->pressure;
        break;
    case 1:
        pin_state = !!s->dav;
        break;
    case 2:
    default:
        pin_state = s->pressure && !s->dav;
    }

    if (!s->enabled)
        pin_state = 0;

    if (pin_state != s->irq) {
        s->irq = pin_state;
        qemu_set_irq(s->pint, !s->irq);
    }

    switch (s->nextfunction) {
    case TSC_MODE_XY_SCAN:
    case TSC_MODE_XYZ_SCAN:
        if (!s->pressure)
            return;
        break;

    case TSC_MODE_X:
    case TSC_MODE_Y:
    case TSC_MODE_Z:
        if (!s->pressure)
            return;
        /* Fall through */
    case TSC_MODE_BAT1:
    case TSC_MODE_BAT2:
    case TSC_MODE_AUX:
    case TSC_MODE_TEMP1:
    case TSC_MODE_TEMP2:
        if (s->dav)
            s->enabled = 0;
        break;

    case TSC_MODE_AUX_SCAN:
    case TSC_MODE_PORT_SCAN:
        break;

    case TSC_MODE_NO_SCAN:
    case TSC_MODE_XX_DRV:
    case TSC_MODE_YY_DRV:
    case TSC_MODE_YX_DRV:
    default:
        return;
    }

    if (!s->enabled || s->busy)
        return;

    s->busy = 1;
    s->precision = s->nextprecision;
    s->function = s->nextfunction;
    expires = qemu_get_clock(vm_clock) + (ticks_per_sec >> 10);
    qemu_mod_timer(s->timer, expires);
}

static uint16_t tsc210x_read(struct tsc210x_state_s *s)
{
    uint16_t ret = 0x0000;

    if (!s->command)
        fprintf(stderr, "tsc210x_read: SPI underrun!\n");

    switch (s->page) {
    case TSC_DATA_REGISTERS_PAGE:
        ret = tsc2102_data_register_read(s, s->offset);
        break;
    case TSC_CONTROL_REGISTERS_PAGE:
        ret = tsc2102_control_register_read(s, s->offset);
        break;
    case TSC_AUDIO_REGISTERS_PAGE:
        ret = tsc2102_audio_register_read(s, s->offset);
        break;
    default:
        cpu_abort(cpu_single_env, "tsc210x_read: wrong memory page\n");
    }

    tsc210x_pin_update(s);

    /* Allow sequential reads.  */
    s->offset ++;
    s->state = 0;
    return ret;
}

static void tsc210x_write(struct tsc210x_state_s *s, uint16_t value)
{
    /*
     * This is a two-state state machine for reading
     * command and data every second time.
     */
    if (!s->state) {
        s->command = value >> 15;
        s->page = (value >> 11) & 0x0f;
        s->offset = (value >> 5) & 0x3f;
        s->state = 1;
    } else {
        if (s->command)
            fprintf(stderr, "tsc210x_write: SPI overrun!\n");
        else
            switch (s->page) {
            case TSC_DATA_REGISTERS_PAGE:
                tsc2102_data_register_write(s, s->offset, value);
                break;
            case TSC_CONTROL_REGISTERS_PAGE:
                tsc2102_control_register_write(s, s->offset, value);
                break;
            case TSC_AUDIO_REGISTERS_PAGE:
                tsc2102_audio_register_write(s, s->offset, value);
                break;
            default:
                cpu_abort(cpu_single_env,
                                "tsc210x_write: wrong memory page\n");
            }

        tsc210x_pin_update(s);
        s->state = 0;
    }
}

static void tsc210x_timer_tick(void *opaque)
{
    struct tsc210x_state_s *s = opaque;

    /* Timer ticked -- a set of conversions has been finished.  */

    if (!s->busy)
        return;

    s->busy = 0;
    s->dav |= mode_regs[s->function];
    tsc210x_pin_update(s);
}

static void tsc210x_touchscreen_event(void *opaque,
                int x, int y, int z, int buttons_state)
{
    struct tsc210x_state_s *s = opaque;
    int p = s->pressure;

    if (buttons_state) {
        s->x = x;
        s->y = y;
    }
    s->pressure = !!buttons_state;

    /*
     * Note: We would get better responsiveness in the guest by
     * signaling TS events immediately, but for now we simulate
     * the first conversion delay for sake of correctness.
     */
    if (p != s->pressure)
        tsc210x_pin_update(s);
}

static void tsc210x_save(QEMUFile *f, void *opaque)
{
    struct tsc210x_state_s *s = (struct tsc210x_state_s *) opaque;
    int64_t now = qemu_get_clock(vm_clock);
    int i;

    qemu_put_be16(f, s->x);
    qemu_put_be16(f, s->y);
    qemu_put_byte(f, s->pressure);

    qemu_put_byte(f, s->state);
    qemu_put_byte(f, s->page);
    qemu_put_byte(f, s->offset);
    qemu_put_byte(f, s->command);

    qemu_put_byte(f, s->irq);
    qemu_put_be16s(f, &s->dav);

    qemu_put_timer(f, s->timer);
    qemu_put_byte(f, s->enabled);
    qemu_put_byte(f, s->host_mode);
    qemu_put_byte(f, s->function);
    qemu_put_byte(f, s->nextfunction);
    qemu_put_byte(f, s->precision);
    qemu_put_byte(f, s->nextprecision);
    qemu_put_byte(f, s->filter);
    qemu_put_byte(f, s->pin_func);
    qemu_put_byte(f, s->ref);
    qemu_put_byte(f, s->timing);
    qemu_put_be32(f, s->noise);

    qemu_put_be16s(f, &s->audio_ctrl1);
    qemu_put_be16s(f, &s->audio_ctrl2);
    qemu_put_be16s(f, &s->audio_ctrl3);
    qemu_put_be16s(f, &s->pll[0]);
    qemu_put_be16s(f, &s->pll[1]);
    qemu_put_be16s(f, &s->volume);
    qemu_put_be64(f, (uint64_t) (s->volume_change - now));
    qemu_put_be64(f, (uint64_t) (s->powerdown - now));
    qemu_put_byte(f, s->softstep);
    qemu_put_be16s(f, &s->dac_power);

    for (i = 0; i < 0x14; i ++)
        qemu_put_be16s(f, &s->filter_data[i]);
}

static int tsc210x_load(QEMUFile *f, void *opaque, int version_id)
{
    struct tsc210x_state_s *s = (struct tsc210x_state_s *) opaque;
    int64_t now = qemu_get_clock(vm_clock);
    int i;

    s->x = qemu_get_be16(f);
    s->y = qemu_get_be16(f);
    s->pressure = qemu_get_byte(f);

    s->state = qemu_get_byte(f);
    s->page = qemu_get_byte(f);
    s->offset = qemu_get_byte(f);
    s->command = qemu_get_byte(f);

    s->irq = qemu_get_byte(f);
    qemu_get_be16s(f, &s->dav);

    qemu_get_timer(f, s->timer);
    s->enabled = qemu_get_byte(f);
    s->host_mode = qemu_get_byte(f);
    s->function = qemu_get_byte(f);
    s->nextfunction = qemu_get_byte(f);
    s->precision = qemu_get_byte(f);
    s->nextprecision = qemu_get_byte(f);
    s->filter = qemu_get_byte(f);
    s->pin_func = qemu_get_byte(f);
    s->ref = qemu_get_byte(f);
    s->timing = qemu_get_byte(f);
    s->noise = qemu_get_be32(f);

    qemu_get_be16s(f, &s->audio_ctrl1);
    qemu_get_be16s(f, &s->audio_ctrl2);
    qemu_get_be16s(f, &s->audio_ctrl3);
    qemu_get_be16s(f, &s->pll[0]);
    qemu_get_be16s(f, &s->pll[1]);
    qemu_get_be16s(f, &s->volume);
    s->volume_change = (int64_t) qemu_get_be64(f) + now;
    s->powerdown = (int64_t) qemu_get_be64(f) + now;
    s->softstep = qemu_get_byte(f);
    qemu_get_be16s(f, &s->dac_power);

    for (i = 0; i < 0x14; i ++)
        qemu_get_be16s(f, &s->filter_data[i]);

    s->busy = qemu_timer_pending(s->timer);
    qemu_set_irq(s->pint, !s->irq);

    return 0;
}

static int tsc2102_iid = 0;

struct uwire_slave_s *tsc2102_init(qemu_irq pint)
{
    struct tsc210x_state_s *s;

    s = (struct tsc210x_state_s *)
            qemu_mallocz(sizeof(struct tsc210x_state_s));
    memset(s, 0, sizeof(struct tsc210x_state_s));
    s->x = 160;
    s->y = 160;
    s->pressure = 0;
    s->precision = s->nextprecision = 0;
    s->timer = qemu_new_timer(vm_clock, tsc210x_timer_tick, s);
    s->pint = pint;

    s->chip.opaque = s;
    s->chip.send = (void *) tsc210x_write;
    s->chip.receive = (void *) tsc210x_read;

    tsc210x_reset(s);

    qemu_add_mouse_event_handler(tsc210x_touchscreen_event, s, 1,
                    "QEMU TSC2102-driven Touchscreen");

    qemu_register_reset((void *) tsc210x_reset, s);
    register_savevm("tsc2102", tsc2102_iid ++, 0,
                    tsc210x_save, tsc210x_load, s);

    return &s->chip;
}
