/*
 * Calculate Error-correcting Codes. Used by NAND Flash controllers
 * (not by NAND chips).
 *
 * Copyright (c) 2006 Openedhand Ltd.
 * Written by Andrzej Zaborowski <balrog@zabor.org>
 *
 * This code is licensed under the GNU GPL v2.
 */

#include "hw.h"
#include "flash.h"

/*
 * Pre-calculated 256-way 1 byte column parity.  Table borrowed from Linux.
 */
static const uint8_t nand_ecc_precalc_table[] = {
    0x00, 0x55, 0x56, 0x03, 0x59, 0x0c, 0x0f, 0x5a,
    0x5a, 0x0f, 0x0c, 0x59, 0x03, 0x56, 0x55, 0x00,
    0x65, 0x30, 0x33, 0x66, 0x3c, 0x69, 0x6a, 0x3f,
    0x3f, 0x6a, 0x69, 0x3c, 0x66, 0x33, 0x30, 0x65,
    0x66, 0x33, 0x30, 0x65, 0x3f, 0x6a, 0x69, 0x3c,
    0x3c, 0x69, 0x6a, 0x3f, 0x65, 0x30, 0x33, 0x66,
    0x03, 0x56, 0x55, 0x00, 0x5a, 0x0f, 0x0c, 0x59,
    0x59, 0x0c, 0x0f, 0x5a, 0x00, 0x55, 0x56, 0x03,
    0x69, 0x3c, 0x3f, 0x6a, 0x30, 0x65, 0x66, 0x33,
    0x33, 0x66, 0x65, 0x30, 0x6a, 0x3f, 0x3c, 0x69,
    0x0c, 0x59, 0x5a, 0x0f, 0x55, 0x00, 0x03, 0x56,
    0x56, 0x03, 0x00, 0x55, 0x0f, 0x5a, 0x59, 0x0c,
    0x0f, 0x5a, 0x59, 0x0c, 0x56, 0x03, 0x00, 0x55,
    0x55, 0x00, 0x03, 0x56, 0x0c, 0x59, 0x5a, 0x0f,
    0x6a, 0x3f, 0x3c, 0x69, 0x33, 0x66, 0x65, 0x30,
    0x30, 0x65, 0x66, 0x33, 0x69, 0x3c, 0x3f, 0x6a,
    0x6a, 0x3f, 0x3c, 0x69, 0x33, 0x66, 0x65, 0x30,
    0x30, 0x65, 0x66, 0x33, 0x69, 0x3c, 0x3f, 0x6a,
    0x0f, 0x5a, 0x59, 0x0c, 0x56, 0x03, 0x00, 0x55,
    0x55, 0x00, 0x03, 0x56, 0x0c, 0x59, 0x5a, 0x0f,
    0x0c, 0x59, 0x5a, 0x0f, 0x55, 0x00, 0x03, 0x56,
    0x56, 0x03, 0x00, 0x55, 0x0f, 0x5a, 0x59, 0x0c,
    0x69, 0x3c, 0x3f, 0x6a, 0x30, 0x65, 0x66, 0x33,
    0x33, 0x66, 0x65, 0x30, 0x6a, 0x3f, 0x3c, 0x69,
    0x03, 0x56, 0x55, 0x00, 0x5a, 0x0f, 0x0c, 0x59,
    0x59, 0x0c, 0x0f, 0x5a, 0x00, 0x55, 0x56, 0x03,
    0x66, 0x33, 0x30, 0x65, 0x3f, 0x6a, 0x69, 0x3c,
    0x3c, 0x69, 0x6a, 0x3f, 0x65, 0x30, 0x33, 0x66,
    0x65, 0x30, 0x33, 0x66, 0x3c, 0x69, 0x6a, 0x3f,
    0x3f, 0x6a, 0x69, 0x3c, 0x66, 0x33, 0x30, 0x65,
    0x00, 0x55, 0x56, 0x03, 0x59, 0x0c, 0x0f, 0x5a,
    0x5a, 0x0f, 0x0c, 0x59, 0x03, 0x56, 0x55, 0x00,
};

/* Update ECC parity count.  */
uint8_t ecc_digest(ECCState *s, uint8_t sample)
{
    uint8_t idx = nand_ecc_precalc_table[sample];

    s->cp ^= idx & 0x3f;
    if (idx & 0x40) {
        s->lp[0] ^= ~s->count;
        s->lp[1] ^= s->count;
    }
    s->count ++;

    return sample;
}

/* Reinitialise the counters.  */
void ecc_reset(ECCState *s)
{
    s->lp[0] = 0x0000;
    s->lp[1] = 0x0000;
    s->cp = 0x00;
    s->count = 0;
}

/* Save/restore */
void ecc_put(QEMUFile *f, ECCState *s)
{
    qemu_put_8s(f, &s->cp);
    qemu_put_be16s(f, &s->lp[0]);
    qemu_put_be16s(f, &s->lp[1]);
    qemu_put_be16s(f, &s->count);
}

void ecc_get(QEMUFile *f, ECCState *s)
{
    qemu_get_8s(f, &s->cp);
    qemu_get_be16s(f, &s->lp[0]);
    qemu_get_be16s(f, &s->lp[1]);
    qemu_get_be16s(f, &s->count);
}
