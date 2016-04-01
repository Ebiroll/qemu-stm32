/*
 * Terminal 3270 implementation
 *
 * Copyright 2017 IBM Corp.
 *
 * Authors: Yang Chen <bjcyang@linux.vnet.ibm.com>
 *          Jing Liu <liujbjl@linux.vnet.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or (at
 * your option) any later version. See the COPYING file in the top-level
 * directory.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "sysemu/char.h"
#include "hw/s390x/3270-ccw.h"

/* Enough spaces for different window sizes. */
#define INPUT_BUFFER_SIZE  1000
/*
 * 1 for header, 1024*2 for datastream, 2 for tail
 * Reserve enough spaces for telnet IAC escape.
 */
#define OUTPUT_BUFFER_SIZE 2051

typedef struct Terminal3270 {
    EmulatedCcw3270Device cdev;
    CharBackend chr;
    uint8_t inv[INPUT_BUFFER_SIZE];
    uint8_t outv[OUTPUT_BUFFER_SIZE];
    int in_len;
    int out_len;
    bool handshake_done;
} Terminal3270;

#define TYPE_TERMINAL_3270 "x-terminal3270"
#define TERMINAL_3270(obj) \
        OBJECT_CHECK(Terminal3270, (obj), TYPE_TERMINAL_3270)

static int terminal_can_read(void *opaque)
{
    Terminal3270 *t = opaque;

    return INPUT_BUFFER_SIZE - t->in_len;
}

/*
 * Protocol handshake done,
 * signal guest by an unsolicited DE irq.
 */
static void TN3270_handshake_done(Terminal3270 *t)
{
    CcwDevice *ccw_dev = CCW_DEVICE(t);
    SubchDev *sch = ccw_dev->sch;

    t->handshake_done = true;
    sch->curr_status.scsw.dstat = SCSW_DSTAT_DEVICE_END;
    css_conditional_io_interrupt(sch);
}

/*
 * Receive inbound data from socket.
 * For data given to guest, drop the data boundary IAC, IAC_EOR.
 * TODO:
 * Using "Reset" key on x3270 may result multiple commands in one packet.
 * This usually happens when the user meets a poor traffic of the network.
 * As of now, for such case, we simply terminate the connection,
 * and we should come back here later with a better solution.
 */
static void terminal_read(void *opaque, const uint8_t *buf, int size)
{
    Terminal3270 *t = opaque;
    CcwDevice *ccw_dev = CCW_DEVICE(t);
    SubchDev *sch = ccw_dev->sch;
    int end;

    assert(size <= (INPUT_BUFFER_SIZE - t->in_len));

    memcpy(&t->inv[t->in_len], buf, size);
    t->in_len += size;
    if (t->in_len < 2) {
        return;
    }

    if (!t->handshake_done) {
        /*
         * Receiving Terminal Type is the last step of handshake.
         * The data format: IAC SB Terminal-Type IS <terminal type> IAC SE
         * The code for Terminal-Type is 0x18, for IS is 0.
         * Simply check the data format and mark handshake_done.
         */
        if (t->in_len > 6 && t->inv[2] == 0x18 && t->inv[3] == 0x0 &&
            t->inv[t->in_len - 2] == IAC && t->inv[t->in_len - 1] == IAC_SE) {
            TN3270_handshake_done(t);
            t->in_len = 0;
        }
        return;
    }

    for (end = 0; end < t->in_len - 1; end++) {
        if (t->inv[end] == IAC && t->inv[end + 1] == IAC_EOR) {
            break;
        }
    }
    if (end == t->in_len - 2) {
        /* Data is valid for consuming. */
        t->in_len -= 2;
        sch->curr_status.scsw.dstat = SCSW_DSTAT_ATTENTION;
        css_conditional_io_interrupt(sch);
    } else if (end < t->in_len - 2) {
        /* "Reset" key is used. */
        qemu_chr_fe_disconnect(&t->chr);
    } else {
        /* Gathering data. */
        return;
    }
}

static void terminal_init(EmulatedCcw3270Device *dev, Error **errp)
{
    Terminal3270 *t = TERMINAL_3270(dev);
    static bool terminal_available;

    if (terminal_available) {
        error_setg(errp, "Multiple 3270 terminals are not supported.");
        return;
    }
    terminal_available = true;
    qemu_chr_fe_set_handlers(&t->chr, terminal_can_read,
                             terminal_read, NULL, t, NULL, true);
}

static int read_payload_3270(EmulatedCcw3270Device *dev, uint32_t cda,
                             uint16_t count)
{
    Terminal3270 *t = TERMINAL_3270(dev);
    int len;

    len = MIN(count, t->in_len);
    cpu_physical_memory_write(cda, t->inv, len);
    t->in_len -= len;

    return len;
}

/* TN3270 uses binary transmission, which needs escape IAC to IAC IAC */
static int insert_IAC_escape_char(uint8_t *outv, int out_len)
{
    int IAC_num = 0, new_out_len, i, j;

    for (i = 0; i < out_len; i++) {
        if (outv[i] == IAC) {
            IAC_num++;
        }
    }
    if (IAC_num == 0) {
        return out_len;
    }
    new_out_len = out_len + IAC_num;
    for (i = out_len - 1, j = new_out_len - 1; j > i && i >= 0; i--, j--) {
        outv[j] = outv[i];
        if (outv[i] == IAC) {
            outv[--j] = IAC;
        }
    }
    return new_out_len;
}

/*
 * Write 3270 outbound to socket.
 * Return the count of 3270 data field if succeeded, zero if failed.
 */
static int write_payload_3270(EmulatedCcw3270Device *dev, uint8_t cmd,
                              uint32_t cda, uint16_t count)
{
    Terminal3270 *t = TERMINAL_3270(dev);
    int retval = 0;

    assert(count <= (OUTPUT_BUFFER_SIZE - 3) / 2);

    if (!t->handshake_done) {
        /*
         * Before having finished 3270 negotiation,
         * sending outbound data is prohibited.
         */
        return 0;
    }
    if (!qemu_chr_fe_get_driver(&t->chr)) {
        /* We just say we consumed all data if there's no backend. */
        return count;
    }
    t->outv[0] = cmd;
    cpu_physical_memory_read(cda, &t->outv[1], count);
    t->out_len = count + 1;

    t->out_len = insert_IAC_escape_char(t->outv, t->out_len);
    t->outv[t->out_len++] = IAC;
    t->outv[t->out_len++] = IAC_EOR;

    retval = qemu_chr_fe_write_all(&t->chr, t->outv, t->out_len);
    return (retval <= 0) ? 0 : (retval - 3);
}

static Property terminal_properties[] = {
    DEFINE_PROP_CHR("chardev", Terminal3270, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void terminal_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    EmulatedCcw3270Class *ck = EMULATED_CCW_3270_CLASS(klass);

    dc->props = terminal_properties;
    ck->init = terminal_init;
    ck->read_payload_3270 = read_payload_3270;
    ck->write_payload_3270 = write_payload_3270;
}

static const TypeInfo ccw_terminal_info = {
    .name = TYPE_TERMINAL_3270,
    .parent = TYPE_EMULATED_CCW_3270,
    .instance_size = sizeof(Terminal3270),
    .class_init = terminal_class_init,
    .class_size = sizeof(EmulatedCcw3270Class),
};

static void register_types(void)
{
    type_register_static(&ccw_terminal_info);
}

type_init(register_types)
