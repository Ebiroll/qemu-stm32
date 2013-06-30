/*
 * QEMU IDE Emulation: MacIO support.
 *
 * Copyright (c) 2003 Fabrice Bellard
 * Copyright (c) 2006 Openedhand Ltd.
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
#include "hw/hw.h"
#include "hw/ppc/mac.h"
#include "hw/ppc/mac_dbdma.h"
#include "block/block.h"
#include "sysemu/dma.h"

#include <hw/ide/internal.h>

/* debug MACIO */
// #define DEBUG_MACIO

#ifdef DEBUG_MACIO
static const int debug_macio = 1;
#else
static const int debug_macio = 0;
#endif

#define MACIO_DPRINTF(fmt, ...) do { \
        if (debug_macio) { \
            printf(fmt , ## __VA_ARGS__); \
        } \
    } while (0)


/***********************************************************/
/* MacIO based PowerPC IDE */

#define MACIO_PAGE_SIZE 4096

static void pmac_ide_atapi_transfer_cb(void *opaque, int ret)
{
    DBDMA_io *io = opaque;
    MACIOIDEState *m = io->opaque;
    IDEState *s = idebus_active_if(&m->bus);

    if (ret < 0) {
        m->aiocb = NULL;
        qemu_sglist_destroy(&s->sg);
        ide_atapi_io_error(s, ret);
        goto done;
    }

    MACIO_DPRINTF("io_buffer_size = %#x\n", s->io_buffer_size);

    if (s->io_buffer_size > 0) {
        m->aiocb = NULL;
        qemu_sglist_destroy(&s->sg);

        s->packet_transfer_size -= s->io_buffer_size;

        s->io_buffer_index += s->io_buffer_size;
        s->lba += s->io_buffer_index >> 11;
        s->io_buffer_index &= 0x7ff;
    }

    if (s->packet_transfer_size <= 0) {
        MACIO_DPRINTF("end of transfer\n");
        ide_atapi_cmd_ok(s);
    }

    if (io->len == 0) {
        MACIO_DPRINTF("end of DMA\n");
        goto done;
    }

    /* launch next transfer */

    MACIO_DPRINTF("io->len = %#x\n", io->len);

    s->io_buffer_size = io->len;

    qemu_sglist_init(&s->sg, DEVICE(m), io->len / MACIO_PAGE_SIZE + 1,
                     &address_space_memory);
    qemu_sglist_add(&s->sg, io->addr, io->len);
    io->addr += io->len;
    io->len = 0;

    MACIO_DPRINTF("sector_num=%d size=%d, cmd_cmd=%d\n",
                  (s->lba << 2) + (s->io_buffer_index >> 9),
                  s->packet_transfer_size, s->dma_cmd);

    m->aiocb = dma_bdrv_read(s->bs, &s->sg,
                             (int64_t)(s->lba << 2) + (s->io_buffer_index >> 9),
                             pmac_ide_atapi_transfer_cb, io);
    return;

done:
    MACIO_DPRINTF("done DMA\n");
    bdrv_acct_done(s->bs, &s->acct);
    io->dma_end(opaque);
}

static void pmac_ide_transfer_cb(void *opaque, int ret)
{
    DBDMA_io *io = opaque;
    MACIOIDEState *m = io->opaque;
    IDEState *s = idebus_active_if(&m->bus);
    int n;
    int64_t sector_num;

    if (ret < 0) {
        MACIO_DPRINTF("DMA error\n");
        m->aiocb = NULL;
        qemu_sglist_destroy(&s->sg);
        ide_dma_error(s);
        goto done;
    }

    sector_num = ide_get_sector(s);
    MACIO_DPRINTF("io_buffer_size = %#x\n", s->io_buffer_size);
    if (s->io_buffer_size > 0) {
        m->aiocb = NULL;
        qemu_sglist_destroy(&s->sg);
        n = (s->io_buffer_size + 0x1ff) >> 9;
        sector_num += n;
        ide_set_sector(s, sector_num);
        s->nsector -= n;
    }

    if (s->nsector == 0) {
        MACIO_DPRINTF("end of transfer\n");
        s->status = READY_STAT | SEEK_STAT;
        ide_set_irq(s->bus);
    }

    if (io->len == 0) {
        MACIO_DPRINTF("end of DMA\n");
        goto done;
    }

    /* launch next transfer */

    s->io_buffer_index = 0;
    s->io_buffer_size = io->len;

    MACIO_DPRINTF("io->len = %#x\n", io->len);

    qemu_sglist_init(&s->sg, DEVICE(m), io->len / MACIO_PAGE_SIZE + 1,
                     &address_space_memory);
    qemu_sglist_add(&s->sg, io->addr, io->len);
    io->addr += io->len;
    io->len = 0;

    MACIO_DPRINTF("sector_num=%" PRId64 " n=%d, nsector=%d, cmd_cmd=%d\n",
                  sector_num, n, s->nsector, s->dma_cmd);

    switch (s->dma_cmd) {
    case IDE_DMA_READ:
        m->aiocb = dma_bdrv_read(s->bs, &s->sg, sector_num,
                                 pmac_ide_transfer_cb, io);
        break;
    case IDE_DMA_WRITE:
        m->aiocb = dma_bdrv_write(s->bs, &s->sg, sector_num,
                                  pmac_ide_transfer_cb, io);
        break;
    case IDE_DMA_TRIM:
        m->aiocb = dma_bdrv_io(s->bs, &s->sg, sector_num,
                               ide_issue_trim, pmac_ide_transfer_cb, io,
                               DMA_DIRECTION_TO_DEVICE);
        break;
    }
    return;

done:
    if (s->dma_cmd == IDE_DMA_READ || s->dma_cmd == IDE_DMA_WRITE) {
        bdrv_acct_done(s->bs, &s->acct);
    }
    io->dma_end(io);
}

static void pmac_ide_transfer(DBDMA_io *io)
{
    MACIOIDEState *m = io->opaque;
    IDEState *s = idebus_active_if(&m->bus);

    MACIO_DPRINTF("\n");

    s->io_buffer_size = 0;
    if (s->drive_kind == IDE_CD) {
        bdrv_acct_start(s->bs, &s->acct, io->len, BDRV_ACCT_READ);
        pmac_ide_atapi_transfer_cb(io, 0);
        return;
    }

    switch (s->dma_cmd) {
    case IDE_DMA_READ:
        bdrv_acct_start(s->bs, &s->acct, io->len, BDRV_ACCT_READ);
        break;
    case IDE_DMA_WRITE:
        bdrv_acct_start(s->bs, &s->acct, io->len, BDRV_ACCT_WRITE);
        break;
    default:
        break;
    }

    pmac_ide_transfer_cb(io, 0);
}

static void pmac_ide_flush(DBDMA_io *io)
{
    MACIOIDEState *m = io->opaque;

    if (m->aiocb) {
        bdrv_drain_all();
    }
}

/* PowerMac IDE memory IO */
static void pmac_ide_writeb (void *opaque,
                             hwaddr addr, uint32_t val)
{
    MACIOIDEState *d = opaque;

    addr = (addr & 0xFFF) >> 4;
    switch (addr) {
    case 1 ... 7:
        ide_ioport_write(&d->bus, addr, val);
        break;
    case 8:
    case 22:
        ide_cmd_write(&d->bus, 0, val);
        break;
    default:
        break;
    }
}

static uint32_t pmac_ide_readb (void *opaque,hwaddr addr)
{
    uint8_t retval;
    MACIOIDEState *d = opaque;

    addr = (addr & 0xFFF) >> 4;
    switch (addr) {
    case 1 ... 7:
        retval = ide_ioport_read(&d->bus, addr);
        break;
    case 8:
    case 22:
        retval = ide_status_read(&d->bus, 0);
        break;
    default:
        retval = 0xFF;
        break;
    }
    return retval;
}

static void pmac_ide_writew (void *opaque,
                             hwaddr addr, uint32_t val)
{
    MACIOIDEState *d = opaque;

    addr = (addr & 0xFFF) >> 4;
    val = bswap16(val);
    if (addr == 0) {
        ide_data_writew(&d->bus, 0, val);
    }
}

static uint32_t pmac_ide_readw (void *opaque,hwaddr addr)
{
    uint16_t retval;
    MACIOIDEState *d = opaque;

    addr = (addr & 0xFFF) >> 4;
    if (addr == 0) {
        retval = ide_data_readw(&d->bus, 0);
    } else {
        retval = 0xFFFF;
    }
    retval = bswap16(retval);
    return retval;
}

static void pmac_ide_writel (void *opaque,
                             hwaddr addr, uint32_t val)
{
    MACIOIDEState *d = opaque;

    addr = (addr & 0xFFF) >> 4;
    val = bswap32(val);
    if (addr == 0) {
        ide_data_writel(&d->bus, 0, val);
    }
}

static uint32_t pmac_ide_readl (void *opaque,hwaddr addr)
{
    uint32_t retval;
    MACIOIDEState *d = opaque;

    addr = (addr & 0xFFF) >> 4;
    if (addr == 0) {
        retval = ide_data_readl(&d->bus, 0);
    } else {
        retval = 0xFFFFFFFF;
    }
    retval = bswap32(retval);
    return retval;
}

static const MemoryRegionOps pmac_ide_ops = {
    .old_mmio = {
        .write = {
            pmac_ide_writeb,
            pmac_ide_writew,
            pmac_ide_writel,
        },
        .read = {
            pmac_ide_readb,
            pmac_ide_readw,
            pmac_ide_readl,
        },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_pmac = {
    .name = "ide",
    .version_id = 3,
    .minimum_version_id = 0,
    .minimum_version_id_old = 0,
    .fields      = (VMStateField []) {
        VMSTATE_IDE_BUS(bus, MACIOIDEState),
        VMSTATE_IDE_DRIVES(bus.ifs, MACIOIDEState),
        VMSTATE_END_OF_LIST()
    }
};

static void macio_ide_reset(DeviceState *dev)
{
    MACIOIDEState *d = MACIO_IDE(dev);

    ide_bus_reset(&d->bus);
}

static int ide_nop(IDEDMA *dma)
{
    return 0;
}

static int ide_nop_int(IDEDMA *dma, int x)
{
    return 0;
}

static void ide_nop_restart(void *opaque, int x, RunState y)
{
}

static void ide_dbdma_start(IDEDMA *dma, IDEState *s,
                            BlockDriverCompletionFunc *cb)
{
    MACIOIDEState *m = container_of(dma, MACIOIDEState, dma);

    MACIO_DPRINTF("\n");
    DBDMA_kick(m->dbdma);
}

static const IDEDMAOps dbdma_ops = {
    .start_dma      = ide_dbdma_start,
    .start_transfer = ide_nop,
    .prepare_buf    = ide_nop_int,
    .rw_buf         = ide_nop_int,
    .set_unit       = ide_nop_int,
    .add_status     = ide_nop_int,
    .set_inactive   = ide_nop,
    .restart_cb     = ide_nop_restart,
    .reset          = ide_nop,
};

static void macio_ide_realizefn(DeviceState *dev, Error **errp)
{
    MACIOIDEState *s = MACIO_IDE(dev);

    ide_init2(&s->bus, s->irq);

    /* Register DMA callbacks */
    s->dma.ops = &dbdma_ops;
    s->bus.dma = &s->dma;
}

static void macio_ide_initfn(Object *obj)
{
    SysBusDevice *d = SYS_BUS_DEVICE(obj);
    MACIOIDEState *s = MACIO_IDE(obj);

    ide_bus_new(&s->bus, DEVICE(obj), 0, 2);
    memory_region_init_io(&s->mem, obj, &pmac_ide_ops, s, "pmac-ide", 0x1000);
    sysbus_init_mmio(d, &s->mem);
    sysbus_init_irq(d, &s->irq);
    sysbus_init_irq(d, &s->dma_irq);
}

static void macio_ide_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = macio_ide_realizefn;
    dc->reset = macio_ide_reset;
    dc->vmsd = &vmstate_pmac;
}

static const TypeInfo macio_ide_type_info = {
    .name = TYPE_MACIO_IDE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MACIOIDEState),
    .instance_init = macio_ide_initfn,
    .class_init = macio_ide_class_init,
};

static void macio_ide_register_types(void)
{
    type_register_static(&macio_ide_type_info);
}

/* hd_table must contain 2 block drivers */
void macio_ide_init_drives(MACIOIDEState *s, DriveInfo **hd_table)
{
    int i;

    for (i = 0; i < 2; i++) {
        if (hd_table[i]) {
            ide_create_drive(&s->bus, i, hd_table[i]);
        }
    }
}

void macio_ide_register_dma(MACIOIDEState *s, void *dbdma, int channel)
{
    s->dbdma = dbdma;
    DBDMA_register_channel(dbdma, channel, s->dma_irq,
                           pmac_ide_transfer, pmac_ide_flush, s);
}

type_init(macio_ide_register_types)
