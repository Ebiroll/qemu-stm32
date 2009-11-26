/*
 * SCSI Device emulation
 *
 * Copyright (c) 2006 CodeSourcery.
 * Based on code by Fabrice Bellard
 *
 * Written by Paul Brook
 *
 * This code is licenced under the LGPL.
 *
 * Note that this file only handles the SCSI architecture model and device
 * commands.  Emulation of interface/link layer protocols is handled by
 * the host adapter emulator.
 */

#include <qemu-common.h>
#include <sysemu.h>
//#define DEBUG_SCSI

#ifdef DEBUG_SCSI
#define DPRINTF(fmt, ...) \
do { printf("scsi-disk: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#endif

#define BADF(fmt, ...) \
do { fprintf(stderr, "scsi-disk: " fmt , ## __VA_ARGS__); } while (0)

#include "qemu-common.h"
#include "block.h"
#include "scsi.h"
#include "scsi-defs.h"

#define SCSI_DMA_BUF_SIZE    131072
#define SCSI_MAX_INQUIRY_LEN 256

#define SCSI_REQ_STATUS_RETRY 0x01

typedef struct SCSIDiskState SCSIDiskState;

typedef struct SCSIDiskReq {
    SCSIRequest req;
    /* ??? We should probably keep track of whether the data transfer is
       a read or a write.  Currently we rely on the host getting it right.  */
    /* Both sector and sector_count are in terms of qemu 512 byte blocks.  */
    uint64_t sector;
    uint32_t sector_count;
    struct iovec iov;
    QEMUIOVector qiov;
    uint32_t status;
} SCSIDiskReq;

struct SCSIDiskState
{
    SCSIDevice qdev;
    DriveInfo *dinfo;
    /* The qemu block layer uses a fixed 512 byte sector size.
       This is the number of 512 byte blocks in a single scsi sector.  */
    int cluster_size;
    uint64_t max_lba;
    int sense;
    char drive_serial_str[21];
    QEMUBH *bh;
};

static SCSIDiskReq *scsi_new_request(SCSIDevice *d, uint32_t tag, uint32_t lun)
{
    SCSIRequest *req;
    SCSIDiskReq *r;

    req = scsi_req_alloc(sizeof(SCSIDiskReq), d, tag, lun);
    r = DO_UPCAST(SCSIDiskReq, req, req);
    r->iov.iov_base = qemu_memalign(512, SCSI_DMA_BUF_SIZE);
    return r;
}

static void scsi_remove_request(SCSIDiskReq *r)
{
    qemu_free(r->iov.iov_base);
    scsi_req_free(&r->req);
}

static SCSIDiskReq *scsi_find_request(SCSIDiskState *s, uint32_t tag)
{
    return DO_UPCAST(SCSIDiskReq, req, scsi_req_find(&s->qdev, tag));
}

/* Helper function for command completion.  */
static void scsi_command_complete(SCSIDiskReq *r, int status, int sense)
{
    SCSIDiskState *s = DO_UPCAST(SCSIDiskState, qdev, r->req.dev);
    uint32_t tag;
    DPRINTF("Command complete tag=0x%x status=%d sense=%d\n",
            r->req.tag, status, sense);
    s->sense = sense;
    tag = r->req.tag;
    r->req.bus->complete(r->req.bus, SCSI_REASON_DONE, tag, status);
    scsi_remove_request(r);
}

/* Cancel a pending data transfer.  */
static void scsi_cancel_io(SCSIDevice *d, uint32_t tag)
{
    SCSIDiskState *s = DO_UPCAST(SCSIDiskState, qdev, d);
    SCSIDiskReq *r;
    DPRINTF("Cancel tag=0x%x\n", tag);
    r = scsi_find_request(s, tag);
    if (r) {
        if (r->req.aiocb)
            bdrv_aio_cancel(r->req.aiocb);
        r->req.aiocb = NULL;
        scsi_remove_request(r);
    }
}

static void scsi_read_complete(void * opaque, int ret)
{
    SCSIDiskReq *r = (SCSIDiskReq *)opaque;

    if (ret) {
        DPRINTF("IO error\n");
        r->req.bus->complete(r->req.bus, SCSI_REASON_DATA, r->req.tag, 0);
        scsi_command_complete(r, CHECK_CONDITION, NO_SENSE);
        return;
    }
    DPRINTF("Data ready tag=0x%x len=%" PRId64 "\n", r->req.tag, r->iov.iov_len);

    r->req.bus->complete(r->req.bus, SCSI_REASON_DATA, r->req.tag, r->iov.iov_len);
}

/* Read more data from scsi device into buffer.  */
static void scsi_read_data(SCSIDevice *d, uint32_t tag)
{
    SCSIDiskState *s = DO_UPCAST(SCSIDiskState, qdev, d);
    SCSIDiskReq *r;
    uint32_t n;

    r = scsi_find_request(s, tag);
    if (!r) {
        BADF("Bad read tag 0x%x\n", tag);
        /* ??? This is the wrong error.  */
        scsi_command_complete(r, CHECK_CONDITION, HARDWARE_ERROR);
        return;
    }
    if (r->sector_count == (uint32_t)-1) {
        DPRINTF("Read buf_len=%" PRId64 "\n", r->iov.iov_len);
        r->sector_count = 0;
        r->req.bus->complete(r->req.bus, SCSI_REASON_DATA, r->req.tag, r->iov.iov_len);
        return;
    }
    DPRINTF("Read sector_count=%d\n", r->sector_count);
    if (r->sector_count == 0) {
        scsi_command_complete(r, GOOD, NO_SENSE);
        return;
    }

    n = r->sector_count;
    if (n > SCSI_DMA_BUF_SIZE / 512)
        n = SCSI_DMA_BUF_SIZE / 512;

    r->iov.iov_len = n * 512;
    qemu_iovec_init_external(&r->qiov, &r->iov, 1);
    r->req.aiocb = bdrv_aio_readv(s->dinfo->bdrv, r->sector, &r->qiov, n,
                              scsi_read_complete, r);
    if (r->req.aiocb == NULL)
        scsi_command_complete(r, CHECK_CONDITION, HARDWARE_ERROR);
    r->sector += n;
    r->sector_count -= n;
}

static int scsi_handle_write_error(SCSIDiskReq *r, int error)
{
    SCSIDiskState *s = DO_UPCAST(SCSIDiskState, qdev, r->req.dev);
    BlockInterfaceErrorAction action = drive_get_onerror(s->dinfo->bdrv);

    if (action == BLOCK_ERR_IGNORE)
        return 0;

    if ((error == ENOSPC && action == BLOCK_ERR_STOP_ENOSPC)
            || action == BLOCK_ERR_STOP_ANY) {
        r->status |= SCSI_REQ_STATUS_RETRY;
        vm_stop(0);
    } else {
        scsi_command_complete(r, CHECK_CONDITION,
                HARDWARE_ERROR);
    }

    return 1;
}

static void scsi_write_complete(void * opaque, int ret)
{
    SCSIDiskReq *r = (SCSIDiskReq *)opaque;
    uint32_t len;
    uint32_t n;

    r->req.aiocb = NULL;

    if (ret) {
        if (scsi_handle_write_error(r, -ret))
            return;
    }

    n = r->iov.iov_len / 512;
    r->sector += n;
    r->sector_count -= n;
    if (r->sector_count == 0) {
        scsi_command_complete(r, GOOD, NO_SENSE);
    } else {
        len = r->sector_count * 512;
        if (len > SCSI_DMA_BUF_SIZE) {
            len = SCSI_DMA_BUF_SIZE;
        }
        r->iov.iov_len = len;
        DPRINTF("Write complete tag=0x%x more=%d\n", r->req.tag, len);
        r->req.bus->complete(r->req.bus, SCSI_REASON_DATA, r->req.tag, len);
    }
}

static void scsi_write_request(SCSIDiskReq *r)
{
    SCSIDiskState *s = DO_UPCAST(SCSIDiskState, qdev, r->req.dev);
    uint32_t n;

    n = r->iov.iov_len / 512;
    if (n) {
        qemu_iovec_init_external(&r->qiov, &r->iov, 1);
        r->req.aiocb = bdrv_aio_writev(s->dinfo->bdrv, r->sector, &r->qiov, n,
                                   scsi_write_complete, r);
        if (r->req.aiocb == NULL)
            scsi_command_complete(r, CHECK_CONDITION,
                                  HARDWARE_ERROR);
    } else {
        /* Invoke completion routine to fetch data from host.  */
        scsi_write_complete(r, 0);
    }
}

/* Write data to a scsi device.  Returns nonzero on failure.
   The transfer may complete asynchronously.  */
static int scsi_write_data(SCSIDevice *d, uint32_t tag)
{
    SCSIDiskState *s = DO_UPCAST(SCSIDiskState, qdev, d);
    SCSIDiskReq *r;

    DPRINTF("Write data tag=0x%x\n", tag);
    r = scsi_find_request(s, tag);
    if (!r) {
        BADF("Bad write tag 0x%x\n", tag);
        scsi_command_complete(r, CHECK_CONDITION, HARDWARE_ERROR);
        return 1;
    }

    if (r->req.aiocb)
        BADF("Data transfer already in progress\n");

    scsi_write_request(r);

    return 0;
}

static void scsi_dma_restart_bh(void *opaque)
{
    SCSIDiskState *s = opaque;
    SCSIRequest *req;
    SCSIDiskReq *r;

    qemu_bh_delete(s->bh);
    s->bh = NULL;

    QTAILQ_FOREACH(req, &s->qdev.requests, next) {
        r = DO_UPCAST(SCSIDiskReq, req, req);
        if (r->status & SCSI_REQ_STATUS_RETRY) {
            r->status &= ~SCSI_REQ_STATUS_RETRY;
            scsi_write_request(r); 
        }
    }
}

static void scsi_dma_restart_cb(void *opaque, int running, int reason)
{
    SCSIDiskState *s = opaque;

    if (!running)
        return;

    if (!s->bh) {
        s->bh = qemu_bh_new(scsi_dma_restart_bh, s);
        qemu_bh_schedule(s->bh);
    }
}

/* Return a pointer to the data buffer.  */
static uint8_t *scsi_get_buf(SCSIDevice *d, uint32_t tag)
{
    SCSIDiskState *s = DO_UPCAST(SCSIDiskState, qdev, d);
    SCSIDiskReq *r;

    r = scsi_find_request(s, tag);
    if (!r) {
        BADF("Bad buffer tag 0x%x\n", tag);
        return NULL;
    }
    return (uint8_t *)r->iov.iov_base;
}

/* Execute a scsi command.  Returns the length of the data expected by the
   command.  This will be Positive for data transfers from the device
   (eg. disk reads), negative for transfers to the device (eg. disk writes),
   and zero if the command does not transfer any data.  */

static int32_t scsi_send_command(SCSIDevice *d, uint32_t tag,
                                 uint8_t *buf, int lun)
{
    SCSIDiskState *s = DO_UPCAST(SCSIDiskState, qdev, d);
    uint64_t nb_sectors;
    uint64_t lba;
    uint32_t len;
    int cmdlen;
    int is_write;
    uint8_t command;
    uint8_t *outbuf;
    SCSIDiskReq *r;

    command = buf[0];
    r = scsi_find_request(s, tag);
    if (r) {
        BADF("Tag 0x%x already in use\n", tag);
        scsi_cancel_io(d, tag);
    }
    /* ??? Tags are not unique for different luns.  We only implement a
       single lun, so this should not matter.  */
    r = scsi_new_request(d, tag, lun);
    outbuf = (uint8_t *)r->iov.iov_base;
    is_write = 0;
    DPRINTF("Command: lun=%d tag=0x%x data=0x%02x", lun, tag, buf[0]);
    switch (command >> 5) {
    case 0:
        lba = (uint64_t) buf[3] | ((uint64_t) buf[2] << 8) |
              (((uint64_t) buf[1] & 0x1f) << 16);
        len = buf[4];
        cmdlen = 6;
        break;
    case 1:
    case 2:
        lba = (uint64_t) buf[5] | ((uint64_t) buf[4] << 8) |
              ((uint64_t) buf[3] << 16) | ((uint64_t) buf[2] << 24);
        len = buf[8] | (buf[7] << 8);
        cmdlen = 10;
        break;
    case 4:
        lba = (uint64_t) buf[9] | ((uint64_t) buf[8] << 8) |
              ((uint64_t) buf[7] << 16) | ((uint64_t) buf[6] << 24) |
              ((uint64_t) buf[5] << 32) | ((uint64_t) buf[4] << 40) |
              ((uint64_t) buf[3] << 48) | ((uint64_t) buf[2] << 56);
        len = buf[13] | (buf[12] << 8) | (buf[11] << 16) | (buf[10] << 24);
        cmdlen = 16;
        break;
    case 5:
        lba = (uint64_t) buf[5] | ((uint64_t) buf[4] << 8) |
              ((uint64_t) buf[3] << 16) | ((uint64_t) buf[2] << 24);
        len = buf[9] | (buf[8] << 8) | (buf[7] << 16) | (buf[6] << 24);
        cmdlen = 12;
        break;
    default:
        BADF("Unsupported command length, command %x\n", command);
        goto fail;
    }
#ifdef DEBUG_SCSI
    {
        int i;
        for (i = 1; i < cmdlen; i++) {
            printf(" 0x%02x", buf[i]);
        }
        printf("\n");
    }
#endif
    if (lun || buf[1] >> 5) {
        /* Only LUN 0 supported.  */
        DPRINTF("Unimplemented LUN %d\n", lun ? lun : buf[1] >> 5);
        if (command != 0x03 && command != 0x12) /* REQUEST SENSE and INQUIRY */
            goto fail;
    }
    switch (command) {
    case 0x0:
	DPRINTF("Test Unit Ready\n");
        if (!bdrv_is_inserted(s->dinfo->bdrv))
            goto notready;
	break;
    case 0x03:
        DPRINTF("Request Sense (len %d)\n", len);
        if (len < 4)
            goto fail;
        memset(outbuf, 0, 4);
        r->iov.iov_len = 4;
        if (s->sense == NOT_READY && len >= 18) {
            memset(outbuf, 0, 18);
            r->iov.iov_len = 18;
            outbuf[7] = 10;
            /* asc 0x3a, ascq 0: Medium not present */
            outbuf[12] = 0x3a;
            outbuf[13] = 0;
        }
        outbuf[0] = 0xf0;
        outbuf[1] = 0;
        outbuf[2] = s->sense;
        break;
    case 0x12:
        DPRINTF("Inquiry (len %d)\n", len);
        if (buf[1] & 0x2) {
            /* Command support data - optional, not implemented */
            BADF("optional INQUIRY command support request not implemented\n");
            goto fail;
        }
        else if (buf[1] & 0x1) {
            /* Vital product data */
            uint8_t page_code = buf[2];
            if (len < 4) {
                BADF("Error: Inquiry (EVPD[%02X]) buffer size %d is "
                     "less than 4\n", page_code, len);
                goto fail;
            }

            switch (page_code) {
                case 0x00:
                    {
                        /* Supported page codes, mandatory */
                        DPRINTF("Inquiry EVPD[Supported pages] "
                                "buffer size %d\n", len);

                        r->iov.iov_len = 0;

                        if (bdrv_get_type_hint(s->dinfo->bdrv) == BDRV_TYPE_CDROM) {
                            outbuf[r->iov.iov_len++] = 5;
                        } else {
                            outbuf[r->iov.iov_len++] = 0;
                        }

                        outbuf[r->iov.iov_len++] = 0x00; // this page
                        outbuf[r->iov.iov_len++] = 0x00;
                        outbuf[r->iov.iov_len++] = 3;    // number of pages
                        outbuf[r->iov.iov_len++] = 0x00; // list of supported pages (this page)
                        outbuf[r->iov.iov_len++] = 0x80; // unit serial number
                        outbuf[r->iov.iov_len++] = 0x83; // device identification
                    }
                    break;
                case 0x80:
                    {
                        int l;

                        /* Device serial number, optional */
                        if (len < 4) {
                            BADF("Error: EVPD[Serial number] Inquiry buffer "
                                 "size %d too small, %d needed\n", len, 4);
                            goto fail;
                        }

                        DPRINTF("Inquiry EVPD[Serial number] buffer size %d\n", len);
                        l = MIN(len, strlen(s->drive_serial_str));

                        r->iov.iov_len = 0;

                        /* Supported page codes */
                        if (bdrv_get_type_hint(s->dinfo->bdrv) == BDRV_TYPE_CDROM) {
                            outbuf[r->iov.iov_len++] = 5;
                        } else {
                            outbuf[r->iov.iov_len++] = 0;
                        }

                        outbuf[r->iov.iov_len++] = 0x80; // this page
                        outbuf[r->iov.iov_len++] = 0x00;
                        outbuf[r->iov.iov_len++] = l;
                        memcpy(&outbuf[r->iov.iov_len], s->drive_serial_str, l);
                        r->iov.iov_len += l;
                    }

                    break;
                case 0x83:
                    {
                        /* Device identification page, mandatory */
                        int max_len = 255 - 8;
                        int id_len = strlen(bdrv_get_device_name(s->dinfo->bdrv));
                        if (id_len > max_len)
                            id_len = max_len;

                        DPRINTF("Inquiry EVPD[Device identification] "
                                "buffer size %d\n", len);
                        r->iov.iov_len = 0;
                        if (bdrv_get_type_hint(s->dinfo->bdrv) == BDRV_TYPE_CDROM) {
                            outbuf[r->iov.iov_len++] = 5;
                        } else {
                            outbuf[r->iov.iov_len++] = 0;
                        }

                        outbuf[r->iov.iov_len++] = 0x83; // this page
                        outbuf[r->iov.iov_len++] = 0x00;
                        outbuf[r->iov.iov_len++] = 3 + id_len;

                        outbuf[r->iov.iov_len++] = 0x2; // ASCII
                        outbuf[r->iov.iov_len++] = 0;   // not officially assigned
                        outbuf[r->iov.iov_len++] = 0;   // reserved
                        outbuf[r->iov.iov_len++] = id_len; // length of data following

                        memcpy(&outbuf[r->iov.iov_len],
                               bdrv_get_device_name(s->dinfo->bdrv), id_len);
                        r->iov.iov_len += id_len;
                    }
                    break;
                default:
                    BADF("Error: unsupported Inquiry (EVPD[%02X]) "
                         "buffer size %d\n", page_code, len);
                    goto fail;
            }
            /* done with EVPD */
            break;
        }
        else {
            /* Standard INQUIRY data */
            if (buf[2] != 0) {
                BADF("Error: Inquiry (STANDARD) page or code "
                     "is non-zero [%02X]\n", buf[2]);
                goto fail;
            }

            /* PAGE CODE == 0 */
            if (len < 5) {
                BADF("Error: Inquiry (STANDARD) buffer size %d "
                     "is less than 5\n", len);
                goto fail;
            }

            if (len < 36) {
                BADF("Error: Inquiry (STANDARD) buffer size %d "
                     "is less than 36 (TODO: only 5 required)\n", len);
            }
        }

        if(len > SCSI_MAX_INQUIRY_LEN)
            len = SCSI_MAX_INQUIRY_LEN;

        memset(outbuf, 0, len);

        if (lun || buf[1] >> 5) {
            outbuf[0] = 0x7f;	/* LUN not supported */
	} else if (bdrv_get_type_hint(s->dinfo->bdrv) == BDRV_TYPE_CDROM) {
	    outbuf[0] = 5;
            outbuf[1] = 0x80;
	    memcpy(&outbuf[16], "QEMU CD-ROM    ", 16);
	} else {
	    outbuf[0] = 0;
	    memcpy(&outbuf[16], "QEMU HARDDISK  ", 16);
	}
	memcpy(&outbuf[8], "QEMU   ", 8);
        memcpy(&outbuf[32], QEMU_VERSION, 4);
        /* Identify device as SCSI-3 rev 1.
           Some later commands are also implemented. */
	outbuf[2] = 3;
	outbuf[3] = 2; /* Format 2 */
	outbuf[4] = len - 5; /* Additional Length = (Len - 1) - 4 */
        /* Sync data transfer and TCQ.  */
        outbuf[7] = 0x10 | (r->req.bus->tcq ? 0x02 : 0);
	r->iov.iov_len = len;
	break;
    case 0x16:
        DPRINTF("Reserve(6)\n");
        if (buf[1] & 1)
            goto fail;
        break;
    case 0x17:
        DPRINTF("Release(6)\n");
        if (buf[1] & 1)
            goto fail;
        break;
    case 0x1a:
    case 0x5a:
        {
            uint8_t *p;
            int page;
            int dbd;
            
            dbd = buf[1]  & 0x8;
            page = buf[2] & 0x3f;
            DPRINTF("Mode Sense (page %d, len %d)\n", page, len);
            p = outbuf;
            memset(p, 0, 4);
            outbuf[1] = 0; /* Default media type.  */
            outbuf[3] = 0; /* Block descriptor length.  */
            if (bdrv_get_type_hint(s->dinfo->bdrv) == BDRV_TYPE_CDROM ||
                bdrv_is_read_only(s->dinfo->bdrv)) {
                outbuf[2] = 0x80; /* Readonly.  */
            }
            p += 4;
            bdrv_get_geometry(s->dinfo->bdrv, &nb_sectors);
            if ((~dbd) & nb_sectors) {
                nb_sectors /= s->cluster_size;
                nb_sectors--;
                if (nb_sectors > 0xffffff)
                    nb_sectors = 0xffffff;
                outbuf[3] = 8; /* Block descriptor length  */
                p[0] = 0; /* media density code */
                p[1] = (nb_sectors >> 16) & 0xff;
                p[2] = (nb_sectors >> 8) & 0xff;
                p[3] = nb_sectors & 0xff;
                p[4] = 0; /* reserved */
                p[5] = 0; /* bytes 5-7 are the sector size in bytes */
                p[6] = s->cluster_size * 2;
                p[7] = 0;
                p += 8;
            }

            if (page == 4) {
                int cylinders, heads, secs;

                /* Rigid disk device geometry page. */
                p[0] = 4;
                p[1] = 0x16;
                /* if a geometry hint is available, use it */
                bdrv_get_geometry_hint(s->dinfo->bdrv, &cylinders, &heads, &secs);
                p[2] = (cylinders >> 16) & 0xff;
                p[3] = (cylinders >> 8) & 0xff;
                p[4] = cylinders & 0xff;
                p[5] = heads & 0xff;
                /* Write precomp start cylinder, disabled */
                p[6] = (cylinders >> 16) & 0xff;
                p[7] = (cylinders >> 8) & 0xff;
                p[8] = cylinders & 0xff;
                /* Reduced current start cylinder, disabled */
                p[9] = (cylinders >> 16) & 0xff;
                p[10] = (cylinders >> 8) & 0xff;
                p[11] = cylinders & 0xff;
                /* Device step rate [ns], 200ns */
                p[12] = 0;
                p[13] = 200;
                /* Landing zone cylinder */
                p[14] = 0xff;
                p[15] =  0xff;
                p[16] = 0xff;
                /* Medium rotation rate [rpm], 5400 rpm */
                p[20] = (5400 >> 8) & 0xff;
                p[21] = 5400 & 0xff;
                p += 0x16;
            } else if (page == 5) {
                int cylinders, heads, secs;

                /* Flexible disk device geometry page. */
                p[0] = 5;
                p[1] = 0x1e;
                /* Transfer rate [kbit/s], 5Mbit/s */
                p[2] = 5000 >> 8;
                p[3] = 5000 & 0xff;
                /* if a geometry hint is available, use it */
                bdrv_get_geometry_hint(s->dinfo->bdrv, &cylinders, &heads, &secs);
                p[4] = heads & 0xff;
                p[5] = secs & 0xff;
                p[6] = s->cluster_size * 2;
                p[8] = (cylinders >> 8) & 0xff;
                p[9] = cylinders & 0xff;
                /* Write precomp start cylinder, disabled */
                p[10] = (cylinders >> 8) & 0xff;
                p[11] = cylinders & 0xff;
                /* Reduced current start cylinder, disabled */
                p[12] = (cylinders >> 8) & 0xff;
                p[13] = cylinders & 0xff;
                /* Device step rate [100us], 100us */
                p[14] = 0;
                p[15] = 1;
                /* Device step pulse width [us], 1us */
                p[16] = 1;
                /* Device head settle delay [100us], 100us */
                p[17] = 0;
                p[18] = 1;
                /* Motor on delay [0.1s], 0.1s */
                p[19] = 1;
                /* Motor off delay [0.1s], 0.1s */
                p[20] = 1;
                /* Medium rotation rate [rpm], 5400 rpm */
                p[28] = (5400 >> 8) & 0xff;
                p[29] = 5400 & 0xff;
                p += 0x1e;
            } else if ((page == 8 || page == 0x3f)) {
                /* Caching page.  */
                memset(p,0,20);
                p[0] = 8;
                p[1] = 0x12;
                if (bdrv_enable_write_cache(s->dinfo->bdrv)) {
                     p[2] = 4; /* WCE */
                }
                p += 20;
            }
            if ((page == 0x3f || page == 0x2a)
                    && (bdrv_get_type_hint(s->dinfo->bdrv) == BDRV_TYPE_CDROM)) {
                /* CD Capabilities and Mechanical Status page. */
                p[0] = 0x2a;
                p[1] = 0x14;
                p[2] = 3; // CD-R & CD-RW read
                p[3] = 0; // Writing not supported
                p[4] = 0x7f; /* Audio, composite, digital out,
                                         mode 2 form 1&2, multi session */
                p[5] = 0xff; /* CD DA, DA accurate, RW supported,
                                         RW corrected, C2 errors, ISRC,
                                         UPC, Bar code */
                p[6] = 0x2d | (bdrv_is_locked(s->dinfo->bdrv)? 2 : 0);
                /* Locking supported, jumper present, eject, tray */
                p[7] = 0; /* no volume & mute control, no
                                      changer */
                p[8] = (50 * 176) >> 8; // 50x read speed
                p[9] = (50 * 176) & 0xff;
                p[10] = 0 >> 8; // No volume
                p[11] = 0 & 0xff;
                p[12] = 2048 >> 8; // 2M buffer
                p[13] = 2048 & 0xff;
                p[14] = (16 * 176) >> 8; // 16x read speed current
                p[15] = (16 * 176) & 0xff;
                p[18] = (16 * 176) >> 8; // 16x write speed
                p[19] = (16 * 176) & 0xff;
                p[20] = (16 * 176) >> 8; // 16x write speed current
                p[21] = (16 * 176) & 0xff;
                p += 22;
            }
            r->iov.iov_len = p - outbuf;
            outbuf[0] = r->iov.iov_len - 4;
            if (r->iov.iov_len > len)
                r->iov.iov_len = len;
        }
        break;
    case 0x1b:
        DPRINTF("Start Stop Unit\n");
        if (bdrv_get_type_hint(s->dinfo->bdrv) == BDRV_TYPE_CDROM &&
            (buf[4] & 2))
            /* load/eject medium */
            bdrv_eject(s->dinfo->bdrv, !(buf[4] & 1));
	break;
    case 0x1e:
        DPRINTF("Prevent Allow Medium Removal (prevent = %d)\n", buf[4] & 3);
        bdrv_set_locked(s->dinfo->bdrv, buf[4] & 1);
	break;
    case 0x25:
	DPRINTF("Read Capacity\n");
        /* The normal LEN field for this command is zero.  */
	memset(outbuf, 0, 8);
	bdrv_get_geometry(s->dinfo->bdrv, &nb_sectors);
        nb_sectors /= s->cluster_size;
        /* Returned value is the address of the last sector.  */
        if (nb_sectors) {
            nb_sectors--;
            /* Remember the new size for read/write sanity checking. */
            s->max_lba = nb_sectors;
            /* Clip to 2TB, instead of returning capacity modulo 2TB. */
            if (nb_sectors > UINT32_MAX)
                nb_sectors = UINT32_MAX;
            outbuf[0] = (nb_sectors >> 24) & 0xff;
            outbuf[1] = (nb_sectors >> 16) & 0xff;
            outbuf[2] = (nb_sectors >> 8) & 0xff;
            outbuf[3] = nb_sectors & 0xff;
            outbuf[4] = 0;
            outbuf[5] = 0;
            outbuf[6] = s->cluster_size * 2;
            outbuf[7] = 0;
            r->iov.iov_len = 8;
        } else {
        notready:
            scsi_command_complete(r, CHECK_CONDITION, NOT_READY);
            return 0;
        }
	break;
    case 0x08:
    case 0x28:
    case 0x88:
        DPRINTF("Read (sector %" PRId64 ", count %d)\n", lba, len);
        if (lba > s->max_lba)
            goto illegal_lba;
        r->sector = lba * s->cluster_size;
        r->sector_count = len * s->cluster_size;
        break;
    case 0x0a:
    case 0x2a:
    case 0x8a:
        DPRINTF("Write (sector %" PRId64 ", count %d)\n", lba, len);
        if (lba > s->max_lba)
            goto illegal_lba;
        r->sector = lba * s->cluster_size;
        r->sector_count = len * s->cluster_size;
        is_write = 1;
        break;
    case 0x35:
        DPRINTF("Synchronise cache (sector %" PRId64 ", count %d)\n", lba, len);
        bdrv_flush(s->dinfo->bdrv);
        break;
    case 0x43:
        {
            int start_track, format, msf, toclen;

            msf = buf[1] & 2;
            format = buf[2] & 0xf;
            start_track = buf[6];
            bdrv_get_geometry(s->dinfo->bdrv, &nb_sectors);
            DPRINTF("Read TOC (track %d format %d msf %d)\n", start_track, format, msf >> 1);
            nb_sectors /= s->cluster_size;
            switch(format) {
            case 0:
                toclen = cdrom_read_toc(nb_sectors, outbuf, msf, start_track);
                break;
            case 1:
                /* multi session : only a single session defined */
                toclen = 12;
                memset(outbuf, 0, 12);
                outbuf[1] = 0x0a;
                outbuf[2] = 0x01;
                outbuf[3] = 0x01;
                break;
            case 2:
                toclen = cdrom_read_toc_raw(nb_sectors, outbuf, msf, start_track);
                break;
            default:
                goto error_cmd;
            }
            if (toclen > 0) {
                if (len > toclen)
                  len = toclen;
                r->iov.iov_len = len;
                break;
            }
        error_cmd:
            DPRINTF("Read TOC error\n");
            goto fail;
        }
    case 0x46:
        DPRINTF("Get Configuration (rt %d, maxlen %d)\n", buf[1] & 3, len);
        memset(outbuf, 0, 8);
        /* ??? This should probably return much more information.  For now
           just return the basic header indicating the CD-ROM profile.  */
        outbuf[7] = 8; // CD-ROM
        r->iov.iov_len = 8;
        break;
    case 0x56:
        DPRINTF("Reserve(10)\n");
        if (buf[1] & 3)
            goto fail;
        break;
    case 0x57:
        DPRINTF("Release(10)\n");
        if (buf[1] & 3)
            goto fail;
        break;
    case 0x9e:
        /* Service Action In subcommands. */
        if ((buf[1] & 31) == 0x10) {
            DPRINTF("SAI READ CAPACITY(16)\n");
            memset(outbuf, 0, len);
            bdrv_get_geometry(s->dinfo->bdrv, &nb_sectors);
            nb_sectors /= s->cluster_size;
            /* Returned value is the address of the last sector.  */
            if (nb_sectors) {
                nb_sectors--;
                /* Remember the new size for read/write sanity checking. */
                s->max_lba = nb_sectors;
                outbuf[0] = (nb_sectors >> 56) & 0xff;
                outbuf[1] = (nb_sectors >> 48) & 0xff;
                outbuf[2] = (nb_sectors >> 40) & 0xff;
                outbuf[3] = (nb_sectors >> 32) & 0xff;
                outbuf[4] = (nb_sectors >> 24) & 0xff;
                outbuf[5] = (nb_sectors >> 16) & 0xff;
                outbuf[6] = (nb_sectors >> 8) & 0xff;
                outbuf[7] = nb_sectors & 0xff;
                outbuf[8] = 0;
                outbuf[9] = 0;
                outbuf[10] = s->cluster_size * 2;
                outbuf[11] = 0;
                /* Protection, exponent and lowest lba field left blank. */
                r->iov.iov_len = len;
            } else {
                scsi_command_complete(r, CHECK_CONDITION, NOT_READY);
                return 0;
            }
            break;
        }
        DPRINTF("Unsupported Service Action In\n");
        goto fail;
    case 0xa0:
        DPRINTF("Report LUNs (len %d)\n", len);
        if (len < 16)
            goto fail;
        memset(outbuf, 0, 16);
        outbuf[3] = 8;
        r->iov.iov_len = 16;
        break;
    case 0x2f:
        DPRINTF("Verify (sector %" PRId64 ", count %d)\n", lba, len);
        break;
    default:
	DPRINTF("Unknown SCSI command (%2.2x)\n", buf[0]);
    fail:
        scsi_command_complete(r, CHECK_CONDITION, ILLEGAL_REQUEST);
	return 0;
    illegal_lba:
        scsi_command_complete(r, CHECK_CONDITION, HARDWARE_ERROR);
        return 0;
    }
    if (r->sector_count == 0 && r->iov.iov_len == 0) {
        scsi_command_complete(r, GOOD, NO_SENSE);
    }
    len = r->sector_count * 512 + r->iov.iov_len;
    if (is_write) {
        return -len;
    } else {
        if (!r->sector_count)
            r->sector_count = -1;
        return len;
    }
}

static void scsi_destroy(SCSIDevice *dev)
{
    SCSIDiskState *s = DO_UPCAST(SCSIDiskState, qdev, dev);
    SCSIDiskReq *r;

    while (!QTAILQ_EMPTY(&s->qdev.requests)) {
        r = DO_UPCAST(SCSIDiskReq, req, QTAILQ_FIRST(&s->qdev.requests));
        scsi_remove_request(r);
    }
    drive_uninit(s->dinfo);
}

static int scsi_disk_initfn(SCSIDevice *dev)
{
    SCSIDiskState *s = DO_UPCAST(SCSIDiskState, qdev, dev);
    uint64_t nb_sectors;

    if (!s->dinfo || !s->dinfo->bdrv) {
        qemu_error("scsi-disk: drive property not set\n");
        return -1;
    }

    if (bdrv_get_type_hint(s->dinfo->bdrv) == BDRV_TYPE_CDROM) {
        s->cluster_size = 4;
    } else {
        s->cluster_size = 1;
    }
    s->qdev.blocksize = 512 * s->cluster_size;
    s->qdev.type = TYPE_DISK;
    bdrv_get_geometry(s->dinfo->bdrv, &nb_sectors);
    nb_sectors /= s->cluster_size;
    if (nb_sectors)
        nb_sectors--;
    s->max_lba = nb_sectors;
    strncpy(s->drive_serial_str, drive_get_serial(s->dinfo->bdrv),
            sizeof(s->drive_serial_str));
    if (strlen(s->drive_serial_str) == 0)
        pstrcpy(s->drive_serial_str, sizeof(s->drive_serial_str), "0");
    qemu_add_vm_change_state_handler(scsi_dma_restart_cb, s);
    return 0;
}

static SCSIDeviceInfo scsi_disk_info = {
    .qdev.name    = "scsi-disk",
    .qdev.desc    = "virtual scsi disk or cdrom",
    .qdev.size    = sizeof(SCSIDiskState),
    .init         = scsi_disk_initfn,
    .destroy      = scsi_destroy,
    .send_command = scsi_send_command,
    .read_data    = scsi_read_data,
    .write_data   = scsi_write_data,
    .cancel_io    = scsi_cancel_io,
    .get_buf      = scsi_get_buf,
    .qdev.props   = (Property[]) {
        DEFINE_PROP_DRIVE("drive", SCSIDiskState, dinfo),
        DEFINE_PROP_END_OF_LIST(),
    },
};

static void scsi_disk_register_devices(void)
{
    scsi_qdev_register(&scsi_disk_info);
}
device_init(scsi_disk_register_devices)
