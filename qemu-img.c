/*
 * QEMU disk image utility
 *
 * Copyright (c) 2003-2007 Fabrice Bellard
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
#include "block_int.h"
#include <assert.h>

#ifdef _WIN32
#include <windows.h>
#endif

void *get_mmap_addr(unsigned long size)
{
    return NULL;
}

void qemu_free(void *ptr)
{
    free(ptr);
}

void *qemu_malloc(size_t size)
{
    return malloc(size);
}

void *qemu_mallocz(size_t size)
{
    void *ptr;
    ptr = qemu_malloc(size);
    if (!ptr)
        return NULL;
    memset(ptr, 0, size);
    return ptr;
}

char *qemu_strdup(const char *str)
{
    char *ptr;
    ptr = qemu_malloc(strlen(str) + 1);
    if (!ptr)
        return NULL;
    strcpy(ptr, str);
    return ptr;
}

void term_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

void term_print_filename(const char *filename)
{
    term_printf(filename);
}

void __attribute__((noreturn)) error(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "qemu-img: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    exit(1);
    va_end(ap);
}

static void format_print(void *opaque, const char *name)
{
    printf(" %s", name);
}

void help(void)
{
    printf("qemu-img version " QEMU_VERSION ", Copyright (c) 2004-2007 Fabrice Bellard\n"
           "usage: qemu-img command [command options]\n"
           "QEMU disk image utility\n"
           "\n"
           "Command syntax:\n"
           "  create [-e] [-6] [-b base_image] [-f fmt] filename [size]\n"
           "  commit [-f fmt] filename\n"
           "  convert [-c] [-e] [-6] [-f fmt] filename [filename2 [...]] [-O output_fmt] output_filename\n"
           "  info [-f fmt] filename\n"
           "\n"
           "Command parameters:\n"
           "  'filename' is a disk image filename\n"
           "  'base_image' is the read-only disk image which is used as base for a copy on\n"
           "    write image; the copy on write image only stores the modified data\n"
           "  'fmt' is the disk image format. It is guessed automatically in most cases\n"
           "  'size' is the disk image size in kilobytes. Optional suffixes 'M' (megabyte)\n"
           "    and 'G' (gigabyte) are supported\n"
           "  'output_filename' is the destination disk image filename\n"
           "  'output_fmt' is the destination format\n"
           "  '-c' indicates that target image must be compressed (qcow format only)\n"
           "  '-e' indicates that the target image must be encrypted (qcow format only)\n"
           "  '-6' indicates that the target image must use compatibility level 6 (vmdk format only)\n"
           );
    printf("\nSupported format:");
    bdrv_iterate_format(format_print, NULL);
    printf("\n");
    exit(1);
}

#if defined(WIN32)
/* XXX: put correct support for win32 */
static int read_password(char *buf, int buf_size)
{
    int c, i;
    printf("Password: ");
    fflush(stdout);
    i = 0;
    for(;;) {
        c = getchar();
        if (c == '\n')
            break;
        if (i < (buf_size - 1))
            buf[i++] = c;
    }
    buf[i] = '\0';
    return 0;
}

#else

#include <termios.h>

static struct termios oldtty;

static void term_exit(void)
{
    tcsetattr (0, TCSANOW, &oldtty);
}

static void term_init(void)
{
    struct termios tty;

    tcgetattr (0, &tty);
    oldtty = tty;

    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP
                          |INLCR|IGNCR|ICRNL|IXON);
    tty.c_oflag |= OPOST;
    tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
    tty.c_cflag &= ~(CSIZE|PARENB);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    tcsetattr (0, TCSANOW, &tty);

    atexit(term_exit);
}

int read_password(char *buf, int buf_size)
{
    uint8_t ch;
    int i, ret;

    printf("password: ");
    fflush(stdout);
    term_init();
    i = 0;
    for(;;) {
        ret = read(0, &ch, 1);
        if (ret == -1) {
            if (errno == EAGAIN || errno == EINTR) {
                continue;
            } else {
                ret = -1;
                break;
            }
        } else if (ret == 0) {
            ret = -1;
            break;
        } else {
            if (ch == '\r') {
                ret = 0;
                break;
            }
            if (i < (buf_size - 1))
                buf[i++] = ch;
        }
    }
    term_exit();
    buf[i] = '\0';
    printf("\n");
    return ret;
}
#endif

static BlockDriverState *bdrv_new_open(const char *filename,
                                       const char *fmt)
{
    BlockDriverState *bs;
    BlockDriver *drv;
    char password[256];

    bs = bdrv_new("");
    if (!bs)
        error("Not enough memory");
    if (fmt) {
        drv = bdrv_find_format(fmt);
        if (!drv)
            error("Unknown file format '%s'", fmt);
    } else {
        drv = NULL;
    }
    if (bdrv_open2(bs, filename, 0, drv) < 0) {
        error("Could not open '%s'", filename);
    }
    if (bdrv_is_encrypted(bs)) {
        printf("Disk image '%s' is encrypted.\n", filename);
        if (read_password(password, sizeof(password)) < 0)
            error("No password given");
        if (bdrv_set_key(bs, password) < 0)
            error("invalid password");
    }
    return bs;
}

static int img_create(int argc, char **argv)
{
    int c, ret, flags;
    const char *fmt = "raw";
    const char *filename;
    const char *base_filename = NULL;
    int64_t size;
    const char *p;
    BlockDriver *drv;

    flags = 0;
    for(;;) {
        c = getopt(argc, argv, "b:f:he6");
        if (c == -1)
            break;
        switch(c) {
        case 'h':
            help();
            break;
        case 'b':
            base_filename = optarg;
            break;
        case 'f':
            fmt = optarg;
            break;
        case 'e':
            flags |= BLOCK_FLAG_ENCRYPT;
            break;
        case '6':
            flags |= BLOCK_FLAG_COMPAT6;
            break;
        }
    }
    if (optind >= argc)
        help();
    filename = argv[optind++];
    size = 0;
    if (base_filename) {
        BlockDriverState *bs;
        bs = bdrv_new_open(base_filename, NULL);
        bdrv_get_geometry(bs, &size);
        size *= 512;
        bdrv_delete(bs);
    } else {
        if (optind >= argc)
            help();
        p = argv[optind];
        size = strtoul(p, (char **)&p, 0);
        if (*p == 'M') {
            size *= 1024 * 1024;
        } else if (*p == 'G') {
            size *= 1024 * 1024 * 1024;
        } else if (*p == 'k' || *p == 'K' || *p == '\0') {
            size *= 1024;
        } else {
            help();
        }
    }
    drv = bdrv_find_format(fmt);
    if (!drv)
        error("Unknown file format '%s'", fmt);
    printf("Formatting '%s', fmt=%s",
           filename, fmt);
    if (flags & BLOCK_FLAG_ENCRYPT)
        printf(", encrypted");
    if (flags & BLOCK_FLAG_COMPAT6)
        printf(", compatibility level=6");
    if (base_filename) {
        printf(", backing_file=%s",
               base_filename);
    }
    printf(", size=%" PRId64 " kB\n", (int64_t) (size / 1024));
    ret = bdrv_create(drv, filename, size / 512, base_filename, flags);
    if (ret < 0) {
        if (ret == -ENOTSUP) {
            error("Formatting or formatting option not supported for file format '%s'", fmt);
        } else {
            error("Error while formatting");
        }
    }
    return 0;
}

static int img_commit(int argc, char **argv)
{
    int c, ret;
    const char *filename, *fmt;
    BlockDriver *drv;
    BlockDriverState *bs;

    fmt = NULL;
    for(;;) {
        c = getopt(argc, argv, "f:h");
        if (c == -1)
            break;
        switch(c) {
        case 'h':
            help();
            break;
        case 'f':
            fmt = optarg;
            break;
        }
    }
    if (optind >= argc)
        help();
    filename = argv[optind++];

    bs = bdrv_new("");
    if (!bs)
        error("Not enough memory");
    if (fmt) {
        drv = bdrv_find_format(fmt);
        if (!drv)
            error("Unknown file format '%s'", fmt);
    } else {
        drv = NULL;
    }
    if (bdrv_open2(bs, filename, 0, drv) < 0) {
        error("Could not open '%s'", filename);
    }
    ret = bdrv_commit(bs);
    switch(ret) {
    case 0:
        printf("Image committed.\n");
        break;
    case -ENOENT:
        error("No disk inserted");
        break;
    case -EACCES:
        error("Image is read-only");
        break;
    case -ENOTSUP:
        error("Image is already committed");
        break;
    default:
        error("Error while committing image");
        break;
    }

    bdrv_delete(bs);
    return 0;
}

static int is_not_zero(const uint8_t *sector, int len)
{
    int i;
    len >>= 2;
    for(i = 0;i < len; i++) {
        if (((uint32_t *)sector)[i] != 0)
            return 1;
    }
    return 0;
}

static int is_allocated_sectors(const uint8_t *buf, int n, int *pnum)
{
    int v, i;

    if (n <= 0) {
        *pnum = 0;
        return 0;
    }
    v = is_not_zero(buf, 512);
    for(i = 1; i < n; i++) {
        buf += 512;
        if (v != is_not_zero(buf, 512))
            break;
    }
    *pnum = i;
    return v;
}

#define IO_BUF_SIZE 65536

static int img_convert(int argc, char **argv)
{
    int c, ret, n, n1, bs_n, bs_i, flags, cluster_size, cluster_sectors;
    const char *fmt, *out_fmt, *out_filename;
    BlockDriver *drv;
    BlockDriverState **bs, *out_bs;
    int64_t total_sectors, nb_sectors, sector_num, bs_offset, bs_sectors;
    uint8_t buf[IO_BUF_SIZE];
    const uint8_t *buf1;
    BlockDriverInfo bdi;

    fmt = NULL;
    out_fmt = "raw";
    flags = 0;
    for(;;) {
        c = getopt(argc, argv, "f:O:hce6");
        if (c == -1)
            break;
        switch(c) {
        case 'h':
            help();
            break;
        case 'f':
            fmt = optarg;
            break;
        case 'O':
            out_fmt = optarg;
            break;
        case 'c':
            flags |= BLOCK_FLAG_COMPRESS;
            break;
        case 'e':
            flags |= BLOCK_FLAG_ENCRYPT;
            break;
        case '6':
            flags |= BLOCK_FLAG_COMPAT6;
            break;
        }
    }

    bs_n = argc - optind - 1;
    if (bs_n < 1) help();

    out_filename = argv[argc - 1];
        
    bs = calloc(bs_n, sizeof(BlockDriverState *));
    if (!bs)
        error("Out of memory");

    total_sectors = 0;
    for (bs_i = 0; bs_i < bs_n; bs_i++) {
        bs[bs_i] = bdrv_new_open(argv[optind + bs_i], fmt);
        if (!bs[bs_i])
            error("Could not open '%s'", argv[optind + bs_i]);
        bdrv_get_geometry(bs[bs_i], &bs_sectors);
        total_sectors += bs_sectors;
    }

    drv = bdrv_find_format(out_fmt);
    if (!drv)
        error("Unknown file format '%s'", out_fmt);
    if (flags & BLOCK_FLAG_COMPRESS && drv != &bdrv_qcow && drv != &bdrv_qcow2)
        error("Compression not supported for this file format");
    if (flags & BLOCK_FLAG_ENCRYPT && drv != &bdrv_qcow && drv != &bdrv_qcow2)
        error("Encryption not supported for this file format");
    if (flags & BLOCK_FLAG_COMPAT6 && drv != &bdrv_vmdk)
        error("Alternative compatibility level not supported for this file format");
    if (flags & BLOCK_FLAG_ENCRYPT && flags & BLOCK_FLAG_COMPRESS)
        error("Compression and encryption not supported at the same time");

    ret = bdrv_create(drv, out_filename, total_sectors, NULL, flags);
    if (ret < 0) {
        if (ret == -ENOTSUP) {
            error("Formatting not supported for file format '%s'", fmt);
        } else {
            error("Error while formatting '%s'", out_filename);
        }
    }

    out_bs = bdrv_new_open(out_filename, out_fmt);

    bs_i = 0;
    bs_offset = 0;
    bdrv_get_geometry(bs[0], &bs_sectors);

    if (flags & BLOCK_FLAG_COMPRESS) {
        if (bdrv_get_info(out_bs, &bdi) < 0)
            error("could not get block driver info");
        cluster_size = bdi.cluster_size;
        if (cluster_size <= 0 || cluster_size > IO_BUF_SIZE)
            error("invalid cluster size");
        cluster_sectors = cluster_size >> 9;
        sector_num = 0;
        for(;;) {
            int64_t bs_num;
            int remainder;
            uint8_t *buf2;

            nb_sectors = total_sectors - sector_num;
            if (nb_sectors <= 0)
                break;
            if (nb_sectors >= cluster_sectors)
                n = cluster_sectors;
            else
                n = nb_sectors;

            bs_num = sector_num - bs_offset;
            assert (bs_num >= 0);
            remainder = n;
            buf2 = buf;
            while (remainder > 0) {
                int nlow;
                while (bs_num == bs_sectors) {
                    bs_i++;
                    assert (bs_i < bs_n);
                    bs_offset += bs_sectors;
                    bdrv_get_geometry(bs[bs_i], &bs_sectors);
                    bs_num = 0;
                    /* printf("changing part: sector_num=%lld, "
                       "bs_i=%d, bs_offset=%lld, bs_sectors=%lld\n",
                       sector_num, bs_i, bs_offset, bs_sectors); */
                }
                assert (bs_num < bs_sectors);

                nlow = (remainder > bs_sectors - bs_num) ? bs_sectors - bs_num : remainder;

                if (bdrv_read(bs[bs_i], bs_num, buf2, nlow) < 0) 
                    error("error while reading");

                buf2 += nlow * 512;
                bs_num += nlow;

                remainder -= nlow;
            }
            assert (remainder == 0);

            if (n < cluster_sectors)
                memset(buf + n * 512, 0, cluster_size - n * 512);
            if (is_not_zero(buf, cluster_size)) {
                if (bdrv_write_compressed(out_bs, sector_num, buf,
                                          cluster_sectors) != 0)
                    error("error while compressing sector %" PRId64,
                          sector_num);
            }
            sector_num += n;
        }
        /* signal EOF to align */
        bdrv_write_compressed(out_bs, 0, NULL, 0);
    } else {
        sector_num = 0;
        for(;;) {
            nb_sectors = total_sectors - sector_num;
            if (nb_sectors <= 0)
                break;
            if (nb_sectors >= (IO_BUF_SIZE / 512))
                n = (IO_BUF_SIZE / 512);
            else
                n = nb_sectors;

            while (sector_num - bs_offset >= bs_sectors) {
                bs_i ++;
                assert (bs_i < bs_n);
                bs_offset += bs_sectors;
                bdrv_get_geometry(bs[bs_i], &bs_sectors);
                /* printf("changing part: sector_num=%lld, bs_i=%d, "
                  "bs_offset=%lld, bs_sectors=%lld\n",
                   sector_num, bs_i, bs_offset, bs_sectors); */
            }

            if (n > bs_offset + bs_sectors - sector_num)
                n = bs_offset + bs_sectors - sector_num;

            if (bdrv_read(bs[bs_i], sector_num - bs_offset, buf, n) < 0) 
                error("error while reading");
            /* NOTE: at the same time we convert, we do not write zero
               sectors to have a chance to compress the image. Ideally, we
               should add a specific call to have the info to go faster */
            buf1 = buf;
            while (n > 0) {
                if (is_allocated_sectors(buf1, n, &n1)) {
                    if (bdrv_write(out_bs, sector_num, buf1, n1) < 0)
                        error("error while writing");
                }
                sector_num += n1;
                n -= n1;
                buf1 += n1 * 512;
            }
        }
    }
    bdrv_delete(out_bs);
    for (bs_i = 0; bs_i < bs_n; bs_i++)
        bdrv_delete(bs[bs_i]);
    free(bs);
    return 0;
}

#ifdef _WIN32
static int64_t get_allocated_file_size(const char *filename)
{
    typedef DWORD (WINAPI * get_compressed_t)(const char *filename, DWORD *high);
    get_compressed_t get_compressed;
    struct _stati64 st;

    /* WinNT support GetCompressedFileSize to determine allocate size */
    get_compressed = (get_compressed_t) GetProcAddress(GetModuleHandle("kernel32"), "GetCompressedFileSizeA");
    if (get_compressed) {
    	DWORD high, low;
    	low = get_compressed(filename, &high);
    	if (low != 0xFFFFFFFFlu || GetLastError() == NO_ERROR)
	    return (((int64_t) high) << 32) + low;
    }

    if (_stati64(filename, &st) < 0)
        return -1;
    return st.st_size;
}
#else
static int64_t get_allocated_file_size(const char *filename)
{
    struct stat st;
    if (stat(filename, &st) < 0)
        return -1;
    return (int64_t)st.st_blocks * 512;
}
#endif

static void dump_snapshots(BlockDriverState *bs)
{
    QEMUSnapshotInfo *sn_tab, *sn;
    int nb_sns, i;
    char buf[256];

    nb_sns = bdrv_snapshot_list(bs, &sn_tab);
    if (nb_sns <= 0)
        return;
    printf("Snapshot list:\n");
    printf("%s\n", bdrv_snapshot_dump(buf, sizeof(buf), NULL));
    for(i = 0; i < nb_sns; i++) {
        sn = &sn_tab[i];
        printf("%s\n", bdrv_snapshot_dump(buf, sizeof(buf), sn));
    }
    qemu_free(sn_tab);
}

static int img_info(int argc, char **argv)
{
    int c;
    const char *filename, *fmt;
    BlockDriver *drv;
    BlockDriverState *bs;
    char fmt_name[128], size_buf[128], dsize_buf[128];
    int64_t total_sectors, allocated_size;
    char backing_filename[1024];
    char backing_filename2[1024];
    BlockDriverInfo bdi;

    fmt = NULL;
    for(;;) {
        c = getopt(argc, argv, "f:h");
        if (c == -1)
            break;
        switch(c) {
        case 'h':
            help();
            break;
        case 'f':
            fmt = optarg;
            break;
        }
    }
    if (optind >= argc)
        help();
    filename = argv[optind++];

    bs = bdrv_new("");
    if (!bs)
        error("Not enough memory");
    if (fmt) {
        drv = bdrv_find_format(fmt);
        if (!drv)
            error("Unknown file format '%s'", fmt);
    } else {
        drv = NULL;
    }
    if (bdrv_open2(bs, filename, 0, drv) < 0) {
        error("Could not open '%s'", filename);
    }
    bdrv_get_format(bs, fmt_name, sizeof(fmt_name));
    bdrv_get_geometry(bs, &total_sectors);
    get_human_readable_size(size_buf, sizeof(size_buf), total_sectors * 512);
    allocated_size = get_allocated_file_size(filename);
    if (allocated_size < 0)
	sprintf(dsize_buf, "unavailable");
    else
        get_human_readable_size(dsize_buf, sizeof(dsize_buf),
                                allocated_size);
    printf("image: %s\n"
           "file format: %s\n"
           "virtual size: %s (%" PRId64 " bytes)\n"
           "disk size: %s\n",
           filename, fmt_name, size_buf,
           (total_sectors * 512),
           dsize_buf);
    if (bdrv_is_encrypted(bs))
        printf("encrypted: yes\n");
    if (bdrv_get_info(bs, &bdi) >= 0) {
        if (bdi.cluster_size != 0)
            printf("cluster_size: %d\n", bdi.cluster_size);
    }
    bdrv_get_backing_filename(bs, backing_filename, sizeof(backing_filename));
    if (backing_filename[0] != '\0') {
        path_combine(backing_filename2, sizeof(backing_filename2),
                     filename, backing_filename);
        printf("backing file: %s (actual path: %s)\n",
               backing_filename,
               backing_filename2);
    }
    dump_snapshots(bs);
    bdrv_delete(bs);
    return 0;
}

int main(int argc, char **argv)
{
    const char *cmd;

    bdrv_init();
    if (argc < 2)
        help();
    cmd = argv[1];
    optind++;
    if (!strcmp(cmd, "create")) {
        img_create(argc, argv);
    } else if (!strcmp(cmd, "commit")) {
        img_commit(argc, argv);
    } else if (!strcmp(cmd, "convert")) {
        img_convert(argc, argv);
    } else if (!strcmp(cmd, "info")) {
        img_info(argc, argv);
    } else {
        help();
    }
    return 0;
}
