/*
 * memfd.c
 *
 * Copyright (c) 2015 Red Hat, Inc.
 *
 * QEMU library functions on POSIX which are shared between QEMU and
 * the QEMU tools.
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

#include "qapi/error.h"
#include "qemu/memfd.h"

#if defined CONFIG_LINUX && !defined CONFIG_MEMFD
#include <sys/syscall.h>
#include <asm/unistd.h>

static int memfd_create(const char *name, unsigned int flags)
{
#ifdef __NR_memfd_create
    return syscall(__NR_memfd_create, name, flags);
#else
    return -1;
#endif
}
#endif

#ifndef MFD_CLOEXEC
#define MFD_CLOEXEC 0x0001U
#endif

#ifndef MFD_ALLOW_SEALING
#define MFD_ALLOW_SEALING 0x0002U
#endif

#ifndef MFD_HUGETLB
#define MFD_HUGETLB 0x0004U
#endif

int qemu_memfd_create(const char *name, size_t size, bool hugetlb,
                      unsigned int seals, Error **errp)
{
#ifdef CONFIG_LINUX
    int mfd = -1;
    unsigned int flags = MFD_CLOEXEC;

    if (seals) {
        flags |= MFD_ALLOW_SEALING;
    }
    if (hugetlb) {
        flags |= MFD_HUGETLB;
    }

    mfd = memfd_create(name, flags);
    if (mfd < 0) {
        goto err;
    }

    if (ftruncate(mfd, size) == -1) {
        goto err;
    }

    if (seals && fcntl(mfd, F_ADD_SEALS, seals) == -1) {
        goto err;
    }

    return mfd;

err:
    if (mfd >= 0) {
        close(mfd);
    }
#endif
    error_setg_errno(errp, errno, "failed to create memfd");
    return -1;
}

/*
 * This is a best-effort helper for shared memory allocation, with
 * optional sealing. The helper will do his best to allocate using
 * memfd with sealing, but may fallback on other methods without
 * sealing.
 */
void *qemu_memfd_alloc(const char *name, size_t size, unsigned int seals,
                       int *fd, Error **errp)
{
    void *ptr;
    int mfd = qemu_memfd_create(name, size, false, seals, NULL);

    /* some systems have memfd without sealing */
    if (mfd == -1) {
        mfd = qemu_memfd_create(name, size, false, 0, NULL);
    }

    if (mfd == -1) {
        const char *tmpdir = g_get_tmp_dir();
        gchar *fname;

        fname = g_strdup_printf("%s/memfd-XXXXXX", tmpdir);
        mfd = mkstemp(fname);
        unlink(fname);
        g_free(fname);

        if (mfd == -1 ||
            ftruncate(mfd, size) == -1) {
            goto err;
        }
    }

    ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, 0);
    if (ptr == MAP_FAILED) {
        goto err;
    }

    *fd = mfd;
    return ptr;

err:
    error_setg_errno(errp, errno, "failed to allocate shared memory");
    if (mfd >= 0) {
        close(mfd);
    }
    return NULL;
}

void qemu_memfd_free(void *ptr, size_t size, int fd)
{
    if (ptr) {
        munmap(ptr, size);
    }

    if (fd != -1) {
        close(fd);
    }
}

enum {
    MEMFD_KO,
    MEMFD_OK,
    MEMFD_TODO
};

bool qemu_memfd_check(void)
{
    static int memfd_check = MEMFD_TODO;

    if (memfd_check == MEMFD_TODO) {
        int fd;
        void *ptr;

        ptr = qemu_memfd_alloc("test", 4096, 0, &fd, NULL);
        memfd_check = ptr ? MEMFD_OK : MEMFD_KO;
        qemu_memfd_free(ptr, 4096, fd);
    }

    return memfd_check == MEMFD_OK;
}
