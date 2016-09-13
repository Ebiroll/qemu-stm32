/*
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/rtas.h"

static void qrtas_copy_args(uint64_t target_args, uint32_t nargs,
                            uint32_t *args)
{
    int i;

    for (i = 0; i < nargs; i++) {
        writel(target_args + i * sizeof(uint32_t), args[i]);
    }
}

static void qrtas_copy_ret(uint64_t target_ret, uint32_t nret, uint32_t *ret)
{
    int i;

    for (i = 0; i < nret; i++) {
        ret[i] = readl(target_ret + i * sizeof(uint32_t));
    }
}

static uint64_t qrtas_call(QGuestAllocator *alloc, const char *name,
                           uint32_t nargs, uint32_t *args,
                           uint32_t nret, uint32_t *ret)
{
    uint64_t res;
    uint64_t target_args, target_ret;

    target_args = guest_alloc(alloc, nargs * sizeof(uint32_t));
    target_ret = guest_alloc(alloc, nret * sizeof(uint32_t));

    qrtas_copy_args(target_args, nargs, args);
    res = qtest_rtas_call(global_qtest, name,
                          nargs, target_args, nret, target_ret);
    qrtas_copy_ret(target_ret, nret, ret);

    guest_free(alloc, target_ret);
    guest_free(alloc, target_args);

    return res;
}

int qrtas_get_time_of_day(QGuestAllocator *alloc, struct tm *tm, uint32_t *ns)
{
    int res;
    uint32_t ret[8];

    res = qrtas_call(alloc, "get-time-of-day", 0, NULL, 8, ret);
    if (res != 0) {
        return res;
    }

    res = ret[0];
    memset(tm, 0, sizeof(*tm));
    tm->tm_year = ret[1] - 1900;
    tm->tm_mon = ret[2] - 1;
    tm->tm_mday = ret[3];
    tm->tm_hour = ret[4];
    tm->tm_min = ret[5];
    tm->tm_sec = ret[6];
    *ns = ret[7];

    return res;
}
