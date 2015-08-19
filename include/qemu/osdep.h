#ifndef QEMU_OSDEP_H
#define QEMU_OSDEP_H

#include "config-host.h"
#include "qemu/compiler.h"
#include <stdarg.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#ifdef __OpenBSD__
#include <sys/signal.h>
#endif

#ifndef _WIN32
#include <sys/wait.h>
#else
#define WIFEXITED(x)   1
#define WEXITSTATUS(x) (x)
#endif

#include <sys/time.h>

#if defined(CONFIG_SOLARIS) && CONFIG_SOLARIS_VERSION < 10
/* [u]int_fast*_t not in <sys/int_types.h> */
typedef unsigned char           uint_fast8_t;
typedef unsigned int            uint_fast16_t;
typedef signed int              int_fast16_t;
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/* Minimum function that returns zero only iff both values are zero.
 * Intended for use with unsigned values only. */
#ifndef MIN_NON_ZERO
#define MIN_NON_ZERO(a, b) (((a) != 0 && (a) < (b)) ? (a) : (b))
#endif

#ifndef ROUND_UP
#define ROUND_UP(n,d) (((n) + (d) - 1) & -(d))
#endif

#ifndef DIV_ROUND_UP
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

int qemu_daemon(int nochdir, int noclose);
void *qemu_try_memalign(size_t alignment, size_t size);
void *qemu_memalign(size_t alignment, size_t size);
void *qemu_anon_ram_alloc(size_t size, uint64_t *align);
void qemu_vfree(void *ptr);
void qemu_anon_ram_free(void *ptr, size_t size);

#define QEMU_MADV_INVALID -1

#if defined(CONFIG_MADVISE)

#define QEMU_MADV_WILLNEED  MADV_WILLNEED
#define QEMU_MADV_DONTNEED  MADV_DONTNEED
#ifdef MADV_DONTFORK
#define QEMU_MADV_DONTFORK  MADV_DONTFORK
#else
#define QEMU_MADV_DONTFORK  QEMU_MADV_INVALID
#endif
#ifdef MADV_MERGEABLE
#define QEMU_MADV_MERGEABLE MADV_MERGEABLE
#else
#define QEMU_MADV_MERGEABLE QEMU_MADV_INVALID
#endif
#ifdef MADV_UNMERGEABLE
#define QEMU_MADV_UNMERGEABLE MADV_UNMERGEABLE
#else
#define QEMU_MADV_UNMERGEABLE QEMU_MADV_INVALID
#endif
#ifdef MADV_DODUMP
#define QEMU_MADV_DODUMP MADV_DODUMP
#else
#define QEMU_MADV_DODUMP QEMU_MADV_INVALID
#endif
#ifdef MADV_DONTDUMP
#define QEMU_MADV_DONTDUMP MADV_DONTDUMP
#else
#define QEMU_MADV_DONTDUMP QEMU_MADV_INVALID
#endif
#ifdef MADV_HUGEPAGE
#define QEMU_MADV_HUGEPAGE MADV_HUGEPAGE
#else
#define QEMU_MADV_HUGEPAGE QEMU_MADV_INVALID
#endif

#elif defined(CONFIG_POSIX_MADVISE)

#define QEMU_MADV_WILLNEED  POSIX_MADV_WILLNEED
#define QEMU_MADV_DONTNEED  POSIX_MADV_DONTNEED
#define QEMU_MADV_DONTFORK  QEMU_MADV_INVALID
#define QEMU_MADV_MERGEABLE QEMU_MADV_INVALID
#define QEMU_MADV_UNMERGEABLE QEMU_MADV_INVALID
#define QEMU_MADV_DODUMP QEMU_MADV_INVALID
#define QEMU_MADV_DONTDUMP QEMU_MADV_INVALID
#define QEMU_MADV_HUGEPAGE  QEMU_MADV_INVALID

#else /* no-op */

#define QEMU_MADV_WILLNEED  QEMU_MADV_INVALID
#define QEMU_MADV_DONTNEED  QEMU_MADV_INVALID
#define QEMU_MADV_DONTFORK  QEMU_MADV_INVALID
#define QEMU_MADV_MERGEABLE QEMU_MADV_INVALID
#define QEMU_MADV_UNMERGEABLE QEMU_MADV_INVALID
#define QEMU_MADV_DODUMP QEMU_MADV_INVALID
#define QEMU_MADV_DONTDUMP QEMU_MADV_INVALID
#define QEMU_MADV_HUGEPAGE  QEMU_MADV_INVALID

#endif

int qemu_madvise(void *addr, size_t len, int advice);

int qemu_open(const char *name, int flags, ...);
int qemu_close(int fd);

#if defined(__HAIKU__) && defined(__i386__)
#define FMT_pid "%ld"
#elif defined(WIN64)
#define FMT_pid "%" PRId64
#else
#define FMT_pid "%d"
#endif

int qemu_create_pidfile(const char *filename);
int qemu_get_thread_id(void);

#ifndef CONFIG_IOVEC
struct iovec {
    void *iov_base;
    size_t iov_len;
};
/*
 * Use the same value as Linux for now.
 */
#define IOV_MAX 1024

ssize_t readv(int fd, const struct iovec *iov, int iov_cnt);
ssize_t writev(int fd, const struct iovec *iov, int iov_cnt);
#else
#include <sys/uio.h>
#endif

#ifdef _WIN32
static inline void qemu_timersub(const struct timeval *val1,
                                 const struct timeval *val2,
                                 struct timeval *res)
{
    res->tv_sec = val1->tv_sec - val2->tv_sec;
    if (val1->tv_usec < val2->tv_usec) {
        res->tv_sec--;
        res->tv_usec = val1->tv_usec - val2->tv_usec + 1000 * 1000;
    } else {
        res->tv_usec = val1->tv_usec - val2->tv_usec;
    }
}
#else
#define qemu_timersub timersub
#endif

void qemu_set_cloexec(int fd);

void qemu_set_version(const char *);
const char *qemu_get_version(void);

void fips_set_state(bool requested);
bool fips_get_state(void);

/* Return a dynamically allocated pathname denoting a file or directory that is
 * appropriate for storing local state.
 *
 * @relative_pathname need not start with a directory separator; one will be
 * added automatically.
 *
 * The caller is responsible for releasing the value returned with g_free()
 * after use.
 */
char *qemu_get_local_state_pathname(const char *relative_pathname);

/* Find program directory, and save it for later usage with
 * qemu_get_exec_dir().
 * Try OS specific API first, if not working, parse from argv0. */
void qemu_init_exec_dir(const char *argv0);

/* Get the saved exec dir.
 * Caller needs to release the returned string by g_free() */
char *qemu_get_exec_dir(void);

/**
 * qemu_getauxval:
 * @type: the auxiliary vector key to lookup
 *
 * Search the auxiliary vector for @type, returning the value
 * or 0 if @type is not present.
 */
unsigned long qemu_getauxval(unsigned long type);

void qemu_set_tty_echo(int fd, bool echo);

void os_mem_prealloc(int fd, char *area, size_t sz);

int qemu_read_password(char *buf, int buf_size);

#endif
