/*
 * QEMU seccomp mode 2 support with libseccomp
 *
 * Copyright IBM, Corp. 2012
 *
 * Authors:
 *  Eduardo Otubo    <eotubo@br.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */
#ifndef QEMU_SECCOMP_H
#define QEMU_SECCOMP_H

#define QEMU_SECCOMP_SET_DEFAULT     (1 << 0)
#define QEMU_SECCOMP_SET_OBSOLETE    (1 << 1)
#define QEMU_SECCOMP_SET_PRIVILEGED  (1 << 2)

#include <seccomp.h>

int seccomp_start(uint32_t seccomp_opts);
#endif
