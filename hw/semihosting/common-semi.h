/*
 *  Semihosting support for systems modeled on the Arm "Angel"
 *  semihosting syscalls design.
 *
 *  Copyright (c) 2005, 2007 CodeSourcery.
 *  Copyright (c) 2019 Linaro
 *  Written by Paul Brook.
 *
 *  Copyright © 2020 by Keith Packard <keithp@keithp.com>
 *  Adapted for systems other than ARM, including RISC-V, by Keith Packard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 *  ARM Semihosting is documented in:
 *     Semihosting for AArch32 and AArch64 Release 2.0
 *     https://static.docs.arm.com/100863/0200/semihosting.pdf
 *
 */

#ifndef COMMON_SEMI_H
#define COMMON_SEMI_H

target_ulong do_common_semihosting(CPUState *cs);

#endif /* COMMON_SEMI_H */
