/*
 *  Stub for target_get_monitor_def.
 *
 *  Copyright IBM Corp., 2015
 *
 *  Author: Alexey Kardashevskiy <aik@ozlabs.ru>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License,
 *  or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/typedefs.h"
#include "stdint.h"

int target_get_monitor_def(CPUState *cs, const char *name, uint64_t *pval);

int target_get_monitor_def(CPUState *cs, const char *name, uint64_t *pval)
{
    return -1;
}
