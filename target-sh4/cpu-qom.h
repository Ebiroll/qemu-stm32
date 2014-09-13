/*
 * QEMU SuperH CPU
 *
 * Copyright (c) 2012 SUSE LINUX Products GmbH
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */
#ifndef QEMU_SUPERH_CPU_QOM_H
#define QEMU_SUPERH_CPU_QOM_H

#include "qom/cpu.h"

#define TYPE_SUPERH_CPU "superh-cpu"

#define TYPE_SH7750R_CPU "sh7750r-" TYPE_SUPERH_CPU
#define TYPE_SH7751R_CPU "sh7751r-" TYPE_SUPERH_CPU
#define TYPE_SH7785_CPU "sh7785-" TYPE_SUPERH_CPU

#define SUPERH_CPU_CLASS(klass) \
    OBJECT_CLASS_CHECK(SuperHCPUClass, (klass), TYPE_SUPERH_CPU)
#define SUPERH_CPU(obj) \
    OBJECT_CHECK(SuperHCPU, (obj), TYPE_SUPERH_CPU)
#define SUPERH_CPU_GET_CLASS(obj) \
    OBJECT_GET_CLASS(SuperHCPUClass, (obj), TYPE_SUPERH_CPU)

/**
 * SuperHCPUClass:
 * @parent_realize: The parent class' realize handler.
 * @parent_reset: The parent class' reset handler.
 * @name: The name.
 * @pvr: Processor Version Register
 * @prr: Processor Revision Register
 * @cvr: Cache Version Register
 *
 * A SuperH CPU model.
 */
typedef struct SuperHCPUClass {
    /*< private >*/
    CPUClass parent_class;
    /*< public >*/

    DeviceRealize parent_realize;
    void (*parent_reset)(CPUState *cpu);

    const char *name;
    uint32_t pvr;
    uint32_t prr;
    uint32_t cvr;
} SuperHCPUClass;

/**
 * SuperHCPU:
 * @env: #CPUSH4State
 *
 * A SuperH CPU.
 */
typedef struct SuperHCPU {
    /*< private >*/
    CPUState parent_obj;
    /*< public >*/

    CPUSH4State env;
} SuperHCPU;

static inline SuperHCPU *sh_env_get_cpu(CPUSH4State *env)
{
    return container_of(env, SuperHCPU, env);
}

#define ENV_GET_CPU(e) CPU(sh_env_get_cpu(e))

#define ENV_OFFSET offsetof(SuperHCPU, env)

void superh_cpu_do_interrupt(CPUState *cpu);
bool superh_cpu_exec_interrupt(CPUState *cpu, int int_req);
void superh_cpu_dump_state(CPUState *cpu, FILE *f,
                           fprintf_function cpu_fprintf, int flags);
hwaddr superh_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
int superh_cpu_gdb_read_register(CPUState *cpu, uint8_t *buf, int reg);
int superh_cpu_gdb_write_register(CPUState *cpu, uint8_t *buf, int reg);

#endif
