/*
 * QEMU LatticeMico32 CPU
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
#ifndef QEMU_LM32_CPU_QOM_H
#define QEMU_LM32_CPU_QOM_H

#include "qemu/cpu.h"
#include "cpu.h"

#define TYPE_LM32_CPU "lm32-cpu"

#define LM32_CPU_CLASS(klass) \
    OBJECT_CLASS_CHECK(LM32CPUClass, (klass), TYPE_LM32_CPU)
#define LM32_CPU(obj) \
    OBJECT_CHECK(LM32CPU, (obj), TYPE_LM32_CPU)
#define LM32_CPU_GET_CLASS(obj) \
    OBJECT_GET_CLASS(LM32CPUClass, (obj), TYPE_LM32_CPU)

/**
 * LM32CPUClass:
 * @parent_reset: The parent class' reset handler.
 *
 * A LatticeMico32 CPU model.
 */
typedef struct LM32CPUClass {
    /*< private >*/
    CPUClass parent_class;
    /*< public >*/

    void (*parent_reset)(CPUState *cpu);
} LM32CPUClass;

/**
 * LM32CPU:
 * @env: #CPULM32State
 *
 * A LatticeMico32 CPU.
 */
typedef struct LM32CPU {
    /*< private >*/
    CPUState parent_obj;
    /*< public >*/

    CPULM32State env;
} LM32CPU;

static inline LM32CPU *lm32_env_get_cpu(CPULM32State *env)
{
    return LM32_CPU(container_of(env, LM32CPU, env));
}

#define ENV_GET_CPU(e) CPU(lm32_env_get_cpu(e))


#endif
