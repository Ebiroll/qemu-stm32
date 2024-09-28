/*
 * ST STM32U5 machine
 *
 * Copyright (c) 2023 Neonode
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
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-clock.h"
#include "qemu/error-report.h"
#include "hw/arm/stm32u535_soc.h"
#include "hw/arm/boot.h"
//#include "hw/arm/asic_sim/nn1002.h"


struct TSMU5MachineClass {
    MachineClass parent;
    //enum spitz_model_e model;
    int arm_id;
};


struct TSMU5MachineState {
    MachineState parent;
    DeviceState *asic1;
};

#define TYPE_TSMU5_MACHINE "tsmu5-common"
OBJECT_DECLARE_TYPE(TSMU5MachineState, TSMU5MachineClass, TSMU5_MACHINE)
 //  // TYPE_TSMU5_MACHINE

static void neon_u5_init(MachineState *machine);


/* Main SYSCLK frequency in Hz (110MHz) */
#define SYSCLK_FRQ 110000000ULL

static void neon_u5_init(MachineState *machine)
{
    DeviceState *dev;
    Clock *sysclk;
    //qemu_irq spi_irq;

    //tsmproMachineClass *smc = u5_MACHINE_GET_CLASS(machine);
    //tsmproMachineState *sms = u5_MACHINE(machine);


    /* This clock doesn't need migration because it is fixed-frequency */
    sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, SYSCLK_FRQ);

    dev = qdev_new(TYPE_STM32U535_SOC);
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m33"));
    qdev_connect_clock_in(dev, "sysclk", sysclk);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);



    

    armv7m_load_kernel(ARM_CPU(first_cpu),
                       machine->kernel_filename,
                       0, FLASH_SIZE);
}


static void u5_common_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    //mc->block_default_type = IF_IDE;
    mc->ignore_memory_transaction_failures = true;
    mc->init = neon_u5_init;

    //machine_add_audiodev_property(mc);
}


static const TypeInfo neon_common_info = {
    .name = TYPE_TSMU5_MACHINE,
    .parent = TYPE_MACHINE,
    .abstract = true,
    .instance_size = sizeof(TSMU5MachineState),
    .class_size = sizeof(TSMU5MachineClass),
    .class_init = u5_common_class_init,
};
#if 0
static void neon_u5_machine_init(MachineClass *mc)
{
    mc->desc = "Neonode tsmpro (Cortex-M4)";
    mc->init = neon_u5_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m4");

    /* SRAM pre-allocated as part of the SoC instantiation */
    mc->default_ram_size = 0;
}
#endif


static void terrierpda_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Neonode tsmpro (Cortex-M33)";
    mc->init = neon_u5_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m33");


}

static const TypeInfo terrierpda_type = {
    .name = MACHINE_TYPE_NAME("tsmu5"),
    .parent = TYPE_TSMU5_MACHINE,
    .class_init = terrierpda_class_init,
};

static void neon_register_types(void)
{
    type_register_static(&neon_common_info);
    type_register_static(&terrierpda_type);
}

// DEFINE_MACHINE("tsmpro", neon_u5_machine_init)

type_init(neon_register_types)