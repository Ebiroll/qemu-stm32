/*
 * ST STM32VLDISCOVERY machine
 * Olimex STM32-H405 machine
 *
 * Copyright (c) 2022 Felipe Balbi <balbi@kernel.org>
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
#include "hw/arm/stm32f405_soc.h"
#include "hw/arm/boot.h"
//#include "hw/arm/asic_sim/nn1002.h"

/*

enum spitz_model_e { spitz, akita, borzoi, terrier };

struct SpitzMachineClass {
    MachineClass parent;
    enum spitz_model_e model;
    int arm_id;
};

struct SpitzMachineState {
    MachineState parent;
    PXA2xxState *mpu;
    DeviceState *mux;
    DeviceState *lcdtg;
    DeviceState *ads7846;
    DeviceState *max1111;
    DeviceState *scp0;
    DeviceState *scp1;
    DeviceState *misc_gpio;
};
*/

struct TSMV2MachineClass {
    MachineClass parent;
    //enum spitz_model_e model;
    int arm_id;
};


struct TSMV2MachineState {
    MachineState parent;
    DeviceState *asic1;
};

#define TYPE_TSMV2_MACHINE "tsmv2-common"
OBJECT_DECLARE_TYPE(TSMV2MachineState, TSMV2MachineClass, TSMV2_MACHINE)

static void neon_tsmv2_init(MachineState *machine);


/* Main SYSCLK frequency in Hz (84MHz) */
#define SYSCLK_FRQ 84000000ULL

static void neon_tsmv2_init(MachineState *machine)
{
    DeviceState *dev;
    Clock *sysclk;
    //qemu_irq spi_irq;

    //TSMV2MachineClass *smc = TSMV2_MACHINE_GET_CLASS(machine);
    //TSMV2MachineState *sms = TSMV2_MACHINE(machine);


    /* This clock doesn't need migration because it is fixed-frequency */
    sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, SYSCLK_FRQ);

    dev = qdev_new(TYPE_STM32F405_SOC);
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    qdev_connect_clock_in(dev, "sysclk", sysclk);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);


    //DeviceState *rcc = qdev_new("stm32fxxx-rcc");
    //qdev_prop_set_uint32(rcc, "osc_freq", 84000000);
    //qdev_prop_set_uint32(rcc, "osc32_freq", 32000);
    //sysbus_realize_and_unref(SYS_BUS_DEVICE(rcc), &error_fatal);
    // object_property_add_child(obj, "rcc", OBJECT(rcc), NULL);
    //s->rcc = SYS_BUS_DEVICE(rcc);

    //void *bus;

    //bus = qdev_get_child_bus(sms->asic1, "spi[*]");
    //sms->asic1 = qdev_new(TYPE_NN1002X);
    //qdev_prop_set_uint8(sms->asic1, "input1" /* BATT_VOLT */,
    //                    47);

    //qdev_prop_set_uint8(sms->max1111, "input2" /* BATT_TEMP */, 0);
    //qdev_prop_set_uint8(sms->max1111, "input3" /* ACIN_VOLT */,
    //                    SPITZ_CHARGEON_ACIN);
    //ssi_realize_and_unref(sms->asic1, bus, &error_fatal);


    

    armv7m_load_kernel(ARM_CPU(first_cpu),
                       machine->kernel_filename,
                       0, FLASH_SIZE);
}


static void tsmv2_common_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    //mc->block_default_type = IF_IDE;
    mc->ignore_memory_transaction_failures = true;
    mc->init = neon_tsmv2_init;

    //machine_add_audiodev_property(mc);
}


static const TypeInfo neon_common_info = {
    .name = TYPE_TSMV2_MACHINE,
    .parent = TYPE_MACHINE,
    .abstract = true,
    .instance_size = sizeof(TSMV2MachineState),
    .class_size = sizeof(TSMV2MachineClass),
    .class_init = tsmv2_common_class_init,
};
#if 0
static void neon_tsmv2_machine_init(MachineClass *mc)
{
    mc->desc = "Neonode tsmv2 (Cortex-M4)";
    mc->init = neon_tsmv2_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m4");

    /* SRAM pre-allocated as part of the SoC instantiation */
    mc->default_ram_size = 0;
}
#endif


static void terrierpda_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Neonode tsmv2 (Cortex-M4)";
    mc->init = neon_tsmv2_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m4");


}

static const TypeInfo terrierpda_type = {
    .name = MACHINE_TYPE_NAME("tsmv2"),
    .parent = TYPE_TSMV2_MACHINE,
    .class_init = terrierpda_class_init,
};

static void neon_register_types(void)
{
    type_register_static(&neon_common_info);
    type_register_static(&terrierpda_type);
}

// DEFINE_MACHINE("tsmv2", neon_tsmv2_machine_init)

type_init(neon_register_types)