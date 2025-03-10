/*
 * STM32F4xx SoC
 *
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
 * Copyright (c) 2018 Martin Schröder <mkschreder.uk@gmail.com>
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
//#include "qemu-common.h"
//#include "hw/arm/arm_"
#include "exec/address-spaces.h"
//#include "hw/arm/arm.h"
#include "hw/arm/armv7m.h"
#include "hw/qdev-properties.h"
#include "hw/or-irq.h"
#include "cpu.h"

#define FLASH_BASE_ADDRESS 0x08000000
#define FLASH_SIZE (2 * 1024 * 1024)
#define SRAM_BASE_ADDRESS 0x20000000
#define SRAM_SIZE (192 * 1024)

#include "hw/misc/stm32f2xx_syscfg.h"
#include "hw/timer/stm32f2xx_timer.h"
#include "hw/ssi/stm32f2xx_spi.h"
#include "hw/or-irq.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/stm32fxxx.h"
#include "hw/arm/stm32/stm32fxxx_rcc.h"
#include "hw/adc/stm32f2xx_adc.h"
#include "sysemu/sysemu.h"
#include "migration/vmstate.h"


//#define STM32F429_439xx
//#include "stm32f4xx.h"
#define STM32F40_41xxx
#include "../stm32f4xx.h"
// #include "stm32fxxx_spi.h"

#define TYPE_STM32FXXX_SOC "stm32f4xx-soc"
#define STM32FXXX_SOC(obj) \
    OBJECT_CHECK(struct stm32f4xx_soc, (obj), TYPE_STM32FXXX_SOC)

#define NAME_SIZE 20

struct stm32f4xx_soc {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    ARMv7MState armv7m;

    SysBusDevice *syscfg;

    SysBusDevice *usart[STM32FXXX_NUM_UARTS];
    SysBusDevice *tim[STM32FXXX_NUM_TIMERS];
    SysBusDevice *adc[STM32FXXX_NUM_ADCS];

    STM32F2XXSPIState spi[STM32FXXX_NUM_SPIS];
    SysBusDevice *rcc;
    SysBusDevice *fmc;
    SysBusDevice *pwr;
    SysBusDevice *dwt;
    SysBusDevice *gpio[STM32FXXX_NUM_GPIOS];

    //qemu_or_irq *adc_irqs;
    qemu_irq*adc_irqs;

    char *cpu_type;
    MemoryRegion mmio;

    struct stm32fxxx_state state;
};

static void stm32f4xx_rogue_mem_write(void *opaque, hwaddr addr,
                                  uint64_t val64, unsigned int size) {
    printf("Rogue mem write to %08x\n", (uint32_t)addr);
}

static uint64_t stm32f4xx_rogue_mem_read(void *opaque, hwaddr addr,
                                       unsigned int size) {
    printf("Rogue mem read from %08x\n", (uint32_t)addr);
    return 0;
}

static const MemoryRegionOps _stm32_rogue_mem_ops = {
    .read = stm32f4xx_rogue_mem_read,
    .write = stm32f4xx_rogue_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int stm32_realize_peripheral(ARMv7MState *cpu, SysBusDevice *dev, hwaddr base, unsigned int irqnr, Error **errp){
    Error *err = NULL;

    object_property_set_bool(OBJECT(dev), "realized", true,  &error_fatal);

    if (err != NULL) {
        error_propagate(errp, err);
        return -1;
    }

    sysbus_mmio_map(dev, 0, base);
    sysbus_connect_irq(dev, 0, qdev_get_gpio_in(DEVICE(cpu), irqnr));

    return 0;
}
/*
 //   dev = qdev_create(bus, type_name);
    +    dev = qdev_new(type_name);
         ... when != dev = expr
    -    qdev_init_nofail(dev);
    +    qdev_realize_and_unref(dev, bus, &error_fatal);
    */

void sysbus_init_child_obj(Object *parent, const char *childname, void *child,
                           size_t childsize, const char *childtype);


static void stm32f4xx_soc_initfn(Object *obj){
    struct stm32f4xx_soc *s = STM32FXXX_SOC(obj);
    int i;
    char name[NAME_SIZE];

    // add memory handler that will catch all access outside of valid range
    memory_region_init_io(&s->mmio, obj, &_stm32_rogue_mem_ops, s,
                          "stm32f4xx-soc", 0xffffffff);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

    object_initialize(&s->armv7m, sizeof(s->armv7m), TYPE_ARMV7M);
    qdev_set_parent_bus(DEVICE(&s->armv7m), sysbus_get_default(),&error_abort);

    s->syscfg = sysbus_create_child_obj(obj, name, "stm32f2xx-syscfg");

    //DeviceState *rcc = qdev_new("stm32fxxx-rcc");
    //qdev_prop_set_uint32(rcc, "osc_freq", 8000000);
    //qdev_prop_set_uint32(rcc, "osc32_freq", 32000);
    //qdev_realize_and_unref(rcc, sysbus_get_default(), &error_fatal);
    //object_property_add_child(obj, "rcc", OBJECT(rcc));
    // s->rcc = SYS_BUS_DEVICE(rcc);

    s->rcc = sysbus_create_child_obj(obj, "rcc", "stm32fxxx-rcc");

    s->fmc = sysbus_create_child_obj(obj, "fmc", "stm32fxxx-fmc");
    s->pwr = sysbus_create_child_obj(obj, "pwr", "stm32fxxx-pwr");
    //s->dwt = sysbus_create_child_obj(obj, "dwt", "armv7m-dwt");

    // qdev_prop_set_ptr(DEVICE(s->pwr), "state", &s->state);

    for (i = 0; i < STM32FXXX_NUM_UARTS; i++) {
        snprintf(name, NAME_SIZE, "usart[%d]", i);
        s->usart[i] = sysbus_create_child_obj(obj, name, "stm32f2xx-usart");
        qdev_prop_set_chr(DEVICE(s->usart[i]), "chardev", serial_hd(i));
    }

    for (i = 0; i < STM32FXXX_NUM_TIMERS; i++) {
        snprintf(name, NAME_SIZE, "tim[%d]", i);
        s->tim[i] = sysbus_create_child_obj(obj, name, "stm32f2xx-timer");
    }

    // s->adc_irqs = OR_IRQ(object_new(TYPE_OR_IRQ));

    for (i = 0; i < STM32FXXX_NUM_ADCS; i++) {
        snprintf(name, NAME_SIZE, "adc[%d]", i);
        s->adc[i] = sysbus_create_child_obj(obj, name, "stm32f2xx-adc");
    }

    for (i = 0; i < STM32FXXX_NUM_SPIS; i++) {
        snprintf(name, NAME_SIZE, "spi[%d]", i);
        object_initialize_child(obj, name, &s->spi[i],"stm32fxxx-spi"); 

        //sysbus_init_child_obj(obj, name, void *child, 
        //                   size_t childsize, const char *childtype);

        //qdev_prop_set_ptr(DEVICE(s->spi[i]), "regs", &s->state.SPI[i]);
        qdev_prop_set_uint8(DEVICE(&s->spi[i]), "device_id", i);
    }

    for (i = 0; i < STM32FXXX_NUM_GPIOS; i++) {
        snprintf(name, NAME_SIZE, "GPIO%c", 'A' + i);
        s->gpio[i] = sysbus_create_child_obj(obj, name, "stm32fxxx-gpio");
        qdev_prop_set_uint8(DEVICE(s->gpio[i]), "port_id", i);
        //qdev_prop_set_ptr(DEVICE(s->gpio[i]), "state", &s->state);
    }
}

static void stm32f4xx_soc_realize(DeviceState *dev_soc, Error **errp) {
    struct stm32f4xx_soc *s = STM32FXXX_SOC(dev_soc);
    Error *err = NULL;
    int i;

    DeviceState *armv7m = DEVICE(&s->armv7m);

    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *flash = g_new(MemoryRegion, 1);
    MemoryRegion *flash_alias = g_new(MemoryRegion, 1);

    memory_region_init_ram(flash, NULL, "STM32F4xx.flash", FLASH_SIZE, &error_fatal);
    memory_region_init_alias(flash_alias, NULL, "STM32F4xx.flash.alias", flash, 0, FLASH_SIZE);

    vmstate_register_ram_global(flash);

    memory_region_set_readonly(flash, true);
    memory_region_set_readonly(flash_alias, true);

    memory_region_add_subregion(system_memory, FLASH_BASE_ADDRESS, flash);
    memory_region_add_subregion(system_memory, 0, flash_alias);

    memory_region_init_ram(sram, NULL, "STM32F4xx.sram", SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(system_memory, SRAM_BASE_ADDRESS, sram);

    memory_region_add_subregion_overlap(system_memory, 0, &s->mmio, -1);

	// init the cpu on the soc
	qdev_prop_set_uint32(armv7m, "num-irq", 96);
    qdev_prop_set_string(armv7m, "cpu-type", s->cpu_type);
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    //         object_property_set_link(cpuobj, "memory",
    //                            OBJECT(&s->cpu_container[i]), &error_abort);

    object_property_set_link(OBJECT(&s->armv7m), "memory", OBJECT(get_system_memory()),
                                     &error_abort);

    object_property_set_bool(OBJECT(&s->armv7m), "realized" , true,  &error_fatal);
    //if (err != NULL) {
    //    error_propagate(errp, err);
    //    return;
    // }

    // map peripherals into memory of the cpu
    // TODO: DAC
    // TODO: BXCAN1 & 2
    // TODO: USBFS
    // TODO: I2C1 & 2
    // TODO: RTC
    // TODO: TIM

    if(stm32_realize_peripheral(&s->armv7m, s->rcc, 0x40023800, 5, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->syscfg, 0x40013800, 91, errp) < 0) return;

    if(stm32_realize_peripheral(&s->armv7m, s->usart[0], 0x40011000, 37, errp) < 0) return; // USART1
    if(stm32_realize_peripheral(&s->armv7m, s->usart[1], 0x40004400, 38, errp) < 0) return; // USART2
    if(stm32_realize_peripheral(&s->armv7m, s->usart[2], 0x40004800, 39, errp) < 0) return; // USART3
    if(stm32_realize_peripheral(&s->armv7m, s->usart[3], 0x40004C00, 52, errp) < 0) return; // UART4
    if(stm32_realize_peripheral(&s->armv7m, s->usart[4], 0x40005000, 53, errp) < 0) return; // UART5
    if(stm32_realize_peripheral(&s->armv7m, s->usart[5], 0x40011400, 71, errp) < 0) return; // USART6
    if(stm32_realize_peripheral(&s->armv7m, s->usart[6], 0x40007800, 82, errp) < 0) return; // UART7
    if(stm32_realize_peripheral(&s->armv7m, s->usart[7], 0x40007C00, 83, errp) < 0) return; // UART8

    for(i = 0; i < STM32FXXX_NUM_TIMERS; i++){
        qdev_prop_set_uint64(DEVICE(s->tim[i]), "clock-frequency", 100000000);
    }

    if(stm32_realize_peripheral(&s->armv7m, s->tim[0], 0x40000000, 28, errp) < 0) return; // TIM2
    if(stm32_realize_peripheral(&s->armv7m, s->tim[1], 0x40000400, 29, errp) < 0) return; // TIM3
    if(stm32_realize_peripheral(&s->armv7m, s->tim[2], 0x40000800, 30, errp) < 0) return; // TIM4
    if(stm32_realize_peripheral(&s->armv7m, s->tim[3], 0x40000C00, 50, errp) < 0) return; // TIM5

    /* ADC 1 to 3 */
    object_property_set_int(OBJECT(s->adc_irqs),"num-lines", STM32FXXX_NUM_ADCS
                            , &err);

    object_property_set_bool(OBJECT(s->adc_irqs), "realized", true , &error_fatal);
    //if (err != NULL) {
    //    error_propagate(errp, err);
    //    return ;
    //}
    qdev_connect_gpio_out(DEVICE(s->adc_irqs), 0, qdev_get_gpio_in(armv7m, 18));

    //if(stm32_realize_peripheral(&s->armv7m, s->adc, 0x40012000, 18, errp) < 0) return; // ADC1 & 2 & 3
 
    static const int spi_irq[] =   { 35, 36, 51, 0, 0, 0 };
    static const uint32_t spi_addr[] =   { 0x40013000, 0x40003800, 0x40003C00,
                                       0x40013400, 0x40015000, 0x40015400 };

    DeviceState *dev;
    SysBusDevice *busdev;

    // Attach SPI
    for (i = 0; i < 5; i++) {
        dev = DEVICE(&(s->spi[i]));
             if (!sysbus_realize(SYS_BUS_DEVICE(&s->spi[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, spi_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, spi_irq[i]));
    }

    //if(stm32_realize_peripheral(&s->armv7m, s->spi[0], 0x40013000, 35, errp) < 0) return;
    //if(stm32_realize_peripheral(&s->armv7m, s->spi[1], 0x40003800, 36, errp) < 0) return;
    //if(stm32_realize_peripheral(&s->armv7m, s->spi[2], 0x40003C00, 51, errp) < 0) return;
    //if(stm32_realize_peripheral(&s->armv7m, s->spi[3], 0x40013400, 84, errp) < 0) return;
    //if(stm32_realize_peripheral(&s->armv7m, s->spi[4], 0x40015000, 85, errp) < 0) return;

    if(stm32_realize_peripheral(&s->armv7m, s->fmc, 0xa0000000, 48, errp) < 0) return;

    if(stm32_realize_peripheral(&s->armv7m, s->pwr, 0x40007000, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->dwt, 0xe0001000, 0, errp) < 0) return;

    if(stm32_realize_peripheral(&s->armv7m, s->gpio[0], 0x40020000, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[1], 0x40020400, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[2], 0x40020800, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[3], 0x40020C00, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[4], 0x40021000, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[5], 0x40021400, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[6], 0x40021800, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[7], 0x40021C00, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[8], 0x40022000, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[9], 0x40022400, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[10], 0x40022800, 0, errp) < 0) return;
}

static Property stm32f4xx_soc_properties[] = {
    DEFINE_PROP_STRING("cpu-type", struct stm32f4xx_soc, cpu_type),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32f4xx_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32f4xx_soc_realize;
    dc->props_ = stm32f4xx_soc_properties;
}

static const TypeInfo stm32f4xx_soc_info = {
    .name          = TYPE_STM32FXXX_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct stm32f4xx_soc),
    .instance_init = stm32f4xx_soc_initfn,
    .class_init    = stm32f4xx_soc_class_init,
};

static void stm32f4xx_soc_types(void)
{
    type_register_static(&stm32f4xx_soc_info);
}

type_init(stm32f4xx_soc_types)
