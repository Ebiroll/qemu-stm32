/*
 * STM32F405 SoC
 *
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
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
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "hw/arm/stm32f405_soc.h"
#include "hw/qdev-clock.h"
#include "hw/misc/unimp.h"
#include "hw/arm/stm32/stm32f2xx_rtc.h"
#include "hw/qdev-core.h"


#define SYSCFG_ADD                     0x40013800
static const uint32_t usart_addr[] = { 0x40011000, 0x40004400, 0x40004800,
                                       0x40004C00, 0x40005000, 0x40011400,
                                       0x40007800, 0x40007C00 };
/* At the moment only Timer 2 to 5 are modelled */
static const uint32_t timer_addr[] = { 0x40000000, 0x40000400,
                                       0x40000800, 0x40000C00 };
static const uint32_t adc_addr[] = { 0x40012000, 0x40012100, 0x40012200,
                                     0x40012300, 0x40012400, 0x40012500 };
static const uint32_t spi_addr[] =   { 0x40013000, 0x40003800, 0x40003C00,
                                       0x40013400, 0x40015000, 0x40015400 };
#define EXTI_ADDR                      0x40013C00

#define SYSCFG_IRQ               71
static const int usart_irq[] = { 37, 38, 39, 52, 53, 71, 82, 83 };
static const int timer_irq[] = { 28, 29, 30, 50 };
#define ADC_IRQ 18
static const int spi_irq[] =   { 35, 36, 51, 0, 0, 0 };
static const int exti_irq[] =  { 6, 7, 8, 9, 10, 23, 23, 23, 23, 23, 40,
                                 40, 40, 40, 40, 40} ;


static void stm32f405_soc_initfn(Object *obj)
{
    STM32F405State *s = STM32F405_SOC(obj);
    int i;

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);

    object_initialize_child(obj, "syscfg", &s->syscfg, TYPE_STM32F4XX_SYSCFG);

    for (i = 0; i < STM_NUM_USARTS; i++) {
        object_initialize_child(obj, "usart[*]", &s->usart[i],
                                TYPE_STM32F2XX_USART);
    }

    for (i = 0; i < STM_NUM_TIMERS; i++) {
        object_initialize_child(obj, "timer[*]", &s->timer[i],
                                TYPE_STM32F2XX_TIMER);
    }

    for (i = 0; i < STM_NUM_ADCS; i++) {
        object_initialize_child(obj, "adc[*]", &s->adc[i], TYPE_STM32F2XX_ADC);

       qdev_prop_set_uint32(DEVICE(&s->adc[i]), "input8" /* Version */,
                815);

       qdev_prop_set_uint32(DEVICE(&s->adc[i]), "input9" /* Length? */,
                815);

    }

    for (i = 0; i < STM_NUM_SPIS; i++) {
        object_initialize_child(obj, "spi[*]", &s->spi[i], TYPE_STM32F2XX_SPI);
    }
    #define NAME_SIZE 32
    char name[NAME_SIZE];

    for (i = 0; i < 5; i++) {
        snprintf(name, NAME_SIZE, "GPIO%c", 'A' + i);
        object_initialize_child(obj, name, &s->gpio[i], TYPE_STM32FXXX_GPIO);
        //qdev_prop_set_uint8(DEVICE(s->gpio[i]), "port_id", i);
        //qdev_prop_set_ptr(DEVICE(s->gpio[i]), "state", &s->state);
    }


    object_initialize_child(obj, "exti", &s->exti, TYPE_STM32F4XX_EXTI);

    object_initialize_child(obj, "i2c", &s->i2c, TYPE_STM32F4XX_I2C);

    

    object_initialize_child(obj, "stm32fxxx-rcc", &s->rcc, TYPE_STM32FXXX_RCC);
    object_initialize_child(obj, "stm32fxxx-rtc", &s->rtc, TYPE_STM32FXXX_RTC);
    object_initialize_child(obj, "stm32fxxx-pwr", &s->pwr, TYPE_STM32FXXX_PWR);



    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);


}


static int gpio_realize_peripheral(ARMv7MState *cpu, stm32fxxx_gpio *dev, hwaddr base, unsigned int irqnr, Error **errp){
     SysBusDevice *busdev;
     DeviceState *armv7m;

    //object_property_set_bool(OBJECT(dev), "realized", true,  &error_fatal);
    if (!sysbus_realize(SYS_BUS_DEVICE(dev), errp)) {
        return 0;
    }
    busdev = SYS_BUS_DEVICE(dev);
    armv7m = DEVICE(cpu);
    sysbus_mmio_map(busdev, 0, base);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, irqnr));
    return 0;
}

static void stm32f405_soc_realize(DeviceState *dev_soc, Error **errp)
{
    STM32F405State *s = STM32F405_SOC(dev_soc);
    MemoryRegion *system_memory = get_system_memory();
    DeviceState *dev, *armv7m;
    SysBusDevice *busdev;
    Error *err = NULL;
    int i;

    /*
     * We use s->refclk internally and only define it with qdev_init_clock_in()
     * so it is correctly parented and not leaked on an init/deinit; it is not
     * intended as an externally exposed clock.
     */
    if (clock_has_source(s->refclk)) {
        error_setg(errp, "refclk clock must not be wired up by the board code");
        return;
    }

    if (!clock_has_source(s->sysclk)) {
        error_setg(errp, "sysclk clock must be wired up by the board code");
        return;
    }


//create_unimplemented_device("RCC",         0x40023800, 0x400);

    /*
     * TODO: ideally we should model the SoC RCC and its ability to
     * change the sysclk frequency and define different sysclk sources.
     */

    /* The refclk always runs at frequency HCLK / 8 */
    clock_set_mul_div(s->refclk, 8, 1);
    clock_set_source(s->refclk, s->sysclk);

    memory_region_init_rom(&s->flash, OBJECT(dev_soc), "STM32F405.flash",
                           FLASH_SIZE, &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_init_alias(&s->flash_alias, OBJECT(dev_soc),
                             "STM32F405.flash.alias", &s->flash, 0,
                             FLASH_SIZE);

    memory_region_add_subregion(system_memory, FLASH_BASE_ADDRESS, &s->flash);
    memory_region_add_subregion(system_memory, 0, &s->flash_alias);

    memory_region_init_ram(&s->sram, NULL, "STM32F405.sram", SRAM_SIZE,
                           &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(system_memory, SRAM_BASE_ADDRESS, &s->sram);

    memory_region_init_ram(&s->ccm, NULL, "STM32F405.ccm", CCM_SIZE,
                           &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(system_memory, CCM_BASE_ADDRESS, &s->ccm);

    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", 96);
    qdev_prop_set_string(armv7m, "cpu-type", s->cpu_type);
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    qdev_connect_clock_in(armv7m, "cpuclk", s->sysclk);
    qdev_connect_clock_in(armv7m, "refclk", s->refclk);
    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return;
    }

    /* System configuration controller */
    dev = DEVICE(&s->syscfg);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->syscfg), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, SYSCFG_ADD);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, SYSCFG_IRQ));

    /* Attach UART (uses USART registers) and USART controllers */
    for (i = 0; i < STM_NUM_USARTS; i++) {
        dev = DEVICE(&(s->usart[i]));
        qdev_prop_set_chr(dev, "chardev", serial_hd(i));
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->usart[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, usart_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, usart_irq[i]));
    }

    /* Timer 2 to 5 */
    for (i = 0; i < STM_NUM_TIMERS; i++) {
        dev = DEVICE(&(s->timer[i]));
        qdev_prop_set_uint64(dev, "clock-frequency", 1000000000);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, timer_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, timer_irq[i]));
    }

    /* ADC device, the IRQs are ORed together */
    if (!object_initialize_child_with_props(OBJECT(s), "adc-orirq",
                                            &s->adc_irqs, sizeof(s->adc_irqs),
                                            TYPE_OR_IRQ, errp, NULL)) {
        return;
    }
    object_property_set_int(OBJECT(&s->adc_irqs), "num-lines", STM_NUM_ADCS,
                            &error_abort);
    if (!qdev_realize(DEVICE(&s->adc_irqs), NULL, errp)) {
        return;
    }
    qdev_connect_gpio_out(DEVICE(&s->adc_irqs), 0,
                          qdev_get_gpio_in(armv7m, ADC_IRQ));

    for (i = 0; i < STM_NUM_ADCS; i++) {
        dev = DEVICE(&(s->adc[i]));
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->adc[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, adc_addr[i]);
        sysbus_connect_irq(busdev, 0,
                           qdev_get_gpio_in(DEVICE(&s->adc_irqs), i));
    }

    /* SPI devices */
    for (i = 0; i < STM_NUM_SPIS; i++) {

        //snprintf(name, NAME_SIZE, "spi[%d]", i);
        //s->spi[i] = sysbus_create_child_obj(obj, name, "stm32fxxx-spi");
        //qdev_prop_set_ptr(DEVICE(s->spi[i]), "regs", &s->state.SPI[i]);
        //qdev_prop_set_uint8(DEVICE(s->spi[i]), "device_id", i);



        dev = DEVICE(&(s->spi[i]));
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->spi[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, spi_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, spi_irq[i]));
    }
    // create_unimplemented_device("RCC",         0x40023800, 0x400);
    /* RCC Device*/
    dev = DEVICE(&s->rcc);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rcc), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40023800);

    // 0x40002800
    /* RCT Device*/
    dev = DEVICE(&s->rtc);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rtc), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40002800);
    

    // create_unimplemented_device("PWR",         , 0x400);4
    dev = DEVICE(&s->pwr);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->pwr), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40007000);

    // I2C device

 
    dev = DEVICE(&s->i2c);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->i2c), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40005400);
    #define NVIC_I2C1_EV_IRQ 38
    #define NVIC_I2C1_ERR_IRQ 39

    #define QEMU_I2C1_EV_IRQ 72
    #define QEMU_I2C1_ERR_IRQ 73

    // I2C
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c), 0, qdev_get_gpio_in(armv7m, QEMU_I2C1_EV_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c), 1, qdev_get_gpio_in(armv7m, QEMU_I2C1_ERR_IRQ));



    /* EXTI device */
    dev = DEVICE(&s->exti);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->exti), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, EXTI_ADDR);
    for (i = 0; i < 16; i++) {
        sysbus_connect_irq(busdev, i, qdev_get_gpio_in(armv7m, exti_irq[i]));
    }
    for (i = 0; i < 16; i++) {
        qdev_connect_gpio_out(DEVICE(&s->syscfg), i, qdev_get_gpio_in(dev, i));
    }




    // Alarm A
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc), 0, qdev_get_gpio_in(dev, 22));
    // Alarm B
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc), 1, qdev_get_gpio_in(dev, 22));
    // Wake up timer
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc), 2, qdev_get_gpio_in(dev, 17));

#define NVIC_RTC_ALARM_IRQ 41
#define EXTI_LINE_17 17

    sysbus_connect_irq(busdev, EXTI_LINE_17, qdev_get_gpio_in(armv7m, NVIC_RTC_ALARM_IRQ));





     // sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, irqnr));

    if(gpio_realize_peripheral(&s->armv7m, &s->gpio[0], 0x40020000, 0, errp) < 0) return;
    if(gpio_realize_peripheral(&s->armv7m, &s->gpio[1], 0x40020400, 0, errp) < 0) return;
    if(gpio_realize_peripheral(&s->armv7m, &s->gpio[2], 0x40020800, 0, errp) < 0) return;
    if(gpio_realize_peripheral(&s->armv7m, &s->gpio[3], 0x40020C00, 0, errp) < 0) return;
    if(gpio_realize_peripheral(&s->armv7m, &s->gpio[4], 0x40021000, 0, errp) < 0) return;
    //if(stm32_realize_peripheral(&s->armv7m, s->gpio[5], 0x40021400, 0, errp) < 0) return;
    //if(stm32_realize_peripheral(&s->armv7m, s->gpio[6], 0x40021800, 0, errp) < 0) return;
    //if(stm32_realize_peripheral(&s->armv7m, s->gpio[7], 0x40021C00, 0, errp) < 0) return;
    //if(stm32_realize_peripheral(&s->armv7m, s->gpio[8], 0x40022000, 0, errp) < 0) return;
    //if(stm32_realize_peripheral(&s->armv7m, s->gpio[9], 0x40022400, 0, errp) < 0) return;
    //if(stm32_realize_peripheral(&s->armv7m, s->gpio[10], 0x40022800, 0, errp) < 0) return;


    create_unimplemented_device("timer[7]",    0x40001400, 0x400);
    create_unimplemented_device("timer[12]",   0x40001800, 0x400);
    create_unimplemented_device("timer[6]",    0x40001000, 0x400);
    create_unimplemented_device("timer[13]",   0x40001C00, 0x400);
    create_unimplemented_device("timer[14]",   0x40002000, 0x400);
    //create_unimplemented_device("RTC and BKP", 0x40002800, 0x400);
    create_unimplemented_device("WWDG",        0x40002C00, 0x400);
    create_unimplemented_device("IWDG",        0x40003000, 0x400);
    create_unimplemented_device("I2S2ext",     0x40003000, 0x400);
    create_unimplemented_device("I2S3ext",     0x40004000, 0x400);
    //create_unimplemented_device("I2C1",        0x40005400, 0x400);
    create_unimplemented_device("I2C2",        0x40005800, 0x400);
    create_unimplemented_device("I2C3",        0x40005C00, 0x400);
    create_unimplemented_device("CAN1",        0x40006400, 0x400);
    create_unimplemented_device("CAN2",        0x40006800, 0x400);
    //create_unimplemented_device("PWR",         0x40007000, 0x400);
    create_unimplemented_device("DAC",         0x40007400, 0x400);
    create_unimplemented_device("timer[1]",    0x40010000, 0x400);
    create_unimplemented_device("timer[8]",    0x40010400, 0x400);
    create_unimplemented_device("SDIO",        0x40012C00, 0x400);
    create_unimplemented_device("timer[9]",    0x40014000, 0x400);
    create_unimplemented_device("timer[10]",   0x40014400, 0x400);
    create_unimplemented_device("timer[11]",   0x40014800, 0x400);
    //create_unimplemented_device("GPIOA",       0x40020000, 0x400);
    //create_unimplemented_device("GPIOB",       0x40020400, 0x400);
    //create_unimplemented_device("GPIOC",       0x40020800, 0x400);
    //create_unimplemented_device("GPIOD",       0x40020C00, 0x400);
    //create_unimplemented_device("GPIOE",       0x40021000, 0x400);
    create_unimplemented_device("GPIOF",       0x40021400, 0x400);
    create_unimplemented_device("GPIOG",       0x40021800, 0x400);
    create_unimplemented_device("GPIOH",       0x40021C00, 0x400);
    create_unimplemented_device("GPIOI",       0x40022000, 0x400);
    create_unimplemented_device("CRC",         0x40023000, 0x400);
    //create_unimplemented_device("RCC",         0x40023800, 0x400);
    create_unimplemented_device("Flash Int",   0x40023C00, 0x400);
    create_unimplemented_device("BKPSRAM",     0x40024000, 0x400);
    create_unimplemented_device("DMA1",        0x40026000, 0x400);
    create_unimplemented_device("DMA2",        0x40026400, 0x400);
    create_unimplemented_device("Ethernet",    0x40028000, 0x1400);
    create_unimplemented_device("USB OTG HS",  0x40040000, 0x30000);
    create_unimplemented_device("USB OTG FS",  0x50000000, 0x31000);
    create_unimplemented_device("DCMI",        0x50050000, 0x400);
    create_unimplemented_device("RNG",         0x50060800, 0x400);
}

static Property stm32f405_soc_properties[] = {
    DEFINE_PROP_STRING("cpu-type", STM32F405State, cpu_type),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32f405_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32f405_soc_realize;
    device_class_set_props(dc, stm32f405_soc_properties);
    /* No vmstate or reset required: device has no internal state */
}

static const TypeInfo stm32f405_soc_info = {
    .name          = TYPE_STM32F405_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F405State),
    .instance_init = stm32f405_soc_initfn,
    .class_init    = stm32f405_soc_class_init,
};

static void stm32f405_soc_types(void)
{
    type_register_static(&stm32f405_soc_info);
}

type_init(stm32f405_soc_types)
