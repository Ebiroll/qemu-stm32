/*
 * STM32L552 SoC
 *
 * Copyright (c) 2023 Olof Astrand
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
#include "hw/arm/stm32l552_soc.h"
#include "hw/qdev-clock.h"
#include "hw/misc/unimp.h"
#include "hw/arm/stm32/stm32l552_rtc.h"
#include "hw/qdev-core.h"
#include "hw/arm/stm32/stm32lxxx_syscfg.h"
#include "hw/arm/asic_sim/nn1002.h"
#include "hw/ssi/ssi.h"
#include "hw/qdev-core.h"


typedef enum IrqNum {
/* =======================================  ARM Cortex-M33 Specific Interrupt Numbers  ======================================= */
  Reset_IRQn                = -15,    /*!< -15 Reset Vector, invoked on Power up and warm reset             */
  NonMaskableInt_IRQn       = -14,    /*!< -14 Non maskable Interrupt, cannot be stopped or preempted       */
  HardFault_IRQn            = -13,    /*!< -13 Hard Fault, all classes of Fault                             */
  MemoryManagement_IRQn     = -12,    /*!< -12 Memory Management, MPU mismatch, including Access Violation
                                               and No Match                                                  */
  BusFault_IRQn             = -11,    /*!< -11 Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                               related Fault                                                 */
  UsageFault_IRQn           = -10,    /*!< -10 Usage Fault, i.e. Undef Instruction, Illegal State Transition */
  SecureFault_IRQn          =  -9,    /*!< -9  Secure Fault                                                  */
  SVCall_IRQn               =  -5,    /*!< -5  System Service Call via SVC instruction                       */
  DebugMonitor_IRQn         =  -4,    /*!< -4  Debug Monitor                                                 */
  PendSV_IRQn               =  -2,    /*!< -2  Pendable request for system service                           */
  SysTick_IRQn              =  -1,    /*!< -1  System Tick Timer                                             */

/* ===========================================  STM32L552xx Specific Interrupt Numbers  ========================================= */
  WWDG_IRQn                 = 0,      /*!< Window WatchDog interrupt                                         */
  PVD_PVM_IRQn              = 1,      /*!< PVD/PVM1/PVM2/PVM3/PVM4 through EXTI Line detection interrupts    */
  RTC_IRQn                  = 2,      /*!< RTC non-secure interrupts through the EXTI line 17                */
  RTC_S_IRQn                = 3,      /*!< RTC secure interrupts through the EXTI line 18                    */
  TAMP_IRQn                 = 4,      /*!< Tamper non-secure interrupts through the EXTI line 19             */
  TAMP_S_IRQn               = 5,      /*!< Tamper and TimeStamp interrupts through the EXTI line 20          */
  FLASH_IRQn                = 6,      /*!< FLASH non-secure global interrupt                                 */
  FLASH_S_IRQn              = 7,      /*!< FLASH secure global interrupt                                     */
  GTZC_IRQn                 = 8,      /*!< Global TrustZone controller global interrupt                      */
  RCC_IRQn                  = 9,      /*!< RCC non secure global interrupt                                   */
  RCC_S_IRQn                = 10,     /*!< RCC secure global interrupt                                       */
  EXTI0_IRQn                = 11,     /*!< EXTI Line0 interrupt                                              */
  EXTI1_IRQn                = 12,     /*!< EXTI Line1 interrupt                                              */
  EXTI2_IRQn                = 13,     /*!< EXTI Line2 interrupt                                              */
  EXTI3_IRQn                = 14,     /*!< EXTI Line3 interrupt                                              */
  EXTI4_IRQn                = 15,     /*!< EXTI Line4 interrupt                                              */
  EXTI5_IRQn                = 16,     /*!< EXTI Line5 interrupt                                              */
  EXTI6_IRQn                = 17,     /*!< EXTI Line6 interrupt                                              */
  EXTI7_IRQn                = 18,     /*!< EXTI Line7 interrupt                                              */
  EXTI8_IRQn                = 19,     /*!< EXTI Line8 interrupt                                              */
  EXTI9_IRQn                = 20,     /*!< EXTI Line9 interrupt                                              */
  EXTI10_IRQn               = 21,     /*!< EXTI Line10 interrupt                                             */
  EXTI11_IRQn               = 22,     /*!< EXTI Line11 interrupt                                             */
  EXTI12_IRQn               = 23,     /*!< EXTI Line12 interrupt                                             */
  EXTI13_IRQn               = 24,     /*!< EXTI Line13 interrupt                                             */
  EXTI14_IRQn               = 25,     /*!< EXTI Line14 interrupt                                             */
  EXTI15_IRQn               = 26,     /*!< EXTI Line15 interrupt                                             */
  DMAMUX1_IRQn              = 27,     /*!< DMAMUX1 non-secure interrupt                                      */
  DMAMUX1_S_IRQn            = 28,     /*!< DMAMUX1 secure interrupt                                          */
  DMA1_Channel1_IRQn        = 29,     /*!< DMA1 Channel 1 global interrupt                                   */
  DMA1_Channel2_IRQn        = 30,     /*!< DMA1 Channel 2 global interrupt                                   */
  DMA1_Channel3_IRQn        = 31,     /*!< DMA1 Channel 3 global interrupt                                   */
  DMA1_Channel4_IRQn        = 32,     /*!< DMA1 Channel 4 global interrupt                                   */
  DMA1_Channel5_IRQn        = 33,     /*!< DMA1 Channel 5 global interrupt                                   */
  DMA1_Channel6_IRQn        = 34,     /*!< DMA1 Channel 6 global interrupt                                   */
  DMA1_Channel7_IRQn        = 35,     /*!< DMA1 Channel 7 global interrupt                                   */
  DMA1_Channel8_IRQn        = 36,     /*!< DMA1 Channel 8 global interrupt                                   */
  ADC1_2_IRQn               = 37,     /*!< ADC1 & ADC2 global interrupts                                     */
  DAC_IRQn                  = 38,     /*!< DAC global interrupts                                             */
  FDCAN1_IT0_IRQn           = 39,     /*!< FDCAN1 interrupt 0                                                */
  FDCAN1_IT1_IRQn           = 40,     /*!< FDCAN1 interrupt 1                                                */
  TIM1_BRK_IRQn             = 41,     /*!< TIM1 Break interrupt                                              */
  TIM1_UP_IRQn              = 42,     /*!< TIM1 Update interrupt                                             */
  TIM1_TRG_COM_IRQn         = 43,     /*!< TIM1 Trigger and Commutation interrupt                            */
  TIM1_CC_IRQn              = 44,     /*!< TIM1 Capture Compare interrupt                                    */
  TIM2_IRQn                 = 45,     /*!< TIM2 global interrupt                                             */
  TIM3_IRQn                 = 46,     /*!< TIM3 global interrupt                                             */
  TIM4_IRQn                 = 47,     /*!< TIM4 global interrupt                                             */
  TIM5_IRQn                 = 48,     /*!< TIM5 global interrupt                                             */
  TIM6_IRQn                 = 49,     /*!< TIM6 global interrupt                                             */
  TIM7_IRQn                 = 50,     /*!< TIM7 global interrupt                                             */
  TIM8_BRK_IRQn             = 51,     /*!< TIM8 Break interrupt                                              */
  TIM8_UP_IRQn              = 52,     /*!< TIM8 Update interrupt                                             */
  TIM8_TRG_COM_IRQn         = 53,     /*!< TIM8 Trigger and Commutation interrupt                            */
  TIM8_CC_IRQn              = 54,     /*!< TIM8 Capture Compare interrupt                                    */
  I2C1_EV_IRQn              = 55,     /*!< I2C1 Event interrupt                                              */
  I2C1_ER_IRQn              = 56,     /*!< I2C1 Error interrupt                                              */
  I2C2_EV_IRQn              = 57,     /*!< I2C2 Event interrupt                                              */
  I2C2_ER_IRQn              = 58,     /*!< I2C2 Error interrupt                                              */
  SPI1_IRQn                 = 59,     /*!< SPI1 global interrupt                                             */
  SPI2_IRQn                 = 60,     /*!< SPI2 global interrupt                                             */
  USART1_IRQn               = 61,     /*!< USART1 global interrupt                                           */
  USART2_IRQn               = 62,     /*!< USART2 global interrupt                                           */
  USART3_IRQn               = 63,     /*!< USART3 global interrupt                                           */
  UART4_IRQn                = 64,     /*!< UART4 global interrupt                                            */
  UART5_IRQn                = 65,     /*!< UART5 global interrupt                                            */
  LPUART1_IRQn              = 66,     /*!< LPUART1 global interrupt                                          */
  LPTIM1_IRQn               = 67,     /*!< LPTIM1 global interrupt                                           */
  LPTIM2_IRQn               = 68,     /*!< LPTIM2 global interrupt                                           */
  TIM15_IRQn                = 69,     /*!< TIM15 global interrupt                                            */
  TIM16_IRQn                = 70,     /*!< TIM16 global interrupt                                            */
  TIM17_IRQn                = 71,     /*!< TIM17 global interrupt                                            */
  COMP_IRQn                 = 72,     /*!< COMP1 and COMP2 through EXTI Lines interrupts                     */
  USB_FS_IRQn               = 73,     /*!< USB FS global interrupt                                           */
  CRS_IRQn                  = 74,     /*!< CRS global interrupt                                              */
  FMC_IRQn                  = 75,     /*!< FMC global interrupt                                              */
  OCTOSPI1_IRQn             = 76,     /*!< OctoSPI1 global interrupt                                         */
  SDMMC1_IRQn               = 78,     /*!< SDMMC1 global interrupt                                           */
  DMA2_Channel1_IRQn        = 80,     /*!< DMA2 Channel 1 global interrupt                                   */
  DMA2_Channel2_IRQn        = 81,     /*!< DMA2 Channel 2 global interrupt                                   */
  DMA2_Channel3_IRQn        = 82,     /*!< DMA2 Channel 3 global interrupt                                   */
  DMA2_Channel4_IRQn        = 83,     /*!< DMA2 Channel 4 global interrupt                                   */
  DMA2_Channel5_IRQn        = 84,     /*!< DMA2 Channel 5 global interrupt                                   */
  DMA2_Channel6_IRQn        = 85,     /*!< DMA2 Channel 6 global interrupt                                   */
  DMA2_Channel7_IRQn        = 86,     /*!< DMA2 Channel 7 global interrupt                                   */
  DMA2_Channel8_IRQn        = 87,     /*!< DMA2 Channel 8 global interrupt                                   */
  I2C3_EV_IRQn              = 88,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn              = 89,     /*!< I2C3 error interrupt                                              */
  SAI1_IRQn                 = 90,     /*!< Serial Audio Interface 1 global interrupt                         */
  SAI2_IRQn                 = 91,     /*!< Serial Audio Interface 2 global interrupt                         */
  TSC_IRQn                  = 92,     /*!< Touch Sense Controller global interrupt                           */
  RNG_IRQn                  = 94,     /*!< RNG global interrupt                                              */
  FPU_IRQn                  = 95,     /*!< FPU global interrupt                                              */
  HASH_IRQn                 = 96,     /*!< HASH global interrupt                                             */
  LPTIM3_IRQn               = 98,     /*!< LPTIM3 global interrupt                                           */
  SPI3_IRQn                 = 99,     /*!< SPI3 global interrupt                                             */
  I2C4_ER_IRQn              = 100,    /*!< I2C4 Error interrupt                                              */
  I2C4_EV_IRQn              = 101,    /*!< I2C4 Event interrupt                                              */
  DFSDM1_FLT0_IRQn          = 102,    /*!< DFSDM1 Filter 0 global interrupt                                  */
  DFSDM1_FLT1_IRQn          = 103,    /*!< DFSDM1 Filter 1 global interrupt                                  */
  DFSDM1_FLT2_IRQn          = 104,    /*!< DFSDM1 Filter 2 global interrupt                                  */
  DFSDM1_FLT3_IRQn          = 105,    /*!< DFSDM1 Filter 3 global interrupt                                  */
  UCPD1_IRQn                = 106,    /*!< UCPD1 global interrupt                                            */
  ICACHE_IRQn               = 107    /*!< Instruction cache global interrupt                                */
} IrqNum;






#define SYSCFG_ADD                     0x40010000

// LPUART1 0x40008000
static const uint32_t usart_addr[] = { 0x40013800, 
        0x40004400, 0x40004800, 0x40004C00 , 0x40005000 }; 
//                                        , 0x40004800,
//                                       0x40007800, 0x40007C00 };
/* At the moment only Timer 2 to 5 are modelled */
//static const uint32_t timer_addr[] = { 0x40000000, 0x40000400,
//                                       0x40000800, 0x40000C00 };

static const uint32_t timer_addr[] = { 0x40012C00, 0x40000000, 0x40000400,
                                       0x40000800, 0x40000C00 , 0x40001000 , 0x40001400 };


// static const uint32_t adc_addr[] = { 0x42028000 };  // 0x42028000


static const uint32_t spi_addr[] =   { 0x40013000, 0x40003800, 0x40003C00 };

//#define EXTI_ADDR                      0x40013C00
#define   EXTI_ADDR         0x4002F400

#define SYSCFG_IRQ               71
static const int usart_irq[] = { 61, 62, 63, 64, 65};
static const int timer_irq[] = { 44,45,46,47,48,49,50,51 };
#define ADC_IRQ 18
// TODO SPI3 IRQ is 99!
static const int spi_irq[] =   { 59,60 ,61, 0, 0 };
static const int exti_irq[] =  { 11, 12,13,14,15,16,17,18,
                                 19, 20,21,22,23,24,25,26,27} ;

void init_m33_features(Object *obj);

void init_m33_features(Object *obj) {
    ARMCPU *cpu = ARM_CPU(obj);
    set_feature(&cpu->env, ARM_FEATURE_V8);
    set_feature(&cpu->env, ARM_FEATURE_M);
    set_feature(&cpu->env, ARM_FEATURE_M_MAIN);
    unset_feature(&cpu->env, ARM_FEATURE_M_SECURITY);
    set_feature(&cpu->env, ARM_FEATURE_THUMB_DSP);
    //set_feature(&cpu->env,ARM_FEATURE_V8_1M);
    cpu->midr = 0x410fd213; // r0p3 
    cpu->pmsav7_dregion = 16;
    cpu->sau_sregion = 8;
    cpu->isar.mvfr0 = 0x10110021;
    cpu->isar.mvfr1 = 0x11000011;
    cpu->isar.mvfr2 = 0x00000040;
    cpu->isar.id_pfr0 = 0x00000030;
    cpu->isar.id_pfr1 = 0x00000210;
    cpu->isar.id_dfr0 = 0x00200000;
    cpu->id_afr0 = 0x00000000;
    cpu->isar.id_mmfr0 = 0x00101F40;
    cpu->isar.id_mmfr1 = 0x00000000;
    cpu->isar.id_mmfr2 = 0x01000000;
    cpu->isar.id_mmfr3 = 0x00000000;
    cpu->isar.id_isar0 = 0x01101110;
    cpu->isar.id_isar1 = 0x02212000;
    cpu->isar.id_isar2 = 0x20232232;
    cpu->isar.id_isar3 = 0x01111131;
    cpu->isar.id_isar4 = 0x01310132;
    cpu->isar.id_isar5 = 0x00000000;
    cpu->isar.id_isar6 = 0x00000000;
    cpu->clidr = 0x00000000;
    cpu->ctr = 0x8000c000;
    cpu->env.v7m.secure = false;
}

extern hwaddr bitband_output_addr[2];

static void stm32l552_soc_initfn(Object *obj)
{
    STM32L552State *s = STM32L552_SOC(obj);
    int i;


  bitband_output_addr[1] = 0x50000000;  // OLAS ACHTUNG Fixme 0x50000000

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);

    object_initialize_child(obj, "syscfg", &s->syscfg, TYPE_STM32LXXX_SYSCFG);

    for (i = 0; i < STM32L552_NUM_USARTS; i++) {
        object_initialize_child(obj, "usart[*]", &s->usart[i],
                                TYPE_STM32L552_USART);
    }

    for (i = 0; i < STM32L552_NUM_TIMERS; i++) {
        object_initialize_child(obj, "timer[*]", &s->timer[i],
                                TYPE_STM32F2XX_TIMER);
    }

    object_initialize_child(obj, "stm32l552-flash", &s->flash_regs, TYPE_STM32_FLASH_REGS);


    object_initialize_child(obj, TYPE_STM32L552_ADC, &s->adc, TYPE_STM32L552_ADC);


    qdev_prop_set_uint32(DEVICE(&s->adc), "input8" /* Version */,
            815);

    qdev_prop_set_uint32(DEVICE(&s->adc), "input9" /* Length? */,
            815);


    //for (i = 0; i < STM_NUM_ADCS; i++) {
    //    object_initialize_child(obj, "adc[*]", &s->adc[i], TYPE_STM32L552_ADC);


    //   qdev_prop_set_uint32(DEVICE(&s->adc[i]), "input9" /* Version */,
    //            122);

    //}

    for (i = 0; i < STM_NUM_SPIS; i++) {
        object_initialize_child(obj, "spi[*]", &s->spi[i], TYPE_STM32F2XX_SPI);

    if (1==1) {
          void *bus;


        bus = qdev_get_child_bus(DEVICE(&s->spi[i]), "ssi");
        object_initialize_child(OBJECT(bus), "nnasic", &s->asic[i], TYPE_NN1002);
        //qdev_prop_set_uint8(&s->asic, "input1" /* Test */,
        //                    47);
        // s->spi[i].ssi = s->asic;


    }

    //qdev_connect_gpio_out(sms->mpu->gpio, SPITZ_GPIO_MAX1111_CS,
    //                    qdev_get_gpio_in(sms->mux, 2));


 


    }
    #define NAME_SIZE 32
    char name[NAME_SIZE];

    for (i = 0; i < STM_NUM_GPIO; i++) {
        snprintf(name, NAME_SIZE, "GPIO%c", 'A' + i);
        object_initialize_child(obj, name, &s->gpio[i], TYPE_STM32FXXX_GPIO);
        s->gpio[i].port_id = i;
        //qdev_prop_set_uint8(DEVICE(&s->gpio[i]), "port_id", i);
        //qdev_prop_set_ptr(DEVICE(s->gpio[i]), "state", &s->state);

    }


    object_initialize_child(obj, "exti", &s->exti, TYPE_STM32L552_EXTI);
    

    object_initialize_child(obj, "stm32l552-rcc", &s->rcc, TYPE_STM32L552_RCC);
    object_initialize_child(obj, "stm32fxxx-rtc", &s->rtc, TYPE_STM32L552_RTC);
    object_initialize_child(obj, "stm32l552-dmamux", &s->dmamux, TYPE_STM32L552_DMAMUX);

    // 
    object_initialize_child(obj, "stm32-pwr", &s->pwr, TYPE_STM32L552_PWR);



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

    //MemoryRegion *flash_alias = g_new(MemoryRegion, 1);
    //memory_region_init_alias(flash_alias, OBJECT(dev),
    //                         "STM32L552.flash.alias", &dev->flash, 0,
    //                         FLASH_SIZE);

    return 0;
}

static void stm32l552_soc_reset(DeviceState *dev)
 {
    STM32L552State *s = STM32L552_SOC(dev);
    /* Failed attempt to set truszone SCB registers 
       by looking at them in the debugger */
    s->armv7m.cpu->env.v7m.secure = s->armv7m.cpu->env.v7m.secure;
    s->armv7m.cpu->env.v7m.aircr = R_V7M_AIRCR_BFHFNMINS_MASK;
    s->armv7m.cpu->env.v7m.nsacr = 0xcff;
    CPUARMState *env = &s->armv7m.cpu->env;
    env->v7m.fpccr[M_REG_S]=env->v7m.fpccr[M_REG_S];
    env->sau.ctrl=0;
    env->sau.rbar[M_REG_NS]=0;
    s->armv7m.cpu->sau_sregion=0;
    env->sau.rlar[M_REG_NS]=0;
    env->v7m.ccr[M_REG_NS]=0;
    env->v7m.ccr[M_REG_S]=0x211;
    env->v7m.aircr=0x0fa05200;


 }


static void stm32l552_soc_realize(DeviceState *dev_soc, Error **errp)
{
    STM32L552State *s = STM32L552_SOC(dev_soc);
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



    /*
     * TODO: ideally we should model the SoC RCC and its ability to
     * change the sysclk frequency and define different sysclk sources.
     */

    /* The refclk always runs at frequency HCLK / 8 */
    clock_set_mul_div(s->refclk, 8, 1);
    clock_set_source(s->refclk, s->sysclk);

// flash_alias

    memory_region_init_rom(&s->flash, OBJECT(dev_soc), "STM32L552.flash",
                           FLASH_SIZE, &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }


    uint32_t *flash_data = (uint32_t *)memory_region_get_ram_ptr(&s->flash);

    for (int j = 0; j < FLASH_SIZE / sizeof(uint32_t); j++) {
        flash_data[j] = 0xFFFFFFFFU;
    }

    memory_region_set_dirty(&s->flash, 0, FLASH_SIZE);
    //memory_region_flush_rom_device(&s->flash, 0, FLASH_SIZE);
    
    memory_region_init_alias(&s->flash_alias, OBJECT(dev_soc),
                             "STM32L552.flash.alias", &s->flash, 0,
                             FLASH_SIZE);

    memory_region_add_subregion(system_memory, FLASH_BASE_ADDRESS, &s->flash);
    memory_region_add_subregion(system_memory, 0, &s->flash_alias);
/*
"FLASH_REG","40022000"

*/
    /* Flash registers controller */
    dev = DEVICE(&s->flash_regs);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->flash_regs), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40022000);
/// ADC
    dev = DEVICE(&(s->adc));
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->adc), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x42028000);



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

    //sysbus_connect_irq(busdev, 0,
    //                    qdev_get_gpio_in(DEVICE(&s->adc_irqs), i));




    memory_region_init_ram(&s->sram, NULL, "STM32L552.sram", SRAM_SIZE,
                           &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(system_memory, SRAM_BASE_ADDRESS, &s->sram);

    memory_region_init_ram(&s->ccm, NULL, "STM32L552.ccm", CCM_SIZE,
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

    init_m33_features(OBJECT(s->armv7m.cpu));

    qdev_connect_gpio_out(DEVICE(&s->adc_irqs), 0,
                          qdev_get_gpio_in(armv7m, ADC_IRQ));


    /* System configuration controller */
    dev = DEVICE(&s->syscfg);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->syscfg), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, SYSCFG_ADD);
    //sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, SYSCFG_IRQ));

    /* Attach UART (uses USART registers) and USART controllers */
    for (i = 0; i < STM32L552_NUM_USARTS; i++) {
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
    for (i = 0; i < STM32L552_NUM_TIMERS; i++) {
        dev = DEVICE(&(s->timer[i]));
        qdev_prop_set_uint64(dev, "clock-frequency", 1000000000);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, timer_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, timer_irq[i]));
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

        if (1==1) {        
            BusState* bus;
            // bus=qdev_get_child_bus(DEVICE(h3), "ssi");
            bus = qdev_get_child_bus(DEVICE(&s->spi[i]), "ssi");
                                 // 
             if (!qdev_realize(DEVICE(&s->asic[i]), bus, errp)) {
                return;
            }

            
            // ssi
            // s->asic1   = ssi_create_peripheral(s->spi[i].ssi ,TYPE_NN1002);
        }

    }


    // create_unimplemented_device("RCC",         0x40023800, 0x400);
    /* RCC Device*/
    dev = DEVICE(&s->rcc);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rcc), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40021000);

    // 0x40002800
    /* RTC Device*/
    dev = DEVICE(&s->rtc);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rtc), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40002800);
    /* DMAMux Device*/
    dev = DEVICE(&s->dmamux);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->dmamux), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40020800);

    ////
    dev = DEVICE(&s->pwr);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->pwr), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40007000);


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

    // sysbus_connect_irq(busdev, 27, qdev_get_gpio_in(armv7m, 62));


    // Alarm A
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc), 0, qdev_get_gpio_in(dev, 22));
    // Alarm B
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc), 1, qdev_get_gpio_in(dev, 22));
    // Wake up timer
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc), 2, qdev_get_gpio_in(dev, 17));

#define NVIC_RTC_ALARM_IRQ 41
#define EXTI_LINE_17 17

    sysbus_connect_irq(busdev, EXTI_LINE_17, qdev_get_gpio_in(armv7m, NVIC_RTC_ALARM_IRQ));
#if 0
  EXTI8_IRQn                = 19,     /*!< EXTI Line8 interrupt                                              */
  EXTI13_IRQn               = 24,     /*!< EXTI Line13 interrupt      
                                     */
#endif
    // ASIC1 & 2 connect interrupts
    dev = DEVICE(&s->exti);
    busdev = SYS_BUS_DEVICE(dev);
    qdev_connect_gpio_out(DEVICE(&s->asic[0]), 0, qdev_get_gpio_in(dev, 8));
    qdev_connect_gpio_out(DEVICE(&s->asic[1]), 0, qdev_get_gpio_in(dev, 13));
    // This tried to connect interrupts direcly to the CPU, Did not work
    //sysbus_connect_irq(busdev, 8, qdev_get_gpio_in(armv7m, 19));
    //sysbus_connect_irq(busdev, 9, qdev_get_gpio_in(armv7m, 19));




    s->armv7m.cpu->env.v7m.secure = false;
    CPUARMState *env = &s->armv7m.cpu->env;
    env->v7m.secure = false;
    env->v7m.fpccr[M_REG_S]=env->v7m.fpccr[M_REG_S];
    env->sau.ctrl=0;
    env->sau.rbar[M_REG_NS]=0;
    s->armv7m.cpu->sau_sregion=0;
    env->sau.rlar[M_REG_NS]=0;
    env->v7m.ccr[M_REG_NS]=0;
    env->v7m.ccr[M_REG_S]=0x211;
    env->v7m.aircr=0x0fa05200;




    //qdev_prop_set_bit(DEVICE(&s->armv7m.cpu->env), "secure", false);

    //for (i = 0; i < 16; i++) {
    //    qdev_connect_gpio_out(DEVICE(&s->syscfg), i, qdev_get_gpio_in(dev, i));
    //}

    if(gpio_realize_peripheral(&s->armv7m, &s->gpio[0], 0x42020000, 0, errp) < 0) return;
    if(gpio_realize_peripheral(&s->armv7m, &s->gpio[1], 0x42020400, 0, errp) < 0) return;
    if(gpio_realize_peripheral(&s->armv7m, &s->gpio[2], 0x42020800, 0, errp) < 0) return;
    if(gpio_realize_peripheral(&s->armv7m, &s->gpio[3], 0x42020C00, 0, errp) < 0) return;
    if(gpio_realize_peripheral(&s->armv7m, &s->gpio[4], 0x42021000, 0, errp) < 0) return;
    if(gpio_realize_peripheral(&s->armv7m, &s->gpio[5], 0x42021400, 0, errp) < 0) return;
    if(gpio_realize_peripheral(&s->armv7m, &s->gpio[6], 0x42021800, 0, errp) < 0) return;
    if(gpio_realize_peripheral(&s->armv7m, &s->gpio[7], 0x42021C00, 0, errp) < 0) return;
   


    for (i = 0; i < STM_NUM_GPIO; i++) {
        //qdev_prop_set_uint8(DEVICE(&s->gpio[i]), "port_id", i);
        //qdev_prop_set_ptr(DEVICE(s->gpio[i]), "state", &s->state);
        s->gpio[i].port_id = i;

    }

    qdev_connect_gpio_out(DEVICE(&s->gpio[0]), 4,  // PA4
                        qdev_get_gpio_in_named(DEVICE(&s->asic[0]), "cs", 0));


    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 12,  // PB12
                        qdev_get_gpio_in_named(DEVICE(&s->asic[1]), "cs", 0));

    s->asic[0].asic_num=1;
    s->asic[1].asic_num=2;
    s->asic[2].asic_num=3;
/*


DeviceState *stm32_init_periph(DeviceState *dev, stm32_periph_t periph,
                                        hwaddr addr, qemu_irq irq)
{
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);
    if (irq) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
    }
    return dev;
}

////////////////////////////////////////////
 gpio_realize_peripheral(ARMv7MState *cpu, stm32fxxx_gpio *dev, hwaddr base, unsigned int irqnr, Error **errp){
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

    //MemoryRegion *flash_alias = g_new(MemoryRegion, 1);
    //memory_region_init_alias(flash_alias, OBJECT(dev),
    //                         "STM32L552.flash.alias", &dev->flash, 0,
    //                         FLASH_SIZE);

    return 0;

*/
       

    DeviceState *dma1 = qdev_new("l552_dma");    
     if (!sysbus_realize(SYS_BUS_DEVICE(dma1), &error_fatal)) {
       // return 0;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(dma1), 0, 0x40020000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 0, qdev_get_gpio_in(armv7m, DMA1_Channel1_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 1, qdev_get_gpio_in(armv7m, DMA1_Channel2_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 2, qdev_get_gpio_in(armv7m, DMA1_Channel3_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 3, qdev_get_gpio_in(armv7m, DMA1_Channel4_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 4, qdev_get_gpio_in(armv7m, DMA1_Channel5_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 5, qdev_get_gpio_in(armv7m, DMA1_Channel6_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 6, qdev_get_gpio_in(armv7m, DMA1_Channel7_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 7, qdev_get_gpio_in(armv7m, DMA1_Channel8_IRQn));


    DeviceState *dma2 = qdev_new("l552_dma");    
     if (!sysbus_realize(SYS_BUS_DEVICE(dma2), &error_fatal)) {
       // return 0;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(dma2), 0, 0x40020400);

    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 0, qdev_get_gpio_in(armv7m, DMA2_Channel1_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 1, qdev_get_gpio_in(armv7m, DMA2_Channel2_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 2, qdev_get_gpio_in(armv7m, DMA2_Channel3_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 3, qdev_get_gpio_in(armv7m, DMA2_Channel4_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 4, qdev_get_gpio_in(armv7m, DMA2_Channel5_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 5, qdev_get_gpio_in(armv7m, DMA2_Channel6_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 6, qdev_get_gpio_in(armv7m, DMA2_Channel7_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 7, qdev_get_gpio_in(armv7m, DMA2_Channel8_IRQn));

    //stm32_init_periph(dma1, STM32_DMA1, 0x40020000, NULL);
    //sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 0, qdev_get_gpio_in(nvic, STM32_DMA2_STREAM0_IRQ));
    //sysbus_connect_irq(SYS_BUS_DEVICE(dma2), 1, qdev_get_gpio_in(nvic, STM32_DMA2_STREAM1_IRQ));

/*
"Name","Start","End","Length","R","W","X","Volatile","Overlay","Type","Initialized","Byte Source","Source","Comment"
"TIM2_TIM3_TIM4_TIM5_TIM6_TIM7","40000000","400017ff","0x1800","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"TAMP_IWDG_SPI2_SPI3_WWDG_RTC","40002800","40003fff","0x1800","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"I2C1_I2C2_I2C3_CRS_UART5_UART4_USART3_USART2","40004400","400063ff","0x2000","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"I2C4_LPUART1_LPTIM1_OPAMP_DAC_PWR","40007000","400087ff","0x1800","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"LPTIM2_LPTIM3","40009400","40009bff","0x800","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"FDCAN1","4000a400","4000afff","0xc00","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"USB_UCPD1","4000d400","4000dfff","0xc00","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SYSCFG_VREFBUF_COMP","40010000","400103ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SPI1_TIM1_TIM8_USART1","40012c00","40013bff","0x1000","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"TIM15_TIM16_TIM17","40014000","40014bff","0xc00","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SAI1_SAI2","40015400","40015bff","0x800","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"DFSDM1","40016000","400167ff","0x800","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"DMAMUX1_DMA2_DMA1","40020000","40020bff","0xc00","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"RCC","40021000","400213ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"FLASH","40022000","400223ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"CRC","40023000","400233ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"TSC","40024000","400243ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"EXTI","4002f400","4002f7ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"ICache","40030400","400307ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"GTZC_MPCBB1_GTZC_MPCBB2_GTZC_TZIC_GTZC_TZSC","40032400","400333ff","0x1000","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"GPIOA_GPIOB_GPIOC_GPIOD_GPIOE_GPIOF_GPIOG_GPIOH","42020000","42021fff","0x2000","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"ADC","42028000","420282fc","0x2fd","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"ADC_Common","42028300","420283ff","0x100","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"RNG","420c0800","420c0bff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SDMMC1","420c8000","420c83fc","0x3fd","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"FMC","44020000","440203ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"OCTOSPI1","44021000","440213ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_TIM2_SEC_TIM3_SEC_TIM4_SEC_TIM5_SEC_TIM6_SEC_TIM7","50000000","500017ff","0x1800","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_TAMP_SEC_IWDG_SEC_SPI2_SEC_SPI3_SEC_WWDG_SEC_RTC","50002800","50003fff","0x1800","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_I2C1_SEC_I2C2_SEC_I2C3_SEC_CRS_SEC_UART5_SEC_UART4_SEC_USART3_SEC_USART2","50004400","500063ff","0x2000","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_I2C4_SEC_LPUART1_SEC_LPTIM1_SEC_OPAMP_SEC_DAC_SEC_PWR","50007000","500087ff","0x1800","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_LPTIM2_SEC_LPTIM3","50009400","50009bff","0x800","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_FDCAN1","5000a400","5000afff","0xc00","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_USB_SEC_UCPD1","5000d400","5000dfff","0xc00","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_SYSCFG_SEC_VREFBUF_SEC_COMP","50010000","500103ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_SPI1_SEC_TIM1_SEC_TIM8_SEC_USART1","50012c00","50013bff","0x1000","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_TIM15_SEC_TIM16_SEC_TIM17","50014000","50014bff","0xc00","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_SAI1_SEC_SAI2","50015400","50015bff","0x800","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_DFSDM1","50016000","500167ff","0x800","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_DMAMUX1_SEC_DMA2_SEC_DMA1","50020000","50020bff","0xc00","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_RCC","50021000","500213ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_FLASH","50022000","500223ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_CRC","50023000","500233ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_TSC","50024000","500243ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_EXTI","5002f400","5002f7ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_ICache","50030400","500307ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_GTZC_MPCBB1_SEC_GTZC_MPCBB2_SEC_GTZC_TZIC_SEC_GTZC_TZSC","50032400","500333ff","0x1000","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_GPIOA_SEC_GPIOB_SEC_GPIOC_SEC_GPIOD_SEC_GPIOE_SEC_GPIOF_SEC_GPIOG_SEC_GPIOH","52020000","52021fff","0x2000","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_ADC","52028000","520282fc","0x2fd","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_ADC_Common","52028300","520283ff","0x100","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_RNG","520c0800","520c0bff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_SDMMC1","520c8000","520c83fc","0x3fd","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_FMC","54020000","540203ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"SEC_OCTOSPI1","54021000","540213ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"NVIC","e000e100","e000e47c","0x37d","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"DCB","e000ee08","e000ee0c","0x5","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"NVIC_STIR","e000ef00","e000ef04","0x5","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
"DBGMCU","e0044000","e00443ff","0x400","true","true","false","true","false","Default","false","","","Generated by SVD-Loader."
*/

    //create_unimplemented_device("PWR",         0x40007000, 0x400);
                                            //   40014000
    //create_unimplemented_device("tim[3]",      0x40000400, 0x400);
    create_unimplemented_device("sec_tim[3]",  0x50000000, 0x400);
    //create_unimplemented_device("tim[3]",       0x40000400, 0x400);
    create_unimplemented_device("sec_tim[3]",   0x50000400, 0x400);
    //create_unimplemented_device("tim[4]",       0x40000800, 0x400);
    //create_unimplemented_device("sec_tim[4]",   0x50000800, 0x400);
    //create_unimplemented_device("tim[5]",       0x40000C00, 0x400);
    //create_unimplemented_device("sec_tim[5]",   0x50000C00, 0x400);
    //create_unimplemented_device("tim[6]",       0x40001000, 0x400);
    //create_unimplemented_device("sec_tim[6]",   0x50001000, 0x400);
    //create_unimplemented_device("tim[7]",       0x40001400, 0x400);
    //create_unimplemented_device("sec_tim[7]",   0x50001400, 0x400);
    create_unimplemented_device("DAC",          0x40007400, 0x400);
    create_unimplemented_device("SEC_DAC",      0x50007400, 0x400);
    create_unimplemented_device("OPAMP",        0x40007800, 0x400);
    create_unimplemented_device("SEC_OPAMP",    0x50007800, 0x400);
    create_unimplemented_device("timer[8]",      0x40013400, 0x400);
    create_unimplemented_device("sec_timer[8]",  0x50013400, 0x400);
    create_unimplemented_device("GTZC_TZIC",  0x40032800, 0x400);
    create_unimplemented_device("SEC_GTZC_TZIC",  0x50032800, 0x400);
    create_unimplemented_device("GTZC_TZSC",  0x40032400, 0x400);
    create_unimplemented_device("SEC_GTZC_TZSC",  0x50032400, 0x400);
    create_unimplemented_device("WWDG",  0x40002C00, 0x400);
    create_unimplemented_device("SEC_WWDG",  0x50002C00, 0x400);
    //create_unimplemented_device("SYSCFG",  0x40010000, 0x400);
    create_unimplemented_device("SEC_SYSCFG",  0x50010000, 0x400);
    create_unimplemented_device("DBGMCU",  0xE0044000, 0x400);
    create_unimplemented_device("USB",  0x4000D400, 0x800);
    create_unimplemented_device("SECUSB",  0x5000D400, 0x800);
    create_unimplemented_device("OCTOSPI1",  0x44021000, 0x400);
    create_unimplemented_device("SEC_OCTOSPI1",  0x54021000, 0x800);
    create_unimplemented_device("LPUART1",  0x40008000, 0x400);
    create_unimplemented_device("SEC_LPUART1",  0x50008000, 0x400);
    create_unimplemented_device("COMP",  0x40010200, 0x400);
    create_unimplemented_device("LP_COMP",  0x50010200, 0x400);
    create_unimplemented_device("VREFBUF",  0x40010030, 0x1D0);
    create_unimplemented_device("SEC_VREFBUF",  0x50010030, 0x400);
    create_unimplemented_device("TSC",  0x40024000, 0x1D0);
    create_unimplemented_device("SEC_TSC",  0x50024000, 0x400);
    create_unimplemented_device("UCPD1",  0x4000DC00, 0x400);
    create_unimplemented_device("SEC_UCPD1",  0x5000DC00, 0x400);
    create_unimplemented_device("FDCAN1",     0x4000A400, 0x400);
    create_unimplemented_device("SEC_FDCAN1", 0x5000A400, 0x400);
    create_unimplemented_device("CRC",     0x40023000, 0x400);
    create_unimplemented_device("SEC_CRC", 0x50023000, 0x400);
    //create_unimplemented_device("USART1",     0x40013800, 0x400);
    create_unimplemented_device("SEC_USART1", 0x50013800, 0x400);
    //create_unimplemented_device("USART2",     0x40004400, 0x400);
    create_unimplemented_device("SEC_USART2", 0x50004400, 0x400);
    //create_unimplemented_device("USART3",     0x40004800, 0x400);
    create_unimplemented_device("SEC_USART3", 0x50004800, 0x400);
    //create_unimplemented_device("USART4",     0x40004C00, 0x400);
    create_unimplemented_device("SEC_USART4", 0x50005000, 0x400);
    //create_unimplemented_device("USART5",     0x40005000, 0x400);
    create_unimplemented_device("SEC_USART5", 0x50005000, 0x400);
    //create_unimplemented_device("ADC_COMMON",     0x42028300, 0x400);
    create_unimplemented_device("SEC_ADC_Common", 0x52028300, 0x400);
    //create_unimplemented_device("ADC",     0x42028000, 0x400);
    create_unimplemented_device("SEC_ADC", 0x52028000, 0x400);

    //create_unimplemented_device("NVIC",     0xE000E100, 0x37D);
    create_unimplemented_device("NVIC_STIR", 0xE000EF00, 0x400);

    create_unimplemented_device("NVIC_SCB", 0xE000ED00, 0x100);

    //create_unimplemented_device("FMC",     0x44020000, 0x400);
    create_unimplemented_device("SEC_FMC", 0x54020000, 0x400);

    create_unimplemented_device("RNG",         0x420C0800, 0x400);
    create_unimplemented_device("SEC_RNG",     0x520C0800, 0x400);

    create_unimplemented_device("SDMMC1",         0x420C8000, 0x3DF);
    create_unimplemented_device("SEC_SDMMC1",     0x520C8000, 0x400);

// 

/*   create_unimplemented_device("timer[13]",   0x40001C00, 0x400);
    create_unimplemented_device("timer[14]",   0x40002000, 0x400);
    //create_unimplemented_device("RTC and BKP", 0x40002800, 0x400);
    create_unimplemented_device("WWDG",        0x40002C00, 0x400);
    create_unimplemented_device("IWDG",        0x40003000, 0x400);
    create_unimplemented_device("I2S2ext",     0x40003000, 0x400);
    create_unimplemented_device("I2S3ext",     0x40004000, 0x400);
    create_unimplemented_device("I2C1",        0x40005400, 0x400);
    create_unimplemented_device("I2C2",        0x40005800, 0x400);
    create_unimplemented_device("I2C3",        0x40005C00, 0x400);
    create_unimplemented_device("CAN1",        0x40006400, 0x400);
    create_unimplemented_device("CAN2",        0x40006800, 0x400);
    //create_unimplemented_device("PWR",         0x40007000, 0x400);
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
    */
}

static Property stm32l552_soc_properties[] = {
    DEFINE_PROP_STRING("cpu-type", STM32L552State, cpu_type),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32l552_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32l552_soc_realize;
    device_class_set_props(dc, stm32l552_soc_properties);
    dc->reset = stm32l552_soc_reset;
    /* No vmstate or reset required: device has no internal state */
}

static const TypeInfo stm32l552_soc_info = {
    .name          = TYPE_STM32L552_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32L552State),
    .instance_init = stm32l552_soc_initfn,
    .class_init    = stm32l552_soc_class_init,
};

static void stm32l552_soc_types(void)
{
    type_register_static(&stm32l552_soc_info);
}

type_init(stm32l552_soc_types)
