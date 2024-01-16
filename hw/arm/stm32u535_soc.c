/*
 * STM32U535 SoC
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
#include "hw/arm/stm32u535_soc.h"
#include "hw/qdev-clock.h"
#include "hw/misc/unimp.h"
#include "hw/arm/stm32/stm32l552_rtc.h"
#include "hw/qdev-core.h"
#include "hw/arm/stm32/stm32lxxx_syscfg.h"
#include "hw/arm/asic_sim/nn1002.h"
#include "hw/ssi/ssi.h"
#include "hw/qdev-core.h"


typedef enum IrqNum
{
/* =======================================  ARM Cortex-M33 Specific Interrupt Numbers  ======================================= */
  Reset_IRQn                = -15,    /*!< -15 Reset Vector, invoked on Power up and warm reset              */
  NonMaskableInt_IRQn       = -14,    /*!< -14 Non maskable Interrupt, cannot be stopped or preempted        */
  HardFault_IRQn            = -13,    /*!< -13 Hard Fault, all classes of Fault                              */
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

/* ===========================================  STM32U535xx Specific Interrupt Numbers  ================================= */
  WWDG_IRQn                 = 0,      /*!< Window WatchDog interrupt                                         */
  PVD_PVM_IRQn              = 1,      /*!< PVD/PVM through EXTI Line detection Interrupt                     */
  RTC_IRQn                  = 2,      /*!< RTC non-secure interrupt                                          */
  RTC_S_IRQn                = 3,      /*!< RTC secure interrupt                                              */
  TAMP_IRQn                 = 4,      /*!< Tamper global interrupt                                           */
  RAMCFG_IRQn               = 5,      /*!< RAMCFG global interrupt                                           */
  FLASH_IRQn                = 6,      /*!< FLASH non-secure global interrupt                                 */
  FLASH_S_IRQn              = 7,      /*!< FLASH secure global interrupt                                     */
  GTZC_IRQn                 = 8,      /*!< Global TrustZone Controller interrupt                             */
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
  IWDG_IRQn                 = 27,     /*!< IWDG global interrupt                                             */
  GPDMA1_Channel0_IRQn      = 29,     /*!< GPDMA1 Channel 0 global interrupt                                 */
  GPDMA1_Channel1_IRQn      = 30,     /*!< GPDMA1 Channel 1 global interrupt                                 */
  GPDMA1_Channel2_IRQn      = 31,     /*!< GPDMA1 Channel 2 global interrupt                                 */
  GPDMA1_Channel3_IRQn      = 32,     /*!< GPDMA1 Channel 3 global interrupt                                 */
  GPDMA1_Channel4_IRQn      = 33,     /*!< GPDMA1 Channel 4 global interrupt                                 */
  GPDMA1_Channel5_IRQn      = 34,     /*!< GPDMA1 Channel 5 global interrupt                                 */
  GPDMA1_Channel6_IRQn      = 35,     /*!< GPDMA1 Channel 6 global interrupt                                 */
  GPDMA1_Channel7_IRQn      = 36,     /*!< GPDMA1 Channel 7 global interrupt                                 */
  ADC1_IRQn                 = 37,     /*!< ADC1 global interrupt                                             */
  DAC1_IRQn                 = 38,     /*!< DAC1 global interrupt                                             */
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
  USB_IRQn                  = 73,     /*!< USB global interrupt                                              */
  CRS_IRQn                  = 74,     /*!< CRS global interrupt                                              */
  OCTOSPI1_IRQn             = 76,     /*!< OctoSPI1 global interrupt                                         */
  PWR_S3WU_IRQn             = 77,     /*!< PWR wake up from Stop3 interrupt                                  */
  SDMMC1_IRQn               = 78,     /*!< SDMMC1 global interrupt                                           */
  GPDMA1_Channel8_IRQn      = 80,     /*!< GPDMA1 Channel 8 global interrupt                                 */
  GPDMA1_Channel9_IRQn      = 81,     /*!< GPDMA1 Channel 9 global interrupt                                 */
  GPDMA1_Channel10_IRQn     = 82,     /*!< GPDMA1 Channel 10 global interrupt                                */
  GPDMA1_Channel11_IRQn     = 83,     /*!< GPDMA1 Channel 11 global interrupt                                */
  GPDMA1_Channel12_IRQn     = 84,     /*!< GPDMA1 Channel 12 global interrupt                                */
  GPDMA1_Channel13_IRQn     = 85,     /*!< GPDMA1 Channel 13 global interrupt                                */
  GPDMA1_Channel14_IRQn     = 86,     /*!< GPDMA1 Channel 14 global interrupt                                */
  GPDMA1_Channel15_IRQn     = 87,     /*!< GPDMA1 Channel 15 global interrupt                                */
  I2C3_EV_IRQn              = 88,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn              = 89,     /*!< I2C3 error interrupt                                              */
  SAI1_IRQn                 = 90,     /*!< Serial Audio Interface 1 global interrupt                         */
  TSC_IRQn                  = 92,     /*!< Touch Sense Controller global interrupt                           */
  RNG_IRQn                  = 94,     /*!< RNG global interrupt                                              */
  FPU_IRQn                  = 95,     /*!< FPU global interrupt                                              */
  HASH_IRQn                 = 96,     /*!< HASH global interrupt                                             */
  LPTIM3_IRQn               = 98,     /*!< LPTIM3 global interrupt                                           */
  SPI3_IRQn                 = 99,     /*!< SPI3 global interrupt                                             */
  I2C4_ER_IRQn              = 100,    /*!< I2C4 Error interrupt                                              */
  I2C4_EV_IRQn              = 101,    /*!< I2C4 Event interrupt                                              */
  MDF1_FLT0_IRQn            = 102,    /*!< MDF1 Filter 0 global interrupt                                    */
  MDF1_FLT1_IRQn            = 103,    /*!< MDF1 Filter 1 global interrupt                                    */
  ICACHE_IRQn               = 107,    /*!< Instruction cache global interrupt                                */
  LPTIM4_IRQn               = 110,    /*!< LPTIM4 global interrupt                                           */
  DCACHE1_IRQn              = 111,    /*!< Data cache global interrupt                                       */
  ADF1_IRQn                 = 112,    /*!< ADF interrupt                                                     */
  ADC4_IRQn                 = 113,    /*!< ADC4 (12bits) global interrupt                                    */
  LPDMA1_Channel0_IRQn      = 114,    /*!< LPDMA1 SmartRun Channel 0 global interrupt                        */
  LPDMA1_Channel1_IRQn      = 115,    /*!< LPDMA1 SmartRun Channel 1 global interrupt                        */
  LPDMA1_Channel2_IRQn      = 116,    /*!< LPDMA1 SmartRun Channel 2 global interrupt                        */
  LPDMA1_Channel3_IRQn      = 117,    /*!< LPDMA1 SmartRun Channel 3 global interrupt                        */
  DCMI_PSSI_IRQn            = 119,    /*!< DCMI/PSSI global interrupt                                        */
  CORDIC_IRQn               = 123,    /*!< CORDIC global interrupt                                           */
  FMAC_IRQn                 = 124,    /*!< FMAC global interrupt                                             */
  LSECSSD_IRQn              = 125,    /*!< LSECSSD and MSI_PLL_UNLOCK global interrupts                      */
} IrqNum;



/**
 * create_unimplemented_dual_device: create and map a dummy device
 * @name: name of the device for debug logging
 * @base: base address of the device's MMIO region
 * @size: size of the device's MMIO region
 *
 * This utility function creates and maps an instance of unimplemented-device,
 * which is a dummy device which simply logs all guest accesses to
 * it via the qemu_log LOG_UNIMP debug log.
 * The device is mapped at priority -1000, which means that you can
 * use it to cover a large region and then map other devices on top of it
 * if necessary.
 */
static void create_unimplemented_dual_device(const char *name,
                                               hwaddr base,
                                               hwaddr size)
{
    char sec_name[32];
    DeviceState *dev = qdev_new(TYPE_UNIMPLEMENTED_DEVICE);

    // create_unimplemented_dual_device("DAC",          0x40007400, 0x400);
    // create_unimplemented_dual_device("SEC_DAC",      0x50007400, 0x400);
 

    qdev_prop_set_string(dev, "name", name);
    qdev_prop_set_uint64(dev, "size", size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, base, -1000);


    sprintf(sec_name, "SEC_%s", name);
    dev = qdev_new(TYPE_UNIMPLEMENTED_DEVICE);
    qdev_prop_set_string(dev, "name", sec_name);
    qdev_prop_set_uint64(dev, "size", size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, base + 0x10000000, -1000);

}




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
#if 0
USART1_IRQn               = 61,     /*!< USART1 global interrupt                                           */
  USART3_IRQn               = 63,     /*!< USART3 global interrupt                                           */
  UART4_IRQn                = 64,     /*!< UART4 global interrupt                                            */
  UART5_IRQn                = 65,     /*!< UART5 global interrupt                                            */
#endif

static const int usart_irq[] = { 61, 62, 63, 64, 65};
static const int timer_irq[] = { 44,45,46,47,48,49,50,51 };
#define ADC_IRQ 18
// TODO SPI3 IRQ is 99!
static const int spi_irq[] =   { 59,60 ,61, 0, 0 };
static const int exti_irq[] =  { 11, 12,13,14,15,16,17,18,
                                 19, 20,21,22,23,24,25,26,27} ;

void init_u5_features(Object *obj);

void init_u5_features(Object *obj) {
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
    STM32U535State *s = STM32U535_SOC(obj);
    int i;


  bitband_output_addr[1] = 0x50000000;  // OLAS ACHTUNG Fixme 0x50000000

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);

    object_initialize_child(obj, "syscfg", &s->syscfg, TYPE_STM32LXXX_SYSCFG);

    object_initialize_child(obj, "lpuart1", &s->lpuart1,
                            TYPE_STM32L552_USART);


    for (i = 0; i < STM32U535_NUM_USARTS; i++) {
        object_initialize_child(obj, "usart[*]", &s->usart[i],
                                TYPE_STM32L552_USART);
    }

    for (i = 0; i < STM32U535_NUM_TIMERS; i++) {
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
    

    object_initialize_child(obj, "stm32l552-rcc", &s->rcc, TYPE_STM32U535_RCC);
    object_initialize_child(obj, "stm32fxxx-rtc", &s->rtc, TYPE_STM32L552_RTC);
    object_initialize_child(obj, "stm32u535-usb", &s->usb, TYPE_STM32U535_USB);

    // object_initialize_child(obj, "stm32l552-dmamux", &s->dmamux, TYPE_STM32L552_DMAMUX);

    // 
    object_initialize_child(obj, "stm32-pwr", &s->pwr, TYPE_STM32U535_PWR);



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
    STM32U535State *s = STM32U535_SOC(dev);
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
    STM32U535State *s = STM32U535_SOC(dev_soc);
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

    init_u5_features(OBJECT(s->armv7m.cpu));

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


    // LPuart1 

    dev = DEVICE(&(s->lpuart1));
    qdev_prop_set_chr(dev, "chardev", serial_hd(0));
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->lpuart1), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0,0x46002400);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, LPUART1_IRQn));

    /* Attach UART (uses USART registers) and USART controllers */
    for (i = 0; i < STM32U535_NUM_USARTS; i++) {
        dev = DEVICE(&(s->usart[i]));
        qdev_prop_set_chr(dev, "chardev", serial_hd(i+1));
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->usart[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, usart_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, usart_irq[i]));
    }


    /* Timer 2 to 5 */
    for (i = 0; i < STM32U535_NUM_TIMERS; i++) {
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


    // create_unimplemented_dual_device("RCC",         0x40023800, 0x400);
    /* RCC Device*/
    dev = DEVICE(&s->rcc);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rcc), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x46020C00);

    /* RTC Device*/
    dev = DEVICE(&s->rtc);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rtc), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x46007800);

    /* USB Device*/
    dev = DEVICE(&s->usb);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->usb), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40016000);

#if 0
    /* DMAMux Device*/
    dev = DEVICE(&s->dmamux);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->dmamux), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40020800);
#endif

    ////
    dev = DEVICE(&s->pwr);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->pwr), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x46020800);


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
    sysbus_mmio_map(SYS_BUS_DEVICE(dma1), 0,  0x46025000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 0, qdev_get_gpio_in(armv7m, GPDMA1_Channel1_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 1, qdev_get_gpio_in(armv7m, GPDMA1_Channel2_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 2, qdev_get_gpio_in(armv7m, GPDMA1_Channel3_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 3, qdev_get_gpio_in(armv7m, GPDMA1_Channel4_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 4, qdev_get_gpio_in(armv7m, GPDMA1_Channel5_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 5, qdev_get_gpio_in(armv7m, GPDMA1_Channel6_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 6, qdev_get_gpio_in(armv7m, GPDMA1_Channel7_IRQn));
    sysbus_connect_irq(SYS_BUS_DEVICE(dma1), 7, qdev_get_gpio_in(armv7m, GPDMA1_Channel8_IRQn));


 

    create_unimplemented_dual_device("ADF",          0x46024000, 0x400);

    create_unimplemented_dual_device("GTZC2_MPCBB4", 0x46023800, 0x400);
    create_unimplemented_dual_device("GTZC2_TZIC",   0x46023400, 0x400);
    create_unimplemented_dual_device("GTZC2_TZSC",   0x46023000, 0x400);

    create_unimplemented_dual_device("EXTI", 0x46022000, 0x400);
    create_unimplemented_dual_device("DAC1", 0x46021800, 0x200);
    create_unimplemented_dual_device("ADC4", 0x46021000, 0x400);
    //create_unimplemented_dual_device("RCC", 0x46021000 , 0x200);
    //create_unimplemented_dual_device("PWR", 0x46020800, 0x400);

    create_unimplemented_dual_device("LPGPIO1", 0x46020000, 0x400);

//0x5600 7C00 - 0x5600 7FFF 0x4600 7C00 - 0x4600 7FFF 1 K TAMP TAMP register map XXXX
//0x5600 7800 - 0x5600 7BFF 0x4600 7800 - 0x4600 7BFF 1 K RTC RTC register map XXXX
//0x5600 7400 - 0x5600 77FF 0x4600 7400 - 0x4600 77FF 1 K VREFBUF VREFBUF register map XXXX
//0x5600 5800 - 0x5600 73FF 0x4600 5800 - 0x4600 73FF 7 K Reserved - ----
//0x5600 5400 - 0x5600 57FF 0x4600 5400 - 0x4600 57FF 1 K COMP(1) COMP register map XXXX
//0x5600 5000 - 0x5600 53FF 0x4600 5000 - 0x4600 53FF 1 K OPAMP OPAMP register map XXXX
//0x5600 4C00 - 0x5600 4FFF 0x4600 4C00 - 0x4600 4FFF 1 K LPTIM4
//LPTIM register map
//XXXX

    create_unimplemented_dual_device("TAMP", 0x46007C00, 0x100);
    // create_unimplemented_dual_device("RTC", 0x46007800, 0x400);
    create_unimplemented_dual_device("VREFBUF", 0x46007400, 0x400);
    create_unimplemented_dual_device("COMP", 0x46005400, 0x400);
    create_unimplemented_dual_device("OPAMP", 0x46005000, 0x400);
    create_unimplemented_dual_device("LPTIM4", 0x46004C00, 0x400);


// 0x5600 4800 - 0x5600 4BFF 0x4600 4800 - 0x4600 4BFF 1 K LPTIM3 XXXX
// 0x5600 4400 - 0x5600 47FF 0x4600 4400 - 0x4600 47FF 1 K LPTIM1 XXXX
// 0x5600 2C00 - 0x5600 43FF 0x4600 2C00 - 0x4600 43FF 6 K Reserved - ----
// 0x5600 2800 - 0x5600 2BFF 0x4600 2800 - 0x4600 2BFF 1 K I2C3 I2C register map XXXX
// 0x5600 2400 - 0x5600 27FF 0x4600 2400 - 0x4600 27FF 1 K LPUART1 LPUART register map XXXX
// 0x5600 2000 - 0x5600 23FF 0x4600 2000 - 0x4600 23FF 1 K SPI3 SPI register map XXXX
// 0x5600 0800 - 0x5600 1FFF 0x4600 0800 - 0x4600 1FFF 6 K Reserved - ----
// 0x5600 0400 - 0x5600 07FF 0x4600 0400 - 0x4600 07FF 1 K SYSCFG SYSCFG register map XXXX
// RM0456 Rev 4 145/3637 RM045 150 AHB2

    create_unimplemented_dual_device("LPTIM3", 0x46004800, 0x400);
    create_unimplemented_dual_device("LPTIM1", 0x46004400, 0x400);
    create_unimplemented_dual_device("I2C3", 0x46002800, 0x400);
    // create_unimplemented_dual_device("LPUART1", 0x46002400, 0x400);
    create_unimplemented_dual_device("SPI3", 0x46002000, 0x400);
    create_unimplemented_dual_device("SYSCFG", 0x46000400, 0x400);

/*
0x520D 3800 - 0x5600 03FF 0x420D 3800 - 0x4600 03FF 64.3 M Reserved - ----
0x520D 3400 - 0x520D 37FF 0x420D 3400 - 0x420D 37FF 1 K HSPI1 HSPI register map - - XX
0x520D 2800 - 0x520D 33FF 0x420D 2800 - 0x420D 33FF 3 K Reserved - ----
0x520D 2400 - 0x520D 27FF 0x420D 2400 - 0x420D 27FF 1 K OCTOSPI2
registers OCTOSPI register map - XXX
0x520D 1800 - 0x520D 23FF 0x420D 1800 - 0x420D 23FF 3 K Reserved - ----
0x520D 1400 - 0x520D 17FF 0x420D 1400 - 0x420D 17FF 1 K OCTOSPI1
registers OCTOSPI register map XXXX
0x520D 0800 - 0x520D 13FF 0x420D 0800 - 0x420D 13FF 3 K Reserved - ----
*/

    create_unimplemented_dual_device("HSPI1", 0x420D3400, 0x400);
    create_unimplemented_dual_device("OCTOSPI2", 0x420D2400, 0x400);
    create_unimplemented_dual_device("OCTOSPI1", 0x420D1400, 0x400);

/*
0x520D 0400 - 0x520D 07FF 0x420D 0400 - 0x420D 07FF 1 K FSMC registers FMC register map - XXX
0x520C F800 - 0x520D 03FF 0x420C F800 - 0x420D 03FF 3K Reserved - ----
0x520C F400 - 0x520C F7FF 0x420C F400 - 0x420C F7FF 1 K DLYBOS2
DLYB register map
- XXX
0x520C F000 - 0x520C F3FF 0x420C F000 - 0x420C F3FF 1 K DLYBOS1 X X X X
0x520C 9000 - 0x520C EFFF 0x420C 9000 - 0x420C EFFF 24 K Reserved - ----
0x520C 8C00 - 0x520C 8FFF 0x420C 8C00 - 0x420C 8FFF 1 K SDMMC2 SDMMC register map - XXX
0x520C 8800 - 0x520C 8BFF 0x420C 8800 - 0x420C 8BFF 1 K DLYBSD2
DLYB register map
- XXX
0x520C 8400 - 0x520C 87FF 0x420C 8400 - 0x420C 87FF 1 K DLYBSD1 XXXX
0x520C 8000 - 0x520C 83FF 0x420C 8000 - 0x420C 83FF 1 K SDMMC1 SDMMC register map XXXX
0x520C 5800 - 0x520C 7FFF 0x420C 5800 - 0x420C 7FFF 10 K Reserved - ----
0x520C 5400 - 0x520C 57FF 0x420C 5400 - 0x420C 57FF 1 K OTFDEC2
*/

    create_unimplemented_dual_device("FSMC", 0x420D0400, 0x400);
    create_unimplemented_dual_device("DLYBOS2", 0x420CF400, 0x400);
    create_unimplemented_dual_device("DLYBOS1", 0x420CF000, 0x400);
    create_unimplemented_dual_device("SDMMC2", 0x420C8C00, 0x400);
    create_unimplemented_dual_device("DLYBSD2", 0x420C8800, 0x400);
    create_unimplemented_dual_device("DLYBSD1", 0x420C8400, 0x400);
    create_unimplemented_dual_device("SDMMC1", 0x420C8000, 0x400);
    create_unimplemented_dual_device("OTFDEC2", 0x420C5400, 0x400);

/*
0x520C 5000 - 0x520C 53FF 0x420C 5000 - 0x420C 53FF 1 K OTFDEC1 XXXX
0x520C 4400 - 0x520C 4FFF 0x420C 4400 - 0x420C 4FFF 3 K Reserved - ----
0x520C 4000 - 0x520C 43FF 0x420C 4000 - 0x420C 43FF 1 K OCTOSPIM OCTOSPIM register map - XXX
0x520C 2000 - 0x520C 3FFF 0x420C 2000 - 0x420C 3FFF 8 K PKA PKA register map XXXX
0x520C 1000 - 0x520C 1FFF 0x420C 1000 - 0x420C 1FFF 4 K Reserved - ----
0x520C 0C00 - 0x520C 0FFF 0x420C 0C00 - 0x420C 0FFF 1 K SAES SAES register map XXXX
0x520C 0800 - 0x520C 0BFF 0x420C 0800 - 0x420C 0BFF 1 K RNG RNG register map XXXX
0x520C 0400 - 0x520C 07FF 0x420C 0400 - 0x420C 07FF 1 K HASH HASH register map XXXX
0x520C 0000 - 0x520C 03FF 0x420C 0000 - 0x420C 03FF 1 K AES AES register map XXXX
0x5204 0000 - 0x5205 FFFF 0x4204 0000 - 0x4205 FFFF 128 K OTG_HS OTG_HS register map - - XX
0x5204 0000 - 0x520B FFFF 0x4204 0000 - 0x420B FFFF 512 K OTG_FS OTG_FS register map -X- -
0x5202 C800 - 0x5203 FFFF 0x4202 C800 - 0x4203 FFFF 78 K Reserved - ----
0x5202 C400 - 0x5202 C7FF 0x4202 C400 - 0x4202 C7FF 1 K PSSI PSSI register map XXXX
0x5202 C000 - 0x5202 C3FF 0x4202 C000 - 0x4202 C3FF 1 K DCMI DCMI register map XXXX
0x5202 8400 - 0x5202 BFFF 0x4202 8400 - 0x4202 BFFF 15 K Reserved - ----
0x5202 8000 - 0x5202 83FF 0x4202 8000 - 0x4202 83FF 1 K ADC12(2) ADC register map XXXX
0x5202 2800 - 0x5202 7FFF 0x4202 2800 - 0x4202 7FFF 22 K Reserved - ----

*/
    
    create_unimplemented_dual_device("OTFDEC1", 0x420C5000, 0x400);
    create_unimplemented_dual_device("OCTOSPIM", 0x420C4000, 0x400);
    create_unimplemented_dual_device("PKA", 0x420C2000, 0x400);
    create_unimplemented_dual_device("SAES", 0x420C0C00, 0x400);
    create_unimplemented_dual_device("RNG", 0x420C0800, 0x400);
    create_unimplemented_dual_device("HASH", 0x420C0400, 0x400);
    create_unimplemented_dual_device("AES", 0x420C0000, 0x400);
    create_unimplemented_dual_device("OTG_HS", 0x42040000, 0x10000);
    create_unimplemented_dual_device("OTG_FS", 0x42040000, 0x80000);
    create_unimplemented_dual_device("PSSI", 0x4202C400, 0x400);
    create_unimplemented_dual_device("DCMI", 0x4202C000, 0x400);
    create_unimplemented_dual_device("ADC12", 0x42028000, 0x8000);

/*
0x5202 2400 - 0x5202 27FF 0x4202 2400 - 0x4202 27FF 1 K GPIOJ
GPIO register map
- - XX
0x5202 2000 - 0x5202 23FF 0x4202 2000 - 0x4202 23FF 1 K GPIOI - X X X
0x5202 1C00 - 0x5202 1FFF 0x4202 1C00 - 0x4202 1FFF 1 K GPIOH XXXX
0x5202 1800 - 0x5202 1BFF 0x4202 1800 - 0x4202 1BFF 1 K GPIOG XXXX
0x5202 1400 - 0x5202 17FF 0x4202 1400 - 0x4202 17FF 1 K GPIOF - X X X
0x5202 1000 - 0x5202 13FF 0x4202 1000 - 0x4202 13FF 1 K GPIOE XXXX
0x5202 0C00 - 0x5202 0FFF 0x4202 0C00 - 0x4202 0FFF 1 K GPIOD XXXX
0x5202 0800 - 0x5202 0BFF 0x4202 0800 - 0x4202 0BFF 1 K GPIOC XXXX
0x5202 0400 - 0x5202 07FF 0x4202 0400 - 0x4202 07FF 1 K GPIOB XXXX
0x5202 0000 - 0x5202 03FF 0x4202 0000 - 0x4202 03FF 1 K GPIOA XXXX
AHB1
*/    
    
        create_unimplemented_dual_device("GPIOJ", 0x42022400, 0x400);
        create_unimplemented_dual_device("GPIOI", 0x42022000, 0x400);
        create_unimplemented_dual_device("GPIOH", 0x42021C00, 0x400);
        create_unimplemented_dual_device("GPIOG", 0x42021800, 0x400);
        create_unimplemented_dual_device("GPIOF", 0x42021400, 0x400);
        create_unimplemented_dual_device("GPIOE", 0x42021000, 0x400);
        create_unimplemented_dual_device("GPIOD", 0x42020C00, 0x400);
        create_unimplemented_dual_device("GPIOC", 0x42020800, 0x400);
        create_unimplemented_dual_device("GPIOB", 0x42020400, 0x400);
        create_unimplemented_dual_device("GPIOA", 0x42020000, 0x400);
/*
0x5003 6C00 - 0x5201 FFFF 0x4003 6C00 - 0x4201 FFFF 32.7 M Reserved - ----
0x5003 6400 - 0x5003 6BFF 0x4003 6400 - 0x4003 6BFF 2 K BKPSRAM - XXXX
0x5003 4000 - 0x5003 63FF 0x4003 4000 - 0x4003 63FF 9 K Reserved - ----
Table 6. Memory map and peripheral register boundary addresses (continued)
0x5003 3C00 - 0x5003 3FFF 0x4003 3800 - 0x4003 3BFF 1 K GTZC1_MPCBB6
0x5003 3800 - 0x5003 3BFF 0x4003 3C00 - 0x4003 3FFF 1 K GTZC1_MPCBB5 - - X X
0x5003 3400 - 0x5003 37FF 0x4003 3400 - 0x4003 37FF 1 K GTZC1_MPCBB3 - X X X
0x5003 3000 - 0x5003 33FF 0x4003 3000 - 0x4003 33FF 1 K GTZC1_MPCBB2 XXXX
0x5003 2C00 - 0x5003 2FFF 0x4003 2C00 - 0x4003 2FFF 1 K GTZC1_MPCBB1 XXXX
0x5003 2800 - 0x5003 2BFF 0x4003 2800 - 0x4003 2BFF 1 K GTZC1_TZIC GTZC1 TZIC register map XXXX
0x5003 2400 - 0x5003 27FF 0x4003 2400 - 0x4003 27FF 1 K GTZC1_TZSC GTZC1 TZSC register map XXXX
0x5003 1C00 - 0x5003 23FF 0x4003 1C00 - 0x4003 23FF 2 K Reserved - ----
0x5003 1800 - 0x5003 1BFF 0x4003 1800 - 0x4003 1BFF 1 K DCACHE2
0x5003 1400 - 0x5003 17FF 0x4003 1400 - 0x4003 17FF 1 K DCACHE1 XXXX
0x5003 0800 - 0x5003 13FF 0x4003 0800 - 0x4003 13FF 3 K Reserved - ----
0x5003 0400 - 0x5003 07FF 0x4003 0400 - 0x4003 07FF 1 K ICACHE ICACHE register map XXXX
*/
            
            create_unimplemented_dual_device("BKPSRAM", 0x40036400, 0x400);
            create_unimplemented_dual_device("GTZC1_MPCBB6", 0x40033C00, 0x400);
            create_unimplemented_dual_device("GTZC1_MPCBB5", 0x40033800, 0x400);
            create_unimplemented_dual_device("GTZC1_MPCBB3", 0x40033400, 0x400);
            create_unimplemented_dual_device("GTZC1_MPCBB2", 0x40033000, 0x400);
            create_unimplemented_dual_device("GTZC1_MPCBB1", 0x40032C00, 0x400);
            create_unimplemented_dual_device("GTZC1_TZIC", 0x40032800, 0x400);
            create_unimplemented_dual_device("GTZC1_TZSC", 0x40032400, 0x400);
            create_unimplemented_dual_device("DCACHE2", 0x40031800, 0x400);
            create_unimplemented_dual_device("DCACHE1", 0x40031400, 0x400);
            create_unimplemented_dual_device("ICACHE", 0x40030400, 0x400);

/*
0x5003 0000 - 0x5003 03FF 0x4003 0000 - 0x4003 03FF 18 K Reserved - ----
0x5002 F000 - 0x5002 FFFF 0x4002 F000 - 0x4002 FFFF 1 K GPU2D - - - X X
0x5002 C000 - 0x5002 EFFF 0x4002 C000 - 0x4002 EFFF 1 K GFXMMU GFXMMU register map - - XX
0x5002 BC00 - 0x5002 BFFF 0x4002 BC00 - 0x4002 BFFF 18 K Reserved - ----
0x5002 B000 - 0x5002 BBFF 0x4002 B000 - 0x4002 BBFF 3 K DMA2D DMA2D register map - XXX
0x5002 A000 - 0x5002 AFFF 0x4002 A000 - 0x4002 AFFF 4 K JPEG JPEG codec register map - - -X
0x5002 7000 - 0x5002 AFFF 0x4002 7000 - 0x4002 AFFF 16 K Reserved - ----
0x5002 6000 - 0x5002 6FFF 0x4002 6000 - 0x4002 6FFF 4 K RAMCFG RAMCFG register map XXXX
0x5002 5000 - 0x5002 5FFF 0x4002 5000 - 0x4002 5FFF 4 K MDF1(3) MDF register map XXXX
0x5002 4400 - 0x5002 4FFF 0x4002 4400 - 0x4002 4FFF 3 K Reserved - ----
0x5002 4000 - 0x5002 43FF 0x4002 4000 - 0x4002 43FF 1 K TSC TSC register map XXXX
0x5002 3400 - 0x5002 3FFF 0x4002 3400 - 0x4002 3FFF 3 K Reserved - ----
0x5002 3000 - 0x5002 33FF 0x4002 3000 - 0x4002 33FF 1 K CRC CRC register map XXXX
0x5002 2400 - 0x5002 2FFF 0x4002 2400 - 0x4002 2FFF 3 K Reserved - ----
*/
            
            create_unimplemented_dual_device("GPU2D", 0x4002F000, 0x1000);
            create_unimplemented_dual_device("GFXMMU", 0x4002C000, 0x3000);
            create_unimplemented_dual_device("DMA2D", 0x4002B000, 0xC00);
            create_unimplemented_dual_device("JPEG", 0x4002A000, 0x1000);
            create_unimplemented_dual_device("RAMCFG", 0x40026000, 0x1000);
            create_unimplemented_dual_device("MDF1", 0x40025000, 0x1000);
            create_unimplemented_dual_device("TSC", 0x40024000, 0x400);
            create_unimplemented_dual_device("CRC", 0x40023000, 0x400);

/*  

0x5002 2000 - 0x5002 23FF 0x4002 2000 - 0x4002 23FF 1 K FLASH registers FLASH register map XXXX
0x5002 1800 - 0x5002 1FFF 0x4002 1800 - 0x4002 1FFF 2 K Reserved - ----
0X5002 1400 - 0x5002 17FF 0X4002 1400 - 0x4002 17FF 1 K FMAC FMAC register map XXXX
0X5002 1000 - 0x5002 13FF 0X4002 1000 - 0x4002 13FF 1 K CORDIC CORDIC register map XXXX
0x5002 0000 - 0x5002 0FFF 0x4002 0000 - 0x4002 0FFF 4 K GPDMA1 GPDMA register map XXXX
0x5001 7C00 - 0x5001 FFFF 0x4001 7C00 - 0x4001 FFFF 33 K Reserved - ----
0x5001 6C00 - 0x5001 7BFF 0x4001 6C00 - 0x4001 7BFF 4 K DSI DSI register map - - XX
0x5001 6800 - 0x5001 6BFF 0x4001 6800 - 0x4001 6BFF 1 K LTDC LTDC register map - - XX
0x5001 6400 - 0x5001 67FF 0x4001 6400 - 0x4001 67FF 1 K GFXTIM GFXTIM register map - - -X
0x5001 6400 - 0x5001 6BFF 0x4001 6400 - 0x4001 6BFF 2 K USB RAM - X - - -
0x5001 6000 - 0x5001 63FF 0x4001 6000 - 0x4001 63FF 1 K USB USB register map X- - -
0x5001 5C00 - 0x5001 5FFF 0x4001 5C00 - 0x4001 5FFF 1 K Reserved - ----
0x5001 5800 - 0x5001 5BFF 0x4001 5800 - 0x4001 5BFF 1 K SAI2
0x5001 5400 - 0x5001 57FF 0x4001 5400 - 0x4001 57FF 1 K SAI1 XXXX
0x5001 4C00 - 0x5001 53FF 0x4001 4C00 - 0x4001 53FF 2 K Reserved - ----
0x5001 4800 - 0x5001 4BFF 0x4001 4800 - 0x4001 4BFF 1 K TIM17
TIM16/TIM17 register map XXXX
*/

        // create_unimplemented_dual_device("FLASH", 0x40022000, 0x400);
        create_unimplemented_dual_device("FMAC", 0x40021400, 0x400);
        create_unimplemented_dual_device("CORDIC", 0x40021000, 0x400);
        create_unimplemented_dual_device("GPDMA1", 0x40020000, 0x1000);
        create_unimplemented_dual_device("DSI", 0x40016C00, 0x400);
        create_unimplemented_dual_device("LTDC", 0x40016800, 0x400);
        create_unimplemented_dual_device("GFXTIM", 0x40016400, 0x400);
        //create_unimplemented_dual_device("USB", 0x40016000, 0x400);
        create_unimplemented_dual_device("SAI2", 0x40015800, 0x400);
        create_unimplemented_dual_device("SAI1", 0x40015400, 0x400);
        create_unimplemented_dual_device("TIM17", 0x40014800, 0x400);
/*
0x5001 4400 - 0x5001 47FF 0x4001 4400 - 0x4001 47FF 1 K TIM16 XXXX
0x5001 4000 - 0x5001 43FF 0x4001 4000 - 0x4001 43FF 1 K TIM15 TIM15 register map XXXX
0x5001 3C00 - 0x5001 3FFF 0x4001 3C00 - 0x4001 3FFF 1 K Reserved - ----
0x5001 3800 - 0x5001 3BFF 0x4001 3800 - 0x4001 3BFF 1 K USART1 USART register map XXXX
0x5001 3400 - 0x5001 37FF 0x4001 3400 - 0x4001 37FF 1 K TIM8 TIMx register map XXXX
0x5001 3000 - 0x5001 33FF 0x4001 3000 - 0x4001 33FF 1 K SPI1 SPI register map XXXX
0x5001 2C00 - 0x5001 2FFF 0x4001 2C00 - 0x4001 2FFF 1 K TIM1 TIMx register map XXXX
0x5000 E000 - 0x5001 2BFF 0x4000 E000 - 0x4001 2BFF 19 K Reserved - - - - -
0x5000 DC00 - 0x5000 DFFF 0x4000 DC00 - 0x4000 DFFF 1 K UCPD1 UCPD register map - XXX
0x5000 B000 - 0x5000 DBFF 0x4000 B000 - 0x4000 DBFF 11 K Reserved - ----
0x5000 AC00 - 0x5000 AFFF 0x4000 AC00 - 0x4000 AFFF 1 K FDCAN1 RAM - XXXX
0x5000 A800 - 0x5000 ABFF 0x4000 A800 - 0x4000 ABFF 1 K Reserved - ----
0x5000 A400 - 0x5000 A7FF 0x4000 A400 - 0x4000 A7FF 1 K FDCAN1 FDCAN register map XXXX
0x5000 A000 - 0x5000 A3FF 0x4000 A000 - 0x4000 A3FF 1 K Reserved - ----
*/
        create_unimplemented_dual_device("TIM16", 0x40014400, 0x400);
        create_unimplemented_dual_device("TIM15", 0x40014000, 0x400);
        //create_unimplemented_dual_device("USART1", 0x40013800, 0x400);
        create_unimplemented_dual_device("TIM8", 0x40013400, 0x400);
        create_unimplemented_dual_device("SPI1", 0x40013000, 0x400);
        //create_unimplemented_dual_device("TIM1", 0x40012C00, 0x400);
        create_unimplemented_dual_device("UCPD1", 0x4000DC00, 0x400);
        create_unimplemented_dual_device("FDCAN1", 0x4000AC00, 0x400);
        create_unimplemented_dual_device("FDCAN1", 0x4000A400, 0x400);

        /*
0x5000 9C00 - 0x5000 9FFF 0x4000 9C00 - 0x4000 9FFF 1 K I2C6
0x5000 9800 - 0x5000 9BFF 0x4000 9800 - 0x4000 9BFF 1 K I2C5 - - X X
0x5000 9400 - 0x5000 97FF 0x4000 9400 - 0x4000 97FF 1 K LPTIM2 LPTIM register map XXXX
0x5000 8800 - 0x5000 93FF 0x4000 8800 - 0x4000 93FF 3 K Reserved - ----
0x5000 8400 - 0x5000 87FF 0x4000 8400 - 0x4000 87FF 1 K I2C4 I2C register map XXXX
0x5000 6800 - 0x5000 83FF 0x4000 6800 - 0x4000 83FF 8 K Reserved - ----
0x5000 6400 - 0x5000 67FF 0x4000 6400 - 0x4000 67FF 1 K USART6 USART register map - - XX
0x5000 6000 - 0x5000 63FF 0x4000 6000 - 0x4000 63FF 1 K CRS CRS register map XXXX
0x5000 5C00 - 0x5000 5FFF 0x4000 5C00 - 0x4000 5FFF 1 K Reserved - ----
0x5000 5800 - 0x5000 5BFF 0x4000 5800 - 0x4000 5BFF 1 K I2C2
0x5000 5400 - 0x5000 57FF 0x4000 5400 - 0x4000 57FF 1 K I2C1 XXXX
0x5000 5000 - 0x5000 53FF 0x4000 5000 - 0x4000 53FF 1 K UART5
0x5000 4C00 - 0x5000 4FFF 0x4000 4C00 - 0x4000 4FFF 1 K UART4 XXXX
0x5000 4800 - 0x5000 4BFF 0x4000 4800 - 0x4000 4BFF 1 K USART3 XXXX
0x5000 4400 - 0x5000 47FF 0x4000 4400 - 0x4000 47FF 1 K USART2 - X X X
0x5000 3C00 - 0x5000 43FF 0x4000 3C00 - 0x4000 43FF 2 K Reserved - ----
*/

        create_unimplemented_dual_device("I2C6", 0x40009C00, 0x400);
        create_unimplemented_dual_device("I2C5", 0x40009800, 0x400);
        create_unimplemented_dual_device("LPTIM2", 0x40009400, 0x400);
        create_unimplemented_dual_device("I2C4", 0x40008400, 0x400);
        create_unimplemented_dual_device("USART6", 0x40006400, 0x400);
        create_unimplemented_dual_device("CRS", 0x40006000, 0x400);
        create_unimplemented_dual_device("I2C2", 0x40005800, 0x400);
        create_unimplemented_dual_device("I2C1", 0x40005400, 0x400);
        // create_unimplemented_dual_device("UART5", 0x40005000, 0x400);
        // create_unimplemented_dual_device("UART4", 0x40004C00, 0x400);
        // create_unimplemented_dual_device("USART3", 0x40004800, 0x400);
        // create_unimplemented_dual_device("USART2", 0x40004400, 0x400);

/*
0x5000 3800 - 0x5000 3BFF 0x4000 3800 - 0x4000 3BFF 1 K SPI2 SPI register map XXXX
0x5000 3400 - 0x5000 37FF 0x4000 3400 - 0x4000 37FF 1 K Reserved - ----
0x5000 3000 - 0x5000 33FF 0x4000 3000 - 0x4000 33FF 1 K IWDG IWDG register map XXXX
0x5000 2C00 - 0x5000 2FFF 0x4000 2C00 - 0x4000 2FFF 1 K WWDG WWDG register map XXXX
0x5000 1800 - 0x5000 2BFF 0x4000 1800 - 0x4000 2BFF 5 K Reserved - ----
0x5000 1400 - 0x5000 17FF 0x4000 1400 - 0x4000 17FF 1 K TIM7
0x5000 1000 - 0x5000 13FF 0x4000 1000 - 0x4000 13FF 1 K TIM6 XXXX
0x5000 0C00 - 0x5000 0FFF 0x4000 0C00 - 0x4000 0FFF 1 K TIM5
0x5000 0800 - 0x5000 0BFF 0x4000 0800 - 0x4000 0BFF 1 K TIM4 XXXX
0x5000 0400 - 0x5000 07FF 0x4000 0400 - 0x4000 07FF 1 K TIM3 XXXX
0x5000 0000 - 0x5000 03FF 0x4000 0000 - 0x4000 03FF 1 K TIM2 

*/

        create_unimplemented_dual_device("SPI2", 0x40003800, 0x400);
        create_unimplemented_dual_device("IWDG", 0x40003000, 0x400);
        create_unimplemented_dual_device("WWDG", 0x40002C00, 0x400);
        create_unimplemented_dual_device("TIM7", 0x40001400, 0x400);
        create_unimplemented_dual_device("TIM6", 0x40001000, 0x400);
        create_unimplemented_dual_device("TIM5", 0x40000C00, 0x400);
        create_unimplemented_dual_device("TIM4", 0x40000800, 0x400);
        create_unimplemented_dual_device("TIM3", 0x40000400, 0x400);
        //create_unimplemented_dual_device("TIM2", 0x40000000, 0x400);



#if 0
0x5602 6000 - 0x5FFF FFFF 0x4602 6000 - 0x4FFF FFFF 164 M Reserved - ----
0x5602 5000 - 0x5602 5FFF 0x4602 5000 - 0x4602 5FFF 4 K LPDMA1 LPDMA register map XXXX
0x5602 4000 - 0x5602 4FFF 0x4602 4000 - 0x4602 4FFF 4 K ADF1 ADF register map XXXX
0x5602 3C00 - 0x5602 3FFF 0x4602 3C00 - 0x4602 3FFF 1 K Reserved - ----
0x5602 3800 - 0x5602 3BFF 0x4602 3800 - 0x4602 3BFF 1 K GTZC2_MPCBB4 GTZC2 MPCBB4 register map XXXX
0x5602 3400 - 0x5602 37FF 0x4602 3400 - 0x4602 37FF 1 K GTZC2_TZIC GTZC2 TZIC register map XXXX
0x5602 3000 - 0x5602 33FF 0x4602 3000 - 0x4602 33FF 1 K GTZC2_TZSC GTZC2 TZSC register map XXXX
0x5602 2400 - 0x5602 2FFF 0x4602 2400 - 0x4602 2FFF 3 K Reserved - ----
#endif


}

static Property stm32l552_soc_properties[] = {
    DEFINE_PROP_STRING("cpu-type", STM32U535State, cpu_type),
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
    .name          = TYPE_STM32U535_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32U535State),
    .instance_init = stm32l552_soc_initfn,
    .class_init    = stm32l552_soc_class_init,
};

static void stm32l552_soc_types(void)
{
    type_register_static(&stm32l552_soc_info);
}

type_init(stm32l552_soc_types)
