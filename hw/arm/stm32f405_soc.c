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
#include "migration/vmstate.h"
#include "sysemu/sysemu.h"
#include "hw/arm/stm32f405_soc.h"
#include "hw/qdev-clock.h"
#include "hw/misc/unimp.h"
#include "hw/arm/stm32/stm32f2xx_rtc.h"
#include "hw/qdev-core.h"
#include "hw/irq.h"


static uint32_t asic_ssp_transfer(SSIPeripheral *dev, uint32_t value)
{
    AsicSSPState *s = ASIC_SSP(dev);
    int i;

    for (i = 0; i < 3; i++) {
        if (s->enable[i]) {
            return ssi_transfer(s->bus[i], value);
        }
    }
    return 0;
}

static void asic_ssp_gpio_cs(void *opaque, int line, int level)
{
    AsicSSPState *s = (AsicSSPState *)opaque;
    assert(line >= 0 && line < 3);
    s->enable[line] = !level;
    if (level) {
        qemu_irq_raise(s->gp_out[line]);

    } else {
        qemu_irq_lower(s->gp_out[line]);
    }
}


static void asic_ssp_realize(SSIPeripheral *d, Error **errp)
{
    DeviceState *dev = DEVICE(d);
    AsicSSPState *s = ASIC_SSP(d);

    qdev_init_gpio_in(dev, asic_ssp_gpio_cs, 3);
    qdev_init_gpio_out(dev, s->gp_out, 3);
    s->bus[0] = ssi_create_bus(dev, "ssi0");
    s->bus[1] = ssi_create_bus(dev, "ssi1");
    s->bus[2] = ssi_create_bus(dev, "ssi2");
}

//  STM32F405State *s = STM32F405_SOC(dev_soc);
static void asic_ssp_attach(STM32F405State *s,Error **errp)
{
    //void *spibus;
    void *bus;
    //spibus = qdev_get_child_bus(DEVICE(&s->spi[1]), "ssi");


    bus = qdev_get_child_bus(s->mux, "ssi0");
    if (!qdev_realize(DEVICE(&s->asic[0]), bus, errp)) {
                return;
    }

    bus = qdev_get_child_bus(s->mux, "ssi1");
    if (!qdev_realize(DEVICE(&s->asic[1]), bus, errp)) {
                return;
    }

    bus = qdev_get_child_bus(s->mux, "ssi2");
    if (!qdev_realize(DEVICE(&s->asic[2]), bus, errp)) {
                return;
    }

    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 8,  // PB8
                        qdev_get_gpio_in(s->mux, 0));


    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 9,  // PB9
                        qdev_get_gpio_in(s->mux, 1));


    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 10,  // PB10
                        qdev_get_gpio_in(s->mux, 2));

#if 0

    qdev_connect_gpio_out(DEVICE(s->mux), 0,  
                        qdev_get_gpio_in_named(DEVICE(&s->asic[0]), "cs", 0));


    qdev_connect_gpio_out(DEVICE(s->mux), 1,  
                        qdev_get_gpio_in_named(DEVICE(&s->asic[1]), "cs", 0));


    qdev_connect_gpio_out(DEVICE(s->mux), 2,  
                        qdev_get_gpio_in_named(DEVICE(&s->asic[2]), "cs", 0));

#endif

#if 0
    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 8,  // PB8
                        qdev_get_gpio_in_named(DEVICE(&s->asic[0]), "cs", 0));


    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 9,  // PB9
                        qdev_get_gpio_in_named(DEVICE(&s->asic[1]), "cs", 0));


    qdev_connect_gpio_out(DEVICE(&s->gpio[1]), 10,  // PB10
                        qdev_get_gpio_in_named(DEVICE(&s->asic[2]), "cs", 0));

#endif
    //qdev_connect_gpio_out(sms->mpu->gpio, SPITZ_GPIO_LCDCON_CS,
    //                    qdev_get_gpio_in(sms->mux, 0));
    //qdev_connect_gpio_out(sms->mpu->gpio, SPITZ_GPIO_ADS7846_CS,
    //                    qdev_get_gpio_in(sms->mux, 1));
    //qdev_connect_gpio_out(sms->mpu->gpio, SPITZ_GPIO_MAX1111_CS,
    //                    qdev_get_gpio_in(sms->mux, 2));
    // "cs"
}

#if 0
  { 6, 7, 8, 9, 10, 23, 23, 23, 23, 23, 40,
                                 40, 40, 40, 40, 40} ;



 0 .word     WWDG_IRQHandler                   /* Window WatchDog              */                                        
 1  .word     PVD_IRQHandler                    /* PVD through EXTI Line detection */                        
 2  .word     TAMP_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */            
 3  .word     RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */                      
 4 .word     FLASH_IRQHandler                  /* FLASH                        */                                          
 5 .word     RCC_IRQHandler                    /* RCC                          */                                            
 6 .word     EXTI0_IRQHandler                  /* EXTI Line0                   */                        
 7  .word     EXTI1_IRQHandler                  /* EXTI Line1                   */                          
 8  .word     EXTI2_IRQHandler                  /* EXTI Line2                   */                          
 9 .word     EXTI3_IRQHandler                  /* EXTI Line3                   */                          
 10 .word     EXTI4_IRQHandler                  /* EXTI Line4                   */                          
 11 .word     DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */                  
 12 .word     DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */                   
 13 .word     DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */                   
 14 .word     DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */                   
 15 .word     DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */                   
 16 .word     DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */                   
 17 .word     DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */                   
 18 .word     ADC_IRQHandler                    /* ADC1                         */                   
 19 .word     0               				  /* Reserved                     */                         
 20 .word     0              					  /* Reserved                     */                          
 21 .word     0                                 /* Reserved                     */                          
 22  .word     0                                 /* Reserved                     */                          
 23 .word     EXTI9_5_IRQHandler                /* External Line[9:5]s          */                          
  .word     TIM1_BRK_TIM9_IRQHandler          /* TIM1 Break and TIM9          */         
  .word     TIM1_UP_TIM10_IRQHandler          /* TIM1 Update and TIM10        */         
  .word     TIM1_TRG_COM_TIM11_IRQHandler     /* TIM1 Trigger and Commutation and TIM11 */
  .word     TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */                          
  .word     TIM2_IRQHandler                   /* TIM2                         */                   
  .word     TIM3_IRQHandler                   /* TIM3                         */                   
  .word     TIM4_IRQHandler                   /* TIM4                         */                   
  .word     I2C1_EV_IRQHandler                /* I2C1 Event                   */                          
  .word     I2C1_ER_IRQHandler                /* I2C1 Error                   */                          
  .word     I2C2_EV_IRQHandler                /* I2C2 Event                   */                          
  .word     I2C2_ER_IRQHandler                /* I2C2 Error                   */                            
  .word     SPI1_IRQHandler                   /* SPI1                         */                   
  .word     SPI2_IRQHandler                   /* SPI2                         */                   
  .word     USART1_IRQHandler                 /* USART1                       */                   
  .word     USART2_IRQHandler                 /* USART2                       */                   
  .word     0               				  /* Reserved                     */                   
  .word     EXTI15_10_IRQHandler              /* External Line[15:10]s        */                          
  .word     RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */                 
  .word     OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI line */                       
  .word     0                                 /* Reserved     				        */         
  .word     0                                 /* Reserved       			        */         
  .word     0                                 /* Reserved 					          */
  .word     0                                 /* Reserved                     */                          
  .word     DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */                          
  .word     0                                 /* Reserved                     */                   
  .word     SDIO_IRQHandler                   /* SDIO                         */                   
  .word     TIM5_IRQHandler                   /* TIM5                         */                   
  .word     SPI3_IRQHandler                   /* SPI3                         */                   
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */
  .word     DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */                   
  .word     DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */                   
  .word     DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */                   
  .word     DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */                   
  .word     DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */                   
  .word     0                    			        /* Reserved                     */                   
  .word     0              					          /* Reserved                     */                     
  .word     0              					          /* Reserved                     */                          
  .word     0             					          /* Reserved                     */                          
  .word     0              					          /* Reserved                     */                          
  .word     0              					          /* Reserved                     */                          
  .word     OTG_FS_IRQHandler                 /* USB OTG FS                   */                   
  .word     DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */                   
  .word     DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */                   
  .word     DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */                   
  .word     USART6_IRQHandler                 /* USART6                       */                    
  .word     I2C3_EV_IRQHandler                /* I2C3 event                   */                          
  .word     I2C3_ER_IRQHandler                /* I2C3 error                   */                          
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */                         
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */
  .word     FPU_IRQHandler                    /* FPU                          */
  .word     0                                 /* Reserved                     */                   
  .word     0                                 /* Reserved                     */
  .word     SPI4_IRQHandler                   /* SPI4                         */     
  
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */  
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */    
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FSMC_IRQn                   = 48,     /*!< FSMC global Interrupt                                             */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  CRYP_IRQn                   = 79,     /*!< CRYP crypto global interrupt                                      */
  HASH_RNG_IRQn               = 80,     /*!< Hash and Rng global interrupt                                     */
  FPU_IRQn                    = 81      /*!< FPU global interrupt                                              */
#endif 


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
        //void *bus;
        object_initialize_child(obj, "spi[*]", &s->spi[i], TYPE_STM32F2XX_SPI);
        //bus = qdev_get_child_bus(DEVICE(&s->spi[i]), "ssi");
        //if (i<3) {
        //    object_initialize_child(OBJECT(bus), "nnasic", &s->asic[i], TYPE_NN1002);
        //}
    }
    void *busm;
    //object_initialize_child(obj, "mux", s->mux, TYPE_ASIC_SSP);

    void *spibus;
    //void *bus;
    spibus = qdev_get_child_bus(DEVICE(&s->spi[0]), "ssi");

    s->mux = ssi_create_peripheral(spibus,
                                     TYPE_ASIC_SSP);


    busm = qdev_get_child_bus(s->mux, "ssi0");
    object_initialize_child(OBJECT(busm), "nnasic", &s->asic[0], TYPE_NN1002);
    busm = qdev_get_child_bus(s->mux, "ssi1");     
    object_initialize_child(OBJECT(busm), "nnasic", &s->asic[1], TYPE_NN1002);
    busm = qdev_get_child_bus(s->mux, "ssi2");     
    object_initialize_child(OBJECT(busm), "nnasic", &s->asic[2], TYPE_NN1002);

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
    qdev_prop_set_uint8(armv7m, "num-prio-bits", 4);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
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

        if (i==1) {
#if 0
            BusState* bus;
            bus = qdev_get_child_bus(DEVICE(&s->spi[i]), "ssi");

            s->mux = qdev_new(TYPE_ASIC_SSP);
            if (!qdev_realize(DEVICE(s->mux), bus, errp)) {
                return;
            }
#endif 
        }        


        if (i<3) {        
            //BusState* bus;
            // bus=qdev_get_child_bus(DEVICE(&s->spi[i]), "ssi");
            //bus = qdev_get_child_bus(DEVICE(&s->spi[i]), "ssi");
                                 // 
             //if (!qdev_realize(DEVICE(s->asic[i]), bus, errp)) {
             //   return;
            //}
        }

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


    
    for (i = 0; i < 5; i++) {
        //qdev_prop_set_uint8(DEVICE(&s->gpio[i]), "port_id", i);
        //qdev_prop_set_ptr(DEVICE(s->gpio[i]), "state", &s->state);
        s->gpio[i].port_id = i;

    }


    s->asic[0].asic_num=1;
    s->asic[1].asic_num=2;
    s->asic[2].asic_num=3;

    asic_ssp_attach(s,errp);


    qdev_connect_gpio_out(DEVICE(s->mux), 0,  
                        qdev_get_gpio_in_named(DEVICE(&s->asic[0]), "cs", 0));


    qdev_connect_gpio_out(DEVICE(s->mux), 1,  
                        qdev_get_gpio_in_named(DEVICE(&s->asic[1]), "cs", 0));


    qdev_connect_gpio_out(DEVICE(s->mux), 2,  
                        qdev_get_gpio_in_named(DEVICE(&s->asic[2]), "cs", 0));


    dev = DEVICE(&s->exti);
    //busdev = SYS_BUS_DEVICE(dev);
    qdev_connect_gpio_out(DEVICE(&s->asic[0]), 0, qdev_get_gpio_in(dev, 9));
    qdev_connect_gpio_out(DEVICE(&s->asic[1]), 0, qdev_get_gpio_in(dev, 14));
    qdev_connect_gpio_out(DEVICE(&s->asic[2]), 0, qdev_get_gpio_in(dev, 15));


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
};


static const VMStateDescription vmstate_asic_ssp_regs = {
    .name = "asic-ssp",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(ssidev, AsicSSPState),
        VMSTATE_UINT32_ARRAY(enable, AsicSSPState, 3),
        VMSTATE_END_OF_LIST(),
    }
};


static void asic_ssp_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = asic_ssp_realize;
    k->transfer = asic_ssp_transfer;
    dc->vmsd = &vmstate_asic_ssp_regs;
}

static const TypeInfo asic_ssp_info = {
    .name          = TYPE_ASIC_SSP,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(AsicSSPState),
    .class_init    = asic_ssp_class_init,
};


//static Property stm32f405_soc_properties[] = {
//    DEFINE_PROP_STRING("cpu-type", STM32F405State, cpu_type),
//    DEFINE_PROP_END_OF_LIST(),
//};

static void stm32f405_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32f405_soc_realize;
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
    type_register_static(&asic_ssp_info);
    type_register_static(&stm32f405_soc_info);
}

type_init(stm32f405_soc_types)
