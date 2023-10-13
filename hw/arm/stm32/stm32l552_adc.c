/*
 * STM32L552 ADC
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
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/qdev-properties-system.h"
#include "hw/arm/stm32/stm32l552_adc.h"

#ifndef STM_ADC_ERR_DEBUG
#define STM_ADC_ERR_DEBUG 5
#endif

#define ADC_CR_ADEN                         ((uint32_t)0x00000001)        /*!<A/D Converter ON / OFF */ 
#define ADC_CR_ADDIS                        ((uint32_t)0x00000002)        /*!<A/D Converter ON / OFF */
#define ADC_CR_ADSTART                      ((uint32_t)0x00000004)        /*!<A/D Conversion Start */
#define ADC_CFGR_ALIGN                      ((uint32_t)0x00000020)        /*!<Data Alignment bit 5 */

#define ADC_CR_ADVREGEN                    ((uint32_t) (1<<28))        /*!<ADC Voltage Regulator Enable */

// ADC_CR_BITS_PROPERTY_RS,
//             ADC_CR_ADVREGEN);




#define DB_PRINT_L(lvl, fmt, args...) do { \
    if (STM_ADC_ERR_DEBUG >= lvl) { \
        qemu_log("%s: " fmt, __func__, ## args); \
    } \
} while (0)

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

static void stm32l552_adc_reset(DeviceState *dev)
{
    STM32L552ADCState *s = STM32L552_ADC(dev);
    int i=0;
    s->adc_isr = 0x00000000;
    s->adc_ier =  0x00000000;
    s->adc_cr = 0x00000000;
    s->adc_cfgr = 0x00000000;
    s->adc_smpr1 = 0x00000000;
    s->adc_smpr2 = 0x00000000;
    s->adc_tr1 = 0x0FFF0000;
    s->adc_tr2 = 0x00000000;
    s->adc_tr3 = 0x00000000;
    s->adc_sqr1 = 0x00000000;
    s->adc_sqr2 = 0x00000000;
    s->adc_sqr3 = 0x00000000;
    s->adc_sqr4 = 0x00000000;
    s->adc_dr = 0x00000000;
    s->adc_jsqr = 0x00000000;
    s->adc_ofr1 = 0x00000000;
    s->adc_ofr2 = 0x00000000;
    s->adc_ofr3 = 0x00000000;
    s->adc_ofr4 = 0x00000000;
    s->adc_jdr1 = 0x00000000;
    s->adc_jdr2 = 0x00000000;
    s->adc_jdr3 = 0x00000000;
    s->adc_jdr4 = 0x00000000;
    s->adc_awd2cr = 0x00000000;
    s->adc_awd3cr = 0x00000000;
    s->adc_difsel = 0x00000000;
    s->adc_calfact = 0x00000000;

    s->adc2_isr = 0x00000000;
    s->adc2_ier =  0x00000000;
    s->adc2_cr = 0x00000000;
    s->adc2_cfgr = 0x00000000;
    s->adc2_smpr1 = 0x00000000;
    s->adc2_smpr2 = 0x00000000;
    s->adc2_tr1 = 0x0FFF0000;
    s->adc2_tr2 = 0x00FF0000;
    s->adc2_tr3 = 0x00FF0000;
    s->adc2_sqr1 = 0x00000000;
    s->adc2_sqr2 = 0x00000000;
    s->adc2_sqr3 = 0x00000000;
    s->adc2_sqr4 = 0x00000000;
    s->adc2_dr = 0x00000000;
    s->adc2_jsqr = 0x00000000;
    s->adc2_ofr1 = 0x00000000;
    s->adc2_ofr2 = 0x00000000;
    s->adc2_ofr3 = 0x00000000;
    s->adc2_ofr4 = 0x00000000;
    s->adc2_jdr1 = 0x00000000;
    s->adc2_jdr2 = 0x00000000;
    s->adc2_jdr3 = 0x00000000;
    s->adc2_jdr4 = 0x00000000;
    s->adc2_awd2cr = 0x00000000;
    s->adc2_awd3cr = 0x00000000;
    s->adc2_difsel = 0x00000000;
    s->adc2_calfact = 0x00000000;

    s->adc_csr = 0x00000000;
    s->adc_ccr = 0x00000000;
    s->adc_cdr = 0x00000000;


// End of conversion

    for (i = 0; i < 16; i++) {
        s->input[i] = s->reset_input[i];
    }

}

#define  ADC_CR1_RES                         ((uint32_t)0x03000000)        /*!<RES[2:0] bits (Resolution)                            */

static uint32_t stm32l552_adc_generate_value(STM32L552ADCState *s,int adc_num)
{
    /* Attempts to fake some ADC values */
    s->adc_dr = s->adc_dr + 7;

    int channel =  s->adc_sqr3 & 0x000000f;


    DB_PRINT("Channel: 0x%d\n", channel);
    s->adc_dr = s->input[channel];

    DB_PRINT("value: 0x%x\n", s->adc_dr);
    int bits = (s->adc_cr & ADC_CR1_RES) >> 24;
    uint32_t result=s->adc_dr;
    if (adc_num==2) {
        bits = (s->adc2_cr & ADC_CR1_RES) >> 24;
        result=s->adc2_dr;
    }


    switch (bits) {
    case 0:
        /* 12-bit */
         result &= 0xFFF;
        break;
    case 1:
        /* 10-bit */
        result &= 0x3FF;
        break;
    case 2:
        /* 8-bit */
        result &= 0xFF;
        break;
    default:
        /* 6-bit */
        result &= 0x3F;
    }

    if (adc_num==1) {
         if (s->adc_cr & ADC_CFGR_ALIGN) {
           result = (s->adc_dr << 1) & 0xFFF0;
         } else {
           result = s->adc_dr;
         }
    }

    if (adc_num==2) {
         if (s->adc2_cr & ADC_CFGR_ALIGN) {
           result = (s->adc_dr << 1) & 0xFFF0;
         } else {
           result = s->adc2_dr;
         }
    }

    return result;
}

static uint64_t stm32l552_adc_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    STM32L552ADCState *s = opaque;

    DB_PRINT("Address: 0x%" HWADDR_PRIx "\n", addr);
/*
    if (addr >= ADC_COMMON_ADDRESS) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: ADC Common Register Unsupported\n", __func__);
    }


            qemu_log_mask(LOG_UNIMP, "%s: " \
                      "Injection ADC is not implemented, the registers are " \
                      "included for compatibility\n", __func__);
        return s->adc_jofr[(addr - ADC_JOFR1) / 4];

*/
    switch (addr) {
    case ADC_ISR:
    {
         DB_PRINT("Vaule: 0x%d\n", s->adc_isr);
        return s->adc_isr;
    }
    case ADC_IER:
        return s->adc_ier;
    case ADC_CR:
        DB_PRINT("Vaule: 0x%d\n", s->adc_cr);

        return s->adc_cr | ADC_CR_ADVREGEN;
    case ADC_CFGR:
        return s->adc_cfgr;
    case ADC_SMPR1:
        return s->adc_smpr1;
    case ADC_SMPR2:
        return s->adc_smpr2;
    case ADC_TR1:
        return s->adc_tr1;
    case ADC_TR2:
        return s->adc_tr2;
    case ADC_TR3:
        return s->adc_tr3;  
    case ADC_SQR1:
        return s->adc_sqr1;
    case ADC_SQR2:
        return s->adc_sqr2;
    case ADC_SQR3:
        return s->adc_sqr3;
    case ADC_SQR4:
        return s->adc_sqr4;
    case ADC_DR:
        if ((s->adc_cr & ADC_CR_ADEN) && (s->adc_cr & ADC_CR_ADSTART)) {
            s->adc_cr ^= ADC_CR_ADSTART;
            return stm32l552_adc_generate_value(s,1);
        } else {
            return 0;
        }
    case ADC_JSQR:
        return s->adc_jsqr;
    case ADC_OFR1:
        return s->adc_ofr1;
    case ADC_OFR2:
        return s->adc_ofr2;
    case ADC_OFR3:
        return s->adc_ofr3;
    case ADC_OFR4:
        return s->adc_ofr4;
    case ADC_JDR1:
        return s->adc_jdr1;
    case ADC_JDR2:
        return s->adc_jdr2;
    case ADC_JDR3:
        return s->adc_jdr3;
    case ADC_JDR4:
        return s->adc_jdr4;
    case ADC_AWD2CR:
        return s->adc_awd2cr;
    case ADC_AWD3CR:
        return s->adc_awd3cr;
    case ADC_DIFSEL:
        return s->adc_difsel;
    case ADC_CALFACT:
        return s->adc_calfact;

    case ADC2_ISR:
        return s->adc2_isr;
    case ADC2_IER:
        return s->adc2_ier;
    case ADC2_CR:
            DB_PRINT("Vaule cr2: 0x%d\n", s->adc2_cr);

        return s->adc2_cr | ADC_CR_ADVREGEN;
    case ADC2_CFGR:
        return s->adc2_cfgr;
    case ADC2_SMPR1:
        return s->adc2_smpr1;
    case ADC2_SMPR2:
        return s->adc2_smpr2;
    case ADC2_TR1:
        return s->adc2_tr1;
    case ADC2_TR2:
        return s->adc2_tr2;
    case ADC2_TR3:
        return s->adc2_tr3;
    case ADC2_SQR1:
        return s->adc2_sqr1;
    case ADC2_SQR2:
        return s->adc2_sqr2;
    case ADC2_SQR3:
        return s->adc2_sqr3;
    case ADC2_SQR4:
        return s->adc2_sqr4;
    case ADC2_DR:
        if ((s->adc2_cr & ADC_CR_ADEN) && (s->adc2_cr & ADC_CR_ADSTART)) {
            s->adc2_cr ^= ADC_CR_ADSTART;
            return stm32l552_adc_generate_value(s,2);
        } else {
            return 0;
        }
    case ADC2_JSQR:
        return s->adc2_jsqr;
    case ADC2_OFR1:
        return s->adc2_ofr1;
    case ADC2_OFR2:
        return s->adc2_ofr2;
    case ADC2_OFR3:
        return s->adc2_ofr3;
    case ADC2_OFR4:
        return s->adc2_ofr4;
    case ADC2_JDR1:
        return s->adc2_jdr1;
    case ADC2_JDR2:
        return s->adc2_jdr2;
    case ADC2_JDR3:
        return s->adc2_jdr3;
    case ADC2_JDR4:
        return s->adc2_jdr4;
    case ADC2_AWD2CR:
        return s->adc2_awd2cr;
    case ADC2_AWD3CR:
        return s->adc2_awd3cr;
    case ADC2_DIFSEL:
        return s->adc2_difsel;
    case ADC2_CALFACT:
        return s->adc2_calfact;
    case ADC_CSR:
        return s->adc_csr;
    case ADC_CCR:
        return s->adc_ccr;
    case ADC_CDR:
        return s->adc_cdr;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    }   


    return 0;
}

static void stm32l552_adc_write(void *opaque, hwaddr addr,
                       uint64_t val64, unsigned int size)
{
    STM32L552ADCState *s = opaque;
    uint32_t value = (uint32_t) val64;

    DB_PRINT("Address: 0x%" HWADDR_PRIx ", Value: 0x%x\n",
             addr, value);

/*
    if (addr >= 0x100) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: ADC Common Register Unsupported\n", __func__);
    }
*/

    switch (addr) {
        case ADC_ISR:
            s->adc_isr &= (value & 0x3F);
            break;
        case ADC_IER:
            s->adc_ier = value;
            break;
        case ADC_CR:
            s->adc_cr = value | ADC_CR_ADVREGEN;
            break;
        case ADC_CFGR:
            s->adc_cfgr = value;
            break;
        case ADC_SMPR1:
            s->adc_smpr1 = value;
            break;
        case ADC_SMPR2:
            s->adc_smpr2 = value;
            break;
        case ADC_TR1:
            s->adc_tr1 = value;
            break;  
        case ADC_TR2:
            s->adc_tr2 = value;
            break;
        case ADC_TR3:
            s->adc_tr3 = value;
            break;
        case ADC_SQR1:
            s->adc_sqr1 = value;
            break;
        case ADC_SQR2:
            s->adc_sqr2 = value;
            break;
        case ADC_SQR3:
            s->adc_sqr3 = value;
            break;
        case ADC_SQR4:  
            s->adc_sqr4 = value;
            break;
        case ADC_DR:
            s->adc_dr = value;
            break;
        case ADC_JSQR:
            s->adc_jsqr = value;
            break;
        case ADC_OFR1:
            s->adc_ofr1 = value;
            break;
        case ADC_OFR2:
            s->adc_ofr2 = value;
            break;
        case ADC_OFR3:
            s->adc_ofr3 = value;
            break;
        case ADC_OFR4:
            s->adc_ofr4 = value;
            break;
        case ADC_JDR1:  
            s->adc_jdr1 = value;
            break;
        case ADC_JDR2:
            s->adc_jdr2 = value;
            break;
        case ADC_JDR3:
            s->adc_jdr3 = value;
            break;
        case ADC_JDR4:
            s->adc_jdr4 = value;
            break;
        case ADC_AWD2CR:
            s->adc_awd2cr = value;
            break;
        case ADC_AWD3CR:
            s->adc_awd3cr = value;
            break;
        case ADC_DIFSEL:
            s->adc_difsel = value;
            break;
        case ADC_CALFACT:
            s->adc_calfact = value;
            break;            
        case ADC2_ISR:
            s->adc2_isr &= (value & 0x3F);
            break;
        case ADC2_IER:
            s->adc2_ier = value;
            break;
        case ADC2_CR:
            s->adc2_cr = value;
            break;
        case ADC2_CFGR:
            s->adc2_cfgr = value;
            break;
        case ADC2_SMPR1:
            s->adc2_smpr1 = value;
            break;
        case ADC2_SMPR2:
            s->adc2_smpr2 = value;
            break;
        case ADC2_TR1:
            s->adc2_tr1 = value;
            break;
        case ADC2_TR2:
            s->adc2_tr2 = value;
            break;
        case ADC2_TR3:
           s->adc2_tr3 = value;
            break;
        case ADC2_SQR1:
            s->adc2_sqr1 = value;
            break;
        case ADC2_SQR2:
            s->adc2_sqr2 = value;
            break;
        case ADC2_SQR3:
            s->adc2_sqr3 = value;
            break;
        case ADC2_SQR4:
            s->adc2_sqr4 = value;
            break;
        case ADC2_DR:
            s->adc2_dr = value;
            break;
        case ADC2_JSQR:
            s->adc2_jsqr = value;
            break;
        case ADC2_OFR1:
            s->adc2_ofr1 = value;
            break;
        case ADC2_OFR2:
            s->adc2_ofr2 = value;
            break;
        case ADC2_OFR3:
            s->adc2_ofr3 = value;
            break;
        case ADC2_OFR4:
            s->adc2_ofr4 = value;
            break;
        case ADC2_JDR1:
            s->adc2_jdr1 = value;
            break;
        case ADC2_JDR2:
            s->adc2_jdr2 = value;
            break;
        case ADC2_JDR3:
            s->adc2_jdr3 = value;
            break;
        case ADC2_JDR4:
            s->adc2_jdr4 = value;
            break;
        case ADC2_AWD2CR:
            s->adc2_awd2cr = value;
            break;
        case ADC2_AWD3CR:
            s->adc2_awd3cr = value;
            break;
        case ADC2_DIFSEL:
            s->adc2_difsel = value;
            break;
        case ADC2_CALFACT:
            s->adc2_calfact = value;
            break;
        case ADC_CSR:
            s->adc_csr = value;
            break;
        case ADC_CCR:
            s->adc_ccr = value;
            break;
        case ADC_CDR:
            s->adc_cdr = value;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    }

}

static const MemoryRegionOps stm32l552_adc_ops = {
    .read = stm32l552_adc_read,
    .write = stm32l552_adc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const VMStateDescription vmstate_stm32l552_adc = {
    .name = TYPE_STM32L552_ADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        //VMSTATE_UINT32(adc_sr, STM32L552ADCState),
        //VMSTATE_UINT32(adc_cr1, STM32L552ADCState),
        //VMSTATE_UINT32(adc_cr2, STM32L552ADCState),
        //VMSTATE_UINT32(adc_smpr1, STM32L552ADCState),
        //VMSTATE_UINT32(adc_smpr2, STM32L552ADCState),
        //VMSTATE_UINT32_ARRAY(adc_jofr, STM32L552ADCState, 4),
        //VMSTATE_UINT32(adc_htr, STM32L552ADCState),
        //VMSTATE_UINT32(adc_ltr, STM32L552ADCState),
        //VMSTATE_UINT32(adc_sqr1, STM32L552ADCState),
        //VMSTATE_UINT32(adc_sqr2, STM32L552ADCState),
        //VMSTATE_UINT32(adc_sqr3, STM32L552ADCState),
        //VMSTATE_UINT32(adc_jsqr, STM32L552ADCState),
        //VMSTATE_UINT32_ARRAY(adc_jdr, STM32L552ADCState, 4),
        VMSTATE_UINT32(adc_dr, STM32L552ADCState),
        VMSTATE_UINT32(adc2_dr, STM32L552ADCState),
        VMSTATE_UINT32_ARRAY(input, STM32L552ADCState, 16),
        VMSTATE_END_OF_LIST()
    }
};


static Property adc_properties[] = {
    /* Reset values for ADC inputs */
    DEFINE_PROP_UINT32("input7", STM32L552ADCState, reset_input[7], 0xf0),
    DEFINE_PROP_UINT32("input8", STM32L552ADCState, reset_input[8], 0xe0),
    DEFINE_PROP_UINT32("input9", STM32L552ADCState, reset_input[9], 0xd0),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32l552_adc_init(Object *obj)
{
    STM32L552ADCState *s = STM32L552_ADC(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    memory_region_init_io(&s->mmio, obj, &stm32l552_adc_ops, s,
                          TYPE_STM32L552_ADC, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

}

static void stm32l552_adc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    //dc->realize = max1110_realize;
    dc->reset = stm32l552_adc_reset;
    dc->vmsd = &vmstate_stm32l552_adc;
    device_class_set_props(dc, adc_properties);
}

static const TypeInfo stm32f2xx_adc_info = {
    .name          = TYPE_STM32L552_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32L552ADCState),
    .instance_init = stm32l552_adc_init,
    .class_init    = stm32l552_adc_class_init,
};

static void stm32l552_adc_register_types(void)
{
    type_register_static(&stm32f2xx_adc_info);
}

type_init(stm32l552_adc_register_types)
