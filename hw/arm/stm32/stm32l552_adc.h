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

#ifndef HW_STM32L552_ADC_H
#define HW_STM32L552_ADC_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define ADC_ISR    0x00
#define ADC_IER   0x04
#define ADC_CR    0x08
#define ADC_CFGR  0x0C
#define ADC_CFGR2 0x10
#define ADC_SMPR1 0x14
#define ADC_SMPR2 0x18
#define ADC_TR1   0x20
#define ADC_TR2   0x24
#define ADC_TR3   0x28
#define ADC_SQR1  0x30
#define ADC_SQR2  0x34
#define ADC_SQR3  0x38
#define ADC_SQR4  0x3C
#define ADC_DR    0x40
#define ADC_JSQR  0x4C
#define ADC_OFR1  0x60
#define ADC_OFR2  0x64
#define ADC_OFR3  0x68
#define ADC_OFR4  0x6C
#define ADC_JDR1  0x80
#define ADC_JDR2  0x84
#define ADC_JDR3  0x88
#define ADC_JDR4  0x8C
#define ADC_AWD2CR 0xA0
#define ADC_AWD3CR 0xA4
#define ADC_DIFSEL 0xB0
#define ADC_CALFACT 0xB4

#define ADC2_ISR    0x100
#define ADC2_IER   0x104
#define ADC2_CR    0x108
#define ADC2_CFGR  0x10C
#define ADC2_CFGR2 0x110
#define ADC2_SMPR1 0x114
#define ADC2_SMPR2 0x118
#define ADC2_TR1   0x120
#define ADC2_TR2   0x124
#define ADC2_TR3   0x128
#define ADC2_SQR1  0x130
#define ADC2_SQR2  0x134
#define ADC2_SQR3  0x138
#define ADC2_SQR4  0x13C
#define ADC2_DR    0x140
#define ADC2_JSQR  0x14C
#define ADC2_OFR1  0x160
#define ADC2_OFR2  0x164
#define ADC2_OFR3  0x168
#define ADC2_OFR4  0x16C
#define ADC2_JDR1  0x180
#define ADC2_JDR2  0x184
#define ADC2_JDR3  0x188
#define ADC2_JDR4  0x18C
#define ADC2_AWD2CR 0x1A0
#define ADC2_AWD3CR 0x1A4
#define ADC2_DIFSEL 0x1B0
#define ADC2_CALFACT 0x1B4


#define ADC_CSR   0x300
#define ADC_CCR   0x308
#define ADC_CDR   0x30C



#define ADC_SQR2_SQ1_Pos 0
#define ADC_SQR2_SQ1_Msk (0x1f << ADC_SQR2_SQ1_Pos)
#define ADC_SQR2_SQ1(x)  ((x) << ADC_SQR2_SQ1_Pos)
#define ADC_SQR2_SQ2_Pos 5
#define ADC_SQR2_SQ2_Msk (0x1f << ADC_SQR2_SQ2_Pos)


#define ADC_COMMON_ADDRESS 0x300

#define TYPE_STM32L552_ADC "stm32l552-adc"
OBJECT_DECLARE_SIMPLE_TYPE(STM32L552ADCState, STM32L552_ADC)

struct STM32L552ADCState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    uint32_t adc_isr;
    uint32_t adc_ier;
    uint32_t adc_cr;
    uint32_t adc_cfgr;
    uint32_t adc_cfgr2;
    uint32_t adc_smpr1;
    uint32_t adc_smpr2;
    uint32_t adc_tr1;
    uint32_t adc_tr2;
    uint32_t adc_tr3;
    uint32_t adc_sqr1;
    uint32_t adc_sqr2;
    uint32_t adc_sqr3;
    uint32_t adc_sqr4;
    uint32_t adc_dr;
    uint32_t adc_jsqr;
    uint32_t adc_ofr1;
    uint32_t adc_ofr2;
    uint32_t adc_ofr3;
    uint32_t adc_ofr4;
    uint32_t adc_jdr1;
    uint32_t adc_jdr2;
    uint32_t adc_jdr3;
    uint32_t adc_jdr4;
    uint32_t adc_awd2cr;
    uint32_t adc_awd3cr;
    uint32_t adc_difsel;
    uint32_t adc_calfact;

    uint32_t adc2_isr;
    uint32_t adc2_ier;
    uint32_t adc2_cr;
    uint32_t adc2_cfgr;
    uint32_t adc2_cfgr2;
    uint32_t adc2_smpr1;
    uint32_t adc2_smpr2;
    uint32_t adc2_tr1;
    uint32_t adc2_tr2;
    uint32_t adc2_tr3;
    uint32_t adc2_sqr1;
    uint32_t adc2_sqr2;
    uint32_t adc2_sqr3;
    uint32_t adc2_sqr4;
    uint32_t adc2_dr;
    uint32_t adc2_jsqr;
    uint32_t adc2_ofr1;
    uint32_t adc2_ofr2;
    uint32_t adc2_ofr3;
    uint32_t adc2_ofr4;
    uint32_t adc2_jdr1;
    uint32_t adc2_jdr2;
    uint32_t adc2_jdr3;
    uint32_t adc2_jdr4;
    uint32_t adc2_awd2cr;
    uint32_t adc2_awd3cr;
    uint32_t adc2_difsel;
    uint32_t adc2_calfact;

    uint32_t adc_csr;
    uint32_t adc_ccr;
    uint32_t adc_cdr;


    uint32_t reset_input[16];
    uint32_t input[16];

    qemu_irq irq;
};

#endif /* HW_STM32L552_ADC_H */
