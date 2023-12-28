/*
 * STM32 Microcontroller Power module
 *
 * Copyright (c) 2023 Olof Astrand
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

// 

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
//#include "chardev/char-fe.h"
#include "hw/hw.h"
#include "hw/qdev-properties.h"
#include "hw/arm/stm32/stm32u535_pwr.h"


#define PWR_CR1_VOS 14
#define SMPSBYPRDY (1 << 12)

#define SMPSHPRDY (1 << 15)
#define CR4_SMPSLPEN  (1 << 15) 

#define PWR_TRACE(fmt, ...) fprintf(stderr, "stm32u535_pwr: " fmt, ##__VA_ARGS__)
#define PWR_ERROR(fmt, ...) fprintf(stderr, "stm32u535_pwr: ERROR: " fmt, ##__VA_ARGS__)



#define PWR_VOSR_VOSRDY_Pos                 (15U)
#define PWR_VOSR_VOSRDY_Msk                 (0x1UL << PWR_VOSR_VOSRDY_Pos)          /*!< 0x00008000 */
#define PWR_VOSR_VOSRDY                     PWR_VOSR_VOSRDY_Msk                     /*!< Ready bit for VCORE voltage scaling output selection */


#define PWR_SVMSR_ACTVOSRDY_Pos             (15U)
#define PWR_SVMSR_ACTVOSRDY_Msk             (0x1UL << PWR_SVMSR_ACTVOSRDY_Pos)      /*!< 0x00008000 */
#define PWR_SVMSR_ACTVOSRDY                 PWR_SVMSR_ACTVOSRDY_Msk                 /*!< Voltage level ready for currently used VOS        */

#define PWR_VOSR_BOOSTRDY_Pos               (14U)
#define PWR_VOSR_BOOSTRDY_Msk               (0x1UL << PWR_VOSR_BOOSTRDY_Pos)        /*!< 0x00004000 */
#define PWR_VOSR_BOOSTRDY                   PWR_VOSR_BOOSTRDY_Msk     

static uint64_t stm32u535_pwr_read(void *opaque, hwaddr addr, unsigned int size){
    struct stm32u535_pwr *self = (struct stm32u535_pwr*)opaque;

    // if (addr!=PWR_SR2) PWR_TRACE("read: from register at offset %08x\n", (uint32_t)addr);

    switch(addr){
        case PWR_CR1: {
  
            return self->pwr_cr1 ;
        } break;
        case PWR_CR2: {
            return self->pwr_cr2;
        } break;
        case PWR_CR3: {
            return self->pwr_cr3;
        } break;
        case PWR_CR4: {
            return self->pwr_cr4;
        } break;
        case PWR_CR5: {
            return self->pwr_cr5;
        } break;
        case PWR_VOSR: {
            // clear bit CR4_SMPSLPEN
            self->pwr_vosr &= ~CR4_SMPSLPEN;

            // Set U5 Voltage scaling ready
            self->pwr_vosr |= PWR_VOSR_VOSRDY;

            // Boost ready
            self->pwr_vosr |= PWR_VOSR_BOOSTRDY;
            //self->pwr_cr4 &= ~PWR_VOSR_VOSRDY;
            PWR_TRACE("val %08x\n", (uint32_t) self->pwr_vosr);


            return self->pwr_vosr;
        } break;
        case PWR_SVMSR: {
            // Set U5 Voltage scaling ready
            self->pwr_svmsr |= PWR_SVMSR_ACTVOSRDY |  (0x3UL << 16);
            PWR_TRACE("val %08x\n", (uint32_t) self->pwr_svmsr);

            return self->pwr_svmsr;
        } break;
        //case PWR_SR1: {
            // Clear SMPSBYPRDY and set SMPSHPRDY
          //  self->pwr_sr1 &= ~SMPSBYPRDY;
          //  self->pwr_sr1 |= SMPSHPRDY;
          //  return self->pwr_sr1;   // (self->pwr_sr1 << 32);
        //} break;
        case PWR_PUCRA: {
            return self->pwr_pucra;
        } break;
        case PWR_PDCRA: {
            return self->pwr_pdcra;
        } break;
        case PWR_PUCRB: {
            return self->pwr_pucrb;
        } break;
        case PWR_PDCRB: {
            return self->pwr_pdcrb;
        } break;
        case PWR_PUCRC: {
            return self->pwr_pucrc;
        } break;
        case PWR_PDCRC: {
            return self->pwr_pdcrc;
        } break;
        case PWR_PUCRD: {
            return self->pwr_pucrd;
        } break;
        case PWR_PDCRD: {

            self->pwr_pdcrd |= PWR_SVMSR_ACTVOSRDY;
            PWR_TRACE("val %08x\n", (uint32_t) self->pwr_pdcrd);

            return self->pwr_pdcrd;
        } break;
        case PWR_PUCRE: {
            return self->pwr_pucre;
        } break;
        case PWR_PDCRE: {
            return self->pwr_pdcre;
        } break;
        case PWR_PUCRF: {
            return self->pwr_pucrf;
        } break;
        case PWR_PDCRF: {
            return self->pwr_pdcrf;
        } break;
        case PWR_PUCRG: {
            return self->pwr_pucrg;
        } break;
        case PWR_PDCRG: {
            return self->pwr_pdcrg;
        } break;
        case PWR_PUCRH: {
            return self->pwr_pucrh;
        } break;
        case PWR_PDCRH: {
            return self->pwr_pdcrh;
        } break;
        case PWR_SECCFGR: {
            return self->pwr_seccfgr;
        } break;
        case PWR_PRIVCFGR: {
            return self->pwr_privcfgr;
        } break;



    }
    return 0;
}

#if 0
        case 0x00: {
            return self->state.PWR_CR;
        } break;
        case 0x04: {
            return self->state.PWR_CSR;
        } break;
        default: {
            fprintf(stderr, "PWR: accessing unknown register at offset %08x\n", (uint32_t)addr);
        }
#endif

#if 0
        case 0x00: { // CR
            uint32_t valx = val ^ self->state.PWR_CR;
            if(valx & PWR_CR_UDEN){
                if((val & PWR_CR_UDEN) == PWR_CR_UDEN) {
                    PWR_TRACE("underdrive enable in stop mode: enabled\n");
                    self->state.PWR_CR |= PWR_CR_UDEN;
                    self->state.PWR_CSR |= PWR_CSR_UDRDY;
                } else if((val & PWR_CR_UDEN) == 0) {
                    PWR_TRACE("underdrive enable in stop mode: disabled\n");
                    self->state.PWR_CR &= ~PWR_CR_UDEN;
                    self->state.PWR_CSR &= ~PWR_CSR_UDRDY;
                } else {
                    PWR_TRACE("invalid UDEN value %08x!\n", val & PWR_CR_UDEN);
                }
            }
            if(valx & PWR_CR_ODSWEN) {
                if(!(self->state.PWR_CSR & PWR_CSR_ODRDY)){
                    PWR_TRACE("overdrive: can not set ODSWEN bit before overdrive has been enabled and ready!\n");
                } else {
                    if((val & PWR_CR_ODSWEN)){
                        PWR_TRACE("overdrive switching: enabled\n");
                        self->state.PWR_CR |= PWR_CR_ODSWEN;
                        self->state.PWR_CSR |= PWR_CSR_ODSWRDY;
                    } else {
                        PWR_TRACE("overdrive switching: disabled\n");
                        self->state.PWR_CR &= ~PWR_CR_ODSWEN;
                        self->state.PWR_CSR &= ~PWR_CSR_ODSWRDY;
                    }
                }
            }
            if(valx & PWR_CR_ODEN) {
                if((val & PWR_CR_ODEN)){
                    PWR_TRACE("overdrive: enabled\n");
                    self->state.PWR_CR |= PWR_CR_ODEN;
                    self->state.PWR_CSR |= PWR_CSR_ODRDY;
                } else {
                    PWR_TRACE("overdrive: disabled\n");
                    self->state.PWR_CR &= ~PWR_CR_ODEN;
                    self->state.PWR_CSR &= ~PWR_CSR_ODRDY;
                }
            }
            if(valx & PWR_CR_VOS) {
                uint32_t vos = (val & PWR_CR_VOS) >> 14;
                switch(vos){
                    case 0: PWR_TRACE("invalid value for VOS (0)\n"); break;
                    case 1: PWR_TRACE("voltage scale 3 selected\n"); break;
                    case 2: PWR_TRACE("voltage scale 2 selected\n"); break;
                    case 3: PWR_TRACE("voltage scale 1 selected\n"); break;
                }
                self->state.PWR_CR = (self->state.PWR_CR & PWR_CR_VOS) | (val & PWR_CR_VOS);
            }
            if(valx & PWR_CR_ADCDC1) {
                if(!(val & PWR_CR_ADCDC1)) PWR_TRACE("setting ADCDC1 to 0 has no effect\n");
                else PWR_TRACE("ADCDC1 bit set to 1\n");
                self->state.PWR_CR = (self->state.PWR_CR & PWR_CR_ADCDC1) | (val & PWR_CR_ADCDC1);
            }
            if(valx & PWR_CR_MRUDS) {
                if(val & PWR_CR_MRUDS) PWR_TRACE("Main Regulator: in under-drive mode and Flash memory in power-down when the device is in Stop under-drive mode.\n");
                else PWR_TRACE("Main regulator: will be ON when the device is in Stop mode\n");
                self->state.PWR_CR = (self->state.PWR_CR & PWR_CR_MRUDS) | (val & PWR_CR_MRUDS);
            }
            if(valx & PWR_CR_LPUDS) {
                if(val & PWR_CR_LPUDS) PWR_TRACE("Low-power regulator: ON if LPDS bit is set when the device is in Stop mode\n");
                else PWR_TRACE("Low-power regulator: in under-drive mode if LPDS bit is set and Flash memory in powerdown when the device is in Stop under-drive mode.\n");
                self->state.PWR_CR = (self->state.PWR_CR & PWR_CR_LPUDS) | (val & PWR_CR_LPUDS);
            }
            if(valx & PWR_CR_FPDS){
                if(val & PWR_CR_FPDS) PWR_TRACE("Flash memory not in power-down when the device is in Stop mode\n");
                else PWR_TRACE("Flash memory in power-down when the device is in Stop mode\n");
                self->state.PWR_CR = (self->state.PWR_CR & PWR_CR_FPDS) | (val & PWR_CR_FPDS);
            }
            if(valx & PWR_CR_DBP) {
                if(val & PWR_CR_DBP) PWR_TRACE("Access to RTC and RTC Backup registers and backup SRAM disabled\n");
                else PWR_TRACE("Access to RTC and RTC Backup registers and backup SRAM enabled\n");
                self->state.PWR_CR = (self->state.PWR_CR & PWR_CR_DBP) | (val & PWR_CR_DBP);
            }
            if(valx & PWR_CR_PLS) {
                const char *voltage[] = {
                    "2.0v",
                    "2.1v",
                    "2.3v",
                    "2.5v",
                    "2.6v",
                    "2.7v",
                    "2.8v",
                    "2.9v",
                };
                PWR_TRACE("power voltage detector level set to %s\n", voltage[(val & PWR_CR_PLS) >> 5]);
                self->state.PWR_CR = (self->state.PWR_CR & PWR_CR_PLS) | (val & PWR_CR_PLS);
            }
            if(valx & PWR_CR_PVDE) {
                if(val & PWR_CR_PVDE) PWR_TRACE("power voltage detector: enabled\n");
                else PWR_TRACE("power voltage detector: disabled\n");
                self->state.PWR_CR = (self->state.PWR_CR & PWR_CR_PVDE) | (val & PWR_CR_PVDE);
            }
            if(!(valx & PWR_CR_CSBF)) { // we check if written value and store value are both 1
                if(val & PWR_CR_CSBF) {
                    PWR_TRACE("standby flag cleared\n");
                    self->state.PWR_CR &= ~PWR_CR_CSBF;
                }
            }
            if(!(valx & PWR_CR_CWUF)){ // check if both values are 1
                if(val & PWR_CR_CWUF){
                    PWR_TRACE("wake up flag will be cleared after 2 clock cycles\n");
                    self->state.PWR_CR &= ~PWR_CR_CWUF;
                }
            }
            if(valx & PWR_CR_PDDS) {
                if(val & PWR_CR_PDDS) {
                    PWR_TRACE("configured cpu to enter stop mode when entering deep sleep\n");
                } else {
                    PWR_TRACE("configured cpu to enter standby mode when entering deep sleep\n");
                }
                self->state.PWR_CR = (self->state.PWR_CR & PWR_CR_PDDS) | (val & PWR_CR_PDDS);
            }
            if(valx & PWR_CR_LPDS){
                if(val & PWR_CR_LPDS){
                    PWR_TRACE("main voltage regulator will be on when in stop mode\n");
                } else {
                    PWR_TRACE("low power voltage regulator will be ON when in stop mode\n");
                }
                self->state.PWR_CR = (self->state.PWR_CR & PWR_CR_LPDS) | (val & PWR_CR_LPDS);
            }
        } break;
        case 0x04: { // CSR
            uint32_t valx = val ^ self->state.PWR_CSR;
            if(val & PWR_CSR_UDRDY) { // check val here and not valx
                uint32_t udrdy = val & PWR_CSR_UDRDY;
                PWR_TRACE("resetting UDRDY bits\n");
                self->state.PWR_CSR &= ~udrdy;
            }
            if(valx & PWR_CSR_ODSWRDY){
                PWR_ERROR("ODSWRDY bit is readonly\n");
            }
            if(valx & PWR_CSR_ODRDY) {
                PWR_ERROR("ODRDY bit is readonly\n");
            }
            if(valx & PWR_CSR_VOSRDY) {
                PWR_ERROR("VOSRDY bit is readonly\n");
            }
            if(valx & PWR_CSR_BRE){
                if(val & PWR_CSR_BRE){
                    PWR_TRACE("backup regulator: enabled\n");
                } else {
                    PWR_TRACE("backup regulator: disabled\n");
                }
            }
            if(valx & PWR_CSR_EWUP) {
                if(val & PWR_CSR_EWUP){
                    PWR_TRACE("wakeup pin: used for wakeup from standby\n");
                } else {
                    PWR_TRACE("wakeup pin: not used (configured as GPIO)\n");
                }
            }
            if(valx & PWR_CSR_BRR){
                PWR_ERROR("BRR bit is readonly\n");
            }
            if(valx & PWR_CSR_PVDO){
                PWR_ERROR("PVDO bit is readonly\n");
            }
            if(valx & PWR_CSR_SBF){
                PWR_ERROR("SBF bit is readonly\n");
            }
            if(valx & PWR_CSR_WUF){
                PWR_ERROR("WUF bit is readonly\n");
            }
        } break;
#endif

static void stm32u535_pwr_write(void *opaque, hwaddr addr, uint64_t val64, unsigned int size){
    struct stm32u535_pwr *self = (struct stm32u535_pwr*)opaque;
    if(size > 4){
        PWR_TRACE("write: invalid write size of %d bytes\n", size);
    }

    PWR_TRACE("write: to register at offset %08x %08x\n", (uint32_t)addr, (uint32_t)val64);

    uint32_t val = (uint32_t) val64;
    switch(addr){
        case PWR_CR1: {
            self->pwr_cr1 = val;
        } break;
        case PWR_CR2: {
            self->pwr_cr2 = val;
        } break;
        case PWR_CR3: {
            self->pwr_cr3 = val;
        } break;
        case PWR_CR4: {
            self->pwr_cr4 = val;
        } break;
        case PWR_CR5: {
            self->pwr_cr5 = val;
        } break;
        case PWR_VOSR: {
            self->pwr_vosr = val;
        } break;
        case PWR_SVMSR: {
            self->pwr_svmsr = val;
        } break;
        case PWR_PUCRA: {
            self->pwr_pucra = val;
        } break;
        case PWR_PDCRA: {
            self->pwr_pdcra = val;
        } break;
        case PWR_PUCRB: {
            self->pwr_pucrb = val;
        } break;
        case PWR_PDCRB: {
            self->pwr_pdcrb = val;
        } break;
        case PWR_PUCRC: {
            self->pwr_pucrc = val;
        } break;
        case PWR_PDCRC: {
            self->pwr_pdcrc = val;
        } break;
        case PWR_PUCRD: {
            self->pwr_pucrd = val;
        } break;
        case PWR_PDCRD: {
            self->pwr_pdcrd = val;
        } break;
        case PWR_PUCRE: {
            self->pwr_pucre = val;
        } break;
        case PWR_PDCRE: {
            self->pwr_pdcre = val;
        } break;
        case PWR_PUCRF: {
            self->pwr_pucrf = val;
        } break;
        case PWR_PDCRF: {
            self->pwr_pdcrf = val;
        } break;
        case PWR_PUCRG: {
            self->pwr_pucrg = val;
        } break;
        case PWR_PDCRG: {
            self->pwr_pdcrg = val;
        } break;
        case PWR_PUCRH: {
            self->pwr_pucrh = val;
        } break;
        case PWR_PDCRH: {
            self->pwr_pdcrh = val;
        } break;
        case PWR_SECCFGR: {
            self->pwr_seccfgr = val;
        } break;
        case PWR_PRIVCFGR: {
            self->pwr_privcfgr = val;
        } break;
        default: {
            PWR_TRACE("write: unknown register at offset %08x\n", (uint32_t)addr);
        }



    }
}

static const MemoryRegionOps stm32u535_pwr_ops = {
    .read = stm32u535_pwr_read,
    .write = stm32u535_pwr_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32u535_pwr_init(Object *obj){
    struct stm32u535_pwr *self = OBJECT_CHECK(struct stm32u535_pwr, obj, TYPE_STM32U535_PWR);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &self->irq);
    memory_region_init_io(&self->mmio, obj, &stm32u535_pwr_ops, self, TYPE_STM32U535_PWR, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &self->mmio);
}

static void stm32u535_pwr_reset(DeviceState *dev){
    struct stm32u535_pwr *self = OBJECT_CHECK(struct stm32u535_pwr, dev, TYPE_STM32U535_PWR);
    self->pwr_cr1 = 0x0000C000;
    self->pwr_cr2 = 0x20;
    self->pwr_cr3 = 0x00000000;
    self->pwr_vosr = 0x00008000;
    //self->pwr_sr1 = 0x00000000;
    //self->pwr_sr2 = 0x00000000;
    //self->pwr_scr = 0x00000000;
    self->pwr_svmsr = 0x00008000;
    self->pwr_pucra = 0x00000000;
    self->pwr_pdcra = 0x00000000;
    self->pwr_pucrb = 0x00000000;
    self->pwr_pdcrb = 0x00000000;
    self->pwr_pucrc = 0x00000000;
    self->pwr_pdcrc = 0x00000000;
    self->pwr_pucrd = 0x00000000;
    self->pwr_pdcrd = 0x00000000;
    self->pwr_pucre = 0x00000000;
    self->pwr_pdcre = 0x00000000;
    self->pwr_pucrf = 0x00000000;
    self->pwr_pdcrf = 0x00000000;
    self->pwr_pucrg = 0x00000000;
    self->pwr_pdcrg = 0x00000000;
    self->pwr_pucrh = 0x00000000;
    self->pwr_pdcrh = 0x00000000;
    self->pwr_seccfgr = 0x00000000;
    self->pwr_privcfgr = 0x00000000;

}

static void stm32u535_pwr_realize(DeviceState *dev, Error **errp){
    stm32u535_pwr_reset(dev);
}

static Property stm32u535_pwr_properties[] = {
    //DEFINE_PROP("state", struct stm32u535_pwr, state, qdev_prop_ptr, struct stm32l552_state*),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32u535_pwr_class_init(ObjectClass *klass, void *data){
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32u535_pwr_reset;
    dc->props_ = stm32u535_pwr_properties;
    dc->realize = stm32u535_pwr_realize;
}

static const TypeInfo stm32u535_pwr_info = {
    .name          = TYPE_STM32U535_PWR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct stm32u535_pwr),
    .instance_init = stm32u535_pwr_init,
    .class_init    = stm32u535_pwr_class_init
};

static void stm32u535_pwr_register_types(void){
    type_register_static(&stm32u535_pwr_info);
}

type_init(stm32u535_pwr_register_types)

