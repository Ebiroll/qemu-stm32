/*
 * STM32 Microcontroller RCC (Reset and Clock Control) module
 *
 * Copyright (c) 2018 Martin Schröder <mkschreder.uk@gmail.com>
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

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "hw/hw.h"
#include "hw/arm/stm32fxxx.h"
#include "stm32fxxx_gpio.h"
#include "hw/qdev-core.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"

static uint64_t stm32fxxx_gpio_read(void *opaque, hwaddr addr, unsigned int size){
    struct stm32fxxx_gpio *self = (struct stm32fxxx_gpio*)opaque;
    if(size != 4) {
        GPIO_ERROR("gpio read of != 4 bytes not implemented\n");
    }
    switch(addr){
        case 0x00: return self->GPIO.MODER; break;
        case 0x04: return self->GPIO.OTYPER; break;
        case 0x08: return self->GPIO.OSPEEDR; break;
        case 0x0c: return self->GPIO.PUPDR; break;
        case 0x10: return self->GPIO.IDR; break;
        case 0x14: return self->GPIO.ODR; break;
        case 0x18: return self->GPIO.BSRR; break;
        case 0x1c: return self->GPIO.LCKR; break;
        case 0x20: return self->GPIO.AFRL; break;
        case 0x24: return self->GPIO.AFRH; break;
        default:  {
            GPIO_ERROR("Unknown offset for gpio register 0x%08x\n", (uint32_t)addr);
        }
    }
    return 0;
}

static void stm32fxxx_gpio_write(void *opaque, hwaddr addr, uint64_t val64, unsigned int size){
    struct stm32fxxx_gpio *self = (struct stm32fxxx_gpio*)opaque;
    uint32_t val = (uint32_t)val64;

    if(size == 1){
        int bitshift = (addr & 3) * 8;
        addr &= ~3;
        val = (self->GPIO.regs[addr] & ~(0xff << bitshift)) | ((val & 0xff) << bitshift);
    } else if(size == 2) {
        int bitshift = (addr & 3) * 8;
        addr &= ~3;
        val = (self->GPIO.regs[addr] & ~(0xffff << bitshift)) | ((val & 0xffff) << bitshift);
    } else if(size == 4) {
        // skip
    } else {
        GPIO_ERROR("gpio write of %d bytes not implemented\n", size);
    }

    switch(addr){
        case 0x00: { // MODER
        // Was GPIO[self->port_id].MODER OLAS, I dont undetstand.
            uint32_t valx = val ^ self->GPIO.MODER;
            for(int c = 0; c < 16; c++){
                if(valx & (3 << (c * 2))) {
                    const char *modestr[] = {
                        "Input",
                        "Output",
                        "AF",
                        "Analog"
                    };
                    GPIO_TRACE("GPIO%c P%c%d: mode set to %s\n", 'A' + self->port_id, 'A' + self->port_id, c, modestr[(val >> (c * 2)) & 3]);
                }
            }
        } break;
        case 0x04: { // OTYPER
            uint32_t valx = val ^ self->GPIO.OTYPER;
            for(int c = 0; c < 16; c++){
                if(valx & (1 << c)){
                    uint8_t outpp = (self->GPIO.OTYPER >> c) & 1;
                    GPIO_TRACE("GPIO%c P%c%d: mode set to %s\n", 'A' + self->port_id, 'A' + self->port_id, c, outpp?"PP":"OD");
                }
            }
            self->GPIO.OTYPER = val;
        } break;
        case 0x08: { // OSPEEDR
            uint32_t valx = val ^ self->GPIO.OSPEEDR;
            for(int c = 0; c < 16; c++){
                if(valx & (3 << (c * 2))) {
                    const char *info[] = {
                        "Low speed",
                        "Medium speed",
                        "High speed",
                        "Very high speed"
                    };
                    GPIO_TRACE("GPIO%c P%c%d: speed set to %s\n", 'A' + self->port_id, 'A' + self->port_id, c, info[(val >> (c * 2)) & 3]);
                }
            }
            self->GPIO.OSPEEDR = val;
        } break;
        case 0x0c: { // PUPDR
            uint32_t valx = val ^ self->GPIO.PUPDR;
            for(int c = 0; c < 16; c++){
                if(valx & (3 << (c * 2))) {
                    const char *info[] = {
                        "No pullup / pulldown",
                        "Pull up",
                        "Pull down",
                        "INVALID"
                    };
                    GPIO_TRACE("GPIO%c P%c%d: pu/pd set to: %s\n", 'A' + self->port_id, 'A' + self->port_id, c, info[(val >> (c * 2)) & 3]);
                }
            }
            self->GPIO.PUPDR = val;
        } break;
        case 0x10: {
            GPIO_ERROR("attempted to write to input data register\n");
        } break;
        case 0x14: { // ODR
            for(int c = 0; c < 16; c++){
                uint8_t mode = (self->GPIO.MODER >> (c * 2)) & 3;
                if(mode != 1) { // if mode not output
                    GPIO_TRACE("GPIO%c P%c%d: writing to ODR has no effect. Pin not configured as output (mode = %d)\n", 'A' + self->port_id, 'A' + self->port_id, c, mode);
                }
            }
            self->GPIO.ODR = val;
        } break;
        case 0x18: { // BSRR
            for(int c = 0; c < 16; c++){
                bool set = (val >> c) & 1;
                bool reset = (val >> (16 + c)) & 1;
                if(set && reset) {
                    GPIO_ERROR("GPIO%c P%c%d: BS and BR both set\n", 'A' + self->port_id, 'A' + self->port_id, c);
                } else if(set){
                    if (self->GPIO.ODR!=(self->GPIO.ODR | (1 << c))) {
                      GPIO_TRACE("GPIO%c P%c%d: write value 1\n", 'A' + self->port_id, 'A' + self->port_id, c);                      
                    }
                    qemu_irq_raise(self->pins[c]);
                    self->GPIO.ODR |= (1 << c);
                    //GPIO_TRACE("GPIO%c P%c%d: write value 1\n", 'A' + self->port_id, 'A' + self->port_id, c);
                } else if(reset) {
                    if (self->GPIO.ODR!=(self->GPIO.ODR & ~(1 << c))) {
                      GPIO_TRACE("GPIO%c P%c%d: write value 0\n", 'A' + self->port_id, 'A' + self->port_id, c);                     
                    }
                    qemu_irq_lower(self->pins[c]);
                    self->GPIO.ODR &= ~(1 << c);
                    //GPIO_TRACE("GPIO%c P%c%d: write value 0\n", 'A' + self->port_id, 'A' + self->port_id, c);
                } else {
                    //GPIO_ERROR("GPIO%c P%c%d: BSRR: no action taken\n", 'A' + self->port_id, 'A' + self->port_id, c);
                }
            }
            self->GPIO.BSRR = 0;
        } break;
        case 0x1c: {
            GPIO_ERROR("Lock register not implemented\n");
        } break;
        case 0x20: { // AFRL
            // Was self->state->GPIO[self->port_id].AFRL;
            uint32_t valx = val ^ self->GPIO.AFRL;
            for(int c = 0; c < 8; c++){
                if(valx & (0xf << (c * 4))) {
                    GPIO_TRACE("GPIO%c P%c%d: connected to AF%d\n", 'A' + self->port_id, 'A' + self->port_id, c, (val >> (c * 4)) & 0xf);
                }
            }
            self->GPIO.AFRL = val;
        } break;
        case 0x24: { // AFRR
            uint32_t valx = val ^ self->GPIO.AFRH;
            for(int c = 0; c < 8; c++){
                if(valx & (0xf << (c * 4))) {
                    GPIO_TRACE("GPIO%c P%c%d: connected to AF%d\n", 'A' + self->port_id, 'A' + self->port_id, c + 8, (val >> (c * 4)) & 0xf);
                }
            }
            self->GPIO.AFRH = val;
        } break;
        case 0x28: {
            // Bit reset register
            // val
            for(int c = 0; c < 16; c++){
                bool set_reg = (val >> c) & 1;
                //    if (set_reg) {
                //        qemu_irq_raise(self->pins[c]);
                //        printf("GPIO%c BRR(2) %08x\n", 'A' + self->port_id, (int)c);
                //    }
                if (set_reg) {   
                    qemu_irq_lower(self->pins[c]);
                    printf("GPIO%c BRR(1) %08x\n", 'A' + self->port_id, (int)c);
                }
            }
        } break;
        default: 
            printf("GPIO%c write %08x\n", 'A' + self->port_id, (int)addr);
    }
}

static const MemoryRegionOps stm32fxxx_gpio_ops = {
    .read = stm32fxxx_gpio_read,
    .write = stm32fxxx_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32fxxx_gpio_init(Object *obj){
    struct stm32fxxx_gpio *self = OBJECT_CHECK(struct stm32fxxx_gpio, obj, TYPE_STM32FXXX_GPIO);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &self->irq);
    memory_region_init_io(&self->mmio, obj, &stm32fxxx_gpio_ops, self, TYPE_STM32FXXX_GPIO, 0x3FF);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &self->mmio);
}

//static void anon_gpio_out_set(void *opaque, int num, int level)
//{
    //stm32fxxx_gpio *s = opaque;
//}

static void stm32fxxx_gpio_realize(DeviceState *dev, Error **errp){
    struct stm32fxxx_gpio *self = OBJECT_CHECK(struct stm32fxxx_gpio, dev, TYPE_STM32FXXX_GPIO);
    //size_t ngpios = (sizeof(self->GPIO) / sizeof(self->state->GPIO[0]));
    self->port_id = self->_port_id;
    //if(self->port_id > ngpios) {
    //    fprintf(stderr, "Wrong id for gpio port. Exceeds number of supported gpio ports in the chip state\n");
    //    exit(1);
    //}
    //self->regs = &self->state->GPIO[self->port_id];

    if(self->port_id == 0) self->GPIO.MODER = 0xA8000000;
    else if(self->port_id == 1) self->GPIO.MODER = 0x280;
    else self->GPIO.MODER = 0;
    self->GPIO.OTYPER = 0;
    self->GPIO.OSPEEDR = (self->port_id == 0)?0x0C000000:((self->port_id == 1)?0xC0:0);
    self->GPIO.PUPDR = (self->port_id == 0)?0x64000000:((self->port_id == 1)?0x100:0);
    self->GPIO.IDR = 0;
    self->GPIO.ODR = 0;
    self->GPIO.BSRR = 0;
    self->GPIO.LCKR = 0;
    self->GPIO.AFRL = 0;
    self->GPIO.AFRH = 0;

    qdev_init_gpio_out(DEVICE(self),(qemu_irq *) &(self->pins),32);

}

static void stm32fxxx_gpio_reset(DeviceState *dev){

}

static Property stm32fxxx_gpio_properties[] = {
    //DEFINE_PROP("state", struct stm32fxxx_gpio, state, qdev_prop_ptr, struct stm32fxxx_state*),
    //DEFINE_PROP_UINT8("port_id", struct stm32fxxx_gpio, _port_id, 0),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32fxxx_gpio_class_init(ObjectClass *klass, void *data){
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32fxxx_gpio_reset;
    dc->props_ = stm32fxxx_gpio_properties;
    dc->realize = stm32fxxx_gpio_realize;
}

static const TypeInfo stm32fxxx_gpio_info = {
    .name          = TYPE_STM32FXXX_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct stm32fxxx_gpio),
    .instance_init = stm32fxxx_gpio_init,
    .class_init    = stm32fxxx_gpio_class_init
};

static void stm32fxxx_gpio_register_types(void){
    type_register_static(&stm32fxxx_gpio_info);
}

type_init(stm32fxxx_gpio_register_types)

