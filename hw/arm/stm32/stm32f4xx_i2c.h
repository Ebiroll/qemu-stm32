#pragma once 
#include "hw/arm/stm32fxxx.h"

#define	R_I2C_CR1      (0x00 / 4)
#define	R_I2C_CR2      (0x04 / 4)
#define R_I2C_OAR1     (0x08 / 4)
#define R_I2C_OAR2     (0x0c / 4)
#define R_I2C_DR       (0x10 / 4)
#define R_I2C_SR1      (0x14 / 4)
#define R_I2C_SR2      (0x18 / 4)
#define R_I2C_CCR      (0x1c / 4)
#define R_I2C_TRISE    (0x20 / 4)
// Digital Noise Filter Register
#define R_I2C_FLTR    (0x24 / 4)
#define R_I2C_MAX      (0x28 / 4)



#define TYPE_STM32F4XX_I2C "stm32f4xx-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F4XXI2cState, STM32F4XX_I2C)

struct STM32F4XXI2cState {
    SysBusDevice busdev;
    MemoryRegion iomem;
    qemu_irq evt_irq;
    qemu_irq err_irq;

    I2CBus *bus;

    // Periferal ID, from old SW
    uint32_t  periph;

    int32_t rx;
    int rx_full; 
    uint16_t regs[R_I2C_MAX];

};
