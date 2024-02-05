/*
 * STM32U35 SPI
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
#include "qemu/log.h"
#include "qemu/module.h"
#include "stm32u5xx_spi.h"
#include "migration/vmstate.h"
#include "hw/irq.h"

#ifndef STM_SPI_ERR_DEBUG
#define STM_SPI_ERR_DEBUG 5
#endif

#define DB_PRINT_L(lvl, fmt, args...) do { \
    if (STM_SPI_ERR_DEBUG >= lvl) { \
        qemu_log("%s: " fmt, __func__, ## args); \
    } \
} while (0)

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

static void stm32u5xx_spi_reset(DeviceState *dev)
{
    STM32U5XXSPIState *s = STM32U5XX_SPI(dev);

    s->spi_cr1 = 0x00000000;
    s->spi_cr2 = 0x00000000;
    s->spi_sr = 0x0000000A;
    s->spi_dr = 0x0000000C;
    s->spi_crcpr = 0x00000007;
    s->spi_rxcrcr = 0x00000000;
    s->spi_txcrcr = 0x00000000;
    s->spi_i2scfgr = 0x00000000;
    s->spi_i2spr = 0x00000002;
}

static void stm32u5xx_spi_transfer(STM32U5XXSPIState *s, int size)
{
    DB_PRINT("Data to send: 0x%x\n", s->spi_dr);
    if (size==1) {
        s->rx_buffer[s->rx_buffer_pos] = ssi_transfer(s->ssi, s->spi_dr);
        s->spi_dr=s->rx_buffer[s->rx_buffer_pos];

        s->rx_buffer_pos++;
    }
    if (size==4) {
        s->rx_buffer[s->rx_buffer_pos] = ssi_transfer(s->ssi, s->spi_dr & 0xFF);
        s->rx_buffer_pos++;

        s->rx_buffer[s->rx_buffer_pos] = ssi_transfer(s->ssi, (s->spi_dr & 0xFF00) >> 8);
        s->rx_buffer_pos++;

        s->spi_dr=s->rx_buffer[s->rx_buffer_pos-2] | s->rx_buffer[s->rx_buffer_pos-1] >> 8;

    }

    s->spi_sr |= STM_SPI_SR_RXNE;

    DB_PRINT("Data received: 0x%x\n", s->spi_dr);
}

static uint64_t stm32u5xx_spi_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    STM32U5XXSPIState *s = opaque;

    DB_PRINT("Address: 0x%" HWADDR_PRIx "\n", addr);

    switch (addr) {
    case STM_SPI_CR1:
        return s->spi_cr1;
    case STM_SPI_CR2:
        qemu_log_mask(LOG_UNIMP, "%s: Interrupts and DMA are not implemented\n",
                      __func__);
        return s->spi_cr2;
    case STM_SPI_SR:
        return s->spi_sr;
    case STM_SPI_DR:
        // Only transfer
        //stm32u5xx_spi_transfer(s);
        s->spi_sr &= ~STM_SPI_SR_RXNE;
        if (size==4) {
            s->spi_dr=s->rx_buffer[s->read_buffer_pos] | (s->rx_buffer[s->read_buffer_pos+1] << 8);
            if (s->read_buffer_pos<s->rx_buffer_pos) {
                s->read_buffer_pos++;
                s->read_buffer_pos++;
            } else {
                s->read_buffer_pos=0;
                s->rx_buffer_pos=0;
            }

        } else  {
            s->spi_dr=s->rx_buffer[s->read_buffer_pos];
            if (s->read_buffer_pos<s->rx_buffer_pos) {
                s->read_buffer_pos++;
            } else {
                s->read_buffer_pos=0;
                s->rx_buffer_pos=0;
            }

        }
        return s->spi_dr;
    case STM_SPI_CRCPR:
        qemu_log_mask(LOG_UNIMP, "%s: CRC is not implemented, the registers " \
                      "are included for compatibility\n", __func__);
        return s->spi_crcpr;
    case STM_SPI_RXCRCR:
        qemu_log_mask(LOG_UNIMP, "%s: CRC is not implemented, the registers " \
                      "are included for compatibility\n", __func__);
        return s->spi_rxcrcr;
    case STM_SPI_TXCRCR:
        qemu_log_mask(LOG_UNIMP, "%s: CRC is not implemented, the registers " \
                      "are included for compatibility\n", __func__);
        return s->spi_txcrcr;
    case STM_SPI_I2SCFGR:
        qemu_log_mask(LOG_UNIMP, "%s: I2S is not implemented, the registers " \
                      "are included for compatibility\n", __func__);
        return s->spi_i2scfgr;
    case STM_SPI_I2SPR:
        qemu_log_mask(LOG_UNIMP, "%s: I2S is not implemented, the registers " \
                      "are included for compatibility\n", __func__);
        return s->spi_i2spr;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
    }

    return 0;
}

static void stm32u5xx_spi_write(void *opaque, hwaddr addr,
                                uint64_t val64, unsigned int size)
{
    STM32U5XXSPIState *s = opaque;
    uint32_t value = val64;

    DB_PRINT("Address: 0x%" HWADDR_PRIx ", Value: 0x%x\n", addr, value);

    switch (addr) {
    case STM_SPI_CR1:
        s->spi_cr1 = value;
        return;
    case STM_SPI_CR2:
        qemu_log_mask(LOG_UNIMP, "%s: " \
                      "Interrupts and DMA are not implemented\n", __func__);
        s->read_buffer_pos=0;
        s->rx_buffer_pos=0;
        s->spi_cr2 = value;
        //  SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);
        //  Value: 0x1722
        if (value == 0x1722) {
            DB_PRINT("Write CR2: 0x%x\n", value);
            // TODO: START DMA!!
            // Fake IRQ!
            qemu_irq_raise(s->irq);
        }
        if (value == 0x1708) {
            // TODO: START DMA!!
        }
        // CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_LDMATX);
        //  Value: 0x1700
        return;
    case STM_SPI_SR:
        /* Read only register, except for clearing the CRCERR bit, which
         * is not supported
         */
        return;
    case STM_SPI_DR:
        s->spi_dr = value;
        stm32u5xx_spi_transfer(s,size);
        return;
    case STM_SPI_CRCPR:
        qemu_log_mask(LOG_UNIMP, "%s: CRC is not implemented\n", __func__);
        return;
    case STM_SPI_RXCRCR:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Read only register: " \
                      "0x%" HWADDR_PRIx "\n", __func__, addr);
        return;
    case STM_SPI_TXCRCR:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Read only register: " \
                      "0x%" HWADDR_PRIx "\n", __func__, addr);
        return;
    case STM_SPI_I2SCFGR:
        qemu_log_mask(LOG_UNIMP, "%s: " \
                      "I2S is not implemented\n", __func__);
        return;
    case STM_SPI_I2SPR:
        qemu_log_mask(LOG_UNIMP, "%s: " \
                      "I2S is not implemented\n", __func__);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    }
}

static const MemoryRegionOps stm32u5xx_spi_ops = {
    .read = stm32u5xx_spi_read,
    .write = stm32u5xx_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_stm32u5xx_spi = {
    .name = TYPE_STM32U5XX_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(spi_cr1, STM32U5XXSPIState),
        VMSTATE_UINT32(spi_cr2, STM32U5XXSPIState),
        VMSTATE_UINT32(spi_sr, STM32U5XXSPIState),
        VMSTATE_UINT32(spi_dr, STM32U5XXSPIState),
        VMSTATE_UINT32(spi_crcpr, STM32U5XXSPIState),
        VMSTATE_UINT32(spi_rxcrcr, STM32U5XXSPIState),
        VMSTATE_UINT32(spi_txcrcr, STM32U5XXSPIState),
        VMSTATE_UINT32(spi_i2scfgr, STM32U5XXSPIState),
        VMSTATE_UINT32(spi_i2spr, STM32U5XXSPIState),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32u5xx_spi_init(Object *obj)
{
    STM32U5XXSPIState *s = STM32U5XX_SPI(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &stm32u5xx_spi_ops, s,
                          TYPE_STM32U5XX_SPI, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    s->ssi = ssi_create_bus(dev, "ssi");
}

static void stm32u5xx_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32u5xx_spi_reset;
    dc->vmsd = &vmstate_stm32u5xx_spi;
}

static const TypeInfo stm32u5xx_spi_info = {
    .name          = TYPE_STM32U5XX_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32U5XXSPIState),
    .instance_init = stm32u5xx_spi_init,
    .class_init    = stm32u5xx_spi_class_init,
};

static void stm32u5xx_spi_register_types(void)
{
    type_register_static(&stm32u5xx_spi_info);
}

type_init(stm32u5xx_spi_register_types)
