/*
 * STM32U5XX SPI
 *
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
 * Copyright (c) 2024 Olof Astrand
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

#ifndef HW_STM32U5XX_SPI_H
#define HW_STM32U5XX_SPI_H

#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"

#define STM_SPI_CR1     0x00
#define STM_SPI_CR2     0x04
#define STM_SPI_SR      0x14
#define STM_SPI_DR      0x20

#define STM_SPI_CRCPR   0x40
#define STM_SPI_TXCRCR  0x44
#define STM_SPI_RXCRCR  0x48

#define STM_SPI_I2SCFGR 0x08
#define STM_SPI_I2SPR   0x18

#if 0
  __IO uint32_t CFG1;          /*!< SPI Configuration register 1,                    Address offset: 0x08 */
  __IO uint32_t CFG2;          /*!< SPI Configuration register 2,                    Address offset: 0x0C */
  __IO uint32_t IER;           /*!< SPI Interrupt Enable register,                   Address offset: 0x10 */
  __IO uint32_t SR;            /*!< SPI Status register,                             Address offset: 0x14 */
  __IO uint32_t IFCR;          /*!< SPI Interrupt/Status Flags Clear register,       Address offset: 0x18 */
  __IO uint32_t AUTOCR;        /*!< SPI Autonomous Mode Control register,            Address offset: 0x1C */
  __IO uint32_t TXDR;          /*!< SPI Transmit data register,                      Address offset: 0x20 */
  uint32_t      RESERVED1[3];  /*!< Reserved, 0x24-0x2C                                                   */
  __IO uint32_t RXDR;          /*!< SPI/I2S data register,                           Address offset: 0x30 */
  uint32_t      RESERVED2[3];  /*!< Reserved, 0x34-0x3C                                                   */
  __IO uint32_t CRCPOLY;       /*!< SPI CRC Polynomial register,                     Address offset: 0x40 */
  __IO uint32_t TXCRC;         /*!< SPI Transmitter CRC register,                    Address offset: 0x44 */
  __IO uint32_t RXCRC;         /*!< SPI Receiver CRC register,                       Address offset: 0x48 */
  __IO uint32_t UDRDR;         /*!< SPI Underrun data register,                      Address offset: 0x4C */

#endif


#define STM_SPI_CR1_SPE  (1 << 6)
#define STM_SPI_CR1_MSTR (1 << 2)

#define STM_SPI_SR_RXNE   1

#define TYPE_STM32U5XX_SPI "stm32u5xx-spi"
OBJECT_DECLARE_SIMPLE_TYPE(STM32U5XXSPIState, STM32U5XX_SPI)

struct STM32U5XXSPIState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    uint32_t spi_cr1;
    uint32_t spi_cr2;
    uint32_t spi_sr;
    uint32_t spi_dr;
    uint32_t spi_crcpr;
    uint32_t spi_rxcrcr;
    uint32_t spi_txcrcr;
    uint32_t spi_i2scfgr;
    uint32_t spi_i2spr;

    uint32_t rx_buffer[32];
    uint32_t rx_buffer_pos;
    uint32_t read_buffer_pos;

    qemu_irq irq;
    SSIBus *ssi;
};

#endif /* HW_STM32U5XX_SPI_H */
