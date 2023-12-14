/*
 * STM32L552 DMAMUX
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
#ifndef DMAMUX_H
#define DMAMUX_H


#include "hw/sysbus.h"
#include "qom/object.h"

#define MUX_CCR    0x00

// /* Configure the source, destination address and the data length & clear flags*/
//    DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);


//  stm32l55_dmamux_write: Invalid write to offset 0x84
// hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;

//(qemu) STM32L552_DMA: l552_dma_write: register HISR (READ-ONLY), data: 0x1
//f2xx dma: invalid write to ISR
//  /* Clear all flags */
//  hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1CU));

//(qemu) STM32L552_DMA: l552_dma_write: register HIFCR, data: 0x12
// /* Configure DMA Channel data length */
//  hdma->Instance->CNDTR = DataLength;

// (qemu) STM32L552_DMA: l552_dma_stream_write: stream: 0, register CR, data:0x4001300c
///* Configure DMA Channel source address */
//    hdma->Instance->CPAR = SrcAddress;

//  
// (qemu) STM32L552_DMA: l552_dma_stream_write: stream: 0, register NDTR, data:0x2002fe98
//  /* Configure DMA Channel destination address */
//    hdma->Instance->CM0AR = DstAddress;
//////////////////////////////////////////////////////////////////////////////////
//
//   if((hdma->DMAmuxChannel->CCR & DMAMUX_CxCR_SE) != 0U)
// (qemu) stm32l55_dmamux_read: Invalid read from offset 0x20
// stm32l55_dmamux_read: Invalid read from offset 0x20
//
// 
// (qemu) STM32L552_DMA: l552_dma_read: addr: 0x8, size:4...
// STM32L552_DMA:    l552_dma_read: register LIFCR
// STM32L552_DMA:     l552_dma_read: result:0x8e
// STM32L552_DMA: l552_dma_write: register LIFCR, data: 0x8f
// /* Enable the Peripheral */
//    __HAL_DMA_ENABLE(hdma);  
// #define __HAL_DMA_ENABLE(__HANDLE__)        ((__HANDLE__)->Instance->CCR |=  DMA_CCR_EN)
//


//void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma)
//{
//  uint32_t flag_it = hdma->DmaBaseAddress->ISR;
//  uint32_t source_it = hdma->Instance->CCR;

//  uint32_t  compare=(DMA_FLAG_TC1 << (hdma->ChannelIndex & 0x1CU));
//  uint32_t  source=DMA_IT_TC;

// flag_it = 119
// source_it = 143
// compare = 2
// source = 2

// Full DMA
// flag_it = 115
// source_it = 139
// compare = 2
// source = 2

//      if(hdma->XferCpltCallback != NULL)
//      {
//        /* Transfer complete callback */
//        hdma->XferCpltCallback(hdma);
//      }
// SPI_DMAReceiveCplt

//  SPI_EndRxTransaction
// hspi->State = HAL_SPI_STATE_READY;
// HAL_SPI_RxCpltCallback
//
// DMA2
// flag_it = 112
// source_it = 155
// compare = 32
// source = 2

#define RG0CR_OFFSET 0x100
#define RG1CR_OFFSET 0x104
#define RG2CR_OFFSET 0x108
#define RG3CR_OFFSET 0x10C
#define RG4CR_OFFSET 0x110
#define RG5CR_OFFSET 0x114
#define RG6CR_OFFSET 0x118
#define RG7CR_OFFSET 0x11C
#define RG8CR_OFFSET 0x120
#define RG9CR_OFFSET 0x124
#define RG10CR_OFFSET 0x128
#define RG11CR_OFFSET 0x12C
#define RG12CR_OFFSET 0x130
#define RG13CR_OFFSET 0x134
#define RG14CR_OFFSET 0x138
#define RG15CR_OFFSET 0x13C


// RG0CR bit fields
#define RG0CR_GNBREQ_MASK   (0x1F << 19) // Number of Request
#define RG0CR_GPOL_MASK     (0x3 << 17)  // Generation Polarity
#define RG0CR_GE_MASK       (0x1 << 16)  // Generation Enable
#define RG0CR_OIE_MASK      (0x1 << 8)   // Overrun Interrupt Enable
#define RG0CR_SIG_ID_MASK   (0x1F)       // Signal ID

#define GCR_COUNT 16


#define TYPE_STM32L552_DMAMUX "stm32l552-dmamux"
OBJECT_DECLARE_SIMPLE_TYPE(STM32L55DmaMuxState, STM32L552_DMAMUX)

struct STM32L55DmaMuxState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    uint32_t ccr;

    uint32_t gcr[GCR_COUNT];           // Array of GCR registers
    qemu_irq irq;
};

#endif /* DMAMUX_H */
