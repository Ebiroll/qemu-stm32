/*
 * STM32F4xx SYSCFG
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

#ifndef HW_STM32LXXX_SYSCFG_H
#define HW_STM32LXXX_SYSCFG_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define SYSCFG_SECCFGR 0x00
/*
Bit 3 FPUSEC: FPU security.
0: SYSCFG_FPUIMR register can be written by secure and non-secure access
1: SYSCFG_FPUIMR register can be written by secure access only.
Bit 2 SRAM2SEC: SRAM2 security.
0: SYSCFG_SKR, SYSCFG_SCR and SYSCFG_SWPRx registers can be written by secure
and non-secure access
1: SYSCFG_SKR, SYSCFG_SCR and SYSCFG_SWPRx register can be written by secure
access only.
Bit 1 CLASSBSEC: ClassB security.
0: SYSCFG_CFGR2 register can be written by secure and non-secure access
1: SYSCFG_CFGR2 register can be written by secure access only.
Bit 0 SYSCFGSEC: SYSCFG clock control security.
0: SYSCFG configuration clock in RCC registers can be written by secure and non-secure
access
1: SYSCFG configuration clock in RCC registers can be written by secure access only.
*/
#define SYSCFG_SECCFGR_FPUSEC (1 << 3)
#define SYSCFG_SECCFGR_SRAM2SEC (1 << 2)
#define SYSCFG_SECCFGR_CLASSBSEC (1 << 1)
#define SYSCFG_SECCFGR_SYSCFGSEC (1 << 0)

#define SYSCFG_CFGR1 0x04
#define SYSCFG_FPUIMR 0x08
#define SYSCFG_CNSLCKR 0x0C
#define SYSCFG_CFGR2 0x14
#define SYSCFG_SCSR 0x18
#define SYSCFG_SKR 0x1C
#define SYSCFG_SWPR 0x20
#define SYSCFG_SWPR2 0x24
#define SYSCFG_RSSCMDR 0x2C


#define VREFBUF_CSR_VRR_Pos     (3U)
#define VREFBUF_CSR_VRR_Msk     (0x1UL << VREFBUF_CSR_VRR_Pos)                 /*!< 0x00000008 */
#define VREFBUF_CSR_VRR         VREFBUF_CSR_VRR_Msk                            /*!<Voltage reference buffer ready  */


#define VREFBUF_CSR    0x100
#define VREFBUF_CCR    0x104


#define TYPE_STM32LXXX_SYSCFG "stm32lxxx-syscfg"
OBJECT_DECLARE_SIMPLE_TYPE(STM32lxxxSyscfgState, STM32LXXX_SYSCFG)

#define SYSCFG_NUM_EXTICR 4

typedef struct STM32lxxxSyscfgState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;
/*
#define SYSCFG_CFGR1 0x04
#define SYSCFG_FPUIMR 0x08
#define SYSCFG_CNSLCKR 0x0C
#define SYSCFG_CFGR2 0x14
#define SYSCFG_SCSR 0x18
#define SYSCFG_SKR 0x1C
#define SYSCFG_SWPR 0x20
#define SYSCFG_SWPR2 0x24
#define SYSCFG_RSSCMDR
*/

    uint32_t syscfg_seccfgr; // SYSCFG_SECCFGR
    uint32_t syscfg_cfgr1;
    uint32_t syscfg_fpuimr;
    uint32_t syscfg_cslckr;
    uint32_t syscfg_cnslckr;
    uint32_t syscfg_cfgr2;
    uint32_t syscfg_scsr;
    uint32_t syscfg_skr;
    uint32_t syscfg_swpr;
    uint32_t syscfg_swpr2;
    uint32_t syscfg_rsscmdr;


    uint32_t vrefbuf_csr;
    uint32_t vrefbuf_ccr;

    qemu_irq irq;
    qemu_irq gpio_out[16];
} STM32lxxxSyscfgState;

#endif
