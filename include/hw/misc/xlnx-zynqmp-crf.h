/*
 * QEMU model of the CRF - Clock Reset FPD.
 *
 * Copyright (c) 2022 Xilinx Inc.
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Written by Edgar E. Iglesias <edgar.iglesias@xilinx.com>
 */
#ifndef HW_MISC_XLNX_ZYNQMP_CRF_H
#define HW_MISC_XLNX_ZYNQMP_CRF_H

#include "hw/sysbus.h"
#include "hw/register.h"

#define TYPE_XLNX_ZYNQMP_CRF "xlnx.zynqmp_crf"
OBJECT_DECLARE_SIMPLE_TYPE(XlnxZynqMPCRF, XLNX_ZYNQMP_CRF)

REG32(ERR_CTRL, 0x0)
    FIELD(ERR_CTRL, SLVERR_ENABLE, 0, 1)
REG32(IR_STATUS, 0x4)
    FIELD(IR_STATUS, ADDR_DECODE_ERR, 0, 1)
REG32(IR_MASK, 0x8)
    FIELD(IR_MASK, ADDR_DECODE_ERR, 0, 1)
REG32(IR_ENABLE, 0xc)
    FIELD(IR_ENABLE, ADDR_DECODE_ERR, 0, 1)
REG32(IR_DISABLE, 0x10)
    FIELD(IR_DISABLE, ADDR_DECODE_ERR, 0, 1)
REG32(CRF_WPROT, 0x1c)
    FIELD(CRF_WPROT, ACTIVE, 0, 1)
REG32(APLL_CTRL, 0x20)
    FIELD(APLL_CTRL, POST_SRC, 24, 3)
    FIELD(APLL_CTRL, PRE_SRC, 20, 3)
    FIELD(APLL_CTRL, CLKOUTDIV, 17, 1)
    FIELD(APLL_CTRL, DIV2, 16, 1)
    FIELD(APLL_CTRL, FBDIV, 8, 7)
    FIELD(APLL_CTRL, BYPASS, 3, 1)
    FIELD(APLL_CTRL, RESET, 0, 1)
REG32(APLL_CFG, 0x24)
    FIELD(APLL_CFG, LOCK_DLY, 25, 7)
    FIELD(APLL_CFG, LOCK_CNT, 13, 10)
    FIELD(APLL_CFG, LFHF, 10, 2)
    FIELD(APLL_CFG, CP, 5, 4)
    FIELD(APLL_CFG, RES, 0, 4)
REG32(APLL_FRAC_CFG, 0x28)
    FIELD(APLL_FRAC_CFG, ENABLED, 31, 1)
    FIELD(APLL_FRAC_CFG, SEED, 22, 3)
    FIELD(APLL_FRAC_CFG, ALGRTHM, 19, 1)
    FIELD(APLL_FRAC_CFG, ORDER, 18, 1)
    FIELD(APLL_FRAC_CFG, DATA, 0, 16)
REG32(DPLL_CTRL, 0x2c)
    FIELD(DPLL_CTRL, POST_SRC, 24, 3)
    FIELD(DPLL_CTRL, PRE_SRC, 20, 3)
    FIELD(DPLL_CTRL, CLKOUTDIV, 17, 1)
    FIELD(DPLL_CTRL, DIV2, 16, 1)
    FIELD(DPLL_CTRL, FBDIV, 8, 7)
    FIELD(DPLL_CTRL, BYPASS, 3, 1)
    FIELD(DPLL_CTRL, RESET, 0, 1)
REG32(DPLL_CFG, 0x30)
    FIELD(DPLL_CFG, LOCK_DLY, 25, 7)
    FIELD(DPLL_CFG, LOCK_CNT, 13, 10)
    FIELD(DPLL_CFG, LFHF, 10, 2)
    FIELD(DPLL_CFG, CP, 5, 4)
    FIELD(DPLL_CFG, RES, 0, 4)
REG32(DPLL_FRAC_CFG, 0x34)
    FIELD(DPLL_FRAC_CFG, ENABLED, 31, 1)
    FIELD(DPLL_FRAC_CFG, SEED, 22, 3)
    FIELD(DPLL_FRAC_CFG, ALGRTHM, 19, 1)
    FIELD(DPLL_FRAC_CFG, ORDER, 18, 1)
    FIELD(DPLL_FRAC_CFG, DATA, 0, 16)
REG32(VPLL_CTRL, 0x38)
    FIELD(VPLL_CTRL, POST_SRC, 24, 3)
    FIELD(VPLL_CTRL, PRE_SRC, 20, 3)
    FIELD(VPLL_CTRL, CLKOUTDIV, 17, 1)
    FIELD(VPLL_CTRL, DIV2, 16, 1)
    FIELD(VPLL_CTRL, FBDIV, 8, 7)
    FIELD(VPLL_CTRL, BYPASS, 3, 1)
    FIELD(VPLL_CTRL, RESET, 0, 1)
REG32(VPLL_CFG, 0x3c)
    FIELD(VPLL_CFG, LOCK_DLY, 25, 7)
    FIELD(VPLL_CFG, LOCK_CNT, 13, 10)
    FIELD(VPLL_CFG, LFHF, 10, 2)
    FIELD(VPLL_CFG, CP, 5, 4)
    FIELD(VPLL_CFG, RES, 0, 4)
REG32(VPLL_FRAC_CFG, 0x40)
    FIELD(VPLL_FRAC_CFG, ENABLED, 31, 1)
    FIELD(VPLL_FRAC_CFG, SEED, 22, 3)
    FIELD(VPLL_FRAC_CFG, ALGRTHM, 19, 1)
    FIELD(VPLL_FRAC_CFG, ORDER, 18, 1)
    FIELD(VPLL_FRAC_CFG, DATA, 0, 16)
REG32(PLL_STATUS, 0x44)
    FIELD(PLL_STATUS, VPLL_STABLE, 5, 1)
    FIELD(PLL_STATUS, DPLL_STABLE, 4, 1)
    FIELD(PLL_STATUS, APLL_STABLE, 3, 1)
    FIELD(PLL_STATUS, VPLL_LOCK, 2, 1)
    FIELD(PLL_STATUS, DPLL_LOCK, 1, 1)
    FIELD(PLL_STATUS, APLL_LOCK, 0, 1)
REG32(APLL_TO_LPD_CTRL, 0x48)
    FIELD(APLL_TO_LPD_CTRL, DIVISOR0, 8, 6)
REG32(DPLL_TO_LPD_CTRL, 0x4c)
    FIELD(DPLL_TO_LPD_CTRL, DIVISOR0, 8, 6)
REG32(VPLL_TO_LPD_CTRL, 0x50)
    FIELD(VPLL_TO_LPD_CTRL, DIVISOR0, 8, 6)
REG32(ACPU_CTRL, 0x60)
    FIELD(ACPU_CTRL, CLKACT_HALF, 25, 1)
    FIELD(ACPU_CTRL, CLKACT_FULL, 24, 1)
    FIELD(ACPU_CTRL, DIVISOR0, 8, 6)
    FIELD(ACPU_CTRL, SRCSEL, 0, 3)
REG32(DBG_TRACE_CTRL, 0x64)
    FIELD(DBG_TRACE_CTRL, CLKACT, 24, 1)
    FIELD(DBG_TRACE_CTRL, DIVISOR0, 8, 6)
    FIELD(DBG_TRACE_CTRL, SRCSEL, 0, 3)
REG32(DBG_FPD_CTRL, 0x68)
    FIELD(DBG_FPD_CTRL, CLKACT, 24, 1)
    FIELD(DBG_FPD_CTRL, DIVISOR0, 8, 6)
    FIELD(DBG_FPD_CTRL, SRCSEL, 0, 3)
REG32(DP_VIDEO_REF_CTRL, 0x70)
    FIELD(DP_VIDEO_REF_CTRL, CLKACT, 24, 1)
    FIELD(DP_VIDEO_REF_CTRL, DIVISOR1, 16, 6)
    FIELD(DP_VIDEO_REF_CTRL, DIVISOR0, 8, 6)
    FIELD(DP_VIDEO_REF_CTRL, SRCSEL, 0, 3)
REG32(DP_AUDIO_REF_CTRL, 0x74)
    FIELD(DP_AUDIO_REF_CTRL, CLKACT, 24, 1)
    FIELD(DP_AUDIO_REF_CTRL, DIVISOR1, 16, 6)
    FIELD(DP_AUDIO_REF_CTRL, DIVISOR0, 8, 6)
    FIELD(DP_AUDIO_REF_CTRL, SRCSEL, 0, 3)
REG32(DP_STC_REF_CTRL, 0x7c)
    FIELD(DP_STC_REF_CTRL, CLKACT, 24, 1)
    FIELD(DP_STC_REF_CTRL, DIVISOR1, 16, 6)
    FIELD(DP_STC_REF_CTRL, DIVISOR0, 8, 6)
    FIELD(DP_STC_REF_CTRL, SRCSEL, 0, 3)
REG32(DDR_CTRL, 0x80)
    FIELD(DDR_CTRL, CLKACT, 24, 1)
    FIELD(DDR_CTRL, DIVISOR0, 8, 6)
    FIELD(DDR_CTRL, SRCSEL, 0, 3)
REG32(GPU_REF_CTRL, 0x84)
    FIELD(GPU_REF_CTRL, PP1_CLKACT, 26, 1)
    FIELD(GPU_REF_CTRL, PP0_CLKACT, 25, 1)
    FIELD(GPU_REF_CTRL, CLKACT, 24, 1)
    FIELD(GPU_REF_CTRL, DIVISOR0, 8, 6)
    FIELD(GPU_REF_CTRL, SRCSEL, 0, 3)
REG32(SATA_REF_CTRL, 0xa0)
    FIELD(SATA_REF_CTRL, CLKACT, 24, 1)
    FIELD(SATA_REF_CTRL, DIVISOR0, 8, 6)
    FIELD(SATA_REF_CTRL, SRCSEL, 0, 3)
REG32(PCIE_REF_CTRL, 0xb4)
    FIELD(PCIE_REF_CTRL, CLKACT, 24, 1)
    FIELD(PCIE_REF_CTRL, DIVISOR0, 8, 6)
    FIELD(PCIE_REF_CTRL, SRCSEL, 0, 3)
REG32(GDMA_REF_CTRL, 0xb8)
    FIELD(GDMA_REF_CTRL, CLKACT, 24, 1)
    FIELD(GDMA_REF_CTRL, DIVISOR0, 8, 6)
    FIELD(GDMA_REF_CTRL, SRCSEL, 0, 3)
REG32(DPDMA_REF_CTRL, 0xbc)
    FIELD(DPDMA_REF_CTRL, CLKACT, 24, 1)
    FIELD(DPDMA_REF_CTRL, DIVISOR0, 8, 6)
    FIELD(DPDMA_REF_CTRL, SRCSEL, 0, 3)
REG32(TOPSW_MAIN_CTRL, 0xc0)
    FIELD(TOPSW_MAIN_CTRL, CLKACT, 24, 1)
    FIELD(TOPSW_MAIN_CTRL, DIVISOR0, 8, 6)
    FIELD(TOPSW_MAIN_CTRL, SRCSEL, 0, 3)
REG32(TOPSW_LSBUS_CTRL, 0xc4)
    FIELD(TOPSW_LSBUS_CTRL, CLKACT, 24, 1)
    FIELD(TOPSW_LSBUS_CTRL, DIVISOR0, 8, 6)
    FIELD(TOPSW_LSBUS_CTRL, SRCSEL, 0, 3)
REG32(DBG_TSTMP_CTRL, 0xf8)
    FIELD(DBG_TSTMP_CTRL, DIVISOR0, 8, 6)
    FIELD(DBG_TSTMP_CTRL, SRCSEL, 0, 3)
REG32(RST_FPD_TOP, 0x100)
    FIELD(RST_FPD_TOP, PCIE_CFG_RESET, 19, 1)
    FIELD(RST_FPD_TOP, PCIE_BRIDGE_RESET, 18, 1)
    FIELD(RST_FPD_TOP, PCIE_CTRL_RESET, 17, 1)
    FIELD(RST_FPD_TOP, DP_RESET, 16, 1)
    FIELD(RST_FPD_TOP, SWDT_RESET, 15, 1)
    FIELD(RST_FPD_TOP, AFI_FM5_RESET, 12, 1)
    FIELD(RST_FPD_TOP, AFI_FM4_RESET, 11, 1)
    FIELD(RST_FPD_TOP, AFI_FM3_RESET, 10, 1)
    FIELD(RST_FPD_TOP, AFI_FM2_RESET, 9, 1)
    FIELD(RST_FPD_TOP, AFI_FM1_RESET, 8, 1)
    FIELD(RST_FPD_TOP, AFI_FM0_RESET, 7, 1)
    FIELD(RST_FPD_TOP, GDMA_RESET, 6, 1)
    FIELD(RST_FPD_TOP, GPU_PP1_RESET, 5, 1)
    FIELD(RST_FPD_TOP, GPU_PP0_RESET, 4, 1)
    FIELD(RST_FPD_TOP, GPU_RESET, 3, 1)
    FIELD(RST_FPD_TOP, GT_RESET, 2, 1)
    FIELD(RST_FPD_TOP, SATA_RESET, 1, 1)
REG32(RST_FPD_APU, 0x104)
    FIELD(RST_FPD_APU, ACPU3_PWRON_RESET, 13, 1)
    FIELD(RST_FPD_APU, ACPU2_PWRON_RESET, 12, 1)
    FIELD(RST_FPD_APU, ACPU1_PWRON_RESET, 11, 1)
    FIELD(RST_FPD_APU, ACPU0_PWRON_RESET, 10, 1)
    FIELD(RST_FPD_APU, APU_L2_RESET, 8, 1)
    FIELD(RST_FPD_APU, ACPU3_RESET, 3, 1)
    FIELD(RST_FPD_APU, ACPU2_RESET, 2, 1)
    FIELD(RST_FPD_APU, ACPU1_RESET, 1, 1)
    FIELD(RST_FPD_APU, ACPU0_RESET, 0, 1)
REG32(RST_DDR_SS, 0x108)
    FIELD(RST_DDR_SS, DDR_RESET, 3, 1)
    FIELD(RST_DDR_SS, APM_RESET, 2, 1)

#define CRF_R_MAX (R_RST_DDR_SS + 1)

struct XlnxZynqMPCRF {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq_ir;

    RegisterInfoArray *reg_array;
    uint32_t regs[CRF_R_MAX];
    RegisterInfo regs_info[CRF_R_MAX];
};

#endif
