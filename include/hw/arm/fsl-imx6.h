/*
 * Freescale i.MX31 SoC emulation
 *
 * Copyright (C) 2015 Jean-Christophe Dubois <jcd@tribudubois.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 */

#ifndef FSL_IMX6_H
#define FSL_IMX6_H

#include "hw/arm/boot.h"
#include "hw/cpu/a9mpcore.h"
#include "hw/misc/imx6_ccm.h"
#include "hw/misc/imx6_src.h"
#include "hw/watchdog/wdt_imx2.h"
#include "hw/char/imx_serial.h"
#include "hw/timer/imx_gpt.h"
#include "hw/timer/imx_epit.h"
#include "hw/i2c/imx_i2c.h"
#include "hw/gpio/imx_gpio.h"
#include "hw/sd/sdhci.h"
#include "hw/ssi/imx_spi.h"
#include "hw/net/imx_fec.h"
#include "hw/usb/chipidea.h"
#include "hw/usb/imx-usb-phy.h"
#include "exec/memory.h"
#include "cpu.h"

#define TYPE_FSL_IMX6 "fsl,imx6"
#define FSL_IMX6(obj) OBJECT_CHECK(FslIMX6State, (obj), TYPE_FSL_IMX6)

#define FSL_IMX6_NUM_CPUS 4
#define FSL_IMX6_NUM_UARTS 5
#define FSL_IMX6_NUM_EPITS 2
#define FSL_IMX6_NUM_I2CS 3
#define FSL_IMX6_NUM_GPIOS 7
#define FSL_IMX6_NUM_ESDHCS 4
#define FSL_IMX6_NUM_ECSPIS 5
#define FSL_IMX6_NUM_WDTS 2
#define FSL_IMX6_NUM_USB_PHYS 2
#define FSL_IMX6_NUM_USBS 4

typedef struct FslIMX6State {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    ARMCPU         cpu[FSL_IMX6_NUM_CPUS];
    A9MPPrivState  a9mpcore;
    IMX6CCMState   ccm;
    IMX6SRCState   src;
    IMXSerialState uart[FSL_IMX6_NUM_UARTS];
    IMXGPTState    gpt;
    IMXEPITState   epit[FSL_IMX6_NUM_EPITS];
    IMXI2CState    i2c[FSL_IMX6_NUM_I2CS];
    IMXGPIOState   gpio[FSL_IMX6_NUM_GPIOS];
    SDHCIState     esdhc[FSL_IMX6_NUM_ESDHCS];
    IMXSPIState    spi[FSL_IMX6_NUM_ECSPIS];
    IMX2WdtState   wdt[FSL_IMX6_NUM_WDTS];
    IMXUSBPHYState usbphy[FSL_IMX6_NUM_USB_PHYS];
    ChipideaState  usb[FSL_IMX6_NUM_USBS];
    IMXFECState    eth;
    MemoryRegion   rom;
    MemoryRegion   caam;
    MemoryRegion   ocram;
    MemoryRegion   ocram_alias;
    uint32_t       phy_num;
} FslIMX6State;


#define FSL_IMX6_MMDC_ADDR 0x10000000
#define FSL_IMX6_MMDC_SIZE 0xF0000000
#define FSL_IMX6_EIM_MEM_ADDR 0x08000000
#define FSL_IMX6_EIM_MEM_SIZE 0x8000000
#define FSL_IMX6_IPU_2_ADDR 0x02800000
#define FSL_IMX6_IPU_2_SIZE 0x400000
#define FSL_IMX6_IPU_1_ADDR 0x02400000
#define FSL_IMX6_IPU_1_SIZE 0x400000
#define FSL_IMX6_MIPI_HSI_ADDR 0x02208000
#define FSL_IMX6_MIPI_HSI_SIZE 0x4000
#define FSL_IMX6_OPENVG_ADDR 0x02204000
#define FSL_IMX6_OPENVG_SIZE 0x4000
#define FSL_IMX6_SATA_ADDR 0x02200000
#define FSL_IMX6_SATA_SIZE 0x4000
#define FSL_IMX6_AIPS_2_ADDR 0x02100000
#define FSL_IMX6_AIPS_2_SIZE 0x100000
/* AIPS2 */
#define FSL_IMX6_UART5_ADDR 0x021F4000
#define FSL_IMX6_UART5_SIZE 0x4000
#define FSL_IMX6_UART4_ADDR 0x021F0000
#define FSL_IMX6_UART4_SIZE 0x4000
#define FSL_IMX6_UART3_ADDR 0x021EC000
#define FSL_IMX6_UART3_SIZE 0x4000
#define FSL_IMX6_UART2_ADDR 0x021E8000
#define FSL_IMX6_UART2_SIZE 0x4000
#define FSL_IMX6_VDOA_ADDR 0x021E4000
#define FSL_IMX6_VDOA_SIZE 0x4000
#define FSL_IMX6_MIPI_DSI_ADDR 0x021E0000
#define FSL_IMX6_MIPI_DSI_SIZE 0x4000
#define FSL_IMX6_MIPI_CSI_ADDR 0x021DC000
#define FSL_IMX6_MIPI_CSI_SIZE 0x4000
#define FSL_IMX6_AUDMUX_ADDR 0x021D8000
#define FSL_IMX6_AUDMUX_SIZE 0x4000
#define FSL_IMX6_TZASC2_ADDR 0x021D4000
#define FSL_IMX6_TZASC2_SIZE 0x4000
#define FSL_IMX6_TZASC1_ADDR 0x021D0000
#define FSL_IMX6_TZASC1_SIZE 0x4000
#define FSL_IMX6_CSU_ADDR 0x021C0000
#define FSL_IMX6_CSU_SIZE 0x4000
#define FSL_IMX6_OCOTPCTRL_ADDR 0x021BC000
#define FSL_IMX6_OCOTPCTRL_SIZE 0x4000
#define FSL_IMX6_EIM_ADDR 0x021B8000
#define FSL_IMX6_EIM_SIZE 0x4000
#define FSL_IMX6_MMDC1_ADDR 0x021B4000
#define FSL_IMX6_MMDC1_SIZE 0x4000
#define FSL_IMX6_MMDC0_ADDR 0x021B0000
#define FSL_IMX6_MMDC0_SIZE 0x4000
#define FSL_IMX6_ROMCP_ADDR 0x021AC000
#define FSL_IMX6_ROMCP_SIZE 0x4000
#define FSL_IMX6_I2C3_ADDR 0x021A8000
#define FSL_IMX6_I2C3_SIZE 0x4000
#define FSL_IMX6_I2C2_ADDR 0x021A4000
#define FSL_IMX6_I2C2_SIZE 0x4000
#define FSL_IMX6_I2C1_ADDR 0x021A0000
#define FSL_IMX6_I2C1_SIZE 0x4000
#define FSL_IMX6_uSDHC4_ADDR 0x0219C000
#define FSL_IMX6_uSDHC4_SIZE 0x4000
#define FSL_IMX6_uSDHC3_ADDR 0x02198000
#define FSL_IMX6_uSDHC3_SIZE 0x4000
#define FSL_IMX6_uSDHC2_ADDR 0x02194000
#define FSL_IMX6_uSDHC2_SIZE 0x4000
#define FSL_IMX6_uSDHC1_ADDR 0x02190000
#define FSL_IMX6_uSDHC1_SIZE 0x4000
#define FSL_IMX6_MLB150_ADDR 0x0218C000
#define FSL_IMX6_MLB150_SIZE 0x4000
#define FSL_IMX6_ENET_ADDR 0x02188000
#define FSL_IMX6_ENET_SIZE 0x4000
#define FSL_IMX6_USBOH3_USB_ADDR 0x02184000
#define FSL_IMX6_USBOH3_USB_SIZE 0x4000
#define FSL_IMX6_AIPS2_CFG_ADDR 0x0217C000
#define FSL_IMX6_AIPS2_CFG_SIZE 0x4000
/* DAP */
#define FSL_IMX6_PTF_CTRL_ADDR 0x02160000
#define FSL_IMX6_PTF_CTRL_SIZE 0x1000
#define FSL_IMX6_PTM3_ADDR 0x0215F000
#define FSL_IMX6_PTM3_SIZE 0x1000
#define FSL_IMX6_PTM2_ADDR 0x0215E000
#define FSL_IMX6_PTM2_SIZE 0x1000
#define FSL_IMX6_PTM1_ADDR 0x0215D000
#define FSL_IMX6_PTM1_SIZE 0x1000
#define FSL_IMX6_PTM0_ADDR 0x0215C000
#define FSL_IMX6_PTM0_SIZE 0x1000
#define FSL_IMX6_CTI3_ADDR 0x0215B000
#define FSL_IMX6_CTI3_SIZE 0x1000
#define FSL_IMX6_CTI2_ADDR 0x0215A000
#define FSL_IMX6_CTI2_SIZE 0x1000
#define FSL_IMX6_CTI1_ADDR 0x02159000
#define FSL_IMX6_CTI1_SIZE 0x1000
#define FSL_IMX6_CTI0_ADDR 0x02158000
#define FSL_IMX6_CTI0_SIZE 0x1000
#define FSL_IMX6_CPU3_PMU_ADDR 0x02157000
#define FSL_IMX6_CPU3_PMU_SIZE 0x1000
#define FSL_IMX6_CPU3_DEBUG_IF_ADDR 0x02156000
#define FSL_IMX6_CPU3_DEBUG_IF_SIZE 0x1000
#define FSL_IMX6_CPU2_PMU_ADDR 0x02155000
#define FSL_IMX6_CPU2_PMU_SIZE 0x1000
#define FSL_IMX6_CPU2_DEBUG_IF_ADDR 0x02154000
#define FSL_IMX6_CPU2_DEBUG_IF_SIZE 0x1000
#define FSL_IMX6_CPU1_PMU_ADDR 0x02153000
#define FSL_IMX6_CPU1_PMU_SIZE 0x1000
#define FSL_IMX6_CPU1_DEBUG_IF_ADDR 0x02152000
#define FSL_IMX6_CPU1_DEBUG_IF_SIZE 0x1000
#define FSL_IMX6_CPU0_PMU_ADDR 0x02151000
#define FSL_IMX6_CPU0_PMU_SIZE 0x1000
#define FSL_IMX6_CPU0_DEBUG_IF_ADDR 0x02150000
#define FSL_IMX6_CPU0_DEBUG_IF_SIZE 0x1000
#define FSL_IMX6_CA9_INTEG_ADDR 0x0214F000
#define FSL_IMX6_CA9_INTEG_SIZE 0x1000
#define FSL_IMX6_FUNNEL_ADDR 0x02144000
#define FSL_IMX6_FUNNEL_SIZE 0x1000
#define FSL_IMX6_TPIU_ADDR 0x02143000
#define FSL_IMX6_TPIU_SIZE 0x1000
#define FSL_IMX6_EXT_CTI_ADDR 0x02142000
#define FSL_IMX6_EXT_CTI_SIZE 0x1000
#define FSL_IMX6_ETB_ADDR 0x02141000
#define FSL_IMX6_ETB_SIZE 0x1000
#define FSL_IMX6_DAP_ROM_TABLE_ADDR 0x02140000
#define FSL_IMX6_DAP_ROM_TABLE_SIZE 0x1000
/* DAP end */
#define FSL_IMX6_CAAM_ADDR 0x02100000
#define FSL_IMX6_CAAM_SIZE 0x10000
/* AIPS2 end */
#define FSL_IMX6_AIPS_1_ADDR 0x02000000
#define FSL_IMX6_AIPS_1_SIZE 0x100000
/* AIPS1 */
#define FSL_IMX6_SDMA_ADDR 0x020EC000
#define FSL_IMX6_SDMA_SIZE 0x4000
#define FSL_IMX6_DCIC2_ADDR 0x020E8000
#define FSL_IMX6_DCIC2_SIZE 0x4000
#define FSL_IMX6_DCIC1_ADDR 0x020E4000
#define FSL_IMX6_DCIC1_SIZE 0x4000
#define FSL_IMX6_IOMUXC_ADDR 0x020E0000
#define FSL_IMX6_IOMUXC_SIZE 0x4000
#define FSL_IMX6_PGCARM_ADDR 0x020DCA00
#define FSL_IMX6_PGCARM_SIZE 0x20
#define FSL_IMX6_PGCPU_ADDR 0x020DC260
#define FSL_IMX6_PGCPU_SIZE 0x20
#define FSL_IMX6_GPC_ADDR 0x020DC000
#define FSL_IMX6_GPC_SIZE 0x4000
#define FSL_IMX6_SRC_ADDR 0x020D8000
#define FSL_IMX6_SRC_SIZE 0x4000
#define FSL_IMX6_EPIT2_ADDR 0x020D4000
#define FSL_IMX6_EPIT2_SIZE 0x4000
#define FSL_IMX6_EPIT1_ADDR 0x020D0000
#define FSL_IMX6_EPIT1_SIZE 0x4000
#define FSL_IMX6_SNVSHP_ADDR 0x020CC000
#define FSL_IMX6_SNVSHP_SIZE 0x4000
#define FSL_IMX6_USBPHY2_ADDR 0x020CA000
#define FSL_IMX6_USBPHY2_SIZE 0x1000
#define FSL_IMX6_USBPHY1_ADDR 0x020C9000
#define FSL_IMX6_USBPHY1_SIZE 0x1000
#define FSL_IMX6_ANALOG_ADDR 0x020C8000
#define FSL_IMX6_ANALOG_SIZE 0x1000
#define FSL_IMX6_CCM_ADDR 0x020C4000
#define FSL_IMX6_CCM_SIZE 0x4000
#define FSL_IMX6_WDOG2_ADDR 0x020C0000
#define FSL_IMX6_WDOG2_SIZE 0x4000
#define FSL_IMX6_WDOG1_ADDR 0x020BC000
#define FSL_IMX6_WDOG1_SIZE 0x4000
#define FSL_IMX6_KPP_ADDR 0x020B8000
#define FSL_IMX6_KPP_SIZE 0x4000
#define FSL_IMX6_GPIO7_ADDR 0x020B4000
#define FSL_IMX6_GPIO7_SIZE 0x4000
#define FSL_IMX6_GPIO6_ADDR 0x020B0000
#define FSL_IMX6_GPIO6_SIZE 0x4000
#define FSL_IMX6_GPIO5_ADDR 0x020AC000
#define FSL_IMX6_GPIO5_SIZE 0x4000
#define FSL_IMX6_GPIO4_ADDR 0x020A8000
#define FSL_IMX6_GPIO4_SIZE 0x4000
#define FSL_IMX6_GPIO3_ADDR 0x020A4000
#define FSL_IMX6_GPIO3_SIZE 0x4000
#define FSL_IMX6_GPIO2_ADDR 0x020A0000
#define FSL_IMX6_GPIO2_SIZE 0x4000
#define FSL_IMX6_GPIO1_ADDR 0x0209C000
#define FSL_IMX6_GPIO1_SIZE 0x4000
#define FSL_IMX6_GPT_ADDR 0x02098000
#define FSL_IMX6_GPT_SIZE 0x4000
#define FSL_IMX6_CAN2_ADDR 0x02094000
#define FSL_IMX6_CAN2_SIZE 0x4000
#define FSL_IMX6_CAN1_ADDR 0x02090000
#define FSL_IMX6_CAN1_SIZE 0x4000
#define FSL_IMX6_PWM4_ADDR 0x0208C000
#define FSL_IMX6_PWM4_SIZE 0x4000
#define FSL_IMX6_PWM3_ADDR 0x02088000
#define FSL_IMX6_PWM3_SIZE 0x4000
#define FSL_IMX6_PWM2_ADDR 0x02084000
#define FSL_IMX6_PWM2_SIZE 0x4000
#define FSL_IMX6_PWM1_ADDR 0x02080000
#define FSL_IMX6_PWM1_SIZE 0x4000
#define FSL_IMX6_AIPS1_CFG_ADDR 0x0207C000
#define FSL_IMX6_AIPS1_CFG_SIZE 0x4000
#define FSL_IMX6_VPU_ADDR 0x02040000
#define FSL_IMX6_VPU_SIZE 0x3C000
#define FSL_IMX6_AIPS1_SPBA_ADDR 0x0203C000
#define FSL_IMX6_AIPS1_SPBA_SIZE 0x4000
#define FSL_IMX6_ASRC_ADDR 0x02034000
#define FSL_IMX6_ASRC_SIZE 0x4000
#define FSL_IMX6_SSI3_ADDR 0x02030000
#define FSL_IMX6_SSI3_SIZE 0x4000
#define FSL_IMX6_SSI2_ADDR 0x0202C000
#define FSL_IMX6_SSI2_SIZE 0x4000
#define FSL_IMX6_SSI1_ADDR 0x02028000
#define FSL_IMX6_SSI1_SIZE 0x4000
#define FSL_IMX6_ESAI_ADDR 0x02024000
#define FSL_IMX6_ESAI_SIZE 0x4000
#define FSL_IMX6_UART1_ADDR 0x02020000
#define FSL_IMX6_UART1_SIZE 0x4000
#define FSL_IMX6_eCSPI5_ADDR 0x02018000
#define FSL_IMX6_eCSPI5_SIZE 0x4000
#define FSL_IMX6_eCSPI4_ADDR 0x02014000
#define FSL_IMX6_eCSPI4_SIZE 0x4000
#define FSL_IMX6_eCSPI3_ADDR 0x02010000
#define FSL_IMX6_eCSPI3_SIZE 0x4000
#define FSL_IMX6_eCSPI2_ADDR 0x0200C000
#define FSL_IMX6_eCSPI2_SIZE 0x4000
#define FSL_IMX6_eCSPI1_ADDR 0x02008000
#define FSL_IMX6_eCSPI1_SIZE 0x4000
#define FSL_IMX6_SPDIF_ADDR 0x02004000
#define FSL_IMX6_SPDIF_SIZE 0x4000
/* AIPS1 end */
#define FSL_IMX6_PCIe_REG_ADDR 0x01FFC000
#define FSL_IMX6_PCIe_REG_SIZE 0x4000
#define FSL_IMX6_PCIe_ADDR 0x01000000
#define FSL_IMX6_PCIe_SIZE 0xFFC000
#define FSL_IMX6_GPV_1_PL301_CFG_ADDR 0x00C00000
#define FSL_IMX6_GPV_1_PL301_CFG_SIZE 0x100000
#define FSL_IMX6_GPV_0_PL301_CFG_ADDR 0x00B00000
#define FSL_IMX6_GPV_0_PL301_CFG_SIZE 0x100000
#define FSL_IMX6_PL310_ADDR 0x00A02000
#define FSL_IMX6_PL310_SIZE 0x1000
#define FSL_IMX6_A9MPCORE_ADDR 0x00A00000
#define FSL_IMX6_A9MPCORE_SIZE 0x2000
#define FSL_IMX6_OCRAM_ALIAS_ADDR 0x00940000
#define FSL_IMX6_OCRAM_ALIAS_SIZE 0xC0000
#define FSL_IMX6_OCRAM_ADDR 0x00900000
#define FSL_IMX6_OCRAM_SIZE 0x40000
#define FSL_IMX6_GPV_4_PL301_CFG_ADDR 0x00800000
#define FSL_IMX6_GPV_4_PL301_CFG_SIZE 0x100000
#define FSL_IMX6_GPV_3_PL301_CFG_ADDR 0x00300000
#define FSL_IMX6_GPV_3_PL301_CFG_SIZE 0x100000
#define FSL_IMX6_GPV_2_PL301_CFG_ADDR 0x00200000
#define FSL_IMX6_GPV_2_PL301_CFG_SIZE 0x100000
#define FSL_IMX6_DTCP_ADDR 0x00138000
#define FSL_IMX6_DTCP_SIZE 0x4000
#define FSL_IMX6_GPU_2D_ADDR 0x00134000
#define FSL_IMX6_GPU_2D_SIZE 0x4000
#define FSL_IMX6_GPU_3D_ADDR 0x00130000
#define FSL_IMX6_GPU_3D_SIZE 0x4000
#define FSL_IMX6_HDMI_ADDR 0x00120000
#define FSL_IMX6_HDMI_SIZE 0x9000
#define FSL_IMX6_BCH_ADDR 0x00114000
#define FSL_IMX6_BCH_SIZE 0x4000
#define FSL_IMX6_GPMI_ADDR 0x00112000
#define FSL_IMX6_GPMI_SIZE 0x2000
#define FSL_IMX6_APBH_BRIDGE_DMA_ADDR 0x00110000
#define FSL_IMX6_APBH_BRIDGE_DMA_SIZE 0x2000
#define FSL_IMX6_CAAM_MEM_ADDR 0x00100000
#define FSL_IMX6_CAAM_MEM_SIZE 0x4000
#define FSL_IMX6_ROM_ADDR 0x00000000
#define FSL_IMX6_ROM_SIZE 0x18000

#define FSL_IMX6_IOMUXC_IRQ 0
#define FSL_IMX6_DAP_IRQ 1
#define FSL_IMX6_SDMA_IRQ 2
#define FSL_IMX6_VPU_JPEG_IRQ 3
#define FSL_IMX6_SNVS_PMIC_IRQ 4
#define FSL_IMX6_IPU1_ERROR_IRQ 5
#define FSL_IMX6_IPU1_SYNC_IRQ 6
#define FSL_IMX6_IPU2_ERROR_IRQ 7
#define FSL_IMX6_IPU2_SYNC_IRQ 8
#define FSL_IMX6_GPU3D_IRQ 9
#define FSL_IMX6_R2D_IRQ 10
#define FSL_IMX6_V2D_IRQ 11
#define FSL_IMX6_VPU_IRQ 12
#define FSL_IMX6_APBH_BRIDGE_DMA_IRQ 13
#define FSL_IMX6_EIM_IRQ 14
#define FSL_IMX6_BCH_IRQ 15
#define FSL_IMX6_GPMI_IRQ 16
#define FSL_IMX6_DTCP_IRQ 17
#define FSL_IMX6_VDOA_IRQ 18
#define FSL_IMX6_SNVS_CONS_IRQ 19
#define FSL_IMX6_SNVS_SEC_IRQ 20
#define FSL_IMX6_CSU_IRQ 21
#define FSL_IMX6_uSDHC1_IRQ 22
#define FSL_IMX6_uSDHC2_IRQ 23
#define FSL_IMX6_uSDHC3_IRQ 24
#define FSL_IMX6_uSDHC4_IRQ 25
#define FSL_IMX6_UART1_IRQ 26
#define FSL_IMX6_UART2_IRQ 27
#define FSL_IMX6_UART3_IRQ 28
#define FSL_IMX6_UART4_IRQ 29
#define FSL_IMX6_UART5_IRQ 30
#define FSL_IMX6_ECSPI1_IRQ 31
#define FSL_IMX6_ECSPI2_IRQ 32
#define FSL_IMX6_ECSPI3_IRQ 33
#define FSL_IMX6_ECSPI4_IRQ 34
#define FSL_IMX6_ECSPI5_IRQ 35
#define FSL_IMX6_I2C1_IRQ 36
#define FSL_IMX6_I2C2_IRQ 37
#define FSL_IMX6_I2C3_IRQ 38
#define FSL_IMX6_SATA_IRQ 39
#define FSL_IMX6_USB_HOST1_IRQ 40
#define FSL_IMX6_USB_HOST2_IRQ 41
#define FSL_IMX6_USB_HOST3_IRQ 42
#define FSL_IMX6_USB_OTG_IRQ 43
#define FSL_IMX6_USB_PHY_UTMI0_IRQ 44
#define FSL_IMX6_USB_PHY_UTMI1_IRQ 45
#define FSL_IMX6_SSI1_IRQ 46
#define FSL_IMX6_SSI2_IRQ 47
#define FSL_IMX6_SSI3_IRQ 48
#define FSL_IMX6_TEMP_IRQ 49
#define FSL_IMX6_ASRC_IRQ 50
#define FSL_IMX6_ESAI_IRQ 51
#define FSL_IMX6_SPDIF_IRQ 52
#define FSL_IMX6_MLB150_IRQ 53
#define FSL_IMX6_PMU1_IRQ 54
#define FSL_IMX6_GPT_IRQ 55
#define FSL_IMX6_EPIT1_IRQ 56
#define FSL_IMX6_EPIT2_IRQ 57
#define FSL_IMX6_GPIO1_INT7_IRQ 58
#define FSL_IMX6_GPIO1_INT6_IRQ 59
#define FSL_IMX6_GPIO1_INT5_IRQ 60
#define FSL_IMX6_GPIO1_INT4_IRQ 61
#define FSL_IMX6_GPIO1_INT3_IRQ 62
#define FSL_IMX6_GPIO1_INT2_IRQ 63
#define FSL_IMX6_GPIO1_INT1_IRQ 64
#define FSL_IMX6_GPIO1_INT0_IRQ 65
#define FSL_IMX6_GPIO1_LOW_IRQ 66
#define FSL_IMX6_GPIO1_HIGH_IRQ 67
#define FSL_IMX6_GPIO2_LOW_IRQ 68
#define FSL_IMX6_GPIO2_HIGH_IRQ 69
#define FSL_IMX6_GPIO3_LOW_IRQ 70
#define FSL_IMX6_GPIO3_HIGH_IRQ 71
#define FSL_IMX6_GPIO4_LOW_IRQ 72
#define FSL_IMX6_GPIO4_HIGH_IRQ 73
#define FSL_IMX6_GPIO5_LOW_IRQ 74
#define FSL_IMX6_GPIO5_HIGH_IRQ 75
#define FSL_IMX6_GPIO6_LOW_IRQ 76
#define FSL_IMX6_GPIO6_HIGH_IRQ 77
#define FSL_IMX6_GPIO7_LOW_IRQ 78
#define FSL_IMX6_GPIO7_HIGH_IRQ 79
#define FSL_IMX6_WDOG1_IRQ 80
#define FSL_IMX6_WDOG2_IRQ 81
#define FSL_IMX6_KPP_IRQ 82
#define FSL_IMX6_PWM1_IRQ 83
#define FSL_IMX6_PWM2_IRQ 84
#define FSL_IMX6_PWM3_IRQ 85
#define FSL_IMX6_PWM4_IRQ 86
#define FSL_IMX6_CCM1_IRQ 87
#define FSL_IMX6_CCM2_IRQ 88
#define FSL_IMX6_GPC_IRQ 89
#define FSL_IMX6_SRC_IRQ 91
#define FSL_IMX6_CPU_L2_IRQ 92
#define FSL_IMX6_CPU_PARITY_IRQ 93
#define FSL_IMX6_CPU_PERF_IRQ 94
#define FSL_IMX6_CPU_CTI_IRQ 95
#define FSL_IMX6_SRC_COMB_IRQ 96
#define FSL_IMX6_MIPI_CSI1_IRQ 100
#define FSL_IMX6_MIPI_CSI2_IRQ 101
#define FSL_IMX6_MIPI_DSI_IRQ 102
#define FSL_IMX6_MIPI_HSI_IRQ 103
#define FSL_IMX6_SJC_IRQ 104
#define FSL_IMX6_CAAM0_IRQ 105
#define FSL_IMX6_CAAM1_IRQ 106
#define FSL_IMX6_ASC1_IRQ 108
#define FSL_IMX6_ASC2_IRQ 109
#define FSL_IMX6_FLEXCAN1_IRQ 110
#define FSL_IMX6_FLEXCAN2_IRQ 111
#define FSL_IMX6_HDMI_MASTER_IRQ 115
#define FSL_IMX6_HDMI_CEC_IRQ 116
#define FSL_IMX6_MLB150_LOW_IRQ 117
#define FSL_IMX6_ENET_MAC_IRQ 118
#define FSL_IMX6_ENET_MAC_1588_IRQ 119
#define FSL_IMX6_PCIE1_IRQ 120
#define FSL_IMX6_PCIE2_IRQ 121
#define FSL_IMX6_PCIE3_IRQ 122
#define FSL_IMX6_PCIE4_IRQ 123
#define FSL_IMX6_DCIC1_IRQ 124
#define FSL_IMX6_DCIC2_IRQ 125
#define FSL_IMX6_MLB150_HIGH_IRQ 126
#define FSL_IMX6_PMU2_IRQ 127
#define FSL_IMX6_MAX_IRQ 128

#endif /* FSL_IMX6_H */
