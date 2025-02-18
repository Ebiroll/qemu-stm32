/*
 * STM32 Microcontroller RCC (Reset and Clock Control) module
 *
 * Copyright (C) 2023 Olof Astrand
 *
 * Source code based on omap_clk.c
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
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
#include "qapi/error.h"
#include "cpu.h"

#include "qemu/timer.h"
#include <stdio.h>

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "stm32fxxx_clktree.h"
#include "stm32u535_rcc.h"
#include "qemu/log.h"
//#include "hw/arm/armv7m.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"


extern int system_clock_scale;

/* DEFINITIONS*/

/* See README for DEBUG details. */
#define DEBUG_STM32_RCC

#ifdef DEBUG_STM32_RCC
#define DPRINTF(fmt, ...)                                       \
do { printf("STM32U535_RCC: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define IS_RESET_VALUE(new_value, mask, reset_value) ((new_value & mask) == (mask & reset_value))

#define WARN_UNIMPLEMENTED(new_value, mask, reset_value) \
    if (!IS_RESET_VALUE(new_value, mask, reset_value)) { \
        qemu_log_mask(LOG_UNIMP, "Not implemented: RCC " #mask ". Masked value: 0x%08x\n", (new_value & mask)); \
    }

#define WARN_UNIMPLEMENTED_REG(offset) \
        qemu_log_mask(LOG_UNIMP, "STM32u535_rcc: unimplemented register: 0x%x", (int)offset)

#define HSI_FREQ 16000000
#define LSI_FREQ 32000
/*
#define RCC_FLAG_HSIRDY                  ((uint8_t)0x21)
#define RCC_FLAG_HSERDY                  ((uint8_t)0x31)
#define RCC_FLAG_PLLRDY                  ((uint8_t)0x39)
#define RCC_FLAG_PLLI2SRDY               ((uint8_t)0x3B)
*/
// SAI & SAI1 and SAI2, 
#if 0
#define RCC_CR_MSISON_Pos                   (0U)
#define RCC_CR_MSIKERON_Pos                 (1U)
#define RCC_CR_MSISRDY_Pos                  (2U)
#define RCC_CR_MSIPLLEN_Pos                 (3U)
#define RCC_CR_MSIKON_Pos                   (4U)


#define RCC_CR_MSIKRDY_Pos                  (5U)
#define RCC_CR_MSIPLLSEL_Pos                (6U)
#define RCC_CR_MSIPLLFAST_Pos               (7U)
#define RCC_CR_HSION_Pos                    (8U)
#define RCC_CR_HSIKERON_Pos                 (9U)
#define RCC_CR_HSIRDY_Pos                   (10U)
#define RCC_CR_HSI48ON_Pos                  (12U)
#define RCC_CR_HSI48RDY_Pos                 (13U)
#define RCC_CR_SHSION_Pos                   (14U)
#define RCC_CR_SHSIRDY_Pos                  (15U)

#define RCC_CR_HSEON_Pos                    (16U)
#define RCC_CR_HSERDY_Pos                   (17U)
#define RCC_CR_HSEBYP_Pos                   (18U)
#define RCC_CR_CSSON_Pos                    (19U)
#define RCC_CR_HSEEXT_Pos                   (20U)
#define RCC_CR_PLL1ON_Pos                   (24U)
#define RCC_CR_PLL1RDY_Pos                  (25U)
#define RCC_CR_PLL2ON_Pos                   (26U)
#define RCC_CR_PLL2RDY_Pos                  (27U)
#define RCC_CR_PLL3ON_Pos                   (28U)
#define RCC_CR_PLL3RDY_Pos                  (29U)
#endif
#define RCC_CR_RESET_VALUE      0x00000035
#define RCC_CR_OFFSET           0x00
//#define RCC_CR_PLLSAI2RDY_BIT   29    // SAI2 PLL clock ready flag
//#define RCC_CR_PLLSAI2ON_BIT    28   // SAI2 PLL enable
//#define RCC_CR_PLLSAI1RDY_BIT   27   // SAI1 PLL clock ready flag
//#define RCC_CR_PLLSAI1ON_BIT    26   // SAI1 PLL enable

#define RCC_CR_PLL3RDY_BIT                  (29U)
#define RCC_CR_PLL3ON_BIT                   (28U)
#define RCC_CR_PLL2RDY_BIT                  (27U)
#define RCC_CR_PLL2ON_BIT                   (26U)
#define RCC_CR_PLL1RDY_BIT       25   // Main PLL (PLL) clock ready flag 
#define RCC_CR_PLL1ON_BIT        24   // Main PLL (PLL) enable
#define RCC_CR_CSSON_BIT        19   // Clock security system enable
#define RCC_CR_HSEBYP_BIT       18   // HSE clock bypass
#define RCC_CR_HSERDY_BIT       17   // HSE clock ready flag
#define RCC_CR_HSEON_BIT        16   // HSE clock enable
#define RCC_CR_HSI48RDY_BIT     13
//#define RCC_CR_HSIASFS_BIT      11   // HSI automatic start from Stop
#define RCC_CR_HSIRDY_BIT         10   // HSI clock ready flag
#define RCC_CR_HSION_BIT           8    // HSI clock enable
#define RCC_CR_MSIRDY_BIT          2    // MSI clock ready flag
#define RCC_CR_MSION_BIT           0    // MSI clock enable

#define RCC_CR_HSICAL_START     8
#define RCC_CR_HSICAL_MASK      0x00ff0000
#define RCC_CR_HSITRIM_START    3
#define RCC_CR_HSITRIM_MASK     0x000000f8

#define RCC_ICSCR_OFFSET         0x04

#define RCC_CFGR1_RESET_VALUE     0x00000000
#define RCC_CFGR1_OFFSET          0x1C


#define RCC_CFGR2_OFFSET          0x20
#define RCC_CFGR3_OFFSET          0x24

#define RCC_PLL1CFGR_POS          0x28
#define RCC_CIER_OFFSET           0x50

#define RCC_CR_HSI48RDY_Pos                 (13U)
#define RCC_CR_HSI48RDY_Msk                 (0x1UL << RCC_CR_HSI48RDY_Pos)          /*!< 0x000002000 */
#define RCC_CR_HSI48RDY                     RCC_CR_HSI48RDY_Msk                     /*!< Internal High Speed Oscillator (HSI48) Clock Ready Flag */


#define RCC_CFGR_PPRE2_START     11
#define RCC_CFGR_PPRE2_MASK      0x0000E000
#define RCC_CFGR_PPRE1_START     8
#define RCC_CFGR_PPRE1_MASK      0x00001E00
#define RCC_CFGR_HPRE_START      4
#define RCC_CFGR_HPRE_MASK       0x000000F0
#define RCC_CFGR_SWS_START       2
#define RCC_CFGR_SWS_MASK        0x0000000C
#define RCC_CFGR_SW_START        0
#define RCC_CFGR_SW_MASK         0x00000003

#define RCC_PLLCFGR_RESET_VALUE  0x00001000
#define RCC_PLLCFGR_OFFSET       0x0C
#define RCC_PLLCFGR_PLLQ_MASK    0x00300000
#define RCC_PLLCFGR_PLLQ_START   21
#define RCC_PLLCFGR_PLLSRC_BIT   0
#define RCC_PLLCFGR_PLLP_MASK    0x00030000
#define RCC_PLLCFGR_PLLP_START   17
//          <bitOffset>8</bitOffset>
//              <bitWidth>7</bitWidth>
#define RCC_PLLCFGR_PLLN_START  8
#define RCC_PLLCFGR_PLLN_MASK   0x0000003F



#define RCC_PLLCFGR_PLLM_START   4

#define RCC_PLLSAI1CFGR_RESET_VALUE  0x00001000
#define RCC_PLLSAI1CFGR_OFFSET       0x10


#define RCC_CIR_RESET_VALUE      0x00000000
#define RCC_CIR_OFFSET           0x20
#define RCC_CIR_CSSC_BIT         23
#define RCC_CIR_PLLI2SRDYC_BIT   21
#define RCC_CIR_PLLRDYC_BIT      20
#define RCC_CIR_HSERDYC_BIT      19
#define RCC_CIR_HSIRDYC_BIT      18
#define RCC_CIR_LSERDYC_BIT      17
#define RCC_CIR_LSIRDYC_BIT      16
#define RCC_CIR_PLLI2SRDYIE_BIT  13
#define RCC_CIR_PLLRDYIE_BIT     12
#define RCC_CIR_HSERDYIE_BIT     11
#define RCC_CIR_HSIRDYIE_BIT     10
#define RCC_CIR_LSERDYIE_BIT     9
#define RCC_CIR_LSIRDYIE_BIT     8
#define RCC_CIR_CSSF_BIT         7
#define RCC_CIR_PLLI2SRDYF_BIT  5
#define RCC_CIR_PLLRDYF_BIT     4
#define RCC_CIR_HSERDYF_BIT     3
#define RCC_CIR_HSIRDYF_BIT     2
#define RCC_CIR_LSERDYF_BIT     1
#define RCC_CIR_LSIRDYF_BIT     0



#define RCC_CRRCR_RESET_VALUE   0x00000003
#define RCC_CRRCR_OFFSET        0x14

//RCC_CFGR1
#define RCC_CFGR1_RESET_VALUE   0x00000000
#define RCC_CFGR1_OFFSET        0x1C

// RCC_CFGR3
#define RCC_CFGR3_RESET_VALUE   0x00000000
#define RCC_CFGR3_OFFSET        0x24

#define RCC_PLL1CFGR_RESET_VALUE  0x00000000
#define RCC_PLL1CFGR_OFFSET       0x28

#define RCC_PLL2CFGR_RESET_VALUE  0x00000000
#define RCC_PLL2CFGR_OFFSET       0x2C

// RCC_PLL3CFGR
#define RCC_PLL3CFGR_RESET_VALUE  0x00000000
#define RCC_PLL3CFGR_OFFSET       0x30

#define RCC_PLL1DIVR_RESET_VALUE  0x01010280
#define RCC_PLL1DIVR_OFFSET       0x34

#define RCC_PLL1FRACR_RESET_VALUE 0x00000000
#define RCC_PLL1FRACR_OFFSET      0x38

#define RCC_PLL2DIVR_RESET_VALUE  0x01010280
#define RCC_PLL2DIVR_OFFSET       0x3C

#define RCC_PLL2FRACR_RESET_VALUE 0x00000000
#define RCC_PLL2FRACR_OFFSET      0x40

#define RCC_PLL3DIVR_RESET_VALUE  0x01010280
#define RCC_PLL3DIVR_OFFSET       0x44

#define RCC_PLL3FRACR_RESET_VALUE 0x00000000
#define RCC_PLL3FRACR_OFFSET      0x48

#define RCC_CIER_RESET_VALUE      0x00000000
#define RCC_CIER_OFFSET           0x50

#define RCC_CIFR_RESET_VALUE      0x00000000
#define RCC_CIFR_OFFSET           0x54

#define RCC_CICR_RESET_VALUE      0x00000000
#define RCC_CICR_OFFSET           0x58

#define RCC_AHB1RSTR_RESET_VALUE  0x00000000
#define RCC_AHB1RSTR_OFFSET       0x60

#define RCC_AHB2RSTR1_RESET_VALUE 0x00000000
#define RCC_AHB2RSTR1_OFFSET      0x64

#define RCC_AHB2RSTR2_RESET_VALUE 0x00000000
#define RCC_AHB2RSTR2_OFFSET      0x68

#define RCC_AHB3RSTR_RESET_VALUE  0x00000000
#define RCC_AHB3RSTR_OFFSET       0x6C

#define RCC_APB1RSTR1_RESET_VALUE 0x00000000
#define RCC_APB1RSTR1_OFFSET      0x74

#define RCC_APB1RSTR2_RESET_VALUE 0x00000000
#define RCC_APB1RSTR2_OFFSET      0x78

#define RCC_APB2RSTR_RESET_VALUE  0x00000000
#define RCC_APB2RSTR_OFFSET       0x7C

#define RCC_APB3RSTR_RESET_VALUE  0x00000000
#define RCC_APB3RSTR_OFFSET       0x80

#define RCC_AHB1ENR_RESET_VALUE   0x00100000
#define RCC_AHB1ENR_OFFSET        0x88

#define RCC_AHB2ENR1_RESET_VALUE  0x40000000
#define RCC_AHB2ENR1_OFFSET       0x8C

#define RCC_AHB2ENR2_RESET_VALUE  0x00000000
#define RCC_AHB2ENR2_OFFSET       0x90

#define RCC_AHB3ENR_RESET_VALUE   0x00000000
#define RCC_AHB3ENR_OFFSET        0x94

#define RCC_APB1ENR1_RESET_VALUE  0x00000000
#define RCC_APB1ENR1_OFFSET       0x9C

#define RCC_APB1ENR2_RESET_VALUE  0x00000000
#define RCC_APB1ENR2_OFFSET       0xA0

#define RCC_APB2ENR_RESET_VALUE   0x00000000
#define RCC_APB2ENR_OFFSET        0xA4

#define RCC_APB3ENR_RESET_VALUE   0x00000000
#define RCC_APB3ENR_OFFSET        0xA8

#define RCC_AHB1SMENR_RESET_VALUE  0xFFFFFFFF
#define RCC_AHB1SMENR_OFFSET       0xB0

#define RCC_AHB2SMENR1_RESET_VALUE 0xFFFFFFFF
#define RCC_AHB2SMENR1_OFFSET      0xB4

#define RCC_AHB2SMENR2_RESET_VALUE 0xFFFFFFFF
#define RCC_AHB2SMENR2_OFFSET      0xB8

#define RCC_AHB3SMENR_RESET_VALUE  0xFFFFFFFF
#define RCC_AHB3SMENR_OFFSET       0xBC

#define RCC_APB1SMENR1_RESET_VALUE 0xFFFFFFFF
#define RCC_APB1SMENR1_OFFSET      0xC4

#define RCC_APB1SMENR2_RESET_VALUE 0xFFFFFFFF
#define RCC_APB1SMENR2_OFFSET      0xC8

#define RCC_APB2SMENR_RESET_VALUE  0xFFFFFFFF
#define RCC_APB2SMENR_OFFSET       0xCC

#define RCC_APB3SMENR_RESET_VALUE  0xFFFFFFFF
#define RCC_APB3SMENR_OFFSET       0xD0

#define RCC_SRDAMR_RESET_VALUE     0x00000000
#define RCC_SRDAMR_OFFSET          0xD8

#define RCC_CCIPR1_RESET_VALUE     0x00000000
#define RCC_CCIPR1_OFFSET          0xE0

#define RCC_CCIPR2_RESET_VALUE     0x00000000
#define RCC_CCIPR2_OFFSET          0xE4

#define RCC_CCIPR3_RESET_VALUE     0x00000000
#define RCC_CCIPR3_OFFSET          0xE8

#define RCC_BDCR_RESET_VALUE       0x00000000
#define RCC_BDCR_OFFSET            0xF0

#define RCC_CSR_RESET_VALUE        0x0C000000
#define RCC_CSR_OFFSET             0xF4

#define RCC_SECCFGR_RESET_VALUE    0x00000000
#define RCC_SECCFGR_OFFSET         0x110

#define RCC_PRIVCFGR_RESET_VALUE   0x00000000
#define RCC_PRIVCFGR_OFFSET        0x114

#define RCC_BDCR_LSEON_BIT        0
#define RCC_BDCR_LSERDY_BIT       1
#define RCC_BDCR_LSEBPY_BIT       2
#define RCC_BDCR_LSEDRV_START     3
#define RCC_BDCR_LSEDRV_MASK      0x00000018

#define RCC_CSR_LSECSON_BIT       5
#define RCC_CSR_LSECSSD_BIT       6
#define RCC_CSR_LSESYSON_BIT      7

#define RCC_BDCR_LSESYSRDY        11

#define RCC_CSR_LSION_BIT         26
#define RCC_CSR_LSIRDY_BIT        27





#define SW_HSI_SELECTED 0
#define SW_HSE_SELECTED 1
#define SW_PLL_SELECTED 2

#define GET_BIT_MASK(position, value) ((value ? 1 : 0) << position)
#define GET_BIT_MASK_ONE(position) (1 << position)
#define GET_BIT_MASK_ZERO(position) (~(1 << position))
#define GET_BIT_VALUE(value, position) \
                ((value & GET_BIT_MASK_ONE(position)) >> position)
#define IS_BIT_SET(value, position) ((value & GET_BIT_MASK_ONE(position)) != 0)
#define IS_BIT_RESET(value, position) ((value & GET_BIT_MASK_ONE(position)) ==0)
#define RESET_BIT(var, position) var &= GET_BIT_MASK_ZERO(position)

/* Can be true, false, 0, or 1 */
#define CHANGE_BIT(var, position, new_value) \
            var = new_value ? \
                    (var | GET_BIT_MASK_ONE(position)) : \
                    (var & GET_BIT_MASK_ZERO(position))
#define CHANGE_BITS(var, start, mask, new_value) \
            var = (var & ~mask) | ((new_value << start) & mask)

# define STM32_BAD_REG(offset, size)       \
        hw_error("%s: Bad register 0x%x - size %u\n", __FUNCTION__, (int)offset, size)
# define STM32_WARN_RO_REG(offset)        \
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Read-only register 0x%x\n", \
                      __FUNCTION__, (int)offset)
# define STM32_WARN_WO_REG(offset)        \
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Write-only register 0x%x\n", \
                      __FUNCTION__, (int)offset)
# define STM32_NOT_IMPL_REG(offset, size)      \
        hw_error("%s: Not implemented yet 0x%x - size %u\n", __FUNCTION__, (int)offset, size)
void stm32_hw_warn(const char *fmt, ...)
    __attribute__ ((__format__ (__printf__, 1, 2)));

#define stm32_unimp(x...) qemu_log_mask(LOG_UNIMP, x)


// #define stm32f1xx_rcc STM32u535RccState
#if 0
struct stm32f1xx_rcc {
    /* Inherited */
    union {
        struct STM32FXXXRccState inherited;
        struct {
            /* Inherited */
            SysBusDevice busdev;

            /* Properties */
            uint32_t osc_freq;
            uint32_t osc32_freq;

            /* Private */
            MemoryRegion iomem;
            qemu_irq irq;
        };
    };
    
    /* Peripheral clocks */
    Clk PERIPHCLK[STM32_PERIPH_COUNT]; // MUST be first field after `inherited`, because Stm32Rcc's last field aliases this array

};
#endif

/* HELPER FUNCTIONS */
struct Clk {                                                                    
    const char *name;                                                           
                                                                                
    bool enabled;                                                               
                                                                                
    uint32_t input_freq, output_freq, max_output_freq;                          
                                                                                
    uint16_t multiplier, divisor;                                               
                                                                                
    unsigned user_count;                                                        
    qemu_irq user[CLKTREE_MAX_IRQ]; /* Who to notify on change */               
                                                                                
    unsigned output_count;                                                      
    struct Clk *output[CLKTREE_MAX_OUTPUT];                                     
                                                                                
    unsigned input_count;                                                       
    int selected_input;                                                         
    struct Clk *input[CLKTREE_MAX_INPUT];                                       
};                                                                              
                                                                                

/* Enable the peripheral clock if the specified bit is set in the value. */
/*
static void stm32_rcc_periph_enable(
                                    struct STM32u535RccState *s,
                                    uint32_t new_value,
                                    bool init,
                                    int periph,
                                    uint32_t bit_mask)
{
    printf("rcc set 0x%x %s %d %x: %sable\n", new_value, s->PERIPHCLK[periph]->name, periph, bit_mask, (new_value & (1 << bit_mask))?"en":"dis");
    clktree_set_enabled(s->PERIPHCLK[periph], (new_value & (1 << bit_mask)) != 0);
}

*/



/* REGISTER IMPLEMENTATION */

/* Read the configuration register.
   NOTE: Not implemented: CSS, PLLI2S, clock calibration and trimming. */
static uint32_t stm32_rcc_RCC_CR_read(struct STM32u535RccState *s)
{
    /* Get the status of the clocks. */
    const bool PLL1ON = clktree_is_enabled(s->PLL1CLK); 
    const bool PLL2ON = clktree_is_enabled(s->PLL2CLK); 
    const bool PLL3ON = clktree_is_enabled(s->PLL3CLK); 

    const bool HSEON = true; //clktree_is_enabled(s->HSECLK);
    const bool HSION = true; //clktree_is_enabled(s->HSICLK);
    const bool PLLRDY = true;
     const bool HSI48RDY = true;

    /*
    RCC_CR_HSI48RDY
    */

uint32_t new_value =  GET_BIT_MASK(RCC_CR_PLL1RDY_BIT, PLL1ON) |
        GET_BIT_MASK(RCC_CR_PLL1ON_BIT, PLL1ON) |
        GET_BIT_MASK(RCC_CR_PLL2ON_BIT, PLL2ON) |
        GET_BIT_MASK(RCC_CR_PLL3ON_BIT, PLL3ON) |

        GET_BIT_MASK(RCC_CR_HSERDY_BIT, HSEON) |
        GET_BIT_MASK(RCC_CR_HSEON_BIT, HSEON) |

        GET_BIT_MASK(RCC_CR_HSIRDY_BIT, HSION) |
        GET_BIT_MASK(RCC_CR_HSION_BIT, HSION) |

        GET_BIT_MASK(RCC_CR_PLL1RDY_BIT,PLLRDY) |
        GET_BIT_MASK(RCC_CR_HSI48RDY_BIT,HSI48RDY);



    DPRINTF("RCC_CR_read %lu \n",
                (unsigned long)new_value);
    /* build the register value based on the clock states.  If a clock is on,
     * then its ready bit is always set.
     */
    return (
        GET_BIT_MASK(RCC_CR_PLL1RDY_BIT, PLL1ON) |
        GET_BIT_MASK(RCC_CR_PLL1ON_BIT, PLL1ON) |

        GET_BIT_MASK(RCC_CR_HSERDY_BIT, HSEON) |
        GET_BIT_MASK(RCC_CR_HSEON_BIT, HSEON) |

        GET_BIT_MASK(RCC_CR_HSIRDY_BIT, HSION) |
        GET_BIT_MASK(RCC_CR_HSION_BIT, HSION)  |
        GET_BIT_MASK(RCC_CR_HSI48RDY_BIT,HSI48RDY)
    );
}

/* Write the Configuration Register.
 * This updates the states of the corresponding clocks.  The bit values are not
 * saved - when the register is read, its value will be built using the clock
 * states.
 */
static void stm32_rcc_RCC_CR_write(struct STM32u535RccState *s, uint32_t new_value, bool init)
{
    bool new_MSION , new_PLL1ON, new_PLL2ON, new_PLL3ON , new_HSEON, new_HSION, new_PLLISON, new_CSSON;
    DPRINTF("RCC_CR_write %lu \n",
            (unsigned long)new_value);
    // RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLL1ON | RCC_CR_PLL2ON | RCC_CR_PLL3ON)

    new_MSION = IS_BIT_SET(new_value, RCC_CR_MSION_BIT);
    clktree_set_enabled(s->MSICLK, new_MSION);


    new_PLL1ON = IS_BIT_SET(new_value, RCC_CR_PLL1ON_BIT);
    if((clktree_is_enabled(s->PLL1CLK) && !new_PLL1ON) &&
       s->RCC_CFGR_SW == SW_PLL_SELECTED) {
        printf("PLL cannot be disabled while it is selected as the system clock.");
    }
    clktree_set_enabled(s->PLL1CLK, new_PLL1ON);


    new_PLL2ON = IS_BIT_SET(new_value, RCC_CR_PLL2ON_BIT);
    if((clktree_is_enabled(s->PLL2CLK) && !new_PLL2ON) &&
       s->RCC_CFGR_SW == SW_PLL_SELECTED) {
        printf("PLL cannot be disabled while it is selected as the system clock.");
    }
    clktree_set_enabled(s->PLL2CLK, new_PLL2ON);

    new_PLL3ON = IS_BIT_SET(new_value, RCC_CR_PLL3ON_BIT);
    if((clktree_is_enabled(s->PLL3CLK) && !new_PLL3ON) &&
       s->RCC_CFGR_SW == SW_PLL_SELECTED) {
        printf("PLL cannot be disabled while it is selected as the system clock.");
    }
    clktree_set_enabled(s->PLL3CLK, new_PLL3ON);


    DPRINTF("RCC_CR_was %lu \n",
                (unsigned long)new_PLL1ON);

    clktree_set_enabled(s->PLL1CLK, new_PLL1ON);

    DPRINTF("PLL On %lu \n",
            (unsigned long)new_PLL1ON);


    new_HSEON = IS_BIT_SET(new_value, RCC_CR_HSEON_BIT);
    if((clktree_is_enabled(s->HSECLK) && !new_HSEON) &&
       (s->RCC_CFGR_SW == SW_HSE_SELECTED || s->RCC_CFGR_SW == SW_PLL_SELECTED)
       ) {
        printf("HSE oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(s->HSECLK, new_HSEON);

    new_HSION = IS_BIT_SET(new_value, RCC_CR_HSION_BIT);
    if((clktree_is_enabled(s->HSECLK) && !new_HSEON) &&
       (s->RCC_CFGR_SW == SW_HSI_SELECTED || s->RCC_CFGR_SW == SW_PLL_SELECTED)
       ) {
        printf("HSI oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(s->HSICLK, new_HSION);

    new_CSSON = IS_BIT_SET(new_value, RCC_CR_CSSON_BIT);
    clktree_set_enabled(s->CSCLK, new_CSSON);


    new_PLLISON = IS_BIT_SET(new_value, RCC_CR_PLL1ON_BIT);
    clktree_set_enabled(s->PLLI2SCLK, new_PLLISON);

    //WARN_UNIMPLEMENTED(new_value, 1 << RCC_CR_CSSON_BIT, RCC_CR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CR_HSICAL_MASK, RCC_CR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CR_HSITRIM_MASK, RCC_CR_RESET_VALUE);
}


static uint32_t stm32_rcc_RCC_PLLCFGR_read(struct STM32u535RccState *s)
{
    return s->RCC_PLLCFGR;
}

static void stm32_rcc_RCC_PLLCFGR_write(struct STM32u535RccState *s, uint32_t new_value, bool init)
{
    /* PLLM division factor */
    //const uint8_t new_PLLM = (new_value & RCC_PLLCFGR_PLLM_MASK) >> RCC_PLLCFGR_PLLM_START;
    //if (new_PLLM <= 1) {
    //    hw_error("PLLM division factor cannot be 0 or 1. Given: %u", new_PLLM);
    //}

    /* PLLN multiplication factor */
    const uint16_t new_PLLN = (new_value & RCC_PLLCFGR_PLLN_MASK) >> RCC_PLLCFGR_PLLN_START;
    //if (new_PLLN <= 1 || new_PLLN >= 433) {
    //    hw_error("PLLN multiplication factor must be between 2 and 432 (inclusive). Given: %u", new_PLLN);
    //}

    /* PLLSRC */
    const uint8_t new_PLLSRC = IS_BIT_SET(new_value, RCC_PLLCFGR_PLLSRC_BIT);

    /* PPLP division factor */
    const uint8_t new_PLLP = 2 + (2 * ((new_value & RCC_PLLCFGR_PLLP_MASK) >> RCC_PLLCFGR_PLLP_START));

    /* Warn in case of illegal writes: 
    if (init == false) {
        const bool are_disabled = (!clktree_is_enabled(s->PLL1CLK) // && TODO: !clktree_is_enabled(s->PLLI2SCLK) );
        if (are_disabled == false) {
            const char *warning_fmt = "Can only change %s while PLL and PLLI2S are disabled\n";
            if (new_PLLM != s->RCC_PLLCFGR_PLLM) {
                printf(warning_fmt, "PLLM");
            }
            if (new_PLLN != s->RCC_PLLCFGR_PLLN) {
                printf(warning_fmt, "PLLN");
            }
            if (new_PLLSRC != s->RCC_PLLCFGR_PLLSRC) {
                printf(warning_fmt, "PLLSRC");
            }
        }
    }
*/
    /* Save new register value */
    s->RCC_PLLCFGR = new_value;

    /* Set the new values: */
    //s->RCC_PLLCFGR_PLLM = new_PLLM;
    s->RCC_PLLCFGR_PLLN = new_PLLN;
    //clktree_set_scale(s->PLLM, new_PLLN, new_PLLM);

    s->RCC_PLLCFGR_PLLSRC = new_PLLSRC;
    //clktree_set_selected_input(s->PLLM, new_PLLSRC);

    s->RCC_PLLCFGR_PLLP = new_PLLP;
    clktree_set_scale(s->PLL1CLK, 1, new_PLLP);

    //WARN_UNIMPLEMENTED(new_value, RCC_PLLCFGR_PLLQ_MASK, RCC_PLLCFGR_RESET_VALUE);
}

#if 0

static uint32_t stm32_rcc_RCC_PLLI2SCFGR_read(struct STM32u535RccState *s)
{
    return s->RCC_PLLI2SCFGR;
}

static void stm32_rcc_RCC_PLLI2SCFGR_write(struct STM32u535RccState *s, uint32_t new_value, bool init)
{
    /* PLLR division factor */
    //const uint16_t new_PLLR = (new_value & RCC_PLLI2SCFGR_PLLR_MASK) >> RCC_PLLI2SCFGR_PLLR_START;
    //if (new_PLLR < 2 || new_PLLR > 7) {
    //    hw_error("PLLR multiplication factor must be between 2 and 7. Given: %u", new_PLLR);
    //}

    /* PLLQ division factor */
    //const uint16_t new_PLLQ = (new_value & RCC_PLLI2SCFGR_PLLQ_MASK) >> RCC_PLLI2SCFGR_PLLQ_START;
    //if (new_PLLQ > 15) {
    //    hw_error("PLLQ multiplication factor must be between 0 and 15 "
    //             "(inclusive). Given: %u", new_PLLQ);
    //}

    /* PLLN multiplication factor */
    //const uint16_t new_PLLN = (new_value & RCC_PLLI2SCFGR_PLLN_MASK) >> RCC_PLLI2SCFGR_PLLN_START;
    //if (new_PLLN < 2 || new_PLLN > 433) {
    //    hw_error("PLLN multiplication factor must be between 2 and 432 (inclusive). Given: %u", new_PLLN);
    //}


    /* Warn in case of illegal writes: */
    if (init == false) {
        const bool are_disabled = (!clktree_is_enabled(s->PLLI2SCLK) /* && TODO: !clktree_is_enabled(s->PLLI2SCLK) */);
        if (are_disabled == false) {
            const char *warning_fmt = "Can only change %s while PLL and PLLI2S are disabled";
            //if (new_PLLR != s->RCC_PLLI2SCFGR_PLLR) {
            //    printf(warning_fmt, "PLLR");
            //}
            //if (new_PLLQ != s->RCC_PLLI2SCFGR_PLLQ) {
            //    printf(warning_fmt, "PLLQ");
            //}
            //if (new_PLLN != s->RCC_PLLCFGR_PLLN) {
            //    printf(warning_fmt, "PLLN");
            //}
        }
    }

    /* Save new register value */
    s->RCC_PLLI2SCFGR = new_value;

    /* Set the new values: */
    s->RCC_PLLI2SCFGR_PLLR = new_PLLR;
    s->RCC_PLLI2SCFGR_PLLQ = new_PLLQ;
    s->RCC_PLLI2SCFGR_PLLN = new_PLLN;
    clktree_set_scale(s->PLLI2SM, new_PLLN, s->RCC_PLLCFGR_PLLM );

    clktree_set_scale(s->PLLI2SCLK, 1, new_PLLR);
    //WARN_UNIMPLEMENTED(new_value, RCC_PLLI2SCFGR_PLLQ_MASK, RCC_PLLI2SCFGR_RESET_VALUE);
}
#endif

static uint32_t stm32_rcc_RCC_CFGR_read(struct STM32u535RccState *s)
{
    return
    (s->RCC_CFGR_PPRE2 << RCC_CFGR_PPRE2_START) |
    (s->RCC_CFGR_PPRE1 << RCC_CFGR_PPRE1_START) |
    (s->RCC_CFGR_HPRE << RCC_CFGR_HPRE_START) |
    (s->RCC_CFGR_SW << RCC_CFGR_SW_START) |
    (s->RCC_CFGR_SW << RCC_CFGR_SWS_START);
}

static void stm32_rcc_RCC_CFGR_write(struct STM32u535RccState *s, uint32_t new_value, bool init)
{
    /* PPRE2 */
    s->RCC_CFGR_PPRE2 = (new_value & RCC_CFGR_PPRE2_MASK) >> RCC_CFGR_PPRE2_START;
    if(s->RCC_CFGR_PPRE2 < 0x4) {
        clktree_set_scale(s->PCLK2, 1, 1);
    } else {
        clktree_set_scale(s->PCLK2, 1, 2 * (s->RCC_CFGR_PPRE2 - 3));
    }

    /* PPRE1 */
    s->RCC_CFGR_PPRE1 = (new_value & RCC_CFGR_PPRE1_MASK) >> RCC_CFGR_PPRE1_START;
    if(s->RCC_CFGR_PPRE1 < 4) {
        clktree_set_scale(s->PCLK1, 1, 1);
    } else {
        clktree_set_scale(s->PCLK1, 1, 2 * (s->RCC_CFGR_PPRE1 - 3));
    }

    /* HPRE */
    s->RCC_CFGR_HPRE = (new_value & RCC_CFGR_HPRE_MASK) >> RCC_CFGR_HPRE_START;
    if(s->RCC_CFGR_HPRE < 8) {
        clktree_set_scale(s->HCLK, 1, 1);
    } else {
        clktree_set_scale(s->HCLK, 1, 2 * (s->RCC_CFGR_HPRE - 7));
    }

    /* SW */
    s->RCC_CFGR_SW = (new_value & RCC_CFGR_SW_MASK) >> RCC_CFGR_SW_START;
    switch(s->RCC_CFGR_SW) {
        case 0x0:
        case 0x1:
        case 0x2:
            clktree_set_selected_input(s->SYSCLK, s->RCC_CFGR_SW);
            break;
        default:
            //hw_error("Invalid input selected for SYSCLK");
            break;
    }

    //WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO2_MASK, RCC_CFGR_RESET_VALUE);
    //WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO2PRE_MASK, RCC_CFGR_RESET_VALUE);
    //WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO1PRE_MASK, RCC_CFGR_RESET_VALUE);
    //WARN_UNIMPLEMENTED(new_value, 1 << RCC_CFGR_I2CSRC_BIT, RCC_CFGR_RESET_VALUE);
    //WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO1_MASK, RCC_CFGR_RESET_VALUE);
    //WARN_UNIMPLEMENTED(new_value, RCC_CFGR_RTCPRE_MASK, RCC_CFGR_RESET_VALUE);
    //WARN_UNIMPLEMENTED(new_value, RCC_CFGR_SWS_MASK, RCC_CFGR_RESET_VALUE);
}


static uint32_t stm32_rcc_RCC_AHB1ENR_read(struct STM32u535RccState *s)
{
    return s->RCC_AHB1ENR;
}

static uint32_t stm32_rcc_RCC_AHB2ENR_read(struct STM32u535RccState *s)
{
    return s->RCC_AHB2ENR;
}

static uint32_t stm32_rcc_RCC_AHB3ENR_read(struct STM32u535RccState *s)
{
    return s->RCC_AHB3ENR;
}

static void stm32_rcc_RCC_AHB1ENR_write(struct STM32u535RccState *s, uint32_t new_value, bool init)
{
    /*
    clktree_set_enabled(s->PERIPHCLK[STM32_DMA2], IS_BIT_SET(new_value, RCC_AHB1ENR_DMA2EN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_DMA1], IS_BIT_SET(new_value, RCC_AHB1ENR_DMA1EN_BIT));

    clktree_set_enabled(s->PERIPHCLK[STM32_CRC], IS_BIT_SET(new_value, RCC_AHB1ENR_CRCEN_BIT));

    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOK], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOKEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOJ], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOJEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOI], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOIEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOH], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOHEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOG], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOGEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOF], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOFEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOE], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOEEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOD], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIODEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOC], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOCEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOB], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOBEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOA], IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOAEN_BIT));
    */
    s->RCC_AHB1ENR = new_value;
/*
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_OTGHSULPIEN_BIT, RCC_AHB1ENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_OTGHSEN_BIT, RCC_AHB1ENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_ETHMACPTPEN_BIT, RCC_AHB1ENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_ETHMACRXEN_BIT, RCC_AHB1ENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_ETHMACTXEN_BIT, RCC_AHB1ENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_ETHMACEN_BIT, RCC_AHB1ENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_BKPSRAMEN_BIT, RCC_AHB1ENR_RESET_VALUE);
    */
}

static void stm32_rcc_RCC_AHB2ENR_write(struct STM32u535RccState *s, uint32_t new_value, bool init)
{
    /*
    clktree_set_enabled(s->PERIPHCLK[STM32_DCMI_PERIPH],
                        IS_BIT_SET(new_value, RCC_AHB2ENR_DCMIEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_CRYP_PERIPH],
                        IS_BIT_SET(new_value, RCC_AHB2ENR_CRYPEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_HASH_PERIPH],
                        IS_BIT_SET(new_value, RCC_AHB2ENR_HASHEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_RNG_PERIPH],
                        IS_BIT_SET(new_value, RCC_AHB2ENR_RNGEN_BIT));
    */
    s->RCC_AHB2ENR = new_value;
}

static void stm32_rcc_RCC_AHB3ENR_write(struct STM32u535RccState *s, uint32_t new_value, bool init)
{
    //clktree_set_enabled(s->PERIPHCLK[STM32_FSMC],
    //                    IS_BIT_SET(new_value, RCC_AHB3ENR_FSMCEN_BIT));
    s->RCC_AHB3ENR = new_value;
}

/* Write the APB2 peripheral clock enable register
 * Enables/Disables the peripheral clocks based on each bit. */
static void stm32_rcc_RCC_APB2ENR_write(struct STM32u535RccState *s, uint32_t new_value,
                                        bool init)
{
    /* TODO: enable/disable missing peripherals */
   // stm32_rcc_periph_enable(s, new_value, init, STM32_SYSCFG, RCC_APB2ENR_SYSCFGEN_BIT);
   // stm32_rcc_periph_enable(s, new_value, init, STM32_UART1, RCC_APB2ENR_USART1EN_BIT);
   // stm32_rcc_periph_enable(s, new_value, init, STM32_UART6, RCC_APB2ENR_USART6EN_BIT);

    s->RCC_APB2ENR = new_value; //  & RCC_APB2ENR_MASK;
}

/* Write the APB1 peripheral clock enable register
 * Enables/Disables the peripheral clocks based on each bit. */
static void stm32_rcc_RCC_APB1ENR_write(struct STM32u535RccState *s, uint32_t new_value,
                                        bool init)
{
    //stm32_rcc_periph_enable(s, new_value, init, STM32_UART8,
    //                        RCC_APB1ENR_USART8EN_BIT);
    //stm32_rcc_periph_enable(s, new_value, init, STM32_UART7,
    //                        RCC_APB1ENR_USART7EN_BIT);
    //stm32_rcc_periph_enable(s, new_value, init, STM32_UART5,
    //                        RCC_APB1ENR_USART5EN_BIT);
    //stm32_rcc_periph_enable(s, new_value, init, STM32_UART4,
    //                        RCC_APB1ENR_USART4EN_BIT);
    //stm32_rcc_periph_enable(s, new_value, init, STM32_UART3,
    //                        RCC_APB1ENR_USART3EN_BIT);
    //stm32_rcc_periph_enable(s, new_value, init, STM32_UART2,
    //                        RCC_APB1ENR_USART2EN_BIT);

    /* 0b00110110111111101100100111111111 */
    s->RCC_APB1ENR = new_value & 0x36fec9ff;
}

static void stm32_rcc_RCC_BDCR_write(struct STM32u535RccState *s, uint32_t new_value,
                                        bool init) {

}


static uint32_t stm32_rcc_RCC_BDCR_read(struct STM32u535RccState *s)
{
    bool lseon = clktree_is_enabled(s->LSECLK);

    return GET_BIT_MASK(RCC_BDCR_LSERDY_BIT, lseon) |
    GET_BIT_MASK(RCC_BDCR_LSEON_BIT, lseon) |
    GET_BIT_MASK(RCC_BDCR_LSESYSRDY, lseon) ; /* XXX force LSE */
    return 0;
}

/* Works the same way as stm32_rcc_RCC_CR_read */
static uint32_t stm32_rcc_RCC_CSR_read(struct STM32u535RccState *s)
{
    bool lseon = clktree_is_enabled(s->LSICLK);

    return GET_BIT_MASK(RCC_CSR_LSIRDY_BIT, lseon) |
    GET_BIT_MASK(RCC_CSR_LSION_BIT, lseon);
}

/* Works the same way as stm32_rcc_RCC_CR_write */
static void stm32_rcc_RCC_CSR_write(struct STM32u535RccState *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(s->LSICLK, IS_BIT_SET(new_value, RCC_CSR_LSION_BIT));
}


static uint64_t stm32_rcc_readw(void *opaque, hwaddr offset)
{
    struct STM32u535RccState *s = (struct STM32u535RccState *)opaque;

    switch (offset) {
        case RCC_CR_OFFSET:
            return stm32_rcc_RCC_CR_read(s);
        case RCC_PLL1CFGR_POS:
            return stm32_rcc_RCC_PLLCFGR_read(s);
        case RCC_CFGR1_OFFSET:
            return stm32_rcc_RCC_CFGR_read(s);
        case RCC_CFGR2_OFFSET:
            return s->RCC_CFGR2;
        case RCC_CFGR3_OFFSET:
            return s->RCC_CFGR3;
        case RCC_CIER_OFFSET:
            return s->RCC_CIER;
        //case RCC_CIFR_OFFSET:
        //    return stm32_rcc_RCC_CIR_read(s);
        case RCC_AHB1ENR_OFFSET:
            return stm32_rcc_RCC_AHB1ENR_read(s);
        case RCC_AHB2ENR2_OFFSET:
            return stm32_rcc_RCC_AHB2ENR_read(s);
        case RCC_AHB3ENR_OFFSET:
            return stm32_rcc_RCC_AHB3ENR_read(s);
        case RCC_APB2ENR_OFFSET:
            return s->RCC_APB2ENR;
        case RCC_BDCR_OFFSET:
            return stm32_rcc_RCC_BDCR_read(s);
        case RCC_CSR_OFFSET:
            return stm32_rcc_RCC_CSR_read(s);
        case RCC_CRRCR_OFFSET:
            return  s->RCC_CRRCR | 0x2;  // Force ready

        //case RCC_PLLI2SCFGR_OFFSET:
        //    return stm32_rcc_RCC_PLLI2SCFGR_read(s);
        default:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
    }
    return 0;
}

 void stm32_u5rcc_writew(void *opaque, hwaddr offset,
                             uint64_t value);

 void stm32_u5rcc_writew(void *opaque, hwaddr offset,
                             uint64_t value)
{
    struct STM32u535RccState *s = (struct STM32u535RccState *)opaque;

    switch(offset) {
        case RCC_CR_OFFSET:
            stm32_rcc_RCC_CR_write(s, value, false);
            break;
        case RCC_PLL1CFGR_POS:
            stm32_rcc_RCC_PLLCFGR_write(s, value, false);
            break;
        case RCC_CFGR1_OFFSET:
            stm32_rcc_RCC_CFGR_write(s, value, false);
            break;
        case RCC_CFGR2_OFFSET:
            s->RCC_CFGR2 = value;
            break;
        case RCC_CFGR3_OFFSET:
            s->RCC_CFGR3 = value;
            break;
        case RCC_CIER_OFFSET:
            s->RCC_CIER = value;
            break;
        //case RCC_CIFR_OFFSET:
        //    stm32_rcc_RCC_CIR_write(s, value, false);
        //    break;
        case RCC_AHB1ENR_OFFSET:
            stm32_rcc_RCC_AHB1ENR_write(s, value, false);
            break;
        case RCC_AHB3ENR_OFFSET:
            stm32_rcc_RCC_AHB3ENR_write(s, value, false);
            break;
        case RCC_AHB2ENR2_OFFSET:
            stm32_rcc_RCC_AHB2ENR_write(s, value, false);
            break;
        case RCC_APB2ENR_OFFSET:
            stm32_rcc_RCC_APB2ENR_write(s, value, false);
            break;
        case RCC_APB1ENR1_OFFSET:
            stm32_rcc_RCC_APB1ENR_write(s, value, false);
            break;
        case RCC_BDCR_OFFSET:
            stm32_rcc_RCC_BDCR_write(s, value, false);
            break;
        case RCC_CSR_OFFSET:
            stm32_rcc_RCC_CSR_write(s, value, false);
            break;
        case RCC_CRRCR_OFFSET:
            s->RCC_CRRCR = value;
            break;
        //case RCC_PLLI2SCFGR_OFFSET:
        //    stm32_rcc_RCC_PLLI2SCFGR_write(s, value, false);
        //    break;
        default:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
    }
}

static uint64_t stm32_rcc_read(void *opaque, hwaddr offset,
                               unsigned size)
{
    switch(size) {
        case 4:
            return stm32_rcc_readw(opaque, offset);
        default:
            stm32_unimp("Unimplemented: RCC read from register at offset %lu", offset);
            return 0;
    }
}

static void stm32_rcc_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    switch(size) {
        case 4:
            stm32_u5rcc_writew(opaque, offset, value);
            break;
        case 1: {
            hwaddr woffset = offset & ~0x3;
            uint32_t boffset = offset & 0x3;
            uint32_t val = stm32_rcc_readw(opaque, woffset);
            val &= ~(0xff << boffset);
            val |= (value << boffset);
            stm32_u5rcc_writew(opaque, woffset, value);
        } break;
        default:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
    }
}

static const MemoryRegionOps stm32_rcc_ops = {
    .read = stm32_rcc_read,
    .write = stm32_rcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};


static void stm32_rcc_reset(DeviceState *dev)
{
    struct STM32u535RccState *s = DO_UPCAST(struct STM32u535RccState, busdev, SYS_BUS_DEVICE(dev));

    stm32_rcc_RCC_CR_write(s, RCC_CR_RESET_VALUE, true);
    stm32_rcc_RCC_PLLCFGR_write(s, RCC_PLLCFGR_RESET_VALUE, true);
    stm32_rcc_RCC_APB2ENR_write(s, RCC_APB2ENR_RESET_VALUE, true);
    stm32_rcc_RCC_APB1ENR_write(s, RCC_APB1ENR1_RESET_VALUE, true);
    stm32_rcc_RCC_BDCR_write(s, RCC_BDCR_RESET_VALUE, true);
    stm32_rcc_RCC_CSR_write(s, RCC_CSR_RESET_VALUE, true);
}

/* IRQ handler to handle updates to the HCLK frequency.
 * This updates the SysTick scales. */
static void stm32_rcc_hclk_upd_irq_handler(void *opaque, int n, int level)
{
    struct STM32u535RccState *s = (struct STM32u535RccState *)opaque;

    uint32_t hclk_freq = 0;

    hclk_freq = clktree_get_output_freq(s->HCLK);

    /* Only update the scales if the frequency is not zero. */
    if (hclk_freq > 0) {
        //ext_ref_freq = hclk_freq / 8;

        /* Update the scales - these are the ratio of QEMU clock ticks
         * (which is an unchanging number independent of the CPU frequency) to
         * system/external clock ticks.
         */
        system_clock_scale = NANOSECONDS_PER_SECOND / hclk_freq;
    }

#ifdef DEBUG_STM32_RCC
    DPRINTF("Cortex SYSTICK frequency set to %lu Hz (scale set to %d).\n",
            (unsigned long)hclk_freq, system_clock_scale);
#endif
}



/* DEVICE INITIALIZATION */

/* Set up the clock tree */
static void stm32_rcc_init_clk(struct STM32u535RccState *s)
{
    int i;
    qemu_irq *hclk_upd_irq =
    qemu_allocate_irqs(stm32_rcc_hclk_upd_irq_handler, s, 1);

    /* Make sure all the peripheral clocks are null initially.
     * This will be used for error checking to make sure
     * an invalid clock is not referenced (not all of the
     * indexes will be used).
     */
    for(i = 0; i < STM32_PERIPH_COUNT; i++) {
        s->PERIPHCLK[i] = NULL;
    }

    /* Initialize clocks */
    /* Source clocks are initially disabled, which represents
     * a disabled oscillator.  Enabling the clock represents
     * turning the clock on.
     */
    s->HSICLK = clktree_create_src_clk("HSI", HSI_FREQ, false);
    s->LSICLK = clktree_create_src_clk("LSI", LSI_FREQ, false);
    s->MSICLK = clktree_create_src_clk("MSI", s->osc_freq, false);

    s->HSECLK = clktree_create_src_clk("HSE", s->osc_freq, false);
    s->LSECLK = clktree_create_src_clk("LSE", s->osc32_freq, false);

    s->CSCLK = clktree_create_src_clk("CSCLK", HSI_FREQ, false);


    s->IWDGCLK = clktree_create_clk("IWDGCLK", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0,
                                    s->LSICLK, NULL);
    s->RTCCLK = clktree_create_clk("RTCCLK", 1, 1, false, CLKTREE_NO_MAX_FREQ,
                                   CLKTREE_NO_INPUT, s->LSECLK, s->LSICLK, s->HSECLK, NULL);

    s->PLLM = clktree_create_clk("PLLM", 1, 16, true, CLKTREE_NO_MAX_FREQ, 0, s->HSICLK,
                                 s->HSECLK, NULL);

    s->PLL2M = clktree_create_clk("PLL2M", 1, 16, true, CLKTREE_NO_MAX_FREQ, 0, s->HSICLK,
                                 s->HSECLK, NULL);

    s->PLL3M = clktree_create_clk("PLL3M", 1, 16, true, CLKTREE_NO_MAX_FREQ, 0, s->HSICLK,
                                 s->HSECLK, NULL);


    s->PLL1CLK = clktree_create_clk("PLL1CLK", 1, 2, false, 120000000, 0, s->PLLM, NULL);
    s->PLL48CLK = clktree_create_clk("PLL48CLK", 1, 1, false, 48000000, 0, s->PLLM, NULL);

    s->PLLI2SM = clktree_create_clk("PLLI2SM", 1, 16, true, CLKTREE_NO_MAX_FREQ, 0, s->PLLM, NULL);
    s->PLLI2SCLK = clktree_create_clk("PLLI2SCLK", 1, 2, false, 120000000, 0, s->PLLI2SM, NULL);

    s->PLL2CLK = clktree_create_clk("PLL2CLK", 1, 2, false, 120000000, 0, s->PLL2M, NULL);
    s->PLL3CLK = clktree_create_clk("PLL3CLK", 1, 2, false, 120000000, 0, s->PLL3M, NULL);


    s->SYSCLK = clktree_create_clk("SYSCLK", 1, 1, true, 160000000, CLKTREE_NO_INPUT,
                                   s->HSICLK, s->HSECLK, s->PLL1CLK, NULL);

    // HCLK: to AHB bus, core memory and DMA
    s->HCLK = clktree_create_clk("HCLK", 0, 1, true, 168000000, 0, s->SYSCLK, NULL);
    clktree_adduser(s->HCLK, hclk_upd_irq[0]);

    // Clock source for APB1 peripherals:
    s->PCLK1 = clktree_create_clk("PCLK1", 0, 1, true, 30000000, 0, s->HCLK, NULL);

    // Clock source for APB2 peripherals:
    s->PCLK2 = clktree_create_clk("PCLK2", 0, 1, true, 60000000, 0, s->HCLK, NULL);

    /* Peripheral clocks */
    s->PERIPHCLK[STM32_GPIOA] =
        clktree_create_clk("GPIOA", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOB] =
        clktree_create_clk("GPIOB", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOC] =
        clktree_create_clk("GPIOC", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOD] =
        clktree_create_clk("GPIOD", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOE] =
        clktree_create_clk("GPIOE", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOF] =
        clktree_create_clk("GPIOF", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOG] =
        clktree_create_clk("GPIOG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOH] =
        clktree_create_clk("GPIOH", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOI] =
        clktree_create_clk("GPIOI", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOJ] =
        clktree_create_clk("GPIOJ", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOK] =
        clktree_create_clk("GPIOK", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);

    s->PERIPHCLK[STM32_CRC] =
        clktree_create_clk("CRC", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);

    s->PERIPHCLK[STM32_DMA1] =
        clktree_create_clk("DMA1", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_DMA2] =
        clktree_create_clk("DMA2", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    
    s->PERIPHCLK[STM32_SYSCFG] =
        clktree_create_clk("SYSCFG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);

    s->PERIPHCLK[STM32_UART1] =
        clktree_create_clk("UART1", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32_UART2] =
        clktree_create_clk("UART2", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART3] =
        clktree_create_clk("UART3", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART4] =
        clktree_create_clk("UART4", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART5] =
        clktree_create_clk("UART5", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART6] =
        clktree_create_clk("UART6", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32_UART7] =
        clktree_create_clk("UART7", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART8] =
        clktree_create_clk("UART8", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);


    s->PERIPHCLK[STM32_DCMI_PERIPH] =
        clktree_create_clk("DCMI", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_CRYP_PERIPH] =
        clktree_create_clk("CRYP", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_HASH_PERIPH] =
        clktree_create_clk("HASH", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_RNG_PERIPH] =
        clktree_create_clk("RNG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);

    s->PERIPHCLK[STM32_FSMC] =
        clktree_create_clk("FSMC", 1, 2, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
}



static void stm32l552_rcc_realize(DeviceState *dev, Error **errp) {
    struct STM32u535RccState *s = OBJECT_CHECK(struct STM32u535RccState, dev, "stm32u535-rcc");
    SysBusDevice *busdev = SYS_BUS_DEVICE(dev);
    memory_region_init_io(&s->iomem, OBJECT(s), &stm32_rcc_ops, s,
                          "my_rcc", 0x400);

    sysbus_init_mmio(busdev, &s->iomem);

    sysbus_init_irq(busdev, &s->irq);

    stm32_rcc_init_clk(s);
}


static Property stm32_rcc_properties[] = {
    DEFINE_PROP_UINT32("osc_freq", struct STM32u535RccState, osc_freq, 0),
    DEFINE_PROP_UINT32("osc32_freq", struct STM32u535RccState, osc32_freq, 0),
    DEFINE_PROP_END_OF_LIST()
};

// TypeInfo


static void stm32_rcc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32_rcc_reset;
    device_class_set_props(dc,stm32_rcc_properties);
    dc->realize = stm32l552_rcc_realize;
}

static TypeInfo stm32_rcc_info = {
    .name  = "stm32u535-rcc",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(struct STM32u535RccState),
    .class_init = stm32_rcc_class_init
};

static void stm32_rcc_register_types(void)
{
    type_register_static(&stm32_rcc_info);
}

type_init(stm32_rcc_register_types)
