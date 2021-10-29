#include "qemu/osdep.h"
#include "hw/sh4/sh.h"
#include "sh7750_regs.h"
#include "sh7750_regnames.h"

#define REGNAME(r) {r, #r},

typedef struct {
    uint32_t regaddr;
    const char *regname;
} regname_t;

static regname_t regnames[] = {
    REGNAME(SH7750_PTEH_A7)
    REGNAME(SH7750_PTEL_A7)
    REGNAME(SH7750_PTEA_A7)
    REGNAME(SH7750_TTB_A7)
    REGNAME(SH7750_TEA_A7)
    REGNAME(SH7750_MMUCR_A7)
    REGNAME(SH7750_CCR_A7)
    REGNAME(SH7750_QACR0_A7)
    REGNAME(SH7750_QACR1_A7)
    REGNAME(SH7750_TRA_A7)
    REGNAME(SH7750_EXPEVT_A7)
    REGNAME(SH7750_INTEVT_A7)
    REGNAME(SH7750_STBCR_A7)
    REGNAME(SH7750_STBCR2_A7)
    REGNAME(SH7750_FRQCR_A7)
    REGNAME(SH7750_WTCNT_A7)
    REGNAME(SH7750_WTCSR_A7)
    REGNAME(SH7750_R64CNT_A7)
    REGNAME(SH7750_RSECCNT_A7)
    REGNAME(SH7750_RMINCNT_A7)
    REGNAME(SH7750_RHRCNT_A7)
    REGNAME(SH7750_RWKCNT_A7)
    REGNAME(SH7750_RDAYCNT_A7)
    REGNAME(SH7750_RMONCNT_A7)
    REGNAME(SH7750_RYRCNT_A7)
    REGNAME(SH7750_RSECAR_A7)
    REGNAME(SH7750_RMINAR_A7)
    REGNAME(SH7750_RHRAR_A7)
    REGNAME(SH7750_RWKAR_A7)
    REGNAME(SH7750_RDAYAR_A7)
    REGNAME(SH7750_RMONAR_A7)
    REGNAME(SH7750_RCR1_A7)
    REGNAME(SH7750_RCR2_A7)
    REGNAME(SH7750_BCR1_A7)
    REGNAME(SH7750_BCR2_A7)
    REGNAME(SH7750_WCR1_A7)
    REGNAME(SH7750_WCR2_A7)
    REGNAME(SH7750_WCR3_A7)
    REGNAME(SH7750_MCR_A7)
    REGNAME(SH7750_PCR_A7)
    REGNAME(SH7750_RTCSR_A7)
    REGNAME(SH7750_RTCNT_A7)
    REGNAME(SH7750_RTCOR_A7)
    REGNAME(SH7750_RFCR_A7)
    REGNAME(SH7750_SAR0_A7)
    REGNAME(SH7750_SAR1_A7)
    REGNAME(SH7750_SAR2_A7)
    REGNAME(SH7750_SAR3_A7)
    REGNAME(SH7750_DAR0_A7)
    REGNAME(SH7750_DAR1_A7)
    REGNAME(SH7750_DAR2_A7)
    REGNAME(SH7750_DAR3_A7)
    REGNAME(SH7750_DMATCR0_A7)
    REGNAME(SH7750_DMATCR1_A7)
    REGNAME(SH7750_DMATCR2_A7)
    REGNAME(SH7750_DMATCR3_A7)
    REGNAME(SH7750_CHCR0_A7)
    REGNAME(SH7750_CHCR1_A7)
    REGNAME(SH7750_CHCR2_A7)
    REGNAME(SH7750_CHCR3_A7)
    REGNAME(SH7750_DMAOR_A7)
    REGNAME(SH7750_PCTRA_A7)
    REGNAME(SH7750_PDTRA_A7)
    REGNAME(SH7750_PCTRB_A7)
    REGNAME(SH7750_PDTRB_A7)
    REGNAME(SH7750_GPIOIC_A7)
    REGNAME(SH7750_ICR_A7)
    REGNAME(SH7750_BCR3_A7)
    REGNAME(SH7750_BCR4_A7)
    REGNAME(SH7750_SDMR2_A7)
    REGNAME(SH7750_SDMR3_A7)
    { (uint32_t)-1, NULL }
};

const char *regname(uint32_t addr)
{
    unsigned int i;

    for (i = 0; regnames[i].regaddr != (uint32_t)-1; i++) {
        if (regnames[i].regaddr == addr)
            return regnames[i].regname;
    }

    return "<unknown reg>";
}
