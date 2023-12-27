#ifndef HW_STM32U535_PWR_H
#define HW_STM32U535_PWR_H

// #include "hw/arm/stm32l552.h"



#define PWR_CR1  0x00
#define PWR_CR2  0x04
#define PWR_CR3  0x08
#define PWR_CR4 0x0C
#define PWR_SR1 0x10
#define PWR_SR2 0x14
#define PWR_SCR 0x18
#define PWR_PUCRA 0x20
#define PWR_PDCRA 0x24
#define PWR_PUCRB 0x28
#define PWR_PDCRB 0x2C
#define PWR_PUCRC 0x30
#define PWR_PDCRC 0x34
#define PWR_PUCRD 0x38
#define PWR_PDCRD 0x3C
#define PWR_PUCRE 0x40
#define PWR_PDCRE 0x44
#define PWR_PUCRF 0x48
#define PWR_PDCRF 0x4C
#define PWR_PUCRG 0x50
#define PWR_PDCRG 0x54
#define PWR_PUCRH 0x58
#define PWR_PDCRH 0x5C
#define PWR_SECCFGR 0x78
#define PWR_PRIVCFGR 0x80








#define  PWR_CR_REG 0x00 

#define PWR_CR_UDEN     (3 << 18)
#define PWR_CR_ODSWEN   (1 << 17)
#define PWR_CR_ODEN     (1 << 16)
#define PWR_CR_VOS      (3 << 14)
#define PWR_CR_ADCDC1   (1 << 13)
#define PWR_CR_MRUDS    (1 << 11)
#define PWR_CR_LPUDS    (1 << 10)
#define PWR_CR_FPDS     (1 << 9)
#define PWR_CR_DBP      (1 << 8)
#define PWR_CR_PLS      (7 << 5)
#define PWR_CR_PVDE     (1 << 4)
#define PWR_CR_CSBF     (1 << 3)
#define PWR_CR_CWUF     (1 << 2)
#define PWR_CR_PDDS     (1 << 1)
#define PWR_CR_LPDS     (1 << 0)

#define PWR_CSR_UDRDY   (3 << 18)
#define PWR_CSR_ODSWRDY (1 << 17)
#define PWR_CSR_ODRDY   (1 << 16)
#define PWR_CSR_VOSRDY  (1 << 14)
#define PWR_CSR_BRE     (1 << 9)
#define PWR_CSR_EWUP    (1 << 8)
#define PWR_CSR_BRR     (1 << 3)
#define PWR_CSR_PVDO    (1 << 2)
#define PWR_CSR_SBF     (1 << 1)
#define PWR_CSR_WUF     (1 << 0)




#define TYPE_STM32U535_PWR "stm32u535-pwr"
OBJECT_DECLARE_SIMPLE_TYPE(stm32u535_pwr, STM32U535_PWR)


struct stm32u535_pwr {
    SysBusDevice parent;
 
    uint32_t pwr_cr1;
    uint32_t pwr_cr2;
    uint32_t pwr_cr3;
    uint32_t pwr_cr4;
    uint32_t pwr_sr1;
    uint32_t pwr_sr2;
    uint32_t pwr_scr;
    uint32_t pwr_pucra;
    uint32_t pwr_pdcra;
    uint32_t pwr_pucrb;
    uint32_t pwr_pdcrb;
    uint32_t pwr_pucrc;
    uint32_t pwr_pdcrc;
    uint32_t pwr_pucrd;
    uint32_t pwr_pdcrd;
    uint32_t pwr_pucre;
    uint32_t pwr_pdcre;
    uint32_t pwr_pucrf;
    uint32_t pwr_pdcrf;
    uint32_t pwr_pucrg;
    uint32_t pwr_pdcrg;
    uint32_t pwr_pucrh;
    uint32_t pwr_pdcrh;
    uint32_t pwr_seccfgr;
    uint32_t pwr_privcfgr;



    MemoryRegion mmio;
    qemu_irq irq;

};

#endif
