#pragma once 
#include "hw/arm/stm32fxxx.h"


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


#define TYPE_STM32FXXX_PWR "stm32fxxx-pwr"
OBJECT_DECLARE_SIMPLE_TYPE(stm32fxxx_pwr, STM32FXXX_PWR)


struct stm32fxxx_pwr {
    SysBusDevice parent;

    MemoryRegion mmio;
    qemu_irq irq;

    struct stm32fxxx_state state;
};

#if 0
struct STM32PowerMgtState {
    SysBusDevice busdev;
    MemoryRegion iomem;

    uint32_t cfg;
    uint32_t cfg2;
    uint32_t csr;


    uint32_t rpcsr;
    uint32_t hsem_lock;

};
#endif