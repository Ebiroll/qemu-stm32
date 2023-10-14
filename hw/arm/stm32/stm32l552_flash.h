#ifndef HW_STM32L552_FLASH_H
#define HW_STM32L552_FLASH_H

#include "qemu/osdep.h"
#include "hw/block/block.h"
#include "hw/block/flash.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "sysemu/block-backend.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/option.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "sysemu/blockdev.h"
#include "sysemu/runstate.h"
#include "trace.h"



/* FLASH */

typedef struct Stm32Flash Stm32Flash;

#define TYPE_STM32L552_FLASH "stm32l552-flash"
#define STM32L552_FLASH(obj) OBJECT_CHECK(Stm32Flash, (obj), TYPE_STM32L552_FLASH)

Stm32Flash *stm32_flash_register(BlockBackend *blk, hwaddr base, hwaddr size);

void stm32_legacy_drive(Stm32Flash *fl, DriveInfo *dinfo);

typedef struct Stm32Flash {
	SysBusDevice busdev;   
    //BlockDriverState * blks;
    hwaddr base_address;
	uint32_t size;
    //MemoryRegion iomem;
    BlockBackend *blk;
    uint32_t nb_blocs;
    uint64_t sector_len;
    uint8_t bank_width;
    uint8_t device_width; /* If 0, device width not specified. */
    uint8_t max_device_width;  /* max device width in bytes */
    uint32_t features;
    uint8_t wcycle; /* if 0, the flash is read normally */
    bool ro;
    ///
    char *name;
    void *storage;
    VMChangeStateEntry *vmstate;
    uint64_t counter;

    ////
    MemoryRegion mem;
    void *data;
    hwaddr SP_init;
    hwaddr PC_init;    
} Stm32Flash;




/* Flash Regs */



//#define TYPE_STM32L552_PWR "stm32l552-pwr"
//OBJECT_DECLARE_SIMPLE_TYPE(stm32l552_pwr, STM32L552_PWR)

#define TYPE_STM32_FLASH_REGS "stm32-flash-regs"
// #define STM32_FLASH_REGS(obj) OBJECT_CHECK(Stm32FlashRegs, (obj), TYPE_STM32_FLASH_REGS)
OBJECT_DECLARE_SIMPLE_TYPE(Stm32FlashRegs, STM32_FLASH_REGS)

struct Stm32FlashRegs {
	SysBusDevice busdev;
	MemoryRegion iomem;

	uint32_t ACR;
	uint32_t KEYR;
	uint32_t OPTKEYR;
	uint32_t SR;
	uint32_t CR;
	uint32_t AR;
	uint32_t RESERVED;
	uint32_t OBR;
	uint32_t WRPR;
} ;




#if 0
typedef struct f2xx_flash {
    SysBusDevice busdev;
    BlockBackend *blk;
    hwaddr base_address;
    uint32_t size;

    MemoryRegion mem;
    void *data;
} f2xx_flash_t;


f2xx_flash_t *f2xx_flash_register(BlockBackend *blk, hwaddr base,
                                  hwaddr size);
#endif
#endif
