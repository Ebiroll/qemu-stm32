/*
 * STM32 Microcontroller Flash Memory
 * The STM32 family stores its Flash memory at some base address in memory
 * (0x08000000 for medium density devices), and then aliases it to the
 * boot memory space, which starts at 0x00000000 (the System Memory can also
 * be aliased to 0x00000000, but this is not implemented here).  The processor
 * executes the code in the aliased memory at 0x00000000, but we need to
 * implement the "real" flash memory as well.  This "real" flash memory will
 * pass reads through to the memory at 0x00000000, which is where QEMU loads
 * the executable image.  Note that this is opposite of real hardware, where the
 * memory at 0x00000000 passes reads through the "real" flash memory, but it
 * works the same either way.
 *
 * Copyright (C) 2023 Olof Astrand
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

#include "hw/arm/stm32/stm32l552_flash.h"
#include "trace/trace-hw_block.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/hw.h"

#define PFLASH_BE          0
#define PFLASH_SECURE      1

#define STM32_FLASH_ADDR_START 0x08000000

#define DEBUG_FLASH 1

static Stm32Flash* flash;
static uint32_t is_flash_locked = 1;
static uint32_t flash_programming_bit = 0;

#define R_FLASH_ACR              (0x00 / 4)
#define R_FLASH_PDKEYR           (0x04 / 4)
#define R_FLASH_NSKEYR           (0x08 / 4)
#define R_FLASH_SECKEYR          (0x0C / 4)
#define R_FLASH_OPTKEYR          (0x10 / 4)
#define R_FLASH_LVEKEYR          (0x14 / 4)
#define R_FLASH_NSSR             (0x20 / 4)
#define R_FLASH_SECSR            (0x24 / 4)
#define R_FLASH_NCSR             (0x28 / 4)
#define R_FLASH_SECCR            (0x2C / 4)
#define R_FLASH_ECCR             (0x30 / 4)
#define R_FLASH_OPT              (0x40 / 4)
#define R_FLASH_NSBOOTADD0R      (0x44 / 4)
#define R_FLASH_NSBOOTADD1R      (0x48 / 4)
#define R_FLASH_SECBOOTADD0R     (0x4C / 4)
#define R_FLASH_SECWM1R1         (0x50 / 4)
#define R_FLASH_SECWM1R2         (0x54 / 4)
#define R_FLASH_WRP1AR           (0x58 / 4)
#define R_FLASH_WRP1BR           (0x5C / 4)
#define R_FLASH_SECWM2R1         (0x60 / 4)
#define R_FLASH_SECWM2R2         (0x64 / 4)
#define R_FLASH_WRP2AR           (0x68 / 4)
#define R_FLASH_WRP2BR           (0x6C / 4)
#define R_FLASH_SECBB1R1         (0x80 / 4)
#define R_FLASH_SECBB1R2         (0x84 / 4)
#define R_FLASH_SECBB1R3         (0x88 / 4)
#define R_FLASH_SECBB1R4         (0x8C / 4)
#define R_FLASH_SECBB2R1         (0xA0 / 4)
#define R_FLASH_SECBB2R2         (0xA4 / 4)
#define R_FLASH_SECBB2R3         (0xA8 / 4)
#define R_FLASH_SECBB2R4         (0xAC / 4)
#define R_FLASH_SECHDPCR         (0xC0 / 4)
#define R_FLASH_PRIVCFGR         (0xC4 / 4)


// Old regs
#define R_FLASH_SR             R_FLASH_NSSR
#define R_FLASH_CR             R_FLASH_PDKEYR
#define R_FLASH_AR             R_FLASH_WRP1AR
#define R_FLASH_KEYR           R_FLASH_LVEKEYR
#define R_FLASH_RESERVED       (0x18 / 4)
#define R_FLASH_OBR            R_FLASH_OPT
#define R_FLASH_WRPR           R_FLASH_WRP1BR
#define R_FLASH_MAX            (0xC4 / 4)


/* */
Stm32Flash *stm32_flash_register(BlockBackend *blk, hwaddr base,
                                  hwaddr size)
{
    DeviceState *dev = qdev_new(TYPE_STM32L552_FLASH);
    //Stm32Flash *flash = (Stm32Flash *)object_dynamic_cast(OBJECT(dev),"stm32-flash");
    qdev_prop_set_uint32(dev, "size", size);
    qdev_prop_set_uint64(dev, "base_address", base);
    if (blk) {
    	//int err = 0;
        qdev_prop_set_drive(dev, "drive", blk);
        //if (err) {
        //    printf("%s, have no drive???\n", __func__);
        //    return NULL;
        //}
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
  
    return STM32L552_FLASH(dev);
}

/*
 * Handle -drive if=pflash for machines that use properties.
 * If @dinfo is null, do nothing.
 * Else if @fl's property "drive" is already set, fatal error.
 * Else set it to the BlockBackend with @dinfo.
 */
void stm32_legacy_drive(Stm32Flash *fl, DriveInfo *dinfo)
{
    Location loc;

    if (!dinfo) {
        return;
    }

    loc_push_none(&loc);
    qemu_opts_loc_restore(dinfo->opts);
    if (fl->blk) {
        error_report("clashes with -machine");
        exit(1);
    }
    qdev_prop_set_drive_err(DEVICE(fl), "drive", blk_by_legacy_dinfo(dinfo),
                            &error_fatal);
    loc_pop(&loc);
}



MemoryRegion *get_system_memory(void); /* XXX */
/*
static int stm32_flash_init(SysBusDevice *dev)
{    
    flash = DO_UPCAST(Stm32Flash, busdev, dev);

//    memory_region_init_rom_device(&flash->mem, &f2xx_flash_ops, flash, "name",
//      size);
    memory_region_init_ram(&flash->iomem, NULL, "f2xx.flash", flash->size);

    vmstate_register_ram(&flash->iomem, DEVICE(flash));
    //vmstate_register_ram_global(&flash->iomem);
    memory_region_set_readonly(&flash->iomem, true);
    memory_region_add_subregion(get_system_memory(), flash->base_address, &flash->iomem);


    flash->data = memory_region_get_ram_ptr(&flash->iomem);
    memset(flash->data, 0xff, flash->size);
    if (flash->blks) {
        int r;
        r = bdrv_read(flash->blks, 0, flash->data, bdrv_getlength(flash->blks)/BDRV_SECTOR_SIZE);
        if (r < 0) {
            vmstate_unregister_ram(&flash->iomem, DEVICE(flash));
            // memory_region_destroy(&flash->mem);
            return 1;
        }
        else{
          uint32_t * mem = flash->data; 
          flash->PC_init = mem[1]; 
          flash->SP_init = mem[0];   
        }
    }

    return 0;
}
*/

static MemTxResult pflash_mem_read_with_attrs(void *opaque, hwaddr addr, uint64_t *value,
                                              unsigned len, MemTxAttrs attrs)
{
    Stm32Flash *pfl = opaque;
    //bool be = !!(pfl->features & (1 << PFLASH_BE));

    if ((pfl->features & (1 << PFLASH_SECURE)) && !attrs.secure) {
//        *value = pflash_data_read(opaque, addr, len, be);
    } else {
//        *value = pflash_read(opaque, addr, len, be);
    }
    return MEMTX_OK;
}

static MemTxResult pflash_mem_write_with_attrs(void *opaque, hwaddr addr, uint64_t value,
                                               unsigned len, MemTxAttrs attrs)
{
    Stm32Flash *pfl = opaque;
    //bool be = !!(pfl->features & (1 << PFLASH_BE));

    if ((pfl->features & (1 << PFLASH_SECURE)) && !attrs.secure) {
        return MEMTX_ERROR;
    } else {
//        pflash_write(opaque, addr, value, len, be);
        return MEMTX_OK;
    }
}



static const MemoryRegionOps pflash_cfi01_ops = {
    .read_with_attrs = pflash_mem_read_with_attrs,
    .write_with_attrs = pflash_mem_write_with_attrs,
    .endianness = DEVICE_NATIVE_ENDIAN,
};




static void stm32_flash_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    Stm32Flash *stm32fl = STM32L552_FLASH(dev);
    uint64_t total_len;
    int ret;

    if (stm32fl->name == NULL) {
        error_setg(errp, "attribute \"name\" not specified.");
        return;
    }

    //total_len = stm32fl->sector_len * stm32fl->nb_blocs;
    total_len = 8192 * 625;

    memory_region_init_rom_device(
        &stm32fl->mem, OBJECT(dev),
        &pflash_cfi01_ops,
        stm32fl,
        stm32fl->name, total_len, errp);
    if (*errp) {
        return;
    }

    stm32fl->storage = memory_region_get_ram_ptr(&stm32fl->mem);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &stm32fl->mem);

    if (stm32fl->blk) {
        uint64_t perm;
        stm32fl->ro = !blk_supports_write_perm(stm32fl->blk);
        perm = BLK_PERM_CONSISTENT_READ | (stm32fl->ro ? 0 : BLK_PERM_WRITE);
        ret = blk_set_perm(stm32fl->blk, perm, BLK_PERM_ALL, errp);
        if (ret < 0) {
            return;
        }
    } else {
        stm32fl->ro = false;
    }

    if (stm32fl->blk) {
        // TODO!!! Fix this
        //if (!blk_check_size_and_read_all(stm32fl->blk, stm32fl->storage, (uint64_t) total_len,
        //                                 errp)) {
        //    vmstate_unregister_ram(&stm32fl->mem, DEVICE(stm32fl));
        //    return;
        //}
    }

    /*
     * Default to devices being used at their maximum device width. This was
     * assumed before the device_width support was added.
     */
    if (!stm32fl->max_device_width) {
        stm32fl->max_device_width = stm32fl->device_width;
    }

    stm32fl->wcycle = 0;
    /*
     * The command 0x00 is not assigned by the CFI open standard,
     * but QEMU historically uses it for the READ_ARRAY command (0xff).
     */
    //stm32fl->cmd = 0x00;
    //stm32fl->status = 0x80; /* WSM ready */
    //stm32flash_cfi01_fill_cfi_table(stm32fl);
}
static int pflash_post_load(void *opaque, int version_id);

/* update flash content on disk */
static void pflash_update(Stm32Flash *pfl, int offset,
                          int size)
{
    int offset_end;
    int ret;
    if (pfl->blk) {
        offset_end = offset + size;
        /* widen to sector boundaries */
        offset = QEMU_ALIGN_DOWN(offset, BDRV_SECTOR_SIZE);
        offset_end = QEMU_ALIGN_UP(offset_end, BDRV_SECTOR_SIZE);
        ret = blk_pwrite(pfl->blk, offset, offset_end - offset,
                         pfl->storage + offset, 0);
        if (ret < 0) {
            /* TODO set error bit in status */
            error_report("Could not update PFLASH: %s", strerror(-ret));
        }
    }
}



static void postload_update_cb(void *opaque, bool running, RunState state)
{
    Stm32Flash *pfl = opaque;

    /* This is called after bdrv_activate_all.  */
    qemu_del_vm_change_state_handler(pfl->vmstate);
    pfl->vmstate = NULL;

    trace_pflash_postload_cb(pfl->name);
    pflash_update(pfl, 0, pfl->sector_len * pfl->nb_blocs);
}


static int pflash_post_load(void *opaque, int version_id)
{
    Stm32Flash *pfl = opaque;

    if (!pfl->ro) {
        pfl->vmstate = qemu_add_vm_change_state_handler(postload_update_cb,
                                                        pfl);
    }
    return 0;
}



static const VMStateDescription vmstate_pflash = {
    .name = "stm32l552-flash",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = pflash_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(wcycle, Stm32Flash),
        VMSTATE_UINT64(counter, Stm32Flash),
        VMSTATE_END_OF_LIST()
    }
};



static void
stm32_flash_reset(DeviceState *ds)
{
	// Stm32Flash *s = STM32L552_FLASH(ds);

	printf("flash reset is called!\n\n");
   
    //ARM M3 FLASH reset
    /*
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    CPUARMState *env = &cpu->env; 
   
    env->regs[13] = s->SP_init & 0xFFFFFFFC;
    env->thumb = s->PC_init & 1;
    env->regs[15] = s->PC_init & ~1;
    */
}

//     DEFINE_PROP_DRIVE("drive", Stm32Flash, blk),

static Property stm32_flash_properties[] = {
    DEFINE_PROP_UINT32("size", Stm32Flash, size, 0),
    DEFINE_PROP_UINT64("base_address", Stm32Flash , base_address, STM32_FLASH_ADDR_START),
    DEFINE_PROP_DRIVE("drive", Stm32Flash, blk),
    /* num-blocks is the number of blocks actually visible to the guest,
     * ie the total size of the device divided by the sector length.
     * If we're emulating flash devices wired in parallel the actual
     * number of blocks per individual device will differ.
     */
    DEFINE_PROP_UINT32("num-blocks", Stm32Flash, nb_blocs, 0),
    DEFINE_PROP_UINT64("sector-length", Stm32Flash, sector_len, 0),
    /* width here is the overall width of this QEMU device in bytes.
     * The QEMU device may be emulating a number of flash devices
     * wired up in parallel; the width of each individual flash
     * device should be specified via device-width. If the individual
     * devices have a maximum width which is greater than the width
     * they are being used for, this maximum width should be set via
     * max-device-width (which otherwise defaults to device-width).
     * So for instance a 32-bit wide QEMU flash device made from four
     * 16-bit flash devices used in 8-bit wide mode would be configured
     * with width = 4, device-width = 1, max-device-width = 2.
     *
     * If device-width is not specified we default to backwards
     * compatible behaviour which is a bad emulation of two
     * 16 bit devices making up a 32 bit wide QEMU device. This
     * is deprecated for new uses of this device.
     */
    DEFINE_PROP_UINT8("width", Stm32Flash, bank_width, 0),
    DEFINE_PROP_UINT8("device-width", Stm32Flash, device_width, 0),
    DEFINE_PROP_UINT8("max-device-width", Stm32Flash, max_device_width, 0),

    DEFINE_PROP_END_OF_LIST(),
};


static void stm32_flash_reg_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	//SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_pflash;
	//k->init = stm32_flash_init;
    dc->realize = stm32_flash_realize;
	//dc->props = stm32_flash_properties;
    device_class_set_props(dc, stm32_flash_properties);
    dc->reset = stm32_flash_reset;
}

static TypeInfo stm32_flash_info = {
	.name          = "stm32-flash",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(Stm32Flash),
	.class_init    = stm32_flash_reg_class_init,
};


static void stm32_flash_register_types(void)
{
	type_register_static(&stm32_flash_info);
}

type_init(stm32_flash_register_types);


//////regs






///
#define FLASH_CR_PG_Pos                     (0U)                               
#define FLASH_CR_PG_Msk                     (0x1U << FLASH_CR_PG_Pos)          /*!< 0x00000001 */
#define FLASH_CR_PG                         FLASH_CR_PG_Msk                    /*!< Programming */
#define FLASH_CR_PER_Pos                    (1U)                               
#define FLASH_CR_PER_Msk                    (0x1U << FLASH_CR_PER_Pos)         /*!< 0x00000002 */
#define FLASH_CR_PER                        FLASH_CR_PER_Msk                   /*!< Page Erase */
#define FLASH_CR_MER_Pos                    (2U)                               
#define FLASH_CR_MER_Msk                    (0x1U << FLASH_CR_MER_Pos)         /*!< 0x00000004 */
#define FLASH_CR_MER                        FLASH_CR_MER_Msk                   /*!< Mass Erase */
#define FLASH_CR_OPTPG_Pos                  (4U)                               
#define FLASH_CR_OPTPG_Msk                  (0x1U << FLASH_CR_OPTPG_Pos)       /*!< 0x00000010 */
#define FLASH_CR_OPTPG                      FLASH_CR_OPTPG_Msk                 /*!< Option Byte Programming */
#define FLASH_CR_OPTER_Pos                  (5U)                               
#define FLASH_CR_OPTER_Msk                  (0x1U << FLASH_CR_OPTER_Pos)       /*!< 0x00000020 */
#define FLASH_CR_OPTER                      FLASH_CR_OPTER_Msk                 /*!< Option Byte Erase */
#define FLASH_CR_STRT_Pos                   (6U)                               
#define FLASH_CR_STRT_Msk                   (0x1U << FLASH_CR_STRT_Pos)        /*!< 0x00000040 */
#define FLASH_CR_STRT                       FLASH_CR_STRT_Msk                  /*!< Start */
#define FLASH_CR_LOCK_Pos                   (7U)                               
#define FLASH_CR_LOCK_Msk                   (0x1U << FLASH_CR_LOCK_Pos)        /*!< 0x00000080 */
#define FLASH_CR_LOCK                       FLASH_CR_LOCK_Msk                  /*!< Lock */
#define FLASH_CR_OPTWRE_Pos                 (9U)                               
#define FLASH_CR_OPTWRE_Msk                 (0x1U << FLASH_CR_OPTWRE_Pos)      /*!< 0x00000200 */
#define FLASH_CR_OPTWRE                     FLASH_CR_OPTWRE_Msk                /*!< Option Bytes Write Enable */
#define FLASH_CR_ERRIE_Pos                  (10U)                              
#define FLASH_CR_ERRIE_Msk                  (0x1U << FLASH_CR_ERRIE_Pos)       /*!< 0x00000400 */
#define FLASH_CR_ERRIE                      FLASH_CR_ERRIE_Msk                 /*!< Error Interrupt Enable */
#define FLASH_CR_EOPIE_Pos                  (12U)                              
#define FLASH_CR_EOPIE_Msk                  (0x1U << FLASH_CR_EOPIE_Pos)       /*!< 0x00001000 */
#define FLASH_CR_EOPIE                      FLASH_CR_EOPIE_Msk                 /*!< End of operation interrupt enable */


//#define FLASH_KEY1   0x45670123U /*!< Flash key1 */
//#define FLASH_KEY2   0xCDEF89ABU /*!< Flash key2: used with FLASH_KEY1


#define FLASH_KEY1                          0x45670123U                     /*!< FPEC Key1 */
#define FLASH_KEY2                          0xCDEF89ABU                     /*!< FPEC Key2 */

#define  FLASH_OPTKEY1                       FLASH_KEY1                    /*!< Option Byte Key1 */
#define  FLASH_OPTKEY2                       FLASH_KEY2                    /*!< Option Byte Key2 */

///


static uint64_t
stm32_flash_regs_read(void *arg, hwaddr addr, unsigned int size)
{
	Stm32FlashRegs *s = arg;

	if (size != 4) {
		qemu_log_mask(LOG_UNIMP, "stm32 flash only supports 4-byte reads\n");
		return 0;
	}

	addr >>= 2;
	if (addr >= R_FLASH_MAX) {
		qemu_log_mask(LOG_GUEST_ERROR, "invalid read stm32 flash register 0x%x\n",
		  (unsigned int)addr << 2);
		return 0;
	}

	switch(addr) {
	case R_FLASH_ACR:
		return s->ACR;
	case R_FLASH_KEYR:
		return s->KEYR;
	case R_FLASH_OPTKEYR:
		return s->OPTKEYR;
	case R_FLASH_SR:
		return s->SR;
	case R_FLASH_CR:
		return s->CR;
	case R_FLASH_AR:
		return s->AR;
	case R_FLASH_RESERVED:
		return s->RESERVED;
	case R_FLASH_OBR:
		return s->OBR;
	case R_FLASH_WRPR:
		return s->WRPR;
	}
	return 0;
}


static void
stm32_flash_regs_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
	Stm32FlashRegs *s = arg;

	/* XXX Check periph clock enable. */
	if (size != 4) {
		qemu_log_mask(LOG_UNIMP, "stm32 flash only supports 4-byte writes\n");
		return;
	}

	addr >>= 2;
	if (addr >= R_FLASH_MAX) {
		qemu_log_mask(LOG_GUEST_ERROR, "invalid write stm32 flash register 0x%x\n",
		  (unsigned int)addr << 2);
		return;
	}
	switch(addr) {
	case R_FLASH_ACR:
		s->ACR = data;
		break;

	case R_FLASH_KEYR:
		if (s->KEYR == FLASH_OPTKEY1 && data == FLASH_OPTKEY2) {
#ifdef DEBUG_FLASH
			printf("Flash is unlocked!\n");
#endif            
			s->CR &= ~FLASH_CR_LOCK;
			is_flash_locked = 0;
		}
		s->KEYR = data;
		break;

	case R_FLASH_OPTKEYR:
		s->OPTKEYR = data;
		break;

	case R_FLASH_SR:
		s->SR = data;
		break;

	case R_FLASH_CR:
		if (is_flash_locked == 0 && (data & FLASH_CR_LOCK)) {
			if (data & FLASH_CR_PG)
				hw_error("stm32_flash: Attempted to write flash lock while flash program is on!");
#ifdef DEBUG_FLASH            
			printf("Flash is locked!\n");
#endif            
			//s->CR &= ~FLASH_CR_LOCK;
			is_flash_locked = 1;
            memory_region_set_readonly(&flash->mem, true);

		} else if ( (s->CR & FLASH_CR_PER) && (data & FLASH_CR_STRT) ) { //erase
			if (data & FLASH_CR_PG || (data & FLASH_CR_LOCK))
				hw_error("stm32_flash: Attempted to erase flash block while flash program/flash lock is on!");
#ifdef DEBUG_FLASH
			printf("start erase address 0x%08X \n", s->AR);
#endif			
            if ( (s->AR % 1024) == 0 && (s->AR >= STM32_FLASH_ADDR_START) && (s->AR <= (STM32_FLASH_ADDR_START+flash->size-1024) ) ) { 
                memset(flash->data+(s->AR-STM32_FLASH_ADDR_START) , 0xFF, 1024);
#ifdef DEBUG_FLASH              
              printf("erased\n");
#endif              
			} else {
				printf("ADDRESS: %u\n", s->AR);
				hw_error("stm32_flash: Attempted to erase flash memory page while address is not alligned!");
			}
            
		} else if (data & FLASH_CR_PG) {
			if (data & FLASH_CR_LOCK || data & FLASH_CR_PER)
				hw_error("stm32_flash: Attempted to write flash program while flash lock/flash erase is on!");
			flash_programming_bit = 1;
            memory_region_set_readonly(&flash->mem, false);

		} else if (data & ~FLASH_CR_PG) {
			flash_programming_bit = 0;
		}

		s->CR = data;
		break;

	case R_FLASH_AR:
		s->AR = data;
		break;

	case R_FLASH_RESERVED:
		s->RESERVED = data;
		break;

	case R_FLASH_OBR:
		s->OBR = data;
		break;

	case R_FLASH_WRPR:
		s->WRPR = data;
		break;

	}

	return;
};


static const MemoryRegionOps stm32_flash_regs_ops = {
	.read = stm32_flash_regs_read,
	.write = stm32_flash_regs_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
	.impl = {
		.min_access_size = 1,
		.max_access_size = 4,
	}
};






static void
stm32_flash_regs_reset(DeviceState *ds)
{
	Stm32FlashRegs *s = STM32_FLASH_REGS(ds);

	s->ACR = 0;
	s->KEYR = 0;
	s->OPTKEYR = 0;
	s->SR = 0;
	s->CR = 0;
	s->AR = 0;
	s->RESERVED = 0;
	s->OBR = 0;
	s->WRPR = 0;

	is_flash_locked = 1;
}

static void stm32_flash_regs_realize(DeviceState *dev, Error **errp) {
    Stm32FlashRegs *s = STM32_FLASH_REGS(dev);
    SysBusDevice *busdev = SYS_BUS_DEVICE(dev);

	memory_region_init_io(&s->iomem, OBJECT(s), &stm32_flash_regs_ops, s, "flash-regs", 0x400);
	sysbus_init_mmio(busdev, &s->iomem);

    stm32_flash_regs_reset(dev);
}


static void
stm32_flash_regs_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	//SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(klass);
	//sc->init = stm32_flash_regs_init;
    dc->realize = stm32_flash_regs_realize;
	dc->reset = stm32_flash_regs_reset;
}

static const TypeInfo
stm32_flash_regs_info = {
	.name          = TYPE_STM32_FLASH_REGS,
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(Stm32FlashRegs),
    // .instance_init = stm32_flash_regs_init,
	.class_init    = stm32_flash_regs_class_init,
};

static void
stm32_flash_regs_register_types(void)
{
	type_register_static(&stm32_flash_regs_info);
}

type_init(stm32_flash_regs_register_types);




