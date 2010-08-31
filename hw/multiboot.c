/*
 * QEMU PC System Emulator
 *
 * Copyright (c) 2003-2004 Fabrice Bellard
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

#include "hw.h"
#include "fw_cfg.h"
#include "multiboot.h"
#include "loader.h"
#include "elf.h"
#include "sysemu.h"

/* Show multiboot debug output */
//#define DEBUG_MULTIBOOT

#ifdef DEBUG_MULTIBOOT
#define mb_debug(a...) fprintf(stderr, ## a)
#else
#define mb_debug(a...)
#endif

#define MULTIBOOT_STRUCT_ADDR 0x9000

#if MULTIBOOT_STRUCT_ADDR > 0xf0000
#error multiboot struct needs to fit in 16 bit real mode
#endif

enum {
    /* Multiboot info */
    MBI_FLAGS       = 0,
    MBI_MEM_LOWER   = 4,
    MBI_MEM_UPPER   = 8,
    MBI_BOOT_DEVICE = 12,
    MBI_CMDLINE     = 16,
    MBI_MODS_COUNT  = 20,
    MBI_MODS_ADDR   = 24,
    MBI_MMAP_ADDR   = 48,

    MBI_SIZE        = 88,

    /* Multiboot modules */
    MB_MOD_START    = 0,
    MB_MOD_END      = 4,
    MB_MOD_CMDLINE  = 8,

    MB_MOD_SIZE     = 16,

    /* Region offsets */
    ADDR_E820_MAP = MULTIBOOT_STRUCT_ADDR + 0,
    ADDR_MBI      = ADDR_E820_MAP + 0x500,

    /* Multiboot flags */
    MULTIBOOT_FLAGS_MEMORY      = 1 << 0,
    MULTIBOOT_FLAGS_BOOT_DEVICE = 1 << 1,
    MULTIBOOT_FLAGS_CMDLINE     = 1 << 2,
    MULTIBOOT_FLAGS_MODULES     = 1 << 3,
    MULTIBOOT_FLAGS_MMAP        = 1 << 6,
};

typedef struct {
    /* buffer holding kernel, cmdlines and mb_infos */
    void *mb_buf;
    /* address in target */
    target_phys_addr_t mb_buf_phys;
    /* size of mb_buf in bytes */
    unsigned mb_buf_size;
    /* offset of mb-info's in bytes */
    target_phys_addr_t offset_mbinfo;
    /* offset in buffer for cmdlines in bytes */
    target_phys_addr_t offset_cmdlines;
    /* offset of modules in bytes */
    target_phys_addr_t offset_mods;
    /* available slots for mb modules infos */
    int mb_mods_avail;
    /* currently used slots of mb modules */
    int mb_mods_count;
} MultibootState;

static uint32_t mb_add_cmdline(MultibootState *s, const char *cmdline)
{
    int len = strlen(cmdline) + 1;
    target_phys_addr_t p = s->offset_cmdlines;

    pstrcpy((char *)s->mb_buf + p, len, cmdline);
    s->offset_cmdlines += len;
    return s->mb_buf_phys + p;
}

static void mb_add_mod(MultibootState *s,
                       target_phys_addr_t start, target_phys_addr_t end,
                       target_phys_addr_t cmdline_phys)
{
    char *p;
    assert(s->mb_mods_count < s->mb_mods_avail);

    p = (char *)s->mb_buf + s->offset_mbinfo + MB_MOD_SIZE * s->mb_mods_count;

    stl_p(p + MB_MOD_START,   start);
    stl_p(p + MB_MOD_END,     end);
    stl_p(p + MB_MOD_CMDLINE, cmdline_phys);

    mb_debug("mod%02d: "TARGET_FMT_plx" - "TARGET_FMT_plx"\n",
             s->mb_mods_count, start, end);

    s->mb_mods_count++;
}

int load_multiboot(void *fw_cfg,
                   FILE *f,
                   const char *kernel_filename,
                   const char *initrd_filename,
                   const char *kernel_cmdline,
                   int kernel_file_size,
                   uint8_t *header)
{
    int i, is_multiboot = 0;
    uint32_t flags = 0;
    uint32_t mh_entry_addr;
    uint32_t mh_load_addr;
    uint32_t mb_kernel_size;
    MultibootState mbs;
    uint8_t bootinfo[MBI_SIZE];
    uint8_t *mb_bootinfo_data;

    /* Ok, let's see if it is a multiboot image.
       The header is 12x32bit long, so the latest entry may be 8192 - 48. */
    for (i = 0; i < (8192 - 48); i += 4) {
        if (ldl_p(header+i) == 0x1BADB002) {
            uint32_t checksum = ldl_p(header+i+8);
            flags = ldl_p(header+i+4);
            checksum += flags;
            checksum += (uint32_t)0x1BADB002;
            if (!checksum) {
                is_multiboot = 1;
                break;
            }
        }
    }

    if (!is_multiboot)
        return 0; /* no multiboot */

    mb_debug("qemu: I believe we found a multiboot image!\n");
    memset(bootinfo, 0, sizeof(bootinfo));
    memset(&mbs, 0, sizeof(mbs));

    if (flags & 0x00000004) { /* MULTIBOOT_HEADER_HAS_VBE */
        fprintf(stderr, "qemu: multiboot knows VBE. we don't.\n");
    }
    if (!(flags & 0x00010000)) { /* MULTIBOOT_HEADER_HAS_ADDR */
        uint64_t elf_entry;
        uint64_t elf_low, elf_high;
        int kernel_size;
        fclose(f);
        kernel_size = load_elf(kernel_filename, NULL, NULL, &elf_entry,
                               &elf_low, &elf_high, 0, ELF_MACHINE, 0);
        if (kernel_size < 0) {
            fprintf(stderr, "Error while loading elf kernel\n");
            exit(1);
        }
        mh_load_addr = elf_low;
        mb_kernel_size = elf_high - elf_low;
        mh_entry_addr = elf_entry;

        mbs.mb_buf = qemu_malloc(mb_kernel_size);
        if (rom_copy(mbs.mb_buf, mh_load_addr, mb_kernel_size) != mb_kernel_size) {
            fprintf(stderr, "Error while fetching elf kernel from rom\n");
            exit(1);
        }

        mb_debug("qemu: loading multiboot-elf kernel (%#x bytes) with entry %#zx\n",
                  mb_kernel_size, (size_t)mh_entry_addr);
    } else {
        /* Valid if mh_flags sets MULTIBOOT_HEADER_HAS_ADDR. */
        uint32_t mh_header_addr = ldl_p(header+i+12);
        mh_load_addr = ldl_p(header+i+16);
        uint32_t mb_kernel_text_offset = i - (mh_header_addr - mh_load_addr);

        mh_entry_addr = ldl_p(header+i+28);
        mb_kernel_size = kernel_file_size - mb_kernel_text_offset;

        /* Valid if mh_flags sets MULTIBOOT_HEADER_HAS_VBE.
        uint32_t mh_mode_type = ldl_p(header+i+32);
        uint32_t mh_width = ldl_p(header+i+36);
        uint32_t mh_height = ldl_p(header+i+40);
        uint32_t mh_depth = ldl_p(header+i+44); */

        mb_debug("multiboot: mh_header_addr = %#x\n", mh_header_addr);
        mb_debug("multiboot: mh_load_addr = %#x\n", mh_load_addr);
        mb_debug("multiboot: mh_load_end_addr = %#x\n", ldl_p(header+i+20));
        mb_debug("multiboot: mh_bss_end_addr = %#x\n", ldl_p(header+i+24));
        mb_debug("qemu: loading multiboot kernel (%#x bytes) at %#x\n",
                 mb_kernel_size, mh_load_addr);

        mbs.mb_buf = qemu_malloc(mb_kernel_size);
        fseek(f, mb_kernel_text_offset, SEEK_SET);
        if (fread(mbs.mb_buf, 1, mb_kernel_size, f) != mb_kernel_size) {
            fprintf(stderr, "fread() failed\n");
            exit(1);
        }
        fclose(f);
    }

    mbs.mb_buf_phys = mh_load_addr;

    mbs.mb_buf_size = TARGET_PAGE_ALIGN(mb_kernel_size);
    mbs.offset_mbinfo = mbs.mb_buf_size;

    /* Calculate space for cmdlines and mb_mods */
    mbs.mb_buf_size += strlen(kernel_filename) + 1;
    mbs.mb_buf_size += strlen(kernel_cmdline) + 1;
    if (initrd_filename) {
        const char *r = initrd_filename;
        mbs.mb_buf_size += strlen(r) + 1;
        mbs.mb_mods_avail = 1;
        while ((r = strchr(r, ','))) {
           mbs.mb_mods_avail++;
           r++;
        }
        mbs.mb_buf_size += MB_MOD_SIZE * mbs.mb_mods_avail;
    }

    mbs.mb_buf_size = TARGET_PAGE_ALIGN(mbs.mb_buf_size);

    /* enlarge mb_buf to hold cmdlines and mb-info structs */
    mbs.mb_buf          = qemu_realloc(mbs.mb_buf, mbs.mb_buf_size);
    mbs.offset_cmdlines = mbs.offset_mbinfo + mbs.mb_mods_avail * MB_MOD_SIZE;

    if (initrd_filename) {
        char *next_initrd;

        mbs.offset_mods = mbs.mb_buf_size;

        do {
            char *next_space;
            int mb_mod_length;
            uint32_t offs = mbs.mb_buf_size;

            next_initrd = strchr(initrd_filename, ',');
            if (next_initrd)
                *next_initrd = '\0';
            /* if a space comes after the module filename, treat everything
               after that as parameters */
            target_phys_addr_t c = mb_add_cmdline(&mbs, initrd_filename);
            if ((next_space = strchr(initrd_filename, ' ')))
                *next_space = '\0';
            mb_debug("multiboot loading module: %s\n", initrd_filename);
            mb_mod_length = get_image_size(initrd_filename);
            if (mb_mod_length < 0) {
                fprintf(stderr, "failed to get %s image size\n", initrd_filename);
                exit(1);
            }

            mbs.mb_buf_size = TARGET_PAGE_ALIGN(mb_mod_length + mbs.mb_buf_size);
            mbs.mb_buf = qemu_realloc(mbs.mb_buf, mbs.mb_buf_size);

            load_image(initrd_filename, (unsigned char *)mbs.mb_buf + offs);
            mb_add_mod(&mbs, mbs.mb_buf_phys + offs,
                       mbs.mb_buf_phys + offs + mb_mod_length, c);

            mb_debug("mod_start: %p\nmod_end:   %p\n  cmdline: "TARGET_FMT_plx"\n",
                     (char *)mbs.mb_buf + offs,
                     (char *)mbs.mb_buf + offs + mb_mod_length, c);
            initrd_filename = next_initrd+1;
        } while (next_initrd);
    }

    /* Commandline support */
    char kcmdline[strlen(kernel_filename) + strlen(kernel_cmdline) + 2];
    snprintf(kcmdline, sizeof(kcmdline), "%s %s",
             kernel_filename, kernel_cmdline);
    stl_p(bootinfo + MBI_CMDLINE, mb_add_cmdline(&mbs, kcmdline));

    stl_p(bootinfo + MBI_MODS_ADDR,  mbs.mb_buf_phys + mbs.offset_mbinfo);
    stl_p(bootinfo + MBI_MODS_COUNT, mbs.mb_mods_count); /* mods_count */

    /* the kernel is where we want it to be now */
    stl_p(bootinfo + MBI_FLAGS, MULTIBOOT_FLAGS_MEMORY
                                | MULTIBOOT_FLAGS_BOOT_DEVICE
                                | MULTIBOOT_FLAGS_CMDLINE
                                | MULTIBOOT_FLAGS_MODULES
                                | MULTIBOOT_FLAGS_MMAP);
    stl_p(bootinfo + MBI_MEM_LOWER,   640);
    stl_p(bootinfo + MBI_MEM_UPPER,   ram_size / 1024);
    stl_p(bootinfo + MBI_BOOT_DEVICE, 0x8001ffff); /* XXX: use the -boot switch? */
    stl_p(bootinfo + MBI_MMAP_ADDR,   ADDR_E820_MAP);

    mb_debug("multiboot: mh_entry_addr = %#x\n", mh_entry_addr);
    mb_debug("           mb_buf_phys   = "TARGET_FMT_plx"\n", mbs.mb_buf_phys);
    mb_debug("           mod_start     = "TARGET_FMT_plx"\n", mbs.mb_buf_phys + mbs.offset_mods);
    mb_debug("           mb_mods_count = %d\n", mbs.mb_mods_count);

    /* save bootinfo off the stack */
    mb_bootinfo_data = qemu_malloc(sizeof(bootinfo));
    memcpy(mb_bootinfo_data, bootinfo, sizeof(bootinfo));

    /* Pass variables to option rom */
    fw_cfg_add_i32(fw_cfg, FW_CFG_KERNEL_ENTRY, mh_entry_addr);
    fw_cfg_add_i32(fw_cfg, FW_CFG_KERNEL_ADDR, mh_load_addr);
    fw_cfg_add_i32(fw_cfg, FW_CFG_KERNEL_SIZE, mbs.mb_buf_size);
    fw_cfg_add_bytes(fw_cfg, FW_CFG_KERNEL_DATA,
                     mbs.mb_buf, mbs.mb_buf_size);

    fw_cfg_add_i32(fw_cfg, FW_CFG_INITRD_ADDR, ADDR_MBI);
    fw_cfg_add_i32(fw_cfg, FW_CFG_INITRD_SIZE, sizeof(bootinfo));
    fw_cfg_add_bytes(fw_cfg, FW_CFG_INITRD_DATA, mb_bootinfo_data,
                     sizeof(bootinfo));

    option_rom[nb_option_roms] = "multiboot.bin";
    nb_option_roms++;

    return 1; /* yes, we are multiboot */
}
