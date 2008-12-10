/*
 * QEMU MIPS Jazz support
 *
 * Copyright (c) 2007-2008 Hervé Poussineau
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
#include "mips.h"
#include "pc.h"
#include "isa.h"
#include "fdc.h"
#include "sysemu.h"
#include "audio/audio.h"
#include "boards.h"
#include "net.h"
#include "scsi.h"

#ifdef TARGET_WORDS_BIGENDIAN
#define BIOS_FILENAME "mips_bios.bin"
#else
#define BIOS_FILENAME "mipsel_bios.bin"
#endif

enum jazz_model_e
{
    JAZZ_MAGNUM,
    JAZZ_PICA61,
};

static void main_cpu_reset(void *opaque)
{
    CPUState *env = opaque;
    cpu_reset(env);
}

static uint32_t rtc_readb(void *opaque, target_phys_addr_t addr)
{
    CPUState *env = opaque;
    return cpu_inw(env, 0x71);
}

static void rtc_writeb(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    CPUState *env = opaque;
    cpu_outw(env, 0x71, val & 0xff);
}

static CPUReadMemoryFunc *rtc_read[3] = {
    rtc_readb,
    rtc_readb,
    rtc_readb,
};

static CPUWriteMemoryFunc *rtc_write[3] = {
    rtc_writeb,
    rtc_writeb,
    rtc_writeb,
};

#ifdef HAS_AUDIO
static void audio_init(qemu_irq *pic)
{
    struct soundhw *c;
    int audio_enabled = 0;

    for (c = soundhw; !audio_enabled && c->name; ++c) {
        audio_enabled = c->enabled;
    }

    if (audio_enabled) {
        AudioState *s;

        s = AUD_init();
        if (s) {
            for (c = soundhw; c->name; ++c) {
                if (c->enabled) {
                    if (c->isa) {
                        c->init.init_isa(s, pic);
                    }
                }
            }
        }
    }
}
#endif

static void espdma_memory_read(void *opaque, uint8_t *buf, int len)
{
    printf("espdma_memory_read(buf %p, len %d) not implemented\n", buf, len);
}

static void espdma_memory_write(void *opaque, uint8_t *buf, int len)
{
    printf("espdma_memory_write(buf %p, len %d) not implemented\n", buf, len);
}

#define MAGNUM_BIOS_SIZE_MAX 0x7e000
#define MAGNUM_BIOS_SIZE (BIOS_SIZE < MAGNUM_BIOS_SIZE_MAX ? BIOS_SIZE : MAGNUM_BIOS_SIZE_MAX)

static
void mips_jazz_init (ram_addr_t ram_size, int vga_ram_size,
                     DisplayState *ds, const char *cpu_model,
                     enum jazz_model_e jazz_model)
{
    char buf[1024];
    unsigned long bios_offset;
    int bios_size, n;
    CPUState *env;
    qemu_irq *rc4030, *i8259;
    void *scsi_hba;
    int hd;
    int s_rtc;
    PITState *pit;
    BlockDriverState *fds[MAX_FD];
    qemu_irq esp_reset;

    /* init CPUs */
    if (cpu_model == NULL) {
#ifdef TARGET_MIPS64
        cpu_model = "R4000";
#else
        /* FIXME: All wrong, this maybe should be R3000 for the older JAZZs. */
        cpu_model = "24Kf";
#endif
    }
    env = cpu_init(cpu_model);
    if (!env) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }
    qemu_register_reset(main_cpu_reset, env);

    /* allocate RAM */
    cpu_register_physical_memory(0, ram_size, IO_MEM_RAM);

    /* load the BIOS image. */
    bios_offset = ram_size + vga_ram_size;
    snprintf(buf, sizeof(buf), "%s/%s", bios_dir, BIOS_FILENAME);
    bios_size = load_image(buf, phys_ram_base + bios_offset);
    if (bios_size < 0 || bios_size > MAGNUM_BIOS_SIZE) {
        fprintf(stderr, "qemu: Could not load MIPS bios '%s'\n",
                buf);
        exit(1);
    }

    cpu_register_physical_memory(0x1fc00000LL,
                                 MAGNUM_BIOS_SIZE, bios_offset | IO_MEM_ROM);
    cpu_register_physical_memory(0xfff00000LL,
                                 MAGNUM_BIOS_SIZE, bios_offset | IO_MEM_ROM);

    /* Init CPU internal devices */
    cpu_mips_irq_init_cpu(env);
    cpu_mips_clock_init(env);

    /* Chipset */
    rc4030 = rc4030_init(env->irq[6], env->irq[3]);

    /* ISA devices */
    i8259 = i8259_init(env->irq[4]);
    pit = pit_init(0x40, i8259[0]);
    pcspk_init(pit);

    /* ISA IO space at 0x90000000 */
    isa_mmio_init(0x90000000, 0x01000000);
    isa_mem_base = 0x11000000;

    /* Video card */
    switch (jazz_model) {
    case JAZZ_MAGNUM:
        g364fb_mm_init(ds, vga_ram_size, 0, 0x40000000, 0x60000000);
        break;
    case JAZZ_PICA61:
        isa_vga_mm_init(ds, phys_ram_base + ram_size, ram_size, vga_ram_size,
                        0x40000000, 0x60000000, 0);
        break;
    default:
        break;
    }

    /* Network controller */
    /* FIXME: missing NS SONIC DP83932 */

    /* SCSI adapter */
    scsi_hba = esp_init(0x80002000, 0,
                        espdma_memory_read, espdma_memory_write, NULL,
                        rc4030[5], &esp_reset);
    for (n = 0; n < ESP_MAX_DEVS; n++) {
        hd = drive_get_index(IF_SCSI, 0, n);
        if (hd != -1) {
            esp_scsi_attach(scsi_hba, drives_table[hd].bdrv, n);
        }
    }

    /* Floppy */
    if (drive_get_max_bus(IF_FLOPPY) >= MAX_FD) {
        fprintf(stderr, "qemu: too many floppy drives\n");
        exit(1);
    }
    for (n = 0; n < MAX_FD; n++) {
        int fd = drive_get_index(IF_FLOPPY, 0, n);
        if (fd != -1)
            fds[n] = drives_table[fd].bdrv;
        else
            fds[n] = NULL;
    }
    fdctrl_init(rc4030[1], 0, 1, 0x80003000, fds);

    /* Real time clock */
    rtc_init(0x70, i8259[8]);
    s_rtc = cpu_register_io_memory(0, rtc_read, rtc_write, env);
    cpu_register_physical_memory(0x80004000, 0x00001000, s_rtc);

    /* Keyboard (i8042) */
    i8042_mm_init(rc4030[6], rc4030[7], 0x80005000, 0x1000, 0x1);

    /* Serial ports */
    if (serial_hds[0])
        serial_mm_init(0x80006000, 0, rc4030[8], 8000000/16, serial_hds[0], 1);
    if (serial_hds[1])
        serial_mm_init(0x80007000, 0, rc4030[9], 8000000/16, serial_hds[1], 1);

    /* Parallel port */
    if (parallel_hds[0])
        parallel_mm_init(0x80008000, 0, rc4030[0], parallel_hds[0]);

    /* Sound card */
    /* FIXME: missing Jazz sound at 0x8000c000, rc4030[2] */
#ifdef HAS_AUDIO
    audio_init(i8259);
#endif

    /* NVRAM: Unprotected at 0x9000, Protected at 0xa000, Read only at 0xb000 */
    ds1225y_init(0x80009000, "nvram");

    /* LED indicator */
    jazz_led_init(ds, 0x8000f000);
}

static
void mips_magnum_init (ram_addr_t ram_size, int vga_ram_size,
                       const char *boot_device, DisplayState *ds,
                       const char *kernel_filename, const char *kernel_cmdline,
                       const char *initrd_filename, const char *cpu_model)
{
    mips_jazz_init(ram_size, vga_ram_size, ds, cpu_model, JAZZ_MAGNUM);
}

static
void mips_pica61_init (ram_addr_t ram_size, int vga_ram_size,
                       const char *boot_device, DisplayState *ds,
                       const char *kernel_filename, const char *kernel_cmdline,
                       const char *initrd_filename, const char *cpu_model)
{
    mips_jazz_init(ram_size, vga_ram_size, ds, cpu_model, JAZZ_PICA61);
}

QEMUMachine mips_magnum_machine = {
    .name = "magnum",
    .desc = "MIPS Magnum",
    .init = mips_magnum_init,
    .ram_require = MAGNUM_BIOS_SIZE + VGA_RAM_SIZE,
    .nodisk_ok = 1,
};

QEMUMachine mips_pica61_machine = {
    .name = "pica61",
    .desc = "Acer Pica 61",
    .init = mips_pica61_init,
    .ram_require = MAGNUM_BIOS_SIZE + VGA_RAM_SIZE,
    .nodisk_ok = 1,
};
