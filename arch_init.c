/*
 * QEMU System Emulator
 *
 * Copyright (c) 2003-2008 Fabrice Bellard
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
#include <stdint.h>
#include <stdarg.h>
#ifndef _WIN32
#include <sys/types.h>
#include <sys/mman.h>
#endif
#include "config.h"
#include "monitor.h"
#include "sysemu.h"
#include "arch_init.h"
#include "audio/audio.h"
#include "hw/pc.h"
#include "hw/pci.h"
#include "hw/audiodev.h"
#include "kvm.h"
#include "migration.h"
#include "net.h"
#include "gdbstub.h"
#include "hw/smbios.h"

#ifdef TARGET_SPARC
int graphic_width = 1024;
int graphic_height = 768;
int graphic_depth = 8;
#else
int graphic_width = 800;
int graphic_height = 600;
int graphic_depth = 15;
#endif

const char arch_config_name[] = CONFIG_QEMU_CONFDIR "/target-" TARGET_ARCH ".conf";

#if defined(TARGET_ALPHA)
#define QEMU_ARCH QEMU_ARCH_ALPHA
#elif defined(TARGET_ARM)
#define QEMU_ARCH QEMU_ARCH_ARM
#elif defined(TARGET_CRIS)
#define QEMU_ARCH QEMU_ARCH_CRIS
#elif defined(TARGET_I386)
#define QEMU_ARCH QEMU_ARCH_I386
#elif defined(TARGET_M68K)
#define QEMU_ARCH QEMU_ARCH_M68K
#elif defined(TARGET_MICROBLAZE)
#define QEMU_ARCH QEMU_ARCH_MICROBLAZE
#elif defined(TARGET_MIPS)
#define QEMU_ARCH QEMU_ARCH_MIPS
#elif defined(TARGET_PPC)
#define QEMU_ARCH QEMU_ARCH_PPC
#elif defined(TARGET_S390X)
#define QEMU_ARCH QEMU_ARCH_S390X
#elif defined(TARGET_SH4)
#define QEMU_ARCH QEMU_ARCH_SH4
#elif defined(TARGET_SPARC)
#define QEMU_ARCH QEMU_ARCH_SPARC
#endif

const uint32_t arch_type = QEMU_ARCH;

/***********************************************************/
/* ram save/restore */

#define RAM_SAVE_FLAG_FULL	0x01 /* Obsolete, not used anymore */
#define RAM_SAVE_FLAG_COMPRESS	0x02
#define RAM_SAVE_FLAG_MEM_SIZE	0x04
#define RAM_SAVE_FLAG_PAGE	0x08
#define RAM_SAVE_FLAG_EOS	0x10

static int is_dup_page(uint8_t *page, uint8_t ch)
{
    uint32_t val = ch << 24 | ch << 16 | ch << 8 | ch;
    uint32_t *array = (uint32_t *)page;
    int i;

    for (i = 0; i < (TARGET_PAGE_SIZE / 4); i++) {
        if (array[i] != val) {
            return 0;
        }
    }

    return 1;
}

static int ram_save_block(QEMUFile *f)
{
    static ram_addr_t current_addr = 0;
    ram_addr_t saved_addr = current_addr;
    ram_addr_t addr = 0;
    uint64_t total_ram = ram_bytes_total();
    int bytes_sent = 0;

    while (addr < total_ram) {
        if (cpu_physical_memory_get_dirty(current_addr, MIGRATION_DIRTY_FLAG)) {
            RAMBlock *block;
            ram_addr_t offset;
            uint8_t *p;

            cpu_physical_memory_reset_dirty(current_addr,
                                            current_addr + TARGET_PAGE_SIZE,
                                            MIGRATION_DIRTY_FLAG);

            QLIST_FOREACH(block, &ram_list.blocks, next) {
                if (current_addr - block->offset < block->length)
                    break;
            }
            offset = current_addr - block->offset;
            p = block->host + offset;

            if (is_dup_page(p, *p)) {
                qemu_put_be64(f, offset | RAM_SAVE_FLAG_COMPRESS);
                qemu_put_byte(f, strlen(block->idstr));
                qemu_put_buffer(f, (uint8_t *)block->idstr,
                                strlen(block->idstr));
                qemu_put_byte(f, *p);
                bytes_sent = 1;
            } else {
                qemu_put_be64(f, offset | RAM_SAVE_FLAG_PAGE);
                qemu_put_byte(f, strlen(block->idstr));
                qemu_put_buffer(f, (uint8_t *)block->idstr,
                                strlen(block->idstr));
                qemu_put_buffer(f, p, TARGET_PAGE_SIZE);
                bytes_sent = TARGET_PAGE_SIZE;
            }

            break;
        }
        addr += TARGET_PAGE_SIZE;
        current_addr = (saved_addr + addr) % total_ram;
    }

    return bytes_sent;
}

static uint64_t bytes_transferred;

static ram_addr_t ram_save_remaining(void)
{
    ram_addr_t addr;
    ram_addr_t count = 0;
    uint64_t total_ram = ram_bytes_total();

    for (addr = 0; addr < total_ram; addr += TARGET_PAGE_SIZE) {
        if (cpu_physical_memory_get_dirty(addr, MIGRATION_DIRTY_FLAG)) {
            count++;
        }
    }

    return count;
}

uint64_t ram_bytes_remaining(void)
{
    return ram_save_remaining() * TARGET_PAGE_SIZE;
}

uint64_t ram_bytes_transferred(void)
{
    return bytes_transferred;
}

uint64_t ram_bytes_total(void)
{
    RAMBlock *block;
    uint64_t total = 0;

    QLIST_FOREACH(block, &ram_list.blocks, next)
        total += block->length;

    return total;
}

int ram_save_live(Monitor *mon, QEMUFile *f, int stage, void *opaque)
{
    ram_addr_t addr;
    uint64_t bytes_transferred_last;
    double bwidth = 0;
    uint64_t expected_time = 0;

    if (stage < 0) {
        cpu_physical_memory_set_dirty_tracking(0);
        return 0;
    }

    if (cpu_physical_sync_dirty_bitmap(0, TARGET_PHYS_ADDR_MAX) != 0) {
        qemu_file_set_error(f);
        return 0;
    }

    if (stage == 1) {
        RAMBlock *block;
        uint64_t total_ram = ram_bytes_total();
        bytes_transferred = 0;

        /* Make sure all dirty bits are set */
        for (addr = 0; addr < total_ram; addr += TARGET_PAGE_SIZE) {
            if (!cpu_physical_memory_get_dirty(addr, MIGRATION_DIRTY_FLAG)) {
                cpu_physical_memory_set_dirty(addr);
            }
        }

        /* Enable dirty memory tracking */
        cpu_physical_memory_set_dirty_tracking(1);

        qemu_put_be64(f, total_ram | RAM_SAVE_FLAG_MEM_SIZE);

        QLIST_FOREACH(block, &ram_list.blocks, next) {
            qemu_put_byte(f, strlen(block->idstr));
            qemu_put_buffer(f, (uint8_t *)block->idstr, strlen(block->idstr));
            qemu_put_be64(f, block->length);
        }
    }

    bytes_transferred_last = bytes_transferred;
    bwidth = qemu_get_clock_ns(rt_clock);

    while (!qemu_file_rate_limit(f)) {
        int bytes_sent;

        bytes_sent = ram_save_block(f);
        bytes_transferred += bytes_sent;
        if (bytes_sent == 0) { /* no more blocks */
            break;
        }
    }

    bwidth = qemu_get_clock_ns(rt_clock) - bwidth;
    bwidth = (bytes_transferred - bytes_transferred_last) / bwidth;

    /* if we haven't transferred anything this round, force expected_time to a
     * a very high value, but without crashing */
    if (bwidth == 0) {
        bwidth = 0.000001;
    }

    /* try transferring iterative blocks of memory */
    if (stage == 3) {
        int bytes_sent;

        /* flush all remaining blocks regardless of rate limiting */
        while ((bytes_sent = ram_save_block(f)) != 0) {
            bytes_transferred += bytes_sent;
        }
        cpu_physical_memory_set_dirty_tracking(0);
    }

    qemu_put_be64(f, RAM_SAVE_FLAG_EOS);

    expected_time = ram_save_remaining() * TARGET_PAGE_SIZE / bwidth;

    return (stage == 2) && (expected_time <= migrate_max_downtime());
}

int ram_load(QEMUFile *f, void *opaque, int version_id)
{
    ram_addr_t addr;
    int flags;

    if (version_id < 3 || version_id > 4) {
        return -EINVAL;
    }

    do {
        addr = qemu_get_be64(f);

        flags = addr & ~TARGET_PAGE_MASK;
        addr &= TARGET_PAGE_MASK;

        if (flags & RAM_SAVE_FLAG_MEM_SIZE) {
            if (version_id == 3) {
                if (addr != ram_bytes_total()) {
                    return -EINVAL;
                }
            } else {
                /* Synchronize RAM block list */
                char id[256];
                ram_addr_t length;
                ram_addr_t total_ram_bytes = addr;

                while (total_ram_bytes) {
                    RAMBlock *block;
                    uint8_t len;

                    len = qemu_get_byte(f);
                    qemu_get_buffer(f, (uint8_t *)id, len);
                    id[len] = 0;
                    length = qemu_get_be64(f);

                    QLIST_FOREACH(block, &ram_list.blocks, next) {
                        if (!strncmp(id, block->idstr, sizeof(id))) {
                            if (block->length != length)
                                return -EINVAL;
                            break;
                        }
                    }

                    if (!block) {
                        if (!qemu_ram_alloc(NULL, id, length))
                            return -ENOMEM;
                    }

                    total_ram_bytes -= length;
                }
            }
        }

        if (flags & RAM_SAVE_FLAG_COMPRESS) {
            void *host;
            uint8_t ch;

            if (version_id == 3) {
                host = qemu_get_ram_ptr(addr);
            } else {
                RAMBlock *block;
                char id[256];
                uint8_t len;

                len = qemu_get_byte(f);
                qemu_get_buffer(f, (uint8_t *)id, len);
                id[len] = 0;

                QLIST_FOREACH(block, &ram_list.blocks, next) {
                    if (!strncmp(id, block->idstr, sizeof(id)))
                        break;
                }
                if (!block)
                    return -EINVAL;

                host = block->host + addr;
            }
            ch = qemu_get_byte(f);
            memset(host, ch, TARGET_PAGE_SIZE);
#ifndef _WIN32
            if (ch == 0 &&
                (!kvm_enabled() || kvm_has_sync_mmu())) {
                madvise(host, TARGET_PAGE_SIZE, MADV_DONTNEED);
            }
#endif
        } else if (flags & RAM_SAVE_FLAG_PAGE) {
            void *host;

            if (version_id == 3) {
                host = qemu_get_ram_ptr(addr);
            } else {
                RAMBlock *block;
                char id[256];
                uint8_t len;

                len = qemu_get_byte(f);
                qemu_get_buffer(f, (uint8_t *)id, len);
                id[len] = 0;

                QLIST_FOREACH(block, &ram_list.blocks, next) {
                    if (!strncmp(id, block->idstr, sizeof(id)))
                        break;
                }
                if (!block)
                    return -EINVAL;

                host = block->host + addr;
            }
            qemu_get_buffer(f, host, TARGET_PAGE_SIZE);
        }
        if (qemu_file_has_error(f)) {
            return -EIO;
        }
    } while (!(flags & RAM_SAVE_FLAG_EOS));

    return 0;
}

void qemu_service_io(void)
{
    qemu_notify_event();
}

#ifdef HAS_AUDIO
struct soundhw soundhw[] = {
#ifdef HAS_AUDIO_CHOICE
#if defined(TARGET_I386) || defined(TARGET_MIPS)
    {
        "pcspk",
        "PC speaker",
        0,
        1,
        { .init_isa = pcspk_audio_init }
    },
#endif

#ifdef CONFIG_SB16
    {
        "sb16",
        "Creative Sound Blaster 16",
        0,
        1,
        { .init_isa = SB16_init }
    },
#endif

#ifdef CONFIG_CS4231A
    {
        "cs4231a",
        "CS4231A",
        0,
        1,
        { .init_isa = cs4231a_init }
    },
#endif

#ifdef CONFIG_ADLIB
    {
        "adlib",
#ifdef HAS_YMF262
        "Yamaha YMF262 (OPL3)",
#else
        "Yamaha YM3812 (OPL2)",
#endif
        0,
        1,
        { .init_isa = Adlib_init }
    },
#endif

#ifdef CONFIG_GUS
    {
        "gus",
        "Gravis Ultrasound GF1",
        0,
        1,
        { .init_isa = GUS_init }
    },
#endif

#ifdef CONFIG_AC97
    {
        "ac97",
        "Intel 82801AA AC97 Audio",
        0,
        0,
        { .init_pci = ac97_init }
    },
#endif

#ifdef CONFIG_ES1370
    {
        "es1370",
        "ENSONIQ AudioPCI ES1370",
        0,
        0,
        { .init_pci = es1370_init }
    },
#endif

#endif /* HAS_AUDIO_CHOICE */

    { NULL, NULL, 0, 0, { NULL } }
};

void select_soundhw(const char *optarg)
{
    struct soundhw *c;

    if (*optarg == '?') {
    show_valid_cards:

        printf("Valid sound card names (comma separated):\n");
        for (c = soundhw; c->name; ++c) {
            printf ("%-11s %s\n", c->name, c->descr);
        }
        printf("\n-soundhw all will enable all of the above\n");
        exit(*optarg != '?');
    }
    else {
        size_t l;
        const char *p;
        char *e;
        int bad_card = 0;

        if (!strcmp(optarg, "all")) {
            for (c = soundhw; c->name; ++c) {
                c->enabled = 1;
            }
            return;
        }

        p = optarg;
        while (*p) {
            e = strchr(p, ',');
            l = !e ? strlen(p) : (size_t) (e - p);

            for (c = soundhw; c->name; ++c) {
                if (!strncmp(c->name, p, l) && !c->name[l]) {
                    c->enabled = 1;
                    break;
                }
            }

            if (!c->name) {
                if (l > 80) {
                    fprintf(stderr,
                            "Unknown sound card name (too big to show)\n");
                }
                else {
                    fprintf(stderr, "Unknown sound card name `%.*s'\n",
                            (int) l, p);
                }
                bad_card = 1;
            }
            p += l + (e != NULL);
        }

        if (bad_card) {
            goto show_valid_cards;
        }
    }
}
#else
void select_soundhw(const char *optarg)
{
}
#endif

int qemu_uuid_parse(const char *str, uint8_t *uuid)
{
    int ret;

    if (strlen(str) != 36) {
        return -1;
    }

    ret = sscanf(str, UUID_FMT, &uuid[0], &uuid[1], &uuid[2], &uuid[3],
                 &uuid[4], &uuid[5], &uuid[6], &uuid[7], &uuid[8], &uuid[9],
                 &uuid[10], &uuid[11], &uuid[12], &uuid[13], &uuid[14],
                 &uuid[15]);

    if (ret != 16) {
        return -1;
    }
#ifdef TARGET_I386
    smbios_add_field(1, offsetof(struct smbios_type_1, uuid), 16, uuid);
#endif
    return 0;
}

void do_acpitable_option(const char *optarg)
{
#ifdef TARGET_I386
    if (acpi_table_add(optarg) < 0) {
        fprintf(stderr, "Wrong acpi table provided\n");
        exit(1);
    }
#endif
}

void do_smbios_option(const char *optarg)
{
#ifdef TARGET_I386
    if (smbios_entry_add(optarg) < 0) {
        fprintf(stderr, "Wrong smbios provided\n");
        exit(1);
    }
#endif
}

void cpudef_init(void)
{
#if defined(cpudef_setup)
    cpudef_setup(); /* parse cpu definitions in target config file */
#endif
}

int audio_available(void)
{
#ifdef HAS_AUDIO
    return 1;
#else
    return 0;
#endif
}

int kvm_available(void)
{
#ifdef CONFIG_KVM
    return 1;
#else
    return 0;
#endif
}

int xen_available(void)
{
#ifdef CONFIG_XEN
    return 1;
#else
    return 0;
#endif
}
