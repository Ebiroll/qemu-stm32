/*
 * QEMU monitor
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
#include "vl.h"
#include "disas.h"
#include <dirent.h>

//#define DEBUG
//#define DEBUG_COMPLETION

#ifndef offsetof
#define offsetof(type, field) ((size_t) &((type *)0)->field)
#endif

/*
 * Supported types:
 * 
 * 'F'          filename
 * 'B'          block device name
 * 's'          string (accept optional quote)
 * 'i'          integer
 * '/'          optional gdb-like print format (like "/10x")
 *
 * '?'          optional type (for 'F', 's' and 'i')
 *
 */

typedef struct term_cmd_t {
    const char *name;
    const char *args_type;
    void (*handler)();
    const char *params;
    const char *help;
} term_cmd_t;

static CharDriverState *monitor_hd;

static term_cmd_t term_cmds[];
static term_cmd_t info_cmds[];

static char term_outbuf[1024];
static int term_outbuf_index;

static void monitor_start_input(void);

void term_flush(void)
{
    if (term_outbuf_index > 0) {
        qemu_chr_write(monitor_hd, term_outbuf, term_outbuf_index);
        term_outbuf_index = 0;
    }
}

/* flush at every end of line or if the buffer is full */
void term_puts(const char *str)
{
    int c;
    for(;;) {
        c = *str++;
        if (c == '\0')
            break;
        term_outbuf[term_outbuf_index++] = c;
        if (term_outbuf_index >= sizeof(term_outbuf) ||
            c == '\n')
            term_flush();
    }
}

void term_vprintf(const char *fmt, va_list ap)
{
    char buf[4096];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    term_puts(buf);
}

void term_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    term_vprintf(fmt, ap);
    va_end(ap);
}

static int compare_cmd(const char *name, const char *list)
{
    const char *p, *pstart;
    int len;
    len = strlen(name);
    p = list;
    for(;;) {
        pstart = p;
        p = strchr(p, '|');
        if (!p)
            p = pstart + strlen(pstart);
        if ((p - pstart) == len && !memcmp(pstart, name, len))
            return 1;
        if (*p == '\0')
            break;
        p++;
    }
    return 0;
}

static void help_cmd1(term_cmd_t *cmds, const char *prefix, const char *name)
{
    term_cmd_t *cmd;

    for(cmd = cmds; cmd->name != NULL; cmd++) {
        if (!name || !strcmp(name, cmd->name))
            term_printf("%s%s %s -- %s\n", prefix, cmd->name, cmd->params, cmd->help);
    }
}

static void help_cmd(const char *name)
{
    if (name && !strcmp(name, "info")) {
        help_cmd1(info_cmds, "info ", NULL);
    } else {
        help_cmd1(term_cmds, "", name);
        if (name && !strcmp(name, "log")) {
            CPULogItem *item;
            term_printf("Log items (comma separated):\n");
            term_printf("%-10s %s\n", "none", "remove all logs");
            for(item = cpu_log_items; item->mask != 0; item++) {
                term_printf("%-10s %s\n", item->name, item->help);
            }
        }
    }
}

static void do_help(const char *name)
{
    help_cmd(name);
}

static void do_commit(void)
{
    int i;

    for (i = 0; i < MAX_DISKS; i++) {
        if (bs_table[i]) {
            bdrv_commit(bs_table[i]);
        }
    }
}

static void do_info(const char *item)
{
    term_cmd_t *cmd;

    if (!item)
        goto help;
    for(cmd = info_cmds; cmd->name != NULL; cmd++) {
        if (compare_cmd(item, cmd->name)) 
            goto found;
    }
 help:
    help_cmd("info");
    return;
 found:
    cmd->handler();
}

static void do_info_network(void)
{
    int i, j;
    NetDriverState *nd;
    
    for(i = 0; i < nb_nics; i++) {
        nd = &nd_table[i];
        term_printf("%d: ifname=%s macaddr=", i, nd->ifname);
        for(j = 0; j < 6; j++) {
            if (j > 0)
                term_printf(":");
            term_printf("%02x", nd->macaddr[j]);
        }
        term_printf("\n");
    }
}
 
static void do_info_block(void)
{
    bdrv_info();
}

static void do_info_registers(void)
{
#ifdef TARGET_I386
    cpu_dump_state(cpu_single_env, stdout, X86_DUMP_FPU | X86_DUMP_CCOP);
#else
    cpu_dump_state(cpu_single_env, stdout, 0);
#endif
}

static void do_info_history (void)
{
    int i;
    const char *str;
    
    i = 0;
    for(;;) {
        str = readline_get_history(i);
        if (!str)
            break;
	term_printf("%d: '%s'\n", i, str);
    }
}

static void do_quit(void)
{
    exit(0);
}

static int eject_device(BlockDriverState *bs, int force)
{
    if (bdrv_is_inserted(bs)) {
        if (!force) {
            if (!bdrv_is_removable(bs)) {
                term_printf("device is not removable\n");
                return -1;
            }
            if (bdrv_is_locked(bs)) {
                term_printf("device is locked\n");
                return -1;
            }
        }
        bdrv_close(bs);
    }
    return 0;
}

static void do_eject(int force, const char *filename)
{
    BlockDriverState *bs;

    bs = bdrv_find(filename);
    if (!bs) {
        term_printf("device not found\n");
        return;
    }
    eject_device(bs, force);
}

static void do_change(const char *device, const char *filename)
{
    BlockDriverState *bs;
    int i;
    char password[256];

    bs = bdrv_find(device);
    if (!bs) {
        term_printf("device not found\n");
        return;
    }
    if (eject_device(bs, 0) < 0)
        return;
    bdrv_open(bs, filename, 0);
    if (bdrv_is_encrypted(bs)) {
        term_printf("%s is encrypted.\n", device);
        for(i = 0; i < 3; i++) {
            monitor_readline("Password: ", 1, password, sizeof(password));
            if (bdrv_set_key(bs, password) == 0)
                break;
            term_printf("invalid password\n");
        }
    }
}

static void do_screen_dump(const char *filename)
{
    vga_screen_dump(filename);
}

static void do_log(const char *items)
{
    int mask;
    
    if (!strcmp(items, "none")) {
        mask = 0;
    } else {
        mask = cpu_str_to_log_mask(items);
        if (!mask) {
            help_cmd("log");
            return;
        }
    }
    cpu_set_log(mask);
}

static void do_savevm(const char *filename)
{
    if (qemu_savevm(filename) < 0)
        term_printf("I/O error when saving VM to '%s'\n", filename);
}

static void do_loadvm(const char *filename)
{
    if (qemu_loadvm(filename) < 0) 
        term_printf("I/O error when loading VM from '%s'\n", filename);
}

static void do_stop(void)
{
    vm_stop(EXCP_INTERRUPT);
}

static void do_cont(void)
{
    vm_start();
}

#ifdef CONFIG_GDBSTUB
static void do_gdbserver(int has_port, int port)
{
    if (!has_port)
        port = DEFAULT_GDBSTUB_PORT;
    if (gdbserver_start(port) < 0) {
        qemu_printf("Could not open gdbserver socket on port %d\n", port);
    } else {
        qemu_printf("Waiting gdb connection on port %d\n", port);
    }
}
#endif

static void term_printc(int c)
{
    term_printf("'");
    switch(c) {
    case '\'':
        term_printf("\\'");
        break;
    case '\\':
        term_printf("\\\\");
        break;
    case '\n':
        term_printf("\\n");
        break;
    case '\r':
        term_printf("\\r");
        break;
    default:
        if (c >= 32 && c <= 126) {
            term_printf("%c", c);
        } else {
            term_printf("\\x%02x", c);
        }
        break;
    }
    term_printf("'");
}

static void memory_dump(int count, int format, int wsize, 
                        target_ulong addr, int is_physical)
{
    int nb_per_line, l, line_size, i, max_digits, len;
    uint8_t buf[16];
    uint64_t v;

    if (format == 'i') {
        int flags;
        flags = 0;
#ifdef TARGET_I386
        if (wsize == 2) {
            flags = 1;
        } else if (wsize == 4) {
            flags = 0;
        } else {
            /* as default we use the current CS size */
            flags = 0;
            if (!(cpu_single_env->segs[R_CS].flags & DESC_B_MASK))
                flags = 1;
        }
#endif
        monitor_disas(addr, count, is_physical, flags);
        return;
    }

    len = wsize * count;
    if (wsize == 1)
        line_size = 8;
    else
        line_size = 16;
    nb_per_line = line_size / wsize;
    max_digits = 0;

    switch(format) {
    case 'o':
        max_digits = (wsize * 8 + 2) / 3;
        break;
    default:
    case 'x':
        max_digits = (wsize * 8) / 4;
        break;
    case 'u':
    case 'd':
        max_digits = (wsize * 8 * 10 + 32) / 33;
        break;
    case 'c':
        wsize = 1;
        break;
    }

    while (len > 0) {
        term_printf("0x%08x:", addr);
        l = len;
        if (l > line_size)
            l = line_size;
        if (is_physical) {
            cpu_physical_memory_rw(addr, buf, l, 0);
        } else {
            cpu_memory_rw_debug(cpu_single_env, addr, buf, l, 0);
        }
        i = 0; 
        while (i < l) {
            switch(wsize) {
            default:
            case 1:
                v = ldub_raw(buf + i);
                break;
            case 2:
                v = lduw_raw(buf + i);
                break;
            case 4:
                v = ldl_raw(buf + i);
                break;
            case 8:
                v = ldq_raw(buf + i);
                break;
            }
            term_printf(" ");
            switch(format) {
            case 'o':
                term_printf("%#*llo", max_digits, v);
                break;
            case 'x':
                term_printf("0x%0*llx", max_digits, v);
                break;
            case 'u':
                term_printf("%*llu", max_digits, v);
                break;
            case 'd':
                term_printf("%*lld", max_digits, v);
                break;
            case 'c':
                term_printc(v);
                break;
            }
            i += wsize;
        }
        term_printf("\n");
        addr += l;
        len -= l;
    }
}

static void do_memory_dump(int count, int format, int size, int addr)
{
    memory_dump(count, format, size, addr, 0);
}

static void do_physical_memory_dump(int count, int format, int size, int addr)
{
    memory_dump(count, format, size, addr, 1);
}

static void do_print(int count, int format, int size, int val)
{
    switch(format) {
    case 'o':
        term_printf("%#o", val);
        break;
    case 'x':
        term_printf("%#x", val);
        break;
    case 'u':
        term_printf("%u", val);
        break;
    default:
    case 'd':
        term_printf("%d", val);
        break;
    case 'c':
        term_printc(val);
        break;
    }
    term_printf("\n");
}

typedef struct {
    int keycode;
    const char *name;
} KeyDef;

static const KeyDef key_defs[] = {
    { 0x2a, "shift" },
    { 0x36, "shift_r" },
    
    { 0x38, "alt" },
    { 0xb8, "alt_r" },
    { 0x1d, "ctrl" },
    { 0x9d, "ctrl_r" },

    { 0xdd, "menu" },

    { 0x01, "esc" },

    { 0x02, "1" },
    { 0x03, "2" },
    { 0x04, "3" },
    { 0x05, "4" },
    { 0x06, "5" },
    { 0x07, "6" },
    { 0x08, "7" },
    { 0x09, "8" },
    { 0x0a, "9" },
    { 0x0b, "0" },
    { 0x0e, "backspace" },

    { 0x0f, "tab" },
    { 0x10, "q" },
    { 0x11, "w" },
    { 0x12, "e" },
    { 0x13, "r" },
    { 0x14, "t" },
    { 0x15, "y" },
    { 0x16, "u" },
    { 0x17, "i" },
    { 0x18, "o" },
    { 0x19, "p" },

    { 0x1c, "ret" },

    { 0x1e, "a" },
    { 0x1f, "s" },
    { 0x20, "d" },
    { 0x21, "f" },
    { 0x22, "g" },
    { 0x23, "h" },
    { 0x24, "j" },
    { 0x25, "k" },
    { 0x26, "l" },

    { 0x2c, "z" },
    { 0x2d, "x" },
    { 0x2e, "c" },
    { 0x2f, "v" },
    { 0x30, "b" },
    { 0x31, "n" },
    { 0x32, "m" },
    
    { 0x39, "spc" },
    { 0x3a, "caps_lock" },
    { 0x3b, "f1" },
    { 0x3c, "f2" },
    { 0x3d, "f3" },
    { 0x3e, "f4" },
    { 0x3f, "f5" },
    { 0x40, "f6" },
    { 0x41, "f7" },
    { 0x42, "f8" },
    { 0x43, "f9" },
    { 0x44, "f10" },
    { 0x45, "num_lock" },
    { 0x46, "scroll_lock" },

    { 0x56, "<" },

    { 0x57, "f11" },
    { 0x58, "f12" },

    { 0xb7, "print" },

    { 0xc7, "home" },
    { 0xc9, "pgup" },
    { 0xd1, "pgdn" },
    { 0xcf, "end" },

    { 0xcb, "left" },
    { 0xc8, "up" },
    { 0xd0, "down" },
    { 0xcd, "right" },

    { 0xd2, "insert" },
    { 0xd3, "delete" },
    { 0, NULL },
};

static int get_keycode(const char *key)
{
    const KeyDef *p;

    for(p = key_defs; p->name != NULL; p++) {
        if (!strcmp(key, p->name))
            return p->keycode;
    }
    return -1;
}

static void do_send_key(const char *string)
{
    char keybuf[16], *q;
    uint8_t keycodes[16];
    const char *p;
    int nb_keycodes, keycode, i;
    
    nb_keycodes = 0;
    p = string;
    while (*p != '\0') {
        q = keybuf;
        while (*p != '\0' && *p != '-') {
            if ((q - keybuf) < sizeof(keybuf) - 1) {
                *q++ = *p;
            }
            p++;
        }
        *q = '\0';
        keycode = get_keycode(keybuf);
        if (keycode < 0) {
            term_printf("unknown key: '%s'\n", keybuf);
            return;
        }
        keycodes[nb_keycodes++] = keycode;
        if (*p == '\0')
            break;
        p++;
    }
    /* key down events */
    for(i = 0; i < nb_keycodes; i++) {
        keycode = keycodes[i];
        if (keycode & 0x80)
            kbd_put_keycode(0xe0);
        kbd_put_keycode(keycode & 0x7f);
    }
    /* key up events */
    for(i = nb_keycodes - 1; i >= 0; i--) {
        keycode = keycodes[i];
        if (keycode & 0x80)
            kbd_put_keycode(0xe0);
        kbd_put_keycode(keycode | 0x80);
    }
}

static void do_ioport_read(int count, int format, int size, int addr, int has_index, int index)
{
    uint32_t val;
    int suffix;

    if (has_index) {
        cpu_outb(NULL, addr & 0xffff, index & 0xff);
        addr++;
    }
    addr &= 0xffff;

    switch(size) {
    default:
    case 1:
        val = cpu_inb(NULL, addr);
        suffix = 'b';
        break;
    case 2:
        val = cpu_inw(NULL, addr);
        suffix = 'w';
        break;
    case 4:
        val = cpu_inl(NULL, addr);
        suffix = 'l';
        break;
    }
    term_printf("port%c[0x%04x] = %#0*x\n",
                suffix, addr, size * 2, val);
}

static void do_system_reset(void)
{
    qemu_system_reset_request();
}

static term_cmd_t term_cmds[] = {
    { "help|?", "s?", do_help, 
      "[cmd]", "show the help" },
    { "commit", "", do_commit, 
      "", "commit changes to the disk images (if -snapshot is used)" },
    { "info", "s?", do_info,
      "subcommand", "show various information about the system state" },
    { "q|quit", "", do_quit,
      "", "quit the emulator" },
    { "eject", "-fB", do_eject,
      "[-f] device", "eject a removable media (use -f to force it)" },
    { "change", "BF", do_change,
      "device filename", "change a removable media" },
    { "screendump", "F", do_screen_dump, 
      "filename", "save screen into PPM image 'filename'" },
    { "log", "s", do_log,
      "item1[,...]", "activate logging of the specified items to '/tmp/qemu.log'" }, 
    { "savevm", "F", do_savevm,
      "filename", "save the whole virtual machine state to 'filename'" }, 
    { "loadvm", "F", do_loadvm,
      "filename", "restore the whole virtual machine state from 'filename'" }, 
    { "stop", "", do_stop, 
      "", "stop emulation", },
    { "c|cont", "", do_cont, 
      "", "resume emulation", },
#ifdef CONFIG_GDBSTUB
    { "gdbserver", "i?", do_gdbserver, 
      "[port]", "start gdbserver session (default port=1234)", },
#endif
    { "x", "/i", do_memory_dump, 
      "/fmt addr", "virtual memory dump starting at 'addr'", },
    { "xp", "/i", do_physical_memory_dump, 
      "/fmt addr", "physical memory dump starting at 'addr'", },
    { "p|print", "/i", do_print, 
      "/fmt expr", "print expression value (use $reg for CPU register access)", },
    { "i", "/ii.", do_ioport_read, 
      "/fmt addr", "I/O port read" },

    { "sendkey", "s", do_send_key, 
      "keys", "send keys to the VM (e.g. 'sendkey ctrl-alt-f1')" },
    { "system_reset", "", do_system_reset, 
      "", "reset the system" },
    { NULL, NULL, }, 
};

static term_cmd_t info_cmds[] = {
    { "network", "", do_info_network,
      "", "show the network state" },
    { "block", "", do_info_block,
      "", "show the block devices" },
    { "registers", "", do_info_registers,
      "", "show the cpu registers" },
    { "history", "", do_info_history,
      "", "show the command line history", },
    { "irq", "", irq_info,
      "", "show the interrupts statistics (if available)", },
    { "pic", "", pic_info,
      "", "show i8259 (PIC) state", },
    { "pci", "", pci_info,
      "", "show PCI info", },
    { NULL, NULL, },
};

/*******************************************************************/

static const char *pch;
static jmp_buf expr_env;

typedef struct MonitorDef {
    const char *name;
    int offset;
    int (*get_value)(struct MonitorDef *md);
} MonitorDef;

#if defined(TARGET_I386)
static int monitor_get_pc (struct MonitorDef *md)
{
    return cpu_single_env->eip + (long)cpu_single_env->segs[R_CS].base;
}
#endif

#if defined(TARGET_PPC)
static int monitor_get_ccr (struct MonitorDef *md)
{
    unsigned int u;
    int i;

    u = 0;
    for (i = 0; i < 8; i++)
	u |= cpu_single_env->crf[i] << (32 - (4 * i));

    return u;
}

static int monitor_get_msr (struct MonitorDef *md)
{
    return (cpu_single_env->msr[MSR_POW] << MSR_POW) |
        (cpu_single_env->msr[MSR_ILE] << MSR_ILE) |
        (cpu_single_env->msr[MSR_EE] << MSR_EE) |
        (cpu_single_env->msr[MSR_PR] << MSR_PR) |
        (cpu_single_env->msr[MSR_FP] << MSR_FP) |
        (cpu_single_env->msr[MSR_ME] << MSR_ME) |
        (cpu_single_env->msr[MSR_FE0] << MSR_FE0) |
        (cpu_single_env->msr[MSR_SE] << MSR_SE) |
        (cpu_single_env->msr[MSR_BE] << MSR_BE) |
        (cpu_single_env->msr[MSR_FE1] << MSR_FE1) |
        (cpu_single_env->msr[MSR_IP] << MSR_IP) |
        (cpu_single_env->msr[MSR_IR] << MSR_IR) |
        (cpu_single_env->msr[MSR_DR] << MSR_DR) |
        (cpu_single_env->msr[MSR_RI] << MSR_RI) |
        (cpu_single_env->msr[MSR_LE] << MSR_LE);
}

static int monitor_get_xer (struct MonitorDef *md)
{
    return (cpu_single_env->xer[XER_SO] << XER_SO) |
        (cpu_single_env->xer[XER_OV] << XER_OV) |
        (cpu_single_env->xer[XER_CA] << XER_CA) |
        (cpu_single_env->xer[XER_BC] << XER_BC);
}

uint32_t cpu_ppc_load_decr (CPUState *env);
static int monitor_get_decr (struct MonitorDef *md)
{
    return cpu_ppc_load_decr(cpu_single_env);
}

uint32_t cpu_ppc_load_tbu (CPUState *env);
static int monitor_get_tbu (struct MonitorDef *md)
{
    return cpu_ppc_load_tbu(cpu_single_env);
}

uint32_t cpu_ppc_load_tbl (CPUState *env);
static int monitor_get_tbl (struct MonitorDef *md)
{
    return cpu_ppc_load_tbl(cpu_single_env);
}
#endif

static MonitorDef monitor_defs[] = {
#ifdef TARGET_I386

#define SEG(name, seg) \
    { name, offsetof(CPUState, segs[seg].selector) },\
    { name ".base", offsetof(CPUState, segs[seg].base) },\
    { name ".limit", offsetof(CPUState, segs[seg].limit) },

    { "eax", offsetof(CPUState, regs[0]) },
    { "ecx", offsetof(CPUState, regs[1]) },
    { "edx", offsetof(CPUState, regs[2]) },
    { "ebx", offsetof(CPUState, regs[3]) },
    { "esp|sp", offsetof(CPUState, regs[4]) },
    { "ebp|fp", offsetof(CPUState, regs[5]) },
    { "esi", offsetof(CPUState, regs[6]) },
    { "edi", offsetof(CPUState, regs[7]) },
    { "eflags", offsetof(CPUState, eflags) },
    { "eip", offsetof(CPUState, eip) },
    SEG("cs", R_CS)
    SEG("ds", R_DS)
    SEG("es", R_ES)
    SEG("ss", R_SS)
    SEG("fs", R_FS)
    SEG("gs", R_GS)
    { "pc", 0, monitor_get_pc, },
#elif defined(TARGET_PPC)
    { "r0", offsetof(CPUState, gpr[0]) },
    { "r1", offsetof(CPUState, gpr[1]) },
    { "r2", offsetof(CPUState, gpr[2]) },
    { "r3", offsetof(CPUState, gpr[3]) },
    { "r4", offsetof(CPUState, gpr[4]) },
    { "r5", offsetof(CPUState, gpr[5]) },
    { "r6", offsetof(CPUState, gpr[6]) },
    { "r7", offsetof(CPUState, gpr[7]) },
    { "r8", offsetof(CPUState, gpr[8]) },
    { "r9", offsetof(CPUState, gpr[9]) },
    { "r10", offsetof(CPUState, gpr[10]) },
    { "r11", offsetof(CPUState, gpr[11]) },
    { "r12", offsetof(CPUState, gpr[12]) },
    { "r13", offsetof(CPUState, gpr[13]) },
    { "r14", offsetof(CPUState, gpr[14]) },
    { "r15", offsetof(CPUState, gpr[15]) },
    { "r16", offsetof(CPUState, gpr[16]) },
    { "r17", offsetof(CPUState, gpr[17]) },
    { "r18", offsetof(CPUState, gpr[18]) },
    { "r19", offsetof(CPUState, gpr[19]) },
    { "r20", offsetof(CPUState, gpr[20]) },
    { "r21", offsetof(CPUState, gpr[21]) },
    { "r22", offsetof(CPUState, gpr[22]) },
    { "r23", offsetof(CPUState, gpr[23]) },
    { "r24", offsetof(CPUState, gpr[24]) },
    { "r25", offsetof(CPUState, gpr[25]) },
    { "r26", offsetof(CPUState, gpr[26]) },
    { "r27", offsetof(CPUState, gpr[27]) },
    { "r28", offsetof(CPUState, gpr[28]) },
    { "r29", offsetof(CPUState, gpr[29]) },
    { "r30", offsetof(CPUState, gpr[30]) },
    { "r31", offsetof(CPUState, gpr[31]) },
    { "nip|pc", offsetof(CPUState, nip) },
    { "lr", offsetof(CPUState, lr) },
    { "ctr", offsetof(CPUState, ctr) },
    { "decr", 0, &monitor_get_decr, },
    { "ccr", 0, &monitor_get_ccr, },
    { "msr", 0, &monitor_get_msr, },
    { "xer", 0, &monitor_get_xer, },
    { "tbu", 0, &monitor_get_tbu, },
    { "tbl", 0, &monitor_get_tbl, },
    { "sdr1", offsetof(CPUState, sdr1) },
    { "sr0", offsetof(CPUState, sr[0]) },
    { "sr1", offsetof(CPUState, sr[1]) },
    { "sr2", offsetof(CPUState, sr[2]) },
    { "sr3", offsetof(CPUState, sr[3]) },
    { "sr4", offsetof(CPUState, sr[4]) },
    { "sr5", offsetof(CPUState, sr[5]) },
    { "sr6", offsetof(CPUState, sr[6]) },
    { "sr7", offsetof(CPUState, sr[7]) },
    { "sr8", offsetof(CPUState, sr[8]) },
    { "sr9", offsetof(CPUState, sr[9]) },
    { "sr10", offsetof(CPUState, sr[10]) },
    { "sr11", offsetof(CPUState, sr[11]) },
    { "sr12", offsetof(CPUState, sr[12]) },
    { "sr13", offsetof(CPUState, sr[13]) },
    { "sr14", offsetof(CPUState, sr[14]) },
    { "sr15", offsetof(CPUState, sr[15]) },
    /* Too lazy to put BATs and SPRs ... */
#endif
    { NULL },
};

static void expr_error(const char *fmt) 
{
    term_printf(fmt);
    term_printf("\n");
    longjmp(expr_env, 1);
}

static int get_monitor_def(int *pval, const char *name)
{
    MonitorDef *md;
    for(md = monitor_defs; md->name != NULL; md++) {
        if (compare_cmd(name, md->name)) {
            if (md->get_value) {
                *pval = md->get_value(md);
            } else {
                *pval = *(uint32_t *)((uint8_t *)cpu_single_env + md->offset);
            }
            return 0;
        }
    }
    return -1;
}

static void next(void)
{
    if (pch != '\0') {
        pch++;
        while (isspace(*pch))
            pch++;
    }
}

static int expr_sum(void);

static int expr_unary(void)
{
    int n;
    char *p;

    switch(*pch) {
    case '+':
        next();
        n = expr_unary();
        break;
    case '-':
        next();
        n = -expr_unary();
        break;
    case '~':
        next();
        n = ~expr_unary();
        break;
    case '(':
        next();
        n = expr_sum();
        if (*pch != ')') {
            expr_error("')' expected");
        }
        next();
        break;
    case '\'':
        pch++;
        if (*pch == '\0')
            expr_error("character constant expected");
        n = *pch;
        pch++;
        if (*pch != '\'')
            expr_error("missing terminating \' character");
        next();
        break;
    case '$':
        {
            char buf[128], *q;
            
            pch++;
            q = buf;
            while ((*pch >= 'a' && *pch <= 'z') ||
                   (*pch >= 'A' && *pch <= 'Z') ||
                   (*pch >= '0' && *pch <= '9') ||
                   *pch == '_' || *pch == '.') {
                if ((q - buf) < sizeof(buf) - 1)
                    *q++ = *pch;
                pch++;
            }
            while (isspace(*pch))
                pch++;
            *q = 0;
            if (get_monitor_def(&n, buf))
                expr_error("unknown register");
        }
        break;
    case '\0':
        expr_error("unexpected end of expression");
        n = 0;
        break;
    default:
        n = strtoul(pch, &p, 0);
        if (pch == p) {
            expr_error("invalid char in expression");
        }
        pch = p;
        while (isspace(*pch))
            pch++;
        break;
    }
    return n;
}


static int expr_prod(void)
{
    int val, val2, op;

    val = expr_unary();
    for(;;) {
        op = *pch;
        if (op != '*' && op != '/' && op != '%')
            break;
        next();
        val2 = expr_unary();
        switch(op) {
        default:
        case '*':
            val *= val2;
            break;
        case '/':
        case '%':
            if (val2 == 0) 
                expr_error("division by zero");
            if (op == '/')
                val /= val2;
            else
                val %= val2;
            break;
        }
    }
    return val;
}

static int expr_logic(void)
{
    int val, val2, op;

    val = expr_prod();
    for(;;) {
        op = *pch;
        if (op != '&' && op != '|' && op != '^')
            break;
        next();
        val2 = expr_prod();
        switch(op) {
        default:
        case '&':
            val &= val2;
            break;
        case '|':
            val |= val2;
            break;
        case '^':
            val ^= val2;
            break;
        }
    }
    return val;
}

static int expr_sum(void)
{
    int val, val2, op;

    val = expr_logic();
    for(;;) {
        op = *pch;
        if (op != '+' && op != '-')
            break;
        next();
        val2 = expr_logic();
        if (op == '+')
            val += val2;
        else
            val -= val2;
    }
    return val;
}

static int get_expr(int *pval, const char **pp)
{
    pch = *pp;
    if (setjmp(expr_env)) {
        *pp = pch;
        return -1;
    }
    while (isspace(*pch))
        pch++;
    *pval = expr_sum();
    *pp = pch;
    return 0;
}

static int get_str(char *buf, int buf_size, const char **pp)
{
    const char *p;
    char *q;
    int c;

    q = buf;
    p = *pp;
    while (isspace(*p))
        p++;
    if (*p == '\0') {
    fail:
        *q = '\0';
        *pp = p;
        return -1;
    }
    if (*p == '\"') {
        p++;
        while (*p != '\0' && *p != '\"') {
            if (*p == '\\') {
                p++;
                c = *p++;
                switch(c) {
                case 'n':
                    c = '\n';
                    break;
                case 'r':
                    c = '\r';
                    break;
                case '\\':
                case '\'':
                case '\"':
                    break;
                default:
                    qemu_printf("unsupported escape code: '\\%c'\n", c);
                    goto fail;
                }
                if ((q - buf) < buf_size - 1) {
                    *q++ = c;
                }
            } else {
                if ((q - buf) < buf_size - 1) {
                    *q++ = *p;
                }
                p++;
            }
        }
        if (*p != '\"') {
            qemu_printf("unterminated string\n");
            goto fail;
        }
        p++;
    } else {
        while (*p != '\0' && !isspace(*p)) {
            if ((q - buf) < buf_size - 1) {
                *q++ = *p;
            }
            p++;
        }
    }
    *q = '\0';
    *pp = p;
    return 0;
}

static int default_fmt_format = 'x';
static int default_fmt_size = 4;

#define MAX_ARGS 16

static void monitor_handle_command(const char *cmdline)
{
    const char *p, *pstart, *typestr;
    char *q;
    int c, nb_args, len, i, has_arg;
    term_cmd_t *cmd;
    char cmdname[256];
    char buf[1024];
    void *str_allocated[MAX_ARGS];
    void *args[MAX_ARGS];

#ifdef DEBUG
    term_printf("command='%s'\n", cmdline);
#endif
    
    /* extract the command name */
    p = cmdline;
    q = cmdname;
    while (isspace(*p))
        p++;
    if (*p == '\0')
        return;
    pstart = p;
    while (*p != '\0' && *p != '/' && !isspace(*p))
        p++;
    len = p - pstart;
    if (len > sizeof(cmdname) - 1)
        len = sizeof(cmdname) - 1;
    memcpy(cmdname, pstart, len);
    cmdname[len] = '\0';
    
    /* find the command */
    for(cmd = term_cmds; cmd->name != NULL; cmd++) {
        if (compare_cmd(cmdname, cmd->name)) 
            goto found;
    }
    term_printf("unknown command: '%s'\n", cmdname);
    return;
 found:

    for(i = 0; i < MAX_ARGS; i++)
        str_allocated[i] = NULL;
    
    /* parse the parameters */
    typestr = cmd->args_type;
    nb_args = 0;
    for(;;) {
        c = *typestr;
        if (c == '\0')
            break;
        typestr++;
        switch(c) {
        case 'F':
        case 'B':
        case 's':
            {
                int ret;
                char *str;
                
                while (isspace(*p)) 
                    p++;
                if (*typestr == '?') {
                    typestr++;
                    if (*p == '\0') {
                        /* no optional string: NULL argument */
                        str = NULL;
                        goto add_str;
                    }
                }
                ret = get_str(buf, sizeof(buf), &p);
                if (ret < 0) {
                    switch(c) {
                    case 'F':
                        term_printf("%s: filename expected\n", cmdname);
                        break;
                    case 'B':
                        term_printf("%s: block device name expected\n", cmdname);
                        break;
                    default:
                        term_printf("%s: string expected\n", cmdname);
                        break;
                    }
                    goto fail;
                }
                str = qemu_malloc(strlen(buf) + 1);
                strcpy(str, buf);
                str_allocated[nb_args] = str;
            add_str:
                if (nb_args >= MAX_ARGS) {
                error_args:
                    term_printf("%s: too many arguments\n", cmdname);
                    goto fail;
                }
                args[nb_args++] = str;
            }
            break;
        case '/':
            {
                int count, format, size;
                
                while (isspace(*p))
                    p++;
                if (*p == '/') {
                    /* format found */
                    p++;
                    count = 1;
                    if (isdigit(*p)) {
                        count = 0;
                        while (isdigit(*p)) {
                            count = count * 10 + (*p - '0');
                            p++;
                        }
                    }
                    size = -1;
                    format = -1;
                    for(;;) {
                        switch(*p) {
                        case 'o':
                        case 'd':
                        case 'u':
                        case 'x':
                        case 'i':
                        case 'c':
                            format = *p++;
                            break;
                        case 'b':
                            size = 1;
                            p++;
                            break;
                        case 'h':
                            size = 2;
                            p++;
                            break;
                        case 'w':
                            size = 4;
                            p++;
                            break;
                        case 'g':
                        case 'L':
                            size = 8;
                            p++;
                            break;
                        default:
                            goto next;
                        }
                    }
                next:
                    if (*p != '\0' && !isspace(*p)) {
                        term_printf("invalid char in format: '%c'\n", *p);
                        goto fail;
                    }
                    if (format < 0)
                        format = default_fmt_format;
                    if (format != 'i') {
                        /* for 'i', not specifying a size gives -1 as size */
                        if (size < 0)
                            size = default_fmt_size;
                    }
                    default_fmt_size = size;
                    default_fmt_format = format;
                } else {
                    count = 1;
                    format = default_fmt_format;
                    if (format != 'i') {
                        size = default_fmt_size;
                    } else {
                        size = -1;
                    }
                }
                if (nb_args + 3 > MAX_ARGS)
                    goto error_args;
                args[nb_args++] = (void*)count;
                args[nb_args++] = (void*)format;
                args[nb_args++] = (void*)size;
            }
            break;
        case 'i':
            {
                int val;
                while (isspace(*p)) 
                    p++;
                if (*typestr == '?' || *typestr == '.') {
                    typestr++;
                    if (*typestr == '?') {
                        if (*p == '\0')
                            has_arg = 0;
                        else
                            has_arg = 1;
                    } else {
                        if (*p == '.') {
                            p++;
                            while (isspace(*p)) 
                                p++;
                            has_arg = 1;
                        } else {
                            has_arg = 0;
                        }
                    }
                    if (nb_args >= MAX_ARGS)
                        goto error_args;
                    args[nb_args++] = (void *)has_arg;
                    if (!has_arg) {
                        if (nb_args >= MAX_ARGS)
                            goto error_args;
                        val = -1;
                        goto add_num;
                    }
                }
                if (get_expr(&val, &p))
                    goto fail;
            add_num:
                if (nb_args >= MAX_ARGS)
                    goto error_args;
                args[nb_args++] = (void *)val;
            }
            break;
        case '-':
            {
                int has_option;
                /* option */
                
                c = *typestr++;
                if (c == '\0')
                    goto bad_type;
                while (isspace(*p)) 
                    p++;
                has_option = 0;
                if (*p == '-') {
                    p++;
                    if (*p != c) {
                        term_printf("%s: unsupported option -%c\n", 
                                    cmdname, *p);
                        goto fail;
                    }
                    p++;
                    has_option = 1;
                }
                if (nb_args >= MAX_ARGS)
                    goto error_args;
                args[nb_args++] = (void *)has_option;
            }
            break;
        default:
        bad_type:
            term_printf("%s: unknown type '%c'\n", cmdname, c);
            goto fail;
        }
    }
    /* check that all arguments were parsed */
    while (isspace(*p))
        p++;
    if (*p != '\0') {
        term_printf("%s: extraneous characters at the end of line\n", 
                    cmdname);
        goto fail;
    }

    switch(nb_args) {
    case 0:
        cmd->handler();
        break;
    case 1:
        cmd->handler(args[0]);
        break;
    case 2:
        cmd->handler(args[0], args[1]);
        break;
    case 3:
        cmd->handler(args[0], args[1], args[2]);
        break;
    case 4:
        cmd->handler(args[0], args[1], args[2], args[3]);
        break;
    case 5:
        cmd->handler(args[0], args[1], args[2], args[3], args[4]);
        break;
    case 6:
        cmd->handler(args[0], args[1], args[2], args[3], args[4], args[5]);
        break;
    default:
        term_printf("unsupported number of arguments: %d\n", nb_args);
        goto fail;
    }
 fail:
    for(i = 0; i < MAX_ARGS; i++)
        qemu_free(str_allocated[i]);
    return;
}

static void cmd_completion(const char *name, const char *list)
{
    const char *p, *pstart;
    char cmd[128];
    int len;

    p = list;
    for(;;) {
        pstart = p;
        p = strchr(p, '|');
        if (!p)
            p = pstart + strlen(pstart);
        len = p - pstart;
        if (len > sizeof(cmd) - 2)
            len = sizeof(cmd) - 2;
        memcpy(cmd, pstart, len);
        cmd[len] = '\0';
        if (name[0] == '\0' || !strncmp(name, cmd, strlen(name))) {
            add_completion(cmd);
        }
        if (*p == '\0')
            break;
        p++;
    }
}

static void file_completion(const char *input)
{
    DIR *ffs;
    struct dirent *d;
    char path[1024];
    char file[1024], file_prefix[1024];
    int input_path_len;
    const char *p;

    p = strrchr(input, '/'); 
    if (!p) {
        input_path_len = 0;
        pstrcpy(file_prefix, sizeof(file_prefix), input);
        strcpy(path, ".");
    } else {
        input_path_len = p - input + 1;
        memcpy(path, input, input_path_len);
        if (input_path_len > sizeof(path) - 1)
            input_path_len = sizeof(path) - 1;
        path[input_path_len] = '\0';
        pstrcpy(file_prefix, sizeof(file_prefix), p + 1);
    }
#ifdef DEBUG_COMPLETION
    term_printf("input='%s' path='%s' prefix='%s'\n", input, path, file_prefix);
#endif
    ffs = opendir(path);
    if (!ffs)
        return;
    for(;;) {
        struct stat sb;
        d = readdir(ffs);
        if (!d)
            break;
        if (strstart(d->d_name, file_prefix, NULL)) {
            memcpy(file, input, input_path_len);
            strcpy(file + input_path_len, d->d_name);
            /* stat the file to find out if it's a directory.
             * In that case add a slash to speed up typing long paths
             */
            stat(file, &sb);
            if(S_ISDIR(sb.st_mode))
                strcat(file, "/");
            add_completion(file);
        }
    }
    closedir(ffs);
}

static void block_completion_it(void *opaque, const char *name)
{
    const char *input = opaque;

    if (input[0] == '\0' ||
        !strncmp(name, (char *)input, strlen(input))) {
        add_completion(name);
    }
}

/* NOTE: this parser is an approximate form of the real command parser */
static void parse_cmdline(const char *cmdline,
                         int *pnb_args, char **args)
{
    const char *p;
    int nb_args, ret;
    char buf[1024];

    p = cmdline;
    nb_args = 0;
    for(;;) {
        while (isspace(*p))
            p++;
        if (*p == '\0')
            break;
        if (nb_args >= MAX_ARGS)
            break;
        ret = get_str(buf, sizeof(buf), &p);
        args[nb_args] = qemu_strdup(buf);
        nb_args++;
        if (ret < 0)
            break;
    }
    *pnb_args = nb_args;
}

void readline_find_completion(const char *cmdline)
{
    const char *cmdname;
    char *args[MAX_ARGS];
    int nb_args, i, len;
    const char *ptype, *str;
    term_cmd_t *cmd;

    parse_cmdline(cmdline, &nb_args, args);
#ifdef DEBUG_COMPLETION
    for(i = 0; i < nb_args; i++) {
        term_printf("arg%d = '%s'\n", i, (char *)args[i]);
    }
#endif

    /* if the line ends with a space, it means we want to complete the
       next arg */
    len = strlen(cmdline);
    if (len > 0 && isspace(cmdline[len - 1])) {
        if (nb_args >= MAX_ARGS)
            return;
        args[nb_args++] = qemu_strdup("");
    }
    if (nb_args <= 1) {
        /* command completion */
        if (nb_args == 0)
            cmdname = "";
        else
            cmdname = args[0];
        completion_index = strlen(cmdname);
        for(cmd = term_cmds; cmd->name != NULL; cmd++) {
            cmd_completion(cmdname, cmd->name);
        }
    } else {
        /* find the command */
        for(cmd = term_cmds; cmd->name != NULL; cmd++) {
            if (compare_cmd(args[0], cmd->name))
                goto found;
        }
        return;
    found:
        ptype = cmd->args_type;
        for(i = 0; i < nb_args - 2; i++) {
            if (*ptype != '\0') {
                ptype++;
                while (*ptype == '?')
                    ptype++;
            }
        }
        str = args[nb_args - 1];
        switch(*ptype) {
        case 'F':
            /* file completion */
            completion_index = strlen(str);
            file_completion(str);
            break;
        case 'B':
            /* block device name completion */
            completion_index = strlen(str);
            bdrv_iterate(block_completion_it, (void *)str);
            break;
        default:
            break;
        }
    }
    for(i = 0; i < nb_args; i++)
        qemu_free(args[i]);
}

static int term_can_read(void *opaque)
{
    return 128;
}

static void term_read(void *opaque, const uint8_t *buf, int size)
{
    int i;
    for(i = 0; i < size; i++)
        readline_handle_byte(buf[i]);
}

static void monitor_start_input(void);

static void monitor_handle_command1(void *opaque, const char *cmdline)
{
    monitor_handle_command(cmdline);
    monitor_start_input();
}

static void monitor_start_input(void)
{
    readline_start("(qemu) ", 0, monitor_handle_command1, NULL);
}

void monitor_init(CharDriverState *hd, int show_banner)
{
    monitor_hd = hd;
    if (show_banner) {
        term_printf("QEMU %s monitor - type 'help' for more information\n",
                    QEMU_VERSION);
    }
    qemu_chr_add_read_handler(hd, term_can_read, term_read, NULL);
    monitor_start_input();
}

/* XXX: use threads ? */
/* modal monitor readline */
static int monitor_readline_started;
static char *monitor_readline_buf;
static int monitor_readline_buf_size;

static void monitor_readline_cb(void *opaque, const char *input)
{
    pstrcpy(monitor_readline_buf, monitor_readline_buf_size, input);
    monitor_readline_started = 0;
}

void monitor_readline(const char *prompt, int is_password,
                      char *buf, int buf_size)
{
    if (is_password) {
        qemu_chr_send_event(monitor_hd, CHR_EVENT_FOCUS);
    }
    readline_start(prompt, is_password, monitor_readline_cb, NULL);
    monitor_readline_buf = buf;
    monitor_readline_buf_size = buf_size;
    monitor_readline_started = 1;
    while (monitor_readline_started) {
        main_loop_wait(10);
    }
}
