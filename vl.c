/*
 * QEMU System Emulator
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
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <getopt.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <malloc.h>
#include <termios.h>
#include <sys/poll.h>
#include <errno.h>
#include <sys/wait.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/if_tun.h>

#include "disas.h"
#include "thunk.h"

#include "vl.h"

#define DEFAULT_NETWORK_SCRIPT "/etc/qemu-ifup"

//#define DEBUG_UNUSED_IOPORT

#if !defined(CONFIG_SOFTMMU)
#define PHYS_RAM_MAX_SIZE (256 * 1024 * 1024)
#else
#define PHYS_RAM_MAX_SIZE (2047 * 1024 * 1024)
#endif

#if defined (TARGET_I386)
#elif defined (TARGET_PPC)
//#define USE_OPEN_FIRMWARE
#if !defined (USE_OPEN_FIRMWARE)
#define KERNEL_LOAD_ADDR    0x01000000
#define KERNEL_STACK_ADDR   0x01200000
#else
#define KERNEL_LOAD_ADDR    0x00000000
#define KERNEL_STACK_ADDR   0x00400000
#endif
#endif

#define GUI_REFRESH_INTERVAL 30 

/* XXX: use a two level table to limit memory usage */
#define MAX_IOPORTS 65536

const char *bios_dir = CONFIG_QEMU_SHAREDIR;
char phys_ram_file[1024];
CPUState *global_env;
CPUState *cpu_single_env;
IOPortReadFunc *ioport_read_table[3][MAX_IOPORTS];
IOPortWriteFunc *ioport_write_table[3][MAX_IOPORTS];
BlockDriverState *bs_table[MAX_DISKS], *fd_table[MAX_FD];
int vga_ram_size;
static DisplayState display_state;
int nographic;
int term_inited;
int64_t ticks_per_sec;
int boot_device = 'c';
static int ram_size;
static char network_script[1024];
int pit_min_timer_count = 0;

/***********************************************************/
/* x86 io ports */

uint32_t default_ioport_readb(CPUState *env, uint32_t address)
{
#ifdef DEBUG_UNUSED_IOPORT
    fprintf(stderr, "inb: port=0x%04x\n", address);
#endif
    return 0xff;
}

void default_ioport_writeb(CPUState *env, uint32_t address, uint32_t data)
{
#ifdef DEBUG_UNUSED_IOPORT
    fprintf(stderr, "outb: port=0x%04x data=0x%02x\n", address, data);
#endif
}

/* default is to make two byte accesses */
uint32_t default_ioport_readw(CPUState *env, uint32_t address)
{
    uint32_t data;
    data = ioport_read_table[0][address & (MAX_IOPORTS - 1)](env, address);
    data |= ioport_read_table[0][(address + 1) & (MAX_IOPORTS - 1)](env, address + 1) << 8;
    return data;
}

void default_ioport_writew(CPUState *env, uint32_t address, uint32_t data)
{
    ioport_write_table[0][address & (MAX_IOPORTS - 1)](env, address, data & 0xff);
    ioport_write_table[0][(address + 1) & (MAX_IOPORTS - 1)](env, address + 1, (data >> 8) & 0xff);
}

uint32_t default_ioport_readl(CPUState *env, uint32_t address)
{
#ifdef DEBUG_UNUSED_IOPORT
    fprintf(stderr, "inl: port=0x%04x\n", address);
#endif
    return 0xffffffff;
}

void default_ioport_writel(CPUState *env, uint32_t address, uint32_t data)
{
#ifdef DEBUG_UNUSED_IOPORT
    fprintf(stderr, "outl: port=0x%04x data=0x%02x\n", address, data);
#endif
}

void init_ioports(void)
{
    int i;

    for(i = 0; i < MAX_IOPORTS; i++) {
        ioport_read_table[0][i] = default_ioport_readb;
        ioport_write_table[0][i] = default_ioport_writeb;
        ioport_read_table[1][i] = default_ioport_readw;
        ioport_write_table[1][i] = default_ioport_writew;
        ioport_read_table[2][i] = default_ioport_readl;
        ioport_write_table[2][i] = default_ioport_writel;
    }
}

/* size is the word size in byte */
int register_ioport_read(int start, int length, IOPortReadFunc *func, int size)
{
    int i, bsize;

    if (size == 1)
        bsize = 0;
    else if (size == 2)
        bsize = 1;
    else if (size == 4)
        bsize = 2;
    else
        return -1;
    for(i = start; i < start + length; i += size)
        ioport_read_table[bsize][i] = func;
    return 0;
}

/* size is the word size in byte */
int register_ioport_write(int start, int length, IOPortWriteFunc *func, int size)
{
    int i, bsize;

    if (size == 1)
        bsize = 0;
    else if (size == 2)
        bsize = 1;
    else if (size == 4)
        bsize = 2;
    else
        return -1;
    for(i = start; i < start + length; i += size)
        ioport_write_table[bsize][i] = func;
    return 0;
}

void pstrcpy(char *buf, int buf_size, const char *str)
{
    int c;
    char *q = buf;

    if (buf_size <= 0)
        return;

    for(;;) {
        c = *str++;
        if (c == 0 || q >= buf + buf_size - 1)
            break;
        *q++ = c;
    }
    *q = '\0';
}

/* strcat and truncate. */
char *pstrcat(char *buf, int buf_size, const char *s)
{
    int len;
    len = strlen(buf);
    if (len < buf_size) 
        pstrcpy(buf + len, buf_size - len, s);
    return buf;
}

/* return the size or -1 if error */
int load_image(const char *filename, uint8_t *addr)
{
    int fd, size;
    fd = open(filename, O_RDONLY);
    if (fd < 0)
        return -1;
    size = lseek(fd, 0, SEEK_END);
    lseek(fd, 0, SEEK_SET);
    if (read(fd, addr, size) != size) {
        close(fd);
        return -1;
    }
    close(fd);
    return size;
}

void cpu_outb(CPUState *env, int addr, int val)
{
    ioport_write_table[0][addr & (MAX_IOPORTS - 1)](env, addr, val);
}

void cpu_outw(CPUState *env, int addr, int val)
{
    ioport_write_table[1][addr & (MAX_IOPORTS - 1)](env, addr, val);
}

void cpu_outl(CPUState *env, int addr, int val)
{
    ioport_write_table[2][addr & (MAX_IOPORTS - 1)](env, addr, val);
}

int cpu_inb(CPUState *env, int addr)
{
    return ioport_read_table[0][addr & (MAX_IOPORTS - 1)](env, addr);
}

int cpu_inw(CPUState *env, int addr)
{
    return ioport_read_table[1][addr & (MAX_IOPORTS - 1)](env, addr);
}

int cpu_inl(CPUState *env, int addr)
{
    return ioport_read_table[2][addr & (MAX_IOPORTS - 1)](env, addr);
}

/***********************************************************/
void hw_error(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    fprintf(stderr, "qemu: hardware error: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
#ifdef TARGET_I386
    cpu_x86_dump_state(global_env, stderr, X86_DUMP_FPU | X86_DUMP_CCOP);
#else
    cpu_dump_state(global_env, stderr, 0);
#endif
    va_end(ap);
    abort();
}

#if defined(__powerpc__)

static inline uint32_t get_tbl(void) 
{
    uint32_t tbl;
    asm volatile("mftb %0" : "=r" (tbl));
    return tbl;
}

static inline uint32_t get_tbu(void) 
{
	uint32_t tbl;
	asm volatile("mftbu %0" : "=r" (tbl));
	return tbl;
}

int64_t cpu_get_real_ticks(void)
{
    uint32_t l, h, h1;
    /* NOTE: we test if wrapping has occurred */
    do {
        h = get_tbu();
        l = get_tbl();
        h1 = get_tbu();
    } while (h != h1);
    return ((int64_t)h << 32) | l;
}

#elif defined(__i386__)

int64_t cpu_get_real_ticks(void)
{
    int64_t val;
    asm("rdtsc" : "=A" (val));
    return val;
}

#else
#error unsupported CPU
#endif

static int64_t cpu_ticks_offset;
static int64_t cpu_ticks_last;

int64_t cpu_get_ticks(void)
{
    return cpu_get_real_ticks() + cpu_ticks_offset;
}

/* enable cpu_get_ticks() */
void cpu_enable_ticks(void)
{
    cpu_ticks_offset = cpu_ticks_last - cpu_get_real_ticks();
}

/* disable cpu_get_ticks() : the clock is stopped. You must not call
   cpu_get_ticks() after that.  */
void cpu_disable_ticks(void)
{
    cpu_ticks_last = cpu_get_ticks();
}

int64_t get_clock(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000LL + tv.tv_usec;
}

void cpu_calibrate_ticks(void)
{
    int64_t usec, ticks;

    usec = get_clock();
    ticks = cpu_get_ticks();
    usleep(50 * 1000);
    usec = get_clock() - usec;
    ticks = cpu_get_ticks() - ticks;
    ticks_per_sec = (ticks * 1000000LL + (usec >> 1)) / usec;
}

/* compute with 96 bit intermediate result: (a*b)/c */
uint64_t muldiv64(uint64_t a, uint32_t b, uint32_t c)
{
    union {
        uint64_t ll;
        struct {
#ifdef WORDS_BIGENDIAN
            uint32_t high, low;
#else
            uint32_t low, high;
#endif            
        } l;
    } u, res;
    uint64_t rl, rh;

    u.ll = a;
    rl = (uint64_t)u.l.low * (uint64_t)b;
    rh = (uint64_t)u.l.high * (uint64_t)b;
    rh += (rl >> 32);
    res.l.high = rh / c;
    res.l.low = (((rh % c) << 32) + (rl & 0xffffffff)) / c;
    return res.ll;
}

#define TERM_ESCAPE 0x01 /* ctrl-a is used for escape */
static int term_got_escape, term_command;
static unsigned char term_cmd_buf[128];

typedef struct term_cmd_t {
    const unsigned char *name;
    void (*handler)(unsigned char *params);
} term_cmd_t;

static void do_change_cdrom (unsigned char *params);
static void do_change_fd0 (unsigned char *params);
static void do_change_fd1 (unsigned char *params);

static term_cmd_t term_cmds[] = {
    { "changecd", &do_change_cdrom, },
    { "changefd0", &do_change_fd0, },
    { "changefd1", &do_change_fd1, },
    { NULL, NULL, },
};

void term_print_help(void)
{
    printf("\n"
           "C-a h    print this help\n"
           "C-a x    exit emulatior\n"
           "C-a d    switch on/off debug log\n"
	   "C-a s    save disk data back to file (if -snapshot)\n"
           "C-a b    send break (magic sysrq)\n"
           "C-a c    send qemu internal command\n"
           "C-a C-a  send C-a\n"
           );
}

static void do_change_cdrom (unsigned char *params)
{
    /* Dunno how to do it... */
}

static void do_change_fd (int fd, unsigned char *params)
{
    unsigned char *name_start, *name_end, *ros;
    int ro;

    for (name_start = params;
         isspace(*name_start); name_start++)
        continue;
    if (*name_start == '\0')
        return;
    for (name_end = name_start;
         !isspace(*name_end) && *name_end != '\0'; name_end++)
        continue;
    for (ros = name_end + 1; isspace(*ros); ros++)
        continue;
    if (ros[0] == 'r' && ros[1] == 'o')
        ro = 1;
    else
        ro = 0;
    *name_end = '\0';
    printf("Change fd %d to %s (%s)\n", fd, name_start, params);
    fdctrl_disk_change(fd, name_start, ro);
}

static void do_change_fd0 (unsigned char *params)
{
    do_change_fd(0, params);
}

static void do_change_fd1 (unsigned char *params)
{
    do_change_fd(1, params);
}

static void term_treat_command(void)
{
    unsigned char *cmd_start, *cmd_end;
    int i;

    for (cmd_start = term_cmd_buf; isspace(*cmd_start); cmd_start++)
        continue;
    for (cmd_end = cmd_start;
         !isspace(*cmd_end) && *cmd_end != '\0'; cmd_end++)
        continue;
    for (i = 0; term_cmds[i].name != NULL; i++) {
        if (strlen(term_cmds[i].name) == (cmd_end - cmd_start) &&
            memcmp(term_cmds[i].name, cmd_start, cmd_end - cmd_start) == 0) {
            (*term_cmds[i].handler)(cmd_end + 1);
            return;
        }
    }
    *cmd_end = '\0';
    printf("Unknown term command: %s\n", cmd_start);
}

extern FILE *logfile;

/* called when a char is received */
void term_received_byte(int ch)
{
    if (term_command) {
        if (ch == '\n' || ch == '\r' || term_command == 127) {
            printf("\n");
            term_treat_command();
            term_command = 0;
        } else {
            if (ch == 0x7F || ch == 0x08) {
                if (term_command > 1) {
                    term_cmd_buf[--term_command - 1] = '\0';
                    printf("\r                                               "
                           "                               ");
                    printf("\r> %s", term_cmd_buf);
                }
            } else if (ch > 0x1f) {
                term_cmd_buf[term_command++ - 1] = ch;
                term_cmd_buf[term_command - 1] = '\0';
                printf("\r> %s", term_cmd_buf);
            }
            fflush(stdout);
        }
    } else if (term_got_escape) {
        term_got_escape = 0;
        switch(ch) {
        case 'h':
            term_print_help();
            break;
        case 'x':
            exit(0);
            break;
	case 's': 
            {
                int i;
                for (i = 0; i < MAX_DISKS; i++) {
                    if (bs_table[i])
                        bdrv_commit(bs_table[i]);
                }
	    }
            break;
        case 'b':
            serial_receive_break();
            break;
        case 'c':
            printf("> ");
            fflush(stdout);
            term_command = 1;
            break;
        case 'd':
            cpu_set_log(CPU_LOG_ALL);
            break;
        case TERM_ESCAPE:
            goto send_char;
        }
    } else if (ch == TERM_ESCAPE) {
        term_got_escape = 1;
    } else {
    send_char:
        serial_receive_byte(ch);
    }
}

/***********************************************************/
/* Linux network device redirector */

int net_init(void)
{
    struct ifreq ifr;
    int fd, ret, pid, status;
    
    fd = open("/dev/net/tun", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "warning: could not open /dev/net/tun: no virtual network emulation\n");
        return -1;
    }
    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
    pstrcpy(ifr.ifr_name, IFNAMSIZ, "tun%d");
    ret = ioctl(fd, TUNSETIFF, (void *) &ifr);
    if (ret != 0) {
        fprintf(stderr, "warning: could not configure /dev/net/tun: no virtual network emulation\n");
        close(fd);
        return -1;
    }
    printf("Connected to host network interface: %s\n", ifr.ifr_name);
    fcntl(fd, F_SETFL, O_NONBLOCK);
    net_fd = fd;

    /* try to launch network init script */
    pid = fork();
    if (pid >= 0) {
        if (pid == 0) {
            execl(network_script, network_script, ifr.ifr_name, NULL);
            exit(1);
        }
        while (waitpid(pid, &status, 0) != pid);
        if (!WIFEXITED(status) ||
            WEXITSTATUS(status) != 0) {
            fprintf(stderr, "%s: could not launch network script for '%s'\n",
                    network_script, ifr.ifr_name);
        }
    }
    return 0;
}

void net_send_packet(int net_fd, const uint8_t *buf, int size)
{
#ifdef DEBUG_NE2000
    printf("NE2000: sending packet size=%d\n", size);
#endif
    write(net_fd, buf, size);
}

/***********************************************************/
/* dumb display */

/* init terminal so that we can grab keys */
static struct termios oldtty;

static void term_exit(void)
{
    tcsetattr (0, TCSANOW, &oldtty);
}

static void term_init(void)
{
    struct termios tty;

    tcgetattr (0, &tty);
    oldtty = tty;

    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP
                          |INLCR|IGNCR|ICRNL|IXON);
    tty.c_oflag |= OPOST;
    tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
    /* if graphical mode, we allow Ctrl-C handling */
    if (nographic)
        tty.c_lflag &= ~ISIG;
    tty.c_cflag &= ~(CSIZE|PARENB);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
    
    tcsetattr (0, TCSANOW, &tty);

    atexit(term_exit);

    fcntl(0, F_SETFL, O_NONBLOCK);
}

static void dumb_update(DisplayState *ds, int x, int y, int w, int h)
{
}

static void dumb_resize(DisplayState *ds, int w, int h)
{
}

static void dumb_refresh(DisplayState *ds)
{
    vga_update_display();
}

void dumb_display_init(DisplayState *ds)
{
    ds->data = NULL;
    ds->linesize = 0;
    ds->depth = 0;
    ds->dpy_update = dumb_update;
    ds->dpy_resize = dumb_resize;
    ds->dpy_refresh = dumb_refresh;
}

#if !defined(CONFIG_SOFTMMU)
/***********************************************************/
/* cpu signal handler */
static void host_segv_handler(int host_signum, siginfo_t *info, 
                              void *puc)
{
    if (cpu_signal_handler(host_signum, info, puc))
        return;
    term_exit();
    abort();
}
#endif

static int timer_irq_pending;
static int timer_irq_count;

static int timer_ms;
static int gui_refresh_pending, gui_refresh_count;

static void host_alarm_handler(int host_signum, siginfo_t *info, 
                               void *puc)
{
    /* NOTE: since usually the OS asks a 100 Hz clock, there can be
       some drift between cpu_get_ticks() and the interrupt time. So
       we queue some interrupts to avoid missing some */
    timer_irq_count += pit_get_out_edges(&pit_channels[0]);
    if (timer_irq_count) {
        if (timer_irq_count > 2)
            timer_irq_count = 2;
        timer_irq_count--;
        timer_irq_pending = 1;
    }
    gui_refresh_count += timer_ms;
    if (gui_refresh_count >= GUI_REFRESH_INTERVAL) {
        gui_refresh_count = 0;
        gui_refresh_pending = 1;
    }

    if (gui_refresh_pending || timer_irq_pending) {
        /* just exit from the cpu to have a chance to handle timers */
        cpu_interrupt(global_env, CPU_INTERRUPT_EXIT);
    }
}

/* main execution loop */

CPUState *cpu_gdbstub_get_env(void *opaque)
{
    return global_env;
}

int main_loop(void *opaque)
{
    struct pollfd ufds[3], *pf, *serial_ufd, *gdb_ufd;
#if defined (TARGET_I386)
    struct pollfd *net_ufd;
#endif
    int ret, n, timeout, serial_ok;
    uint8_t ch;
    CPUState *env = global_env;

    if (!term_inited) {
        /* initialize terminal only there so that the user has a
           chance to stop QEMU with Ctrl-C before the gdb connection
           is launched */
        term_inited = 1;
        term_init();
    }

    serial_ok = 1;
    cpu_enable_ticks();
    for(;;) {
#if defined (DO_TB_FLUSH)
        tb_flush();
#endif
        ret = cpu_exec(env);
        if (reset_requested) {
            ret = EXCP_INTERRUPT; 
            break;
        }
        if (ret == EXCP_DEBUG) {
            ret = EXCP_DEBUG;
            break;
        }
        /* if hlt instruction, we wait until the next IRQ */
        if (ret == EXCP_HLT) 
            timeout = 10;
        else
            timeout = 0;
        /* poll any events */
        serial_ufd = NULL;
        pf = ufds;
        if (serial_ok && serial_can_receive()) {
            serial_ufd = pf;
            pf->fd = 0;
            pf->events = POLLIN;
            pf++;
        }
#if defined (TARGET_I386)
        net_ufd = NULL;
        if (net_fd > 0 && ne2000_can_receive()) {
            net_ufd = pf;
            pf->fd = net_fd;
            pf->events = POLLIN;
            pf++;
        }
#endif
        gdb_ufd = NULL;
        if (gdbstub_fd > 0) {
            gdb_ufd = pf;
            pf->fd = gdbstub_fd;
            pf->events = POLLIN;
            pf++;
        }

        ret = poll(ufds, pf - ufds, timeout);
        if (ret > 0) {
            if (serial_ufd && (serial_ufd->revents & POLLIN)) {
                n = read(0, &ch, 1);
                if (n == 1) {
                    term_received_byte(ch);
                } else {
		    /* Closed, stop polling. */
                    serial_ok = 0;
                }
            }
#if defined (TARGET_I386)
            if (net_ufd && (net_ufd->revents & POLLIN)) {
                uint8_t buf[MAX_ETH_FRAME_SIZE];

                n = read(net_fd, buf, MAX_ETH_FRAME_SIZE);
                if (n > 0) {
                    if (n < 60) {
                        memset(buf + n, 0, 60 - n);
                        n = 60;
                    }
                    ne2000_receive(buf, n);
                }
            }
#endif
            if (gdb_ufd && (gdb_ufd->revents & POLLIN)) {
                uint8_t buf[1];
                /* stop emulation if requested by gdb */
                n = read(gdbstub_fd, buf, 1);
                if (n == 1) {
                    ret = EXCP_INTERRUPT; 
                    break;
                }
            }
        }

        /* timer IRQ */
        if (timer_irq_pending) {
#if defined (TARGET_I386)
            pic_set_irq(0, 1);
            pic_set_irq(0, 0);
            timer_irq_pending = 0;
            rtc_timer();
#endif
        }
        /* XXX: add explicit timer */
        SB16_run();

        /* run dma transfers, if any */
        DMA_run();

        /* VGA */
        if (gui_refresh_pending) {
            display_state.dpy_refresh(&display_state);
            gui_refresh_pending = 0;
        }
    }
    cpu_disable_ticks();
    return ret;
}

void help(void)
{
    printf("QEMU PC emulator version " QEMU_VERSION ", Copyright (c) 2003 Fabrice Bellard\n"
           "usage: %s [options] [disk_image]\n"
           "\n"
           "'disk_image' is a raw hard image image for IDE hard disk 0\n"
           "\n"
           "Standard options:\n"
           "-fda/-fdb file  use 'file' as floppy disk 0/1 image\n"
           "-hda/-hdb file  use 'file' as IDE hard disk 0/1 image\n"
           "-hdc/-hdd file  use 'file' as IDE hard disk 2/3 image\n"
           "-cdrom file     use 'file' as IDE cdrom 2 image\n"
           "-boot [a|b|c|d] boot on floppy (a, b), hard disk (c) or CD-ROM (d)\n"
	   "-snapshot       write to temporary files instead of disk image files\n"
           "-m megs         set virtual RAM size to megs MB\n"
           "-n script       set network init script [default=%s]\n"
           "-tun-fd fd      this fd talks to tap/tun, use it.\n"
           "-nographic      disable graphical output\n"
           "\n"
           "Linux boot specific (does not require PC BIOS):\n"
           "-kernel bzImage use 'bzImage' as kernel image\n"
           "-append cmdline use 'cmdline' as kernel command line\n"
           "-initrd file    use 'file' as initial ram disk\n"
           "\n"
           "Debug/Expert options:\n"
           "-s              wait gdb connection to port %d\n"
           "-p port         change gdb connection port\n"
           "-d              output log to %s\n"
           "-hdachs c,h,s   force hard disk 0 geometry (usually qemu can guess it)\n"
           "-L path         set the directory for the BIOS and VGA BIOS\n"
#ifdef USE_CODE_COPY
           "-no-code-copy   disable code copy acceleration\n"
#endif

           "\n"
           "During emulation, use C-a h to get terminal commands:\n",
#ifdef CONFIG_SOFTMMU
           "qemu",
#else
           "qemu-fast",
#endif
           DEFAULT_NETWORK_SCRIPT, 
           DEFAULT_GDBSTUB_PORT,
           "/tmp/qemu.log");
    term_print_help();
#ifndef CONFIG_SOFTMMU
    printf("\n"
           "NOTE: this version of QEMU is faster but it needs slightly patched OSes to\n"
           "work. Please use the 'qemu' executable to have a more accurate (but slower)\n"
           "PC emulation.\n");
#endif
    exit(1);
}

struct option long_options[] = {
    { "initrd", 1, NULL, 0, },
    { "hda", 1, NULL, 0, },
    { "hdb", 1, NULL, 0, },
    { "snapshot", 0, NULL, 0, },
    { "hdachs", 1, NULL, 0, },
    { "nographic", 0, NULL, 0, },
    { "kernel", 1, NULL, 0, },
    { "append", 1, NULL, 0, },
    { "tun-fd", 1, NULL, 0, },
    { "hdc", 1, NULL, 0, },
    { "hdd", 1, NULL, 0, },
    { "cdrom", 1, NULL, 0, },
    { "boot", 1, NULL, 0, },
    { "fda", 1, NULL, 0, },
    { "fdb", 1, NULL, 0, },
    { "no-code-copy", 0, NULL, 0},
    { NULL, 0, NULL, 0 },
};

#ifdef CONFIG_SDL
/* SDL use the pthreads and they modify sigaction. We don't
   want that. */
#if __GLIBC__ > 2 || (__GLIBC__ == 2 && __GLIBC_MINOR__ >= 2)
extern void __libc_sigaction();
#define sigaction(sig, act, oact) __libc_sigaction(sig, act, oact)
#else
extern void __sigaction();
#define sigaction(sig, act, oact) __sigaction(sig, act, oact)
#endif
#endif /* CONFIG_SDL */

#if defined (TARGET_I386) && defined(USE_CODE_COPY)

/* this stack is only used during signal handling */
#define SIGNAL_STACK_SIZE 32768

static uint8_t *signal_stack;

#endif

int main(int argc, char **argv)
{
    int c, i, use_gdbstub, gdbstub_port, long_index;
    int snapshot, linux_boot;
    struct sigaction act;
    struct itimerval itv;
    CPUState *env;
    const char *initrd_filename;
    const char *hd_filename[MAX_DISKS], *fd_filename[MAX_FD];
    const char *kernel_filename, *kernel_cmdline;
    DisplayState *ds = &display_state;

    /* we never want that malloc() uses mmap() */
    mallopt(M_MMAP_THRESHOLD, 4096 * 1024);
    initrd_filename = NULL;
    for(i = 0; i < MAX_FD; i++)
        fd_filename[i] = NULL;
    for(i = 0; i < MAX_DISKS; i++)
        hd_filename[i] = NULL;
    ram_size = 32 * 1024 * 1024;
    vga_ram_size = VGA_RAM_SIZE;
#if defined (TARGET_I386)
    pstrcpy(network_script, sizeof(network_script), DEFAULT_NETWORK_SCRIPT);
#endif
    use_gdbstub = 0;
    gdbstub_port = DEFAULT_GDBSTUB_PORT;
    snapshot = 0;
    nographic = 0;
    kernel_filename = NULL;
    kernel_cmdline = "";
    for(;;) {
        c = getopt_long_only(argc, argv, "hm:dn:sp:L:", long_options, &long_index);
        if (c == -1)
            break;
        switch(c) {
        case 0:
            switch(long_index) {
            case 0:
                initrd_filename = optarg;
                break;
            case 1:
                hd_filename[0] = optarg;
                break;
            case 2:
                hd_filename[1] = optarg;
                break;
            case 3:
                snapshot = 1;
                break;
            case 4:
                {
                    int cyls, heads, secs;
                    const char *p;
                    p = optarg;
                    cyls = strtol(p, (char **)&p, 0);
                    if (*p != ',')
                        goto chs_fail;
                    p++;
                    heads = strtol(p, (char **)&p, 0);
                    if (*p != ',')
                        goto chs_fail;
                    p++;
                    secs = strtol(p, (char **)&p, 0);
                    if (*p != '\0')
                        goto chs_fail;
                    ide_set_geometry(0, cyls, heads, secs);
                chs_fail: ;
                }
                break;
            case 5:
                nographic = 1;
                break;
            case 6:
                kernel_filename = optarg;
                break;
            case 7:
                kernel_cmdline = optarg;
                break;
#if defined (TARGET_I386)
	    case 8:
		net_fd = atoi(optarg);
		break;
#endif
            case 9:
                hd_filename[2] = optarg;
                break;
            case 10:
                hd_filename[3] = optarg;
                break;
            case 11:
                hd_filename[2] = optarg;
                ide_set_cdrom(2, 1);
                break;
            case 12:
                boot_device = optarg[0];
                if (boot_device != 'a' && boot_device != 'b' &&
                    boot_device != 'c' && boot_device != 'd') {
                    fprintf(stderr, "qemu: invalid boot device '%c'\n", boot_device);
                    exit(1);
                }
                break;
            case 13:
                fd_filename[0] = optarg;
                break;
            case 14:
                fd_filename[1] = optarg;
                break;
            case 15:
                code_copy_enabled = 0;
                break;
            }
            break;
        case 'h':
            help();
            break;
        case 'm':
            ram_size = atoi(optarg) * 1024 * 1024;
            if (ram_size <= 0)
                help();
            if (ram_size > PHYS_RAM_MAX_SIZE) {
                fprintf(stderr, "qemu: at most %d MB RAM can be simulated\n",
                        PHYS_RAM_MAX_SIZE / (1024 * 1024));
                exit(1);
            }
            break;
        case 'd':
            cpu_set_log(CPU_LOG_ALL);
            break;
#if defined (TARGET_I386)
        case 'n':
            pstrcpy(network_script, sizeof(network_script), optarg);
            break;
#endif
        case 's':
            use_gdbstub = 1;
            break;
        case 'p':
            gdbstub_port = atoi(optarg);
            break;
        case 'L':
            bios_dir = optarg;
            break;
        }
    }

    if (optind < argc) {
        hd_filename[0] = argv[optind++];
    }

    linux_boot = (kernel_filename != NULL);
        
    if (!linux_boot && hd_filename[0] == '\0' && hd_filename[2] == '\0' &&
        fd_filename[0] == '\0')
        help();
    
    /* boot to cd by default if no hard disk */
    if (hd_filename[0] == '\0' && boot_device == 'c') {
        if (fd_filename[0] != '\0')
            boot_device = 'a';
        else
            boot_device = 'd';
    }

#if !defined(CONFIG_SOFTMMU)
    /* must avoid mmap() usage of glibc by setting a buffer "by hand" */
    {
        static uint8_t stdout_buf[4096];
        setvbuf(stdout, stdout_buf, _IOLBF, sizeof(stdout_buf));
    }
#else
    setvbuf(stdout, NULL, _IOLBF, 0);
#endif

    /* init network tun interface */
#if defined (TARGET_I386)
    if (net_fd < 0)
	net_init();
#endif

    /* init the memory */
    phys_ram_size = ram_size + vga_ram_size;

#ifdef CONFIG_SOFTMMU
    phys_ram_base = memalign(TARGET_PAGE_SIZE, phys_ram_size);
    if (!phys_ram_base) {
        fprintf(stderr, "Could not allocate physical memory\n");
        exit(1);
    }
#else
    /* as we must map the same page at several addresses, we must use
       a fd */
    {
        const char *tmpdir;

        tmpdir = getenv("QEMU_TMPDIR");
        if (!tmpdir)
            tmpdir = "/tmp";
        snprintf(phys_ram_file, sizeof(phys_ram_file), "%s/vlXXXXXX", tmpdir);
        if (mkstemp(phys_ram_file) < 0) {
            fprintf(stderr, "Could not create temporary memory file '%s'\n", 
                    phys_ram_file);
            exit(1);
        }
        phys_ram_fd = open(phys_ram_file, O_CREAT | O_TRUNC | O_RDWR, 0600);
        if (phys_ram_fd < 0) {
            fprintf(stderr, "Could not open temporary memory file '%s'\n", 
                    phys_ram_file);
            exit(1);
        }
        ftruncate(phys_ram_fd, phys_ram_size);
        unlink(phys_ram_file);
        phys_ram_base = mmap(get_mmap_addr(phys_ram_size), 
                             phys_ram_size, 
                             PROT_WRITE | PROT_READ, MAP_SHARED | MAP_FIXED, 
                             phys_ram_fd, 0);
        if (phys_ram_base == MAP_FAILED) {
            fprintf(stderr, "Could not map physical memory\n");
            exit(1);
        }
    }
#endif

    /* open the virtual block devices */
    for(i = 0; i < MAX_DISKS; i++) {
        if (hd_filename[i]) {
            bs_table[i] = bdrv_open(hd_filename[i], snapshot);
            if (!bs_table[i]) {
                fprintf(stderr, "qemu: could not open hard disk image '%s\n",
                        hd_filename[i]);
                exit(1);
            }
        }
    }

    /* init CPU state */
    env = cpu_init();
    global_env = env;
    cpu_single_env = env;

    init_ioports();
    cpu_calibrate_ticks();

    /* terminal init */
    if (nographic) {
        dumb_display_init(ds);
    } else {
#ifdef CONFIG_SDL
        sdl_display_init(ds);
#else
        dumb_display_init(ds);
#endif
    }

#if defined(TARGET_I386)
    pc_init(ram_size, vga_ram_size, boot_device,
            ds, fd_filename, snapshot,
            kernel_filename, kernel_cmdline, initrd_filename);
#elif defined(TARGET_PPC)
    ppc_init();
#endif

    /* setup cpu signal handlers for MMU / self modifying code handling */
#if !defined(CONFIG_SOFTMMU)

#if defined (TARGET_I386) && defined(USE_CODE_COPY)
    {
        stack_t stk;
        signal_stack = malloc(SIGNAL_STACK_SIZE);
        stk.ss_sp = signal_stack;
        stk.ss_size = SIGNAL_STACK_SIZE;
        stk.ss_flags = 0;

        if (sigaltstack(&stk, NULL) < 0) {
            perror("sigaltstack");
            exit(1);
        }
    }
#endif
        
    sigfillset(&act.sa_mask);
    act.sa_flags = SA_SIGINFO;
#if defined (TARGET_I386) && defined(USE_CODE_COPY)
    act.sa_flags |= SA_ONSTACK;
#endif
    act.sa_sigaction = host_segv_handler;
    sigaction(SIGSEGV, &act, NULL);
    sigaction(SIGBUS, &act, NULL);
#if defined (TARGET_I386) && defined(USE_CODE_COPY)
    sigaction(SIGFPE, &act, NULL);
#endif
#endif

    /* timer signal */
    sigfillset(&act.sa_mask);
    act.sa_flags = SA_SIGINFO;
#if defined (TARGET_I386) && defined(USE_CODE_COPY)
    act.sa_flags |= SA_ONSTACK;
#endif
    act.sa_sigaction = host_alarm_handler;
    sigaction(SIGALRM, &act, NULL);

    itv.it_interval.tv_sec = 0;
    itv.it_interval.tv_usec = 1000;
    itv.it_value.tv_sec = 0;
    itv.it_value.tv_usec = 10 * 1000;
    setitimer(ITIMER_REAL, &itv, NULL);
    /* we probe the tick duration of the kernel to inform the user if
       the emulated kernel requested a too high timer frequency */
    getitimer(ITIMER_REAL, &itv);
    timer_ms = itv.it_interval.tv_usec / 1000;
    pit_min_timer_count = ((uint64_t)itv.it_interval.tv_usec * PIT_FREQ) / 
        1000000;

    if (use_gdbstub) {
        cpu_gdbstub(NULL, main_loop, gdbstub_port);
    } else {
        main_loop(NULL);
    }
    return 0;
}
