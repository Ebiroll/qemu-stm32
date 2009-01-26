#ifndef CONSOLE_H
#define CONSOLE_H

#include "qemu-char.h"

/* keyboard/mouse support */

#define MOUSE_EVENT_LBUTTON 0x01
#define MOUSE_EVENT_RBUTTON 0x02
#define MOUSE_EVENT_MBUTTON 0x04

/* in ms */
#define GUI_REFRESH_INTERVAL 30

typedef void QEMUPutKBDEvent(void *opaque, int keycode);
typedef void QEMUPutMouseEvent(void *opaque, int dx, int dy, int dz, int buttons_state);

typedef struct QEMUPutMouseEntry {
    QEMUPutMouseEvent *qemu_put_mouse_event;
    void *qemu_put_mouse_event_opaque;
    int qemu_put_mouse_event_absolute;
    char *qemu_put_mouse_event_name;

    /* used internally by qemu for handling mice */
    struct QEMUPutMouseEntry *next;
} QEMUPutMouseEntry;

void qemu_add_kbd_event_handler(QEMUPutKBDEvent *func, void *opaque);
QEMUPutMouseEntry *qemu_add_mouse_event_handler(QEMUPutMouseEvent *func,
                                                void *opaque, int absolute,
                                                const char *name);
void qemu_remove_mouse_event_handler(QEMUPutMouseEntry *entry);

void kbd_put_keycode(int keycode);
void kbd_mouse_event(int dx, int dy, int dz, int buttons_state);
int kbd_mouse_is_absolute(void);

struct mouse_transform_info_s {
    /* Touchscreen resolution */
    int x;
    int y;
    /* Calibration values as used/generated by tslib */
    int a[7];
};

void do_info_mice(void);
void do_mouse_set(int index);

/* keysym is a unicode code except for special keys (see QEMU_KEY_xxx
   constants) */
#define QEMU_KEY_ESC1(c) ((c) | 0xe100)
#define QEMU_KEY_BACKSPACE  0x007f
#define QEMU_KEY_UP         QEMU_KEY_ESC1('A')
#define QEMU_KEY_DOWN       QEMU_KEY_ESC1('B')
#define QEMU_KEY_RIGHT      QEMU_KEY_ESC1('C')
#define QEMU_KEY_LEFT       QEMU_KEY_ESC1('D')
#define QEMU_KEY_HOME       QEMU_KEY_ESC1(1)
#define QEMU_KEY_END        QEMU_KEY_ESC1(4)
#define QEMU_KEY_PAGEUP     QEMU_KEY_ESC1(5)
#define QEMU_KEY_PAGEDOWN   QEMU_KEY_ESC1(6)
#define QEMU_KEY_DELETE     QEMU_KEY_ESC1(3)

#define QEMU_KEY_CTRL_UP         0xe400
#define QEMU_KEY_CTRL_DOWN       0xe401
#define QEMU_KEY_CTRL_LEFT       0xe402
#define QEMU_KEY_CTRL_RIGHT      0xe403
#define QEMU_KEY_CTRL_HOME       0xe404
#define QEMU_KEY_CTRL_END        0xe405
#define QEMU_KEY_CTRL_PAGEUP     0xe406
#define QEMU_KEY_CTRL_PAGEDOWN   0xe407

void kbd_put_keysym(int keysym);

/* consoles */

#define QEMU_BIG_ENDIAN_FLAG    0x01
#define QEMU_ALLOCATED_FLAG     0x02

struct PixelFormat {
    uint8_t bits_per_pixel;
    uint8_t bytes_per_pixel;
    uint8_t depth; /* color depth in bits */
    uint32_t rmask, gmask, bmask, amask;
    uint8_t rshift, gshift, bshift, ashift;
    uint8_t rmax, gmax, bmax, amax;
    uint8_t rbits, gbits, bbits, abits;
};

struct DisplaySurface {
    uint8_t flags;
    int width;
    int height;
    int linesize;        /* bytes per line */
    uint8_t *data;

    struct PixelFormat pf;
};

struct DisplayChangeListener {
    int idle;
    uint64_t gui_timer_interval;

    void (*dpy_update)(struct DisplayState *s, int x, int y, int w, int h);
    void (*dpy_resize)(struct DisplayState *s);
    void (*dpy_setdata)(struct DisplayState *s);
    void (*dpy_refresh)(struct DisplayState *s);
    void (*dpy_copy)(struct DisplayState *s, int src_x, int src_y,
                     int dst_x, int dst_y, int w, int h);
    void (*dpy_fill)(struct DisplayState *s, int x, int y,
                     int w, int h, uint32_t c);
    void (*dpy_text_cursor)(struct DisplayState *s, int x, int y);

    struct DisplayChangeListener *next;
};

struct DisplayState {
    struct DisplaySurface *surface;
    void *opaque;
    struct QEMUTimer *gui_timer;

    struct DisplayChangeListener* listeners;

    void (*mouse_set)(int x, int y, int on);
    void (*cursor_define)(int width, int height, int bpp, int hot_x, int hot_y,
                          uint8_t *image, uint8_t *mask);

    struct DisplayState *next;
};

void register_displaystate(DisplayState *ds);
DisplayState *get_displaystate(void);
DisplaySurface* qemu_create_displaysurface(int width, int height, int bpp, int linesize);
DisplaySurface* qemu_resize_displaysurface(DisplaySurface *surface,
                                           int width, int height, int bpp, int linesize);
DisplaySurface* qemu_create_displaysurface_from(int width, int height, int bpp,
                                                int linesize, uint8_t *data);
void qemu_free_displaysurface(DisplaySurface *surface);
PixelFormat qemu_different_endianness_pixelformat(int bpp);
PixelFormat qemu_default_pixelformat(int bpp);

static inline int is_buffer_shared(DisplaySurface *surface)
{
    return (!(surface->flags & QEMU_ALLOCATED_FLAG));
}

static inline void register_displaychangelistener(DisplayState *ds, DisplayChangeListener *dcl)
{
    dcl->next = ds->listeners;
    ds->listeners = dcl;
}

static inline void dpy_update(DisplayState *s, int x, int y, int w, int h)
{
    struct DisplayChangeListener *dcl = s->listeners;
    while (dcl != NULL) {
        dcl->dpy_update(s, x, y, w, h);
        dcl = dcl->next;
    }
}

static inline void dpy_resize(DisplayState *s)
{
    struct DisplayChangeListener *dcl = s->listeners;
    while (dcl != NULL) {
        dcl->dpy_resize(s);
        dcl = dcl->next;
    }
}

static inline void dpy_setdata(DisplayState *s)
{
    struct DisplayChangeListener *dcl = s->listeners;
    while (dcl != NULL) {
        if (dcl->dpy_setdata) dcl->dpy_setdata(s);
        dcl = dcl->next;
    }
}

static inline void dpy_refresh(DisplayState *s)
{
    struct DisplayChangeListener *dcl = s->listeners;
    while (dcl != NULL) {
        if (dcl->dpy_refresh) dcl->dpy_refresh(s);
        dcl = dcl->next;
    }
}

static inline void dpy_copy(struct DisplayState *s, int src_x, int src_y,
                             int dst_x, int dst_y, int w, int h) {
    struct DisplayChangeListener *dcl = s->listeners;
    while (dcl != NULL) {
        if (dcl->dpy_copy)
            dcl->dpy_copy(s, src_x, src_y, dst_x, dst_y, w, h);
        else /* TODO */
            dcl->dpy_update(s, dst_x, dst_y, w, h);
        dcl = dcl->next;
    }
}

static inline void dpy_fill(struct DisplayState *s, int x, int y,
                             int w, int h, uint32_t c) {
    struct DisplayChangeListener *dcl = s->listeners;
    while (dcl != NULL) {
        if (dcl->dpy_fill) dcl->dpy_fill(s, x, y, w, h, c);
        dcl = dcl->next;
    }
}

static inline void dpy_cursor(struct DisplayState *s, int x, int y) {
    struct DisplayChangeListener *dcl = s->listeners;
    while (dcl != NULL) {
        if (dcl->dpy_text_cursor) dcl->dpy_text_cursor(s, x, y);
        dcl = dcl->next;
    }
}

static inline int ds_get_linesize(DisplayState *ds)
{
    return ds->surface->linesize;
}

static inline uint8_t* ds_get_data(DisplayState *ds)
{
    return ds->surface->data;
}

static inline int ds_get_width(DisplayState *ds)
{
    return ds->surface->width;
}

static inline int ds_get_height(DisplayState *ds)
{
    return ds->surface->height;
}

static inline int ds_get_bits_per_pixel(DisplayState *ds)
{
    return ds->surface->pf.bits_per_pixel;
}

static inline int ds_get_bytes_per_pixel(DisplayState *ds)
{
    return ds->surface->pf.bytes_per_pixel;
}

typedef unsigned long console_ch_t;
static inline void console_write_ch(console_ch_t *dest, uint32_t ch)
{
    cpu_to_le32wu((uint32_t *) dest, ch);
}

typedef void (*vga_hw_update_ptr)(void *);
typedef void (*vga_hw_invalidate_ptr)(void *);
typedef void (*vga_hw_screen_dump_ptr)(void *, const char *);
typedef void (*vga_hw_text_update_ptr)(void *, console_ch_t *);

DisplayState *graphic_console_init(vga_hw_update_ptr update,
                                   vga_hw_invalidate_ptr invalidate,
                                   vga_hw_screen_dump_ptr screen_dump,
                                   vga_hw_text_update_ptr text_update,
                                   void *opaque);

void vga_hw_update(void);
void vga_hw_invalidate(void);
void vga_hw_screen_dump(const char *filename);
void vga_hw_text_update(console_ch_t *chardata);

int is_graphic_console(void);
int is_fixedsize_console(void);
CharDriverState *text_console_init(const char *p);
void text_consoles_set_display(DisplayState *ds);
void console_select(unsigned int index);
void console_color_init(DisplayState *ds);
void qemu_console_resize(DisplayState *ds, int width, int height);
void qemu_console_copy(DisplayState *ds, int src_x, int src_y,
                       int dst_x, int dst_y, int w, int h);

/* sdl.c */
void sdl_display_init(DisplayState *ds, int full_screen, int no_frame);

/* cocoa.m */
void cocoa_display_init(DisplayState *ds, int full_screen);

/* vnc.c */
void vnc_display_init(DisplayState *ds);
void vnc_display_close(DisplayState *ds);
int vnc_display_open(DisplayState *ds, const char *display);
int vnc_display_password(DisplayState *ds, const char *password);
void do_info_vnc(void);

/* curses.c */
void curses_display_init(DisplayState *ds, int full_screen);

/* x_keymap.c */
extern uint8_t _translate_keycode(const int key);

/* FIXME: term_printf et al should probably go elsewhere so everything
   does not need to include console.h  */
/* monitor.c */
void monitor_init(CharDriverState *hd, int show_banner);
void term_puts(const char *str);
void term_vprintf(const char *fmt, va_list ap);
void term_printf(const char *fmt, ...) __attribute__ ((__format__ (__printf__, 1, 2)));
void term_print_filename(const char *filename);
void term_flush(void);
void term_print_help(void);
void monitor_readline(const char *prompt, int is_password,
                      char *buf, int buf_size);
void monitor_suspend(void);
void monitor_resume(void);

/* readline.c */
typedef void ReadLineFunc(void *opaque, const char *str);

extern int completion_index;
void add_completion(const char *str);
void readline_handle_byte(int ch);
void readline_find_completion(const char *cmdline);
const char *readline_get_history(unsigned int index);
void readline_start(const char *prompt, int is_password,
                    ReadLineFunc *readline_func, void *opaque);

#endif
