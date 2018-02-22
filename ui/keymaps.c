/*
 * QEMU keysym to keycode conversion using rdesktop keymaps
 *
 * Copyright (c) 2004 Johannes Schindelin
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

#include "qemu/osdep.h"
#include "keymaps.h"
#include "sysemu/sysemu.h"
#include "trace.h"
#include "qemu/error-report.h"

struct keysym2code {
    uint16_t keycode;
};

struct kbd_layout_t {
    GHashTable *hash;
};

static int get_keysym(const name2keysym_t *table,
                      const char *name)
{
    const name2keysym_t *p;
    for(p = table; p->name != NULL; p++) {
        if (!strcmp(p->name, name)) {
            return p->keysym;
        }
    }
    if (name[0] == 'U' && strlen(name) == 5) { /* try unicode Uxxxx */
        char *end;
        int ret = (int)strtoul(name + 1, &end, 16);
        if (*end == '\0' && ret > 0) {
            return ret;
        }
    }
    return 0;
}


static void add_keysym(char *line, int keysym, int keycode, kbd_layout_t *k)
{
    struct keysym2code *keysym2code;

    keysym2code = g_hash_table_lookup(k->hash, GINT_TO_POINTER(keysym));
    if (keysym2code) {
        return;
    }

    keysym2code = g_new0(struct keysym2code, 1);
    keysym2code->keycode = keycode;
    g_hash_table_replace(k->hash, GINT_TO_POINTER(keysym), keysym2code);
    trace_keymap_add(keysym, keycode, line);
}

static kbd_layout_t *parse_keyboard_layout(const name2keysym_t *table,
                                           const char *language,
                                           kbd_layout_t *k)
{
    FILE *f;
    char * filename;
    char line[1024];
    char keyname[64];
    int len;

    filename = qemu_find_file(QEMU_FILE_TYPE_KEYMAP, language);
    trace_keymap_parse(filename);
    f = filename ? fopen(filename, "r") : NULL;
    g_free(filename);
    if (!f) {
        fprintf(stderr, "Could not read keymap file: '%s'\n", language);
        return NULL;
    }

    if (!k) {
        k = g_new0(kbd_layout_t, 1);
        k->hash = g_hash_table_new(NULL, NULL);
    }

    for(;;) {
        if (fgets(line, 1024, f) == NULL) {
            break;
        }
        len = strlen(line);
        if (len > 0 && line[len - 1] == '\n') {
            line[len - 1] = '\0';
        }
        if (line[0] == '#') {
            continue;
        }
        if (!strncmp(line, "map ", 4)) {
            continue;
        }
        if (!strncmp(line, "include ", 8)) {
            parse_keyboard_layout(table, line + 8, k);
        } else {
            int offset = 0;
            while (line[offset] != 0 &&
                   line[offset] != ' ' &&
                   offset < sizeof(keyname) - 1) {
                keyname[offset] = line[offset];
                offset++;
            }
            keyname[offset] = 0;
            if (strlen(keyname)) {
                int keysym;
                keysym = get_keysym(table, keyname);
                if (keysym == 0) {
                    /* warn_report("unknown keysym %s", line);*/
                } else {
                    const char *rest = line + offset + 1;
                    int keycode = strtol(rest, NULL, 0);

                    if (strstr(rest, "shift")) {
                        keycode |= SCANCODE_SHIFT;
                    }
                    if (strstr(rest, "altgr")) {
                        keycode |= SCANCODE_ALTGR;
                    }
                    if (strstr(rest, "ctrl")) {
                        keycode |= SCANCODE_CTRL;
                    }

                    add_keysym(line, keysym, keycode, k);

                    if (strstr(rest, "addupper")) {
                        char *c;
                        for (c = keyname; *c; c++) {
                            *c = qemu_toupper(*c);
                        }
                        keysym = get_keysym(table, keyname);
                        if (keysym) {
                            add_keysym(line, keysym,
                                       keycode | SCANCODE_SHIFT, k);
                        }
                    }
                }
            }
        }
    }
    fclose(f);
    return k;
}


kbd_layout_t *init_keyboard_layout(const name2keysym_t *table,
                                   const char *language)
{
    return parse_keyboard_layout(table, language, NULL);
}


int keysym2scancode(kbd_layout_t *k, int keysym)
{
    struct keysym2code *keysym2code;

#ifdef XK_ISO_Left_Tab
    if (keysym == XK_ISO_Left_Tab) {
        keysym = XK_Tab;
    }
#endif

    keysym2code = g_hash_table_lookup(k->hash, GINT_TO_POINTER(keysym));
    if (!keysym2code) {
        trace_keymap_unmapped(keysym);
        warn_report("no scancode found for keysym %d", keysym);
        return 0;
    }

    return keysym2code->keycode;
}

int keycode_is_keypad(kbd_layout_t *k, int keycode)
{
    if (keycode >= 0x47 && keycode <= 0x53) {
        return true;
    }
    return false;
}

int keysym_is_numlock(kbd_layout_t *k, int keysym)
{
    switch (keysym) {
    case 0xffb0 ... 0xffb9:  /* KP_0 .. KP_9 */
    case 0xffac:             /* KP_Separator */
    case 0xffae:             /* KP_Decimal   */
        return true;
    }
    return false;
}
