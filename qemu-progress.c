/*
 * QEMU progress printing utility functions
 *
 * Copyright (C) 2011 Jes Sorensen <Jes.Sorensen@redhat.com>
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

#include "qemu-common.h"
#include "osdep.h"
#include "sysemu.h"
#include <stdio.h>

struct progress_state {
    int enabled;
    float current;
    float last_print;
    float min_skip;
};

static struct progress_state state;

/*
 * Simple progress print function.
 * @percent relative percent of current operation
 * @max percent of total operation
 */
static void progress_simple_print(void)
{
    if (state.enabled) {
        printf("    (%3.2f/100%%)\r", state.current);
        fflush(stdout);
    }
}

static void progress_simple_end(void)
{
    if (state.enabled) {
        printf("\n");
    }
}

void qemu_progress_init(int enabled, float min_skip)
{
    state.enabled = enabled;
    state.min_skip = min_skip;
}

void qemu_progress_end(void)
{
    progress_simple_end();
}

void qemu_progress_print(float percent, int max)
{
    float current;

    if (max == 0) {
        current = percent;
    } else {
        current = state.current + percent / 100 * max;
    }
    if (current > 100) {
        current = 100;
    }
    state.current = current;

    if (current > (state.last_print + state.min_skip) ||
        (current == 100) || (current == 0)) {
        state.last_print = state.current;
        progress_simple_print();
    }
}
