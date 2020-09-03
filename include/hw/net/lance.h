/*
 * QEMU Lance (Am7990) device emulation
 *
 * Copyright (c) 2004 Antony T Curtis
 * Copyright (c) 2017 Mark Cave-Ayland
 *
 * This represents the Sparc32 lance (Am7990) ethernet device which is an
 * earlier register-compatible member of the AMD PC-Net II (Am79C970A) family.
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

#ifndef LANCE_H
#define LANCE_H

#include "net/net.h"
#include "hw/net/pcnet.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_LANCE "lance"
typedef struct SysBusPCNetState SysBusPCNetState;
#define SYSBUS_PCNET(obj) \
    OBJECT_CHECK(SysBusPCNetState, (obj), TYPE_LANCE)

struct SysBusPCNetState {
    SysBusDevice parent_obj;

    PCNetState state;
};

#endif
