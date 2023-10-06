/*-
 * Copyright (c) 2013
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
/*
 * QEMU stm32f2xx RTC emulation
 */
#pragma once

#include <stdio.h>
#include <inttypes.h>
#include "qemu/osdep.h"
#include "qapi/error.h"
//#include "qemu-common.h"
#include "qemu/log.h"
#include <sys/time.h>
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qemu/bcd.h"
#include "hw/arm/stm32fxxx.h"
#include "sysemu/rtc.h"
#include "qemu/cutils.h"

// Define this to add extra BKUP registers past the normal ones implemented by the STM.
// For Pebble emulation, we use these to pass settings flags to the emulated target
#define STM32F2XX_RTC_NUM_EXTRA_BKUP_REG  1


#define R_RTC_TR     (0x00 / 4)
#define R_RTC_DR     (0x04 / 4)
#define R_RTC_CR     (0x08 / 4)
#define R_RTC_CR_ALRAE_BIT 8
#define R_RTC_CR_WUTE   0x00000400
#define R_RTC_CR_WUTIE  0x00004000

#define R_RTC_ISR    (0x0c / 4)
#define R_RTC_ISR_RESET 0x00000007
#define R_RTC_ISR_RSF   0x00000020
#define R_RTC_ISR_WUT   0x00000400

#define R_RTC_PRER   (0x10 / 4)
#define R_RTC_PRER_PREDIV_A_MASK 0x7f
#define R_RTC_PRER_PREDIV_A_SHIFT 16
#define R_RTC_PRER_PREDIV_S_MASK 0x1fff
#define R_RTC_PRER_PREDIV_S_SHIFT 0
#define R_RTC_PRER_RESET 0x007f00ff
#define R_RTC_WUTR   (0x14 / 4)
#define R_RTC_WUTR_RESET 0x0000ffff
#define R_RTC_CALIBR (0x18 / 4)
#define R_RTC_ALRMAR (0x1c / 4)
#define R_RTC_ALRMBR (0x20 / 4)
#define R_RTC_WPR    (0x24 / 4)
#define R_RTC_SSR    (0x28 / 4)
#define R_RTC_TSTR   (0x30 / 4)
#define R_RTC_TSDR   (0x34 / 4)
#define R_RTC_TAFCR  (0x40 / 4)
#define R_RTC_BKPxR  (0x50 / 4)
#define R_RTC_BKPxR_LAST (0x9c / 4)
#define R_RTC_BKPxR_INC_EXTRA_LAST (R_RTC_BKPxR_LAST + STM32F2XX_RTC_NUM_EXTRA_BKUP_REG)
#define R_RTC_MAX    (R_RTC_BKPxR_INC_EXTRA_LAST + 1)

#define R_RTC_CR_FMT_MASK (0x01 << 6)

#define DEBUG_ALARM(x...)




#define TYPE_STM32FXXX_RTC "stm32f2xx-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F2XXRtcState, STM32FXXX_RTC)

// from
// f2xx_rtc
// STM32F2XXRtcState

typedef struct STM32F2XXRtcState {
    SysBusDevice  busdev;
    MemoryRegion  iomem;
    QEMUTimer     *timer;
    QEMUTimer     *wu_timer;
    qemu_irq      irq[2];
    qemu_irq      wut_irq;

    // target_us = host_us + host_to_target_offset_us
    int64_t       host_to_target_offset_us;

    // target time in ticks (seconds according to the RTC registers)
    time_t        ticks;

    uint32_t      regs[R_RTC_MAX];
    int           wp_count; /* Number of correct writes to WP reg */
} f2xx_rtc;


// Update target date and time from the host

