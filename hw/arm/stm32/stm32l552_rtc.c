/*-
 * Copyright (c) 2023
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
 * QEMU stm32l552 RTC emulation
 */
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
//#include "hw/arm/stm32fxxx.h"
#include "sysemu/rtc.h"
#include "qemu/cutils.h"
#include "hw/arm/stm32/stm32l552_rtc.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"


// from
// f2xx_rtc
#if 0
  uint32_t TR;      /*!< RTC time register,                                        Address offset: 0x00 */
  uint32_t DR;      /*!< RTC date register,                                        Address offset: 0x04 */
  uint32_t CR;      /*!< RTC control register,                                     Address offset: 0x08 */
  uint32_t ISR;     /*!< RTC initialization and status register,                   Address offset: 0x0C */
  uint32_t PRER;    /*!< RTC prescaler register,                                   Address offset: 0x10 */
  uint32_t WUTR;    /*!< RTC wakeup timer register,                                Address offset: 0x14 */
  uint32_t CALIBR;  /*!< RTC calibration register,                                 Address offset: 0x18 */
  uint32_t ALRMAR;  /*!< RTC alarm A register,                                     Address offset: 0x1C */
  uint32_t ALRMBR;  /*!< RTC alarm B register,                                     Address offset: 0x20 */
  uint32_t WPR;     /*!< RTC write protection register,                            Address offset: 0x24 */
  uint32_t SSR;     /*!< RTC sub second register,                                  Address offset: 0x28 */
  uint32_t SHIFTR;  /*!< RTC shift control register,                               Address offset: 0x2C */
  uint32_t TSTR;    /*!< RTC time stamp time register,                             Address offset: 0x30 */
  uint32_t TSDR;    /*!< RTC time stamp date register,                             Address offset: 0x34 */
  uint32_t TSSSR;   /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
  uint32_t CALR;    /*!< RTC calibration register,                                 Address offset: 0x3C */
  uint32_t TAFCR;   /*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
  uint32_t ALRMASSR;/*!< RTC alarm A sub second register,                          Address offset: 0x44 */
  uint32_t ALRMBSSR;/*!< RTC alarm B sub second register,                          Address offset: 0x48 */
  uint32_t RESERVED7;    /*!< Reserved, 0x4C                                                                 */
  uint32_t BKP0R;   /*!< RTC backup register 1,                                    Address offset: 0x50 */
  uint32_t BKP1R;   /*!< RTC backup register 1,                                    Address offset: 0x54 */
  uint32_t BKP2R;   /*!< RTC backup register 2,                                    Address offset: 0x58 */
  uint32_t BKP3R;   /*!< RTC backup register 3,                                    Address offset: 0x5C */
  uint32_t BKP4R;   /*!< RTC backup register 4,                                    Address offset: 0x60 */
  uint32_t BKP5R;   /*!< RTC backup register 5,                                    Address offset: 0x64 */
  uint32_t BKP6R;   /*!< RTC backup register 6,                                    Address offset: 0x68 */
  uint32_t BKP7R;   /*!< RTC backup register 7,                                    Address offset: 0x6C */
  uint32_t BKP8R;   /*!< RTC backup register 8,                                    Address offset: 0x70 */
  uint32_t BKP9R;   /*!< RTC backup register 9,                                    Address offset: 0x74 */
  uint32_t BKP10R;  /*!< RTC backup register 10,                                   Address offset: 0x78 */
  uint32_t BKP11R;  /*!< RTC backup register 11,                                   Address offset: 0x7C */
  uint32_t BKP12R;  /*!< RTC backup register 12,                                   Address offset: 0x80 */
  uint32_t BKP13R;  /*!< RTC backup register 13,                                   Address offset: 0x84 */
  uint32_t BKP14R;  /*!< RTC backup register 14,                                   Address offset: 0x88 */
  uint32_t BKP15R;  /*!< RTC backup register 15,                                   Address offset: 0x8C */
  uint32_t BKP16R;  /*!< RTC backup register 16,                                   Address offset: 0x90 */
  uint32_t BKP17R;  /*!< RTC backup register 17,                                   Address offset: 0x94 */
  uint32_t BKP18R;  /*!< RTC backup register 18,                                   Address offset: 0x98 */
  uint32_t BKP19R;  /*!< RTC backup register 19,                                   Address offset: 0x9C */

#endif
// STM32L552RtcState

static void l552_update_current_date_and_time(void *arg);

#define DEBUG_STM32F2XX_RTC
#ifdef DEBUG_STM32F2XX_RTC
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("DEBUG_STM32L552_RTC: " fmt , ## __VA_ARGS__); \
         usleep(100); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

void l552_rtc_set_extra_bkup_reg(void *opaque, uint32_t idx, uint32_t value);


// Compute the period for the clock (seconds increments) in nanoseconds
static uint64_t
l552_clock_period_ns(STM32L552RtcState *s)
{
    uint32_t prer = s->regs[R_RTC_PRER];
    unsigned int prescale;

    // NOTE: We are making the assumption here, as in l552_wut_period_ns, that RTC_CLK
    // is the 32768 LSE clock
    prescale = (((prer >> R_RTC_PRER_PREDIV_A_SHIFT) & R_RTC_PRER_PREDIV_A_MASK) + 1) *
               (((prer >> R_RTC_PRER_PREDIV_S_SHIFT) & R_RTC_PRER_PREDIV_S_MASK) + 1);
    uint64_t result = 1000000000LL * prescale / 32768;

    //DPRINTF("%s: period = %lldns\n", __func__, result);
    return result;
}


// Compute the period for the wakeup timer in nanoseconds if the WUT counter is set to the
// given value
static uint64_t
l552_wut_period_ns(STM32L552RtcState *s, uint32_t counter)
{
    uint32_t clock_sel = s->regs[R_RTC_CR] & 0x07;

    // NOTE: We are making the assumption here, as in l552_clock_period_ns, that RTC_CLK
    // is the 32768 LSE clock
    uint64_t rtc_clk_period_ns = 1000000000LL / 32768;

    // Using the synchronous clock (clk_spre)
    if (clock_sel & 0x04) {
        if (clock_sel & 0x02) {
            return counter * l552_clock_period_ns(s);
        } else {
            return (counter + 65536) * l552_clock_period_ns(s);
        }
    } else if (clock_sel == 0) {
        return counter * rtc_clk_period_ns * 16;
    } else if (clock_sel == 1) {
        return counter * rtc_clk_period_ns * 8;
    } else if (clock_sel == 2) {
        return counter * rtc_clk_period_ns * 4;
    } else if (clock_sel == 3) {
        return counter * rtc_clk_period_ns * 2;
    } else {
        abort();
    }
}


// Set the time and registers based on the content of the passed in tm struct
static void
l552_rtc_set_time_and_date_registers(STM32L552RtcState *s, struct tm *tm)
{
    uint8_t wday;

    wday = tm->tm_wday == 0 ? tm->tm_wday : 7;
    s->regs[R_RTC_TR] = to_bcd(tm->tm_sec) |
                        to_bcd(tm->tm_min) << 8 |
                        to_bcd(tm->tm_hour) << 16;
    s->regs[R_RTC_DR] = to_bcd(tm->tm_mday) |
                        to_bcd(tm->tm_mon + 1) << 8 |
                        wday << 13 |
                        to_bcd(tm->tm_year % 100) << 16;
}


// Compute what time we want in the target based on the host's current date and time.
// This takes into consideration the host_to_target_offset_us we have computed and captured
// previously.
// The rtc_period_ns passed in is the number of nanoseconds in host time that corresponds to 1
// "tick" (i.e. a second increment in the RTC register). Usually, you would expect this to
// be 1e9, but on Pebble plastic for example, the firmware sets up the RTC so that the
// seconds increment 1024 times/sec.
static time_t
l552_rtc_compute_target_time_from_host_time(STM32L552RtcState *s, uint64_t rtc_period_ns,
                                            struct tm *target_tm)
{
    // Get the host time in microseconds
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t host_time_us = tv.tv_sec * 1000000LL + (tv.tv_usec);

    // Compute the target time by adding the offset
    int64_t target_time_us = host_time_us + s->host_to_target_offset_us;

    // Convert to target ticks according period set in the RTC
    time_t target_time_ticks = (target_time_us * 1000) / rtc_period_ns;
    
    // Convert to date, hour, min, sec components
    gmtime_r(&target_time_ticks, target_tm);


#ifdef DEBUG_STM32F2XX_RTC
    char new_date_time[256];
    //strftime(new_date_time, sizeof(new_date_time), "%c", target_tm);

    //char s[1024];
    /* fool gcc to prevent a Y2K warning */
    char time_format[] = "_c";
    time_format[0] = '%';
    // localtime (&t)
    strftime (new_date_time, sizeof (new_date_time), time_format, target_tm);

    //DPRINTF("%s: setting new date & time to: %s\n", __func__, new_date_time);
#endif
    return target_time_ticks;
}


// Return the current date and time as stored in the RTC TR and DR registers in two forms:
// By filling in the passed in tm struct and by returning the UTC time in seconds.
static time_t
l552_rtc_get_current_target_time(STM32L552RtcState *s, struct tm *target_tm)
{
    memset(target_tm, 0, sizeof(*target_tm));

    // Fill in the target_tm from the contents of the time register and date registers
    uint32_t time_reg = s->regs[R_RTC_TR];

    target_tm->tm_hour = from_bcd((time_reg & 0x003F0000) >> 16);
    bool pm = (time_reg & 0x00800000) != 0;
    if (pm && (s->regs[R_RTC_CR] & R_RTC_CR_FMT_MASK)) {
        target_tm->tm_hour += 12;
    }
    target_tm->tm_min = from_bcd((time_reg & 0x00007F00) >> 8);
    target_tm->tm_sec = from_bcd(time_reg & 0x0000007F);

    uint32_t date_reg = s->regs[R_RTC_DR];
    target_tm->tm_mday = from_bcd(date_reg & 0x3f);
    target_tm->tm_mon = from_bcd((date_reg & 0x00001F00) >> 8) - 1;
    target_tm->tm_year = from_bcd((date_reg & 0x00FF0000) >> 16) + 100;

    // Have mktime fill in the remaining fields and return the UTC seconds as well
    return mktimegm(target_tm);
}



// Compute the host to target offset based on the passed in target ticks and the current
//  host time.
static int64_t
l552_rtc_compute_host_to_target_offset(STM32L552RtcState *s, int64_t period_ns, time_t target_ticks)
{
    // Convert target ticks to real clock microseconds
    int64_t target_time_us = target_ticks * period_ns / 1000;

    // Get the host time in microseconds
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t host_time_us = tv.tv_sec * 1000000LL + (tv.tv_usec);

    // Get the host to target offset in micro seconds
    return target_time_us - host_time_us;
}

#define RTC_ISR_INITF_Pos             (6U)                                     
#define RTC_ISR_INITF_Msk             (0x1UL << RTC_ISR_INITF_Pos)              /*!< 0x00000040 */
#define RTC_ISR_INITF                 RTC_ISR_INITF_Msk                        

#define RTC_ISR_WUTF_Pos             (2U)                                     
#define RTC_ISR_WUTF_Msk             (0x1UL << RTC_ISR_WUTF_Pos)              /*!< 0x00000004 */
#define RTC_ISR_WUTF                 RTC_ISR_WUTF_Msk


static uint64_t
l552_rtc_read(void *arg, hwaddr addr, unsigned int size)
{
    STM32L552RtcState *s = arg;
    uint32_t r;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_RTC_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read l552 rtc register 0x%x\n",
          (unsigned int)addr << 2);
        DPRINTF("  %s: result: 0\n", __func__);
        return 0;
    }

    // If reading the time or date register, make sure they are brought up to date first
    if (addr == R_RTC_TR || addr == R_RTC_DR) {
        l552_update_current_date_and_time(s);
    }

    uint32_t value = s->regs[addr];
    if (addr == R_RTC_ISR) {
        value |= R_RTC_ISR_RSF;
        // Hack for TSMV2. The RTC Wakeup timer write flag 

        /* Make sure RTC WUTWF flag is reset only when wakeup timer enabled */
        if ((s->regs[R_RTC_CR] & (0x1UL <<10U)) == 0) {
            // WUTWF should be set if the wakeup timer is enabled unless s->regs[R_RTC_ISR] & RTC_ISR_INITF

            
            if (s->regs[R_RTC_ISR] & RTC_ISR_INITF) {
                //value &= ~RTC_ISR_WUTF_Msk;
                value |= RTC_ISR_WUTF_Msk;
            }
        }
    }

    // HACK for Pebble. Clear the "entered standby" bit. If this bit is set, the Pebble
    // will only continue booting if one of the buttons is held down. Because the button GPIOs
    // are setup AFTER QEMU gets the key press event, it is difficult to set the GPIOs according
    // to the buttons held down before the reset. 
    if (addr == R_RTC_BKPxR) {
        value &= ~0x10000;
    }

    // If reading the sub-second register, determine what it should be from the current
    //  host time
    if (addr == R_RTC_SSR) {
        // How many ns are in in a full cycle of the subsecond counter?
        uint32_t prer = s->regs[R_RTC_PRER];;
        uint32_t prer_s = (prer >> R_RTC_PRER_PREDIV_S_SHIFT) & R_RTC_PRER_PREDIV_S_MASK;

        uint64_t full_cycle_us = l552_clock_period_ns(s) / 1000;

        // What fraction of a full cycle are we in?
        struct timeval tv;
        gettimeofday(&tv, NULL);
        int64_t host_time_us = tv.tv_sec * 1000000LL + (tv.tv_usec);
        host_time_us += s->host_to_target_offset_us;

        int64_t host_mod = host_time_us % full_cycle_us;
        double fract = (double)host_mod / full_cycle_us;

        // Compute the prescaler value
        value = prer_s - (fract * prer_s);
        DPRINTF("%s: SSR value = %d\n", __func__, value);
    }

    r = (value >> offset * 8) & ((1ull << (8 * size)) - 1);

#ifdef DEBUG_STM32F2XX_RTC
    // Debug printing
    if (addr == R_RTC_TR || addr == R_RTC_DR) {
        struct tm target_tm;
        l552_rtc_get_current_target_time(s, &target_tm);

        //char date_time_str[256];
        //strftime(date_time_str, sizeof(date_time_str), "%X", &target_tm);
        //DPRINTF("%s: current date/time: %s\n", __func__, date_time_str);
    } else {
        //DPRINTF("%s: addr: " PRIu64 ", size: %" PRIu64 ", value: " PRIu64 "\n", __func__, addr << 2, size, r);
        DPRINTF("%s: addr: %" PRIu64 ", data: %" PRIu64 ", size: %lu\n", __func__, addr,(long unsigned int) r,(long unsigned int) size);
    }
#endif
    if (addr==3) {
        static int times_c=0;
        if (times_c++>10) {
            r|=RTC_ISR_INITF;
        } 
        // RTC_ISR_INITF
        // r=0;
    }

    return r;
}

static void
l552_rtc_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    STM32L552RtcState *s = arg;
    int offset = addr & 0x3;
    bool    compute_new_target_offset = false;
    bool    update_wut = false;

    //DPRINTF("%s: addr: 0x%llx, data: 0x%llx, size: %d\n", __func__, addr, data, size);
    // Same but use PRIu64
    DPRINTF("%s: addr: %" PRIu64 ", data: %" PRIu64 ", size: %x\n", __func__, addr, data, size);

    addr >>= 2;
    if (addr >= R_RTC_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid write l552 rtc register 0x%x\n",
          (unsigned int)addr << 2);
        return;
    }

    /* Special case for write protect state machine. */
    if (addr == R_RTC_WPR) {
        if (offset > 0) {
            return;
        }
        data &= 0xff;
        if ((s->wp_count == 0 && data == 0xca) ||
          (s->wp_count == 1 && data == 0x53)) {
            s->wp_count++;
        } else {
            s->wp_count = 0;
        }
        s->regs[addr] = data;
        return;
    }

    switch(size) {
    case 1:
        data = (s->regs[addr] & ~(0xff << (offset * 8))) | data << (offset * 8);
        break;
    case 2:
        data = (s->regs[addr] & ~(0xffff << (offset * 8))) | data << (offset * 8);
        break;
    case 4:
        break;
    default:
        abort();
    }
    if (addr >= R_RTC_BKPxR && addr <= R_RTC_BKPxR_INC_EXTRA_LAST) {
        s->regs[addr] = data;
        return;
    }
    /* Write protect */
    if (s->wp_count < 2 && addr != R_RTC_TAFCR && addr != R_RTC_ISR
            && addr != R_RTC_WPR) {
        qemu_log_mask(LOG_GUEST_ERROR, "l552 rtc write reg 0x%x+%u without wp disable\n",
                      (unsigned int)addr << 2, offset);
        return;
    }
    switch(addr) {
    case R_RTC_TR:
    case R_RTC_DR:
        compute_new_target_offset = true;
        break;
    case R_RTC_CR:
        if ((data & R_RTC_CR_WUTE) != (s->regs[R_RTC_CR] & R_RTC_CR_WUTE)) {
            update_wut = true;
        }
        {
            int64_t delay_ns;

            if (data & 0x03) {
                // Assuming ck_apre is derived from RTCCLK and has some frequency
                // Let's say it's some constant CK_APRE_PERIOD_IN_NS for this example
                delay_ns = 62500 + 62500; // +1 RTCCLK cycle
            } else {
                delay_ns = 62500; // Just 1 RTCCLK cycle
            }
            timer_mod_ns(s->wut_delay_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns);
        }
        break;
    case R_RTC_ISR:
        if ((data & 1<<8) == 0 && (s->regs[R_RTC_ISR] & 1<<8) != 0) {
            DPRINTF("l552 rtc isr lowered\n");
            qemu_irq_lower(s->irq[0]);
        }
        if ((data & 1<<10) == 0 && (s->regs[R_RTC_ISR] & 1<<10) != 0) {
            DPRINTF("l552 rtc WUT isr lowered\n");
            qemu_irq_lower(s->wut_irq);
        }
        s->regs[R_RTC_ISR] = data;
        break;
    case R_RTC_PRER:
        /*
         * XXX currently updates upon next clock tick.  To do this properly we
         * would need to account for the time already elapsed, and then update
         * the timer for the remaining period.
         */
        break;
    case R_RTC_WUTR:
        update_wut = true;
        break;
    case R_RTC_ALRMAR:
    case R_RTC_ALRMBR:
        break;
        /*
    case R_RTC_TAFCR:
        if (data) {
            qemu_log_mask(LOG_UNIMP,
              "l552 rtc unimplemented write TAFCR+%u size %u val %u\n",
              offset, size, (unsigned int)data);
        }
    */
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "l552 rtc unimplemented write 0x%x+%u size %u val 0x%x\n",
          (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }
    s->regs[addr] = data;


    // Do we need to recompute the host to target offset?
    if (compute_new_target_offset) {
        struct tm target_tm;
        // Recompute ticks based on the modified contents of the TR and DR registers
        s->ticks = l552_rtc_get_current_target_time(s, &target_tm);
        // Update the host to target offset as well
        s->host_to_target_offset_us = l552_rtc_compute_host_to_target_offset(s,
        								l552_clock_period_ns(s), s->ticks);
    }

    // Do we need to update the timer for the wake-up-timer?
    if (update_wut) {
        if (s->regs[R_RTC_CR] & R_RTC_CR_WUTE) {
            int64_t elapsed = l552_wut_period_ns(s, s->regs[R_RTC_WUTR]);
            DPRINTF("%s: scheduling WUT to fire in %f ms\n", __func__, (float)elapsed/1000000.0);
            timer_mod(s->wu_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + elapsed);
        } else {
            DPRINTF("%s: Cancelling WUT\n", __func__);
            qemu_set_irq(s->wut_irq, 0);
            timer_del(s->wu_timer);
        }
    }
}


static bool
l552_alarm_match(STM32L552RtcState *s, uint32_t alarm_reg)
{
    uint32_t tr = s->regs[R_RTC_TR];

    if ((alarm_reg & (1<<7)) == 0 && (tr & 0x7f) != (alarm_reg & 0x7f)) {
        /* Seconds match requested, but do not match. */
        return false;
    }
    if ((alarm_reg & (1<<15)) == 0 && (tr & 0x7f00) != (alarm_reg & 0x7f00)) {
        /* Minutes match requested, but do not match. */
        return false;
    }
    if ((alarm_reg & (1<<23)) == 0 && (tr & 0x7f0000) != (alarm_reg & 0x7f0000)) {
        /* Hours match requested, but do not match. */
        return false;
    }
    if ((alarm_reg & (1<<31)) == 0) { /* Day match. */
        uint32_t dr = s->regs[R_RTC_DR];
        if (alarm_reg & (1<<30)) { /* Day is week day. */
            if (((alarm_reg>>24) & 0xf) != ((dr>>13) & 0xf)) {
                return false;
            }
        } else { /* Day is day of month. */
            if (((alarm_reg>>24) & 0x3f) != (dr & 0x3f)) {
                return false;
            }
        }
    }
    return true;
}

static void
l552_alarm_check(STM32L552RtcState *s, int unit)
{
    uint32_t cr = s->regs[R_RTC_CR];
    uint32_t isr = s->regs[R_RTC_ISR];

    if ((cr & 1<<(R_RTC_CR_ALRAE_BIT + unit)) == 0) {
        return; /* Not enabled. */
    }

    if ((isr & 1<<(R_RTC_CR_ALRAE_BIT + unit)) == 0) {
        if (l552_alarm_match(s, s->regs[R_RTC_ALRMAR + unit])) {
            isr |= 1<<(8 + unit);
            s->regs[R_RTC_ISR] = isr;
            DPRINTF("l552 rtc alarm activated 0x%x 0x%x\n", isr, cr);
        }
    }
	qemu_set_irq(s->irq[unit], cr & 1<<(12 + unit) && isr & 1<<(8 + unit));
}

// This method updates the current time and date registers to match the current
// host time. While it advances the target time, it checks for alarms that need to fire.
static void
l552_update_current_date_and_time(void *arg)
{
    STM32L552RtcState *s = arg;
    uint64_t period_ns = l552_clock_period_ns(s);
    
    struct tm new_target_tm;
    time_t new_target_ticks = l552_rtc_compute_target_time_from_host_time(s,
                                  period_ns, &new_target_tm);


    // Normally, we would advance the target ticks until we catch up to the host ticks and
    // check for an alarm at each tick. But, if the clocks got too far off (host or target
    // changed time), just jam in the new target time without checking alarms
    int delta = new_target_ticks - s->ticks;
    //DPRINTF("%s: advancing target by %d ticks\n", __func__, delta);
    if (delta < 0 || delta > 1000) {
        printf("DEBUG_STM32F2XX_RTC %s: detected %d mismatch between host and target ticks, "
              "jamming new host time into RTC without checking for alarms\n", __func__, delta);
        s->ticks = new_target_ticks;
        l552_rtc_set_time_and_date_registers(s, &new_target_tm);
    } else {
        while (s->ticks != new_target_ticks) {
            s->ticks += 1;
            gmtime_r(&s->ticks, &new_target_tm);
            l552_rtc_set_time_and_date_registers(s, &new_target_tm);

            l552_alarm_check(s, 0);
            l552_alarm_check(s, 1);
        }
    }

    // Reschedule tick timer to run one tick from now to check for alarms again
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_HOST) + period_ns);
}


// This timer runs on every tick (usually second)
static void
l552_timer(void *arg)
{
    STM32L552RtcState *s = arg;
    l552_update_current_date_and_time(s);
}


// This timer fires when the wake up time has expired
static void
l552_wu_timer(void *arg)
{
    STM32L552RtcState *s = arg;

    //DPRINTF("%s: fired\n", __func__);

    // Fire the interrupt?
    uint32_t cr = s->regs[R_RTC_CR];
    uint32_t isr = s->regs[R_RTC_ISR];

    // Make sure WUT is enabled
    if ( (cr & R_RTC_CR_WUTE) == 0 ) {
        return; /* Not enabled */
    }

    // If interrupt not already asserted, assert it
    if ( (isr & R_RTC_ISR_WUT) == 0 ) {
        isr |= R_RTC_ISR_WUT;
        s->regs[R_RTC_ISR] = isr;
        DPRINTF("l552 wakeup timer ISR activated 0x%x 0x%x\n", isr, cr);
    }

    qemu_set_irq(s->wut_irq, (cr & R_RTC_CR_WUTIE) && (isr & R_RTC_ISR_WUT));

    // Reschedule again
    int64_t elapsed = l552_wut_period_ns(s, s->regs[R_RTC_WUTR]);
    timer_mod(s->wu_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + elapsed);
}


// External interface to set the value of an "extra" backup register. This can be used to
// communicate emulator specific settings to the target
void l552_rtc_set_extra_bkup_reg(void *opaque, uint32_t idx, uint32_t value)
{
    STM32L552RtcState *s = (STM32L552RtcState *)opaque;

     // Copy in the extra backup registers that were specified via properties
    assert(idx < STM32F2XX_RTC_NUM_EXTRA_BKUP_REG);
    s->regs[R_RTC_BKPxR_LAST + 1 + idx] = value;
}



static const MemoryRegionOps l552_rtc_ops = {
    .read = l552_rtc_read,
    .write = l552_rtc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static void l552_rtc_reset(DeviceState *dev)
{
    struct STM32L552RtcState *s = OBJECT_CHECK(struct STM32L552RtcState, dev, "stm32l552-rtc");
    // STM32L552RtcState *s = FROM_SYSBUS(STM32L552RtcState, SYS_BUS_DEVICE(dev));
    s->regs[R_RTC_CR] = 0;
}


static void wut_delay_callback(void *opaque)
{
    STM32L552RtcState *s = (STM32L552RtcState *)opaque;

    if ((s->regs[R_RTC_CR] & R_RTC_CR_WUTE) == 0) {
        /* Set WUTWF */
        s->regs[R_RTC_ISR]  |= RTC_ISR_WUTF_Msk;
    } else {
        /* Clear WUTWF */

        s->regs[R_RTC_ISR]  &= ~RTC_ISR_WUTF_Msk;
    }

    /* Further handling if needed */
}



static void
l552_rtc_realize(DeviceState *dev, Error **errp)
{
    // STM32L552RtcState *s = FROM_SYSBUS(STM32L552RtcState, dev);
    struct STM32L552RtcState *s = OBJECT_CHECK(struct STM32L552RtcState, dev, "stm32l552-rtc");
    SysBusDevice *busdev = SYS_BUS_DEVICE(dev);


    memory_region_init_io(&s->iomem, OBJECT(s), &l552_rtc_ops, s, "rtc", 0x03ff);
    sysbus_init_mmio(busdev, &s->iomem);

    sysbus_init_irq(busdev, &s->irq[0]);
    sysbus_init_irq(busdev, &s->irq[1]);
    sysbus_init_irq(busdev, &s->wut_irq);

    s->regs[R_RTC_ISR] = R_RTC_ISR_RESET;
    s->regs[R_RTC_PRER] = R_RTC_PRER_RESET;
    s->regs[R_RTC_WUTR] = R_RTC_WUTR_RESET;

    uint32_t period_ns = l552_clock_period_ns(s);
    DPRINTF("%s: period: %d ns\n", __func__, period_ns);

    // Init the time and date registers from the time on the host as the default
    s->host_to_target_offset_us = 0;
    struct tm now;
    qemu_get_timedate(&now, 0);

    // Set time and date registers from the now struct
    l552_rtc_set_time_and_date_registers(s, &now);

    // Compute current ticks and host to target offset
    s->ticks = mktimegm(&now);
    s->host_to_target_offset_us = l552_rtc_compute_host_to_target_offset(s,
                                        l552_clock_period_ns(s), s->ticks);

    s->timer = timer_new_ns(QEMU_CLOCK_HOST, l552_timer, s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_HOST) + period_ns);

    s->wu_timer = timer_new_ns(QEMU_CLOCK_REALTIME, l552_wu_timer, s);

    s->wut_delay_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wut_delay_callback, s);
}

static Property l552_rtc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void
l552_rtc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    //SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(klass);
    dc->realize = l552_rtc_realize;
    //TODO: fix this: dc->no_user = 1;
    dc->props_ = l552_rtc_properties;
    dc->reset = l552_rtc_reset;
}

static const TypeInfo
l552_rtc_info = {
    .name          = "stm32l552-rtc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(l552_rtc),
    .class_init    = l552_rtc_class_init,
};

static void
l552_rtc_register_types(void)
{
    type_register_static(&l552_rtc_info);
}

type_init(l552_rtc_register_types)
