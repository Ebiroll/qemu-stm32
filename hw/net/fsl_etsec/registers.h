/*
 * QEMU Freescale eTSEC Emulator
 *
 * Copyright (c) 2011-2013 AdaCore
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

#ifndef ETSEC_REGISTERS_H
#define ETSEC_REGISTERS_H

enum eTSEC_Register_Access_Type {
    ACC_RW      = 1,            /* Read/Write */
    ACC_RO      = 2,            /* Read Only */
    ACC_WO      = 3,            /* Write Only */
    ACC_W1C     = 4,            /* Write 1 to clear */
    ACC_UNKNOWN = 5             /* Unknown register*/
};

typedef struct eTSEC_Register_Definition {
    uint32_t                         offset;
    const char                      *name;
    const char                      *desc;
    enum eTSEC_Register_Access_Type  access;
    uint32_t                         reset;
} eTSEC_Register_Definition;

extern const eTSEC_Register_Definition eTSEC_registers_def[];

#define DMACTRL_LE  (1 << 15)
#define DMACTRL_GRS (1 <<  4)
#define DMACTRL_GTS (1 <<  3)
#define DMACTRL_WOP (1 <<  0)

#define IEVENT_PERR  (1 <<  0)
#define IEVENT_DPE   (1 <<  1)
#define IEVENT_FIQ   (1 <<  2)
#define IEVENT_FIR   (1 <<  3)
#define IEVENT_FGPI  (1 <<  4)
#define IEVENT_RXF   (1 <<  7)
#define IEVENT_GRSC  (1 <<  8)
#define IEVENT_MMRW  (1 <<  9)
#define IEVENT_MMRD  (1 << 10)
#define IEVENT_MAG   (1 << 11)
#define IEVENT_RXB   (1 << 15)
#define IEVENT_XFUN  (1 << 16)
#define IEVENT_CRL   (1 << 17)
#define IEVENT_LC    (1 << 18)
#define IEVENT_TXF   (1 << 20)
#define IEVENT_TXB   (1 << 21)
#define IEVENT_TXE   (1 << 22)
#define IEVENT_TXC   (1 << 23)
#define IEVENT_BABT  (1 << 24)
#define IEVENT_GTSC  (1 << 25)
#define IEVENT_MSRO  (1 << 26)
#define IEVENT_EBERR (1 << 28)
#define IEVENT_BSY   (1 << 29)
#define IEVENT_RXC   (1 << 30)
#define IEVENT_BABR  (1 << 31)

/* Mapping between interrupt pin and interrupt flags */
#define IEVENT_RX_MASK (IEVENT_RXF | IEVENT_RXB)
#define IEVENT_TX_MASK (IEVENT_TXF | IEVENT_TXB)
#define IEVENT_ERR_MASK (IEVENT_MAG | IEVENT_GTSC | IEVENT_GRSC | IEVENT_TXC | \
    IEVENT_RXC | IEVENT_BABR | IEVENT_BABT | IEVENT_LC | \
    IEVENT_CRL | IEVENT_FGPI | IEVENT_FIR | IEVENT_FIQ | \
    IEVENT_DPE | IEVENT_PERR | IEVENT_EBERR | IEVENT_TXE | \
    IEVENT_XFUN | IEVENT_BSY | IEVENT_MSRO | IEVENT_MMRD | \
    IEVENT_MMRW)

#define IMASK_RXFEN  (1 <<  7)
#define IMASK_GRSCEN (1 <<  8)
#define IMASK_RXBEN  (1 << 15)
#define IMASK_TXFEN  (1 << 20)
#define IMASK_TXBEN  (1 << 21)
#define IMASK_GTSCEN (1 << 25)

#define MACCFG1_TX_EN  (1 << 0)
#define MACCFG1_RX_EN  (1 << 2)

#define MACCFG2_CRC_EN  (1 << 1)
#define MACCFG2_PADCRC  (1 << 2)

#define MIIMCOM_READ (1 << 0)
#define MIIMCOM_SCAN (1 << 1)

#define RCTRL_PRSDEP_MASK   (0x3)
#define RCTRL_PRSDEP_OFFSET (6)
#define RCTRL_RSF           (1 << 2)

/* Index of each register */

#define TSEC_ID      (0x000 / 4)
#define TSEC_ID2     (0x004 / 4)
#define IEVENT       (0x010 / 4)
#define IMASK        (0x014 / 4)
#define EDIS         (0x018 / 4)
#define ECNTRL       (0x020 / 4)
#define PTV          (0x028 / 4)
#define DMACTRL      (0x02C / 4)
#define TBIPA        (0x030 / 4)
#define TCTRL        (0x100 / 4)
#define TSTAT        (0x104 / 4)
#define DFVLAN       (0x108 / 4)
#define TXIC         (0x110 / 4)
#define TQUEUE       (0x114 / 4)
#define TR03WT       (0x140 / 4)
#define TR47WT       (0x144 / 4)
#define TBDBPH       (0x180 / 4)
#define TBPTR0       (0x184 / 4)
#define TBPTR1       (0x18C / 4)
#define TBPTR2       (0x194 / 4)
#define TBPTR3       (0x19C / 4)
#define TBPTR4       (0x1A4 / 4)
#define TBPTR5       (0x1AC / 4)
#define TBPTR6       (0x1B4 / 4)
#define TBPTR7       (0x1BC / 4)
#define TBASEH       (0x200 / 4)
#define TBASE0       (0x204 / 4)
#define TBASE1       (0x20C / 4)
#define TBASE2       (0x214 / 4)
#define TBASE3       (0x21C / 4)
#define TBASE4       (0x224 / 4)
#define TBASE5       (0x22C / 4)
#define TBASE6       (0x234 / 4)
#define TBASE7       (0x23C / 4)
#define TMR_TXTS1_ID (0x280 / 4)
#define TMR_TXTS2_ID (0x284 / 4)
#define TMR_TXTS1_H  (0x2C0 / 4)
#define TMR_TXTS1_L  (0x2C4 / 4)
#define TMR_TXTS2_H  (0x2C8 / 4)
#define TMR_TXTS2_L  (0x2CC / 4)
#define RCTRL        (0x300 / 4)
#define RSTAT        (0x304 / 4)
#define RXIC         (0x310 / 4)
#define RQUEUE       (0x314 / 4)
#define RBIFX        (0x330 / 4)
#define RQFAR        (0x334 / 4)
#define RQFCR        (0x338 / 4)
#define RQFPR        (0x33C / 4)
#define MRBLR        (0x340 / 4)
#define RBDBPH       (0x380 / 4)
#define RBPTR0       (0x384 / 4)
#define RBPTR1       (0x38C / 4)
#define RBPTR2       (0x394 / 4)
#define RBPTR3       (0x39C / 4)
#define RBPTR4       (0x3A4 / 4)
#define RBPTR5       (0x3AC / 4)
#define RBPTR6       (0x3B4 / 4)
#define RBPTR7       (0x3BC / 4)
#define RBASEH       (0x400 / 4)
#define RBASE0       (0x404 / 4)
#define RBASE1       (0x40C / 4)
#define RBASE2       (0x414 / 4)
#define RBASE3       (0x41C / 4)
#define RBASE4       (0x424 / 4)
#define RBASE5       (0x42C / 4)
#define RBASE6       (0x434 / 4)
#define RBASE7       (0x43C / 4)
#define TMR_RXTS_H   (0x4C0 / 4)
#define TMR_RXTS_L   (0x4C4 / 4)
#define MACCFG1      (0x500 / 4)
#define MACCFG2      (0x504 / 4)
#define IPGIFG       (0x508 / 4)
#define HAFDUP       (0x50C / 4)
#define MAXFRM       (0x510 / 4)
#define MIIMCFG      (0x520 / 4)
#define MIIMCOM      (0x524 / 4)
#define MIIMADD      (0x528 / 4)
#define MIIMCON      (0x52C / 4)
#define MIIMSTAT     (0x530 / 4)
#define MIIMIND      (0x534 / 4)
#define IFSTAT       (0x53C / 4)
#define MACSTNADDR1  (0x540 / 4)
#define MACSTNADDR2  (0x544 / 4)
#define MAC01ADDR1   (0x548 / 4)
#define MAC01ADDR2   (0x54C / 4)
#define MAC02ADDR1   (0x550 / 4)
#define MAC02ADDR2   (0x554 / 4)
#define MAC03ADDR1   (0x558 / 4)
#define MAC03ADDR2   (0x55C / 4)
#define MAC04ADDR1   (0x560 / 4)
#define MAC04ADDR2   (0x564 / 4)
#define MAC05ADDR1   (0x568 / 4)
#define MAC05ADDR2   (0x56C / 4)
#define MAC06ADDR1   (0x570 / 4)
#define MAC06ADDR2   (0x574 / 4)
#define MAC07ADDR1   (0x578 / 4)
#define MAC07ADDR2   (0x57C / 4)
#define MAC08ADDR1   (0x580 / 4)
#define MAC08ADDR2   (0x584 / 4)
#define MAC09ADDR1   (0x588 / 4)
#define MAC09ADDR2   (0x58C / 4)
#define MAC10ADDR1   (0x590 / 4)
#define MAC10ADDR2   (0x594 / 4)
#define MAC11ADDR1   (0x598 / 4)
#define MAC11ADDR2   (0x59C / 4)
#define MAC12ADDR1   (0x5A0 / 4)
#define MAC12ADDR2   (0x5A4 / 4)
#define MAC13ADDR1   (0x5A8 / 4)
#define MAC13ADDR2   (0x5AC / 4)
#define MAC14ADDR1   (0x5B0 / 4)
#define MAC14ADDR2   (0x5B4 / 4)
#define MAC15ADDR1   (0x5B8 / 4)
#define MAC15ADDR2   (0x5BC / 4)
#define TR64         (0x680 / 4)
#define TR127        (0x684 / 4)
#define TR255        (0x688 / 4)
#define TR511        (0x68C / 4)
#define TR1K         (0x690 / 4)
#define TRMAX        (0x694 / 4)
#define TRMGV        (0x698 / 4)
#define RBYT         (0x69C / 4)
#define RPKT         (0x6A0 / 4)
#define RFCS         (0x6A4 / 4)
#define RMCA         (0x6A8 / 4)
#define RBCA         (0x6AC / 4)
#define RXCF         (0x6B0 / 4)
#define RXPF         (0x6B4 / 4)
#define RXUO         (0x6B8 / 4)
#define RALN         (0x6BC / 4)
#define RFLR         (0x6C0 / 4)
#define RCDE         (0x6C4 / 4)
#define RCSE         (0x6C8 / 4)
#define RUND         (0x6CC / 4)
#define ROVR         (0x6D0 / 4)
#define RFRG         (0x6D4 / 4)
#define RJBR         (0x6D8 / 4)
#define RDRP         (0x6DC / 4)
#define TBYT         (0x6E0 / 4)
#define TPKT         (0x6E4 / 4)
#define TMCA         (0x6E8 / 4)
#define TBCA         (0x6EC / 4)
#define TXPF         (0x6F0 / 4)
#define TDFR         (0x6F4 / 4)
#define TEDF         (0x6F8 / 4)
#define TSCL         (0x6FC / 4)
#define TMCL         (0x700 / 4)
#define TLCL         (0x704 / 4)
#define TXCL         (0x708 / 4)
#define TNCL         (0x70C / 4)
#define TDRP         (0x714 / 4)
#define TJBR         (0x718 / 4)
#define TFCS         (0x71C / 4)
#define TXCF         (0x720 / 4)
#define TOVR         (0x724 / 4)
#define TUND         (0x728 / 4)
#define TFRG         (0x72C / 4)
#define CAR1         (0x730 / 4)
#define CAR2         (0x734 / 4)
#define CAM1         (0x738 / 4)
#define CAM2         (0x73C / 4)
#define RREJ         (0x740 / 4)
#define IGADDR0      (0x800 / 4)
#define IGADDR1      (0x804 / 4)
#define IGADDR2      (0x808 / 4)
#define IGADDR3      (0x80C / 4)
#define IGADDR4      (0x810 / 4)
#define IGADDR5      (0x814 / 4)
#define IGADDR6      (0x818 / 4)
#define IGADDR7      (0x81C / 4)
#define GADDR0       (0x880 / 4)
#define GADDR1       (0x884 / 4)
#define GADDR2       (0x888 / 4)
#define GADDR3       (0x88C / 4)
#define GADDR4       (0x890 / 4)
#define GADDR5       (0x894 / 4)
#define GADDR6       (0x898 / 4)
#define GADDR7       (0x89C / 4)
#define ATTR         (0xBF8 / 4)
#define ATTRELI      (0xBFC / 4)
#define RQPRM0       (0xC00 / 4)
#define RQPRM1       (0xC04 / 4)
#define RQPRM2       (0xC08 / 4)
#define RQPRM3       (0xC0C / 4)
#define RQPRM4       (0xC10 / 4)
#define RQPRM5       (0xC14 / 4)
#define RQPRM6       (0xC18 / 4)
#define RQPRM7       (0xC1C / 4)
#define RFBPTR0      (0xC44 / 4)
#define RFBPTR1      (0xC4C / 4)
#define RFBPTR2      (0xC54 / 4)
#define RFBPTR3      (0xC5C / 4)
#define RFBPTR4      (0xC64 / 4)
#define RFBPTR5      (0xC6C / 4)
#define RFBPTR6      (0xC74 / 4)
#define RFBPTR7      (0xC7C / 4)
#define TMR_CTRL     (0xE00 / 4)
#define TMR_TEVENT   (0xE04 / 4)
#define TMR_TEMASK   (0xE08 / 4)
#define TMR_PEVENT   (0xE0C / 4)
#define TMR_PEMASK   (0xE10 / 4)
#define TMR_STAT     (0xE14 / 4)
#define TMR_CNT_H    (0xE18 / 4)
#define TMR_CNT_L    (0xE1C / 4)
#define TMR_ADD      (0xE20 / 4)
#define TMR_ACC      (0xE24 / 4)
#define TMR_PRSC     (0xE28 / 4)
#define TMROFF_H     (0xE30 / 4)
#define TMROFF_L     (0xE34 / 4)
#define TMR_ALARM1_H (0xE40 / 4)
#define TMR_ALARM1_L (0xE44 / 4)
#define TMR_ALARM2_H (0xE48 / 4)
#define TMR_ALARM2_L (0xE4C / 4)
#define TMR_FIPER1   (0xE80 / 4)
#define TMR_FIPER2   (0xE84 / 4)
#define TMR_FIPER3   (0xE88 / 4)
#define TMR_ETTS1_H  (0xEA0 / 4)
#define TMR_ETTS1_L  (0xEA4 / 4)
#define TMR_ETTS2_H  (0xEA8 / 4)
#define TMR_ETTS2_L  (0xEAC / 4)

#endif /* ETSEC_REGISTERS_H */
