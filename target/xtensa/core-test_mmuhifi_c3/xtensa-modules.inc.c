/* Xtensa configuration-specific ISA information.
   Copyright 2003, 2004, 2005, 2007, 2008 Free Software Foundation, Inc.

   This file is part of BFD, the Binary File Descriptor library.

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation; either version 3 of the
   License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street - Fifth Floor, Boston, MA
   02110-1301, USA.  */

#include "qemu/osdep.h"
#include "xtensa-isa.h"
#include "xtensa-isa-internal.h"


/* Sysregs.  */

static xtensa_sysreg_internal sysregs[] = {
  { "LBEG", 0, 0 },
  { "LEND", 1, 0 },
  { "LCOUNT", 2, 0 },
  { "BR", 4, 0 },
  { "PTEVADDR", 83, 0 },
  { "DDR", 104, 0 },
  { "176", 176, 0 },
  { "208", 208, 0 },
  { "INTERRUPT", 226, 0 },
  { "INTCLEAR", 227, 0 },
  { "CCOUNT", 234, 0 },
  { "PRID", 235, 0 },
  { "ICOUNT", 236, 0 },
  { "CCOMPARE0", 240, 0 },
  { "CCOMPARE1", 241, 0 },
  { "VECBASE", 231, 0 },
  { "EPC1", 177, 0 },
  { "EPC2", 178, 0 },
  { "EXCSAVE1", 209, 0 },
  { "EXCSAVE2", 210, 0 },
  { "EPS2", 194, 0 },
  { "EXCCAUSE", 232, 0 },
  { "DEPC", 192, 0 },
  { "EXCVADDR", 238, 0 },
  { "WINDOWBASE", 72, 0 },
  { "WINDOWSTART", 73, 0 },
  { "SAR", 3, 0 },
  { "LITBASE", 5, 0 },
  { "PS", 230, 0 },
  { "MISC0", 244, 0 },
  { "MISC1", 245, 0 },
  { "INTENABLE", 228, 0 },
  { "ICOUNTLEVEL", 237, 0 },
  { "DEBUGCAUSE", 233, 0 },
  { "RASID", 90, 0 },
  { "ITLBCFG", 91, 0 },
  { "DTLBCFG", 92, 0 },
  { "CPENABLE", 224, 0 },
  { "SCOMPARE1", 12, 0 },
  { "ATOMCTL", 99, 0 },
  { "THREADPTR", 231, 1 },
  { "AE_OVF_SAR", 240, 1 },
  { "AE_BITHEAD", 241, 1 },
  { "AE_TS_FTS_BU_BP", 242, 1 },
  { "AE_SD_NO", 243, 1 }
};

#define NUM_SYSREGS 45
#define MAX_SPECIAL_REG 245
#define MAX_USER_REG 243


/* Processor states.  */

static xtensa_state_internal states[] = {
  { "LCOUNT", 32, 0 },
  { "PC", 32, 0 },
  { "ICOUNT", 32, 0 },
  { "DDR", 32, 0 },
  { "INTERRUPT", 12, 0 },
  { "CCOUNT", 32, 0 },
  { "XTSYNC", 1, 0 },
  { "VECBASE", 22, 0 },
  { "EPC1", 32, 0 },
  { "EPC2", 32, 0 },
  { "EXCSAVE1", 32, 0 },
  { "EXCSAVE2", 32, 0 },
  { "EPS2", 15, 0 },
  { "EXCCAUSE", 6, 0 },
  { "PSINTLEVEL", 4, 0 },
  { "PSUM", 1, 0 },
  { "PSWOE", 1, 0 },
  { "PSRING", 2, 0 },
  { "PSEXCM", 1, 0 },
  { "DEPC", 32, 0 },
  { "EXCVADDR", 32, 0 },
  { "WindowBase", 3, 0 },
  { "WindowStart", 8, 0 },
  { "PSCALLINC", 2, 0 },
  { "PSOWB", 4, 0 },
  { "LBEG", 32, 0 },
  { "LEND", 32, 0 },
  { "SAR", 6, 0 },
  { "THREADPTR", 32, 0 },
  { "LITBADDR", 20, 0 },
  { "LITBEN", 1, 0 },
  { "MISC0", 32, 0 },
  { "MISC1", 32, 0 },
  { "InOCDMode", 1, 0 },
  { "INTENABLE", 12, 0 },
  { "ICOUNTLEVEL", 4, 0 },
  { "DEBUGCAUSE", 6, 0 },
  { "DBNUM", 4, 0 },
  { "CCOMPARE0", 32, 0 },
  { "CCOMPARE1", 32, 0 },
  { "ASID3", 8, 0 },
  { "ASID2", 8, 0 },
  { "ASID1", 8, 0 },
  { "INSTPGSZID4", 2, 0 },
  { "DATAPGSZID4", 2, 0 },
  { "PTBASE", 10, 0 },
  { "CPENABLE", 2, 0 },
  { "SCOMPARE1", 32, 0 },
  { "ATOMCTL", 6, 0 },
  { "CCON", 1, XTENSA_STATE_IS_EXPORTED },
  { "MPSCORE", 16, XTENSA_STATE_IS_EXPORTED },
  { "WMPINT_ADDR", 12, XTENSA_STATE_IS_EXPORTED },
  { "WMPINT_DATA", 32, XTENSA_STATE_IS_EXPORTED },
  { "WMPINT_TOGGLEEN", 1, XTENSA_STATE_IS_EXPORTED },
  { "AE_OVERFLOW", 1, 0 },
  { "AE_SAR", 6, 0 },
  { "AE_BITHEAD", 32, 0 },
  { "AE_BITPTR", 4, 0 },
  { "AE_BITSUSED", 4, 0 },
  { "AE_TABLESIZE", 4, 0 },
  { "AE_FIRST_TS", 4, 0 },
  { "AE_NEXTOFFSET", 27, 0 },
  { "AE_SEARCHDONE", 1, 0 }
};

#define NUM_STATES 63

enum xtensa_state_id {
  STATE_LCOUNT,
  STATE_PC,
  STATE_ICOUNT,
  STATE_DDR,
  STATE_INTERRUPT,
  STATE_CCOUNT,
  STATE_XTSYNC,
  STATE_VECBASE,
  STATE_EPC1,
  STATE_EPC2,
  STATE_EXCSAVE1,
  STATE_EXCSAVE2,
  STATE_EPS2,
  STATE_EXCCAUSE,
  STATE_PSINTLEVEL,
  STATE_PSUM,
  STATE_PSWOE,
  STATE_PSRING,
  STATE_PSEXCM,
  STATE_DEPC,
  STATE_EXCVADDR,
  STATE_WindowBase,
  STATE_WindowStart,
  STATE_PSCALLINC,
  STATE_PSOWB,
  STATE_LBEG,
  STATE_LEND,
  STATE_SAR,
  STATE_THREADPTR,
  STATE_LITBADDR,
  STATE_LITBEN,
  STATE_MISC0,
  STATE_MISC1,
  STATE_InOCDMode,
  STATE_INTENABLE,
  STATE_ICOUNTLEVEL,
  STATE_DEBUGCAUSE,
  STATE_DBNUM,
  STATE_CCOMPARE0,
  STATE_CCOMPARE1,
  STATE_ASID3,
  STATE_ASID2,
  STATE_ASID1,
  STATE_INSTPGSZID4,
  STATE_DATAPGSZID4,
  STATE_PTBASE,
  STATE_CPENABLE,
  STATE_SCOMPARE1,
  STATE_ATOMCTL,
  STATE_CCON,
  STATE_MPSCORE,
  STATE_WMPINT_ADDR,
  STATE_WMPINT_DATA,
  STATE_WMPINT_TOGGLEEN,
  STATE_AE_OVERFLOW,
  STATE_AE_SAR,
  STATE_AE_BITHEAD,
  STATE_AE_BITPTR,
  STATE_AE_BITSUSED,
  STATE_AE_TABLESIZE,
  STATE_AE_FIRST_TS,
  STATE_AE_NEXTOFFSET,
  STATE_AE_SEARCHDONE
};


/* Field definitions.  */

static unsigned
Field_t_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 24) >> 28);
  return tie_t;
}

static void
Field_t_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf0) | (tie_t << 4);
}

static unsigned
Field_s_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  return tie_t;
}

static void
Field_s_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
}

static unsigned
Field_r_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_r_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_op2_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 8) >> 28);
  return tie_t;
}

static void
Field_op2_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf00000) | (tie_t << 20);
}

static unsigned
Field_op1_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 12) >> 28);
  return tie_t;
}

static void
Field_op1_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf0000) | (tie_t << 16);
}

static unsigned
Field_op0_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_op0_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
}

static unsigned
Field_n_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_n_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
}

static unsigned
Field_m_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  return tie_t;
}

static void
Field_m_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
}

static unsigned
Field_sr_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  return tie_t;
}

static void
Field_sr_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
  tie_t = (val << 24) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_st_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  tie_t = (tie_t << 4) | ((insn[0] << 24) >> 28);
  return tie_t;
}

static void
Field_st_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf0) | (tie_t << 4);
  tie_t = (val << 24) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
}

static unsigned
Field_thi3_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 24) >> 29);
  return tie_t;
}

static void
Field_thi3_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe0) | (tie_t << 5);
}

static unsigned
Field_ae_r3_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 16) >> 31);
  return tie_t;
}

static void
Field_ae_r3_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8000) | (tie_t << 15);
}

static unsigned
Field_ae_r10_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 18) >> 30);
  return tie_t;
}

static void
Field_ae_r10_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x3000) | (tie_t << 12);
}

static unsigned
Field_ae_r32_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  return tie_t;
}

static void
Field_ae_r32_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ae_s3_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 20) >> 31);
  return tie_t;
}

static void
Field_ae_s3_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x800) | (tie_t << 11);
}

static unsigned
Field_ae_s_non_samt_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 20) >> 30);
  return tie_t;
}

static void
Field_ae_s_non_samt_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc00) | (tie_t << 10);
}

static unsigned
Field_op0_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_op0_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
}

static unsigned
Field_t_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 24) >> 28);
  return tie_t;
}

static void
Field_t_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf0) | (tie_t << 4);
}

static unsigned
Field_r_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_r_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_op0_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_op0_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
}

static unsigned
Field_z_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 25) >> 31);
  return tie_t;
}

static void
Field_z_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x40) | (tie_t << 6);
}

static unsigned
Field_i_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_i_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
}

static unsigned
Field_s_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  return tie_t;
}

static void
Field_s_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
}

static unsigned
Field_ftsf61ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 1) | ((insn[0] << 23) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf61ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x100) | (tie_t << 8);
  tie_t = (val << 22) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_op0_s3_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 7) | ((insn[0] << 9) >> 25);
  return tie_t;
}

static void
Field_op0_s3_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f0000) | (tie_t << 16);
}

static unsigned
Field_ftsf330ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 21) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 24) >> 28);
  return tie_t;
}

static void
Field_ftsf330ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf0) | (tie_t << 4);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x600) | (tie_t << 9);
}

static unsigned
Field_ftsf81ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 9) | ((insn[0] << 16) >> 23);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf81ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 22) >> 23;
  insn[0] = (insn[0] & ~0xff80) | (tie_t << 7);
}

static unsigned
Field_ae_r20_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 25) >> 29);
  return tie_t;
}

static void
Field_ae_r20_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x70) | (tie_t << 4);
}

static unsigned
Field_ftsf73ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 9) | ((insn[0] << 16) >> 23);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf73ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 22) >> 23;
  insn[0] = (insn[0] & ~0xff80) | (tie_t << 7);
}

static unsigned
Field_ftsf35ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf35ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf34ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf34ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf32ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf32ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf33ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf33ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf96ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 21) >> 30);
  return tie_t;
}

static void
Field_ftsf96ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x600) | (tie_t << 9);
  tie_t = (val << 28) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ae_s20_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 29) >> 29);
  return tie_t;
}

static void
Field_ae_s20_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x7) | (tie_t << 0);
}

static unsigned
Field_ftsf94ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 21) >> 30);
  tie_t = (tie_t << 1) | ((insn[0] << 29) >> 31);
  return tie_t;
}

static void
Field_ftsf94ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x4) | (tie_t << 2);
  tie_t = (val << 29) >> 30;
  insn[0] = (insn[0] & ~0x600) | (tie_t << 9);
  tie_t = (val << 27) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf347_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 30) >> 30);
  return tie_t;
}

static void
Field_ftsf347_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x3) | (tie_t << 0);
}

static unsigned
Field_ftsf24ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  return tie_t;
}

static void
Field_ftsf24ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf23ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  return tie_t;
}

static void
Field_ftsf23ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf125ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 18) >> 30);
  return tie_t;
}

static void
Field_ftsf125ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x3000) | (tie_t << 12);
}

static unsigned
Field_ftsf350ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 20) >> 29);
  tie_t = (tie_t << 4) | ((insn[0] << 25) >> 28);
  return tie_t;
}

static void
Field_ftsf350ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0x78) | (tie_t << 3);
  tie_t = (val << 25) >> 29;
  insn[0] = (insn[0] & ~0xe00) | (tie_t << 9);
}

static unsigned
Field_ftsf80ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 9) | ((insn[0] << 16) >> 23);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf80ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 22) >> 23;
  insn[0] = (insn[0] & ~0xff80) | (tie_t << 7);
}

static unsigned
Field_ftsf88ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 7) | ((insn[0] << 16) >> 25);
  tie_t = (tie_t << 1) | ((insn[0] << 25) >> 31);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf88ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 30) >> 31;
  insn[0] = (insn[0] & ~0x40) | (tie_t << 6);
  tie_t = (val << 23) >> 25;
  insn[0] = (insn[0] & ~0xfe00) | (tie_t << 9);
}

static unsigned
Field_ftsf340_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_ftsf340_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
}

static unsigned
Field_ftsf87ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 7) | ((insn[0] << 16) >> 25);
  tie_t = (tie_t << 2) | ((insn[0] << 25) >> 30);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf87ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 29) >> 30;
  insn[0] = (insn[0] & ~0x60) | (tie_t << 5);
  tie_t = (val << 22) >> 25;
  insn[0] = (insn[0] & ~0xfe00) | (tie_t << 9);
}

static unsigned
Field_ftsf342ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 27) >> 31);
  return tie_t;
}

static void
Field_ftsf342ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x10) | (tie_t << 4);
}

static unsigned
Field_ftsf86ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 7) | ((insn[0] << 16) >> 25);
  tie_t = (tie_t << 4) | ((insn[0] << 25) >> 28);
  return tie_t;
}

static void
Field_ftsf86ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0x78) | (tie_t << 3);
  tie_t = (val << 21) >> 25;
  insn[0] = (insn[0] & ~0xfe00) | (tie_t << 9);
}

static unsigned
Field_ftsf84ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 7) | ((insn[0] << 16) >> 25);
  tie_t = (tie_t << 4) | ((insn[0] << 25) >> 28);
  return tie_t;
}

static void
Field_ftsf84ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0x78) | (tie_t << 3);
  tie_t = (val << 21) >> 25;
  insn[0] = (insn[0] & ~0xfe00) | (tie_t << 9);
}

static unsigned
Field_ftsf76ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 9) | ((insn[0] << 16) >> 23);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf76ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 22) >> 23;
  insn[0] = (insn[0] & ~0xff80) | (tie_t << 7);
}

static unsigned
Field_ftsf75ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 9) | ((insn[0] << 16) >> 23);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf75ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 22) >> 23;
  insn[0] = (insn[0] & ~0xff80) | (tie_t << 7);
}

static unsigned
Field_ftsf60ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf60ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 21) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf64ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 7) | ((insn[0] << 25) >> 25);
  return tie_t;
}

static void
Field_ftsf64ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f) | (tie_t << 0);
  tie_t = (val << 20) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf63ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf63ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ae_r10_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  return tie_t;
}

static void
Field_ae_r10_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
}

static unsigned
Field_ftsf59ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf59ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 21) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf119ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 1) | ((insn[0] << 23) >> 31);
  tie_t = (tie_t << 7) | ((insn[0] << 25) >> 25);
  return tie_t;
}

static void
Field_ftsf119ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f) | (tie_t << 0);
  tie_t = (val << 24) >> 31;
  insn[0] = (insn[0] & ~0x100) | (tie_t << 8);
  tie_t = (val << 21) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf338_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf338_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
}

static unsigned
Field_ftsf69ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 1) | ((insn[0] << 25) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf69ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x40) | (tie_t << 6);
  tie_t = (val << 22) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf67ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 2) | ((insn[0] << 25) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf67ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x60) | (tie_t << 5);
  tie_t = (val << 21) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf66ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 7) | ((insn[0] << 25) >> 25);
  return tie_t;
}

static void
Field_ftsf66ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f) | (tie_t << 0);
  tie_t = (val << 20) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf25ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf25ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf36ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf36ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf103ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 21) >> 30);
  return tie_t;
}

static void
Field_ftsf103ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x600) | (tie_t << 9);
  tie_t = (val << 28) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf349ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 6) | ((insn[0] << 23) >> 26);
  return tie_t;
}

static void
Field_ftsf349ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 26) >> 26;
  insn[0] = (insn[0] & ~0x1f8) | (tie_t << 3);
}

static unsigned
Field_ftsf99ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf99ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf27ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf27ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf28ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf28ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf21ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 21) >> 30);
  return tie_t;
}

static void
Field_ftsf21ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x600) | (tie_t << 9);
  tie_t = (val << 28) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf22ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 21) >> 30);
  return tie_t;
}

static void
Field_ftsf22ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x600) | (tie_t << 9);
  tie_t = (val << 28) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf29ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf29ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf97ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf97ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf100ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf100ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf101ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 3) | ((insn[0] << 21) >> 29);
  return tie_t;
}

static void
Field_ftsf101ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x700) | (tie_t << 8);
  tie_t = (val << 27) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf348ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 24) >> 27);
  return tie_t;
}

static void
Field_ftsf348ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 27) >> 27;
  insn[0] = (insn[0] & ~0xf8) | (tie_t << 3);
}

static unsigned
Field_ftsf26ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf26ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf30ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf30ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf31ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf31ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf98ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf98ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf92ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 21) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 29) >> 30);
  return tie_t;
}

static void
Field_ftsf92ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x6) | (tie_t << 1);
  tie_t = (val << 28) >> 30;
  insn[0] = (insn[0] & ~0x600) | (tie_t << 9);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf208_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 31) >> 31);
  return tie_t;
}

static void
Field_ftsf208_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x1) | (tie_t << 0);
}

static unsigned
Field_ftsf91ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 21) >> 30);
  tie_t = (tie_t << 3) | ((insn[0] << 29) >> 29);
  return tie_t;
}

static void
Field_ftsf91ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x7) | (tie_t << 0);
  tie_t = (val << 27) >> 30;
  insn[0] = (insn[0] & ~0x600) | (tie_t << 9);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf90ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 21) >> 30);
  tie_t = (tie_t << 3) | ((insn[0] << 29) >> 29);
  return tie_t;
}

static void
Field_ftsf90ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x7) | (tie_t << 0);
  tie_t = (val << 27) >> 30;
  insn[0] = (insn[0] & ~0x600) | (tie_t << 9);
  tie_t = (val << 25) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf126ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 18) >> 31);
  return tie_t;
}

static void
Field_ftsf126ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x2000) | (tie_t << 13);
}

static unsigned
Field_ftsf344ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 19) >> 30);
  tie_t = (tie_t << 7) | ((insn[0] << 25) >> 25);
  return tie_t;
}

static void
Field_ftsf344ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f) | (tie_t << 0);
  tie_t = (val << 23) >> 30;
  insn[0] = (insn[0] & ~0x1800) | (tie_t << 11);
}

static unsigned
Field_ftsf112ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 7) | ((insn[0] << 25) >> 25);
  return tie_t;
}

static void
Field_ftsf112ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f) | (tie_t << 0);
  tie_t = (val << 22) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf122ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 5) | ((insn[0] << 25) >> 27);
  return tie_t;
}

static void
Field_ftsf122ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 27) >> 27;
  insn[0] = (insn[0] & ~0x7c) | (tie_t << 2);
  tie_t = (val << 24) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf346ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 30) >> 30);
  return tie_t;
}

static void
Field_ftsf346ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x3) | (tie_t << 0);
  tie_t = (val << 28) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
}

static unsigned
Field_ftsf116ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 9) | ((insn[0] << 23) >> 23);
  return tie_t;
}

static void
Field_ftsf116ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 23) >> 23;
  insn[0] = (insn[0] & ~0x1ff) | (tie_t << 0);
  tie_t = (val << 20) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf109ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 7) | ((insn[0] << 25) >> 25);
  return tie_t;
}

static void
Field_ftsf109ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f) | (tie_t << 0);
  tie_t = (val << 22) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf111ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 7) | ((insn[0] << 25) >> 25);
  return tie_t;
}

static void
Field_ftsf111ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f) | (tie_t << 0);
  tie_t = (val << 22) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf104ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 3) | ((insn[0] << 25) >> 29);
  return tie_t;
}

static void
Field_ftsf104ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x70) | (tie_t << 4);
  tie_t = (val << 26) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf105ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 3) | ((insn[0] << 25) >> 29);
  return tie_t;
}

static void
Field_ftsf105ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x70) | (tie_t << 4);
  tie_t = (val << 26) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf107ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 7) | ((insn[0] << 25) >> 25);
  return tie_t;
}

static void
Field_ftsf107ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f) | (tie_t << 0);
  tie_t = (val << 22) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf113ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 7) | ((insn[0] << 25) >> 25);
  return tie_t;
}

static void
Field_ftsf113ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f) | (tie_t << 0);
  tie_t = (val << 22) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf118ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 9) | ((insn[0] << 23) >> 23);
  return tie_t;
}

static void
Field_ftsf118ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 23) >> 23;
  insn[0] = (insn[0] & ~0x1ff) | (tie_t << 0);
  tie_t = (val << 20) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf120ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 6) | ((insn[0] << 25) >> 26);
  return tie_t;
}

static void
Field_ftsf120ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 26) >> 26;
  insn[0] = (insn[0] & ~0x7e) | (tie_t << 1);
  tie_t = (val << 23) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf343ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 1) | ((insn[0] << 31) >> 31);
  return tie_t;
}

static void
Field_ftsf343ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x1) | (tie_t << 0);
  tie_t = (val << 29) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
}

static unsigned
Field_ftsf108ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 7) | ((insn[0] << 25) >> 25);
  return tie_t;
}

static void
Field_ftsf108ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f) | (tie_t << 0);
  tie_t = (val << 22) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf115ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 7) | ((insn[0] << 25) >> 25);
  return tie_t;
}

static void
Field_ftsf115ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f) | (tie_t << 0);
  tie_t = (val << 22) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf110ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 7) | ((insn[0] << 25) >> 25);
  return tie_t;
}

static void
Field_ftsf110ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f) | (tie_t << 0);
  tie_t = (val << 22) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf114ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 7) | ((insn[0] << 25) >> 25);
  return tie_t;
}

static void
Field_ftsf114ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f) | (tie_t << 0);
  tie_t = (val << 22) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf37ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  return tie_t;
}

static void
Field_ftsf37ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 27) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf78ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 9) | ((insn[0] << 16) >> 23);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf78ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 22) >> 23;
  insn[0] = (insn[0] & ~0xff80) | (tie_t << 7);
}

static unsigned
Field_ftsf79ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 9) | ((insn[0] << 16) >> 23);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf79ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 22) >> 23;
  insn[0] = (insn[0] & ~0xff80) | (tie_t << 7);
}

static unsigned
Field_ftsf77ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 9) | ((insn[0] << 16) >> 23);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf77ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 22) >> 23;
  insn[0] = (insn[0] & ~0xff80) | (tie_t << 7);
}

static unsigned
Field_ftsf13_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  return tie_t;
}

static void
Field_ftsf13_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf12_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  return tie_t;
}

static void
Field_ftsf12_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf82ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 7) | ((insn[0] << 16) >> 25);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf82ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 24) >> 25;
  insn[0] = (insn[0] & ~0xfe00) | (tie_t << 9);
}

static unsigned
Field_ftsf341ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 21) >> 30);
  tie_t = (tie_t << 3) | ((insn[0] << 25) >> 29);
  return tie_t;
}

static void
Field_ftsf341ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x70) | (tie_t << 4);
  tie_t = (val << 27) >> 30;
  insn[0] = (insn[0] & ~0x600) | (tie_t << 9);
}

static unsigned
Field_ftsf124ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 1) | ((insn[0] << 25) >> 31);
  return tie_t;
}

static void
Field_ftsf124ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x40) | (tie_t << 6);
  tie_t = (val << 28) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ftsf339ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 21) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_ftsf339ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
  tie_t = (val << 28) >> 30;
  insn[0] = (insn[0] & ~0x600) | (tie_t << 9);
}

static unsigned
Field_ftsf106ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 18) >> 29);
  tie_t = (tie_t << 3) | ((insn[0] << 25) >> 29);
  return tie_t;
}

static void
Field_ftsf106ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x70) | (tie_t << 4);
  tie_t = (val << 26) >> 29;
  insn[0] = (insn[0] & ~0x3800) | (tie_t << 11);
}

static unsigned
Field_ae_r32_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 21) >> 30);
  return tie_t;
}

static void
Field_ae_r32_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x600) | (tie_t << 9);
}

static unsigned
Field_ftsf160ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf160ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf154ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf154ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf175ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf175ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf158ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf158ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf155ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf155ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf167ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf167ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf157ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf157ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf153ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf153ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf163ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf163ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf156ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf156ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf152ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf152ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf161ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf161ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf133ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf133ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf191ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf191ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf142ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf142ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf132ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf132ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf159ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf159ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf141ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf141ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf130ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf130ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf143ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf143ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf140ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf140ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf211ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 16) >> 31);
  return tie_t;
}

static void
Field_ftsf211ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8000) | (tie_t << 15);
}

static unsigned
Field_ftsf332ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 17) >> 31);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf332ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 25) >> 31;
  insn[0] = (insn[0] & ~0x4000) | (tie_t << 14);
}

static unsigned
Field_ftsf135ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf135ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf138ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf138ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf176ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf176ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf170ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf170ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf184ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf184ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf174ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf174ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf171ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf171ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf182ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf182ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf173ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf173ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf169ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf169ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf181ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf181ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf172ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf172ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf168ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf168ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf180ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf180ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf139ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf139ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf151ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf151ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf137ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf137ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf147ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf147ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf136ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf136ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf145ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf145ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf134ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf134ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf144ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf144ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf178ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf178ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf188ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf188ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf183ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf183ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf186ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf186ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf179ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf179ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf187ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf187ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf177ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf177ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf185ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf185ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf45ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf45ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf44ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf44ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf48ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf48ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf47ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf47ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf49ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf49ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf50ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf50ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf52ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf52ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf51ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf51ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf38ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf38ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf54ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf54ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf40ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf40ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf39ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf39ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf46ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf46ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf42ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf42ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf43ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf43ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf41ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf41ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf55ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf55ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf53ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf53ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf58ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf58ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf56ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf56ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf72ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf72ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 26) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf71ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf71ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 26) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf57ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 16) >> 27);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf57ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 23) >> 27;
  insn[0] = (insn[0] & ~0xf800) | (tie_t << 11);
}

static unsigned
Field_ftsf89ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_ftsf89ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_ftsf334ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 20) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf334ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x800) | (tie_t << 11);
}

static unsigned
Field_t_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_t_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
}

static unsigned
Field_ftsf195ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf195ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf207ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 1) | ((insn[0] << 31) >> 31);
  return tie_t;
}

static void
Field_ftsf207ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x1) | (tie_t << 0);
  tie_t = (val << 29) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 27) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf336ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 28) >> 29);
  return tie_t;
}

static void
Field_ftsf336ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe) | (tie_t << 1);
}

static unsigned
Field_ftsf199ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf199ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf210ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 1) | ((insn[0] << 23) >> 31);
  return tie_t;
}

static void
Field_ftsf210ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x100) | (tie_t << 8);
  tie_t = (val << 29) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf337ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf337ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
}

static unsigned
Field_ftsf194ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf194ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf197ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf197ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf196ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf196ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf198ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf198ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf200ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf200ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf203ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf203ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf201ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf201ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf202ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf202ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf204ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf204ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf206ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf206ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf205ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf205ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf209ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 1) | ((insn[0] << 31) >> 31);
  return tie_t;
}

static void
Field_ftsf209ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x1) | (tie_t << 0);
  tie_t = (val << 29) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 27) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf127ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf127ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf129ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf129ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf128ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf128ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf131ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf131ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf146ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf146ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf149ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf149ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf148ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf148ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf150ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf150ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf162ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf162ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf165ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf165ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf164ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf164ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf166ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf166ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf189ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf189ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf192ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf192ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf190ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf190ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_ftsf193ae_slot1_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf193ae_slot1_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
  tie_t = (val << 24) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_r_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 24) >> 28);
  return tie_t;
}

static void
Field_r_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf0) | (tie_t << 4);
}

static unsigned
Field_op0_s4_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 7) | ((insn[0] << 5) >> 25);
  return tie_t;
}

static void
Field_op0_s4_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0x7f00000) | (tie_t << 20);
}

static unsigned
Field_imm8_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  return tie_t;
}

static void
Field_imm8_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 24) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_t_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_t_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
}

static unsigned
Field_ftsf293_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 24) >> 29);
  return tie_t;
}

static void
Field_ftsf293_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe0) | (tie_t << 5);
}

static unsigned
Field_ftsf321_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf321_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
}

static unsigned
Field_ae_s20_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 29) >> 29);
  return tie_t;
}

static void
Field_ae_s20_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x7) | (tie_t << 0);
}

static unsigned
Field_ftsf214ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 12) >> 28);
  return tie_t;
}

static void
Field_ftsf214ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf0000) | (tie_t << 16);
}

static unsigned
Field_ftsf213ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 12) >> 29);
  return tie_t;
}

static void
Field_ftsf213ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe0000) | (tie_t << 17);
}

static unsigned
Field_ftsf212ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 12) >> 30);
  return tie_t;
}

static void
Field_ftsf212ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc0000) | (tie_t << 18);
}

static unsigned
Field_ftsf281ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 8) | ((insn[0] << 24) >> 24);
  return tie_t;
}

static void
Field_ftsf281ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 24) >> 24;
  insn[0] = (insn[0] & ~0xff) | (tie_t << 0);
  tie_t = (val << 16) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf217_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf217_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
}

static unsigned
Field_ae_r20_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 25) >> 29);
  return tie_t;
}

static void
Field_ae_r20_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x70) | (tie_t << 4);
}

static unsigned
Field_ftsf300ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 12) | ((insn[0] << 12) >> 20);
  return tie_t;
}

static void
Field_ftsf300ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 20) >> 20;
  insn[0] = (insn[0] & ~0xfff00) | (tie_t << 8);
}

static unsigned
Field_ftsf283ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  tie_t = (tie_t << 6) | ((insn[0] << 26) >> 26);
  return tie_t;
}

static void
Field_ftsf283ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 26) >> 26;
  insn[0] = (insn[0] & ~0x3f) | (tie_t << 0);
  tie_t = (val << 25) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 17) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf352ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 25) >> 31);
  return tie_t;
}

static void
Field_ftsf352ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x40) | (tie_t << 6);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
}

static unsigned
Field_ftsf282ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 8) | ((insn[0] << 24) >> 24);
  return tie_t;
}

static void
Field_ftsf282ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 24) >> 24;
  insn[0] = (insn[0] & ~0xff) | (tie_t << 0);
  tie_t = (val << 16) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf288ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 3) | ((insn[0] << 26) >> 29);
  return tie_t;
}

static void
Field_ftsf288ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x38) | (tie_t << 3);
  tie_t = (val << 21) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf359ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  tie_t = (tie_t << 3) | ((insn[0] << 29) >> 29);
  return tie_t;
}

static void
Field_ftsf359ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x7) | (tie_t << 0);
  tie_t = (val << 27) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
}

static unsigned
Field_ftsf286ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 4) | ((insn[0] << 26) >> 28);
  return tie_t;
}

static void
Field_ftsf286ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0x3c) | (tie_t << 2);
  tie_t = (val << 20) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf356ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 30) >> 30);
  return tie_t;
}

static void
Field_ftsf356ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x3) | (tie_t << 0);
  tie_t = (val << 28) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
}

static unsigned
Field_ftsf284ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 5) | ((insn[0] << 26) >> 27);
  return tie_t;
}

static void
Field_ftsf284ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 27) >> 27;
  insn[0] = (insn[0] & ~0x3e) | (tie_t << 1);
  tie_t = (val << 19) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf354ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  tie_t = (tie_t << 1) | ((insn[0] << 31) >> 31);
  return tie_t;
}

static void
Field_ftsf354ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x1) | (tie_t << 0);
  tie_t = (val << 29) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
}

static unsigned
Field_ftsf295ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  tie_t = (tie_t << 1) | ((insn[0] << 26) >> 31);
  return tie_t;
}

static void
Field_ftsf295ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x20) | (tie_t << 5);
  tie_t = (val << 30) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf358ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 25) >> 31);
  return tie_t;
}

static void
Field_ftsf358ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x40) | (tie_t << 6);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
}

static unsigned
Field_ftsf325ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf325ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 20) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf215ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 7) | ((insn[0] << 12) >> 25);
  return tie_t;
}

static void
Field_ftsf215ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 25) >> 25;
  insn[0] = (insn[0] & ~0xfe000) | (tie_t << 13);
}

static unsigned
Field_ftsf301ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 13) | ((insn[0] << 12) >> 19);
  return tie_t;
}

static void
Field_ftsf301ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 19) >> 19;
  insn[0] = (insn[0] & ~0xfff80) | (tie_t << 7);
}

static unsigned
Field_ftsf353_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 25) >> 31);
  return tie_t;
}

static void
Field_ftsf353_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x40) | (tie_t << 6);
}

static unsigned
Field_ftsf309ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 9) | ((insn[0] << 12) >> 23);
  return tie_t;
}

static void
Field_ftsf309ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 23) >> 23;
  insn[0] = (insn[0] & ~0xff800) | (tie_t << 11);
}

static unsigned
Field_ftsf360ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 21) >> 27);
  return tie_t;
}

static void
Field_ftsf360ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 27) >> 27;
  insn[0] = (insn[0] & ~0x7c0) | (tie_t << 6);
}

static unsigned
Field_ftsf294ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 3) | ((insn[0] << 24) >> 29);
  return tie_t;
}

static void
Field_ftsf294ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe0) | (tie_t << 5);
  tie_t = (val << 21) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_s_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  return tie_t;
}

static void
Field_s_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
}

static unsigned
Field_ftsf292ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 3) | ((insn[0] << 24) >> 29);
  return tie_t;
}

static void
Field_ftsf292ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe0) | (tie_t << 5);
  tie_t = (val << 21) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf319_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 28) >> 29);
  return tie_t;
}

static void
Field_ftsf319_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe) | (tie_t << 1);
}

static unsigned
Field_ftsf361ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 24) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 31) >> 31);
  return tie_t;
}

static void
Field_ftsf361ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x1) | (tie_t << 0);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0xf0) | (tie_t << 4);
}

static unsigned
Field_ftsf218ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf218ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf220ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf220ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf221ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf221ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf222ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf222ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf228ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf228ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf229ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf229ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf230ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf230ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf232ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf232ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf233ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf233ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf235ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf235ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf239ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf239ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf234ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf234ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf224ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf224ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf225ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf225ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf227ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf227ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf226ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf226ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf241ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf241ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf243ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf243ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf242ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf242ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf244ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf244ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf236ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf236ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf237ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf237ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf238ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf238ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf240ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf240ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf261ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf261ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf296ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf296ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf248ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf248ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf250ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf250ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf269ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf269ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf264ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf264ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf266ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf266ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf267ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf267ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf260ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf260ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf262ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf262ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf263ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf263ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf265ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf265ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf246ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf246ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf247ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf247ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf249ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf249ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf253ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf253ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf257ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf257ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf256ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf256ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf258ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf258ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf259ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf259ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf251ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf251ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf252ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf252ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf254ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf254ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf255ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf255ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf275ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_ftsf275ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf277ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_ftsf277ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf278ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_ftsf278ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf290ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 26) >> 31);
  return tie_t;
}

static void
Field_ftsf290ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x20) | (tie_t << 5);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_s8_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 27) >> 31);
  return tie_t;
}

static void
Field_s8_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x10) | (tie_t << 4);
}

static unsigned
Field_ftsf272ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_ftsf272ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf276ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_ftsf276ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf273ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_ftsf273ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf274ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_ftsf274ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf297ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  return tie_t;
}

static void
Field_ftsf297ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf298ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  return tie_t;
}

static void
Field_ftsf298ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf310ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  return tie_t;
}

static void
Field_ftsf310ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf311ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  return tie_t;
}

static void
Field_ftsf311ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf270ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  return tie_t;
}

static void
Field_ftsf270ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf271ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  return tie_t;
}

static void
Field_ftsf271ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ae_r32_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  return tie_t;
}

static void
Field_ae_r32_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
}

static unsigned
Field_ftsf329ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 12) >> 27);
  return tie_t;
}

static void
Field_ftsf329ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 27) >> 27;
  insn[0] = (insn[0] & ~0xf8000) | (tie_t << 15);
}

static unsigned
Field_ftsf362ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 17) >> 29);
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  return tie_t;
}

static void
Field_ftsf362ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
  tie_t = (val << 27) >> 29;
  insn[0] = (insn[0] & ~0x7000) | (tie_t << 12);
}

static unsigned
Field_ftsf245ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf245ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf268ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf268ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf313ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf313ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 19) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf312ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf312ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 19) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf231ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf231ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf223ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf223ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf219ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf219ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf216ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_ftsf216ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf302ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 12) | ((insn[0] << 12) >> 20);
  tie_t = (tie_t << 3) | ((insn[0] << 29) >> 29);
  return tie_t;
}

static void
Field_ftsf302ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x7) | (tie_t << 0);
  tie_t = (val << 17) >> 20;
  insn[0] = (insn[0] & ~0xfff00) | (tie_t << 8);
}

static unsigned
Field_ftsf364ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 19) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf364ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x1000) | (tie_t << 12);
}

static unsigned
Field_ftsf322ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf322ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 20) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf279ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 6) | ((insn[0] << 26) >> 26);
  return tie_t;
}

static void
Field_ftsf279ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 26) >> 26;
  insn[0] = (insn[0] & ~0x3f) | (tie_t << 0);
  tie_t = (val << 18) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf318ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  tie_t = (tie_t << 3) | ((insn[0] << 28) >> 29);
  return tie_t;
}

static void
Field_ftsf318ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe) | (tie_t << 1);
  tie_t = (val << 28) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 20) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf365ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 25) >> 31);
  tie_t = (tie_t << 1) | ((insn[0] << 31) >> 31);
  return tie_t;
}

static void
Field_ftsf365ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x1) | (tie_t << 0);
  tie_t = (val << 30) >> 31;
  insn[0] = (insn[0] & ~0x40) | (tie_t << 6);
}

static unsigned
Field_ftsf316ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf316ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 19) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf314ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf314ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 19) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf315ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf315ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 19) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf320ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf320ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 30) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf299ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 10) | ((insn[0] << 12) >> 22);
  return tie_t;
}

static void
Field_ftsf299ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 22) >> 22;
  insn[0] = (insn[0] & ~0xffc00) | (tie_t << 10);
}

static unsigned
Field_ftsf308ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 11) | ((insn[0] << 12) >> 21);
  return tie_t;
}

static void
Field_ftsf308ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 21) >> 21;
  insn[0] = (insn[0] & ~0xffe00) | (tie_t << 9);
}

static unsigned
Field_ftsf366ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 23) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf366ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x100) | (tie_t << 8);
}

static unsigned
Field_ftsf306ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 12) | ((insn[0] << 12) >> 20);
  tie_t = (tie_t << 1) | ((insn[0] << 29) >> 31);
  return tie_t;
}

static void
Field_ftsf306ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x4) | (tie_t << 2);
  tie_t = (val << 19) >> 20;
  insn[0] = (insn[0] & ~0xfff00) | (tie_t << 8);
}

static unsigned
Field_ftsf368ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  tie_t = (tie_t << 2) | ((insn[0] << 30) >> 30);
  return tie_t;
}

static void
Field_ftsf368ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x3) | (tie_t << 0);
  tie_t = (val << 29) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
}

static unsigned
Field_ftsf304ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 12) | ((insn[0] << 12) >> 20);
  tie_t = (tie_t << 2) | ((insn[0] << 29) >> 30);
  return tie_t;
}

static void
Field_ftsf304ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x6) | (tie_t << 1);
  tie_t = (val << 18) >> 20;
  insn[0] = (insn[0] & ~0xfff00) | (tie_t << 8);
}

static unsigned
Field_ftsf369ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  tie_t = (tie_t << 1) | ((insn[0] << 31) >> 31);
  return tie_t;
}

static void
Field_ftsf369ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x1) | (tie_t << 0);
  tie_t = (val << 30) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
}

static unsigned
Field_ftsf323ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf323ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 20) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf328ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf328ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 23) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf326ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 2) | ((insn[0] << 28) >> 30);
  return tie_t;
}

static void
Field_ftsf326ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc) | (tie_t << 2);
  tie_t = (val << 22) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf357_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 30) >> 30);
  return tie_t;
}

static void
Field_ftsf357_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x3) | (tie_t << 0);
}

static unsigned
Field_ftsf303ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 12) | ((insn[0] << 12) >> 20);
  tie_t = (tie_t << 3) | ((insn[0] << 29) >> 29);
  return tie_t;
}

static void
Field_ftsf303ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x7) | (tie_t << 0);
  tie_t = (val << 17) >> 20;
  insn[0] = (insn[0] & ~0xfff00) | (tie_t << 8);
}

static unsigned
Field_ftsf324ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf324ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 20) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_ftsf317ae_slot0_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 12) >> 24);
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ftsf317ae_slot0_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
  tie_t = (val << 19) >> 24;
  insn[0] = (insn[0] & ~0xff000) | (tie_t << 12);
}

static unsigned
Field_t_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 24) >> 28);
  return tie_t;
}

static void
Field_t_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf0) | (tie_t << 4);
}

static unsigned
Field_bbi4_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 19) >> 31);
  return tie_t;
}

static void
Field_bbi4_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x1000) | (tie_t << 12);
}

static unsigned
Field_bbi_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 19) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 24) >> 28);
  return tie_t;
}

static void
Field_bbi_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf0) | (tie_t << 4);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x1000) | (tie_t << 12);
}

static unsigned
Field_bbi_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 27) >> 27);
  return tie_t;
}

static void
Field_bbi_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 27) >> 27;
  insn[0] = (insn[0] & ~0x1f) | (tie_t << 0);
}

static unsigned
Field_imm12_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 12) | ((insn[0] << 8) >> 20);
  return tie_t;
}

static void
Field_imm12_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 20) >> 20;
  insn[0] = (insn[0] & ~0xfff000) | (tie_t << 12);
}

static unsigned
Field_imm12_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  tie_t = (tie_t << 8) | ((insn[0] << 24) >> 24);
  return tie_t;
}

static void
Field_imm12_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 24) >> 24;
  insn[0] = (insn[0] & ~0xff) | (tie_t << 0);
  tie_t = (val << 20) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_imm8_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 8) | ((insn[0] << 8) >> 24);
  return tie_t;
}

static void
Field_imm8_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 24) >> 24;
  insn[0] = (insn[0] & ~0xff0000) | (tie_t << 16);
}

static unsigned
Field_s_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  return tie_t;
}

static void
Field_s_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
}

static unsigned
Field_imm12b_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  tie_t = (tie_t << 8) | ((insn[0] << 8) >> 24);
  return tie_t;
}

static void
Field_imm12b_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 24) >> 24;
  insn[0] = (insn[0] & ~0xff0000) | (tie_t << 16);
  tie_t = (val << 20) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
}

static unsigned
Field_imm12b_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 12) | ((insn[0] << 16) >> 20);
  return tie_t;
}

static void
Field_imm12b_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 20) >> 20;
  insn[0] = (insn[0] & ~0xfff0) | (tie_t << 4);
}

static unsigned
Field_imm16_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 16) | ((insn[0] << 8) >> 16);
  return tie_t;
}

static void
Field_imm16_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 16) >> 16;
  insn[0] = (insn[0] & ~0xffff00) | (tie_t << 8);
}

static unsigned
Field_imm16_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 16) | ((insn[0] << 12) >> 16);
  return tie_t;
}

static void
Field_imm16_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 16) >> 16;
  insn[0] = (insn[0] & ~0xffff0) | (tie_t << 4);
}

static unsigned
Field_offset_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 18) | ((insn[0] << 8) >> 14);
  return tie_t;
}

static void
Field_offset_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 14) >> 14;
  insn[0] = (insn[0] & ~0xffffc0) | (tie_t << 6);
}

static unsigned
Field_offset_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 18) | ((insn[0] << 14) >> 14);
  return tie_t;
}

static void
Field_offset_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 14) >> 14;
  insn[0] = (insn[0] & ~0x3ffff) | (tie_t << 0);
}

static unsigned
Field_op2_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  return tie_t;
}

static void
Field_op2_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
}

static unsigned
Field_r_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_r_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_sa4_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 11) >> 31);
  return tie_t;
}

static void
Field_sa4_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x100000) | (tie_t << 20);
}

static unsigned
Field_sae4_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 15) >> 31);
  return tie_t;
}

static void
Field_sae4_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x10000) | (tie_t << 16);
}

static unsigned
Field_sae_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 15) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  return tie_t;
}

static void
Field_sae_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x10000) | (tie_t << 16);
}

static unsigned
Field_sae_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 15) >> 27);
  return tie_t;
}

static void
Field_sae_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 27) >> 27;
  insn[0] = (insn[0] & ~0x1f000) | (tie_t << 12);
}

static unsigned
Field_sal_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 11) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 24) >> 28);
  return tie_t;
}

static void
Field_sal_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf0) | (tie_t << 4);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x100000) | (tie_t << 20);
}

static unsigned
Field_sal_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 19) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_sal_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x1000) | (tie_t << 12);
}

static unsigned
Field_sargt_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 11) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  return tie_t;
}

static void
Field_sargt_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x100000) | (tie_t << 20);
}

static unsigned
Field_sargt_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 19) >> 27);
  return tie_t;
}

static void
Field_sargt_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 27) >> 27;
  insn[0] = (insn[0] & ~0x1f00) | (tie_t << 8);
}

static unsigned
Field_sas4_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 27) >> 31);
  return tie_t;
}

static void
Field_sas4_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x10) | (tie_t << 4);
}

static unsigned
Field_sas_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 27) >> 31);
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  return tie_t;
}

static void
Field_sas_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
  tie_t = (val << 27) >> 31;
  insn[0] = (insn[0] & ~0x10) | (tie_t << 4);
}

static unsigned
Field_sas_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 5) | ((insn[0] << 27) >> 27);
  return tie_t;
}

static void
Field_sas_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 27) >> 27;
  insn[0] = (insn[0] & ~0x1f) | (tie_t << 0);
}

static unsigned
Field_sr_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  return tie_t;
}

static void
Field_sr_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
  tie_t = (val << 24) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_sr_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  return tie_t;
}

static void
Field_sr_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
  tie_t = (val << 24) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_st_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  tie_t = (tie_t << 4) | ((insn[0] << 24) >> 28);
  return tie_t;
}

static void
Field_st_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf0) | (tie_t << 4);
  tie_t = (val << 24) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
}

static unsigned
Field_st_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 20) >> 28);
  tie_t = (tie_t << 4) | ((insn[0] << 24) >> 28);
  return tie_t;
}

static void
Field_st_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf0) | (tie_t << 4);
  tie_t = (val << 24) >> 28;
  insn[0] = (insn[0] & ~0xf00) | (tie_t << 8);
}

static unsigned
Field_imm4_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_imm4_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_imm4_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_imm4_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_imm4_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_imm4_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_mn_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_mn_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
  tie_t = (val << 28) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
}

static unsigned
Field_i_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_i_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
}

static unsigned
Field_imm6lo_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_imm6lo_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_imm6lo_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_imm6lo_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_imm6hi_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_imm6hi_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
}

static unsigned
Field_imm6hi_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_imm6hi_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
}

static unsigned
Field_imm7lo_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_imm7lo_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_imm7lo_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_imm7lo_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
}

static unsigned
Field_imm7hi_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 25) >> 29);
  return tie_t;
}

static void
Field_imm7hi_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x70) | (tie_t << 4);
}

static unsigned
Field_imm7hi_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 25) >> 29);
  return tie_t;
}

static void
Field_imm7hi_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x70) | (tie_t << 4);
}

static unsigned
Field_z_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 25) >> 31);
  return tie_t;
}

static void
Field_z_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x40) | (tie_t << 6);
}

static unsigned
Field_imm6_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_imm6_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
}

static unsigned
Field_imm6_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_imm6_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
}

static unsigned
Field_imm7_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 25) >> 29);
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_imm7_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
  tie_t = (val << 25) >> 29;
  insn[0] = (insn[0] & ~0x70) | (tie_t << 4);
}

static unsigned
Field_imm7_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 25) >> 29);
  tie_t = (tie_t << 4) | ((insn[0] << 16) >> 28);
  return tie_t;
}

static void
Field_imm7_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf000) | (tie_t << 12);
  tie_t = (val << 25) >> 29;
  insn[0] = (insn[0] & ~0x70) | (tie_t << 4);
}

static unsigned
Field_t2_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 24) >> 29);
  return tie_t;
}

static void
Field_t2_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe0) | (tie_t << 5);
}

static unsigned
Field_t2_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 24) >> 29);
  return tie_t;
}

static void
Field_t2_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe0) | (tie_t << 5);
}

static unsigned
Field_t2_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 24) >> 29);
  return tie_t;
}

static void
Field_t2_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe0) | (tie_t << 5);
}

static unsigned
Field_t2_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 23) >> 30);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_t2_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 29) >> 30;
  insn[0] = (insn[0] & ~0x180) | (tie_t << 7);
}

static unsigned
Field_s2_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 20) >> 29);
  return tie_t;
}

static void
Field_s2_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe00) | (tie_t << 9);
}

static unsigned
Field_s2_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 20) >> 29);
  return tie_t;
}

static void
Field_s2_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe00) | (tie_t << 9);
}

static unsigned
Field_s2_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 20) >> 29);
  return tie_t;
}

static void
Field_s2_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe00) | (tie_t << 9);
}

static unsigned
Field_r2_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 16) >> 29);
  return tie_t;
}

static void
Field_r2_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe000) | (tie_t << 13);
}

static unsigned
Field_r2_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 16) >> 29);
  return tie_t;
}

static void
Field_r2_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe000) | (tie_t << 13);
}

static unsigned
Field_r2_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 16) >> 29);
  return tie_t;
}

static void
Field_r2_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0xe000) | (tie_t << 13);
}

static unsigned
Field_t4_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  return tie_t;
}

static void
Field_t4_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
}

static unsigned
Field_t4_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  return tie_t;
}

static void
Field_t4_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
}

static unsigned
Field_t4_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 24) >> 30);
  return tie_t;
}

static void
Field_t4_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc0) | (tie_t << 6);
}

static unsigned
Field_s4_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 20) >> 30);
  return tie_t;
}

static void
Field_s4_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc00) | (tie_t << 10);
}

static unsigned
Field_s4_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 20) >> 30);
  return tie_t;
}

static void
Field_s4_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc00) | (tie_t << 10);
}

static unsigned
Field_s4_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 20) >> 30);
  return tie_t;
}

static void
Field_s4_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc00) | (tie_t << 10);
}

static unsigned
Field_s4_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_s4_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
}

static unsigned
Field_r4_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  return tie_t;
}

static void
Field_r4_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_r4_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  return tie_t;
}

static void
Field_r4_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_r4_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 16) >> 30);
  return tie_t;
}

static void
Field_r4_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0xc000) | (tie_t << 14);
}

static unsigned
Field_t8_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_t8_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
}

static unsigned
Field_t8_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_t8_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
}

static unsigned
Field_t8_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 24) >> 31);
  return tie_t;
}

static void
Field_t8_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x80) | (tie_t << 7);
}

static unsigned
Field_s8_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 20) >> 31);
  return tie_t;
}

static void
Field_s8_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x800) | (tie_t << 11);
}

static unsigned
Field_s8_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 20) >> 31);
  return tie_t;
}

static void
Field_s8_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x800) | (tie_t << 11);
}

static unsigned
Field_s8_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 20) >> 31);
  return tie_t;
}

static void
Field_s8_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x800) | (tie_t << 11);
}

static unsigned
Field_r8_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 16) >> 31);
  return tie_t;
}

static void
Field_r8_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8000) | (tie_t << 15);
}

static unsigned
Field_r8_Slot_inst16a_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 16) >> 31);
  return tie_t;
}

static void
Field_r8_Slot_inst16a_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8000) | (tie_t << 15);
}

static unsigned
Field_r8_Slot_inst16b_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 1) | ((insn[0] << 16) >> 31);
  return tie_t;
}

static void
Field_r8_Slot_inst16b_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8000) | (tie_t << 15);
}

static unsigned
Field_xt_wbr15_imm_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 15) | ((insn[0] << 8) >> 17);
  return tie_t;
}

static void
Field_xt_wbr15_imm_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 17) >> 17;
  insn[0] = (insn[0] & ~0xfffe00) | (tie_t << 9);
}

static unsigned
Field_xt_wbr18_imm_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 18) | ((insn[0] << 8) >> 14);
  return tie_t;
}

static void
Field_xt_wbr18_imm_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 14) >> 14;
  insn[0] = (insn[0] & ~0xffffc0) | (tie_t << 6);
}

static unsigned
Field_ae_samt_s_t_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 6) | ((insn[0] << 22) >> 26);
  return tie_t;
}

static void
Field_ae_samt_s_t_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 26) >> 26;
  insn[0] = (insn[0] & ~0x3f0) | (tie_t << 4);
}

static unsigned
Field_ae_samt_s_t_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 22) >> 30);
  tie_t = (tie_t << 4) | ((insn[0] << 28) >> 28);
  return tie_t;
}

static void
Field_ae_samt_s_t_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 28) >> 28;
  insn[0] = (insn[0] & ~0xf) | (tie_t << 0);
  tie_t = (val << 26) >> 30;
  insn[0] = (insn[0] & ~0x300) | (tie_t << 8);
}

static unsigned
Field_ae_r20_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 17) >> 29);
  return tie_t;
}

static void
Field_ae_r20_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x7000) | (tie_t << 12);
}

static unsigned
Field_ae_r10_Slot_ae_slot0_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_ae_r10_Slot_ae_slot0_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
}

static unsigned
Field_ae_s20_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 21) >> 29);
  return tie_t;
}

static void
Field_ae_s20_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x700) | (tie_t << 8);
}

static unsigned
Field_ftsf12_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 3) | ((insn[0] << 25) >> 29);
  return tie_t;
}

static void
Field_ftsf12_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 29) >> 29;
  insn[0] = (insn[0] & ~0x70) | (tie_t << 4);
}

static unsigned
Field_ftsf13_Slot_inst_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 2) | ((insn[0] << 26) >> 30);
  return tie_t;
}

static void
Field_ftsf13_Slot_inst_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 30) >> 30;
  insn[0] = (insn[0] & ~0x30) | (tie_t << 4);
}

static unsigned
Field_ftsf14_Slot_ae_slot1_get (const xtensa_insnbuf insn)
{
  unsigned tie_t = 0;
  tie_t = (tie_t << 4) | ((insn[0] << 21) >> 28);
  tie_t = (tie_t << 1) | ((insn[0] << 28) >> 31);
  return tie_t;
}

static void
Field_ftsf14_Slot_ae_slot1_set (xtensa_insnbuf insn, uint32 val)
{
  uint32 tie_t;
  tie_t = (val << 31) >> 31;
  insn[0] = (insn[0] & ~0x8) | (tie_t << 3);
  tie_t = (val << 27) >> 28;
  insn[0] = (insn[0] & ~0x780) | (tie_t << 7);
}

static void
Implicit_Field_set (xtensa_insnbuf insn ATTRIBUTE_UNUSED,
		    uint32 val ATTRIBUTE_UNUSED)
{
  /* Do nothing.  */
}

static unsigned
Implicit_Field_ar0_get (const xtensa_insnbuf insn ATTRIBUTE_UNUSED)
{
  return 0;
}

static unsigned
Implicit_Field_ar4_get (const xtensa_insnbuf insn ATTRIBUTE_UNUSED)
{
  return 4;
}

static unsigned
Implicit_Field_ar8_get (const xtensa_insnbuf insn ATTRIBUTE_UNUSED)
{
  return 8;
}

static unsigned
Implicit_Field_ar12_get (const xtensa_insnbuf insn ATTRIBUTE_UNUSED)
{
  return 12;
}

static unsigned
Implicit_Field_bt16_get (const xtensa_insnbuf insn ATTRIBUTE_UNUSED)
{
  return 0;
}

static unsigned
Implicit_Field_bs16_get (const xtensa_insnbuf insn ATTRIBUTE_UNUSED)
{
  return 0;
}

static unsigned
Implicit_Field_br16_get (const xtensa_insnbuf insn ATTRIBUTE_UNUSED)
{
  return 0;
}

static unsigned
Implicit_Field_brall_get (const xtensa_insnbuf insn ATTRIBUTE_UNUSED)
{
  return 0;
}

enum xtensa_field_id {
  FIELD_t,
  FIELD_bbi4,
  FIELD_bbi,
  FIELD_imm12,
  FIELD_imm8,
  FIELD_s,
  FIELD_imm12b,
  FIELD_imm16,
  FIELD_m,
  FIELD_n,
  FIELD_offset,
  FIELD_op0,
  FIELD_op1,
  FIELD_op2,
  FIELD_r,
  FIELD_sa4,
  FIELD_sae4,
  FIELD_sae,
  FIELD_sal,
  FIELD_sargt,
  FIELD_sas4,
  FIELD_sas,
  FIELD_sr,
  FIELD_st,
  FIELD_thi3,
  FIELD_imm4,
  FIELD_mn,
  FIELD_i,
  FIELD_imm6lo,
  FIELD_imm6hi,
  FIELD_imm7lo,
  FIELD_imm7hi,
  FIELD_z,
  FIELD_imm6,
  FIELD_imm7,
  FIELD_t2,
  FIELD_s2,
  FIELD_r2,
  FIELD_t4,
  FIELD_s4,
  FIELD_r4,
  FIELD_t8,
  FIELD_s8,
  FIELD_r8,
  FIELD_xt_wbr15_imm,
  FIELD_xt_wbr18_imm,
  FIELD_ae_r3,
  FIELD_ae_s_non_samt,
  FIELD_ae_s3,
  FIELD_ae_r32,
  FIELD_ae_samt_s_t,
  FIELD_ae_r20,
  FIELD_ae_r10,
  FIELD_ae_s20,
  FIELD_op0_s3,
  FIELD_ftsf12,
  FIELD_ftsf13,
  FIELD_ftsf14,
  FIELD_ftsf21ae_slot1,
  FIELD_ftsf22ae_slot1,
  FIELD_ftsf23ae_slot1,
  FIELD_ftsf24ae_slot1,
  FIELD_ftsf25ae_slot1,
  FIELD_ftsf26ae_slot1,
  FIELD_ftsf27ae_slot1,
  FIELD_ftsf28ae_slot1,
  FIELD_ftsf29ae_slot1,
  FIELD_ftsf30ae_slot1,
  FIELD_ftsf31ae_slot1,
  FIELD_ftsf32ae_slot1,
  FIELD_ftsf33ae_slot1,
  FIELD_ftsf34ae_slot1,
  FIELD_ftsf35ae_slot1,
  FIELD_ftsf36ae_slot1,
  FIELD_ftsf37ae_slot1,
  FIELD_ftsf38ae_slot1,
  FIELD_ftsf39ae_slot1,
  FIELD_ftsf40ae_slot1,
  FIELD_ftsf41ae_slot1,
  FIELD_ftsf42ae_slot1,
  FIELD_ftsf43ae_slot1,
  FIELD_ftsf44ae_slot1,
  FIELD_ftsf45ae_slot1,
  FIELD_ftsf46ae_slot1,
  FIELD_ftsf47ae_slot1,
  FIELD_ftsf48ae_slot1,
  FIELD_ftsf49ae_slot1,
  FIELD_ftsf50ae_slot1,
  FIELD_ftsf51ae_slot1,
  FIELD_ftsf52ae_slot1,
  FIELD_ftsf53ae_slot1,
  FIELD_ftsf54ae_slot1,
  FIELD_ftsf55ae_slot1,
  FIELD_ftsf56ae_slot1,
  FIELD_ftsf57ae_slot1,
  FIELD_ftsf58ae_slot1,
  FIELD_ftsf59ae_slot1,
  FIELD_ftsf60ae_slot1,
  FIELD_ftsf61ae_slot1,
  FIELD_ftsf63ae_slot1,
  FIELD_ftsf64ae_slot1,
  FIELD_ftsf66ae_slot1,
  FIELD_ftsf67ae_slot1,
  FIELD_ftsf69ae_slot1,
  FIELD_ftsf71ae_slot1,
  FIELD_ftsf72ae_slot1,
  FIELD_ftsf73ae_slot1,
  FIELD_ftsf75ae_slot1,
  FIELD_ftsf76ae_slot1,
  FIELD_ftsf77ae_slot1,
  FIELD_ftsf78ae_slot1,
  FIELD_ftsf79ae_slot1,
  FIELD_ftsf80ae_slot1,
  FIELD_ftsf81ae_slot1,
  FIELD_ftsf82ae_slot1,
  FIELD_ftsf84ae_slot1,
  FIELD_ftsf86ae_slot1,
  FIELD_ftsf87ae_slot1,
  FIELD_ftsf88ae_slot1,
  FIELD_ftsf89ae_slot1,
  FIELD_ftsf90ae_slot1,
  FIELD_ftsf91ae_slot1,
  FIELD_ftsf92ae_slot1,
  FIELD_ftsf94ae_slot1,
  FIELD_ftsf96ae_slot1,
  FIELD_ftsf97ae_slot1,
  FIELD_ftsf98ae_slot1,
  FIELD_ftsf99ae_slot1,
  FIELD_ftsf100ae_slot1,
  FIELD_ftsf101ae_slot1,
  FIELD_ftsf103ae_slot1,
  FIELD_ftsf104ae_slot1,
  FIELD_ftsf105ae_slot1,
  FIELD_ftsf106ae_slot1,
  FIELD_ftsf107ae_slot1,
  FIELD_ftsf108ae_slot1,
  FIELD_ftsf109ae_slot1,
  FIELD_ftsf110ae_slot1,
  FIELD_ftsf111ae_slot1,
  FIELD_ftsf112ae_slot1,
  FIELD_ftsf113ae_slot1,
  FIELD_ftsf114ae_slot1,
  FIELD_ftsf115ae_slot1,
  FIELD_ftsf116ae_slot1,
  FIELD_ftsf118ae_slot1,
  FIELD_ftsf119ae_slot1,
  FIELD_ftsf120ae_slot1,
  FIELD_ftsf122ae_slot1,
  FIELD_ftsf124ae_slot1,
  FIELD_ftsf125ae_slot1,
  FIELD_ftsf126ae_slot1,
  FIELD_ftsf127ae_slot1,
  FIELD_ftsf128ae_slot1,
  FIELD_ftsf129ae_slot1,
  FIELD_ftsf130ae_slot1,
  FIELD_ftsf131ae_slot1,
  FIELD_ftsf132ae_slot1,
  FIELD_ftsf133ae_slot1,
  FIELD_ftsf134ae_slot1,
  FIELD_ftsf135ae_slot1,
  FIELD_ftsf136ae_slot1,
  FIELD_ftsf137ae_slot1,
  FIELD_ftsf138ae_slot1,
  FIELD_ftsf139ae_slot1,
  FIELD_ftsf140ae_slot1,
  FIELD_ftsf141ae_slot1,
  FIELD_ftsf142ae_slot1,
  FIELD_ftsf143ae_slot1,
  FIELD_ftsf144ae_slot1,
  FIELD_ftsf145ae_slot1,
  FIELD_ftsf146ae_slot1,
  FIELD_ftsf147ae_slot1,
  FIELD_ftsf148ae_slot1,
  FIELD_ftsf149ae_slot1,
  FIELD_ftsf150ae_slot1,
  FIELD_ftsf151ae_slot1,
  FIELD_ftsf152ae_slot1,
  FIELD_ftsf153ae_slot1,
  FIELD_ftsf154ae_slot1,
  FIELD_ftsf155ae_slot1,
  FIELD_ftsf156ae_slot1,
  FIELD_ftsf157ae_slot1,
  FIELD_ftsf158ae_slot1,
  FIELD_ftsf159ae_slot1,
  FIELD_ftsf160ae_slot1,
  FIELD_ftsf161ae_slot1,
  FIELD_ftsf162ae_slot1,
  FIELD_ftsf163ae_slot1,
  FIELD_ftsf164ae_slot1,
  FIELD_ftsf165ae_slot1,
  FIELD_ftsf166ae_slot1,
  FIELD_ftsf167ae_slot1,
  FIELD_ftsf168ae_slot1,
  FIELD_ftsf169ae_slot1,
  FIELD_ftsf170ae_slot1,
  FIELD_ftsf171ae_slot1,
  FIELD_ftsf172ae_slot1,
  FIELD_ftsf173ae_slot1,
  FIELD_ftsf174ae_slot1,
  FIELD_ftsf175ae_slot1,
  FIELD_ftsf176ae_slot1,
  FIELD_ftsf177ae_slot1,
  FIELD_ftsf178ae_slot1,
  FIELD_ftsf179ae_slot1,
  FIELD_ftsf180ae_slot1,
  FIELD_ftsf181ae_slot1,
  FIELD_ftsf182ae_slot1,
  FIELD_ftsf183ae_slot1,
  FIELD_ftsf184ae_slot1,
  FIELD_ftsf185ae_slot1,
  FIELD_ftsf186ae_slot1,
  FIELD_ftsf187ae_slot1,
  FIELD_ftsf188ae_slot1,
  FIELD_ftsf189ae_slot1,
  FIELD_ftsf190ae_slot1,
  FIELD_ftsf191ae_slot1,
  FIELD_ftsf192ae_slot1,
  FIELD_ftsf193ae_slot1,
  FIELD_ftsf194ae_slot1,
  FIELD_ftsf195ae_slot1,
  FIELD_ftsf196ae_slot1,
  FIELD_ftsf197ae_slot1,
  FIELD_ftsf198ae_slot1,
  FIELD_ftsf199ae_slot1,
  FIELD_ftsf200ae_slot1,
  FIELD_ftsf201ae_slot1,
  FIELD_ftsf202ae_slot1,
  FIELD_ftsf203ae_slot1,
  FIELD_ftsf204ae_slot1,
  FIELD_ftsf205ae_slot1,
  FIELD_ftsf206ae_slot1,
  FIELD_ftsf207ae_slot1,
  FIELD_ftsf208,
  FIELD_ftsf209ae_slot1,
  FIELD_ftsf210ae_slot1,
  FIELD_ftsf211ae_slot1,
  FIELD_ftsf330ae_slot1,
  FIELD_ftsf332ae_slot1,
  FIELD_ftsf334ae_slot1,
  FIELD_ftsf336ae_slot1,
  FIELD_ftsf337ae_slot1,
  FIELD_ftsf338,
  FIELD_ftsf339ae_slot1,
  FIELD_ftsf340,
  FIELD_ftsf341ae_slot1,
  FIELD_ftsf342ae_slot1,
  FIELD_ftsf343ae_slot1,
  FIELD_ftsf344ae_slot1,
  FIELD_ftsf346ae_slot1,
  FIELD_ftsf347,
  FIELD_ftsf348ae_slot1,
  FIELD_ftsf349ae_slot1,
  FIELD_ftsf350ae_slot1,
  FIELD_op0_s4,
  FIELD_ftsf212ae_slot0,
  FIELD_ftsf213ae_slot0,
  FIELD_ftsf214ae_slot0,
  FIELD_ftsf215ae_slot0,
  FIELD_ftsf216ae_slot0,
  FIELD_ftsf217,
  FIELD_ftsf218ae_slot0,
  FIELD_ftsf219ae_slot0,
  FIELD_ftsf220ae_slot0,
  FIELD_ftsf221ae_slot0,
  FIELD_ftsf222ae_slot0,
  FIELD_ftsf223ae_slot0,
  FIELD_ftsf224ae_slot0,
  FIELD_ftsf225ae_slot0,
  FIELD_ftsf226ae_slot0,
  FIELD_ftsf227ae_slot0,
  FIELD_ftsf228ae_slot0,
  FIELD_ftsf229ae_slot0,
  FIELD_ftsf230ae_slot0,
  FIELD_ftsf231ae_slot0,
  FIELD_ftsf232ae_slot0,
  FIELD_ftsf233ae_slot0,
  FIELD_ftsf234ae_slot0,
  FIELD_ftsf235ae_slot0,
  FIELD_ftsf236ae_slot0,
  FIELD_ftsf237ae_slot0,
  FIELD_ftsf238ae_slot0,
  FIELD_ftsf239ae_slot0,
  FIELD_ftsf240ae_slot0,
  FIELD_ftsf241ae_slot0,
  FIELD_ftsf242ae_slot0,
  FIELD_ftsf243ae_slot0,
  FIELD_ftsf244ae_slot0,
  FIELD_ftsf245ae_slot0,
  FIELD_ftsf246ae_slot0,
  FIELD_ftsf247ae_slot0,
  FIELD_ftsf248ae_slot0,
  FIELD_ftsf249ae_slot0,
  FIELD_ftsf250ae_slot0,
  FIELD_ftsf251ae_slot0,
  FIELD_ftsf252ae_slot0,
  FIELD_ftsf253ae_slot0,
  FIELD_ftsf254ae_slot0,
  FIELD_ftsf255ae_slot0,
  FIELD_ftsf256ae_slot0,
  FIELD_ftsf257ae_slot0,
  FIELD_ftsf258ae_slot0,
  FIELD_ftsf259ae_slot0,
  FIELD_ftsf260ae_slot0,
  FIELD_ftsf261ae_slot0,
  FIELD_ftsf262ae_slot0,
  FIELD_ftsf263ae_slot0,
  FIELD_ftsf264ae_slot0,
  FIELD_ftsf265ae_slot0,
  FIELD_ftsf266ae_slot0,
  FIELD_ftsf267ae_slot0,
  FIELD_ftsf268ae_slot0,
  FIELD_ftsf269ae_slot0,
  FIELD_ftsf270ae_slot0,
  FIELD_ftsf271ae_slot0,
  FIELD_ftsf272ae_slot0,
  FIELD_ftsf273ae_slot0,
  FIELD_ftsf274ae_slot0,
  FIELD_ftsf275ae_slot0,
  FIELD_ftsf276ae_slot0,
  FIELD_ftsf277ae_slot0,
  FIELD_ftsf278ae_slot0,
  FIELD_ftsf279ae_slot0,
  FIELD_ftsf281ae_slot0,
  FIELD_ftsf282ae_slot0,
  FIELD_ftsf283ae_slot0,
  FIELD_ftsf284ae_slot0,
  FIELD_ftsf286ae_slot0,
  FIELD_ftsf288ae_slot0,
  FIELD_ftsf290ae_slot0,
  FIELD_ftsf292ae_slot0,
  FIELD_ftsf293,
  FIELD_ftsf294ae_slot0,
  FIELD_ftsf295ae_slot0,
  FIELD_ftsf296ae_slot0,
  FIELD_ftsf297ae_slot0,
  FIELD_ftsf298ae_slot0,
  FIELD_ftsf299ae_slot0,
  FIELD_ftsf300ae_slot0,
  FIELD_ftsf301ae_slot0,
  FIELD_ftsf302ae_slot0,
  FIELD_ftsf303ae_slot0,
  FIELD_ftsf304ae_slot0,
  FIELD_ftsf306ae_slot0,
  FIELD_ftsf308ae_slot0,
  FIELD_ftsf309ae_slot0,
  FIELD_ftsf310ae_slot0,
  FIELD_ftsf311ae_slot0,
  FIELD_ftsf312ae_slot0,
  FIELD_ftsf313ae_slot0,
  FIELD_ftsf314ae_slot0,
  FIELD_ftsf315ae_slot0,
  FIELD_ftsf316ae_slot0,
  FIELD_ftsf317ae_slot0,
  FIELD_ftsf318ae_slot0,
  FIELD_ftsf319,
  FIELD_ftsf320ae_slot0,
  FIELD_ftsf321,
  FIELD_ftsf322ae_slot0,
  FIELD_ftsf323ae_slot0,
  FIELD_ftsf324ae_slot0,
  FIELD_ftsf325ae_slot0,
  FIELD_ftsf326ae_slot0,
  FIELD_ftsf328ae_slot0,
  FIELD_ftsf329ae_slot0,
  FIELD_ftsf352ae_slot0,
  FIELD_ftsf353,
  FIELD_ftsf354ae_slot0,
  FIELD_ftsf356ae_slot0,
  FIELD_ftsf357,
  FIELD_ftsf358ae_slot0,
  FIELD_ftsf359ae_slot0,
  FIELD_ftsf360ae_slot0,
  FIELD_ftsf361ae_slot0,
  FIELD_ftsf362ae_slot0,
  FIELD_ftsf364ae_slot0,
  FIELD_ftsf365ae_slot0,
  FIELD_ftsf366ae_slot0,
  FIELD_ftsf368ae_slot0,
  FIELD_ftsf369ae_slot0,
  FIELD__ar0,
  FIELD__ar4,
  FIELD__ar8,
  FIELD__ar12,
  FIELD__bt16,
  FIELD__bs16,
  FIELD__br16,
  FIELD__brall
};


/* Functional units.  */

static xtensa_funcUnit_internal funcUnits[] = {
  { "ae_add32", 1 },
  { "ae_shift32x4", 1 },
  { "ae_shift32x5", 1 },
  { "ae_subshift", 1 }
};

enum xtensa_funcUnit_id {
  FUNCUNIT_ae_add32,
  FUNCUNIT_ae_shift32x4,
  FUNCUNIT_ae_shift32x5,
  FUNCUNIT_ae_subshift
};


/* Register files.  */

enum xtensa_regfile_id {
  REGFILE_AR,
  REGFILE_BR,
  REGFILE_AE_PR,
  REGFILE_AE_QR,
  REGFILE_BR2,
  REGFILE_BR4,
  REGFILE_BR8,
  REGFILE_BR16
};

static xtensa_regfile_internal regfiles[] = {
  { "AR", "a", REGFILE_AR, 32, 32 },
  { "BR", "b", REGFILE_BR, 1, 16 },
  { "AE_PR", "aep", REGFILE_AE_PR, 48, 8 },
  { "AE_QR", "aeq", REGFILE_AE_QR, 56, 4 },
  { "BR2", "b", REGFILE_BR, 2, 8 },
  { "BR4", "b", REGFILE_BR, 4, 4 },
  { "BR8", "b", REGFILE_BR, 8, 2 },
  { "BR16", "b", REGFILE_BR, 16, 1 }
};


/* Interfaces.  */

static xtensa_interface_internal interfaces[] = {
  { "RMPINT_Out", 12, 0, 0, 'o' },
  { "RMPINT_In", 32, 0, 1, 'i' }
};

enum xtensa_interface_id {
  INTERFACE_RMPINT_Out,
  INTERFACE_RMPINT_In
};


/* Constant tables.  */

/* constant table ai4c */
static const unsigned CONST_TBL_ai4c_0[] = {
  0xffffffff,
  0x1,
  0x2,
  0x3,
  0x4,
  0x5,
  0x6,
  0x7,
  0x8,
  0x9,
  0xa,
  0xb,
  0xc,
  0xd,
  0xe,
  0xf,
  0
};

/* constant table b4c */
static const unsigned CONST_TBL_b4c_0[] = {
  0xffffffff,
  0x1,
  0x2,
  0x3,
  0x4,
  0x5,
  0x6,
  0x7,
  0x8,
  0xa,
  0xc,
  0x10,
  0x20,
  0x40,
  0x80,
  0x100,
  0
};

/* constant table b4cu */
static const unsigned CONST_TBL_b4cu_0[] = {
  0x8000,
  0x10000,
  0x2,
  0x3,
  0x4,
  0x5,
  0x6,
  0x7,
  0x8,
  0xa,
  0xc,
  0x10,
  0x20,
  0x40,
  0x80,
  0x100,
  0
};


/* Instruction operands.  */

static int
Operand_soffsetx4_decode (uint32 *valp)
{
  unsigned soffsetx4_0, offset_0;
  offset_0 = *valp & 0x3ffff;
  soffsetx4_0 = 0x4 + ((((int) offset_0 << 14) >> 14) << 2);
  *valp = soffsetx4_0;
  return 0;
}

static int
Operand_soffsetx4_encode (uint32 *valp)
{
  unsigned offset_0, soffsetx4_0;
  soffsetx4_0 = *valp;
  offset_0 = ((soffsetx4_0 - 0x4) >> 2) & 0x3ffff;
  *valp = offset_0;
  return 0;
}

static int
Operand_soffsetx4_ator (uint32 *valp, uint32 pc)
{
  *valp -= (pc & ~0x3);
  return 0;
}

static int
Operand_soffsetx4_rtoa (uint32 *valp, uint32 pc)
{
  *valp += (pc & ~0x3);
  return 0;
}

static int
Operand_uimm12x8_decode (uint32 *valp)
{
  unsigned uimm12x8_0, imm12_0;
  imm12_0 = *valp & 0xfff;
  uimm12x8_0 = imm12_0 << 3;
  *valp = uimm12x8_0;
  return 0;
}

static int
Operand_uimm12x8_encode (uint32 *valp)
{
  unsigned imm12_0, uimm12x8_0;
  uimm12x8_0 = *valp;
  imm12_0 = ((uimm12x8_0 >> 3) & 0xfff);
  *valp = imm12_0;
  return 0;
}

static int
Operand_simm4_decode (uint32 *valp)
{
  unsigned simm4_0, mn_0;
  mn_0 = *valp & 0xf;
  simm4_0 = ((int) mn_0 << 28) >> 28;
  *valp = simm4_0;
  return 0;
}

static int
Operand_simm4_encode (uint32 *valp)
{
  unsigned mn_0, simm4_0;
  simm4_0 = *valp;
  mn_0 = (simm4_0 & 0xf);
  *valp = mn_0;
  return 0;
}

static int
Operand_arr_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_arr_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0xf) != 0;
  return error;
}

static int
Operand_ars_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_ars_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0xf) != 0;
  return error;
}

static int
Operand_art_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_art_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0xf) != 0;
  return error;
}

static int
Operand_ar0_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_ar0_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0x1f) != 0;
  return error;
}

static int
Operand_ar4_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_ar4_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0x1f) != 0;
  return error;
}

static int
Operand_ar8_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_ar8_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0x1f) != 0;
  return error;
}

static int
Operand_ar12_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_ar12_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0x1f) != 0;
  return error;
}

static int
Operand_ars_entry_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_ars_entry_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0x1f) != 0;
  return error;
}

static int
Operand_immrx4_decode (uint32 *valp)
{
  unsigned immrx4_0, r_0;
  r_0 = *valp & 0xf;
  immrx4_0 = (((0xfffffff) << 4) | r_0) << 2;
  *valp = immrx4_0;
  return 0;
}

static int
Operand_immrx4_encode (uint32 *valp)
{
  unsigned r_0, immrx4_0;
  immrx4_0 = *valp;
  r_0 = ((immrx4_0 >> 2) & 0xf);
  *valp = r_0;
  return 0;
}

static int
Operand_lsi4x4_decode (uint32 *valp)
{
  unsigned lsi4x4_0, r_0;
  r_0 = *valp & 0xf;
  lsi4x4_0 = r_0 << 2;
  *valp = lsi4x4_0;
  return 0;
}

static int
Operand_lsi4x4_encode (uint32 *valp)
{
  unsigned r_0, lsi4x4_0;
  lsi4x4_0 = *valp;
  r_0 = ((lsi4x4_0 >> 2) & 0xf);
  *valp = r_0;
  return 0;
}

static int
Operand_simm7_decode (uint32 *valp)
{
  unsigned simm7_0, imm7_0;
  imm7_0 = *valp & 0x7f;
  simm7_0 = ((((-((((imm7_0 >> 6) & 1)) & (((imm7_0 >> 5) & 1)))) & 0x1ffffff)) << 7) | imm7_0;
  *valp = simm7_0;
  return 0;
}

static int
Operand_simm7_encode (uint32 *valp)
{
  unsigned imm7_0, simm7_0;
  simm7_0 = *valp;
  imm7_0 = (simm7_0 & 0x7f);
  *valp = imm7_0;
  return 0;
}

static int
Operand_uimm6_decode (uint32 *valp)
{
  unsigned uimm6_0, imm6_0;
  imm6_0 = *valp & 0x3f;
  uimm6_0 = 0x4 + (((0) << 6) | imm6_0);
  *valp = uimm6_0;
  return 0;
}

static int
Operand_uimm6_encode (uint32 *valp)
{
  unsigned imm6_0, uimm6_0;
  uimm6_0 = *valp;
  imm6_0 = (uimm6_0 - 0x4) & 0x3f;
  *valp = imm6_0;
  return 0;
}

static int
Operand_uimm6_ator (uint32 *valp, uint32 pc)
{
  *valp -= pc;
  return 0;
}

static int
Operand_uimm6_rtoa (uint32 *valp, uint32 pc)
{
  *valp += pc;
  return 0;
}

static int
Operand_ai4const_decode (uint32 *valp)
{
  unsigned ai4const_0, t_0;
  t_0 = *valp & 0xf;
  ai4const_0 = CONST_TBL_ai4c_0[t_0 & 0xf];
  *valp = ai4const_0;
  return 0;
}

static int
Operand_ai4const_encode (uint32 *valp)
{
  unsigned t_0, ai4const_0;
  ai4const_0 = *valp;
  switch (ai4const_0)
    {
    case 0xffffffff: t_0 = 0; break;
    case 0x1: t_0 = 0x1; break;
    case 0x2: t_0 = 0x2; break;
    case 0x3: t_0 = 0x3; break;
    case 0x4: t_0 = 0x4; break;
    case 0x5: t_0 = 0x5; break;
    case 0x6: t_0 = 0x6; break;
    case 0x7: t_0 = 0x7; break;
    case 0x8: t_0 = 0x8; break;
    case 0x9: t_0 = 0x9; break;
    case 0xa: t_0 = 0xa; break;
    case 0xb: t_0 = 0xb; break;
    case 0xc: t_0 = 0xc; break;
    case 0xd: t_0 = 0xd; break;
    case 0xe: t_0 = 0xe; break;
    default: t_0 = 0xf; break;
    }
  *valp = t_0;
  return 0;
}

static int
Operand_b4const_decode (uint32 *valp)
{
  unsigned b4const_0, r_0;
  r_0 = *valp & 0xf;
  b4const_0 = CONST_TBL_b4c_0[r_0 & 0xf];
  *valp = b4const_0;
  return 0;
}

static int
Operand_b4const_encode (uint32 *valp)
{
  unsigned r_0, b4const_0;
  b4const_0 = *valp;
  switch (b4const_0)
    {
    case 0xffffffff: r_0 = 0; break;
    case 0x1: r_0 = 0x1; break;
    case 0x2: r_0 = 0x2; break;
    case 0x3: r_0 = 0x3; break;
    case 0x4: r_0 = 0x4; break;
    case 0x5: r_0 = 0x5; break;
    case 0x6: r_0 = 0x6; break;
    case 0x7: r_0 = 0x7; break;
    case 0x8: r_0 = 0x8; break;
    case 0xa: r_0 = 0x9; break;
    case 0xc: r_0 = 0xa; break;
    case 0x10: r_0 = 0xb; break;
    case 0x20: r_0 = 0xc; break;
    case 0x40: r_0 = 0xd; break;
    case 0x80: r_0 = 0xe; break;
    default: r_0 = 0xf; break;
    }
  *valp = r_0;
  return 0;
}

static int
Operand_b4constu_decode (uint32 *valp)
{
  unsigned b4constu_0, r_0;
  r_0 = *valp & 0xf;
  b4constu_0 = CONST_TBL_b4cu_0[r_0 & 0xf];
  *valp = b4constu_0;
  return 0;
}

static int
Operand_b4constu_encode (uint32 *valp)
{
  unsigned r_0, b4constu_0;
  b4constu_0 = *valp;
  switch (b4constu_0)
    {
    case 0x8000: r_0 = 0; break;
    case 0x10000: r_0 = 0x1; break;
    case 0x2: r_0 = 0x2; break;
    case 0x3: r_0 = 0x3; break;
    case 0x4: r_0 = 0x4; break;
    case 0x5: r_0 = 0x5; break;
    case 0x6: r_0 = 0x6; break;
    case 0x7: r_0 = 0x7; break;
    case 0x8: r_0 = 0x8; break;
    case 0xa: r_0 = 0x9; break;
    case 0xc: r_0 = 0xa; break;
    case 0x10: r_0 = 0xb; break;
    case 0x20: r_0 = 0xc; break;
    case 0x40: r_0 = 0xd; break;
    case 0x80: r_0 = 0xe; break;
    default: r_0 = 0xf; break;
    }
  *valp = r_0;
  return 0;
}

static int
Operand_uimm8_decode (uint32 *valp)
{
  unsigned uimm8_0, imm8_0;
  imm8_0 = *valp & 0xff;
  uimm8_0 = imm8_0;
  *valp = uimm8_0;
  return 0;
}

static int
Operand_uimm8_encode (uint32 *valp)
{
  unsigned imm8_0, uimm8_0;
  uimm8_0 = *valp;
  imm8_0 = (uimm8_0 & 0xff);
  *valp = imm8_0;
  return 0;
}

static int
Operand_uimm8x2_decode (uint32 *valp)
{
  unsigned uimm8x2_0, imm8_0;
  imm8_0 = *valp & 0xff;
  uimm8x2_0 = imm8_0 << 1;
  *valp = uimm8x2_0;
  return 0;
}

static int
Operand_uimm8x2_encode (uint32 *valp)
{
  unsigned imm8_0, uimm8x2_0;
  uimm8x2_0 = *valp;
  imm8_0 = ((uimm8x2_0 >> 1) & 0xff);
  *valp = imm8_0;
  return 0;
}

static int
Operand_uimm8x4_decode (uint32 *valp)
{
  unsigned uimm8x4_0, imm8_0;
  imm8_0 = *valp & 0xff;
  uimm8x4_0 = imm8_0 << 2;
  *valp = uimm8x4_0;
  return 0;
}

static int
Operand_uimm8x4_encode (uint32 *valp)
{
  unsigned imm8_0, uimm8x4_0;
  uimm8x4_0 = *valp;
  imm8_0 = ((uimm8x4_0 >> 2) & 0xff);
  *valp = imm8_0;
  return 0;
}

static int
Operand_uimm4x16_decode (uint32 *valp)
{
  unsigned uimm4x16_0, op2_0;
  op2_0 = *valp & 0xf;
  uimm4x16_0 = op2_0 << 4;
  *valp = uimm4x16_0;
  return 0;
}

static int
Operand_uimm4x16_encode (uint32 *valp)
{
  unsigned op2_0, uimm4x16_0;
  uimm4x16_0 = *valp;
  op2_0 = ((uimm4x16_0 >> 4) & 0xf);
  *valp = op2_0;
  return 0;
}

static int
Operand_simm8_decode (uint32 *valp)
{
  unsigned simm8_0, imm8_0;
  imm8_0 = *valp & 0xff;
  simm8_0 = ((int) imm8_0 << 24) >> 24;
  *valp = simm8_0;
  return 0;
}

static int
Operand_simm8_encode (uint32 *valp)
{
  unsigned imm8_0, simm8_0;
  simm8_0 = *valp;
  imm8_0 = (simm8_0 & 0xff);
  *valp = imm8_0;
  return 0;
}

static int
Operand_simm8x256_decode (uint32 *valp)
{
  unsigned simm8x256_0, imm8_0;
  imm8_0 = *valp & 0xff;
  simm8x256_0 = (((int) imm8_0 << 24) >> 24) << 8;
  *valp = simm8x256_0;
  return 0;
}

static int
Operand_simm8x256_encode (uint32 *valp)
{
  unsigned imm8_0, simm8x256_0;
  simm8x256_0 = *valp;
  imm8_0 = ((simm8x256_0 >> 8) & 0xff);
  *valp = imm8_0;
  return 0;
}

static int
Operand_simm12b_decode (uint32 *valp)
{
  unsigned simm12b_0, imm12b_0;
  imm12b_0 = *valp & 0xfff;
  simm12b_0 = ((int) imm12b_0 << 20) >> 20;
  *valp = simm12b_0;
  return 0;
}

static int
Operand_simm12b_encode (uint32 *valp)
{
  unsigned imm12b_0, simm12b_0;
  simm12b_0 = *valp;
  imm12b_0 = (simm12b_0 & 0xfff);
  *valp = imm12b_0;
  return 0;
}

static int
Operand_msalp32_decode (uint32 *valp)
{
  unsigned msalp32_0, sal_0;
  sal_0 = *valp & 0x1f;
  msalp32_0 = 0x20 - sal_0;
  *valp = msalp32_0;
  return 0;
}

static int
Operand_msalp32_encode (uint32 *valp)
{
  unsigned sal_0, msalp32_0;
  msalp32_0 = *valp;
  sal_0 = (0x20 - msalp32_0) & 0x1f;
  *valp = sal_0;
  return 0;
}

static int
Operand_op2p1_decode (uint32 *valp)
{
  unsigned op2p1_0, op2_0;
  op2_0 = *valp & 0xf;
  op2p1_0 = op2_0 + 0x1;
  *valp = op2p1_0;
  return 0;
}

static int
Operand_op2p1_encode (uint32 *valp)
{
  unsigned op2_0, op2p1_0;
  op2p1_0 = *valp;
  op2_0 = (op2p1_0 - 0x1) & 0xf;
  *valp = op2_0;
  return 0;
}

static int
Operand_label8_decode (uint32 *valp)
{
  unsigned label8_0, imm8_0;
  imm8_0 = *valp & 0xff;
  label8_0 = 0x4 + (((int) imm8_0 << 24) >> 24);
  *valp = label8_0;
  return 0;
}

static int
Operand_label8_encode (uint32 *valp)
{
  unsigned imm8_0, label8_0;
  label8_0 = *valp;
  imm8_0 = (label8_0 - 0x4) & 0xff;
  *valp = imm8_0;
  return 0;
}

static int
Operand_label8_ator (uint32 *valp, uint32 pc)
{
  *valp -= pc;
  return 0;
}

static int
Operand_label8_rtoa (uint32 *valp, uint32 pc)
{
  *valp += pc;
  return 0;
}

static int
Operand_ulabel8_decode (uint32 *valp)
{
  unsigned ulabel8_0, imm8_0;
  imm8_0 = *valp & 0xff;
  ulabel8_0 = 0x4 + (((0) << 8) | imm8_0);
  *valp = ulabel8_0;
  return 0;
}

static int
Operand_ulabel8_encode (uint32 *valp)
{
  unsigned imm8_0, ulabel8_0;
  ulabel8_0 = *valp;
  imm8_0 = (ulabel8_0 - 0x4) & 0xff;
  *valp = imm8_0;
  return 0;
}

static int
Operand_ulabel8_ator (uint32 *valp, uint32 pc)
{
  *valp -= pc;
  return 0;
}

static int
Operand_ulabel8_rtoa (uint32 *valp, uint32 pc)
{
  *valp += pc;
  return 0;
}

static int
Operand_label12_decode (uint32 *valp)
{
  unsigned label12_0, imm12_0;
  imm12_0 = *valp & 0xfff;
  label12_0 = 0x4 + (((int) imm12_0 << 20) >> 20);
  *valp = label12_0;
  return 0;
}

static int
Operand_label12_encode (uint32 *valp)
{
  unsigned imm12_0, label12_0;
  label12_0 = *valp;
  imm12_0 = (label12_0 - 0x4) & 0xfff;
  *valp = imm12_0;
  return 0;
}

static int
Operand_label12_ator (uint32 *valp, uint32 pc)
{
  *valp -= pc;
  return 0;
}

static int
Operand_label12_rtoa (uint32 *valp, uint32 pc)
{
  *valp += pc;
  return 0;
}

static int
Operand_soffset_decode (uint32 *valp)
{
  unsigned soffset_0, offset_0;
  offset_0 = *valp & 0x3ffff;
  soffset_0 = 0x4 + (((int) offset_0 << 14) >> 14);
  *valp = soffset_0;
  return 0;
}

static int
Operand_soffset_encode (uint32 *valp)
{
  unsigned offset_0, soffset_0;
  soffset_0 = *valp;
  offset_0 = (soffset_0 - 0x4) & 0x3ffff;
  *valp = offset_0;
  return 0;
}

static int
Operand_soffset_ator (uint32 *valp, uint32 pc)
{
  *valp -= pc;
  return 0;
}

static int
Operand_soffset_rtoa (uint32 *valp, uint32 pc)
{
  *valp += pc;
  return 0;
}

static int
Operand_uimm16x4_decode (uint32 *valp)
{
  unsigned uimm16x4_0, imm16_0;
  imm16_0 = *valp & 0xffff;
  uimm16x4_0 = (((0xffff) << 16) | imm16_0) << 2;
  *valp = uimm16x4_0;
  return 0;
}

static int
Operand_uimm16x4_encode (uint32 *valp)
{
  unsigned imm16_0, uimm16x4_0;
  uimm16x4_0 = *valp;
  imm16_0 = (uimm16x4_0 >> 2) & 0xffff;
  *valp = imm16_0;
  return 0;
}

static int
Operand_uimm16x4_ator (uint32 *valp, uint32 pc)
{
  *valp -= ((pc + 3) & ~0x3);
  return 0;
}

static int
Operand_uimm16x4_rtoa (uint32 *valp, uint32 pc)
{
  *valp += ((pc + 3) & ~0x3);
  return 0;
}

static int
Operand_immt_decode (uint32 *valp)
{
  unsigned immt_0, t_0;
  t_0 = *valp & 0xf;
  immt_0 = t_0;
  *valp = immt_0;
  return 0;
}

static int
Operand_immt_encode (uint32 *valp)
{
  unsigned t_0, immt_0;
  immt_0 = *valp;
  t_0 = immt_0 & 0xf;
  *valp = t_0;
  return 0;
}

static int
Operand_imms_decode (uint32 *valp)
{
  unsigned imms_0, s_0;
  s_0 = *valp & 0xf;
  imms_0 = s_0;
  *valp = imms_0;
  return 0;
}

static int
Operand_imms_encode (uint32 *valp)
{
  unsigned s_0, imms_0;
  imms_0 = *valp;
  s_0 = imms_0 & 0xf;
  *valp = s_0;
  return 0;
}

static int
Operand_bt_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_bt_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0xf) != 0;
  return error;
}

static int
Operand_bs_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_bs_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0xf) != 0;
  return error;
}

static int
Operand_br_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_br_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0xf) != 0;
  return error;
}

static int
Operand_bt2_decode (uint32 *valp)
{
  *valp = *valp << 1;
  return 0;
}

static int
Operand_bt2_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~(0x7 << 1)) != 0;
  *valp = *valp >> 1;
  return error;
}

static int
Operand_bs2_decode (uint32 *valp)
{
  *valp = *valp << 1;
  return 0;
}

static int
Operand_bs2_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~(0x7 << 1)) != 0;
  *valp = *valp >> 1;
  return error;
}

static int
Operand_br2_decode (uint32 *valp)
{
  *valp = *valp << 1;
  return 0;
}

static int
Operand_br2_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~(0x7 << 1)) != 0;
  *valp = *valp >> 1;
  return error;
}

static int
Operand_bt4_decode (uint32 *valp)
{
  *valp = *valp << 2;
  return 0;
}

static int
Operand_bt4_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~(0x3 << 2)) != 0;
  *valp = *valp >> 2;
  return error;
}

static int
Operand_bs4_decode (uint32 *valp)
{
  *valp = *valp << 2;
  return 0;
}

static int
Operand_bs4_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~(0x3 << 2)) != 0;
  *valp = *valp >> 2;
  return error;
}

static int
Operand_br4_decode (uint32 *valp)
{
  *valp = *valp << 2;
  return 0;
}

static int
Operand_br4_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~(0x3 << 2)) != 0;
  *valp = *valp >> 2;
  return error;
}

static int
Operand_bt8_decode (uint32 *valp)
{
  *valp = *valp << 3;
  return 0;
}

static int
Operand_bt8_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~(0x1 << 3)) != 0;
  *valp = *valp >> 3;
  return error;
}

static int
Operand_bs8_decode (uint32 *valp)
{
  *valp = *valp << 3;
  return 0;
}

static int
Operand_bs8_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~(0x1 << 3)) != 0;
  *valp = *valp >> 3;
  return error;
}

static int
Operand_br8_decode (uint32 *valp)
{
  *valp = *valp << 3;
  return 0;
}

static int
Operand_br8_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~(0x1 << 3)) != 0;
  *valp = *valp >> 3;
  return error;
}

static int
Operand_bt16_decode (uint32 *valp)
{
  *valp = *valp << 4;
  return 0;
}

static int
Operand_bt16_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~(0 << 4)) != 0;
  *valp = *valp >> 4;
  return error;
}

static int
Operand_bs16_decode (uint32 *valp)
{
  *valp = *valp << 4;
  return 0;
}

static int
Operand_bs16_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~(0 << 4)) != 0;
  *valp = *valp >> 4;
  return error;
}

static int
Operand_br16_decode (uint32 *valp)
{
  *valp = *valp << 4;
  return 0;
}

static int
Operand_br16_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~(0 << 4)) != 0;
  *valp = *valp >> 4;
  return error;
}

static int
Operand_brall_decode (uint32 *valp)
{
  *valp = *valp << 4;
  return 0;
}

static int
Operand_brall_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~(0 << 4)) != 0;
  *valp = *valp >> 4;
  return error;
}

static int
Operand_tp7_decode (uint32 *valp)
{
  unsigned tp7_0, t_0;
  t_0 = *valp & 0xf;
  tp7_0 = t_0 + 0x7;
  *valp = tp7_0;
  return 0;
}

static int
Operand_tp7_encode (uint32 *valp)
{
  unsigned t_0, tp7_0;
  tp7_0 = *valp;
  t_0 = (tp7_0 - 0x7) & 0xf;
  *valp = t_0;
  return 0;
}

static int
Operand_xt_wbr15_label_decode (uint32 *valp)
{
  unsigned xt_wbr15_label_0, xt_wbr15_imm_0;
  xt_wbr15_imm_0 = *valp & 0x7fff;
  xt_wbr15_label_0 = 0x4 + (((int) xt_wbr15_imm_0 << 17) >> 17);
  *valp = xt_wbr15_label_0;
  return 0;
}

static int
Operand_xt_wbr15_label_encode (uint32 *valp)
{
  unsigned xt_wbr15_imm_0, xt_wbr15_label_0;
  xt_wbr15_label_0 = *valp;
  xt_wbr15_imm_0 = (xt_wbr15_label_0 - 0x4) & 0x7fff;
  *valp = xt_wbr15_imm_0;
  return 0;
}

static int
Operand_xt_wbr15_label_ator (uint32 *valp, uint32 pc)
{
  *valp -= pc;
  return 0;
}

static int
Operand_xt_wbr15_label_rtoa (uint32 *valp, uint32 pc)
{
  *valp += pc;
  return 0;
}

static int
Operand_xt_wbr18_label_decode (uint32 *valp)
{
  unsigned xt_wbr18_label_0, xt_wbr18_imm_0;
  xt_wbr18_imm_0 = *valp & 0x3ffff;
  xt_wbr18_label_0 = 0x4 + (((int) xt_wbr18_imm_0 << 14) >> 14);
  *valp = xt_wbr18_label_0;
  return 0;
}

static int
Operand_xt_wbr18_label_encode (uint32 *valp)
{
  unsigned xt_wbr18_imm_0, xt_wbr18_label_0;
  xt_wbr18_label_0 = *valp;
  xt_wbr18_imm_0 = (xt_wbr18_label_0 - 0x4) & 0x3ffff;
  *valp = xt_wbr18_imm_0;
  return 0;
}

static int
Operand_xt_wbr18_label_ator (uint32 *valp, uint32 pc)
{
  *valp -= pc;
  return 0;
}

static int
Operand_xt_wbr18_label_rtoa (uint32 *valp, uint32 pc)
{
  *valp += pc;
  return 0;
}

static int
Operand_ae_samt32_decode (uint32 *valp)
{
  unsigned ae_samt32_0, ftsf14_0;
  ftsf14_0 = *valp & 0x1f;
  ae_samt32_0 = (0 << 5) | ftsf14_0;
  *valp = ae_samt32_0;
  return 0;
}

static int
Operand_ae_samt32_encode (uint32 *valp)
{
  unsigned ftsf14_0, ae_samt32_0;
  ae_samt32_0 = *valp;
  ftsf14_0 = (ae_samt32_0 & 0x1f);
  *valp = ftsf14_0;
  return 0;
}

static int
Operand_pr0_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_pr0_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0x7) != 0;
  return error;
}

static int
Operand_qr0_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_qr0_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0x3) != 0;
  return error;
}

static int
Operand_ae_lsimm16_decode (uint32 *valp)
{
  unsigned ae_lsimm16_0, t_0;
  t_0 = *valp & 0xf;
  ae_lsimm16_0 = (((int) t_0 << 28) >> 28) << 1;
  *valp = ae_lsimm16_0;
  return 0;
}

static int
Operand_ae_lsimm16_encode (uint32 *valp)
{
  unsigned t_0, ae_lsimm16_0;
  ae_lsimm16_0 = *valp;
  t_0 = ((ae_lsimm16_0 >> 1) & 0xf);
  *valp = t_0;
  return 0;
}

static int
Operand_ae_lsimm32_decode (uint32 *valp)
{
  unsigned ae_lsimm32_0, t_0;
  t_0 = *valp & 0xf;
  ae_lsimm32_0 = (((int) t_0 << 28) >> 28) << 2;
  *valp = ae_lsimm32_0;
  return 0;
}

static int
Operand_ae_lsimm32_encode (uint32 *valp)
{
  unsigned t_0, ae_lsimm32_0;
  ae_lsimm32_0 = *valp;
  t_0 = ((ae_lsimm32_0 >> 2) & 0xf);
  *valp = t_0;
  return 0;
}

static int
Operand_ae_lsimm64_decode (uint32 *valp)
{
  unsigned ae_lsimm64_0, t_0;
  t_0 = *valp & 0xf;
  ae_lsimm64_0 = (((int) t_0 << 28) >> 28) << 3;
  *valp = ae_lsimm64_0;
  return 0;
}

static int
Operand_ae_lsimm64_encode (uint32 *valp)
{
  unsigned t_0, ae_lsimm64_0;
  ae_lsimm64_0 = *valp;
  t_0 = ((ae_lsimm64_0 >> 3) & 0xf);
  *valp = t_0;
  return 0;
}

static int
Operand_ae_samt64_decode (uint32 *valp)
{
  unsigned ae_samt64_0, ae_samt_s_t_0;
  ae_samt_s_t_0 = *valp & 0x3f;
  ae_samt64_0 = (0 << 6) | ae_samt_s_t_0;
  *valp = ae_samt64_0;
  return 0;
}

static int
Operand_ae_samt64_encode (uint32 *valp)
{
  unsigned ae_samt_s_t_0, ae_samt64_0;
  ae_samt64_0 = *valp;
  ae_samt_s_t_0 = (ae_samt64_0 & 0x3f);
  *valp = ae_samt_s_t_0;
  return 0;
}

static int
Operand_ae_ohba_decode (uint32 *valp)
{
  unsigned ae_ohba_0, op1_0;
  op1_0 = *valp & 0xf;
  ae_ohba_0 = (0 << 5) | (((((op1_0 & 0xf))) == 0) << 4) | ((op1_0 & 0xf));
  *valp = ae_ohba_0;
  return 0;
}

static int
Operand_ae_ohba_encode (uint32 *valp)
{
  unsigned op1_0, ae_ohba_0;
  ae_ohba_0 = *valp;
  op1_0 = (ae_ohba_0 & 0xf);
  *valp = op1_0;
  return 0;
}

static int
Operand_pr_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_pr_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0x7) != 0;
  return error;
}

static int
Operand_qr0_rw_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_qr0_rw_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0x3) != 0;
  return error;
}

static int
Operand_qr1_w_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_qr1_w_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0x3) != 0;
  return error;
}

static int
Operand_ps_decode (uint32 *valp ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
Operand_ps_encode (uint32 *valp)
{
  int error;
  error = (*valp & ~0x7) != 0;
  return error;
}

static xtensa_operand_internal operands[] = {
  { "soffsetx4", FIELD_offset, -1, 0,
    XTENSA_OPERAND_IS_PCRELATIVE,
    Operand_soffsetx4_encode, Operand_soffsetx4_decode,
    Operand_soffsetx4_ator, Operand_soffsetx4_rtoa },
  { "uimm12x8", FIELD_imm12, -1, 0,
    0,
    Operand_uimm12x8_encode, Operand_uimm12x8_decode,
    0, 0 },
  { "simm4", FIELD_mn, -1, 0,
    0,
    Operand_simm4_encode, Operand_simm4_decode,
    0, 0 },
  { "arr", FIELD_r, REGFILE_AR, 1,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_arr_encode, Operand_arr_decode,
    0, 0 },
  { "ars", FIELD_s, REGFILE_AR, 1,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_ars_encode, Operand_ars_decode,
    0, 0 },
  { "*ars_invisible", FIELD_s, REGFILE_AR, 1,
    XTENSA_OPERAND_IS_REGISTER | XTENSA_OPERAND_IS_INVISIBLE,
    Operand_ars_encode, Operand_ars_decode,
    0, 0 },
  { "art", FIELD_t, REGFILE_AR, 1,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_art_encode, Operand_art_decode,
    0, 0 },
  { "ar0", FIELD__ar0, REGFILE_AR, 1,
    XTENSA_OPERAND_IS_REGISTER | XTENSA_OPERAND_IS_INVISIBLE,
    Operand_ar0_encode, Operand_ar0_decode,
    0, 0 },
  { "ar4", FIELD__ar4, REGFILE_AR, 1,
    XTENSA_OPERAND_IS_REGISTER | XTENSA_OPERAND_IS_INVISIBLE,
    Operand_ar4_encode, Operand_ar4_decode,
    0, 0 },
  { "ar8", FIELD__ar8, REGFILE_AR, 1,
    XTENSA_OPERAND_IS_REGISTER | XTENSA_OPERAND_IS_INVISIBLE,
    Operand_ar8_encode, Operand_ar8_decode,
    0, 0 },
  { "ar12", FIELD__ar12, REGFILE_AR, 1,
    XTENSA_OPERAND_IS_REGISTER | XTENSA_OPERAND_IS_INVISIBLE,
    Operand_ar12_encode, Operand_ar12_decode,
    0, 0 },
  { "ars_entry", FIELD_s, REGFILE_AR, 1,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_ars_entry_encode, Operand_ars_entry_decode,
    0, 0 },
  { "immrx4", FIELD_r, -1, 0,
    0,
    Operand_immrx4_encode, Operand_immrx4_decode,
    0, 0 },
  { "lsi4x4", FIELD_r, -1, 0,
    0,
    Operand_lsi4x4_encode, Operand_lsi4x4_decode,
    0, 0 },
  { "simm7", FIELD_imm7, -1, 0,
    0,
    Operand_simm7_encode, Operand_simm7_decode,
    0, 0 },
  { "uimm6", FIELD_imm6, -1, 0,
    XTENSA_OPERAND_IS_PCRELATIVE,
    Operand_uimm6_encode, Operand_uimm6_decode,
    Operand_uimm6_ator, Operand_uimm6_rtoa },
  { "ai4const", FIELD_t, -1, 0,
    0,
    Operand_ai4const_encode, Operand_ai4const_decode,
    0, 0 },
  { "b4const", FIELD_r, -1, 0,
    0,
    Operand_b4const_encode, Operand_b4const_decode,
    0, 0 },
  { "b4constu", FIELD_r, -1, 0,
    0,
    Operand_b4constu_encode, Operand_b4constu_decode,
    0, 0 },
  { "uimm8", FIELD_imm8, -1, 0,
    0,
    Operand_uimm8_encode, Operand_uimm8_decode,
    0, 0 },
  { "uimm8x2", FIELD_imm8, -1, 0,
    0,
    Operand_uimm8x2_encode, Operand_uimm8x2_decode,
    0, 0 },
  { "uimm8x4", FIELD_imm8, -1, 0,
    0,
    Operand_uimm8x4_encode, Operand_uimm8x4_decode,
    0, 0 },
  { "uimm4x16", FIELD_op2, -1, 0,
    0,
    Operand_uimm4x16_encode, Operand_uimm4x16_decode,
    0, 0 },
  { "simm8", FIELD_imm8, -1, 0,
    0,
    Operand_simm8_encode, Operand_simm8_decode,
    0, 0 },
  { "simm8x256", FIELD_imm8, -1, 0,
    0,
    Operand_simm8x256_encode, Operand_simm8x256_decode,
    0, 0 },
  { "simm12b", FIELD_imm12b, -1, 0,
    0,
    Operand_simm12b_encode, Operand_simm12b_decode,
    0, 0 },
  { "msalp32", FIELD_sal, -1, 0,
    0,
    Operand_msalp32_encode, Operand_msalp32_decode,
    0, 0 },
  { "op2p1", FIELD_op2, -1, 0,
    0,
    Operand_op2p1_encode, Operand_op2p1_decode,
    0, 0 },
  { "label8", FIELD_imm8, -1, 0,
    XTENSA_OPERAND_IS_PCRELATIVE,
    Operand_label8_encode, Operand_label8_decode,
    Operand_label8_ator, Operand_label8_rtoa },
  { "ulabel8", FIELD_imm8, -1, 0,
    XTENSA_OPERAND_IS_PCRELATIVE,
    Operand_ulabel8_encode, Operand_ulabel8_decode,
    Operand_ulabel8_ator, Operand_ulabel8_rtoa },
  { "label12", FIELD_imm12, -1, 0,
    XTENSA_OPERAND_IS_PCRELATIVE,
    Operand_label12_encode, Operand_label12_decode,
    Operand_label12_ator, Operand_label12_rtoa },
  { "soffset", FIELD_offset, -1, 0,
    XTENSA_OPERAND_IS_PCRELATIVE,
    Operand_soffset_encode, Operand_soffset_decode,
    Operand_soffset_ator, Operand_soffset_rtoa },
  { "uimm16x4", FIELD_imm16, -1, 0,
    XTENSA_OPERAND_IS_PCRELATIVE,
    Operand_uimm16x4_encode, Operand_uimm16x4_decode,
    Operand_uimm16x4_ator, Operand_uimm16x4_rtoa },
  { "immt", FIELD_t, -1, 0,
    0,
    Operand_immt_encode, Operand_immt_decode,
    0, 0 },
  { "imms", FIELD_s, -1, 0,
    0,
    Operand_imms_encode, Operand_imms_decode,
    0, 0 },
  { "bt", FIELD_t, REGFILE_BR, 1,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_bt_encode, Operand_bt_decode,
    0, 0 },
  { "bs", FIELD_s, REGFILE_BR, 1,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_bs_encode, Operand_bs_decode,
    0, 0 },
  { "br", FIELD_r, REGFILE_BR, 1,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_br_encode, Operand_br_decode,
    0, 0 },
  { "bt2", FIELD_t2, REGFILE_BR, 2,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_bt2_encode, Operand_bt2_decode,
    0, 0 },
  { "bs2", FIELD_s2, REGFILE_BR, 2,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_bs2_encode, Operand_bs2_decode,
    0, 0 },
  { "br2", FIELD_r2, REGFILE_BR, 2,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_br2_encode, Operand_br2_decode,
    0, 0 },
  { "bt4", FIELD_t4, REGFILE_BR, 4,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_bt4_encode, Operand_bt4_decode,
    0, 0 },
  { "bs4", FIELD_s4, REGFILE_BR, 4,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_bs4_encode, Operand_bs4_decode,
    0, 0 },
  { "br4", FIELD_r4, REGFILE_BR, 4,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_br4_encode, Operand_br4_decode,
    0, 0 },
  { "bt8", FIELD_t8, REGFILE_BR, 8,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_bt8_encode, Operand_bt8_decode,
    0, 0 },
  { "bs8", FIELD_s8, REGFILE_BR, 8,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_bs8_encode, Operand_bs8_decode,
    0, 0 },
  { "br8", FIELD_r8, REGFILE_BR, 8,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_br8_encode, Operand_br8_decode,
    0, 0 },
  { "bt16", FIELD__bt16, REGFILE_BR, 16,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_bt16_encode, Operand_bt16_decode,
    0, 0 },
  { "bs16", FIELD__bs16, REGFILE_BR, 16,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_bs16_encode, Operand_bs16_decode,
    0, 0 },
  { "br16", FIELD__br16, REGFILE_BR, 16,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_br16_encode, Operand_br16_decode,
    0, 0 },
  { "brall", FIELD__brall, REGFILE_BR, 16,
    XTENSA_OPERAND_IS_REGISTER | XTENSA_OPERAND_IS_INVISIBLE,
    Operand_brall_encode, Operand_brall_decode,
    0, 0 },
  { "tp7", FIELD_t, -1, 0,
    0,
    Operand_tp7_encode, Operand_tp7_decode,
    0, 0 },
  { "xt_wbr15_label", FIELD_xt_wbr15_imm, -1, 0,
    XTENSA_OPERAND_IS_PCRELATIVE,
    Operand_xt_wbr15_label_encode, Operand_xt_wbr15_label_decode,
    Operand_xt_wbr15_label_ator, Operand_xt_wbr15_label_rtoa },
  { "xt_wbr18_label", FIELD_xt_wbr18_imm, -1, 0,
    XTENSA_OPERAND_IS_PCRELATIVE,
    Operand_xt_wbr18_label_encode, Operand_xt_wbr18_label_decode,
    Operand_xt_wbr18_label_ator, Operand_xt_wbr18_label_rtoa },
  { "ae_samt32", FIELD_ftsf14, -1, 0,
    0,
    Operand_ae_samt32_encode, Operand_ae_samt32_decode,
    0, 0 },
  { "pr0", FIELD_ftsf12, REGFILE_AE_PR, 1,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_pr0_encode, Operand_pr0_decode,
    0, 0 },
  { "qr0", FIELD_ftsf13, REGFILE_AE_QR, 1,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_qr0_encode, Operand_qr0_decode,
    0, 0 },
  { "ae_lsimm16", FIELD_t, -1, 0,
    0,
    Operand_ae_lsimm16_encode, Operand_ae_lsimm16_decode,
    0, 0 },
  { "ae_lsimm32", FIELD_t, -1, 0,
    0,
    Operand_ae_lsimm32_encode, Operand_ae_lsimm32_decode,
    0, 0 },
  { "ae_lsimm64", FIELD_t, -1, 0,
    0,
    Operand_ae_lsimm64_encode, Operand_ae_lsimm64_decode,
    0, 0 },
  { "ae_samt64", FIELD_ae_samt_s_t, -1, 0,
    0,
    Operand_ae_samt64_encode, Operand_ae_samt64_decode,
    0, 0 },
  { "ae_ohba", FIELD_op1, -1, 0,
    0,
    Operand_ae_ohba_encode, Operand_ae_ohba_decode,
    0, 0 },
  { "pr", FIELD_ae_r20, REGFILE_AE_PR, 1,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_pr_encode, Operand_pr_decode,
    0, 0 },
  { "qr0_rw", FIELD_ae_r10, REGFILE_AE_QR, 1,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_qr0_rw_encode, Operand_qr0_rw_decode,
    0, 0 },
  { "qr1_w", FIELD_ae_r32, REGFILE_AE_QR, 1,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_qr1_w_encode, Operand_qr1_w_decode,
    0, 0 },
  { "ps", FIELD_ae_s20, REGFILE_AE_PR, 1,
    XTENSA_OPERAND_IS_REGISTER,
    Operand_ps_encode, Operand_ps_decode,
    0, 0 },
  { "t", FIELD_t, -1, 0, 0, 0, 0, 0, 0 },
  { "bbi4", FIELD_bbi4, -1, 0, 0, 0, 0, 0, 0 },
  { "bbi", FIELD_bbi, -1, 0, 0, 0, 0, 0, 0 },
  { "imm12", FIELD_imm12, -1, 0, 0, 0, 0, 0, 0 },
  { "imm8", FIELD_imm8, -1, 0, 0, 0, 0, 0, 0 },
  { "s", FIELD_s, -1, 0, 0, 0, 0, 0, 0 },
  { "imm12b", FIELD_imm12b, -1, 0, 0, 0, 0, 0, 0 },
  { "imm16", FIELD_imm16, -1, 0, 0, 0, 0, 0, 0 },
  { "m", FIELD_m, -1, 0, 0, 0, 0, 0, 0 },
  { "n", FIELD_n, -1, 0, 0, 0, 0, 0, 0 },
  { "offset", FIELD_offset, -1, 0, 0, 0, 0, 0, 0 },
  { "op0", FIELD_op0, -1, 0, 0, 0, 0, 0, 0 },
  { "op1", FIELD_op1, -1, 0, 0, 0, 0, 0, 0 },
  { "op2", FIELD_op2, -1, 0, 0, 0, 0, 0, 0 },
  { "r", FIELD_r, -1, 0, 0, 0, 0, 0, 0 },
  { "sa4", FIELD_sa4, -1, 0, 0, 0, 0, 0, 0 },
  { "sae4", FIELD_sae4, -1, 0, 0, 0, 0, 0, 0 },
  { "sae", FIELD_sae, -1, 0, 0, 0, 0, 0, 0 },
  { "sal", FIELD_sal, -1, 0, 0, 0, 0, 0, 0 },
  { "sargt", FIELD_sargt, -1, 0, 0, 0, 0, 0, 0 },
  { "sas4", FIELD_sas4, -1, 0, 0, 0, 0, 0, 0 },
  { "sas", FIELD_sas, -1, 0, 0, 0, 0, 0, 0 },
  { "sr", FIELD_sr, -1, 0, 0, 0, 0, 0, 0 },
  { "st", FIELD_st, -1, 0, 0, 0, 0, 0, 0 },
  { "thi3", FIELD_thi3, -1, 0, 0, 0, 0, 0, 0 },
  { "imm4", FIELD_imm4, -1, 0, 0, 0, 0, 0, 0 },
  { "mn", FIELD_mn, -1, 0, 0, 0, 0, 0, 0 },
  { "i", FIELD_i, -1, 0, 0, 0, 0, 0, 0 },
  { "imm6lo", FIELD_imm6lo, -1, 0, 0, 0, 0, 0, 0 },
  { "imm6hi", FIELD_imm6hi, -1, 0, 0, 0, 0, 0, 0 },
  { "imm7lo", FIELD_imm7lo, -1, 0, 0, 0, 0, 0, 0 },
  { "imm7hi", FIELD_imm7hi, -1, 0, 0, 0, 0, 0, 0 },
  { "z", FIELD_z, -1, 0, 0, 0, 0, 0, 0 },
  { "imm6", FIELD_imm6, -1, 0, 0, 0, 0, 0, 0 },
  { "imm7", FIELD_imm7, -1, 0, 0, 0, 0, 0, 0 },
  { "t2", FIELD_t2, -1, 0, 0, 0, 0, 0, 0 },
  { "s2", FIELD_s2, -1, 0, 0, 0, 0, 0, 0 },
  { "r2", FIELD_r2, -1, 0, 0, 0, 0, 0, 0 },
  { "t4", FIELD_t4, -1, 0, 0, 0, 0, 0, 0 },
  { "s4", FIELD_s4, -1, 0, 0, 0, 0, 0, 0 },
  { "r4", FIELD_r4, -1, 0, 0, 0, 0, 0, 0 },
  { "t8", FIELD_t8, -1, 0, 0, 0, 0, 0, 0 },
  { "s8", FIELD_s8, -1, 0, 0, 0, 0, 0, 0 },
  { "r8", FIELD_r8, -1, 0, 0, 0, 0, 0, 0 },
  { "xt_wbr15_imm", FIELD_xt_wbr15_imm, -1, 0, 0, 0, 0, 0, 0 },
  { "xt_wbr18_imm", FIELD_xt_wbr18_imm, -1, 0, 0, 0, 0, 0, 0 },
  { "ae_r3", FIELD_ae_r3, -1, 0, 0, 0, 0, 0, 0 },
  { "ae_s_non_samt", FIELD_ae_s_non_samt, -1, 0, 0, 0, 0, 0, 0 },
  { "ae_s3", FIELD_ae_s3, -1, 0, 0, 0, 0, 0, 0 },
  { "ae_r32", FIELD_ae_r32, -1, 0, 0, 0, 0, 0, 0 },
  { "ae_samt_s_t", FIELD_ae_samt_s_t, -1, 0, 0, 0, 0, 0, 0 },
  { "ae_r20", FIELD_ae_r20, -1, 0, 0, 0, 0, 0, 0 },
  { "ae_r10", FIELD_ae_r10, -1, 0, 0, 0, 0, 0, 0 },
  { "ae_s20", FIELD_ae_s20, -1, 0, 0, 0, 0, 0, 0 },
  { "op0_s3", FIELD_op0_s3, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf12", FIELD_ftsf12, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf13", FIELD_ftsf13, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf14", FIELD_ftsf14, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf21ae_slot1", FIELD_ftsf21ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf22ae_slot1", FIELD_ftsf22ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf23ae_slot1", FIELD_ftsf23ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf24ae_slot1", FIELD_ftsf24ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf25ae_slot1", FIELD_ftsf25ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf26ae_slot1", FIELD_ftsf26ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf27ae_slot1", FIELD_ftsf27ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf28ae_slot1", FIELD_ftsf28ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf29ae_slot1", FIELD_ftsf29ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf30ae_slot1", FIELD_ftsf30ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf31ae_slot1", FIELD_ftsf31ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf32ae_slot1", FIELD_ftsf32ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf33ae_slot1", FIELD_ftsf33ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf34ae_slot1", FIELD_ftsf34ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf35ae_slot1", FIELD_ftsf35ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf36ae_slot1", FIELD_ftsf36ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf37ae_slot1", FIELD_ftsf37ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf38ae_slot1", FIELD_ftsf38ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf39ae_slot1", FIELD_ftsf39ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf40ae_slot1", FIELD_ftsf40ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf41ae_slot1", FIELD_ftsf41ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf42ae_slot1", FIELD_ftsf42ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf43ae_slot1", FIELD_ftsf43ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf44ae_slot1", FIELD_ftsf44ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf45ae_slot1", FIELD_ftsf45ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf46ae_slot1", FIELD_ftsf46ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf47ae_slot1", FIELD_ftsf47ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf48ae_slot1", FIELD_ftsf48ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf49ae_slot1", FIELD_ftsf49ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf50ae_slot1", FIELD_ftsf50ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf51ae_slot1", FIELD_ftsf51ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf52ae_slot1", FIELD_ftsf52ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf53ae_slot1", FIELD_ftsf53ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf54ae_slot1", FIELD_ftsf54ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf55ae_slot1", FIELD_ftsf55ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf56ae_slot1", FIELD_ftsf56ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf57ae_slot1", FIELD_ftsf57ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf58ae_slot1", FIELD_ftsf58ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf59ae_slot1", FIELD_ftsf59ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf60ae_slot1", FIELD_ftsf60ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf61ae_slot1", FIELD_ftsf61ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf63ae_slot1", FIELD_ftsf63ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf64ae_slot1", FIELD_ftsf64ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf66ae_slot1", FIELD_ftsf66ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf67ae_slot1", FIELD_ftsf67ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf69ae_slot1", FIELD_ftsf69ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf71ae_slot1", FIELD_ftsf71ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf72ae_slot1", FIELD_ftsf72ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf73ae_slot1", FIELD_ftsf73ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf75ae_slot1", FIELD_ftsf75ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf76ae_slot1", FIELD_ftsf76ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf77ae_slot1", FIELD_ftsf77ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf78ae_slot1", FIELD_ftsf78ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf79ae_slot1", FIELD_ftsf79ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf80ae_slot1", FIELD_ftsf80ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf81ae_slot1", FIELD_ftsf81ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf82ae_slot1", FIELD_ftsf82ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf84ae_slot1", FIELD_ftsf84ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf86ae_slot1", FIELD_ftsf86ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf87ae_slot1", FIELD_ftsf87ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf88ae_slot1", FIELD_ftsf88ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf89ae_slot1", FIELD_ftsf89ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf90ae_slot1", FIELD_ftsf90ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf91ae_slot1", FIELD_ftsf91ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf92ae_slot1", FIELD_ftsf92ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf94ae_slot1", FIELD_ftsf94ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf96ae_slot1", FIELD_ftsf96ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf97ae_slot1", FIELD_ftsf97ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf98ae_slot1", FIELD_ftsf98ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf99ae_slot1", FIELD_ftsf99ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf100ae_slot1", FIELD_ftsf100ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf101ae_slot1", FIELD_ftsf101ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf103ae_slot1", FIELD_ftsf103ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf104ae_slot1", FIELD_ftsf104ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf105ae_slot1", FIELD_ftsf105ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf106ae_slot1", FIELD_ftsf106ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf107ae_slot1", FIELD_ftsf107ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf108ae_slot1", FIELD_ftsf108ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf109ae_slot1", FIELD_ftsf109ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf110ae_slot1", FIELD_ftsf110ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf111ae_slot1", FIELD_ftsf111ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf112ae_slot1", FIELD_ftsf112ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf113ae_slot1", FIELD_ftsf113ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf114ae_slot1", FIELD_ftsf114ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf115ae_slot1", FIELD_ftsf115ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf116ae_slot1", FIELD_ftsf116ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf118ae_slot1", FIELD_ftsf118ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf119ae_slot1", FIELD_ftsf119ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf120ae_slot1", FIELD_ftsf120ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf122ae_slot1", FIELD_ftsf122ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf124ae_slot1", FIELD_ftsf124ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf125ae_slot1", FIELD_ftsf125ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf126ae_slot1", FIELD_ftsf126ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf127ae_slot1", FIELD_ftsf127ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf128ae_slot1", FIELD_ftsf128ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf129ae_slot1", FIELD_ftsf129ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf130ae_slot1", FIELD_ftsf130ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf131ae_slot1", FIELD_ftsf131ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf132ae_slot1", FIELD_ftsf132ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf133ae_slot1", FIELD_ftsf133ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf134ae_slot1", FIELD_ftsf134ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf135ae_slot1", FIELD_ftsf135ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf136ae_slot1", FIELD_ftsf136ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf137ae_slot1", FIELD_ftsf137ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf138ae_slot1", FIELD_ftsf138ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf139ae_slot1", FIELD_ftsf139ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf140ae_slot1", FIELD_ftsf140ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf141ae_slot1", FIELD_ftsf141ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf142ae_slot1", FIELD_ftsf142ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf143ae_slot1", FIELD_ftsf143ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf144ae_slot1", FIELD_ftsf144ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf145ae_slot1", FIELD_ftsf145ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf146ae_slot1", FIELD_ftsf146ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf147ae_slot1", FIELD_ftsf147ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf148ae_slot1", FIELD_ftsf148ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf149ae_slot1", FIELD_ftsf149ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf150ae_slot1", FIELD_ftsf150ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf151ae_slot1", FIELD_ftsf151ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf152ae_slot1", FIELD_ftsf152ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf153ae_slot1", FIELD_ftsf153ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf154ae_slot1", FIELD_ftsf154ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf155ae_slot1", FIELD_ftsf155ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf156ae_slot1", FIELD_ftsf156ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf157ae_slot1", FIELD_ftsf157ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf158ae_slot1", FIELD_ftsf158ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf159ae_slot1", FIELD_ftsf159ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf160ae_slot1", FIELD_ftsf160ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf161ae_slot1", FIELD_ftsf161ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf162ae_slot1", FIELD_ftsf162ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf163ae_slot1", FIELD_ftsf163ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf164ae_slot1", FIELD_ftsf164ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf165ae_slot1", FIELD_ftsf165ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf166ae_slot1", FIELD_ftsf166ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf167ae_slot1", FIELD_ftsf167ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf168ae_slot1", FIELD_ftsf168ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf169ae_slot1", FIELD_ftsf169ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf170ae_slot1", FIELD_ftsf170ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf171ae_slot1", FIELD_ftsf171ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf172ae_slot1", FIELD_ftsf172ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf173ae_slot1", FIELD_ftsf173ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf174ae_slot1", FIELD_ftsf174ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf175ae_slot1", FIELD_ftsf175ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf176ae_slot1", FIELD_ftsf176ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf177ae_slot1", FIELD_ftsf177ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf178ae_slot1", FIELD_ftsf178ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf179ae_slot1", FIELD_ftsf179ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf180ae_slot1", FIELD_ftsf180ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf181ae_slot1", FIELD_ftsf181ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf182ae_slot1", FIELD_ftsf182ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf183ae_slot1", FIELD_ftsf183ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf184ae_slot1", FIELD_ftsf184ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf185ae_slot1", FIELD_ftsf185ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf186ae_slot1", FIELD_ftsf186ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf187ae_slot1", FIELD_ftsf187ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf188ae_slot1", FIELD_ftsf188ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf189ae_slot1", FIELD_ftsf189ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf190ae_slot1", FIELD_ftsf190ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf191ae_slot1", FIELD_ftsf191ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf192ae_slot1", FIELD_ftsf192ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf193ae_slot1", FIELD_ftsf193ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf194ae_slot1", FIELD_ftsf194ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf195ae_slot1", FIELD_ftsf195ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf196ae_slot1", FIELD_ftsf196ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf197ae_slot1", FIELD_ftsf197ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf198ae_slot1", FIELD_ftsf198ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf199ae_slot1", FIELD_ftsf199ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf200ae_slot1", FIELD_ftsf200ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf201ae_slot1", FIELD_ftsf201ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf202ae_slot1", FIELD_ftsf202ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf203ae_slot1", FIELD_ftsf203ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf204ae_slot1", FIELD_ftsf204ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf205ae_slot1", FIELD_ftsf205ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf206ae_slot1", FIELD_ftsf206ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf207ae_slot1", FIELD_ftsf207ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf208", FIELD_ftsf208, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf209ae_slot1", FIELD_ftsf209ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf210ae_slot1", FIELD_ftsf210ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf211ae_slot1", FIELD_ftsf211ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf330ae_slot1", FIELD_ftsf330ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf332ae_slot1", FIELD_ftsf332ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf334ae_slot1", FIELD_ftsf334ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf336ae_slot1", FIELD_ftsf336ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf337ae_slot1", FIELD_ftsf337ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf338", FIELD_ftsf338, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf339ae_slot1", FIELD_ftsf339ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf340", FIELD_ftsf340, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf341ae_slot1", FIELD_ftsf341ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf342ae_slot1", FIELD_ftsf342ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf343ae_slot1", FIELD_ftsf343ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf344ae_slot1", FIELD_ftsf344ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf346ae_slot1", FIELD_ftsf346ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf347", FIELD_ftsf347, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf348ae_slot1", FIELD_ftsf348ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf349ae_slot1", FIELD_ftsf349ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf350ae_slot1", FIELD_ftsf350ae_slot1, -1, 0, 0, 0, 0, 0, 0 },
  { "op0_s4", FIELD_op0_s4, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf212ae_slot0", FIELD_ftsf212ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf213ae_slot0", FIELD_ftsf213ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf214ae_slot0", FIELD_ftsf214ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf215ae_slot0", FIELD_ftsf215ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf216ae_slot0", FIELD_ftsf216ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf217", FIELD_ftsf217, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf218ae_slot0", FIELD_ftsf218ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf219ae_slot0", FIELD_ftsf219ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf220ae_slot0", FIELD_ftsf220ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf221ae_slot0", FIELD_ftsf221ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf222ae_slot0", FIELD_ftsf222ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf223ae_slot0", FIELD_ftsf223ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf224ae_slot0", FIELD_ftsf224ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf225ae_slot0", FIELD_ftsf225ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf226ae_slot0", FIELD_ftsf226ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf227ae_slot0", FIELD_ftsf227ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf228ae_slot0", FIELD_ftsf228ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf229ae_slot0", FIELD_ftsf229ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf230ae_slot0", FIELD_ftsf230ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf231ae_slot0", FIELD_ftsf231ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf232ae_slot0", FIELD_ftsf232ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf233ae_slot0", FIELD_ftsf233ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf234ae_slot0", FIELD_ftsf234ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf235ae_slot0", FIELD_ftsf235ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf236ae_slot0", FIELD_ftsf236ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf237ae_slot0", FIELD_ftsf237ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf238ae_slot0", FIELD_ftsf238ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf239ae_slot0", FIELD_ftsf239ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf240ae_slot0", FIELD_ftsf240ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf241ae_slot0", FIELD_ftsf241ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf242ae_slot0", FIELD_ftsf242ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf243ae_slot0", FIELD_ftsf243ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf244ae_slot0", FIELD_ftsf244ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf245ae_slot0", FIELD_ftsf245ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf246ae_slot0", FIELD_ftsf246ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf247ae_slot0", FIELD_ftsf247ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf248ae_slot0", FIELD_ftsf248ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf249ae_slot0", FIELD_ftsf249ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf250ae_slot0", FIELD_ftsf250ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf251ae_slot0", FIELD_ftsf251ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf252ae_slot0", FIELD_ftsf252ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf253ae_slot0", FIELD_ftsf253ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf254ae_slot0", FIELD_ftsf254ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf255ae_slot0", FIELD_ftsf255ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf256ae_slot0", FIELD_ftsf256ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf257ae_slot0", FIELD_ftsf257ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf258ae_slot0", FIELD_ftsf258ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf259ae_slot0", FIELD_ftsf259ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf260ae_slot0", FIELD_ftsf260ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf261ae_slot0", FIELD_ftsf261ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf262ae_slot0", FIELD_ftsf262ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf263ae_slot0", FIELD_ftsf263ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf264ae_slot0", FIELD_ftsf264ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf265ae_slot0", FIELD_ftsf265ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf266ae_slot0", FIELD_ftsf266ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf267ae_slot0", FIELD_ftsf267ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf268ae_slot0", FIELD_ftsf268ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf269ae_slot0", FIELD_ftsf269ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf270ae_slot0", FIELD_ftsf270ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf271ae_slot0", FIELD_ftsf271ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf272ae_slot0", FIELD_ftsf272ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf273ae_slot0", FIELD_ftsf273ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf274ae_slot0", FIELD_ftsf274ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf275ae_slot0", FIELD_ftsf275ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf276ae_slot0", FIELD_ftsf276ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf277ae_slot0", FIELD_ftsf277ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf278ae_slot0", FIELD_ftsf278ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf279ae_slot0", FIELD_ftsf279ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf281ae_slot0", FIELD_ftsf281ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf282ae_slot0", FIELD_ftsf282ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf283ae_slot0", FIELD_ftsf283ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf284ae_slot0", FIELD_ftsf284ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf286ae_slot0", FIELD_ftsf286ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf288ae_slot0", FIELD_ftsf288ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf290ae_slot0", FIELD_ftsf290ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf292ae_slot0", FIELD_ftsf292ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf293", FIELD_ftsf293, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf294ae_slot0", FIELD_ftsf294ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf295ae_slot0", FIELD_ftsf295ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf296ae_slot0", FIELD_ftsf296ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf297ae_slot0", FIELD_ftsf297ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf298ae_slot0", FIELD_ftsf298ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf299ae_slot0", FIELD_ftsf299ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf300ae_slot0", FIELD_ftsf300ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf301ae_slot0", FIELD_ftsf301ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf302ae_slot0", FIELD_ftsf302ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf303ae_slot0", FIELD_ftsf303ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf304ae_slot0", FIELD_ftsf304ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf306ae_slot0", FIELD_ftsf306ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf308ae_slot0", FIELD_ftsf308ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf309ae_slot0", FIELD_ftsf309ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf310ae_slot0", FIELD_ftsf310ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf311ae_slot0", FIELD_ftsf311ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf312ae_slot0", FIELD_ftsf312ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf313ae_slot0", FIELD_ftsf313ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf314ae_slot0", FIELD_ftsf314ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf315ae_slot0", FIELD_ftsf315ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf316ae_slot0", FIELD_ftsf316ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf317ae_slot0", FIELD_ftsf317ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf318ae_slot0", FIELD_ftsf318ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf319", FIELD_ftsf319, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf320ae_slot0", FIELD_ftsf320ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf321", FIELD_ftsf321, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf322ae_slot0", FIELD_ftsf322ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf323ae_slot0", FIELD_ftsf323ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf324ae_slot0", FIELD_ftsf324ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf325ae_slot0", FIELD_ftsf325ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf326ae_slot0", FIELD_ftsf326ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf328ae_slot0", FIELD_ftsf328ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf329ae_slot0", FIELD_ftsf329ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf352ae_slot0", FIELD_ftsf352ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf353", FIELD_ftsf353, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf354ae_slot0", FIELD_ftsf354ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf356ae_slot0", FIELD_ftsf356ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf357", FIELD_ftsf357, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf358ae_slot0", FIELD_ftsf358ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf359ae_slot0", FIELD_ftsf359ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf360ae_slot0", FIELD_ftsf360ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf361ae_slot0", FIELD_ftsf361ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf362ae_slot0", FIELD_ftsf362ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf364ae_slot0", FIELD_ftsf364ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf365ae_slot0", FIELD_ftsf365ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf366ae_slot0", FIELD_ftsf366ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf368ae_slot0", FIELD_ftsf368ae_slot0, -1, 0, 0, 0, 0, 0, 0 },
  { "ftsf369ae_slot0", FIELD_ftsf369ae_slot0, -1, 0, 0, 0, 0, 0, 0 }
};

enum xtensa_operand_id {
  OPERAND_soffsetx4,
  OPERAND_uimm12x8,
  OPERAND_simm4,
  OPERAND_arr,
  OPERAND_ars,
  OPERAND__ars_invisible,
  OPERAND_art,
  OPERAND_ar0,
  OPERAND_ar4,
  OPERAND_ar8,
  OPERAND_ar12,
  OPERAND_ars_entry,
  OPERAND_immrx4,
  OPERAND_lsi4x4,
  OPERAND_simm7,
  OPERAND_uimm6,
  OPERAND_ai4const,
  OPERAND_b4const,
  OPERAND_b4constu,
  OPERAND_uimm8,
  OPERAND_uimm8x2,
  OPERAND_uimm8x4,
  OPERAND_uimm4x16,
  OPERAND_simm8,
  OPERAND_simm8x256,
  OPERAND_simm12b,
  OPERAND_msalp32,
  OPERAND_op2p1,
  OPERAND_label8,
  OPERAND_ulabel8,
  OPERAND_label12,
  OPERAND_soffset,
  OPERAND_uimm16x4,
  OPERAND_immt,
  OPERAND_imms,
  OPERAND_bt,
  OPERAND_bs,
  OPERAND_br,
  OPERAND_bt2,
  OPERAND_bs2,
  OPERAND_br2,
  OPERAND_bt4,
  OPERAND_bs4,
  OPERAND_br4,
  OPERAND_bt8,
  OPERAND_bs8,
  OPERAND_br8,
  OPERAND_bt16,
  OPERAND_bs16,
  OPERAND_br16,
  OPERAND_brall,
  OPERAND_tp7,
  OPERAND_xt_wbr15_label,
  OPERAND_xt_wbr18_label,
  OPERAND_ae_samt32,
  OPERAND_pr0,
  OPERAND_qr0,
  OPERAND_ae_lsimm16,
  OPERAND_ae_lsimm32,
  OPERAND_ae_lsimm64,
  OPERAND_ae_samt64,
  OPERAND_ae_ohba,
  OPERAND_pr,
  OPERAND_qr0_rw,
  OPERAND_qr1_w,
  OPERAND_ps,
  OPERAND_t,
  OPERAND_bbi4,
  OPERAND_bbi,
  OPERAND_imm12,
  OPERAND_imm8,
  OPERAND_s,
  OPERAND_imm12b,
  OPERAND_imm16,
  OPERAND_m,
  OPERAND_n,
  OPERAND_offset,
  OPERAND_op0,
  OPERAND_op1,
  OPERAND_op2,
  OPERAND_r,
  OPERAND_sa4,
  OPERAND_sae4,
  OPERAND_sae,
  OPERAND_sal,
  OPERAND_sargt,
  OPERAND_sas4,
  OPERAND_sas,
  OPERAND_sr,
  OPERAND_st,
  OPERAND_thi3,
  OPERAND_imm4,
  OPERAND_mn,
  OPERAND_i,
  OPERAND_imm6lo,
  OPERAND_imm6hi,
  OPERAND_imm7lo,
  OPERAND_imm7hi,
  OPERAND_z,
  OPERAND_imm6,
  OPERAND_imm7,
  OPERAND_t2,
  OPERAND_s2,
  OPERAND_r2,
  OPERAND_t4,
  OPERAND_s4,
  OPERAND_r4,
  OPERAND_t8,
  OPERAND_s8,
  OPERAND_r8,
  OPERAND_xt_wbr15_imm,
  OPERAND_xt_wbr18_imm,
  OPERAND_ae_r3,
  OPERAND_ae_s_non_samt,
  OPERAND_ae_s3,
  OPERAND_ae_r32,
  OPERAND_ae_samt_s_t,
  OPERAND_ae_r20,
  OPERAND_ae_r10,
  OPERAND_ae_s20,
  OPERAND_op0_s3,
  OPERAND_ftsf12,
  OPERAND_ftsf13,
  OPERAND_ftsf14,
  OPERAND_ftsf21ae_slot1,
  OPERAND_ftsf22ae_slot1,
  OPERAND_ftsf23ae_slot1,
  OPERAND_ftsf24ae_slot1,
  OPERAND_ftsf25ae_slot1,
  OPERAND_ftsf26ae_slot1,
  OPERAND_ftsf27ae_slot1,
  OPERAND_ftsf28ae_slot1,
  OPERAND_ftsf29ae_slot1,
  OPERAND_ftsf30ae_slot1,
  OPERAND_ftsf31ae_slot1,
  OPERAND_ftsf32ae_slot1,
  OPERAND_ftsf33ae_slot1,
  OPERAND_ftsf34ae_slot1,
  OPERAND_ftsf35ae_slot1,
  OPERAND_ftsf36ae_slot1,
  OPERAND_ftsf37ae_slot1,
  OPERAND_ftsf38ae_slot1,
  OPERAND_ftsf39ae_slot1,
  OPERAND_ftsf40ae_slot1,
  OPERAND_ftsf41ae_slot1,
  OPERAND_ftsf42ae_slot1,
  OPERAND_ftsf43ae_slot1,
  OPERAND_ftsf44ae_slot1,
  OPERAND_ftsf45ae_slot1,
  OPERAND_ftsf46ae_slot1,
  OPERAND_ftsf47ae_slot1,
  OPERAND_ftsf48ae_slot1,
  OPERAND_ftsf49ae_slot1,
  OPERAND_ftsf50ae_slot1,
  OPERAND_ftsf51ae_slot1,
  OPERAND_ftsf52ae_slot1,
  OPERAND_ftsf53ae_slot1,
  OPERAND_ftsf54ae_slot1,
  OPERAND_ftsf55ae_slot1,
  OPERAND_ftsf56ae_slot1,
  OPERAND_ftsf57ae_slot1,
  OPERAND_ftsf58ae_slot1,
  OPERAND_ftsf59ae_slot1,
  OPERAND_ftsf60ae_slot1,
  OPERAND_ftsf61ae_slot1,
  OPERAND_ftsf63ae_slot1,
  OPERAND_ftsf64ae_slot1,
  OPERAND_ftsf66ae_slot1,
  OPERAND_ftsf67ae_slot1,
  OPERAND_ftsf69ae_slot1,
  OPERAND_ftsf71ae_slot1,
  OPERAND_ftsf72ae_slot1,
  OPERAND_ftsf73ae_slot1,
  OPERAND_ftsf75ae_slot1,
  OPERAND_ftsf76ae_slot1,
  OPERAND_ftsf77ae_slot1,
  OPERAND_ftsf78ae_slot1,
  OPERAND_ftsf79ae_slot1,
  OPERAND_ftsf80ae_slot1,
  OPERAND_ftsf81ae_slot1,
  OPERAND_ftsf82ae_slot1,
  OPERAND_ftsf84ae_slot1,
  OPERAND_ftsf86ae_slot1,
  OPERAND_ftsf87ae_slot1,
  OPERAND_ftsf88ae_slot1,
  OPERAND_ftsf89ae_slot1,
  OPERAND_ftsf90ae_slot1,
  OPERAND_ftsf91ae_slot1,
  OPERAND_ftsf92ae_slot1,
  OPERAND_ftsf94ae_slot1,
  OPERAND_ftsf96ae_slot1,
  OPERAND_ftsf97ae_slot1,
  OPERAND_ftsf98ae_slot1,
  OPERAND_ftsf99ae_slot1,
  OPERAND_ftsf100ae_slot1,
  OPERAND_ftsf101ae_slot1,
  OPERAND_ftsf103ae_slot1,
  OPERAND_ftsf104ae_slot1,
  OPERAND_ftsf105ae_slot1,
  OPERAND_ftsf106ae_slot1,
  OPERAND_ftsf107ae_slot1,
  OPERAND_ftsf108ae_slot1,
  OPERAND_ftsf109ae_slot1,
  OPERAND_ftsf110ae_slot1,
  OPERAND_ftsf111ae_slot1,
  OPERAND_ftsf112ae_slot1,
  OPERAND_ftsf113ae_slot1,
  OPERAND_ftsf114ae_slot1,
  OPERAND_ftsf115ae_slot1,
  OPERAND_ftsf116ae_slot1,
  OPERAND_ftsf118ae_slot1,
  OPERAND_ftsf119ae_slot1,
  OPERAND_ftsf120ae_slot1,
  OPERAND_ftsf122ae_slot1,
  OPERAND_ftsf124ae_slot1,
  OPERAND_ftsf125ae_slot1,
  OPERAND_ftsf126ae_slot1,
  OPERAND_ftsf127ae_slot1,
  OPERAND_ftsf128ae_slot1,
  OPERAND_ftsf129ae_slot1,
  OPERAND_ftsf130ae_slot1,
  OPERAND_ftsf131ae_slot1,
  OPERAND_ftsf132ae_slot1,
  OPERAND_ftsf133ae_slot1,
  OPERAND_ftsf134ae_slot1,
  OPERAND_ftsf135ae_slot1,
  OPERAND_ftsf136ae_slot1,
  OPERAND_ftsf137ae_slot1,
  OPERAND_ftsf138ae_slot1,
  OPERAND_ftsf139ae_slot1,
  OPERAND_ftsf140ae_slot1,
  OPERAND_ftsf141ae_slot1,
  OPERAND_ftsf142ae_slot1,
  OPERAND_ftsf143ae_slot1,
  OPERAND_ftsf144ae_slot1,
  OPERAND_ftsf145ae_slot1,
  OPERAND_ftsf146ae_slot1,
  OPERAND_ftsf147ae_slot1,
  OPERAND_ftsf148ae_slot1,
  OPERAND_ftsf149ae_slot1,
  OPERAND_ftsf150ae_slot1,
  OPERAND_ftsf151ae_slot1,
  OPERAND_ftsf152ae_slot1,
  OPERAND_ftsf153ae_slot1,
  OPERAND_ftsf154ae_slot1,
  OPERAND_ftsf155ae_slot1,
  OPERAND_ftsf156ae_slot1,
  OPERAND_ftsf157ae_slot1,
  OPERAND_ftsf158ae_slot1,
  OPERAND_ftsf159ae_slot1,
  OPERAND_ftsf160ae_slot1,
  OPERAND_ftsf161ae_slot1,
  OPERAND_ftsf162ae_slot1,
  OPERAND_ftsf163ae_slot1,
  OPERAND_ftsf164ae_slot1,
  OPERAND_ftsf165ae_slot1,
  OPERAND_ftsf166ae_slot1,
  OPERAND_ftsf167ae_slot1,
  OPERAND_ftsf168ae_slot1,
  OPERAND_ftsf169ae_slot1,
  OPERAND_ftsf170ae_slot1,
  OPERAND_ftsf171ae_slot1,
  OPERAND_ftsf172ae_slot1,
  OPERAND_ftsf173ae_slot1,
  OPERAND_ftsf174ae_slot1,
  OPERAND_ftsf175ae_slot1,
  OPERAND_ftsf176ae_slot1,
  OPERAND_ftsf177ae_slot1,
  OPERAND_ftsf178ae_slot1,
  OPERAND_ftsf179ae_slot1,
  OPERAND_ftsf180ae_slot1,
  OPERAND_ftsf181ae_slot1,
  OPERAND_ftsf182ae_slot1,
  OPERAND_ftsf183ae_slot1,
  OPERAND_ftsf184ae_slot1,
  OPERAND_ftsf185ae_slot1,
  OPERAND_ftsf186ae_slot1,
  OPERAND_ftsf187ae_slot1,
  OPERAND_ftsf188ae_slot1,
  OPERAND_ftsf189ae_slot1,
  OPERAND_ftsf190ae_slot1,
  OPERAND_ftsf191ae_slot1,
  OPERAND_ftsf192ae_slot1,
  OPERAND_ftsf193ae_slot1,
  OPERAND_ftsf194ae_slot1,
  OPERAND_ftsf195ae_slot1,
  OPERAND_ftsf196ae_slot1,
  OPERAND_ftsf197ae_slot1,
  OPERAND_ftsf198ae_slot1,
  OPERAND_ftsf199ae_slot1,
  OPERAND_ftsf200ae_slot1,
  OPERAND_ftsf201ae_slot1,
  OPERAND_ftsf202ae_slot1,
  OPERAND_ftsf203ae_slot1,
  OPERAND_ftsf204ae_slot1,
  OPERAND_ftsf205ae_slot1,
  OPERAND_ftsf206ae_slot1,
  OPERAND_ftsf207ae_slot1,
  OPERAND_ftsf208,
  OPERAND_ftsf209ae_slot1,
  OPERAND_ftsf210ae_slot1,
  OPERAND_ftsf211ae_slot1,
  OPERAND_ftsf330ae_slot1,
  OPERAND_ftsf332ae_slot1,
  OPERAND_ftsf334ae_slot1,
  OPERAND_ftsf336ae_slot1,
  OPERAND_ftsf337ae_slot1,
  OPERAND_ftsf338,
  OPERAND_ftsf339ae_slot1,
  OPERAND_ftsf340,
  OPERAND_ftsf341ae_slot1,
  OPERAND_ftsf342ae_slot1,
  OPERAND_ftsf343ae_slot1,
  OPERAND_ftsf344ae_slot1,
  OPERAND_ftsf346ae_slot1,
  OPERAND_ftsf347,
  OPERAND_ftsf348ae_slot1,
  OPERAND_ftsf349ae_slot1,
  OPERAND_ftsf350ae_slot1,
  OPERAND_op0_s4,
  OPERAND_ftsf212ae_slot0,
  OPERAND_ftsf213ae_slot0,
  OPERAND_ftsf214ae_slot0,
  OPERAND_ftsf215ae_slot0,
  OPERAND_ftsf216ae_slot0,
  OPERAND_ftsf217,
  OPERAND_ftsf218ae_slot0,
  OPERAND_ftsf219ae_slot0,
  OPERAND_ftsf220ae_slot0,
  OPERAND_ftsf221ae_slot0,
  OPERAND_ftsf222ae_slot0,
  OPERAND_ftsf223ae_slot0,
  OPERAND_ftsf224ae_slot0,
  OPERAND_ftsf225ae_slot0,
  OPERAND_ftsf226ae_slot0,
  OPERAND_ftsf227ae_slot0,
  OPERAND_ftsf228ae_slot0,
  OPERAND_ftsf229ae_slot0,
  OPERAND_ftsf230ae_slot0,
  OPERAND_ftsf231ae_slot0,
  OPERAND_ftsf232ae_slot0,
  OPERAND_ftsf233ae_slot0,
  OPERAND_ftsf234ae_slot0,
  OPERAND_ftsf235ae_slot0,
  OPERAND_ftsf236ae_slot0,
  OPERAND_ftsf237ae_slot0,
  OPERAND_ftsf238ae_slot0,
  OPERAND_ftsf239ae_slot0,
  OPERAND_ftsf240ae_slot0,
  OPERAND_ftsf241ae_slot0,
  OPERAND_ftsf242ae_slot0,
  OPERAND_ftsf243ae_slot0,
  OPERAND_ftsf244ae_slot0,
  OPERAND_ftsf245ae_slot0,
  OPERAND_ftsf246ae_slot0,
  OPERAND_ftsf247ae_slot0,
  OPERAND_ftsf248ae_slot0,
  OPERAND_ftsf249ae_slot0,
  OPERAND_ftsf250ae_slot0,
  OPERAND_ftsf251ae_slot0,
  OPERAND_ftsf252ae_slot0,
  OPERAND_ftsf253ae_slot0,
  OPERAND_ftsf254ae_slot0,
  OPERAND_ftsf255ae_slot0,
  OPERAND_ftsf256ae_slot0,
  OPERAND_ftsf257ae_slot0,
  OPERAND_ftsf258ae_slot0,
  OPERAND_ftsf259ae_slot0,
  OPERAND_ftsf260ae_slot0,
  OPERAND_ftsf261ae_slot0,
  OPERAND_ftsf262ae_slot0,
  OPERAND_ftsf263ae_slot0,
  OPERAND_ftsf264ae_slot0,
  OPERAND_ftsf265ae_slot0,
  OPERAND_ftsf266ae_slot0,
  OPERAND_ftsf267ae_slot0,
  OPERAND_ftsf268ae_slot0,
  OPERAND_ftsf269ae_slot0,
  OPERAND_ftsf270ae_slot0,
  OPERAND_ftsf271ae_slot0,
  OPERAND_ftsf272ae_slot0,
  OPERAND_ftsf273ae_slot0,
  OPERAND_ftsf274ae_slot0,
  OPERAND_ftsf275ae_slot0,
  OPERAND_ftsf276ae_slot0,
  OPERAND_ftsf277ae_slot0,
  OPERAND_ftsf278ae_slot0,
  OPERAND_ftsf279ae_slot0,
  OPERAND_ftsf281ae_slot0,
  OPERAND_ftsf282ae_slot0,
  OPERAND_ftsf283ae_slot0,
  OPERAND_ftsf284ae_slot0,
  OPERAND_ftsf286ae_slot0,
  OPERAND_ftsf288ae_slot0,
  OPERAND_ftsf290ae_slot0,
  OPERAND_ftsf292ae_slot0,
  OPERAND_ftsf293,
  OPERAND_ftsf294ae_slot0,
  OPERAND_ftsf295ae_slot0,
  OPERAND_ftsf296ae_slot0,
  OPERAND_ftsf297ae_slot0,
  OPERAND_ftsf298ae_slot0,
  OPERAND_ftsf299ae_slot0,
  OPERAND_ftsf300ae_slot0,
  OPERAND_ftsf301ae_slot0,
  OPERAND_ftsf302ae_slot0,
  OPERAND_ftsf303ae_slot0,
  OPERAND_ftsf304ae_slot0,
  OPERAND_ftsf306ae_slot0,
  OPERAND_ftsf308ae_slot0,
  OPERAND_ftsf309ae_slot0,
  OPERAND_ftsf310ae_slot0,
  OPERAND_ftsf311ae_slot0,
  OPERAND_ftsf312ae_slot0,
  OPERAND_ftsf313ae_slot0,
  OPERAND_ftsf314ae_slot0,
  OPERAND_ftsf315ae_slot0,
  OPERAND_ftsf316ae_slot0,
  OPERAND_ftsf317ae_slot0,
  OPERAND_ftsf318ae_slot0,
  OPERAND_ftsf319,
  OPERAND_ftsf320ae_slot0,
  OPERAND_ftsf321,
  OPERAND_ftsf322ae_slot0,
  OPERAND_ftsf323ae_slot0,
  OPERAND_ftsf324ae_slot0,
  OPERAND_ftsf325ae_slot0,
  OPERAND_ftsf326ae_slot0,
  OPERAND_ftsf328ae_slot0,
  OPERAND_ftsf329ae_slot0,
  OPERAND_ftsf352ae_slot0,
  OPERAND_ftsf353,
  OPERAND_ftsf354ae_slot0,
  OPERAND_ftsf356ae_slot0,
  OPERAND_ftsf357,
  OPERAND_ftsf358ae_slot0,
  OPERAND_ftsf359ae_slot0,
  OPERAND_ftsf360ae_slot0,
  OPERAND_ftsf361ae_slot0,
  OPERAND_ftsf362ae_slot0,
  OPERAND_ftsf364ae_slot0,
  OPERAND_ftsf365ae_slot0,
  OPERAND_ftsf366ae_slot0,
  OPERAND_ftsf368ae_slot0,
  OPERAND_ftsf369ae_slot0
};


/* Iclass table.  */

static xtensa_arg_internal Iclass_xt_iclass_rfe_stateArgs[] = {
  { { STATE_PSRING }, 'i' },
  { { STATE_PSEXCM }, 'm' },
  { { STATE_EPC1 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rfde_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_DEPC }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_call12_args[] = {
  { { OPERAND_soffsetx4 }, 'i' },
  { { OPERAND_ar12 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_call12_stateArgs[] = {
  { { STATE_PSCALLINC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_call8_args[] = {
  { { OPERAND_soffsetx4 }, 'i' },
  { { OPERAND_ar8 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_call8_stateArgs[] = {
  { { STATE_PSCALLINC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_call4_args[] = {
  { { OPERAND_soffsetx4 }, 'i' },
  { { OPERAND_ar4 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_call4_stateArgs[] = {
  { { STATE_PSCALLINC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_callx12_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ar12 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_callx12_stateArgs[] = {
  { { STATE_PSCALLINC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_callx8_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ar8 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_callx8_stateArgs[] = {
  { { STATE_PSCALLINC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_callx4_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ar4 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_callx4_stateArgs[] = {
  { { STATE_PSCALLINC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_entry_args[] = {
  { { OPERAND_ars_entry }, 's' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm12x8 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_entry_stateArgs[] = {
  { { STATE_PSCALLINC }, 'i' },
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSWOE }, 'i' },
  { { STATE_WindowBase }, 'm' },
  { { STATE_WindowStart }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_movsp_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_movsp_stateArgs[] = {
  { { STATE_WindowBase }, 'i' },
  { { STATE_WindowStart }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rotw_args[] = {
  { { OPERAND_simm4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rotw_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_WindowBase }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_retw_args[] = {
  { { OPERAND__ars_invisible }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_retw_stateArgs[] = {
  { { STATE_WindowBase }, 'm' },
  { { STATE_WindowStart }, 'm' },
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSWOE }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rfwou_stateArgs[] = {
  { { STATE_EPC1 }, 'i' },
  { { STATE_PSEXCM }, 'm' },
  { { STATE_PSRING }, 'i' },
  { { STATE_WindowBase }, 'm' },
  { { STATE_WindowStart }, 'm' },
  { { STATE_PSOWB }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_l32e_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_immrx4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_l32e_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_s32e_args[] = {
  { { OPERAND_art }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_immrx4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_s32e_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_windowbase_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_windowbase_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_WindowBase }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_windowbase_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_windowbase_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_WindowBase }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_windowbase_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_windowbase_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_WindowBase }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_windowstart_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_windowstart_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_WindowStart }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_windowstart_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_windowstart_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_WindowStart }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_windowstart_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_windowstart_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_WindowStart }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_add_n_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_addi_n_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ai4const }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_bz6_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm6 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_loadi4_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_lsi4x4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_mov_n_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_movi_n_args[] = {
  { { OPERAND_ars }, 'o' },
  { { OPERAND_simm7 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_retn_args[] = {
  { { OPERAND__ars_invisible }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_storei4_args[] = {
  { { OPERAND_art }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_lsi4x4 }, 'i' }
};

static xtensa_arg_internal Iclass_rur_threadptr_args[] = {
  { { OPERAND_arr }, 'o' }
};

static xtensa_arg_internal Iclass_rur_threadptr_stateArgs[] = {
  { { STATE_THREADPTR }, 'i' }
};

static xtensa_arg_internal Iclass_wur_threadptr_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_wur_threadptr_stateArgs[] = {
  { { STATE_THREADPTR }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_addi_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_simm8 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_addmi_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_simm8x256 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_addsub_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_bit_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_bsi8_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_b4const }, 'i' },
  { { OPERAND_label8 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_bsi8b_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_bbi }, 'i' },
  { { OPERAND_label8 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_bsi8u_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_b4constu }, 'i' },
  { { OPERAND_label8 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_bst8_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' },
  { { OPERAND_label8 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_bsz12_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_label12 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_call0_args[] = {
  { { OPERAND_soffsetx4 }, 'i' },
  { { OPERAND_ar0 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_callx0_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ar0 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_exti_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_art }, 'i' },
  { { OPERAND_sae }, 'i' },
  { { OPERAND_op2p1 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_jump_args[] = {
  { { OPERAND_soffset }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_jumpx_args[] = {
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_l16ui_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8x2 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_l16si_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8x2 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_l32i_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8x4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_l32r_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_uimm16x4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_l32r_stateArgs[] = {
  { { STATE_LITBADDR }, 'i' },
  { { STATE_LITBEN }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_l8i_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_loop_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ulabel8 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_loop_stateArgs[] = {
  { { STATE_LBEG }, 'o' },
  { { STATE_LEND }, 'o' },
  { { STATE_LCOUNT }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_loopz_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ulabel8 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_loopz_stateArgs[] = {
  { { STATE_LBEG }, 'o' },
  { { STATE_LEND }, 'o' },
  { { STATE_LCOUNT }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_movi_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_simm12b }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_movz_args[] = {
  { { OPERAND_arr }, 'm' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_neg_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_return_args[] = {
  { { OPERAND__ars_invisible }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_s16i_args[] = {
  { { OPERAND_art }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8x2 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_s32i_args[] = {
  { { OPERAND_art }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8x4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_s8i_args[] = {
  { { OPERAND_art }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_sar_args[] = {
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_sar_stateArgs[] = {
  { { STATE_SAR }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_sari_args[] = {
  { { OPERAND_sas }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_sari_stateArgs[] = {
  { { STATE_SAR }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_shifts_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_shifts_stateArgs[] = {
  { { STATE_SAR }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_shiftst_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_shiftst_stateArgs[] = {
  { { STATE_SAR }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_shiftt_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_shiftt_stateArgs[] = {
  { { STATE_SAR }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_slli_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_msalp32 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_srai_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_art }, 'i' },
  { { OPERAND_sargt }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_srli_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_art }, 'i' },
  { { OPERAND_s }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_sync_stateArgs[] = {
  { { STATE_XTSYNC }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsil_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_s }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsil_stateArgs[] = {
  { { STATE_PSWOE }, 'i' },
  { { STATE_PSCALLINC }, 'i' },
  { { STATE_PSOWB }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_PSUM }, 'i' },
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSINTLEVEL }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_lend_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_lend_stateArgs[] = {
  { { STATE_LEND }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_lend_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_lend_stateArgs[] = {
  { { STATE_LEND }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_lend_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_lend_stateArgs[] = {
  { { STATE_LEND }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_lcount_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_lcount_stateArgs[] = {
  { { STATE_LCOUNT }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_lcount_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_lcount_stateArgs[] = {
  { { STATE_XTSYNC }, 'o' },
  { { STATE_LCOUNT }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_lcount_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_lcount_stateArgs[] = {
  { { STATE_XTSYNC }, 'o' },
  { { STATE_LCOUNT }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_lbeg_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_lbeg_stateArgs[] = {
  { { STATE_LBEG }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_lbeg_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_lbeg_stateArgs[] = {
  { { STATE_LBEG }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_lbeg_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_lbeg_stateArgs[] = {
  { { STATE_LBEG }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_sar_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_sar_stateArgs[] = {
  { { STATE_SAR }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_sar_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_sar_stateArgs[] = {
  { { STATE_SAR }, 'o' },
  { { STATE_XTSYNC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_sar_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_sar_stateArgs[] = {
  { { STATE_SAR }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_litbase_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_litbase_stateArgs[] = {
  { { STATE_LITBADDR }, 'i' },
  { { STATE_LITBEN }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_litbase_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_litbase_stateArgs[] = {
  { { STATE_LITBADDR }, 'o' },
  { { STATE_LITBEN }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_litbase_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_litbase_stateArgs[] = {
  { { STATE_LITBADDR }, 'm' },
  { { STATE_LITBEN }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_176_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_176_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_176_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_176_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_208_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_208_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_ps_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_ps_stateArgs[] = {
  { { STATE_PSWOE }, 'i' },
  { { STATE_PSCALLINC }, 'i' },
  { { STATE_PSOWB }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_PSUM }, 'i' },
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSINTLEVEL }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_ps_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_ps_stateArgs[] = {
  { { STATE_PSWOE }, 'o' },
  { { STATE_PSCALLINC }, 'o' },
  { { STATE_PSOWB }, 'o' },
  { { STATE_PSRING }, 'm' },
  { { STATE_PSUM }, 'o' },
  { { STATE_PSEXCM }, 'm' },
  { { STATE_PSINTLEVEL }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_ps_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_ps_stateArgs[] = {
  { { STATE_PSWOE }, 'm' },
  { { STATE_PSCALLINC }, 'm' },
  { { STATE_PSOWB }, 'm' },
  { { STATE_PSRING }, 'm' },
  { { STATE_PSUM }, 'm' },
  { { STATE_PSEXCM }, 'm' },
  { { STATE_PSINTLEVEL }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_epc1_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_epc1_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EPC1 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_epc1_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_epc1_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EPC1 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_epc1_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_epc1_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EPC1 }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_excsave1_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_excsave1_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EXCSAVE1 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_excsave1_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_excsave1_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EXCSAVE1 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_excsave1_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_excsave1_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EXCSAVE1 }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_epc2_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_epc2_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EPC2 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_epc2_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_epc2_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EPC2 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_epc2_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_epc2_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EPC2 }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_excsave2_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_excsave2_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EXCSAVE2 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_excsave2_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_excsave2_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EXCSAVE2 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_excsave2_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_excsave2_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EXCSAVE2 }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_eps2_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_eps2_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EPS2 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_eps2_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_eps2_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EPS2 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_eps2_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_eps2_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EPS2 }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_excvaddr_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_excvaddr_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EXCVADDR }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_excvaddr_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_excvaddr_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EXCVADDR }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_excvaddr_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_excvaddr_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EXCVADDR }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_depc_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_depc_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_DEPC }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_depc_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_depc_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_DEPC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_depc_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_depc_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_DEPC }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_exccause_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_exccause_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EXCCAUSE }, 'i' },
  { { STATE_XTSYNC }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_exccause_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_exccause_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EXCCAUSE }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_exccause_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_exccause_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_EXCCAUSE }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_misc0_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_misc0_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_MISC0 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_misc0_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_misc0_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_MISC0 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_misc0_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_misc0_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_MISC0 }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_misc1_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_misc1_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_MISC1 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_misc1_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_misc1_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_MISC1 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_misc1_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_misc1_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_MISC1 }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_prid_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_prid_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_vecbase_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_vecbase_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_VECBASE }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_vecbase_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_vecbase_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_VECBASE }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_vecbase_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_vecbase_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_VECBASE }, 'm' }
};

static xtensa_arg_internal Iclass_xt_mul16_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_mul32_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rfi_args[] = {
  { { OPERAND_s }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rfi_stateArgs[] = {
  { { STATE_PSWOE }, 'o' },
  { { STATE_PSCALLINC }, 'o' },
  { { STATE_PSOWB }, 'o' },
  { { STATE_PSRING }, 'm' },
  { { STATE_PSUM }, 'o' },
  { { STATE_PSEXCM }, 'm' },
  { { STATE_PSINTLEVEL }, 'o' },
  { { STATE_EPC1 }, 'i' },
  { { STATE_EPC2 }, 'i' },
  { { STATE_EPS2 }, 'i' },
  { { STATE_InOCDMode }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_wait_args[] = {
  { { OPERAND_s }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wait_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_PSINTLEVEL }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_interrupt_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_interrupt_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_INTERRUPT }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_intset_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_intset_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_XTSYNC }, 'o' },
  { { STATE_INTERRUPT }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_intclear_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_intclear_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_XTSYNC }, 'o' },
  { { STATE_INTERRUPT }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_intenable_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_intenable_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_INTENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_intenable_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_intenable_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_INTENABLE }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_intenable_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_intenable_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_INTENABLE }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_break_args[] = {
  { { OPERAND_imms }, 'i' },
  { { OPERAND_immt }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_break_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSINTLEVEL }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_break_n_args[] = {
  { { OPERAND_imms }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_break_n_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSINTLEVEL }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_debugcause_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_debugcause_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_DEBUGCAUSE }, 'i' },
  { { STATE_DBNUM }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_debugcause_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_debugcause_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_DEBUGCAUSE }, 'o' },
  { { STATE_DBNUM }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_debugcause_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_debugcause_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_DEBUGCAUSE }, 'm' },
  { { STATE_DBNUM }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_icount_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_icount_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_ICOUNT }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_icount_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_icount_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_XTSYNC }, 'o' },
  { { STATE_ICOUNT }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_icount_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_icount_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_XTSYNC }, 'o' },
  { { STATE_ICOUNT }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_icountlevel_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_icountlevel_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_ICOUNTLEVEL }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_icountlevel_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_icountlevel_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_ICOUNTLEVEL }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_icountlevel_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_icountlevel_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_ICOUNTLEVEL }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_ddr_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_ddr_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_DDR }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_ddr_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_ddr_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_XTSYNC }, 'o' },
  { { STATE_DDR }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_ddr_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_ddr_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_XTSYNC }, 'o' },
  { { STATE_DDR }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rfdo_args[] = {
  { { OPERAND_imms }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rfdo_stateArgs[] = {
  { { STATE_InOCDMode }, 'm' },
  { { STATE_EPC2 }, 'i' },
  { { STATE_PSWOE }, 'o' },
  { { STATE_PSCALLINC }, 'o' },
  { { STATE_PSOWB }, 'o' },
  { { STATE_PSRING }, 'o' },
  { { STATE_PSUM }, 'o' },
  { { STATE_PSEXCM }, 'o' },
  { { STATE_PSINTLEVEL }, 'o' },
  { { STATE_EPS2 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rfdd_stateArgs[] = {
  { { STATE_InOCDMode }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_bbool1_args[] = {
  { { OPERAND_br }, 'o' },
  { { OPERAND_bs }, 'i' },
  { { OPERAND_bt }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_bbool4_args[] = {
  { { OPERAND_bt }, 'o' },
  { { OPERAND_bs4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_bbool8_args[] = {
  { { OPERAND_bt }, 'o' },
  { { OPERAND_bs8 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_bbranch_args[] = {
  { { OPERAND_bs }, 'i' },
  { { OPERAND_label8 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_bmove_args[] = {
  { { OPERAND_arr }, 'm' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_bt }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_RSR_BR_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_brall }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_WSR_BR_args[] = {
  { { OPERAND_art }, 'i' },
  { { OPERAND_brall }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_XSR_BR_args[] = {
  { { OPERAND_art }, 'm' },
  { { OPERAND_brall }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_ccount_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_ccount_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_CCOUNT }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_ccount_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_ccount_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_XTSYNC }, 'o' },
  { { STATE_CCOUNT }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_ccount_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_ccount_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_XTSYNC }, 'o' },
  { { STATE_CCOUNT }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_ccompare0_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_ccompare0_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_CCOMPARE0 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_ccompare0_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_ccompare0_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_CCOMPARE0 }, 'o' },
  { { STATE_INTERRUPT }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_ccompare0_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_ccompare0_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_CCOMPARE0 }, 'm' },
  { { STATE_INTERRUPT }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_ccompare1_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_ccompare1_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_CCOMPARE1 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_ccompare1_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_ccompare1_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_CCOMPARE1 }, 'o' },
  { { STATE_INTERRUPT }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_ccompare1_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_ccompare1_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_CCOMPARE1 }, 'm' },
  { { STATE_INTERRUPT }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_icache_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8x4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_icache_inv_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8x4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_icache_inv_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_licx_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_licx_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_sicx_args[] = {
  { { OPERAND_art }, 'i' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_sicx_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_dcache_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8x4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_dcache_ind_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm4x16 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_dcache_ind_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_dcache_inv_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8x4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_dcache_inv_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_dpf_args[] = {
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8x4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_sdct_args[] = {
  { { OPERAND_art }, 'i' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_sdct_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_ldct_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_ldct_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_ptevaddr_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_ptevaddr_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_PTBASE }, 'o' },
  { { STATE_XTSYNC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_ptevaddr_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_ptevaddr_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_PTBASE }, 'i' },
  { { STATE_EXCVADDR }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_ptevaddr_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_ptevaddr_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_PTBASE }, 'm' },
  { { STATE_EXCVADDR }, 'i' },
  { { STATE_XTSYNC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_rasid_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_rasid_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_ASID3 }, 'i' },
  { { STATE_ASID2 }, 'i' },
  { { STATE_ASID1 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_rasid_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_rasid_stateArgs[] = {
  { { STATE_XTSYNC }, 'o' },
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_ASID3 }, 'o' },
  { { STATE_ASID2 }, 'o' },
  { { STATE_ASID1 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_rasid_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_rasid_stateArgs[] = {
  { { STATE_XTSYNC }, 'o' },
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_ASID3 }, 'm' },
  { { STATE_ASID2 }, 'm' },
  { { STATE_ASID1 }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_itlbcfg_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_itlbcfg_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_INSTPGSZID4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_itlbcfg_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_itlbcfg_stateArgs[] = {
  { { STATE_XTSYNC }, 'o' },
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_INSTPGSZID4 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_itlbcfg_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_itlbcfg_stateArgs[] = {
  { { STATE_XTSYNC }, 'o' },
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_INSTPGSZID4 }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_dtlbcfg_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_dtlbcfg_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_DATAPGSZID4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_dtlbcfg_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_dtlbcfg_stateArgs[] = {
  { { STATE_XTSYNC }, 'o' },
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_DATAPGSZID4 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_dtlbcfg_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_dtlbcfg_stateArgs[] = {
  { { STATE_XTSYNC }, 'o' },
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_DATAPGSZID4 }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_idtlb_args[] = {
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_idtlb_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_XTSYNC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rdtlb_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rdtlb_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wdtlb_args[] = {
  { { OPERAND_art }, 'i' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wdtlb_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_XTSYNC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_iitlb_args[] = {
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_iitlb_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_ritlb_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_ritlb_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_witlb_args[] = {
  { { OPERAND_art }, 'i' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_witlb_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_ldpte_stateArgs[] = {
  { { STATE_PTBASE }, 'i' },
  { { STATE_EXCVADDR }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_hwwitlba_stateArgs[] = {
  { { STATE_EXCVADDR }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_hwwdtlba_stateArgs[] = {
  { { STATE_EXCVADDR }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_cpenable_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_cpenable_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_cpenable_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_cpenable_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_CPENABLE }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_cpenable_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_cpenable_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_CPENABLE }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_clamp_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_tp7 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_minmax_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_nsa_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_sx_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_tp7 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_l32ai_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8x4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_s32ri_args[] = {
  { { OPERAND_art }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8x4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_s32c1i_args[] = {
  { { OPERAND_art }, 'm' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_uimm8x4 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_s32c1i_stateArgs[] = {
  { { STATE_SCOMPARE1 }, 'i' },
  { { STATE_XTSYNC }, 'i' },
  { { STATE_SCOMPARE1 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_scompare1_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_scompare1_stateArgs[] = {
  { { STATE_SCOMPARE1 }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_scompare1_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_scompare1_stateArgs[] = {
  { { STATE_SCOMPARE1 }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_scompare1_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_scompare1_stateArgs[] = {
  { { STATE_SCOMPARE1 }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_atomctl_args[] = {
  { { OPERAND_art }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rsr_atomctl_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_ATOMCTL }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_atomctl_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wsr_atomctl_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_ATOMCTL }, 'o' },
  { { STATE_XTSYNC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_atomctl_args[] = {
  { { OPERAND_art }, 'm' }
};

static xtensa_arg_internal Iclass_xt_iclass_xsr_atomctl_stateArgs[] = {
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_ATOMCTL }, 'm' },
  { { STATE_XTSYNC }, 'o' }
};

static xtensa_arg_internal Iclass_xt_iclass_rer_args[] = {
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_rer_stateArgs[] = {
  { { STATE_CCON }, 'i' },
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_MPSCORE }, 'i' }
};

static xtensa_interface Iclass_xt_iclass_rer_intfArgs[] = {
  INTERFACE_RMPINT_Out,
  INTERFACE_RMPINT_In
};

static xtensa_arg_internal Iclass_xt_iclass_wer_args[] = {
  { { OPERAND_art }, 'i' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_xt_iclass_wer_stateArgs[] = {
  { { STATE_CCON }, 'm' },
  { { STATE_PSEXCM }, 'i' },
  { { STATE_PSRING }, 'i' },
  { { STATE_WMPINT_DATA }, 'o' },
  { { STATE_WMPINT_ADDR }, 'o' },
  { { STATE_MPSCORE }, 'm' },
  { { STATE_WMPINT_TOGGLEEN }, 'm' }
};

static xtensa_arg_internal Iclass_rur_ae_ovf_sar_args[] = {
  { { OPERAND_arr }, 'o' }
};

static xtensa_arg_internal Iclass_rur_ae_ovf_sar_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'i' },
  { { STATE_AE_SAR }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_wur_ae_ovf_sar_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_wur_ae_ovf_sar_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'o' },
  { { STATE_AE_SAR }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_rur_ae_bithead_args[] = {
  { { OPERAND_arr }, 'o' }
};

static xtensa_arg_internal Iclass_rur_ae_bithead_stateArgs[] = {
  { { STATE_AE_BITHEAD }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_wur_ae_bithead_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_wur_ae_bithead_stateArgs[] = {
  { { STATE_AE_BITHEAD }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_rur_ae_ts_fts_bu_bp_args[] = {
  { { OPERAND_arr }, 'o' }
};

static xtensa_arg_internal Iclass_rur_ae_ts_fts_bu_bp_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'i' },
  { { STATE_AE_BITSUSED }, 'i' },
  { { STATE_AE_TABLESIZE }, 'i' },
  { { STATE_AE_FIRST_TS }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_wur_ae_ts_fts_bu_bp_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_wur_ae_ts_fts_bu_bp_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'o' },
  { { STATE_AE_BITSUSED }, 'o' },
  { { STATE_AE_TABLESIZE }, 'o' },
  { { STATE_AE_FIRST_TS }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_rur_ae_sd_no_args[] = {
  { { OPERAND_arr }, 'o' }
};

static xtensa_arg_internal Iclass_rur_ae_sd_no_stateArgs[] = {
  { { STATE_AE_NEXTOFFSET }, 'i' },
  { { STATE_AE_SEARCHDONE }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_wur_ae_sd_no_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_wur_ae_sd_no_stateArgs[] = {
  { { STATE_AE_NEXTOFFSET }, 'o' },
  { { STATE_AE_SEARCHDONE }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_overflow_args[] = {
  { { OPERAND_arr }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_overflow_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_overflow_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_overflow_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_sar_args[] = {
  { { OPERAND_arr }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_sar_stateArgs[] = {
  { { STATE_AE_SAR }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_sar_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_sar_stateArgs[] = {
  { { STATE_AE_SAR }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_bitptr_args[] = {
  { { OPERAND_arr }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_bitptr_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_bitptr_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_bitptr_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_bitsused_args[] = {
  { { OPERAND_arr }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_bitsused_stateArgs[] = {
  { { STATE_AE_BITSUSED }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_bitsused_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_bitsused_stateArgs[] = {
  { { STATE_AE_BITSUSED }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_tablesize_args[] = {
  { { OPERAND_arr }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_tablesize_stateArgs[] = {
  { { STATE_AE_TABLESIZE }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_tablesize_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_tablesize_stateArgs[] = {
  { { STATE_AE_TABLESIZE }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_first_ts_args[] = {
  { { OPERAND_arr }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_first_ts_stateArgs[] = {
  { { STATE_AE_FIRST_TS }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_first_ts_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_first_ts_stateArgs[] = {
  { { STATE_AE_FIRST_TS }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_nextoffset_args[] = {
  { { OPERAND_arr }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_nextoffset_stateArgs[] = {
  { { STATE_AE_NEXTOFFSET }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_nextoffset_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_nextoffset_stateArgs[] = {
  { { STATE_AE_NEXTOFFSET }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_searchdone_args[] = {
  { { OPERAND_arr }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_rur_ae_searchdone_stateArgs[] = {
  { { STATE_AE_SEARCHDONE }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_searchdone_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_wur_ae_searchdone_stateArgs[] = {
  { { STATE_AE_SEARCHDONE }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16f_i_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm16 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16f_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16f_iu_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm16 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16f_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16f_x_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16f_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16f_xu_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16f_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24_i_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24_iu_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24_x_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24_xu_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24f_i_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24f_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24f_iu_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24f_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24f_x_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24f_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24f_xu_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24f_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16x2f_i_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16x2f_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16x2f_iu_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16x2f_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16x2f_x_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16x2f_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16x2f_xu_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp16x2f_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2f_i_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2f_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2f_iu_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2f_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2f_x_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2f_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2f_xu_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2f_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2_i_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2_iu_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2_x_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2_xu_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lp24x2_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16x2f_i_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16x2f_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16x2f_iu_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16x2f_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16x2f_x_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16x2f_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16x2f_xu_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16x2f_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2s_i_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2s_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2s_iu_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2s_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2s_x_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2s_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2s_xu_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2s_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2f_i_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2f_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2f_iu_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2f_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2f_x_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2f_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2f_xu_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24x2f_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16f_l_i_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm16 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16f_l_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16f_l_iu_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm16 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16f_l_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16f_l_x_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16f_l_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16f_l_xu_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp16f_l_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24s_l_i_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24s_l_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24s_l_iu_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24s_l_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24s_l_x_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24s_l_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24s_l_xu_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24s_l_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24f_l_i_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24f_l_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24f_l_iu_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24f_l_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24f_l_x_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24f_l_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24f_l_xu_args[] = {
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sp24f_l_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq56_i_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq56_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq56_iu_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq56_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq56_x_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq56_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq56_xu_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq56_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq32f_i_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq32f_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq32f_iu_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq32f_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq32f_x_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq32f_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq32f_xu_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lq32f_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq56s_i_args[] = {
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq56s_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq56s_iu_args[] = {
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq56s_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq56s_x_args[] = {
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq56s_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq56s_xu_args[] = {
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq56s_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq32f_i_args[] = {
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq32f_i_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq32f_iu_args[] = {
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_lsimm32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq32f_iu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq32f_x_args[] = {
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq32f_x_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq32f_xu_args[] = {
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sq32f_xu_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_zerop48_args[] = {
  { { OPERAND_ps }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_zerop48_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movp48_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movp48_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_selp24_ll_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_selp24_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_selp24_lh_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_selp24_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_selp24_hl_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_selp24_hl_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_selp24_hh_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_selp24_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movtp24x2_args[] = {
  { { OPERAND_pr }, 'm' },
  { { OPERAND_pr0 }, 'i' },
  { { OPERAND_bt2 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movtp24x2_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movfp24x2_args[] = {
  { { OPERAND_pr }, 'm' },
  { { OPERAND_pr0 }, 'i' },
  { { OPERAND_bt2 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movfp24x2_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movtp48_args[] = {
  { { OPERAND_pr }, 'm' },
  { { OPERAND_pr0 }, 'i' },
  { { OPERAND_bt }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movtp48_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movfp48_args[] = {
  { { OPERAND_pr }, 'm' },
  { { OPERAND_pr0 }, 'i' },
  { { OPERAND_bt }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movfp48_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movpa24x2_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movpa24x2_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_truncp24a32x2_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_truncp24a32x2_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvta32p24_l_args[] = {
  { { OPERAND_ars }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvta32p24_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvta32p24_h_args[] = {
  { { OPERAND_ars }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvta32p24_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtp24a16x2_ll_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtp24a16x2_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtp24a16x2_lh_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtp24a16x2_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtp24a16x2_hl_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtp24a16x2_hl_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtp24a16x2_hh_args[] = {
  { { OPERAND_pr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtp24a16x2_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_truncp24q48x2_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_qr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_truncp24q48x2_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_truncp16_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_truncp16_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsp24q48sym_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsp24q48sym_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsp24q48asym_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsp24q48asym_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsp16q48sym_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsp16q48sym_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsp16q48asym_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsp16q48asym_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsp16sym_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsp16sym_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsp16asym_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsp16asym_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_zeroq56_args[] = {
  { { OPERAND_qr1_w }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_zeroq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movtq56_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_bs }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movtq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movfq56_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_bs }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movfq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtq48a32s_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtq48a32s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtq48p24s_l_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtq48p24s_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtq48p24s_h_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_cvtq48p24s_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_satq48s_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_satq48s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_truncq32_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_truncq32_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsq32sym_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsq32sym_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsq32asym_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_roundsq32asym_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_trunca32q48_args[] = {
  { { OPERAND_ars }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_trunca32q48_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movap24s_l_args[] = {
  { { OPERAND_ars }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movap24s_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movap24s_h_args[] = {
  { { OPERAND_ars }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_movap24s_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_trunca16p24s_l_args[] = {
  { { OPERAND_ars }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_trunca16p24s_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_trunca16p24s_h_args[] = {
  { { OPERAND_ars }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_trunca16p24s_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_addp24_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_addp24_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_subp24_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_subp24_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_negp24_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_negp24_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_absp24_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_absp24_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_maxp24s_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_maxp24s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_minp24s_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_minp24s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_maxbp24s_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' },
  { { OPERAND_bt2 }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_maxbp24s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_minbp24s_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' },
  { { OPERAND_bt2 }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_minbp24s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_addsp24s_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_addsp24s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_subsp24s_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_subsp24s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_negsp24s_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_negsp24s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_abssp24s_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_abssp24s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_andp48_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_andp48_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_nandp48_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_nandp48_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_orp48_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_orp48_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_xorp48_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_xorp48_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_ltp24s_args[] = {
  { { OPERAND_bt2 }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_ltp24s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lep24s_args[] = {
  { { OPERAND_bt2 }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lep24s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_eqp24_args[] = {
  { { OPERAND_bt2 }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_eqp24_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_addq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_qr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_addq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_subq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_qr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_subq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_negq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_negq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_absq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_absq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_maxq56s_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_maxq56s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_minq56s_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_minq56s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_maxbq56s_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_bt }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_maxbq56s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_minbq56s_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_bt }, 'o' }
};

static xtensa_arg_internal Iclass_ae_iclass_minbq56s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_addsq56s_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_qr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_addsq56s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_subsq56s_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_qr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_subsq56s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_negsq56s_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_negsq56s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_abssq56s_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_abssq56s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_andq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_andq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_nandq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_nandq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_orq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_orq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_xorq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_xorq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllip24_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ae_samt32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllip24_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srlip24_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ae_samt32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srlip24_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sraip24_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ae_samt32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sraip24_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllsp24_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllsp24_stateArgs[] = {
  { { STATE_AE_SAR }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srlsp24_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srlsp24_stateArgs[] = {
  { { STATE_AE_SAR }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srasp24_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srasp24_stateArgs[] = {
  { { STATE_AE_SAR }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllisp24s_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_ae_samt32 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllisp24s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllssp24s_args[] = {
  { { OPERAND_ps }, 'o' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllssp24s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_AE_SAR }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_slliq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ae_samt64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_slliq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srliq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ae_samt64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srliq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sraiq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ae_samt64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sraiq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllsq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllsq56_stateArgs[] = {
  { { STATE_AE_SAR }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srlsq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srlsq56_stateArgs[] = {
  { { STATE_AE_SAR }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srasq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srasq56_stateArgs[] = {
  { { STATE_AE_SAR }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllaq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllaq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srlaq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_srlaq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sraaq56_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sraaq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllisq56s_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ae_samt64 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllisq56s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllssq56s_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllssq56s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_AE_SAR }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllasq56s_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sllasq56s_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_ltq56s_args[] = {
  { { OPERAND_bt }, 'o' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_ltq56s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_leq56s_args[] = {
  { { OPERAND_bt }, 'o' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_leq56s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_eqq56_args[] = {
  { { OPERAND_bt }, 'o' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_eqq56_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_nsaq56s_args[] = {
  { { OPERAND_ars }, 'o' },
  { { OPERAND_qr0_rw }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_nsaq56s_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfs32p16s_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfs32p16s_ll_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfp24s_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfp24s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulp24s_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulp24s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfs32p16s_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfs32p16s_lh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfp24s_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfp24s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulp24s_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulp24s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfs32p16s_hl_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfs32p16s_hl_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfp24s_hl_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfp24s_hl_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulp24s_hl_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulp24s_hl_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfs32p16s_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfs32p16s_hh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfp24s_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfp24s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulp24s_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulp24s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs32p16s_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs32p16s_ll_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafp24s_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafp24s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulap24s_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulap24s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs32p16s_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs32p16s_lh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafp24s_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafp24s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulap24s_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulap24s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs32p16s_hl_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs32p16s_hl_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafp24s_hl_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafp24s_hl_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulap24s_hl_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulap24s_hl_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs32p16s_hh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs32p16s_hh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafp24s_hh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafp24s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulap24s_hh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulap24s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs32p16s_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs32p16s_ll_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfp24s_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfp24s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsp24s_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsp24s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs32p16s_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs32p16s_lh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfp24s_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfp24s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsp24s_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsp24s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs32p16s_hl_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs32p16s_hl_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfp24s_hl_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfp24s_hl_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsp24s_hl_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsp24s_hl_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs32p16s_hh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs32p16s_hh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfp24s_hh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfp24s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsp24s_hh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsp24s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs56p24s_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs56p24s_ll_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulas56p24s_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulas56p24s_ll_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs56p24s_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs56p24s_lh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulas56p24s_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulas56p24s_lh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs56p24s_hl_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs56p24s_hl_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulas56p24s_hl_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulas56p24s_hl_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs56p24s_hh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafs56p24s_hh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulas56p24s_hh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulas56p24s_hh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs56p24s_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs56p24s_ll_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulss56p24s_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulss56p24s_ll_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs56p24s_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs56p24s_lh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulss56p24s_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulss56p24s_lh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs56p24s_hl_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs56p24s_hl_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulss56p24s_hl_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulss56p24s_hl_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs56p24s_hh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfs56p24s_hh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulss56p24s_hh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulss56p24s_hh_stateArgs[] = {
  { { STATE_AE_OVERFLOW }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfq32sp16s_l_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfq32sp16s_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfq32sp16s_h_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfq32sp16s_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfq32sp16u_l_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfq32sp16u_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfq32sp16u_h_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulfq32sp16u_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulq32sp16s_l_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulq32sp16s_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulq32sp16s_h_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulq32sp16s_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulq32sp16u_l_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulq32sp16u_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulq32sp16u_h_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulq32sp16u_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafq32sp16s_l_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafq32sp16s_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafq32sp16s_h_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafq32sp16s_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafq32sp16u_l_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafq32sp16u_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafq32sp16u_h_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulafq32sp16u_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaq32sp16s_l_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaq32sp16s_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaq32sp16s_h_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaq32sp16s_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaq32sp16u_l_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaq32sp16u_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaq32sp16u_h_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaq32sp16u_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfq32sp16s_l_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfq32sp16s_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfq32sp16s_h_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfq32sp16s_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfq32sp16u_l_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfq32sp16u_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfq32sp16u_h_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsfq32sp16u_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsq32sp16s_l_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsq32sp16s_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsq32sp16s_h_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsq32sp16s_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsq32sp16u_l_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsq32sp16u_l_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsq32sp16u_h_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsq32sp16u_h_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaaq32sp16s_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaaq32sp16s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafq32sp16s_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafq32sp16s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaaq32sp16u_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaaq32sp16u_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafq32sp16u_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafq32sp16u_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaaq32sp16s_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaaq32sp16s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafq32sp16s_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafq32sp16s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaaq32sp16u_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaaq32sp16u_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafq32sp16u_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafq32sp16u_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaaq32sp16s_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaaq32sp16s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafq32sp16s_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafq32sp16s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaaq32sp16u_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaaq32sp16u_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafq32sp16u_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafq32sp16u_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasq32sp16s_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasq32sp16s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfq32sp16s_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfq32sp16s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasq32sp16u_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasq32sp16u_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfq32sp16u_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfq32sp16u_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasq32sp16s_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasq32sp16s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfq32sp16s_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfq32sp16s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasq32sp16u_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasq32sp16u_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfq32sp16u_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfq32sp16u_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasq32sp16s_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasq32sp16s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfq32sp16s_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfq32sp16s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasq32sp16u_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasq32sp16u_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfq32sp16u_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfq32sp16u_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsaq32sp16s_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsaq32sp16s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafq32sp16s_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafq32sp16s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsaq32sp16u_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsaq32sp16u_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafq32sp16u_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafq32sp16u_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsaq32sp16s_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsaq32sp16s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafq32sp16s_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafq32sp16s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsaq32sp16u_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsaq32sp16u_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafq32sp16u_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafq32sp16u_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsaq32sp16s_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsaq32sp16s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafq32sp16s_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafq32sp16s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsaq32sp16u_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsaq32sp16u_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafq32sp16u_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafq32sp16u_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssq32sp16s_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssq32sp16s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfq32sp16s_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfq32sp16s_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssq32sp16u_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssq32sp16u_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfq32sp16u_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfq32sp16u_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssq32sp16s_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssq32sp16s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfq32sp16s_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfq32sp16s_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssq32sp16u_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssq32sp16u_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfq32sp16u_hh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfq32sp16u_hh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssq32sp16s_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssq32sp16s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfq32sp16s_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfq32sp16s_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssq32sp16u_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssq32sp16u_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfq32sp16u_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_qr0_rw }, 'i' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_qr0 }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfq32sp16u_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafp24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafp24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaap24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaap24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafp24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaafp24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaap24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzaap24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfp24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfp24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasp24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasp24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfp24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasfp24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasp24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzasp24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafp24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafp24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsap24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsap24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafp24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsafp24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsap24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzsap24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfp24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfp24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssp24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssp24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfp24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssfp24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssp24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'o' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulzssp24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaafp24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaafp24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaap24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaap24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaafp24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaafp24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaap24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulaap24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulasfp24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulasfp24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulasp24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulasp24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulasfp24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulasfp24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulasp24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulasp24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsafp24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsafp24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsap24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsap24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsafp24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsafp24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsap24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulsap24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulssfp24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulssfp24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulssp24s_hh_ll_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulssp24s_hh_ll_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulssfp24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulssfp24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulssp24s_hl_lh_args[] = {
  { { OPERAND_qr1_w }, 'm' },
  { { OPERAND_pr }, 'i' },
  { { OPERAND_pr0 }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_mulssp24s_hl_lh_stateArgs[] = {
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sha32_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_vldl32t_args[] = {
  { { OPERAND_br }, 'o' },
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_vldl32t_stateArgs[] = {
  { { STATE_AE_TABLESIZE }, 'm' },
  { { STATE_AE_BITSUSED }, 'o' },
  { { STATE_AE_NEXTOFFSET }, 'm' },
  { { STATE_AE_SEARCHDONE }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_vldl16t_args[] = {
  { { OPERAND_br }, 'o' },
  { { OPERAND_art }, 'o' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_vldl16t_stateArgs[] = {
  { { STATE_AE_TABLESIZE }, 'm' },
  { { STATE_AE_BITSUSED }, 'o' },
  { { STATE_AE_NEXTOFFSET }, 'm' },
  { { STATE_AE_SEARCHDONE }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_vldl16c_args[] = {
  { { OPERAND_ars }, 'm' }
};

static xtensa_arg_internal Iclass_ae_iclass_vldl16c_stateArgs[] = {
  { { STATE_AE_NEXTOFFSET }, 'm' },
  { { STATE_AE_TABLESIZE }, 'm' },
  { { STATE_AE_BITPTR }, 'm' },
  { { STATE_AE_BITHEAD }, 'm' },
  { { STATE_AE_FIRST_TS }, 'i' },
  { { STATE_AE_BITSUSED }, 'i' },
  { { STATE_AE_SEARCHDONE }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_vldsht_args[] = {
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_vldsht_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'i' },
  { { STATE_AE_BITHEAD }, 'i' },
  { { STATE_AE_FIRST_TS }, 'o' },
  { { STATE_AE_NEXTOFFSET }, 'o' },
  { { STATE_AE_TABLESIZE }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lb_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lb_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'i' },
  { { STATE_AE_BITHEAD }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lbi_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ae_ohba }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lbi_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'i' },
  { { STATE_AE_BITHEAD }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lbk_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lbk_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'i' },
  { { STATE_AE_BITHEAD }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lbki_args[] = {
  { { OPERAND_arr }, 'o' },
  { { OPERAND_ars }, 'i' },
  { { OPERAND_ae_ohba }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_lbki_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'i' },
  { { STATE_AE_BITHEAD }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_db_args[] = {
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_db_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'm' },
  { { STATE_AE_BITHEAD }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_dbi_args[] = {
  { { OPERAND_ars }, 'm' },
  { { OPERAND_ae_ohba }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_dbi_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'm' },
  { { STATE_AE_BITHEAD }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_vlel32t_args[] = {
  { { OPERAND_br }, 'o' },
  { { OPERAND_art }, 'm' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_vlel32t_stateArgs[] = {
  { { STATE_AE_BITSUSED }, 'o' },
  { { STATE_AE_NEXTOFFSET }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_vlel16t_args[] = {
  { { OPERAND_br }, 'o' },
  { { OPERAND_art }, 'm' },
  { { OPERAND_ars }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_vlel16t_stateArgs[] = {
  { { STATE_AE_BITSUSED }, 'o' },
  { { STATE_AE_NEXTOFFSET }, 'o' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sb_args[] = {
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sb_stateArgs[] = {
  { { STATE_AE_BITSUSED }, 'i' },
  { { STATE_AE_BITPTR }, 'm' },
  { { STATE_AE_BITHEAD }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sbi_args[] = {
  { { OPERAND_ars }, 'm' },
  { { OPERAND_art }, 'i' },
  { { OPERAND_ae_ohba }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sbi_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'm' },
  { { STATE_AE_BITHEAD }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_vles16c_args[] = {
  { { OPERAND_ars }, 'm' }
};

static xtensa_arg_internal Iclass_ae_iclass_vles16c_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'm' },
  { { STATE_AE_BITHEAD }, 'm' },
  { { STATE_AE_BITSUSED }, 'i' },
  { { STATE_AE_NEXTOFFSET }, 'i' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_arg_internal Iclass_ae_iclass_sbf_args[] = {
  { { OPERAND_ars }, 'm' }
};

static xtensa_arg_internal Iclass_ae_iclass_sbf_stateArgs[] = {
  { { STATE_AE_BITPTR }, 'i' },
  { { STATE_AE_BITHEAD }, 'm' },
  { { STATE_CPENABLE }, 'i' }
};

static xtensa_iclass_internal iclasses[] = {
  { 0, 0 /* xt_iclass_excw */,
    0, 0, 0, 0 },
  { 0, 0 /* xt_iclass_rfe */,
    3, Iclass_xt_iclass_rfe_stateArgs, 0, 0 },
  { 0, 0 /* xt_iclass_rfde */,
    3, Iclass_xt_iclass_rfde_stateArgs, 0, 0 },
  { 0, 0 /* xt_iclass_syscall */,
    0, 0, 0, 0 },
  { 0, 0 /* xt_iclass_simcall */,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_call12_args,
    1, Iclass_xt_iclass_call12_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_call8_args,
    1, Iclass_xt_iclass_call8_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_call4_args,
    1, Iclass_xt_iclass_call4_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_callx12_args,
    1, Iclass_xt_iclass_callx12_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_callx8_args,
    1, Iclass_xt_iclass_callx8_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_callx4_args,
    1, Iclass_xt_iclass_callx4_stateArgs, 0, 0 },
  { 3, Iclass_xt_iclass_entry_args,
    5, Iclass_xt_iclass_entry_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_movsp_args,
    2, Iclass_xt_iclass_movsp_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rotw_args,
    3, Iclass_xt_iclass_rotw_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_retw_args,
    4, Iclass_xt_iclass_retw_stateArgs, 0, 0 },
  { 0, 0 /* xt_iclass_rfwou */,
    6, Iclass_xt_iclass_rfwou_stateArgs, 0, 0 },
  { 3, Iclass_xt_iclass_l32e_args,
    2, Iclass_xt_iclass_l32e_stateArgs, 0, 0 },
  { 3, Iclass_xt_iclass_s32e_args,
    2, Iclass_xt_iclass_s32e_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_windowbase_args,
    3, Iclass_xt_iclass_rsr_windowbase_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_windowbase_args,
    3, Iclass_xt_iclass_wsr_windowbase_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_windowbase_args,
    3, Iclass_xt_iclass_xsr_windowbase_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_windowstart_args,
    3, Iclass_xt_iclass_rsr_windowstart_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_windowstart_args,
    3, Iclass_xt_iclass_wsr_windowstart_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_windowstart_args,
    3, Iclass_xt_iclass_xsr_windowstart_stateArgs, 0, 0 },
  { 3, Iclass_xt_iclass_add_n_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_addi_n_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_bz6_args,
    0, 0, 0, 0 },
  { 0, 0 /* xt_iclass_ill_n */,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_loadi4_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_mov_n_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_movi_n_args,
    0, 0, 0, 0 },
  { 0, 0 /* xt_iclass_nopn */,
    0, 0, 0, 0 },
  { 1, Iclass_xt_iclass_retn_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_storei4_args,
    0, 0, 0, 0 },
  { 1, Iclass_rur_threadptr_args,
    1, Iclass_rur_threadptr_stateArgs, 0, 0 },
  { 1, Iclass_wur_threadptr_args,
    1, Iclass_wur_threadptr_stateArgs, 0, 0 },
  { 3, Iclass_xt_iclass_addi_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_addmi_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_addsub_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_bit_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_bsi8_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_bsi8b_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_bsi8u_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_bst8_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_bsz12_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_call0_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_callx0_args,
    0, 0, 0, 0 },
  { 4, Iclass_xt_iclass_exti_args,
    0, 0, 0, 0 },
  { 0, 0 /* xt_iclass_ill */,
    0, 0, 0, 0 },
  { 1, Iclass_xt_iclass_jump_args,
    0, 0, 0, 0 },
  { 1, Iclass_xt_iclass_jumpx_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_l16ui_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_l16si_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_l32i_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_l32r_args,
    2, Iclass_xt_iclass_l32r_stateArgs, 0, 0 },
  { 3, Iclass_xt_iclass_l8i_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_loop_args,
    3, Iclass_xt_iclass_loop_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_loopz_args,
    3, Iclass_xt_iclass_loopz_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_movi_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_movz_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_neg_args,
    0, 0, 0, 0 },
  { 0, 0 /* xt_iclass_nop */,
    0, 0, 0, 0 },
  { 1, Iclass_xt_iclass_return_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_s16i_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_s32i_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_s8i_args,
    0, 0, 0, 0 },
  { 1, Iclass_xt_iclass_sar_args,
    1, Iclass_xt_iclass_sar_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_sari_args,
    1, Iclass_xt_iclass_sari_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_shifts_args,
    1, Iclass_xt_iclass_shifts_stateArgs, 0, 0 },
  { 3, Iclass_xt_iclass_shiftst_args,
    1, Iclass_xt_iclass_shiftst_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_shiftt_args,
    1, Iclass_xt_iclass_shiftt_stateArgs, 0, 0 },
  { 3, Iclass_xt_iclass_slli_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_srai_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_srli_args,
    0, 0, 0, 0 },
  { 0, 0 /* xt_iclass_memw */,
    0, 0, 0, 0 },
  { 0, 0 /* xt_iclass_extw */,
    0, 0, 0, 0 },
  { 0, 0 /* xt_iclass_isync */,
    0, 0, 0, 0 },
  { 0, 0 /* xt_iclass_sync */,
    1, Iclass_xt_iclass_sync_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_rsil_args,
    7, Iclass_xt_iclass_rsil_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_lend_args,
    1, Iclass_xt_iclass_rsr_lend_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_lend_args,
    1, Iclass_xt_iclass_wsr_lend_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_lend_args,
    1, Iclass_xt_iclass_xsr_lend_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_lcount_args,
    1, Iclass_xt_iclass_rsr_lcount_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_lcount_args,
    2, Iclass_xt_iclass_wsr_lcount_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_lcount_args,
    2, Iclass_xt_iclass_xsr_lcount_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_lbeg_args,
    1, Iclass_xt_iclass_rsr_lbeg_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_lbeg_args,
    1, Iclass_xt_iclass_wsr_lbeg_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_lbeg_args,
    1, Iclass_xt_iclass_xsr_lbeg_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_sar_args,
    1, Iclass_xt_iclass_rsr_sar_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_sar_args,
    2, Iclass_xt_iclass_wsr_sar_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_sar_args,
    1, Iclass_xt_iclass_xsr_sar_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_litbase_args,
    2, Iclass_xt_iclass_rsr_litbase_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_litbase_args,
    2, Iclass_xt_iclass_wsr_litbase_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_litbase_args,
    2, Iclass_xt_iclass_xsr_litbase_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_176_args,
    2, Iclass_xt_iclass_rsr_176_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_176_args,
    2, Iclass_xt_iclass_wsr_176_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_208_args,
    2, Iclass_xt_iclass_rsr_208_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_ps_args,
    7, Iclass_xt_iclass_rsr_ps_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_ps_args,
    7, Iclass_xt_iclass_wsr_ps_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_ps_args,
    7, Iclass_xt_iclass_xsr_ps_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_epc1_args,
    3, Iclass_xt_iclass_rsr_epc1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_epc1_args,
    3, Iclass_xt_iclass_wsr_epc1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_epc1_args,
    3, Iclass_xt_iclass_xsr_epc1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_excsave1_args,
    3, Iclass_xt_iclass_rsr_excsave1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_excsave1_args,
    3, Iclass_xt_iclass_wsr_excsave1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_excsave1_args,
    3, Iclass_xt_iclass_xsr_excsave1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_epc2_args,
    3, Iclass_xt_iclass_rsr_epc2_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_epc2_args,
    3, Iclass_xt_iclass_wsr_epc2_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_epc2_args,
    3, Iclass_xt_iclass_xsr_epc2_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_excsave2_args,
    3, Iclass_xt_iclass_rsr_excsave2_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_excsave2_args,
    3, Iclass_xt_iclass_wsr_excsave2_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_excsave2_args,
    3, Iclass_xt_iclass_xsr_excsave2_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_eps2_args,
    3, Iclass_xt_iclass_rsr_eps2_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_eps2_args,
    3, Iclass_xt_iclass_wsr_eps2_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_eps2_args,
    3, Iclass_xt_iclass_xsr_eps2_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_excvaddr_args,
    3, Iclass_xt_iclass_rsr_excvaddr_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_excvaddr_args,
    3, Iclass_xt_iclass_wsr_excvaddr_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_excvaddr_args,
    3, Iclass_xt_iclass_xsr_excvaddr_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_depc_args,
    3, Iclass_xt_iclass_rsr_depc_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_depc_args,
    3, Iclass_xt_iclass_wsr_depc_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_depc_args,
    3, Iclass_xt_iclass_xsr_depc_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_exccause_args,
    4, Iclass_xt_iclass_rsr_exccause_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_exccause_args,
    3, Iclass_xt_iclass_wsr_exccause_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_exccause_args,
    3, Iclass_xt_iclass_xsr_exccause_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_misc0_args,
    3, Iclass_xt_iclass_rsr_misc0_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_misc0_args,
    3, Iclass_xt_iclass_wsr_misc0_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_misc0_args,
    3, Iclass_xt_iclass_xsr_misc0_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_misc1_args,
    3, Iclass_xt_iclass_rsr_misc1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_misc1_args,
    3, Iclass_xt_iclass_wsr_misc1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_misc1_args,
    3, Iclass_xt_iclass_xsr_misc1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_prid_args,
    2, Iclass_xt_iclass_rsr_prid_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_vecbase_args,
    3, Iclass_xt_iclass_rsr_vecbase_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_vecbase_args,
    3, Iclass_xt_iclass_wsr_vecbase_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_vecbase_args,
    3, Iclass_xt_iclass_xsr_vecbase_stateArgs, 0, 0 },
  { 3, Iclass_xt_mul16_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_mul32_args,
    0, 0, 0, 0 },
  { 1, Iclass_xt_iclass_rfi_args,
    11, Iclass_xt_iclass_rfi_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wait_args,
    3, Iclass_xt_iclass_wait_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_interrupt_args,
    3, Iclass_xt_iclass_rsr_interrupt_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_intset_args,
    4, Iclass_xt_iclass_wsr_intset_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_intclear_args,
    4, Iclass_xt_iclass_wsr_intclear_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_intenable_args,
    3, Iclass_xt_iclass_rsr_intenable_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_intenable_args,
    3, Iclass_xt_iclass_wsr_intenable_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_intenable_args,
    3, Iclass_xt_iclass_xsr_intenable_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_break_args,
    2, Iclass_xt_iclass_break_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_break_n_args,
    2, Iclass_xt_iclass_break_n_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_debugcause_args,
    4, Iclass_xt_iclass_rsr_debugcause_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_debugcause_args,
    4, Iclass_xt_iclass_wsr_debugcause_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_debugcause_args,
    4, Iclass_xt_iclass_xsr_debugcause_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_icount_args,
    3, Iclass_xt_iclass_rsr_icount_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_icount_args,
    4, Iclass_xt_iclass_wsr_icount_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_icount_args,
    4, Iclass_xt_iclass_xsr_icount_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_icountlevel_args,
    3, Iclass_xt_iclass_rsr_icountlevel_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_icountlevel_args,
    3, Iclass_xt_iclass_wsr_icountlevel_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_icountlevel_args,
    3, Iclass_xt_iclass_xsr_icountlevel_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_ddr_args,
    3, Iclass_xt_iclass_rsr_ddr_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_ddr_args,
    4, Iclass_xt_iclass_wsr_ddr_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_ddr_args,
    4, Iclass_xt_iclass_xsr_ddr_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rfdo_args,
    10, Iclass_xt_iclass_rfdo_stateArgs, 0, 0 },
  { 0, 0 /* xt_iclass_rfdd */,
    1, Iclass_xt_iclass_rfdd_stateArgs, 0, 0 },
  { 3, Iclass_xt_iclass_bbool1_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_bbool4_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_bbool8_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_bbranch_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_bmove_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_RSR_BR_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_WSR_BR_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_XSR_BR_args,
    0, 0, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_ccount_args,
    3, Iclass_xt_iclass_rsr_ccount_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_ccount_args,
    4, Iclass_xt_iclass_wsr_ccount_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_ccount_args,
    4, Iclass_xt_iclass_xsr_ccount_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_ccompare0_args,
    3, Iclass_xt_iclass_rsr_ccompare0_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_ccompare0_args,
    4, Iclass_xt_iclass_wsr_ccompare0_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_ccompare0_args,
    4, Iclass_xt_iclass_xsr_ccompare0_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_ccompare1_args,
    3, Iclass_xt_iclass_rsr_ccompare1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_ccompare1_args,
    4, Iclass_xt_iclass_wsr_ccompare1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_ccompare1_args,
    4, Iclass_xt_iclass_xsr_ccompare1_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_icache_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_icache_inv_args,
    2, Iclass_xt_iclass_icache_inv_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_licx_args,
    2, Iclass_xt_iclass_licx_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_sicx_args,
    2, Iclass_xt_iclass_sicx_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_dcache_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_dcache_ind_args,
    2, Iclass_xt_iclass_dcache_ind_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_dcache_inv_args,
    2, Iclass_xt_iclass_dcache_inv_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_dpf_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_sdct_args,
    2, Iclass_xt_iclass_sdct_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_ldct_args,
    2, Iclass_xt_iclass_ldct_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_ptevaddr_args,
    4, Iclass_xt_iclass_wsr_ptevaddr_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_ptevaddr_args,
    4, Iclass_xt_iclass_rsr_ptevaddr_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_ptevaddr_args,
    5, Iclass_xt_iclass_xsr_ptevaddr_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_rasid_args,
    5, Iclass_xt_iclass_rsr_rasid_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_rasid_args,
    6, Iclass_xt_iclass_wsr_rasid_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_rasid_args,
    6, Iclass_xt_iclass_xsr_rasid_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_itlbcfg_args,
    3, Iclass_xt_iclass_rsr_itlbcfg_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_itlbcfg_args,
    4, Iclass_xt_iclass_wsr_itlbcfg_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_itlbcfg_args,
    4, Iclass_xt_iclass_xsr_itlbcfg_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_dtlbcfg_args,
    3, Iclass_xt_iclass_rsr_dtlbcfg_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_dtlbcfg_args,
    4, Iclass_xt_iclass_wsr_dtlbcfg_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_dtlbcfg_args,
    4, Iclass_xt_iclass_xsr_dtlbcfg_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_idtlb_args,
    3, Iclass_xt_iclass_idtlb_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_rdtlb_args,
    2, Iclass_xt_iclass_rdtlb_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_wdtlb_args,
    3, Iclass_xt_iclass_wdtlb_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_iitlb_args,
    2, Iclass_xt_iclass_iitlb_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_ritlb_args,
    2, Iclass_xt_iclass_ritlb_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_witlb_args,
    2, Iclass_xt_iclass_witlb_stateArgs, 0, 0 },
  { 0, 0 /* xt_iclass_ldpte */,
    2, Iclass_xt_iclass_ldpte_stateArgs, 0, 0 },
  { 0, 0 /* xt_iclass_hwwitlba */,
    1, Iclass_xt_iclass_hwwitlba_stateArgs, 0, 0 },
  { 0, 0 /* xt_iclass_hwwdtlba */,
    1, Iclass_xt_iclass_hwwdtlba_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_cpenable_args,
    3, Iclass_xt_iclass_rsr_cpenable_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_cpenable_args,
    3, Iclass_xt_iclass_wsr_cpenable_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_cpenable_args,
    3, Iclass_xt_iclass_xsr_cpenable_stateArgs, 0, 0 },
  { 3, Iclass_xt_iclass_clamp_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_minmax_args,
    0, 0, 0, 0 },
  { 2, Iclass_xt_iclass_nsa_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_sx_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_l32ai_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_s32ri_args,
    0, 0, 0, 0 },
  { 3, Iclass_xt_iclass_s32c1i_args,
    3, Iclass_xt_iclass_s32c1i_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_scompare1_args,
    1, Iclass_xt_iclass_rsr_scompare1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_scompare1_args,
    1, Iclass_xt_iclass_wsr_scompare1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_scompare1_args,
    1, Iclass_xt_iclass_xsr_scompare1_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_rsr_atomctl_args,
    3, Iclass_xt_iclass_rsr_atomctl_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_wsr_atomctl_args,
    4, Iclass_xt_iclass_wsr_atomctl_stateArgs, 0, 0 },
  { 1, Iclass_xt_iclass_xsr_atomctl_args,
    4, Iclass_xt_iclass_xsr_atomctl_stateArgs, 0, 0 },
  { 2, Iclass_xt_iclass_rer_args,
    4, Iclass_xt_iclass_rer_stateArgs, 2, Iclass_xt_iclass_rer_intfArgs },
  { 2, Iclass_xt_iclass_wer_args,
    7, Iclass_xt_iclass_wer_stateArgs, 0, 0 },
  { 1, Iclass_rur_ae_ovf_sar_args,
    3, Iclass_rur_ae_ovf_sar_stateArgs, 0, 0 },
  { 1, Iclass_wur_ae_ovf_sar_args,
    3, Iclass_wur_ae_ovf_sar_stateArgs, 0, 0 },
  { 1, Iclass_rur_ae_bithead_args,
    2, Iclass_rur_ae_bithead_stateArgs, 0, 0 },
  { 1, Iclass_wur_ae_bithead_args,
    2, Iclass_wur_ae_bithead_stateArgs, 0, 0 },
  { 1, Iclass_rur_ae_ts_fts_bu_bp_args,
    5, Iclass_rur_ae_ts_fts_bu_bp_stateArgs, 0, 0 },
  { 1, Iclass_wur_ae_ts_fts_bu_bp_args,
    5, Iclass_wur_ae_ts_fts_bu_bp_stateArgs, 0, 0 },
  { 1, Iclass_rur_ae_sd_no_args,
    3, Iclass_rur_ae_sd_no_stateArgs, 0, 0 },
  { 1, Iclass_wur_ae_sd_no_args,
    3, Iclass_wur_ae_sd_no_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_rur_ae_overflow_args,
    2, Iclass_ae_iclass_rur_ae_overflow_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_wur_ae_overflow_args,
    2, Iclass_ae_iclass_wur_ae_overflow_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_rur_ae_sar_args,
    2, Iclass_ae_iclass_rur_ae_sar_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_wur_ae_sar_args,
    2, Iclass_ae_iclass_wur_ae_sar_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_rur_ae_bitptr_args,
    2, Iclass_ae_iclass_rur_ae_bitptr_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_wur_ae_bitptr_args,
    2, Iclass_ae_iclass_wur_ae_bitptr_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_rur_ae_bitsused_args,
    2, Iclass_ae_iclass_rur_ae_bitsused_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_wur_ae_bitsused_args,
    2, Iclass_ae_iclass_wur_ae_bitsused_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_rur_ae_tablesize_args,
    2, Iclass_ae_iclass_rur_ae_tablesize_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_wur_ae_tablesize_args,
    2, Iclass_ae_iclass_wur_ae_tablesize_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_rur_ae_first_ts_args,
    2, Iclass_ae_iclass_rur_ae_first_ts_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_wur_ae_first_ts_args,
    2, Iclass_ae_iclass_wur_ae_first_ts_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_rur_ae_nextoffset_args,
    2, Iclass_ae_iclass_rur_ae_nextoffset_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_wur_ae_nextoffset_args,
    2, Iclass_ae_iclass_wur_ae_nextoffset_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_rur_ae_searchdone_args,
    2, Iclass_ae_iclass_rur_ae_searchdone_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_wur_ae_searchdone_args,
    2, Iclass_ae_iclass_wur_ae_searchdone_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp16f_i_args,
    1, Iclass_ae_iclass_lp16f_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp16f_iu_args,
    1, Iclass_ae_iclass_lp16f_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp16f_x_args,
    1, Iclass_ae_iclass_lp16f_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp16f_xu_args,
    1, Iclass_ae_iclass_lp16f_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24_i_args,
    1, Iclass_ae_iclass_lp24_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24_iu_args,
    1, Iclass_ae_iclass_lp24_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24_x_args,
    1, Iclass_ae_iclass_lp24_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24_xu_args,
    1, Iclass_ae_iclass_lp24_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24f_i_args,
    1, Iclass_ae_iclass_lp24f_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24f_iu_args,
    1, Iclass_ae_iclass_lp24f_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24f_x_args,
    1, Iclass_ae_iclass_lp24f_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24f_xu_args,
    1, Iclass_ae_iclass_lp24f_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp16x2f_i_args,
    1, Iclass_ae_iclass_lp16x2f_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp16x2f_iu_args,
    1, Iclass_ae_iclass_lp16x2f_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp16x2f_x_args,
    1, Iclass_ae_iclass_lp16x2f_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp16x2f_xu_args,
    1, Iclass_ae_iclass_lp16x2f_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24x2f_i_args,
    1, Iclass_ae_iclass_lp24x2f_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24x2f_iu_args,
    1, Iclass_ae_iclass_lp24x2f_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24x2f_x_args,
    1, Iclass_ae_iclass_lp24x2f_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24x2f_xu_args,
    1, Iclass_ae_iclass_lp24x2f_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24x2_i_args,
    1, Iclass_ae_iclass_lp24x2_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24x2_iu_args,
    1, Iclass_ae_iclass_lp24x2_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24x2_x_args,
    1, Iclass_ae_iclass_lp24x2_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lp24x2_xu_args,
    1, Iclass_ae_iclass_lp24x2_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp16x2f_i_args,
    1, Iclass_ae_iclass_sp16x2f_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp16x2f_iu_args,
    1, Iclass_ae_iclass_sp16x2f_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp16x2f_x_args,
    1, Iclass_ae_iclass_sp16x2f_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp16x2f_xu_args,
    1, Iclass_ae_iclass_sp16x2f_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24x2s_i_args,
    1, Iclass_ae_iclass_sp24x2s_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24x2s_iu_args,
    1, Iclass_ae_iclass_sp24x2s_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24x2s_x_args,
    1, Iclass_ae_iclass_sp24x2s_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24x2s_xu_args,
    1, Iclass_ae_iclass_sp24x2s_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24x2f_i_args,
    1, Iclass_ae_iclass_sp24x2f_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24x2f_iu_args,
    1, Iclass_ae_iclass_sp24x2f_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24x2f_x_args,
    1, Iclass_ae_iclass_sp24x2f_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24x2f_xu_args,
    1, Iclass_ae_iclass_sp24x2f_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp16f_l_i_args,
    1, Iclass_ae_iclass_sp16f_l_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp16f_l_iu_args,
    1, Iclass_ae_iclass_sp16f_l_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp16f_l_x_args,
    1, Iclass_ae_iclass_sp16f_l_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp16f_l_xu_args,
    1, Iclass_ae_iclass_sp16f_l_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24s_l_i_args,
    1, Iclass_ae_iclass_sp24s_l_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24s_l_iu_args,
    1, Iclass_ae_iclass_sp24s_l_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24s_l_x_args,
    1, Iclass_ae_iclass_sp24s_l_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24s_l_xu_args,
    1, Iclass_ae_iclass_sp24s_l_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24f_l_i_args,
    1, Iclass_ae_iclass_sp24f_l_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24f_l_iu_args,
    1, Iclass_ae_iclass_sp24f_l_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24f_l_x_args,
    1, Iclass_ae_iclass_sp24f_l_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sp24f_l_xu_args,
    1, Iclass_ae_iclass_sp24f_l_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lq56_i_args,
    1, Iclass_ae_iclass_lq56_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lq56_iu_args,
    1, Iclass_ae_iclass_lq56_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lq56_x_args,
    1, Iclass_ae_iclass_lq56_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lq56_xu_args,
    1, Iclass_ae_iclass_lq56_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lq32f_i_args,
    1, Iclass_ae_iclass_lq32f_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lq32f_iu_args,
    1, Iclass_ae_iclass_lq32f_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lq32f_x_args,
    1, Iclass_ae_iclass_lq32f_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lq32f_xu_args,
    1, Iclass_ae_iclass_lq32f_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sq56s_i_args,
    1, Iclass_ae_iclass_sq56s_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sq56s_iu_args,
    1, Iclass_ae_iclass_sq56s_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sq56s_x_args,
    1, Iclass_ae_iclass_sq56s_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sq56s_xu_args,
    1, Iclass_ae_iclass_sq56s_xu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sq32f_i_args,
    1, Iclass_ae_iclass_sq32f_i_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sq32f_iu_args,
    1, Iclass_ae_iclass_sq32f_iu_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sq32f_x_args,
    1, Iclass_ae_iclass_sq32f_x_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sq32f_xu_args,
    1, Iclass_ae_iclass_sq32f_xu_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_zerop48_args,
    1, Iclass_ae_iclass_zerop48_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_movp48_args,
    1, Iclass_ae_iclass_movp48_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_selp24_ll_args,
    1, Iclass_ae_iclass_selp24_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_selp24_lh_args,
    1, Iclass_ae_iclass_selp24_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_selp24_hl_args,
    1, Iclass_ae_iclass_selp24_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_selp24_hh_args,
    1, Iclass_ae_iclass_selp24_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_movtp24x2_args,
    1, Iclass_ae_iclass_movtp24x2_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_movfp24x2_args,
    1, Iclass_ae_iclass_movfp24x2_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_movtp48_args,
    1, Iclass_ae_iclass_movtp48_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_movfp48_args,
    1, Iclass_ae_iclass_movfp48_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_movpa24x2_args,
    1, Iclass_ae_iclass_movpa24x2_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_truncp24a32x2_args,
    1, Iclass_ae_iclass_truncp24a32x2_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_cvta32p24_l_args,
    1, Iclass_ae_iclass_cvta32p24_l_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_cvta32p24_h_args,
    1, Iclass_ae_iclass_cvta32p24_h_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_cvtp24a16x2_ll_args,
    1, Iclass_ae_iclass_cvtp24a16x2_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_cvtp24a16x2_lh_args,
    1, Iclass_ae_iclass_cvtp24a16x2_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_cvtp24a16x2_hl_args,
    1, Iclass_ae_iclass_cvtp24a16x2_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_cvtp24a16x2_hh_args,
    1, Iclass_ae_iclass_cvtp24a16x2_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_truncp24q48x2_args,
    1, Iclass_ae_iclass_truncp24q48x2_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_truncp16_args,
    1, Iclass_ae_iclass_truncp16_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_roundsp24q48sym_args,
    2, Iclass_ae_iclass_roundsp24q48sym_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_roundsp24q48asym_args,
    2, Iclass_ae_iclass_roundsp24q48asym_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_roundsp16q48sym_args,
    2, Iclass_ae_iclass_roundsp16q48sym_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_roundsp16q48asym_args,
    2, Iclass_ae_iclass_roundsp16q48asym_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_roundsp16sym_args,
    2, Iclass_ae_iclass_roundsp16sym_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_roundsp16asym_args,
    2, Iclass_ae_iclass_roundsp16asym_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_zeroq56_args,
    1, Iclass_ae_iclass_zeroq56_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_movq56_args,
    1, Iclass_ae_iclass_movq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_movtq56_args,
    1, Iclass_ae_iclass_movtq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_movfq56_args,
    1, Iclass_ae_iclass_movfq56_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_cvtq48a32s_args,
    1, Iclass_ae_iclass_cvtq48a32s_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_cvtq48p24s_l_args,
    1, Iclass_ae_iclass_cvtq48p24s_l_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_cvtq48p24s_h_args,
    1, Iclass_ae_iclass_cvtq48p24s_h_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_satq48s_args,
    2, Iclass_ae_iclass_satq48s_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_truncq32_args,
    1, Iclass_ae_iclass_truncq32_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_roundsq32sym_args,
    2, Iclass_ae_iclass_roundsq32sym_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_roundsq32asym_args,
    2, Iclass_ae_iclass_roundsq32asym_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_trunca32q48_args,
    1, Iclass_ae_iclass_trunca32q48_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_movap24s_l_args,
    1, Iclass_ae_iclass_movap24s_l_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_movap24s_h_args,
    1, Iclass_ae_iclass_movap24s_h_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_trunca16p24s_l_args,
    1, Iclass_ae_iclass_trunca16p24s_l_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_trunca16p24s_h_args,
    1, Iclass_ae_iclass_trunca16p24s_h_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_addp24_args,
    1, Iclass_ae_iclass_addp24_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_subp24_args,
    1, Iclass_ae_iclass_subp24_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_negp24_args,
    1, Iclass_ae_iclass_negp24_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_absp24_args,
    1, Iclass_ae_iclass_absp24_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_maxp24s_args,
    1, Iclass_ae_iclass_maxp24s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_minp24s_args,
    1, Iclass_ae_iclass_minp24s_stateArgs, 0, 0 },
  { 4, Iclass_ae_iclass_maxbp24s_args,
    1, Iclass_ae_iclass_maxbp24s_stateArgs, 0, 0 },
  { 4, Iclass_ae_iclass_minbp24s_args,
    1, Iclass_ae_iclass_minbp24s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_addsp24s_args,
    2, Iclass_ae_iclass_addsp24s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_subsp24s_args,
    2, Iclass_ae_iclass_subsp24s_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_negsp24s_args,
    2, Iclass_ae_iclass_negsp24s_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_abssp24s_args,
    2, Iclass_ae_iclass_abssp24s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_andp48_args,
    1, Iclass_ae_iclass_andp48_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_nandp48_args,
    1, Iclass_ae_iclass_nandp48_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_orp48_args,
    1, Iclass_ae_iclass_orp48_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_xorp48_args,
    1, Iclass_ae_iclass_xorp48_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_ltp24s_args,
    1, Iclass_ae_iclass_ltp24s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lep24s_args,
    1, Iclass_ae_iclass_lep24s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_eqp24_args,
    1, Iclass_ae_iclass_eqp24_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_addq56_args,
    1, Iclass_ae_iclass_addq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_subq56_args,
    1, Iclass_ae_iclass_subq56_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_negq56_args,
    1, Iclass_ae_iclass_negq56_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_absq56_args,
    1, Iclass_ae_iclass_absq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_maxq56s_args,
    1, Iclass_ae_iclass_maxq56s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_minq56s_args,
    1, Iclass_ae_iclass_minq56s_stateArgs, 0, 0 },
  { 4, Iclass_ae_iclass_maxbq56s_args,
    1, Iclass_ae_iclass_maxbq56s_stateArgs, 0, 0 },
  { 4, Iclass_ae_iclass_minbq56s_args,
    1, Iclass_ae_iclass_minbq56s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_addsq56s_args,
    2, Iclass_ae_iclass_addsq56s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_subsq56s_args,
    2, Iclass_ae_iclass_subsq56s_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_negsq56s_args,
    2, Iclass_ae_iclass_negsq56s_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_abssq56s_args,
    2, Iclass_ae_iclass_abssq56s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_andq56_args,
    1, Iclass_ae_iclass_andq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_nandq56_args,
    1, Iclass_ae_iclass_nandq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_orq56_args,
    1, Iclass_ae_iclass_orq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_xorq56_args,
    1, Iclass_ae_iclass_xorq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sllip24_args,
    1, Iclass_ae_iclass_sllip24_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_srlip24_args,
    1, Iclass_ae_iclass_srlip24_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sraip24_args,
    1, Iclass_ae_iclass_sraip24_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_sllsp24_args,
    2, Iclass_ae_iclass_sllsp24_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_srlsp24_args,
    2, Iclass_ae_iclass_srlsp24_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_srasp24_args,
    2, Iclass_ae_iclass_srasp24_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sllisp24s_args,
    2, Iclass_ae_iclass_sllisp24s_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_sllssp24s_args,
    3, Iclass_ae_iclass_sllssp24s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_slliq56_args,
    1, Iclass_ae_iclass_slliq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_srliq56_args,
    1, Iclass_ae_iclass_srliq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sraiq56_args,
    1, Iclass_ae_iclass_sraiq56_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_sllsq56_args,
    2, Iclass_ae_iclass_sllsq56_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_srlsq56_args,
    2, Iclass_ae_iclass_srlsq56_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_srasq56_args,
    2, Iclass_ae_iclass_srasq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sllaq56_args,
    1, Iclass_ae_iclass_sllaq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_srlaq56_args,
    1, Iclass_ae_iclass_srlaq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sraaq56_args,
    1, Iclass_ae_iclass_sraaq56_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sllisq56s_args,
    2, Iclass_ae_iclass_sllisq56s_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_sllssq56s_args,
    3, Iclass_ae_iclass_sllssq56s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sllasq56s_args,
    2, Iclass_ae_iclass_sllasq56s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_ltq56s_args,
    1, Iclass_ae_iclass_ltq56s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_leq56s_args,
    1, Iclass_ae_iclass_leq56s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_eqq56_args,
    1, Iclass_ae_iclass_eqq56_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_nsaq56s_args,
    1, Iclass_ae_iclass_nsaq56s_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulfs32p16s_ll_args,
    2, Iclass_ae_iclass_mulfs32p16s_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulfp24s_ll_args,
    1, Iclass_ae_iclass_mulfp24s_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulp24s_ll_args,
    1, Iclass_ae_iclass_mulp24s_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulfs32p16s_lh_args,
    2, Iclass_ae_iclass_mulfs32p16s_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulfp24s_lh_args,
    1, Iclass_ae_iclass_mulfp24s_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulp24s_lh_args,
    1, Iclass_ae_iclass_mulp24s_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulfs32p16s_hl_args,
    2, Iclass_ae_iclass_mulfs32p16s_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulfp24s_hl_args,
    1, Iclass_ae_iclass_mulfp24s_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulp24s_hl_args,
    1, Iclass_ae_iclass_mulp24s_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulfs32p16s_hh_args,
    2, Iclass_ae_iclass_mulfs32p16s_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulfp24s_hh_args,
    1, Iclass_ae_iclass_mulfp24s_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulp24s_hh_args,
    1, Iclass_ae_iclass_mulp24s_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafs32p16s_ll_args,
    2, Iclass_ae_iclass_mulafs32p16s_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafp24s_ll_args,
    1, Iclass_ae_iclass_mulafp24s_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulap24s_ll_args,
    1, Iclass_ae_iclass_mulap24s_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafs32p16s_lh_args,
    2, Iclass_ae_iclass_mulafs32p16s_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafp24s_lh_args,
    1, Iclass_ae_iclass_mulafp24s_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulap24s_lh_args,
    1, Iclass_ae_iclass_mulap24s_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafs32p16s_hl_args,
    2, Iclass_ae_iclass_mulafs32p16s_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafp24s_hl_args,
    1, Iclass_ae_iclass_mulafp24s_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulap24s_hl_args,
    1, Iclass_ae_iclass_mulap24s_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafs32p16s_hh_args,
    2, Iclass_ae_iclass_mulafs32p16s_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafp24s_hh_args,
    1, Iclass_ae_iclass_mulafp24s_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulap24s_hh_args,
    1, Iclass_ae_iclass_mulap24s_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfs32p16s_ll_args,
    2, Iclass_ae_iclass_mulsfs32p16s_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfp24s_ll_args,
    1, Iclass_ae_iclass_mulsfp24s_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsp24s_ll_args,
    1, Iclass_ae_iclass_mulsp24s_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfs32p16s_lh_args,
    2, Iclass_ae_iclass_mulsfs32p16s_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfp24s_lh_args,
    1, Iclass_ae_iclass_mulsfp24s_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsp24s_lh_args,
    1, Iclass_ae_iclass_mulsp24s_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfs32p16s_hl_args,
    2, Iclass_ae_iclass_mulsfs32p16s_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfp24s_hl_args,
    1, Iclass_ae_iclass_mulsfp24s_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsp24s_hl_args,
    1, Iclass_ae_iclass_mulsp24s_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfs32p16s_hh_args,
    2, Iclass_ae_iclass_mulsfs32p16s_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfp24s_hh_args,
    1, Iclass_ae_iclass_mulsfp24s_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsp24s_hh_args,
    1, Iclass_ae_iclass_mulsp24s_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafs56p24s_ll_args,
    2, Iclass_ae_iclass_mulafs56p24s_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulas56p24s_ll_args,
    2, Iclass_ae_iclass_mulas56p24s_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafs56p24s_lh_args,
    2, Iclass_ae_iclass_mulafs56p24s_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulas56p24s_lh_args,
    2, Iclass_ae_iclass_mulas56p24s_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafs56p24s_hl_args,
    2, Iclass_ae_iclass_mulafs56p24s_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulas56p24s_hl_args,
    2, Iclass_ae_iclass_mulas56p24s_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafs56p24s_hh_args,
    2, Iclass_ae_iclass_mulafs56p24s_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulas56p24s_hh_args,
    2, Iclass_ae_iclass_mulas56p24s_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfs56p24s_ll_args,
    2, Iclass_ae_iclass_mulsfs56p24s_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulss56p24s_ll_args,
    2, Iclass_ae_iclass_mulss56p24s_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfs56p24s_lh_args,
    2, Iclass_ae_iclass_mulsfs56p24s_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulss56p24s_lh_args,
    2, Iclass_ae_iclass_mulss56p24s_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfs56p24s_hl_args,
    2, Iclass_ae_iclass_mulsfs56p24s_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulss56p24s_hl_args,
    2, Iclass_ae_iclass_mulss56p24s_hl_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfs56p24s_hh_args,
    2, Iclass_ae_iclass_mulsfs56p24s_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulss56p24s_hh_args,
    2, Iclass_ae_iclass_mulss56p24s_hh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulfq32sp16s_l_args,
    1, Iclass_ae_iclass_mulfq32sp16s_l_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulfq32sp16s_h_args,
    1, Iclass_ae_iclass_mulfq32sp16s_h_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulfq32sp16u_l_args,
    1, Iclass_ae_iclass_mulfq32sp16u_l_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulfq32sp16u_h_args,
    1, Iclass_ae_iclass_mulfq32sp16u_h_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulq32sp16s_l_args,
    1, Iclass_ae_iclass_mulq32sp16s_l_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulq32sp16s_h_args,
    1, Iclass_ae_iclass_mulq32sp16s_h_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulq32sp16u_l_args,
    1, Iclass_ae_iclass_mulq32sp16u_l_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulq32sp16u_h_args,
    1, Iclass_ae_iclass_mulq32sp16u_h_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafq32sp16s_l_args,
    1, Iclass_ae_iclass_mulafq32sp16s_l_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafq32sp16s_h_args,
    1, Iclass_ae_iclass_mulafq32sp16s_h_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafq32sp16u_l_args,
    1, Iclass_ae_iclass_mulafq32sp16u_l_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulafq32sp16u_h_args,
    1, Iclass_ae_iclass_mulafq32sp16u_h_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulaq32sp16s_l_args,
    1, Iclass_ae_iclass_mulaq32sp16s_l_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulaq32sp16s_h_args,
    1, Iclass_ae_iclass_mulaq32sp16s_h_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulaq32sp16u_l_args,
    1, Iclass_ae_iclass_mulaq32sp16u_l_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulaq32sp16u_h_args,
    1, Iclass_ae_iclass_mulaq32sp16u_h_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfq32sp16s_l_args,
    1, Iclass_ae_iclass_mulsfq32sp16s_l_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfq32sp16s_h_args,
    1, Iclass_ae_iclass_mulsfq32sp16s_h_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfq32sp16u_l_args,
    1, Iclass_ae_iclass_mulsfq32sp16u_l_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsfq32sp16u_h_args,
    1, Iclass_ae_iclass_mulsfq32sp16u_h_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsq32sp16s_l_args,
    1, Iclass_ae_iclass_mulsq32sp16s_l_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsq32sp16s_h_args,
    1, Iclass_ae_iclass_mulsq32sp16s_h_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsq32sp16u_l_args,
    1, Iclass_ae_iclass_mulsq32sp16u_l_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsq32sp16u_h_args,
    1, Iclass_ae_iclass_mulsq32sp16u_h_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzaaq32sp16s_ll_args,
    1, Iclass_ae_iclass_mulzaaq32sp16s_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzaafq32sp16s_ll_args,
    1, Iclass_ae_iclass_mulzaafq32sp16s_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzaaq32sp16u_ll_args,
    1, Iclass_ae_iclass_mulzaaq32sp16u_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzaafq32sp16u_ll_args,
    1, Iclass_ae_iclass_mulzaafq32sp16u_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzaaq32sp16s_hh_args,
    1, Iclass_ae_iclass_mulzaaq32sp16s_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzaafq32sp16s_hh_args,
    1, Iclass_ae_iclass_mulzaafq32sp16s_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzaaq32sp16u_hh_args,
    1, Iclass_ae_iclass_mulzaaq32sp16u_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzaafq32sp16u_hh_args,
    1, Iclass_ae_iclass_mulzaafq32sp16u_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzaaq32sp16s_lh_args,
    1, Iclass_ae_iclass_mulzaaq32sp16s_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzaafq32sp16s_lh_args,
    1, Iclass_ae_iclass_mulzaafq32sp16s_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzaaq32sp16u_lh_args,
    1, Iclass_ae_iclass_mulzaaq32sp16u_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzaafq32sp16u_lh_args,
    1, Iclass_ae_iclass_mulzaafq32sp16u_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzasq32sp16s_ll_args,
    1, Iclass_ae_iclass_mulzasq32sp16s_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzasfq32sp16s_ll_args,
    1, Iclass_ae_iclass_mulzasfq32sp16s_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzasq32sp16u_ll_args,
    1, Iclass_ae_iclass_mulzasq32sp16u_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzasfq32sp16u_ll_args,
    1, Iclass_ae_iclass_mulzasfq32sp16u_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzasq32sp16s_hh_args,
    1, Iclass_ae_iclass_mulzasq32sp16s_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzasfq32sp16s_hh_args,
    1, Iclass_ae_iclass_mulzasfq32sp16s_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzasq32sp16u_hh_args,
    1, Iclass_ae_iclass_mulzasq32sp16u_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzasfq32sp16u_hh_args,
    1, Iclass_ae_iclass_mulzasfq32sp16u_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzasq32sp16s_lh_args,
    1, Iclass_ae_iclass_mulzasq32sp16s_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzasfq32sp16s_lh_args,
    1, Iclass_ae_iclass_mulzasfq32sp16s_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzasq32sp16u_lh_args,
    1, Iclass_ae_iclass_mulzasq32sp16u_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzasfq32sp16u_lh_args,
    1, Iclass_ae_iclass_mulzasfq32sp16u_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzsaq32sp16s_ll_args,
    1, Iclass_ae_iclass_mulzsaq32sp16s_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzsafq32sp16s_ll_args,
    1, Iclass_ae_iclass_mulzsafq32sp16s_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzsaq32sp16u_ll_args,
    1, Iclass_ae_iclass_mulzsaq32sp16u_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzsafq32sp16u_ll_args,
    1, Iclass_ae_iclass_mulzsafq32sp16u_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzsaq32sp16s_hh_args,
    1, Iclass_ae_iclass_mulzsaq32sp16s_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzsafq32sp16s_hh_args,
    1, Iclass_ae_iclass_mulzsafq32sp16s_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzsaq32sp16u_hh_args,
    1, Iclass_ae_iclass_mulzsaq32sp16u_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzsafq32sp16u_hh_args,
    1, Iclass_ae_iclass_mulzsafq32sp16u_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzsaq32sp16s_lh_args,
    1, Iclass_ae_iclass_mulzsaq32sp16s_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzsafq32sp16s_lh_args,
    1, Iclass_ae_iclass_mulzsafq32sp16s_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzsaq32sp16u_lh_args,
    1, Iclass_ae_iclass_mulzsaq32sp16u_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzsafq32sp16u_lh_args,
    1, Iclass_ae_iclass_mulzsafq32sp16u_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzssq32sp16s_ll_args,
    1, Iclass_ae_iclass_mulzssq32sp16s_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzssfq32sp16s_ll_args,
    1, Iclass_ae_iclass_mulzssfq32sp16s_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzssq32sp16u_ll_args,
    1, Iclass_ae_iclass_mulzssq32sp16u_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzssfq32sp16u_ll_args,
    1, Iclass_ae_iclass_mulzssfq32sp16u_ll_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzssq32sp16s_hh_args,
    1, Iclass_ae_iclass_mulzssq32sp16s_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzssfq32sp16s_hh_args,
    1, Iclass_ae_iclass_mulzssfq32sp16s_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzssq32sp16u_hh_args,
    1, Iclass_ae_iclass_mulzssq32sp16u_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzssfq32sp16u_hh_args,
    1, Iclass_ae_iclass_mulzssfq32sp16u_hh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzssq32sp16s_lh_args,
    1, Iclass_ae_iclass_mulzssq32sp16s_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzssfq32sp16s_lh_args,
    1, Iclass_ae_iclass_mulzssfq32sp16s_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzssq32sp16u_lh_args,
    1, Iclass_ae_iclass_mulzssq32sp16u_lh_stateArgs, 0, 0 },
  { 5, Iclass_ae_iclass_mulzssfq32sp16u_lh_args,
    1, Iclass_ae_iclass_mulzssfq32sp16u_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzaafp24s_hh_ll_args,
    1, Iclass_ae_iclass_mulzaafp24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzaap24s_hh_ll_args,
    1, Iclass_ae_iclass_mulzaap24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzaafp24s_hl_lh_args,
    1, Iclass_ae_iclass_mulzaafp24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzaap24s_hl_lh_args,
    1, Iclass_ae_iclass_mulzaap24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzasfp24s_hh_ll_args,
    1, Iclass_ae_iclass_mulzasfp24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzasp24s_hh_ll_args,
    1, Iclass_ae_iclass_mulzasp24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzasfp24s_hl_lh_args,
    1, Iclass_ae_iclass_mulzasfp24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzasp24s_hl_lh_args,
    1, Iclass_ae_iclass_mulzasp24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzsafp24s_hh_ll_args,
    1, Iclass_ae_iclass_mulzsafp24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzsap24s_hh_ll_args,
    1, Iclass_ae_iclass_mulzsap24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzsafp24s_hl_lh_args,
    1, Iclass_ae_iclass_mulzsafp24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzsap24s_hl_lh_args,
    1, Iclass_ae_iclass_mulzsap24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzssfp24s_hh_ll_args,
    1, Iclass_ae_iclass_mulzssfp24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzssp24s_hh_ll_args,
    1, Iclass_ae_iclass_mulzssp24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzssfp24s_hl_lh_args,
    1, Iclass_ae_iclass_mulzssfp24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulzssp24s_hl_lh_args,
    1, Iclass_ae_iclass_mulzssp24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulaafp24s_hh_ll_args,
    1, Iclass_ae_iclass_mulaafp24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulaap24s_hh_ll_args,
    1, Iclass_ae_iclass_mulaap24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulaafp24s_hl_lh_args,
    1, Iclass_ae_iclass_mulaafp24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulaap24s_hl_lh_args,
    1, Iclass_ae_iclass_mulaap24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulasfp24s_hh_ll_args,
    1, Iclass_ae_iclass_mulasfp24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulasp24s_hh_ll_args,
    1, Iclass_ae_iclass_mulasp24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulasfp24s_hl_lh_args,
    1, Iclass_ae_iclass_mulasfp24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulasp24s_hl_lh_args,
    1, Iclass_ae_iclass_mulasp24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsafp24s_hh_ll_args,
    1, Iclass_ae_iclass_mulsafp24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsap24s_hh_ll_args,
    1, Iclass_ae_iclass_mulsap24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsafp24s_hl_lh_args,
    1, Iclass_ae_iclass_mulsafp24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulsap24s_hl_lh_args,
    1, Iclass_ae_iclass_mulsap24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulssfp24s_hh_ll_args,
    1, Iclass_ae_iclass_mulssfp24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulssp24s_hh_ll_args,
    1, Iclass_ae_iclass_mulssp24s_hh_ll_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulssfp24s_hl_lh_args,
    1, Iclass_ae_iclass_mulssfp24s_hl_lh_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_mulssp24s_hl_lh_args,
    1, Iclass_ae_iclass_mulssp24s_hl_lh_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_sha32_args,
    0, 0, 0, 0 },
  { 3, Iclass_ae_iclass_vldl32t_args,
    5, Iclass_ae_iclass_vldl32t_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_vldl16t_args,
    5, Iclass_ae_iclass_vldl16t_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_vldl16c_args,
    8, Iclass_ae_iclass_vldl16c_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_vldsht_args,
    6, Iclass_ae_iclass_vldsht_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_lb_args,
    3, Iclass_ae_iclass_lb_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_lbi_args,
    3, Iclass_ae_iclass_lbi_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lbk_args,
    3, Iclass_ae_iclass_lbk_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_lbki_args,
    3, Iclass_ae_iclass_lbki_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_db_args,
    3, Iclass_ae_iclass_db_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_dbi_args,
    3, Iclass_ae_iclass_dbi_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_vlel32t_args,
    3, Iclass_ae_iclass_vlel32t_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_vlel16t_args,
    3, Iclass_ae_iclass_vlel16t_stateArgs, 0, 0 },
  { 2, Iclass_ae_iclass_sb_args,
    4, Iclass_ae_iclass_sb_stateArgs, 0, 0 },
  { 3, Iclass_ae_iclass_sbi_args,
    3, Iclass_ae_iclass_sbi_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_vles16c_args,
    5, Iclass_ae_iclass_vles16c_stateArgs, 0, 0 },
  { 1, Iclass_ae_iclass_sbf_args,
    3, Iclass_ae_iclass_sbf_stateArgs, 0, 0 }
};

enum xtensa_iclass_id {
  ICLASS_xt_iclass_excw,
  ICLASS_xt_iclass_rfe,
  ICLASS_xt_iclass_rfde,
  ICLASS_xt_iclass_syscall,
  ICLASS_xt_iclass_simcall,
  ICLASS_xt_iclass_call12,
  ICLASS_xt_iclass_call8,
  ICLASS_xt_iclass_call4,
  ICLASS_xt_iclass_callx12,
  ICLASS_xt_iclass_callx8,
  ICLASS_xt_iclass_callx4,
  ICLASS_xt_iclass_entry,
  ICLASS_xt_iclass_movsp,
  ICLASS_xt_iclass_rotw,
  ICLASS_xt_iclass_retw,
  ICLASS_xt_iclass_rfwou,
  ICLASS_xt_iclass_l32e,
  ICLASS_xt_iclass_s32e,
  ICLASS_xt_iclass_rsr_windowbase,
  ICLASS_xt_iclass_wsr_windowbase,
  ICLASS_xt_iclass_xsr_windowbase,
  ICLASS_xt_iclass_rsr_windowstart,
  ICLASS_xt_iclass_wsr_windowstart,
  ICLASS_xt_iclass_xsr_windowstart,
  ICLASS_xt_iclass_add_n,
  ICLASS_xt_iclass_addi_n,
  ICLASS_xt_iclass_bz6,
  ICLASS_xt_iclass_ill_n,
  ICLASS_xt_iclass_loadi4,
  ICLASS_xt_iclass_mov_n,
  ICLASS_xt_iclass_movi_n,
  ICLASS_xt_iclass_nopn,
  ICLASS_xt_iclass_retn,
  ICLASS_xt_iclass_storei4,
  ICLASS_rur_threadptr,
  ICLASS_wur_threadptr,
  ICLASS_xt_iclass_addi,
  ICLASS_xt_iclass_addmi,
  ICLASS_xt_iclass_addsub,
  ICLASS_xt_iclass_bit,
  ICLASS_xt_iclass_bsi8,
  ICLASS_xt_iclass_bsi8b,
  ICLASS_xt_iclass_bsi8u,
  ICLASS_xt_iclass_bst8,
  ICLASS_xt_iclass_bsz12,
  ICLASS_xt_iclass_call0,
  ICLASS_xt_iclass_callx0,
  ICLASS_xt_iclass_exti,
  ICLASS_xt_iclass_ill,
  ICLASS_xt_iclass_jump,
  ICLASS_xt_iclass_jumpx,
  ICLASS_xt_iclass_l16ui,
  ICLASS_xt_iclass_l16si,
  ICLASS_xt_iclass_l32i,
  ICLASS_xt_iclass_l32r,
  ICLASS_xt_iclass_l8i,
  ICLASS_xt_iclass_loop,
  ICLASS_xt_iclass_loopz,
  ICLASS_xt_iclass_movi,
  ICLASS_xt_iclass_movz,
  ICLASS_xt_iclass_neg,
  ICLASS_xt_iclass_nop,
  ICLASS_xt_iclass_return,
  ICLASS_xt_iclass_s16i,
  ICLASS_xt_iclass_s32i,
  ICLASS_xt_iclass_s8i,
  ICLASS_xt_iclass_sar,
  ICLASS_xt_iclass_sari,
  ICLASS_xt_iclass_shifts,
  ICLASS_xt_iclass_shiftst,
  ICLASS_xt_iclass_shiftt,
  ICLASS_xt_iclass_slli,
  ICLASS_xt_iclass_srai,
  ICLASS_xt_iclass_srli,
  ICLASS_xt_iclass_memw,
  ICLASS_xt_iclass_extw,
  ICLASS_xt_iclass_isync,
  ICLASS_xt_iclass_sync,
  ICLASS_xt_iclass_rsil,
  ICLASS_xt_iclass_rsr_lend,
  ICLASS_xt_iclass_wsr_lend,
  ICLASS_xt_iclass_xsr_lend,
  ICLASS_xt_iclass_rsr_lcount,
  ICLASS_xt_iclass_wsr_lcount,
  ICLASS_xt_iclass_xsr_lcount,
  ICLASS_xt_iclass_rsr_lbeg,
  ICLASS_xt_iclass_wsr_lbeg,
  ICLASS_xt_iclass_xsr_lbeg,
  ICLASS_xt_iclass_rsr_sar,
  ICLASS_xt_iclass_wsr_sar,
  ICLASS_xt_iclass_xsr_sar,
  ICLASS_xt_iclass_rsr_litbase,
  ICLASS_xt_iclass_wsr_litbase,
  ICLASS_xt_iclass_xsr_litbase,
  ICLASS_xt_iclass_rsr_176,
  ICLASS_xt_iclass_wsr_176,
  ICLASS_xt_iclass_rsr_208,
  ICLASS_xt_iclass_rsr_ps,
  ICLASS_xt_iclass_wsr_ps,
  ICLASS_xt_iclass_xsr_ps,
  ICLASS_xt_iclass_rsr_epc1,
  ICLASS_xt_iclass_wsr_epc1,
  ICLASS_xt_iclass_xsr_epc1,
  ICLASS_xt_iclass_rsr_excsave1,
  ICLASS_xt_iclass_wsr_excsave1,
  ICLASS_xt_iclass_xsr_excsave1,
  ICLASS_xt_iclass_rsr_epc2,
  ICLASS_xt_iclass_wsr_epc2,
  ICLASS_xt_iclass_xsr_epc2,
  ICLASS_xt_iclass_rsr_excsave2,
  ICLASS_xt_iclass_wsr_excsave2,
  ICLASS_xt_iclass_xsr_excsave2,
  ICLASS_xt_iclass_rsr_eps2,
  ICLASS_xt_iclass_wsr_eps2,
  ICLASS_xt_iclass_xsr_eps2,
  ICLASS_xt_iclass_rsr_excvaddr,
  ICLASS_xt_iclass_wsr_excvaddr,
  ICLASS_xt_iclass_xsr_excvaddr,
  ICLASS_xt_iclass_rsr_depc,
  ICLASS_xt_iclass_wsr_depc,
  ICLASS_xt_iclass_xsr_depc,
  ICLASS_xt_iclass_rsr_exccause,
  ICLASS_xt_iclass_wsr_exccause,
  ICLASS_xt_iclass_xsr_exccause,
  ICLASS_xt_iclass_rsr_misc0,
  ICLASS_xt_iclass_wsr_misc0,
  ICLASS_xt_iclass_xsr_misc0,
  ICLASS_xt_iclass_rsr_misc1,
  ICLASS_xt_iclass_wsr_misc1,
  ICLASS_xt_iclass_xsr_misc1,
  ICLASS_xt_iclass_rsr_prid,
  ICLASS_xt_iclass_rsr_vecbase,
  ICLASS_xt_iclass_wsr_vecbase,
  ICLASS_xt_iclass_xsr_vecbase,
  ICLASS_xt_mul16,
  ICLASS_xt_mul32,
  ICLASS_xt_iclass_rfi,
  ICLASS_xt_iclass_wait,
  ICLASS_xt_iclass_rsr_interrupt,
  ICLASS_xt_iclass_wsr_intset,
  ICLASS_xt_iclass_wsr_intclear,
  ICLASS_xt_iclass_rsr_intenable,
  ICLASS_xt_iclass_wsr_intenable,
  ICLASS_xt_iclass_xsr_intenable,
  ICLASS_xt_iclass_break,
  ICLASS_xt_iclass_break_n,
  ICLASS_xt_iclass_rsr_debugcause,
  ICLASS_xt_iclass_wsr_debugcause,
  ICLASS_xt_iclass_xsr_debugcause,
  ICLASS_xt_iclass_rsr_icount,
  ICLASS_xt_iclass_wsr_icount,
  ICLASS_xt_iclass_xsr_icount,
  ICLASS_xt_iclass_rsr_icountlevel,
  ICLASS_xt_iclass_wsr_icountlevel,
  ICLASS_xt_iclass_xsr_icountlevel,
  ICLASS_xt_iclass_rsr_ddr,
  ICLASS_xt_iclass_wsr_ddr,
  ICLASS_xt_iclass_xsr_ddr,
  ICLASS_xt_iclass_rfdo,
  ICLASS_xt_iclass_rfdd,
  ICLASS_xt_iclass_bbool1,
  ICLASS_xt_iclass_bbool4,
  ICLASS_xt_iclass_bbool8,
  ICLASS_xt_iclass_bbranch,
  ICLASS_xt_iclass_bmove,
  ICLASS_xt_iclass_RSR_BR,
  ICLASS_xt_iclass_WSR_BR,
  ICLASS_xt_iclass_XSR_BR,
  ICLASS_xt_iclass_rsr_ccount,
  ICLASS_xt_iclass_wsr_ccount,
  ICLASS_xt_iclass_xsr_ccount,
  ICLASS_xt_iclass_rsr_ccompare0,
  ICLASS_xt_iclass_wsr_ccompare0,
  ICLASS_xt_iclass_xsr_ccompare0,
  ICLASS_xt_iclass_rsr_ccompare1,
  ICLASS_xt_iclass_wsr_ccompare1,
  ICLASS_xt_iclass_xsr_ccompare1,
  ICLASS_xt_iclass_icache,
  ICLASS_xt_iclass_icache_inv,
  ICLASS_xt_iclass_licx,
  ICLASS_xt_iclass_sicx,
  ICLASS_xt_iclass_dcache,
  ICLASS_xt_iclass_dcache_ind,
  ICLASS_xt_iclass_dcache_inv,
  ICLASS_xt_iclass_dpf,
  ICLASS_xt_iclass_sdct,
  ICLASS_xt_iclass_ldct,
  ICLASS_xt_iclass_wsr_ptevaddr,
  ICLASS_xt_iclass_rsr_ptevaddr,
  ICLASS_xt_iclass_xsr_ptevaddr,
  ICLASS_xt_iclass_rsr_rasid,
  ICLASS_xt_iclass_wsr_rasid,
  ICLASS_xt_iclass_xsr_rasid,
  ICLASS_xt_iclass_rsr_itlbcfg,
  ICLASS_xt_iclass_wsr_itlbcfg,
  ICLASS_xt_iclass_xsr_itlbcfg,
  ICLASS_xt_iclass_rsr_dtlbcfg,
  ICLASS_xt_iclass_wsr_dtlbcfg,
  ICLASS_xt_iclass_xsr_dtlbcfg,
  ICLASS_xt_iclass_idtlb,
  ICLASS_xt_iclass_rdtlb,
  ICLASS_xt_iclass_wdtlb,
  ICLASS_xt_iclass_iitlb,
  ICLASS_xt_iclass_ritlb,
  ICLASS_xt_iclass_witlb,
  ICLASS_xt_iclass_ldpte,
  ICLASS_xt_iclass_hwwitlba,
  ICLASS_xt_iclass_hwwdtlba,
  ICLASS_xt_iclass_rsr_cpenable,
  ICLASS_xt_iclass_wsr_cpenable,
  ICLASS_xt_iclass_xsr_cpenable,
  ICLASS_xt_iclass_clamp,
  ICLASS_xt_iclass_minmax,
  ICLASS_xt_iclass_nsa,
  ICLASS_xt_iclass_sx,
  ICLASS_xt_iclass_l32ai,
  ICLASS_xt_iclass_s32ri,
  ICLASS_xt_iclass_s32c1i,
  ICLASS_xt_iclass_rsr_scompare1,
  ICLASS_xt_iclass_wsr_scompare1,
  ICLASS_xt_iclass_xsr_scompare1,
  ICLASS_xt_iclass_rsr_atomctl,
  ICLASS_xt_iclass_wsr_atomctl,
  ICLASS_xt_iclass_xsr_atomctl,
  ICLASS_xt_iclass_rer,
  ICLASS_xt_iclass_wer,
  ICLASS_rur_ae_ovf_sar,
  ICLASS_wur_ae_ovf_sar,
  ICLASS_rur_ae_bithead,
  ICLASS_wur_ae_bithead,
  ICLASS_rur_ae_ts_fts_bu_bp,
  ICLASS_wur_ae_ts_fts_bu_bp,
  ICLASS_rur_ae_sd_no,
  ICLASS_wur_ae_sd_no,
  ICLASS_ae_iclass_rur_ae_overflow,
  ICLASS_ae_iclass_wur_ae_overflow,
  ICLASS_ae_iclass_rur_ae_sar,
  ICLASS_ae_iclass_wur_ae_sar,
  ICLASS_ae_iclass_rur_ae_bitptr,
  ICLASS_ae_iclass_wur_ae_bitptr,
  ICLASS_ae_iclass_rur_ae_bitsused,
  ICLASS_ae_iclass_wur_ae_bitsused,
  ICLASS_ae_iclass_rur_ae_tablesize,
  ICLASS_ae_iclass_wur_ae_tablesize,
  ICLASS_ae_iclass_rur_ae_first_ts,
  ICLASS_ae_iclass_wur_ae_first_ts,
  ICLASS_ae_iclass_rur_ae_nextoffset,
  ICLASS_ae_iclass_wur_ae_nextoffset,
  ICLASS_ae_iclass_rur_ae_searchdone,
  ICLASS_ae_iclass_wur_ae_searchdone,
  ICLASS_ae_iclass_lp16f_i,
  ICLASS_ae_iclass_lp16f_iu,
  ICLASS_ae_iclass_lp16f_x,
  ICLASS_ae_iclass_lp16f_xu,
  ICLASS_ae_iclass_lp24_i,
  ICLASS_ae_iclass_lp24_iu,
  ICLASS_ae_iclass_lp24_x,
  ICLASS_ae_iclass_lp24_xu,
  ICLASS_ae_iclass_lp24f_i,
  ICLASS_ae_iclass_lp24f_iu,
  ICLASS_ae_iclass_lp24f_x,
  ICLASS_ae_iclass_lp24f_xu,
  ICLASS_ae_iclass_lp16x2f_i,
  ICLASS_ae_iclass_lp16x2f_iu,
  ICLASS_ae_iclass_lp16x2f_x,
  ICLASS_ae_iclass_lp16x2f_xu,
  ICLASS_ae_iclass_lp24x2f_i,
  ICLASS_ae_iclass_lp24x2f_iu,
  ICLASS_ae_iclass_lp24x2f_x,
  ICLASS_ae_iclass_lp24x2f_xu,
  ICLASS_ae_iclass_lp24x2_i,
  ICLASS_ae_iclass_lp24x2_iu,
  ICLASS_ae_iclass_lp24x2_x,
  ICLASS_ae_iclass_lp24x2_xu,
  ICLASS_ae_iclass_sp16x2f_i,
  ICLASS_ae_iclass_sp16x2f_iu,
  ICLASS_ae_iclass_sp16x2f_x,
  ICLASS_ae_iclass_sp16x2f_xu,
  ICLASS_ae_iclass_sp24x2s_i,
  ICLASS_ae_iclass_sp24x2s_iu,
  ICLASS_ae_iclass_sp24x2s_x,
  ICLASS_ae_iclass_sp24x2s_xu,
  ICLASS_ae_iclass_sp24x2f_i,
  ICLASS_ae_iclass_sp24x2f_iu,
  ICLASS_ae_iclass_sp24x2f_x,
  ICLASS_ae_iclass_sp24x2f_xu,
  ICLASS_ae_iclass_sp16f_l_i,
  ICLASS_ae_iclass_sp16f_l_iu,
  ICLASS_ae_iclass_sp16f_l_x,
  ICLASS_ae_iclass_sp16f_l_xu,
  ICLASS_ae_iclass_sp24s_l_i,
  ICLASS_ae_iclass_sp24s_l_iu,
  ICLASS_ae_iclass_sp24s_l_x,
  ICLASS_ae_iclass_sp24s_l_xu,
  ICLASS_ae_iclass_sp24f_l_i,
  ICLASS_ae_iclass_sp24f_l_iu,
  ICLASS_ae_iclass_sp24f_l_x,
  ICLASS_ae_iclass_sp24f_l_xu,
  ICLASS_ae_iclass_lq56_i,
  ICLASS_ae_iclass_lq56_iu,
  ICLASS_ae_iclass_lq56_x,
  ICLASS_ae_iclass_lq56_xu,
  ICLASS_ae_iclass_lq32f_i,
  ICLASS_ae_iclass_lq32f_iu,
  ICLASS_ae_iclass_lq32f_x,
  ICLASS_ae_iclass_lq32f_xu,
  ICLASS_ae_iclass_sq56s_i,
  ICLASS_ae_iclass_sq56s_iu,
  ICLASS_ae_iclass_sq56s_x,
  ICLASS_ae_iclass_sq56s_xu,
  ICLASS_ae_iclass_sq32f_i,
  ICLASS_ae_iclass_sq32f_iu,
  ICLASS_ae_iclass_sq32f_x,
  ICLASS_ae_iclass_sq32f_xu,
  ICLASS_ae_iclass_zerop48,
  ICLASS_ae_iclass_movp48,
  ICLASS_ae_iclass_selp24_ll,
  ICLASS_ae_iclass_selp24_lh,
  ICLASS_ae_iclass_selp24_hl,
  ICLASS_ae_iclass_selp24_hh,
  ICLASS_ae_iclass_movtp24x2,
  ICLASS_ae_iclass_movfp24x2,
  ICLASS_ae_iclass_movtp48,
  ICLASS_ae_iclass_movfp48,
  ICLASS_ae_iclass_movpa24x2,
  ICLASS_ae_iclass_truncp24a32x2,
  ICLASS_ae_iclass_cvta32p24_l,
  ICLASS_ae_iclass_cvta32p24_h,
  ICLASS_ae_iclass_cvtp24a16x2_ll,
  ICLASS_ae_iclass_cvtp24a16x2_lh,
  ICLASS_ae_iclass_cvtp24a16x2_hl,
  ICLASS_ae_iclass_cvtp24a16x2_hh,
  ICLASS_ae_iclass_truncp24q48x2,
  ICLASS_ae_iclass_truncp16,
  ICLASS_ae_iclass_roundsp24q48sym,
  ICLASS_ae_iclass_roundsp24q48asym,
  ICLASS_ae_iclass_roundsp16q48sym,
  ICLASS_ae_iclass_roundsp16q48asym,
  ICLASS_ae_iclass_roundsp16sym,
  ICLASS_ae_iclass_roundsp16asym,
  ICLASS_ae_iclass_zeroq56,
  ICLASS_ae_iclass_movq56,
  ICLASS_ae_iclass_movtq56,
  ICLASS_ae_iclass_movfq56,
  ICLASS_ae_iclass_cvtq48a32s,
  ICLASS_ae_iclass_cvtq48p24s_l,
  ICLASS_ae_iclass_cvtq48p24s_h,
  ICLASS_ae_iclass_satq48s,
  ICLASS_ae_iclass_truncq32,
  ICLASS_ae_iclass_roundsq32sym,
  ICLASS_ae_iclass_roundsq32asym,
  ICLASS_ae_iclass_trunca32q48,
  ICLASS_ae_iclass_movap24s_l,
  ICLASS_ae_iclass_movap24s_h,
  ICLASS_ae_iclass_trunca16p24s_l,
  ICLASS_ae_iclass_trunca16p24s_h,
  ICLASS_ae_iclass_addp24,
  ICLASS_ae_iclass_subp24,
  ICLASS_ae_iclass_negp24,
  ICLASS_ae_iclass_absp24,
  ICLASS_ae_iclass_maxp24s,
  ICLASS_ae_iclass_minp24s,
  ICLASS_ae_iclass_maxbp24s,
  ICLASS_ae_iclass_minbp24s,
  ICLASS_ae_iclass_addsp24s,
  ICLASS_ae_iclass_subsp24s,
  ICLASS_ae_iclass_negsp24s,
  ICLASS_ae_iclass_abssp24s,
  ICLASS_ae_iclass_andp48,
  ICLASS_ae_iclass_nandp48,
  ICLASS_ae_iclass_orp48,
  ICLASS_ae_iclass_xorp48,
  ICLASS_ae_iclass_ltp24s,
  ICLASS_ae_iclass_lep24s,
  ICLASS_ae_iclass_eqp24,
  ICLASS_ae_iclass_addq56,
  ICLASS_ae_iclass_subq56,
  ICLASS_ae_iclass_negq56,
  ICLASS_ae_iclass_absq56,
  ICLASS_ae_iclass_maxq56s,
  ICLASS_ae_iclass_minq56s,
  ICLASS_ae_iclass_maxbq56s,
  ICLASS_ae_iclass_minbq56s,
  ICLASS_ae_iclass_addsq56s,
  ICLASS_ae_iclass_subsq56s,
  ICLASS_ae_iclass_negsq56s,
  ICLASS_ae_iclass_abssq56s,
  ICLASS_ae_iclass_andq56,
  ICLASS_ae_iclass_nandq56,
  ICLASS_ae_iclass_orq56,
  ICLASS_ae_iclass_xorq56,
  ICLASS_ae_iclass_sllip24,
  ICLASS_ae_iclass_srlip24,
  ICLASS_ae_iclass_sraip24,
  ICLASS_ae_iclass_sllsp24,
  ICLASS_ae_iclass_srlsp24,
  ICLASS_ae_iclass_srasp24,
  ICLASS_ae_iclass_sllisp24s,
  ICLASS_ae_iclass_sllssp24s,
  ICLASS_ae_iclass_slliq56,
  ICLASS_ae_iclass_srliq56,
  ICLASS_ae_iclass_sraiq56,
  ICLASS_ae_iclass_sllsq56,
  ICLASS_ae_iclass_srlsq56,
  ICLASS_ae_iclass_srasq56,
  ICLASS_ae_iclass_sllaq56,
  ICLASS_ae_iclass_srlaq56,
  ICLASS_ae_iclass_sraaq56,
  ICLASS_ae_iclass_sllisq56s,
  ICLASS_ae_iclass_sllssq56s,
  ICLASS_ae_iclass_sllasq56s,
  ICLASS_ae_iclass_ltq56s,
  ICLASS_ae_iclass_leq56s,
  ICLASS_ae_iclass_eqq56,
  ICLASS_ae_iclass_nsaq56s,
  ICLASS_ae_iclass_mulfs32p16s_ll,
  ICLASS_ae_iclass_mulfp24s_ll,
  ICLASS_ae_iclass_mulp24s_ll,
  ICLASS_ae_iclass_mulfs32p16s_lh,
  ICLASS_ae_iclass_mulfp24s_lh,
  ICLASS_ae_iclass_mulp24s_lh,
  ICLASS_ae_iclass_mulfs32p16s_hl,
  ICLASS_ae_iclass_mulfp24s_hl,
  ICLASS_ae_iclass_mulp24s_hl,
  ICLASS_ae_iclass_mulfs32p16s_hh,
  ICLASS_ae_iclass_mulfp24s_hh,
  ICLASS_ae_iclass_mulp24s_hh,
  ICLASS_ae_iclass_mulafs32p16s_ll,
  ICLASS_ae_iclass_mulafp24s_ll,
  ICLASS_ae_iclass_mulap24s_ll,
  ICLASS_ae_iclass_mulafs32p16s_lh,
  ICLASS_ae_iclass_mulafp24s_lh,
  ICLASS_ae_iclass_mulap24s_lh,
  ICLASS_ae_iclass_mulafs32p16s_hl,
  ICLASS_ae_iclass_mulafp24s_hl,
  ICLASS_ae_iclass_mulap24s_hl,
  ICLASS_ae_iclass_mulafs32p16s_hh,
  ICLASS_ae_iclass_mulafp24s_hh,
  ICLASS_ae_iclass_mulap24s_hh,
  ICLASS_ae_iclass_mulsfs32p16s_ll,
  ICLASS_ae_iclass_mulsfp24s_ll,
  ICLASS_ae_iclass_mulsp24s_ll,
  ICLASS_ae_iclass_mulsfs32p16s_lh,
  ICLASS_ae_iclass_mulsfp24s_lh,
  ICLASS_ae_iclass_mulsp24s_lh,
  ICLASS_ae_iclass_mulsfs32p16s_hl,
  ICLASS_ae_iclass_mulsfp24s_hl,
  ICLASS_ae_iclass_mulsp24s_hl,
  ICLASS_ae_iclass_mulsfs32p16s_hh,
  ICLASS_ae_iclass_mulsfp24s_hh,
  ICLASS_ae_iclass_mulsp24s_hh,
  ICLASS_ae_iclass_mulafs56p24s_ll,
  ICLASS_ae_iclass_mulas56p24s_ll,
  ICLASS_ae_iclass_mulafs56p24s_lh,
  ICLASS_ae_iclass_mulas56p24s_lh,
  ICLASS_ae_iclass_mulafs56p24s_hl,
  ICLASS_ae_iclass_mulas56p24s_hl,
  ICLASS_ae_iclass_mulafs56p24s_hh,
  ICLASS_ae_iclass_mulas56p24s_hh,
  ICLASS_ae_iclass_mulsfs56p24s_ll,
  ICLASS_ae_iclass_mulss56p24s_ll,
  ICLASS_ae_iclass_mulsfs56p24s_lh,
  ICLASS_ae_iclass_mulss56p24s_lh,
  ICLASS_ae_iclass_mulsfs56p24s_hl,
  ICLASS_ae_iclass_mulss56p24s_hl,
  ICLASS_ae_iclass_mulsfs56p24s_hh,
  ICLASS_ae_iclass_mulss56p24s_hh,
  ICLASS_ae_iclass_mulfq32sp16s_l,
  ICLASS_ae_iclass_mulfq32sp16s_h,
  ICLASS_ae_iclass_mulfq32sp16u_l,
  ICLASS_ae_iclass_mulfq32sp16u_h,
  ICLASS_ae_iclass_mulq32sp16s_l,
  ICLASS_ae_iclass_mulq32sp16s_h,
  ICLASS_ae_iclass_mulq32sp16u_l,
  ICLASS_ae_iclass_mulq32sp16u_h,
  ICLASS_ae_iclass_mulafq32sp16s_l,
  ICLASS_ae_iclass_mulafq32sp16s_h,
  ICLASS_ae_iclass_mulafq32sp16u_l,
  ICLASS_ae_iclass_mulafq32sp16u_h,
  ICLASS_ae_iclass_mulaq32sp16s_l,
  ICLASS_ae_iclass_mulaq32sp16s_h,
  ICLASS_ae_iclass_mulaq32sp16u_l,
  ICLASS_ae_iclass_mulaq32sp16u_h,
  ICLASS_ae_iclass_mulsfq32sp16s_l,
  ICLASS_ae_iclass_mulsfq32sp16s_h,
  ICLASS_ae_iclass_mulsfq32sp16u_l,
  ICLASS_ae_iclass_mulsfq32sp16u_h,
  ICLASS_ae_iclass_mulsq32sp16s_l,
  ICLASS_ae_iclass_mulsq32sp16s_h,
  ICLASS_ae_iclass_mulsq32sp16u_l,
  ICLASS_ae_iclass_mulsq32sp16u_h,
  ICLASS_ae_iclass_mulzaaq32sp16s_ll,
  ICLASS_ae_iclass_mulzaafq32sp16s_ll,
  ICLASS_ae_iclass_mulzaaq32sp16u_ll,
  ICLASS_ae_iclass_mulzaafq32sp16u_ll,
  ICLASS_ae_iclass_mulzaaq32sp16s_hh,
  ICLASS_ae_iclass_mulzaafq32sp16s_hh,
  ICLASS_ae_iclass_mulzaaq32sp16u_hh,
  ICLASS_ae_iclass_mulzaafq32sp16u_hh,
  ICLASS_ae_iclass_mulzaaq32sp16s_lh,
  ICLASS_ae_iclass_mulzaafq32sp16s_lh,
  ICLASS_ae_iclass_mulzaaq32sp16u_lh,
  ICLASS_ae_iclass_mulzaafq32sp16u_lh,
  ICLASS_ae_iclass_mulzasq32sp16s_ll,
  ICLASS_ae_iclass_mulzasfq32sp16s_ll,
  ICLASS_ae_iclass_mulzasq32sp16u_ll,
  ICLASS_ae_iclass_mulzasfq32sp16u_ll,
  ICLASS_ae_iclass_mulzasq32sp16s_hh,
  ICLASS_ae_iclass_mulzasfq32sp16s_hh,
  ICLASS_ae_iclass_mulzasq32sp16u_hh,
  ICLASS_ae_iclass_mulzasfq32sp16u_hh,
  ICLASS_ae_iclass_mulzasq32sp16s_lh,
  ICLASS_ae_iclass_mulzasfq32sp16s_lh,
  ICLASS_ae_iclass_mulzasq32sp16u_lh,
  ICLASS_ae_iclass_mulzasfq32sp16u_lh,
  ICLASS_ae_iclass_mulzsaq32sp16s_ll,
  ICLASS_ae_iclass_mulzsafq32sp16s_ll,
  ICLASS_ae_iclass_mulzsaq32sp16u_ll,
  ICLASS_ae_iclass_mulzsafq32sp16u_ll,
  ICLASS_ae_iclass_mulzsaq32sp16s_hh,
  ICLASS_ae_iclass_mulzsafq32sp16s_hh,
  ICLASS_ae_iclass_mulzsaq32sp16u_hh,
  ICLASS_ae_iclass_mulzsafq32sp16u_hh,
  ICLASS_ae_iclass_mulzsaq32sp16s_lh,
  ICLASS_ae_iclass_mulzsafq32sp16s_lh,
  ICLASS_ae_iclass_mulzsaq32sp16u_lh,
  ICLASS_ae_iclass_mulzsafq32sp16u_lh,
  ICLASS_ae_iclass_mulzssq32sp16s_ll,
  ICLASS_ae_iclass_mulzssfq32sp16s_ll,
  ICLASS_ae_iclass_mulzssq32sp16u_ll,
  ICLASS_ae_iclass_mulzssfq32sp16u_ll,
  ICLASS_ae_iclass_mulzssq32sp16s_hh,
  ICLASS_ae_iclass_mulzssfq32sp16s_hh,
  ICLASS_ae_iclass_mulzssq32sp16u_hh,
  ICLASS_ae_iclass_mulzssfq32sp16u_hh,
  ICLASS_ae_iclass_mulzssq32sp16s_lh,
  ICLASS_ae_iclass_mulzssfq32sp16s_lh,
  ICLASS_ae_iclass_mulzssq32sp16u_lh,
  ICLASS_ae_iclass_mulzssfq32sp16u_lh,
  ICLASS_ae_iclass_mulzaafp24s_hh_ll,
  ICLASS_ae_iclass_mulzaap24s_hh_ll,
  ICLASS_ae_iclass_mulzaafp24s_hl_lh,
  ICLASS_ae_iclass_mulzaap24s_hl_lh,
  ICLASS_ae_iclass_mulzasfp24s_hh_ll,
  ICLASS_ae_iclass_mulzasp24s_hh_ll,
  ICLASS_ae_iclass_mulzasfp24s_hl_lh,
  ICLASS_ae_iclass_mulzasp24s_hl_lh,
  ICLASS_ae_iclass_mulzsafp24s_hh_ll,
  ICLASS_ae_iclass_mulzsap24s_hh_ll,
  ICLASS_ae_iclass_mulzsafp24s_hl_lh,
  ICLASS_ae_iclass_mulzsap24s_hl_lh,
  ICLASS_ae_iclass_mulzssfp24s_hh_ll,
  ICLASS_ae_iclass_mulzssp24s_hh_ll,
  ICLASS_ae_iclass_mulzssfp24s_hl_lh,
  ICLASS_ae_iclass_mulzssp24s_hl_lh,
  ICLASS_ae_iclass_mulaafp24s_hh_ll,
  ICLASS_ae_iclass_mulaap24s_hh_ll,
  ICLASS_ae_iclass_mulaafp24s_hl_lh,
  ICLASS_ae_iclass_mulaap24s_hl_lh,
  ICLASS_ae_iclass_mulasfp24s_hh_ll,
  ICLASS_ae_iclass_mulasp24s_hh_ll,
  ICLASS_ae_iclass_mulasfp24s_hl_lh,
  ICLASS_ae_iclass_mulasp24s_hl_lh,
  ICLASS_ae_iclass_mulsafp24s_hh_ll,
  ICLASS_ae_iclass_mulsap24s_hh_ll,
  ICLASS_ae_iclass_mulsafp24s_hl_lh,
  ICLASS_ae_iclass_mulsap24s_hl_lh,
  ICLASS_ae_iclass_mulssfp24s_hh_ll,
  ICLASS_ae_iclass_mulssp24s_hh_ll,
  ICLASS_ae_iclass_mulssfp24s_hl_lh,
  ICLASS_ae_iclass_mulssp24s_hl_lh,
  ICLASS_ae_iclass_sha32,
  ICLASS_ae_iclass_vldl32t,
  ICLASS_ae_iclass_vldl16t,
  ICLASS_ae_iclass_vldl16c,
  ICLASS_ae_iclass_vldsht,
  ICLASS_ae_iclass_lb,
  ICLASS_ae_iclass_lbi,
  ICLASS_ae_iclass_lbk,
  ICLASS_ae_iclass_lbki,
  ICLASS_ae_iclass_db,
  ICLASS_ae_iclass_dbi,
  ICLASS_ae_iclass_vlel32t,
  ICLASS_ae_iclass_vlel16t,
  ICLASS_ae_iclass_sb,
  ICLASS_ae_iclass_sbi,
  ICLASS_ae_iclass_vles16c,
  ICLASS_ae_iclass_sbf
};


/*  Opcode encodings.  */

static void
Opcode_excw_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2080;
}

static void
Opcode_rfe_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3000;
}

static void
Opcode_rfde_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3200;
}

static void
Opcode_syscall_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x5000;
}

static void
Opcode_simcall_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x5100;
}

static void
Opcode_call12_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x35;
}

static void
Opcode_call8_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x25;
}

static void
Opcode_call4_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x15;
}

static void
Opcode_callx12_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf0;
}

static void
Opcode_callx8_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe0;
}

static void
Opcode_callx4_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xd0;
}

static void
Opcode_entry_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x36;
}

static void
Opcode_movsp_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1000;
}

static void
Opcode_rotw_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x408000;
}

static void
Opcode_retw_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x90;
}

static void
Opcode_retw_n_Slot_inst16b_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf01d;
}

static void
Opcode_rfwo_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3400;
}

static void
Opcode_rfwu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3500;
}

static void
Opcode_l32e_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x90000;
}

static void
Opcode_s32e_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x490000;
}

static void
Opcode_rsr_windowbase_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x34800;
}

static void
Opcode_wsr_windowbase_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x134800;
}

static void
Opcode_xsr_windowbase_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x614800;
}

static void
Opcode_rsr_windowstart_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x34900;
}

static void
Opcode_wsr_windowstart_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x134900;
}

static void
Opcode_xsr_windowstart_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x614900;
}

static void
Opcode_add_n_Slot_inst16a_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa;
}

static void
Opcode_addi_n_Slot_inst16a_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb;
}

static void
Opcode_beqz_n_Slot_inst16b_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x8c;
}

static void
Opcode_bnez_n_Slot_inst16b_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xcc;
}

static void
Opcode_ill_n_Slot_inst16b_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf06d;
}

static void
Opcode_l32i_n_Slot_inst16a_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x8;
}

static void
Opcode_mov_n_Slot_inst16b_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xd;
}

static void
Opcode_movi_n_Slot_inst16b_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc;
}

static void
Opcode_nop_n_Slot_inst16b_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf03d;
}

static void
Opcode_ret_n_Slot_inst16b_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf00d;
}

static void
Opcode_s32i_n_Slot_inst16a_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x9;
}

static void
Opcode_rur_threadptr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe30e70;
}

static void
Opcode_wur_threadptr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf3e700;
}

static void
Opcode_addi_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc002;
}

static void
Opcode_addi_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x200040;
}

static void
Opcode_addmi_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xd002;
}

static void
Opcode_addmi_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x200080;
}

static void
Opcode_add_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x800000;
}

static void
Opcode_add_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1b2000;
}

static void
Opcode_sub_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc00000;
}

static void
Opcode_sub_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ca000;
}

static void
Opcode_addx2_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x900000;
}

static void
Opcode_addx2_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1b4000;
}

static void
Opcode_addx4_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa00000;
}

static void
Opcode_addx4_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1b8000;
}

static void
Opcode_addx8_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb00000;
}

static void
Opcode_addx8_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1b3000;
}

static void
Opcode_subx2_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xd00000;
}

static void
Opcode_subx2_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1cc000;
}

static void
Opcode_subx4_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe00000;
}

static void
Opcode_subx4_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1cb000;
}

static void
Opcode_subx8_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf00000;
}

static void
Opcode_subx8_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1cd000;
}

static void
Opcode_and_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x100000;
}

static void
Opcode_and_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1b5000;
}

static void
Opcode_or_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x200000;
}

static void
Opcode_or_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e0000;
}

static void
Opcode_xor_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x300000;
}

static void
Opcode_xor_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ce000;
}

static void
Opcode_beqi_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x26;
}

static void
Opcode_beqi_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x300000;
}

static void
Opcode_bnei_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x66;
}

static void
Opcode_bnei_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x300003;
}

static void
Opcode_bgei_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe6;
}

static void
Opcode_bgei_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x300001;
}

static void
Opcode_blti_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa6;
}

static void
Opcode_blti_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x300004;
}

static void
Opcode_bbci_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6007;
}

static void
Opcode_bbci_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x200000;
}

static void
Opcode_bbsi_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe007;
}

static void
Opcode_bbsi_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x200020;
}

static void
Opcode_bgeui_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf6;
}

static void
Opcode_bgeui_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x300002;
}

static void
Opcode_bltui_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb6;
}

static void
Opcode_bltui_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x300008;
}

static void
Opcode_beq_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1007;
}

static void
Opcode_beq_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2000a0;
}

static void
Opcode_bne_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x9007;
}

static void
Opcode_bne_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x400000;
}

static void
Opcode_bge_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa007;
}

static void
Opcode_bge_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2000c0;
}

static void
Opcode_blt_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2007;
}

static void
Opcode_blt_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2000d0;
}

static void
Opcode_bgeu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb007;
}

static void
Opcode_bgeu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2000b0;
}

static void
Opcode_bltu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3007;
}

static void
Opcode_bltu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2000e0;
}

static void
Opcode_bany_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x8007;
}

static void
Opcode_bany_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x200060;
}

static void
Opcode_bnone_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x7;
}

static void
Opcode_bnone_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x400010;
}

static void
Opcode_ball_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x4007;
}

static void
Opcode_ball_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x200050;
}

static void
Opcode_bnall_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc007;
}

static void
Opcode_bnall_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2000f0;
}

static void
Opcode_bbc_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x5007;
}

static void
Opcode_bbc_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x200070;
}

static void
Opcode_bbs_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xd007;
}

static void
Opcode_bbs_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x200090;
}

static void
Opcode_beqz_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16;
}

static void
Opcode_beqz_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x180000;
}

static void
Opcode_bnez_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x56;
}

static void
Opcode_bnez_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x190000;
}

static void
Opcode_bgez_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xd6;
}

static void
Opcode_bgez_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x160000;
}

static void
Opcode_bltz_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x96;
}

static void
Opcode_bltz_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x170000;
}

static void
Opcode_call0_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x5;
}

static void
Opcode_callx0_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc0;
}

static void
Opcode_extui_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x40000;
}

static void
Opcode_extui_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x140000;
}

static void
Opcode_ill_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0;
}

static void
Opcode_j_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6;
}

static void
Opcode_j_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x100000;
}

static void
Opcode_jx_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa0;
}

static void
Opcode_jx_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ee031;
}

static void
Opcode_l16ui_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1002;
}

static void
Opcode_l16ui_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x400040;
}

static void
Opcode_l16si_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x9002;
}

static void
Opcode_l16si_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x400020;
}

static void
Opcode_l32i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2002;
}

static void
Opcode_l32i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x400080;
}

static void
Opcode_l32r_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1;
}

static void
Opcode_l32r_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x500000;
}

static void
Opcode_l8ui_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2;
}

static void
Opcode_l8ui_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x400030;
}

static void
Opcode_loop_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x8076;
}

static void
Opcode_loopnez_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x9076;
}

static void
Opcode_loopgtz_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa076;
}

static void
Opcode_movi_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa002;
}

static void
Opcode_movi_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1a0000;
}

static void
Opcode_moveqz_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x830000;
}

static void
Opcode_moveqz_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1be000;
}

static void
Opcode_movnez_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x930000;
}

static void
Opcode_movnez_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c8000;
}

static void
Opcode_movltz_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa30000;
}

static void
Opcode_movltz_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c4000;
}

static void
Opcode_movgez_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb30000;
}

static void
Opcode_movgez_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c2000;
}

static void
Opcode_neg_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x600000;
}

static void
Opcode_neg_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f1d00;
}

static void
Opcode_abs_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x600100;
}

static void
Opcode_abs_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f1c00;
}

static void
Opcode_nop_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20f0;
}

static void
Opcode_nop_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16105;
}

static void
Opcode_nop_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ee0b1;
}

static void
Opcode_ret_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x80;
}

static void
Opcode_s16i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x5002;
}

static void
Opcode_s16i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x400050;
}

static void
Opcode_s32i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6002;
}

static void
Opcode_s32i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x400060;
}

static void
Opcode_s8i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x4002;
}

static void
Opcode_s8i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x400070;
}

static void
Opcode_ssr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x400000;
}

static void
Opcode_ssr_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ee071;
}

static void
Opcode_ssl_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x401000;
}

static void
Opcode_ssl_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ee038;
}

static void
Opcode_ssa8l_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x402000;
}

static void
Opcode_ssa8l_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ee034;
}

static void
Opcode_ssa8b_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x403000;
}

static void
Opcode_ssa8b_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ee032;
}

static void
Opcode_ssai_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x404000;
}

static void
Opcode_ssai_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ef0a0;
}

static void
Opcode_sll_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa10000;
}

static void
Opcode_sll_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f5003;
}

static void
Opcode_src_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x810000;
}

static void
Opcode_src_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c7000;
}

static void
Opcode_srl_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x910000;
}

static void
Opcode_srl_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f1f00;
}

static void
Opcode_sra_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb10000;
}

static void
Opcode_sra_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f1e00;
}

static void
Opcode_slli_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10000;
}

static void
Opcode_slli_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c0000;
}

static void
Opcode_srai_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x210000;
}

static void
Opcode_srai_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1b0000;
}

static void
Opcode_srli_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x410000;
}

static void
Opcode_srli_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c9000;
}

static void
Opcode_memw_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20c0;
}

static void
Opcode_extw_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20d0;
}

static void
Opcode_isync_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2000;
}

static void
Opcode_rsync_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2010;
}

static void
Opcode_esync_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2020;
}

static void
Opcode_dsync_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2030;
}

static void
Opcode_rsil_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6000;
}

static void
Opcode_rsr_lend_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30100;
}

static void
Opcode_wsr_lend_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x130100;
}

static void
Opcode_xsr_lend_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x610100;
}

static void
Opcode_rsr_lcount_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30200;
}

static void
Opcode_wsr_lcount_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x130200;
}

static void
Opcode_xsr_lcount_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x610200;
}

static void
Opcode_rsr_lbeg_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30000;
}

static void
Opcode_wsr_lbeg_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x130000;
}

static void
Opcode_xsr_lbeg_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x610000;
}

static void
Opcode_rsr_sar_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30300;
}

static void
Opcode_wsr_sar_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x130300;
}

static void
Opcode_xsr_sar_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x610300;
}

static void
Opcode_rsr_litbase_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30500;
}

static void
Opcode_wsr_litbase_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x130500;
}

static void
Opcode_xsr_litbase_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x610500;
}

static void
Opcode_rsr_176_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3b000;
}

static void
Opcode_wsr_176_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13b000;
}

static void
Opcode_rsr_208_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3d000;
}

static void
Opcode_rsr_ps_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3e600;
}

static void
Opcode_wsr_ps_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13e600;
}

static void
Opcode_xsr_ps_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61e600;
}

static void
Opcode_rsr_epc1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3b100;
}

static void
Opcode_wsr_epc1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13b100;
}

static void
Opcode_xsr_epc1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61b100;
}

static void
Opcode_rsr_excsave1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3d100;
}

static void
Opcode_wsr_excsave1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13d100;
}

static void
Opcode_xsr_excsave1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61d100;
}

static void
Opcode_rsr_epc2_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3b200;
}

static void
Opcode_wsr_epc2_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13b200;
}

static void
Opcode_xsr_epc2_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61b200;
}

static void
Opcode_rsr_excsave2_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3d200;
}

static void
Opcode_wsr_excsave2_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13d200;
}

static void
Opcode_xsr_excsave2_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61d200;
}

static void
Opcode_rsr_eps2_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3c200;
}

static void
Opcode_wsr_eps2_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13c200;
}

static void
Opcode_xsr_eps2_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61c200;
}

static void
Opcode_rsr_excvaddr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3ee00;
}

static void
Opcode_wsr_excvaddr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13ee00;
}

static void
Opcode_xsr_excvaddr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61ee00;
}

static void
Opcode_rsr_depc_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3c000;
}

static void
Opcode_wsr_depc_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13c000;
}

static void
Opcode_xsr_depc_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61c000;
}

static void
Opcode_rsr_exccause_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3e800;
}

static void
Opcode_wsr_exccause_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13e800;
}

static void
Opcode_xsr_exccause_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61e800;
}

static void
Opcode_rsr_misc0_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3f400;
}

static void
Opcode_wsr_misc0_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13f400;
}

static void
Opcode_xsr_misc0_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61f400;
}

static void
Opcode_rsr_misc1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3f500;
}

static void
Opcode_wsr_misc1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13f500;
}

static void
Opcode_xsr_misc1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61f500;
}

static void
Opcode_rsr_prid_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3eb00;
}

static void
Opcode_rsr_vecbase_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3e700;
}

static void
Opcode_wsr_vecbase_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13e700;
}

static void
Opcode_xsr_vecbase_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61e700;
}

static void
Opcode_mul16u_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc10000;
}

static void
Opcode_mul16s_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xd10000;
}

static void
Opcode_mull_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x820000;
}

static void
Opcode_rfi_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3010;
}

static void
Opcode_waiti_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x7000;
}

static void
Opcode_rsr_interrupt_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3e200;
}

static void
Opcode_wsr_intset_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13e200;
}

static void
Opcode_wsr_intclear_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13e300;
}

static void
Opcode_rsr_intenable_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3e400;
}

static void
Opcode_wsr_intenable_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13e400;
}

static void
Opcode_xsr_intenable_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61e400;
}

static void
Opcode_break_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x4000;
}

static void
Opcode_break_n_Slot_inst16b_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf02d;
}

static void
Opcode_rsr_debugcause_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3e900;
}

static void
Opcode_wsr_debugcause_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13e900;
}

static void
Opcode_xsr_debugcause_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61e900;
}

static void
Opcode_rsr_icount_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3ec00;
}

static void
Opcode_wsr_icount_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13ec00;
}

static void
Opcode_xsr_icount_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61ec00;
}

static void
Opcode_rsr_icountlevel_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3ed00;
}

static void
Opcode_wsr_icountlevel_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13ed00;
}

static void
Opcode_xsr_icountlevel_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61ed00;
}

static void
Opcode_rsr_ddr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x36800;
}

static void
Opcode_wsr_ddr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x136800;
}

static void
Opcode_xsr_ddr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x616800;
}

static void
Opcode_rfdo_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf1e000;
}

static void
Opcode_rfdd_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf1e010;
}

static void
Opcode_andb_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20000;
}

static void
Opcode_andb_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1b6000;
}

static void
Opcode_andbc_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x120000;
}

static void
Opcode_andbc_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1b7000;
}

static void
Opcode_orb_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x220000;
}

static void
Opcode_orb_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c3000;
}

static void
Opcode_orbc_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x320000;
}

static void
Opcode_orbc_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c5000;
}

static void
Opcode_xorb_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x420000;
}

static void
Opcode_xorb_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1cf000;
}

static void
Opcode_any4_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x8000;
}

static void
Opcode_any4_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f2480;
}

static void
Opcode_all4_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x9000;
}

static void
Opcode_all4_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f2800;
}

static void
Opcode_any8_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa000;
}

static void
Opcode_any8_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ef060;
}

static void
Opcode_all8_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb000;
}

static void
Opcode_all8_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ef020;
}

static void
Opcode_bf_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x76;
}

static void
Opcode_bf_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x300005;
}

static void
Opcode_bt_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1076;
}

static void
Opcode_bt_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x300006;
}

static void
Opcode_movf_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc30000;
}

static void
Opcode_movf_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1bf000;
}

static void
Opcode_movt_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xd30000;
}

static void
Opcode_movt_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d0000;
}

static void
Opcode_rsr_br_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30400;
}

static void
Opcode_wsr_br_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x130400;
}

static void
Opcode_xsr_br_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x610400;
}

static void
Opcode_rsr_ccount_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3ea00;
}

static void
Opcode_wsr_ccount_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13ea00;
}

static void
Opcode_xsr_ccount_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61ea00;
}

static void
Opcode_rsr_ccompare0_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3f000;
}

static void
Opcode_wsr_ccompare0_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13f000;
}

static void
Opcode_xsr_ccompare0_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61f000;
}

static void
Opcode_rsr_ccompare1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3f100;
}

static void
Opcode_wsr_ccompare1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13f100;
}

static void
Opcode_xsr_ccompare1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61f100;
}

static void
Opcode_ipf_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x70c2;
}

static void
Opcode_ihi_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x70e2;
}

static void
Opcode_iii_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x70f2;
}

static void
Opcode_lict_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf10000;
}

static void
Opcode_licw_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf12000;
}

static void
Opcode_sict_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf11000;
}

static void
Opcode_sicw_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf13000;
}

static void
Opcode_dhwb_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x7042;
}

static void
Opcode_dhwbi_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x7052;
}

static void
Opcode_diwb_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x47082;
}

static void
Opcode_diwbi_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x57082;
}

static void
Opcode_dhi_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x7062;
}

static void
Opcode_dii_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x7072;
}

static void
Opcode_dpfr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x7002;
}

static void
Opcode_dpfw_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x7012;
}

static void
Opcode_dpfro_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x7022;
}

static void
Opcode_dpfwo_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x7032;
}

static void
Opcode_sdct_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf19000;
}

static void
Opcode_ldct_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf18000;
}

static void
Opcode_wsr_ptevaddr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x135300;
}

static void
Opcode_rsr_ptevaddr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x35300;
}

static void
Opcode_xsr_ptevaddr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x615300;
}

static void
Opcode_rsr_rasid_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x35a00;
}

static void
Opcode_wsr_rasid_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x135a00;
}

static void
Opcode_xsr_rasid_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x615a00;
}

static void
Opcode_rsr_itlbcfg_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x35b00;
}

static void
Opcode_wsr_itlbcfg_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x135b00;
}

static void
Opcode_xsr_itlbcfg_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x615b00;
}

static void
Opcode_rsr_dtlbcfg_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x35c00;
}

static void
Opcode_wsr_dtlbcfg_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x135c00;
}

static void
Opcode_xsr_dtlbcfg_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x615c00;
}

static void
Opcode_idtlb_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50c000;
}

static void
Opcode_pdtlb_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50d000;
}

static void
Opcode_rdtlb0_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50b000;
}

static void
Opcode_rdtlb1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50f000;
}

static void
Opcode_wdtlb_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50e000;
}

static void
Opcode_iitlb_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x504000;
}

static void
Opcode_pitlb_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x505000;
}

static void
Opcode_ritlb0_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x503000;
}

static void
Opcode_ritlb1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x507000;
}

static void
Opcode_witlb_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x506000;
}

static void
Opcode_ldpte_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf1f000;
}

static void
Opcode_hwwitlba_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x501000;
}

static void
Opcode_hwwdtlba_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x509000;
}

static void
Opcode_rsr_cpenable_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3e000;
}

static void
Opcode_wsr_cpenable_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x13e000;
}

static void
Opcode_xsr_cpenable_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x61e000;
}

static void
Opcode_clamps_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x330000;
}

static void
Opcode_clamps_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1b9000;
}

static void
Opcode_min_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x430000;
}

static void
Opcode_min_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1bb000;
}

static void
Opcode_max_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x530000;
}

static void
Opcode_max_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ba000;
}

static void
Opcode_minu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x630000;
}

static void
Opcode_minu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1bd000;
}

static void
Opcode_maxu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x730000;
}

static void
Opcode_maxu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1bc000;
}

static void
Opcode_nsa_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x40e000;
}

static void
Opcode_nsau_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x40f000;
}

static void
Opcode_sext_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x230000;
}

static void
Opcode_sext_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c6000;
}

static void
Opcode_l32ai_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb002;
}

static void
Opcode_s32ri_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf002;
}

static void
Opcode_s32c1i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe002;
}

static void
Opcode_rsr_scompare1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30c00;
}

static void
Opcode_wsr_scompare1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x130c00;
}

static void
Opcode_xsr_scompare1_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x610c00;
}

static void
Opcode_rsr_atomctl_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x36300;
}

static void
Opcode_wsr_atomctl_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x136300;
}

static void
Opcode_xsr_atomctl_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x616300;
}

static void
Opcode_rer_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x406000;
}

static void
Opcode_wer_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x407000;
}

static void
Opcode_rur_ae_ovf_sar_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe30f00;
}

static void
Opcode_wur_ae_ovf_sar_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf3f000;
}

static void
Opcode_rur_ae_bithead_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe30f10;
}

static void
Opcode_wur_ae_bithead_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf3f100;
}

static void
Opcode_rur_ae_ts_fts_bu_bp_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe30f20;
}

static void
Opcode_wur_ae_ts_fts_bu_bp_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf3f200;
}

static void
Opcode_rur_ae_sd_no_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe30f30;
}

static void
Opcode_wur_ae_sd_no_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf3f300;
}

static void
Opcode_rur_ae_overflow_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90804;
}

static void
Opcode_wur_ae_overflow_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xca0004;
}

static void
Opcode_rur_ae_sar_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90904;
}

static void
Opcode_wur_ae_sar_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xca1004;
}

static void
Opcode_rur_ae_bitptr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90a04;
}

static void
Opcode_wur_ae_bitptr_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xca2004;
}

static void
Opcode_rur_ae_bitsused_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90b04;
}

static void
Opcode_wur_ae_bitsused_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xca3004;
}

static void
Opcode_rur_ae_tablesize_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90c04;
}

static void
Opcode_wur_ae_tablesize_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xca4004;
}

static void
Opcode_rur_ae_first_ts_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90d04;
}

static void
Opcode_wur_ae_first_ts_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xca5004;
}

static void
Opcode_rur_ae_nextoffset_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90e04;
}

static void
Opcode_wur_ae_nextoffset_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xca6004;
}

static void
Opcode_rur_ae_searchdone_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90f04;
}

static void
Opcode_wur_ae_searchdone_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xca7004;
}

static void
Opcode_ae_lp16f_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d1080;
}

static void
Opcode_ae_lp16f_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa50004;
}

static void
Opcode_ae_lp16f_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d2080;
}

static void
Opcode_ae_lp16f_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa90004;
}

static void
Opcode_ae_lp16f_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d3000;
}

static void
Opcode_ae_lp16f_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xac0004;
}

static void
Opcode_ae_lp16f_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d3080;
}

static void
Opcode_ae_lp16f_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xaf0004;
}

static void
Opcode_ae_lp24_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d6080;
}

static void
Opcode_ae_lp24_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa58004;
}

static void
Opcode_ae_lp24_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d7000;
}

static void
Opcode_ae_lp24_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa98004;
}

static void
Opcode_ae_lp24_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d7080;
}

static void
Opcode_ae_lp24_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xac8004;
}

static void
Opcode_ae_lp24_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d8080;
}

static void
Opcode_ae_lp24_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xaf8004;
}

static void
Opcode_ae_lp24f_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d9000;
}

static void
Opcode_ae_lp24f_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa60004;
}

static void
Opcode_ae_lp24f_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1da000;
}

static void
Opcode_ae_lp24f_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xaa0004;
}

static void
Opcode_ae_lp24f_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1dc000;
}

static void
Opcode_ae_lp24f_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xad0004;
}

static void
Opcode_ae_lp24f_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d9080;
}

static void
Opcode_ae_lp24f_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb00004;
}

static void
Opcode_ae_lp16x2f_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d4080;
}

static void
Opcode_ae_lp16x2f_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa68004;
}

static void
Opcode_ae_lp16x2f_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d5000;
}

static void
Opcode_ae_lp16x2f_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xaa8004;
}

static void
Opcode_ae_lp16x2f_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d6000;
}

static void
Opcode_ae_lp16x2f_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xad8004;
}

static void
Opcode_ae_lp16x2f_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d5080;
}

static void
Opcode_ae_lp16x2f_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb08004;
}

static void
Opcode_ae_lp24x2f_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1dd000;
}

static void
Opcode_ae_lp24x2f_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa70004;
}

static void
Opcode_ae_lp24x2f_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1de000;
}

static void
Opcode_ae_lp24x2f_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xab0004;
}

static void
Opcode_ae_lp24x2f_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1dd080;
}

static void
Opcode_ae_lp24x2f_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xae0004;
}

static void
Opcode_ae_lp24x2f_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1de080;
}

static void
Opcode_ae_lp24x2f_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb10004;
}

static void
Opcode_ae_lp24x2_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1da080;
}

static void
Opcode_ae_lp24x2_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa78004;
}

static void
Opcode_ae_lp24x2_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1db000;
}

static void
Opcode_ae_lp24x2_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xab8004;
}

static void
Opcode_ae_lp24x2_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1db080;
}

static void
Opcode_ae_lp24x2_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xae8004;
}

static void
Opcode_ae_lp24x2_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1dc080;
}

static void
Opcode_ae_lp24x2_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb18004;
}

static void
Opcode_ae_sp16x2f_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e8000;
}

static void
Opcode_ae_sp16x2f_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb20004;
}

static void
Opcode_ae_sp16x2f_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f0000;
}

static void
Opcode_ae_sp16x2f_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb50004;
}

static void
Opcode_ae_sp16x2f_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e1080;
}

static void
Opcode_ae_sp16x2f_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb80004;
}

static void
Opcode_ae_sp16x2f_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e2080;
}

static void
Opcode_ae_sp16x2f_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xbb0004;
}

static void
Opcode_ae_sp24x2s_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ec000;
}

static void
Opcode_ae_sp24x2s_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb28004;
}

static void
Opcode_ae_sp24x2s_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e9080;
}

static void
Opcode_ae_sp24x2s_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb58004;
}

static void
Opcode_ae_sp24x2s_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ea080;
}

static void
Opcode_ae_sp24x2s_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb88004;
}

static void
Opcode_ae_sp24x2s_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1eb000;
}

static void
Opcode_ae_sp24x2s_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xbb8004;
}

static void
Opcode_ae_sp24x2f_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e7080;
}

static void
Opcode_ae_sp24x2f_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb30004;
}

static void
Opcode_ae_sp24x2f_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e8080;
}

static void
Opcode_ae_sp24x2f_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb60004;
}

static void
Opcode_ae_sp24x2f_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e9000;
}

static void
Opcode_ae_sp24x2f_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb90004;
}

static void
Opcode_ae_sp24x2f_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ea000;
}

static void
Opcode_ae_sp24x2f_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xbc0004;
}

static void
Opcode_ae_sp16f_l_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1df080;
}

static void
Opcode_ae_sp16f_l_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb38004;
}

static void
Opcode_ae_sp16f_l_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e1000;
}

static void
Opcode_ae_sp16f_l_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb68004;
}

static void
Opcode_ae_sp16f_l_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e2000;
}

static void
Opcode_ae_sp16f_l_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb98004;
}

static void
Opcode_ae_sp16f_l_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e4000;
}

static void
Opcode_ae_sp16f_l_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xbc8004;
}

static void
Opcode_ae_sp24s_l_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e6000;
}

static void
Opcode_ae_sp24s_l_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb40004;
}

static void
Opcode_ae_sp24s_l_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e5080;
}

static void
Opcode_ae_sp24s_l_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb70004;
}

static void
Opcode_ae_sp24s_l_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e6080;
}

static void
Opcode_ae_sp24s_l_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xba0004;
}

static void
Opcode_ae_sp24s_l_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e7000;
}

static void
Opcode_ae_sp24s_l_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xbd0004;
}

static void
Opcode_ae_sp24f_l_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e3000;
}

static void
Opcode_ae_sp24f_l_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb48004;
}

static void
Opcode_ae_sp24f_l_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e3080;
}

static void
Opcode_ae_sp24f_l_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xb78004;
}

static void
Opcode_ae_sp24f_l_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e4080;
}

static void
Opcode_ae_sp24f_l_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xba8004;
}

static void
Opcode_ae_sp24f_l_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1e5000;
}

static void
Opcode_ae_sp24f_l_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xbd8004;
}

static void
Opcode_ae_lq56_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ed030;
}

static void
Opcode_ae_lq56_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc10004;
}

static void
Opcode_ae_lq56_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ee010;
}

static void
Opcode_ae_lq56_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc12004;
}

static void
Opcode_ae_lq56_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ee020;
}

static void
Opcode_ae_lq56_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc20004;
}

static void
Opcode_ae_lq56_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ef000;
}

static void
Opcode_ae_lq56_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc22004;
}

static void
Opcode_ae_lq32f_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ed000;
}

static void
Opcode_ae_lq32f_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc11004;
}

static void
Opcode_ae_lq32f_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ee000;
}

static void
Opcode_ae_lq32f_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc13004;
}

static void
Opcode_ae_lq32f_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ed010;
}

static void
Opcode_ae_lq32f_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc21004;
}

static void
Opcode_ae_lq32f_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ed020;
}

static void
Opcode_ae_lq32f_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc23004;
}

static void
Opcode_ae_sq56s_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f0080;
}

static void
Opcode_ae_sq56s_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc30004;
}

static void
Opcode_ae_sq56s_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f00c0;
}

static void
Opcode_ae_sq56s_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc38004;
}

static void
Opcode_ae_sq56s_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f3000;
}

static void
Opcode_ae_sq56s_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc40004;
}

static void
Opcode_ae_sq56s_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f3040;
}

static void
Opcode_ae_sq56s_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc48004;
}

static void
Opcode_ae_sq32f_i_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ec080;
}

static void
Opcode_ae_sq32f_i_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc34004;
}

static void
Opcode_ae_sq32f_iu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ec0c0;
}

static void
Opcode_ae_sq32f_iu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc3c004;
}

static void
Opcode_ae_sq32f_x_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f4000;
}

static void
Opcode_ae_sq32f_x_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc44004;
}

static void
Opcode_ae_sq32f_xu_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f8000;
}

static void
Opcode_ae_sq32f_xu_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc4c004;
}

static void
Opcode_ae_zerop48_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16b88;
}

static void
Opcode_ae_movp48_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16808;
}

static void
Opcode_ae_movp48_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f2400;
}

static void
Opcode_ae_movp48_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90004;
}

static void
Opcode_ae_selp24_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10780;
}

static void
Opcode_ae_selp24_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10708;
}

static void
Opcode_ae_selp24_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10688;
}

static void
Opcode_ae_selp24_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10700;
}

static void
Opcode_ae_movtp24x2_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c200;
}

static void
Opcode_ae_movfp24x2_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c004;
}

static void
Opcode_ae_movtp48_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10480;
}

static void
Opcode_ae_movfp48_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10400;
}

static void
Opcode_ae_movpa24x2_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1df000;
}

static void
Opcode_ae_movpa24x2_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc00004;
}

static void
Opcode_ae_truncp24a32x2_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1eb080;
}

static void
Opcode_ae_truncp24a32x2_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc08004;
}

static void
Opcode_ae_cvta32p24_l_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f3081;
}

static void
Opcode_ae_cvta32p24_l_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xcb0004;
}

static void
Opcode_ae_cvta32p24_h_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f3080;
}

static void
Opcode_ae_cvta32p24_h_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xcb8004;
}

static void
Opcode_ae_cvtp24a16x2_ll_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d8000;
}

static void
Opcode_ae_cvtp24a16x2_ll_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xbe0004;
}

static void
Opcode_ae_cvtp24a16x2_lh_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d4000;
}

static void
Opcode_ae_cvtp24a16x2_lh_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xbe8004;
}

static void
Opcode_ae_cvtp24a16x2_hl_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d2000;
}

static void
Opcode_ae_cvtp24a16x2_hl_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xbf0004;
}

static void
Opcode_ae_cvtp24a16x2_hh_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1d1000;
}

static void
Opcode_ae_cvtp24a16x2_hh_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xbf8004;
}

static void
Opcode_ae_truncp24q48x2_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x51000;
}

static void
Opcode_ae_truncp16_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16b08;
}

static void
Opcode_ae_roundsp24q48sym_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16e48;
}

static void
Opcode_ae_roundsp24q48asym_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16e28;
}

static void
Opcode_ae_roundsp16q48sym_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16e18;
}

static void
Opcode_ae_roundsp16q48asym_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16e08;
}

static void
Opcode_ae_roundsp16sym_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16908;
}

static void
Opcode_ae_roundsp16asym_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16888;
}

static void
Opcode_ae_zeroq56_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16085;
}

static void
Opcode_ae_movq56_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16007;
}

static void
Opcode_ae_movq56_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f2500;
}

static void
Opcode_ae_movq56_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90414;
}

static void
Opcode_ae_movtq56_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f6000;
}

static void
Opcode_ae_movtq56_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe50014;
}

static void
Opcode_ae_movfq56_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f5000;
}

static void
Opcode_ae_movfq56_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe60014;
}

static void
Opcode_ae_cvtq48a32s_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1ee030;
}

static void
Opcode_ae_cvtq48a32s_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe72034;
}

static void
Opcode_ae_cvtq48p24s_l_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16006;
}

static void
Opcode_ae_cvtq48p24s_h_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16005;
}

static void
Opcode_ae_satq48s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50139;
}

static void
Opcode_ae_truncq32_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16047;
}

static void
Opcode_ae_roundsq32sym_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16027;
}

static void
Opcode_ae_roundsq32asym_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16017;
}

static void
Opcode_ae_trunca32q48_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f3086;
}

static void
Opcode_ae_trunca32q48_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe70014;
}

static void
Opcode_ae_movap24s_l_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f3084;
}

static void
Opcode_ae_movap24s_l_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc70004;
}

static void
Opcode_ae_movap24s_h_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f3082;
}

static void
Opcode_ae_movap24s_h_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc78004;
}

static void
Opcode_ae_trunca16p24s_l_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f3083;
}

static void
Opcode_ae_trunca16p24s_l_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc80004;
}

static void
Opcode_ae_trunca16p24s_h_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f3088;
}

static void
Opcode_ae_trunca16p24s_h_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc88004;
}

static void
Opcode_ae_addp24_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10500;
}

static void
Opcode_ae_subp24_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10788;
}

static void
Opcode_ae_negp24_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c600;
}

static void
Opcode_ae_absp24_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c480;
}

static void
Opcode_ae_maxp24s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10580;
}

static void
Opcode_ae_minp24s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10588;
}

static void
Opcode_ae_maxbp24s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10000;
}

static void
Opcode_ae_minbp24s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10200;
}

static void
Opcode_ae_addsp24s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10600;
}

static void
Opcode_ae_subsp24s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c400;
}

static void
Opcode_ae_negsp24s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c488;
}

static void
Opcode_ae_abssp24s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c500;
}

static void
Opcode_ae_andp48_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10508;
}

static void
Opcode_ae_nandp48_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10608;
}

static void
Opcode_ae_orp48_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x10680;
}

static void
Opcode_ae_xorp48_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c408;
}

static void
Opcode_ae_ltp24s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c002;
}

static void
Opcode_ae_lep24s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c001;
}

static void
Opcode_ae_eqp24_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1c000;
}

static void
Opcode_ae_addq56_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x52000;
}

static void
Opcode_ae_subq56_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50035;
}

static void
Opcode_ae_negq56_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x5003c;
}

static void
Opcode_ae_absq56_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50039;
}

static void
Opcode_ae_maxq56s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50032;
}

static void
Opcode_ae_minq56s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50034;
}

static void
Opcode_ae_maxbq56s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50000;
}

static void
Opcode_ae_minbq56s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50010;
}

static void
Opcode_ae_addsq56s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50030;
}

static void
Opcode_ae_subsq56s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50036;
}

static void
Opcode_ae_negsq56s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x500b9;
}

static void
Opcode_ae_abssq56s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x5003a;
}

static void
Opcode_ae_andq56_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50031;
}

static void
Opcode_ae_nandq56_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50038;
}

static void
Opcode_ae_orq56_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50033;
}

static void
Opcode_ae_xorq56_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50037;
}

static void
Opcode_ae_sllip24_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x14000;
}

static void
Opcode_ae_srlip24_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x15000;
}

static void
Opcode_ae_sraip24_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x14800;
}

static void
Opcode_ae_sllsp24_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16a08;
}

static void
Opcode_ae_srlsp24_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16a88;
}

static void
Opcode_ae_srasp24_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16988;
}

static void
Opcode_ae_sllisp24s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x18000;
}

static void
Opcode_ae_sllssp24s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16c08;
}

static void
Opcode_ae_slliq56_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f1000;
}

static void
Opcode_ae_slliq56_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc50004;
}

static void
Opcode_ae_srliq56_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f1800;
}

static void
Opcode_ae_srliq56_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc50404;
}

static void
Opcode_ae_sraiq56_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f1400;
}

static void
Opcode_ae_sraiq56_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc50804;
}

static void
Opcode_ae_sllsq56_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f2600;
}

static void
Opcode_ae_sllsq56_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90014;
}

static void
Opcode_ae_srlsq56_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f2504;
}

static void
Opcode_ae_srlsq56_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90114;
}

static void
Opcode_ae_srasq56_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f2502;
}

static void
Opcode_ae_srasq56_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90214;
}

static void
Opcode_ae_sllaq56_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f5001;
}

static void
Opcode_ae_sllaq56_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe10014;
}

static void
Opcode_ae_srlaq56_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f5008;
}

static void
Opcode_ae_srlaq56_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe20014;
}

static void
Opcode_ae_sraaq56_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f5004;
}

static void
Opcode_ae_sraaq56_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe30014;
}

static void
Opcode_ae_sllisq56s_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f2000;
}

static void
Opcode_ae_sllisq56s_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc50c04;
}

static void
Opcode_ae_sllssq56s_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f2501;
}

static void
Opcode_ae_sllssq56s_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc90314;
}

static void
Opcode_ae_sllasq56s_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f5002;
}

static void
Opcode_ae_sllasq56s_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe40014;
}

static void
Opcode_ae_ltq56s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50800;
}

static void
Opcode_ae_leq56s_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50040;
}

static void
Opcode_ae_eqq56_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x50020;
}

static void
Opcode_ae_nsaq56s_Slot_ae_slot0_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1f3085;
}

static void
Opcode_ae_nsaq56s_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe74014;
}

static void
Opcode_ae_mulfs32p16s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60101;
}

static void
Opcode_ae_mulfp24s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6008b;
}

static void
Opcode_ae_mulp24s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60180;
}

static void
Opcode_ae_mulfs32p16s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6008f;
}

static void
Opcode_ae_mulfp24s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6008c;
}

static void
Opcode_ae_mulp24s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60108;
}

static void
Opcode_ae_mulfs32p16s_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6008e;
}

static void
Opcode_ae_mulfp24s_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6008a;
}

static void
Opcode_ae_mulp24s_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60104;
}

static void
Opcode_ae_mulfs32p16s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6008d;
}

static void
Opcode_ae_mulfp24s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60089;
}

static void
Opcode_ae_mulp24s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60102;
}

static void
Opcode_ae_mulafs32p16s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60006;
}

static void
Opcode_ae_mulafp24s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x64000;
}

static void
Opcode_ae_mulap24s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6000f;
}

static void
Opcode_ae_mulafs32p16s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60005;
}

static void
Opcode_ae_mulafp24s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60100;
}

static void
Opcode_ae_mulap24s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6000e;
}

static void
Opcode_ae_mulafs32p16s_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60003;
}

static void
Opcode_ae_mulafp24s_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60080;
}

static void
Opcode_ae_mulap24s_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6000d;
}

static void
Opcode_ae_mulafs32p16s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x68000;
}

static void
Opcode_ae_mulafp24s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60008;
}

static void
Opcode_ae_mulap24s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6000b;
}

static void
Opcode_ae_mulsfs32p16s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60181;
}

static void
Opcode_ae_mulsfp24s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6010b;
}

static void
Opcode_ae_mulsp24s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60189;
}

static void
Opcode_ae_mulsfs32p16s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6010f;
}

static void
Opcode_ae_mulsfp24s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6010c;
}

static void
Opcode_ae_mulsp24s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60187;
}

static void
Opcode_ae_mulsfs32p16s_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6010e;
}

static void
Opcode_ae_mulsfp24s_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6010a;
}

static void
Opcode_ae_mulsp24s_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60186;
}

static void
Opcode_ae_mulsfs32p16s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6010d;
}

static void
Opcode_ae_mulsfp24s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60109;
}

static void
Opcode_ae_mulsp24s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60185;
}

static void
Opcode_ae_mulafs56p24s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6000c;
}

static void
Opcode_ae_mulas56p24s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60088;
}

static void
Opcode_ae_mulafs56p24s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6000a;
}

static void
Opcode_ae_mulas56p24s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60084;
}

static void
Opcode_ae_mulafs56p24s_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60009;
}

static void
Opcode_ae_mulas56p24s_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60082;
}

static void
Opcode_ae_mulafs56p24s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60007;
}

static void
Opcode_ae_mulas56p24s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60081;
}

static void
Opcode_ae_mulsfs56p24s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60183;
}

static void
Opcode_ae_mulss56p24s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6018d;
}

static void
Opcode_ae_mulsfs56p24s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60188;
}

static void
Opcode_ae_mulss56p24s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6018b;
}

static void
Opcode_ae_mulsfs56p24s_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60184;
}

static void
Opcode_ae_mulss56p24s_hl_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6018c;
}

static void
Opcode_ae_mulsfs56p24s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60182;
}

static void
Opcode_ae_mulss56p24s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6018a;
}

static void
Opcode_ae_mulfq32sp16s_l_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x15807;
}

static void
Opcode_ae_mulfq32sp16s_h_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x15806;
}

static void
Opcode_ae_mulfq32sp16u_l_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1580a;
}

static void
Opcode_ae_mulfq32sp16u_h_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x15809;
}

static void
Opcode_ae_mulq32sp16s_l_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1580b;
}

static void
Opcode_ae_mulq32sp16s_h_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1580c;
}

static void
Opcode_ae_mulq32sp16u_l_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1580e;
}

static void
Opcode_ae_mulq32sp16u_h_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1580d;
}

static void
Opcode_ae_mulafq32sp16s_l_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x15800;
}

static void
Opcode_ae_mulafq32sp16s_h_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16000;
}

static void
Opcode_ae_mulafq32sp16u_l_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x15802;
}

static void
Opcode_ae_mulafq32sp16u_h_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x15801;
}

static void
Opcode_ae_mulaq32sp16s_l_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x15808;
}

static void
Opcode_ae_mulaq32sp16s_h_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x15804;
}

static void
Opcode_ae_mulaq32sp16u_l_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x15805;
}

static void
Opcode_ae_mulaq32sp16u_h_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x15803;
}

static void
Opcode_ae_mulsfq32sp16s_l_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16001;
}

static void
Opcode_ae_mulsfq32sp16s_h_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x1580f;
}

static void
Opcode_ae_mulsfq32sp16u_l_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16004;
}

static void
Opcode_ae_mulsfq32sp16u_h_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16002;
}

static void
Opcode_ae_mulsq32sp16s_l_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16800;
}

static void
Opcode_ae_mulsq32sp16s_h_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16008;
}

static void
Opcode_ae_mulsq32sp16u_l_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x16003;
}

static void
Opcode_ae_mulsq32sp16u_h_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x17000;
}

static void
Opcode_ae_mulzaaq32sp16s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20007;
}

static void
Opcode_ae_mulzaafq32sp16s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20002;
}

static void
Opcode_ae_mulzaaq32sp16u_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2000c;
}

static void
Opcode_ae_mulzaafq32sp16u_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20003;
}

static void
Opcode_ae_mulzaaq32sp16s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20005;
}

static void
Opcode_ae_mulzaafq32sp16s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20000;
}

static void
Opcode_ae_mulzaaq32sp16u_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20009;
}

static void
Opcode_ae_mulzaafq32sp16u_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20004;
}

static void
Opcode_ae_mulzaaq32sp16s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20006;
}

static void
Opcode_ae_mulzaafq32sp16s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20001;
}

static void
Opcode_ae_mulzaaq32sp16u_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2000a;
}

static void
Opcode_ae_mulzaafq32sp16u_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x20008;
}

static void
Opcode_ae_mulzasq32sp16s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30008;
}

static void
Opcode_ae_mulzasfq32sp16s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2000e;
}

static void
Opcode_ae_mulzasq32sp16u_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30006;
}

static void
Opcode_ae_mulzasfq32sp16u_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30001;
}

static void
Opcode_ae_mulzasq32sp16s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30002;
}

static void
Opcode_ae_mulzasfq32sp16s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2000b;
}

static void
Opcode_ae_mulzasq32sp16u_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30003;
}

static void
Opcode_ae_mulzasfq32sp16u_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2000f;
}

static void
Opcode_ae_mulzasq32sp16s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30004;
}

static void
Opcode_ae_mulzasfq32sp16s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x2000d;
}

static void
Opcode_ae_mulzasq32sp16u_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30005;
}

static void
Opcode_ae_mulzasfq32sp16u_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30000;
}

static void
Opcode_ae_mulzsaq32sp16s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x40000;
}

static void
Opcode_ae_mulzsafq32sp16s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3000a;
}

static void
Opcode_ae_mulzsaq32sp16u_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x40004;
}

static void
Opcode_ae_mulzsafq32sp16u_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3000d;
}

static void
Opcode_ae_mulzsaq32sp16s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3000e;
}

static void
Opcode_ae_mulzsafq32sp16s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30007;
}

static void
Opcode_ae_mulzsaq32sp16u_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x40001;
}

static void
Opcode_ae_mulzsafq32sp16u_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3000c;
}

static void
Opcode_ae_mulzsaq32sp16s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3000f;
}

static void
Opcode_ae_mulzsafq32sp16s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x30009;
}

static void
Opcode_ae_mulzsaq32sp16u_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x40002;
}

static void
Opcode_ae_mulzsafq32sp16u_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x3000b;
}

static void
Opcode_ae_mulzssq32sp16s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x4000b;
}

static void
Opcode_ae_mulzssfq32sp16s_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x40005;
}

static void
Opcode_ae_mulzssq32sp16u_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x4000f;
}

static void
Opcode_ae_mulzssfq32sp16u_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x40009;
}

static void
Opcode_ae_mulzssq32sp16s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x4000a;
}

static void
Opcode_ae_mulzssfq32sp16s_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x40008;
}

static void
Opcode_ae_mulzssq32sp16u_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x4000d;
}

static void
Opcode_ae_mulzssfq32sp16u_hh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x40006;
}

static void
Opcode_ae_mulzssq32sp16s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x4000c;
}

static void
Opcode_ae_mulzssfq32sp16s_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x40003;
}

static void
Opcode_ae_mulzssq32sp16u_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x4000e;
}

static void
Opcode_ae_mulzssfq32sp16u_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x40007;
}

static void
Opcode_ae_mulzaafp24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x64004;
}

static void
Opcode_ae_mulzaap24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x64080;
}

static void
Opcode_ae_mulzaafp24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x64008;
}

static void
Opcode_ae_mulzaap24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x64100;
}

static void
Opcode_ae_mulzasfp24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x64003;
}

static void
Opcode_ae_mulzasp24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x64006;
}

static void
Opcode_ae_mulzasfp24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x64005;
}

static void
Opcode_ae_mulzasp24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x64007;
}

static void
Opcode_ae_mulzsafp24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x64009;
}

static void
Opcode_ae_mulzsap24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6400c;
}

static void
Opcode_ae_mulzsafp24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6400a;
}

static void
Opcode_ae_mulzsap24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6400b;
}

static void
Opcode_ae_mulzssfp24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6400d;
}

static void
Opcode_ae_mulzssp24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6400f;
}

static void
Opcode_ae_mulzssfp24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6400e;
}

static void
Opcode_ae_mulzssp24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x64081;
}

static void
Opcode_ae_mulaafp24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60000;
}

static void
Opcode_ae_mulaap24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60002;
}

static void
Opcode_ae_mulaafp24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60001;
}

static void
Opcode_ae_mulaap24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60004;
}

static void
Opcode_ae_mulasfp24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60083;
}

static void
Opcode_ae_mulasp24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60086;
}

static void
Opcode_ae_mulasfp24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60085;
}

static void
Opcode_ae_mulasp24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60087;
}

static void
Opcode_ae_mulsafp24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60103;
}

static void
Opcode_ae_mulsap24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60106;
}

static void
Opcode_ae_mulsafp24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60105;
}

static void
Opcode_ae_mulsap24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x60107;
}

static void
Opcode_ae_mulssfp24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6018e;
}

static void
Opcode_ae_mulssp24s_hh_ll_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x64001;
}

static void
Opcode_ae_mulssfp24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x6018f;
}

static void
Opcode_ae_mulssp24s_hl_lh_Slot_ae_slot1_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0x64002;
}

static void
Opcode_ae_sha32_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe00014;
}

static void
Opcode_ae_vldl32t_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa00004;
}

static void
Opcode_ae_vldl16t_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa10004;
}

static void
Opcode_ae_vldl16c_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe7e014;
}

static void
Opcode_ae_vldsht_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xca8004;
}

static void
Opcode_ae_lb_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xc60004;
}

static void
Opcode_ae_lbi_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe00024;
}

static void
Opcode_ae_lbk_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa20004;
}

static void
Opcode_ae_lbki_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe00004;
}

static void
Opcode_ae_db_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf01004;
}

static void
Opcode_ae_dbi_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf02004;
}

static void
Opcode_ae_vlel32t_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa30004;
}

static void
Opcode_ae_vlel16t_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xa40004;
}

static void
Opcode_ae_sb_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf11004;
}

static void
Opcode_ae_sbi_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xf00004;
}

static void
Opcode_ae_vles16c_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe7c014;
}

static void
Opcode_ae_sbf_Slot_inst_encode (xtensa_insnbuf slotbuf)
{
  slotbuf[0] = 0xe7d014;
}

xtensa_opcode_encode_fn Opcode_excw_encode_fns[] = {
  Opcode_excw_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rfe_encode_fns[] = {
  Opcode_rfe_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rfde_encode_fns[] = {
  Opcode_rfde_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_syscall_encode_fns[] = {
  Opcode_syscall_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_simcall_encode_fns[] = {
  Opcode_simcall_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_call12_encode_fns[] = {
  Opcode_call12_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_call8_encode_fns[] = {
  Opcode_call8_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_call4_encode_fns[] = {
  Opcode_call4_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_callx12_encode_fns[] = {
  Opcode_callx12_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_callx8_encode_fns[] = {
  Opcode_callx8_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_callx4_encode_fns[] = {
  Opcode_callx4_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_entry_encode_fns[] = {
  Opcode_entry_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_movsp_encode_fns[] = {
  Opcode_movsp_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rotw_encode_fns[] = {
  Opcode_rotw_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_retw_encode_fns[] = {
  Opcode_retw_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_retw_n_encode_fns[] = {
  0, 0, Opcode_retw_n_Slot_inst16b_encode, 0, 0
};

xtensa_opcode_encode_fn Opcode_rfwo_encode_fns[] = {
  Opcode_rfwo_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rfwu_encode_fns[] = {
  Opcode_rfwu_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_l32e_encode_fns[] = {
  Opcode_l32e_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_s32e_encode_fns[] = {
  Opcode_s32e_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_windowbase_encode_fns[] = {
  Opcode_rsr_windowbase_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_windowbase_encode_fns[] = {
  Opcode_wsr_windowbase_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_windowbase_encode_fns[] = {
  Opcode_xsr_windowbase_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_windowstart_encode_fns[] = {
  Opcode_rsr_windowstart_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_windowstart_encode_fns[] = {
  Opcode_wsr_windowstart_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_windowstart_encode_fns[] = {
  Opcode_xsr_windowstart_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_add_n_encode_fns[] = {
  0, Opcode_add_n_Slot_inst16a_encode, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_addi_n_encode_fns[] = {
  0, Opcode_addi_n_Slot_inst16a_encode, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_beqz_n_encode_fns[] = {
  0, 0, Opcode_beqz_n_Slot_inst16b_encode, 0, 0
};

xtensa_opcode_encode_fn Opcode_bnez_n_encode_fns[] = {
  0, 0, Opcode_bnez_n_Slot_inst16b_encode, 0, 0
};

xtensa_opcode_encode_fn Opcode_ill_n_encode_fns[] = {
  0, 0, Opcode_ill_n_Slot_inst16b_encode, 0, 0
};

xtensa_opcode_encode_fn Opcode_l32i_n_encode_fns[] = {
  0, Opcode_l32i_n_Slot_inst16a_encode, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_mov_n_encode_fns[] = {
  0, 0, Opcode_mov_n_Slot_inst16b_encode, 0, 0
};

xtensa_opcode_encode_fn Opcode_movi_n_encode_fns[] = {
  0, 0, Opcode_movi_n_Slot_inst16b_encode, 0, 0
};

xtensa_opcode_encode_fn Opcode_nop_n_encode_fns[] = {
  0, 0, Opcode_nop_n_Slot_inst16b_encode, 0, 0
};

xtensa_opcode_encode_fn Opcode_ret_n_encode_fns[] = {
  0, 0, Opcode_ret_n_Slot_inst16b_encode, 0, 0
};

xtensa_opcode_encode_fn Opcode_s32i_n_encode_fns[] = {
  0, Opcode_s32i_n_Slot_inst16a_encode, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rur_threadptr_encode_fns[] = {
  Opcode_rur_threadptr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wur_threadptr_encode_fns[] = {
  Opcode_wur_threadptr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_addi_encode_fns[] = {
  Opcode_addi_Slot_inst_encode, 0, 0, 0, Opcode_addi_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_addmi_encode_fns[] = {
  Opcode_addmi_Slot_inst_encode, 0, 0, 0, Opcode_addmi_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_add_encode_fns[] = {
  Opcode_add_Slot_inst_encode, 0, 0, 0, Opcode_add_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_sub_encode_fns[] = {
  Opcode_sub_Slot_inst_encode, 0, 0, 0, Opcode_sub_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_addx2_encode_fns[] = {
  Opcode_addx2_Slot_inst_encode, 0, 0, 0, Opcode_addx2_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_addx4_encode_fns[] = {
  Opcode_addx4_Slot_inst_encode, 0, 0, 0, Opcode_addx4_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_addx8_encode_fns[] = {
  Opcode_addx8_Slot_inst_encode, 0, 0, 0, Opcode_addx8_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_subx2_encode_fns[] = {
  Opcode_subx2_Slot_inst_encode, 0, 0, 0, Opcode_subx2_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_subx4_encode_fns[] = {
  Opcode_subx4_Slot_inst_encode, 0, 0, 0, Opcode_subx4_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_subx8_encode_fns[] = {
  Opcode_subx8_Slot_inst_encode, 0, 0, 0, Opcode_subx8_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_and_encode_fns[] = {
  Opcode_and_Slot_inst_encode, 0, 0, 0, Opcode_and_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_or_encode_fns[] = {
  Opcode_or_Slot_inst_encode, 0, 0, 0, Opcode_or_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_xor_encode_fns[] = {
  Opcode_xor_Slot_inst_encode, 0, 0, 0, Opcode_xor_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_beqi_encode_fns[] = {
  Opcode_beqi_Slot_inst_encode, 0, 0, 0, Opcode_beqi_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bnei_encode_fns[] = {
  Opcode_bnei_Slot_inst_encode, 0, 0, 0, Opcode_bnei_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bgei_encode_fns[] = {
  Opcode_bgei_Slot_inst_encode, 0, 0, 0, Opcode_bgei_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_blti_encode_fns[] = {
  Opcode_blti_Slot_inst_encode, 0, 0, 0, Opcode_blti_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bbci_encode_fns[] = {
  Opcode_bbci_Slot_inst_encode, 0, 0, 0, Opcode_bbci_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bbsi_encode_fns[] = {
  Opcode_bbsi_Slot_inst_encode, 0, 0, 0, Opcode_bbsi_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bgeui_encode_fns[] = {
  Opcode_bgeui_Slot_inst_encode, 0, 0, 0, Opcode_bgeui_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bltui_encode_fns[] = {
  Opcode_bltui_Slot_inst_encode, 0, 0, 0, Opcode_bltui_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_beq_encode_fns[] = {
  Opcode_beq_Slot_inst_encode, 0, 0, 0, Opcode_beq_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bne_encode_fns[] = {
  Opcode_bne_Slot_inst_encode, 0, 0, 0, Opcode_bne_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bge_encode_fns[] = {
  Opcode_bge_Slot_inst_encode, 0, 0, 0, Opcode_bge_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_blt_encode_fns[] = {
  Opcode_blt_Slot_inst_encode, 0, 0, 0, Opcode_blt_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bgeu_encode_fns[] = {
  Opcode_bgeu_Slot_inst_encode, 0, 0, 0, Opcode_bgeu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bltu_encode_fns[] = {
  Opcode_bltu_Slot_inst_encode, 0, 0, 0, Opcode_bltu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bany_encode_fns[] = {
  Opcode_bany_Slot_inst_encode, 0, 0, 0, Opcode_bany_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bnone_encode_fns[] = {
  Opcode_bnone_Slot_inst_encode, 0, 0, 0, Opcode_bnone_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ball_encode_fns[] = {
  Opcode_ball_Slot_inst_encode, 0, 0, 0, Opcode_ball_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bnall_encode_fns[] = {
  Opcode_bnall_Slot_inst_encode, 0, 0, 0, Opcode_bnall_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bbc_encode_fns[] = {
  Opcode_bbc_Slot_inst_encode, 0, 0, 0, Opcode_bbc_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bbs_encode_fns[] = {
  Opcode_bbs_Slot_inst_encode, 0, 0, 0, Opcode_bbs_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_beqz_encode_fns[] = {
  Opcode_beqz_Slot_inst_encode, 0, 0, 0, Opcode_beqz_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bnez_encode_fns[] = {
  Opcode_bnez_Slot_inst_encode, 0, 0, 0, Opcode_bnez_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bgez_encode_fns[] = {
  Opcode_bgez_Slot_inst_encode, 0, 0, 0, Opcode_bgez_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bltz_encode_fns[] = {
  Opcode_bltz_Slot_inst_encode, 0, 0, 0, Opcode_bltz_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_call0_encode_fns[] = {
  Opcode_call0_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_callx0_encode_fns[] = {
  Opcode_callx0_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_extui_encode_fns[] = {
  Opcode_extui_Slot_inst_encode, 0, 0, 0, Opcode_extui_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ill_encode_fns[] = {
  Opcode_ill_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_j_encode_fns[] = {
  Opcode_j_Slot_inst_encode, 0, 0, 0, Opcode_j_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_jx_encode_fns[] = {
  Opcode_jx_Slot_inst_encode, 0, 0, 0, Opcode_jx_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_l16ui_encode_fns[] = {
  Opcode_l16ui_Slot_inst_encode, 0, 0, 0, Opcode_l16ui_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_l16si_encode_fns[] = {
  Opcode_l16si_Slot_inst_encode, 0, 0, 0, Opcode_l16si_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_l32i_encode_fns[] = {
  Opcode_l32i_Slot_inst_encode, 0, 0, 0, Opcode_l32i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_l32r_encode_fns[] = {
  Opcode_l32r_Slot_inst_encode, 0, 0, 0, Opcode_l32r_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_l8ui_encode_fns[] = {
  Opcode_l8ui_Slot_inst_encode, 0, 0, 0, Opcode_l8ui_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_loop_encode_fns[] = {
  Opcode_loop_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_loopnez_encode_fns[] = {
  Opcode_loopnez_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_loopgtz_encode_fns[] = {
  Opcode_loopgtz_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_movi_encode_fns[] = {
  Opcode_movi_Slot_inst_encode, 0, 0, 0, Opcode_movi_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_moveqz_encode_fns[] = {
  Opcode_moveqz_Slot_inst_encode, 0, 0, 0, Opcode_moveqz_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_movnez_encode_fns[] = {
  Opcode_movnez_Slot_inst_encode, 0, 0, 0, Opcode_movnez_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_movltz_encode_fns[] = {
  Opcode_movltz_Slot_inst_encode, 0, 0, 0, Opcode_movltz_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_movgez_encode_fns[] = {
  Opcode_movgez_Slot_inst_encode, 0, 0, 0, Opcode_movgez_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_neg_encode_fns[] = {
  Opcode_neg_Slot_inst_encode, 0, 0, 0, Opcode_neg_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_abs_encode_fns[] = {
  Opcode_abs_Slot_inst_encode, 0, 0, 0, Opcode_abs_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_nop_encode_fns[] = {
  Opcode_nop_Slot_inst_encode, 0, 0, Opcode_nop_Slot_ae_slot1_encode, Opcode_nop_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ret_encode_fns[] = {
  Opcode_ret_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_s16i_encode_fns[] = {
  Opcode_s16i_Slot_inst_encode, 0, 0, 0, Opcode_s16i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_s32i_encode_fns[] = {
  Opcode_s32i_Slot_inst_encode, 0, 0, 0, Opcode_s32i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_s8i_encode_fns[] = {
  Opcode_s8i_Slot_inst_encode, 0, 0, 0, Opcode_s8i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ssr_encode_fns[] = {
  Opcode_ssr_Slot_inst_encode, 0, 0, 0, Opcode_ssr_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ssl_encode_fns[] = {
  Opcode_ssl_Slot_inst_encode, 0, 0, 0, Opcode_ssl_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ssa8l_encode_fns[] = {
  Opcode_ssa8l_Slot_inst_encode, 0, 0, 0, Opcode_ssa8l_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ssa8b_encode_fns[] = {
  Opcode_ssa8b_Slot_inst_encode, 0, 0, 0, Opcode_ssa8b_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ssai_encode_fns[] = {
  Opcode_ssai_Slot_inst_encode, 0, 0, 0, Opcode_ssai_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_sll_encode_fns[] = {
  Opcode_sll_Slot_inst_encode, 0, 0, 0, Opcode_sll_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_src_encode_fns[] = {
  Opcode_src_Slot_inst_encode, 0, 0, 0, Opcode_src_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_srl_encode_fns[] = {
  Opcode_srl_Slot_inst_encode, 0, 0, 0, Opcode_srl_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_sra_encode_fns[] = {
  Opcode_sra_Slot_inst_encode, 0, 0, 0, Opcode_sra_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_slli_encode_fns[] = {
  Opcode_slli_Slot_inst_encode, 0, 0, 0, Opcode_slli_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_srai_encode_fns[] = {
  Opcode_srai_Slot_inst_encode, 0, 0, 0, Opcode_srai_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_srli_encode_fns[] = {
  Opcode_srli_Slot_inst_encode, 0, 0, 0, Opcode_srli_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_memw_encode_fns[] = {
  Opcode_memw_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_extw_encode_fns[] = {
  Opcode_extw_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_isync_encode_fns[] = {
  Opcode_isync_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsync_encode_fns[] = {
  Opcode_rsync_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_esync_encode_fns[] = {
  Opcode_esync_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_dsync_encode_fns[] = {
  Opcode_dsync_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsil_encode_fns[] = {
  Opcode_rsil_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_lend_encode_fns[] = {
  Opcode_rsr_lend_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_lend_encode_fns[] = {
  Opcode_wsr_lend_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_lend_encode_fns[] = {
  Opcode_xsr_lend_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_lcount_encode_fns[] = {
  Opcode_rsr_lcount_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_lcount_encode_fns[] = {
  Opcode_wsr_lcount_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_lcount_encode_fns[] = {
  Opcode_xsr_lcount_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_lbeg_encode_fns[] = {
  Opcode_rsr_lbeg_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_lbeg_encode_fns[] = {
  Opcode_wsr_lbeg_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_lbeg_encode_fns[] = {
  Opcode_xsr_lbeg_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_sar_encode_fns[] = {
  Opcode_rsr_sar_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_sar_encode_fns[] = {
  Opcode_wsr_sar_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_sar_encode_fns[] = {
  Opcode_xsr_sar_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_litbase_encode_fns[] = {
  Opcode_rsr_litbase_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_litbase_encode_fns[] = {
  Opcode_wsr_litbase_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_litbase_encode_fns[] = {
  Opcode_xsr_litbase_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_176_encode_fns[] = {
  Opcode_rsr_176_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_176_encode_fns[] = {
  Opcode_wsr_176_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_208_encode_fns[] = {
  Opcode_rsr_208_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_ps_encode_fns[] = {
  Opcode_rsr_ps_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_ps_encode_fns[] = {
  Opcode_wsr_ps_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_ps_encode_fns[] = {
  Opcode_xsr_ps_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_epc1_encode_fns[] = {
  Opcode_rsr_epc1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_epc1_encode_fns[] = {
  Opcode_wsr_epc1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_epc1_encode_fns[] = {
  Opcode_xsr_epc1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_excsave1_encode_fns[] = {
  Opcode_rsr_excsave1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_excsave1_encode_fns[] = {
  Opcode_wsr_excsave1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_excsave1_encode_fns[] = {
  Opcode_xsr_excsave1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_epc2_encode_fns[] = {
  Opcode_rsr_epc2_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_epc2_encode_fns[] = {
  Opcode_wsr_epc2_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_epc2_encode_fns[] = {
  Opcode_xsr_epc2_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_excsave2_encode_fns[] = {
  Opcode_rsr_excsave2_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_excsave2_encode_fns[] = {
  Opcode_wsr_excsave2_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_excsave2_encode_fns[] = {
  Opcode_xsr_excsave2_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_eps2_encode_fns[] = {
  Opcode_rsr_eps2_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_eps2_encode_fns[] = {
  Opcode_wsr_eps2_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_eps2_encode_fns[] = {
  Opcode_xsr_eps2_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_excvaddr_encode_fns[] = {
  Opcode_rsr_excvaddr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_excvaddr_encode_fns[] = {
  Opcode_wsr_excvaddr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_excvaddr_encode_fns[] = {
  Opcode_xsr_excvaddr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_depc_encode_fns[] = {
  Opcode_rsr_depc_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_depc_encode_fns[] = {
  Opcode_wsr_depc_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_depc_encode_fns[] = {
  Opcode_xsr_depc_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_exccause_encode_fns[] = {
  Opcode_rsr_exccause_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_exccause_encode_fns[] = {
  Opcode_wsr_exccause_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_exccause_encode_fns[] = {
  Opcode_xsr_exccause_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_misc0_encode_fns[] = {
  Opcode_rsr_misc0_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_misc0_encode_fns[] = {
  Opcode_wsr_misc0_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_misc0_encode_fns[] = {
  Opcode_xsr_misc0_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_misc1_encode_fns[] = {
  Opcode_rsr_misc1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_misc1_encode_fns[] = {
  Opcode_wsr_misc1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_misc1_encode_fns[] = {
  Opcode_xsr_misc1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_prid_encode_fns[] = {
  Opcode_rsr_prid_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_vecbase_encode_fns[] = {
  Opcode_rsr_vecbase_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_vecbase_encode_fns[] = {
  Opcode_wsr_vecbase_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_vecbase_encode_fns[] = {
  Opcode_xsr_vecbase_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_mul16u_encode_fns[] = {
  Opcode_mul16u_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_mul16s_encode_fns[] = {
  Opcode_mul16s_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_mull_encode_fns[] = {
  Opcode_mull_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rfi_encode_fns[] = {
  Opcode_rfi_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_waiti_encode_fns[] = {
  Opcode_waiti_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_interrupt_encode_fns[] = {
  Opcode_rsr_interrupt_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_intset_encode_fns[] = {
  Opcode_wsr_intset_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_intclear_encode_fns[] = {
  Opcode_wsr_intclear_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_intenable_encode_fns[] = {
  Opcode_rsr_intenable_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_intenable_encode_fns[] = {
  Opcode_wsr_intenable_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_intenable_encode_fns[] = {
  Opcode_xsr_intenable_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_break_encode_fns[] = {
  Opcode_break_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_break_n_encode_fns[] = {
  0, 0, Opcode_break_n_Slot_inst16b_encode, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_debugcause_encode_fns[] = {
  Opcode_rsr_debugcause_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_debugcause_encode_fns[] = {
  Opcode_wsr_debugcause_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_debugcause_encode_fns[] = {
  Opcode_xsr_debugcause_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_icount_encode_fns[] = {
  Opcode_rsr_icount_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_icount_encode_fns[] = {
  Opcode_wsr_icount_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_icount_encode_fns[] = {
  Opcode_xsr_icount_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_icountlevel_encode_fns[] = {
  Opcode_rsr_icountlevel_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_icountlevel_encode_fns[] = {
  Opcode_wsr_icountlevel_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_icountlevel_encode_fns[] = {
  Opcode_xsr_icountlevel_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_ddr_encode_fns[] = {
  Opcode_rsr_ddr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_ddr_encode_fns[] = {
  Opcode_wsr_ddr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_ddr_encode_fns[] = {
  Opcode_xsr_ddr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rfdo_encode_fns[] = {
  Opcode_rfdo_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rfdd_encode_fns[] = {
  Opcode_rfdd_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_andb_encode_fns[] = {
  Opcode_andb_Slot_inst_encode, 0, 0, 0, Opcode_andb_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_andbc_encode_fns[] = {
  Opcode_andbc_Slot_inst_encode, 0, 0, 0, Opcode_andbc_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_orb_encode_fns[] = {
  Opcode_orb_Slot_inst_encode, 0, 0, 0, Opcode_orb_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_orbc_encode_fns[] = {
  Opcode_orbc_Slot_inst_encode, 0, 0, 0, Opcode_orbc_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_xorb_encode_fns[] = {
  Opcode_xorb_Slot_inst_encode, 0, 0, 0, Opcode_xorb_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_any4_encode_fns[] = {
  Opcode_any4_Slot_inst_encode, 0, 0, 0, Opcode_any4_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_all4_encode_fns[] = {
  Opcode_all4_Slot_inst_encode, 0, 0, 0, Opcode_all4_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_any8_encode_fns[] = {
  Opcode_any8_Slot_inst_encode, 0, 0, 0, Opcode_any8_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_all8_encode_fns[] = {
  Opcode_all8_Slot_inst_encode, 0, 0, 0, Opcode_all8_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bf_encode_fns[] = {
  Opcode_bf_Slot_inst_encode, 0, 0, 0, Opcode_bf_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_bt_encode_fns[] = {
  Opcode_bt_Slot_inst_encode, 0, 0, 0, Opcode_bt_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_movf_encode_fns[] = {
  Opcode_movf_Slot_inst_encode, 0, 0, 0, Opcode_movf_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_movt_encode_fns[] = {
  Opcode_movt_Slot_inst_encode, 0, 0, 0, Opcode_movt_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_rsr_br_encode_fns[] = {
  Opcode_rsr_br_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_br_encode_fns[] = {
  Opcode_wsr_br_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_br_encode_fns[] = {
  Opcode_xsr_br_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_ccount_encode_fns[] = {
  Opcode_rsr_ccount_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_ccount_encode_fns[] = {
  Opcode_wsr_ccount_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_ccount_encode_fns[] = {
  Opcode_xsr_ccount_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_ccompare0_encode_fns[] = {
  Opcode_rsr_ccompare0_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_ccompare0_encode_fns[] = {
  Opcode_wsr_ccompare0_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_ccompare0_encode_fns[] = {
  Opcode_xsr_ccompare0_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_ccompare1_encode_fns[] = {
  Opcode_rsr_ccompare1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_ccompare1_encode_fns[] = {
  Opcode_wsr_ccompare1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_ccompare1_encode_fns[] = {
  Opcode_xsr_ccompare1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ipf_encode_fns[] = {
  Opcode_ipf_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ihi_encode_fns[] = {
  Opcode_ihi_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_iii_encode_fns[] = {
  Opcode_iii_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_lict_encode_fns[] = {
  Opcode_lict_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_licw_encode_fns[] = {
  Opcode_licw_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_sict_encode_fns[] = {
  Opcode_sict_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_sicw_encode_fns[] = {
  Opcode_sicw_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_dhwb_encode_fns[] = {
  Opcode_dhwb_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_dhwbi_encode_fns[] = {
  Opcode_dhwbi_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_diwb_encode_fns[] = {
  Opcode_diwb_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_diwbi_encode_fns[] = {
  Opcode_diwbi_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_dhi_encode_fns[] = {
  Opcode_dhi_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_dii_encode_fns[] = {
  Opcode_dii_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_dpfr_encode_fns[] = {
  Opcode_dpfr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_dpfw_encode_fns[] = {
  Opcode_dpfw_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_dpfro_encode_fns[] = {
  Opcode_dpfro_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_dpfwo_encode_fns[] = {
  Opcode_dpfwo_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_sdct_encode_fns[] = {
  Opcode_sdct_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ldct_encode_fns[] = {
  Opcode_ldct_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_ptevaddr_encode_fns[] = {
  Opcode_wsr_ptevaddr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_ptevaddr_encode_fns[] = {
  Opcode_rsr_ptevaddr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_ptevaddr_encode_fns[] = {
  Opcode_xsr_ptevaddr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_rasid_encode_fns[] = {
  Opcode_rsr_rasid_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_rasid_encode_fns[] = {
  Opcode_wsr_rasid_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_rasid_encode_fns[] = {
  Opcode_xsr_rasid_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_itlbcfg_encode_fns[] = {
  Opcode_rsr_itlbcfg_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_itlbcfg_encode_fns[] = {
  Opcode_wsr_itlbcfg_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_itlbcfg_encode_fns[] = {
  Opcode_xsr_itlbcfg_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_dtlbcfg_encode_fns[] = {
  Opcode_rsr_dtlbcfg_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_dtlbcfg_encode_fns[] = {
  Opcode_wsr_dtlbcfg_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_dtlbcfg_encode_fns[] = {
  Opcode_xsr_dtlbcfg_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_idtlb_encode_fns[] = {
  Opcode_idtlb_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_pdtlb_encode_fns[] = {
  Opcode_pdtlb_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rdtlb0_encode_fns[] = {
  Opcode_rdtlb0_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rdtlb1_encode_fns[] = {
  Opcode_rdtlb1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wdtlb_encode_fns[] = {
  Opcode_wdtlb_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_iitlb_encode_fns[] = {
  Opcode_iitlb_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_pitlb_encode_fns[] = {
  Opcode_pitlb_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ritlb0_encode_fns[] = {
  Opcode_ritlb0_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ritlb1_encode_fns[] = {
  Opcode_ritlb1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_witlb_encode_fns[] = {
  Opcode_witlb_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ldpte_encode_fns[] = {
  Opcode_ldpte_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_hwwitlba_encode_fns[] = {
  Opcode_hwwitlba_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_hwwdtlba_encode_fns[] = {
  Opcode_hwwdtlba_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_cpenable_encode_fns[] = {
  Opcode_rsr_cpenable_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_cpenable_encode_fns[] = {
  Opcode_wsr_cpenable_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_cpenable_encode_fns[] = {
  Opcode_xsr_cpenable_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_clamps_encode_fns[] = {
  Opcode_clamps_Slot_inst_encode, 0, 0, 0, Opcode_clamps_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_min_encode_fns[] = {
  Opcode_min_Slot_inst_encode, 0, 0, 0, Opcode_min_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_max_encode_fns[] = {
  Opcode_max_Slot_inst_encode, 0, 0, 0, Opcode_max_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_minu_encode_fns[] = {
  Opcode_minu_Slot_inst_encode, 0, 0, 0, Opcode_minu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_maxu_encode_fns[] = {
  Opcode_maxu_Slot_inst_encode, 0, 0, 0, Opcode_maxu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_nsa_encode_fns[] = {
  Opcode_nsa_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_nsau_encode_fns[] = {
  Opcode_nsau_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_sext_encode_fns[] = {
  Opcode_sext_Slot_inst_encode, 0, 0, 0, Opcode_sext_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_l32ai_encode_fns[] = {
  Opcode_l32ai_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_s32ri_encode_fns[] = {
  Opcode_s32ri_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_s32c1i_encode_fns[] = {
  Opcode_s32c1i_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_scompare1_encode_fns[] = {
  Opcode_rsr_scompare1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_scompare1_encode_fns[] = {
  Opcode_wsr_scompare1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_scompare1_encode_fns[] = {
  Opcode_xsr_scompare1_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rsr_atomctl_encode_fns[] = {
  Opcode_rsr_atomctl_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wsr_atomctl_encode_fns[] = {
  Opcode_wsr_atomctl_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_xsr_atomctl_encode_fns[] = {
  Opcode_xsr_atomctl_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rer_encode_fns[] = {
  Opcode_rer_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wer_encode_fns[] = {
  Opcode_wer_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rur_ae_ovf_sar_encode_fns[] = {
  Opcode_rur_ae_ovf_sar_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wur_ae_ovf_sar_encode_fns[] = {
  Opcode_wur_ae_ovf_sar_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rur_ae_bithead_encode_fns[] = {
  Opcode_rur_ae_bithead_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wur_ae_bithead_encode_fns[] = {
  Opcode_wur_ae_bithead_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rur_ae_ts_fts_bu_bp_encode_fns[] = {
  Opcode_rur_ae_ts_fts_bu_bp_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wur_ae_ts_fts_bu_bp_encode_fns[] = {
  Opcode_wur_ae_ts_fts_bu_bp_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rur_ae_sd_no_encode_fns[] = {
  Opcode_rur_ae_sd_no_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wur_ae_sd_no_encode_fns[] = {
  Opcode_wur_ae_sd_no_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rur_ae_overflow_encode_fns[] = {
  Opcode_rur_ae_overflow_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wur_ae_overflow_encode_fns[] = {
  Opcode_wur_ae_overflow_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rur_ae_sar_encode_fns[] = {
  Opcode_rur_ae_sar_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wur_ae_sar_encode_fns[] = {
  Opcode_wur_ae_sar_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rur_ae_bitptr_encode_fns[] = {
  Opcode_rur_ae_bitptr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wur_ae_bitptr_encode_fns[] = {
  Opcode_wur_ae_bitptr_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rur_ae_bitsused_encode_fns[] = {
  Opcode_rur_ae_bitsused_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wur_ae_bitsused_encode_fns[] = {
  Opcode_wur_ae_bitsused_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rur_ae_tablesize_encode_fns[] = {
  Opcode_rur_ae_tablesize_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wur_ae_tablesize_encode_fns[] = {
  Opcode_wur_ae_tablesize_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rur_ae_first_ts_encode_fns[] = {
  Opcode_rur_ae_first_ts_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wur_ae_first_ts_encode_fns[] = {
  Opcode_wur_ae_first_ts_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rur_ae_nextoffset_encode_fns[] = {
  Opcode_rur_ae_nextoffset_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wur_ae_nextoffset_encode_fns[] = {
  Opcode_wur_ae_nextoffset_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_rur_ae_searchdone_encode_fns[] = {
  Opcode_rur_ae_searchdone_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_wur_ae_searchdone_encode_fns[] = {
  Opcode_wur_ae_searchdone_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_lp16f_i_encode_fns[] = {
  Opcode_ae_lp16f_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp16f_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp16f_iu_encode_fns[] = {
  Opcode_ae_lp16f_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp16f_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp16f_x_encode_fns[] = {
  Opcode_ae_lp16f_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp16f_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp16f_xu_encode_fns[] = {
  Opcode_ae_lp16f_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp16f_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24_i_encode_fns[] = {
  Opcode_ae_lp24_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24_iu_encode_fns[] = {
  Opcode_ae_lp24_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24_x_encode_fns[] = {
  Opcode_ae_lp24_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24_xu_encode_fns[] = {
  Opcode_ae_lp24_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24f_i_encode_fns[] = {
  Opcode_ae_lp24f_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24f_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24f_iu_encode_fns[] = {
  Opcode_ae_lp24f_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24f_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24f_x_encode_fns[] = {
  Opcode_ae_lp24f_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24f_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24f_xu_encode_fns[] = {
  Opcode_ae_lp24f_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24f_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp16x2f_i_encode_fns[] = {
  Opcode_ae_lp16x2f_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp16x2f_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp16x2f_iu_encode_fns[] = {
  Opcode_ae_lp16x2f_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp16x2f_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp16x2f_x_encode_fns[] = {
  Opcode_ae_lp16x2f_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp16x2f_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp16x2f_xu_encode_fns[] = {
  Opcode_ae_lp16x2f_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp16x2f_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24x2f_i_encode_fns[] = {
  Opcode_ae_lp24x2f_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24x2f_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24x2f_iu_encode_fns[] = {
  Opcode_ae_lp24x2f_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24x2f_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24x2f_x_encode_fns[] = {
  Opcode_ae_lp24x2f_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24x2f_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24x2f_xu_encode_fns[] = {
  Opcode_ae_lp24x2f_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24x2f_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24x2_i_encode_fns[] = {
  Opcode_ae_lp24x2_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24x2_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24x2_iu_encode_fns[] = {
  Opcode_ae_lp24x2_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24x2_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24x2_x_encode_fns[] = {
  Opcode_ae_lp24x2_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24x2_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lp24x2_xu_encode_fns[] = {
  Opcode_ae_lp24x2_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lp24x2_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp16x2f_i_encode_fns[] = {
  Opcode_ae_sp16x2f_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp16x2f_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp16x2f_iu_encode_fns[] = {
  Opcode_ae_sp16x2f_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp16x2f_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp16x2f_x_encode_fns[] = {
  Opcode_ae_sp16x2f_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp16x2f_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp16x2f_xu_encode_fns[] = {
  Opcode_ae_sp16x2f_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp16x2f_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24x2s_i_encode_fns[] = {
  Opcode_ae_sp24x2s_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24x2s_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24x2s_iu_encode_fns[] = {
  Opcode_ae_sp24x2s_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24x2s_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24x2s_x_encode_fns[] = {
  Opcode_ae_sp24x2s_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24x2s_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24x2s_xu_encode_fns[] = {
  Opcode_ae_sp24x2s_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24x2s_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24x2f_i_encode_fns[] = {
  Opcode_ae_sp24x2f_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24x2f_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24x2f_iu_encode_fns[] = {
  Opcode_ae_sp24x2f_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24x2f_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24x2f_x_encode_fns[] = {
  Opcode_ae_sp24x2f_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24x2f_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24x2f_xu_encode_fns[] = {
  Opcode_ae_sp24x2f_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24x2f_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp16f_l_i_encode_fns[] = {
  Opcode_ae_sp16f_l_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp16f_l_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp16f_l_iu_encode_fns[] = {
  Opcode_ae_sp16f_l_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp16f_l_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp16f_l_x_encode_fns[] = {
  Opcode_ae_sp16f_l_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp16f_l_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp16f_l_xu_encode_fns[] = {
  Opcode_ae_sp16f_l_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp16f_l_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24s_l_i_encode_fns[] = {
  Opcode_ae_sp24s_l_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24s_l_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24s_l_iu_encode_fns[] = {
  Opcode_ae_sp24s_l_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24s_l_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24s_l_x_encode_fns[] = {
  Opcode_ae_sp24s_l_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24s_l_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24s_l_xu_encode_fns[] = {
  Opcode_ae_sp24s_l_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24s_l_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24f_l_i_encode_fns[] = {
  Opcode_ae_sp24f_l_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24f_l_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24f_l_iu_encode_fns[] = {
  Opcode_ae_sp24f_l_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24f_l_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24f_l_x_encode_fns[] = {
  Opcode_ae_sp24f_l_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24f_l_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sp24f_l_xu_encode_fns[] = {
  Opcode_ae_sp24f_l_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sp24f_l_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lq56_i_encode_fns[] = {
  Opcode_ae_lq56_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_lq56_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lq56_iu_encode_fns[] = {
  Opcode_ae_lq56_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lq56_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lq56_x_encode_fns[] = {
  Opcode_ae_lq56_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_lq56_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lq56_xu_encode_fns[] = {
  Opcode_ae_lq56_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lq56_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lq32f_i_encode_fns[] = {
  Opcode_ae_lq32f_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_lq32f_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lq32f_iu_encode_fns[] = {
  Opcode_ae_lq32f_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lq32f_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lq32f_x_encode_fns[] = {
  Opcode_ae_lq32f_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_lq32f_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_lq32f_xu_encode_fns[] = {
  Opcode_ae_lq32f_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_lq32f_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sq56s_i_encode_fns[] = {
  Opcode_ae_sq56s_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_sq56s_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sq56s_iu_encode_fns[] = {
  Opcode_ae_sq56s_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sq56s_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sq56s_x_encode_fns[] = {
  Opcode_ae_sq56s_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_sq56s_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sq56s_xu_encode_fns[] = {
  Opcode_ae_sq56s_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sq56s_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sq32f_i_encode_fns[] = {
  Opcode_ae_sq32f_i_Slot_inst_encode, 0, 0, 0, Opcode_ae_sq32f_i_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sq32f_iu_encode_fns[] = {
  Opcode_ae_sq32f_iu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sq32f_iu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sq32f_x_encode_fns[] = {
  Opcode_ae_sq32f_x_Slot_inst_encode, 0, 0, 0, Opcode_ae_sq32f_x_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sq32f_xu_encode_fns[] = {
  Opcode_ae_sq32f_xu_Slot_inst_encode, 0, 0, 0, Opcode_ae_sq32f_xu_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_zerop48_encode_fns[] = {
  0, 0, 0, Opcode_ae_zerop48_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_movp48_encode_fns[] = {
  Opcode_ae_movp48_Slot_inst_encode, 0, 0, Opcode_ae_movp48_Slot_ae_slot1_encode, Opcode_ae_movp48_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_selp24_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_selp24_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_selp24_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_selp24_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_selp24_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_selp24_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_selp24_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_selp24_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_movtp24x2_encode_fns[] = {
  0, 0, 0, Opcode_ae_movtp24x2_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_movfp24x2_encode_fns[] = {
  0, 0, 0, Opcode_ae_movfp24x2_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_movtp48_encode_fns[] = {
  0, 0, 0, Opcode_ae_movtp48_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_movfp48_encode_fns[] = {
  0, 0, 0, Opcode_ae_movfp48_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_movpa24x2_encode_fns[] = {
  Opcode_ae_movpa24x2_Slot_inst_encode, 0, 0, 0, Opcode_ae_movpa24x2_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_truncp24a32x2_encode_fns[] = {
  Opcode_ae_truncp24a32x2_Slot_inst_encode, 0, 0, 0, Opcode_ae_truncp24a32x2_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_cvta32p24_l_encode_fns[] = {
  Opcode_ae_cvta32p24_l_Slot_inst_encode, 0, 0, 0, Opcode_ae_cvta32p24_l_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_cvta32p24_h_encode_fns[] = {
  Opcode_ae_cvta32p24_h_Slot_inst_encode, 0, 0, 0, Opcode_ae_cvta32p24_h_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_cvtp24a16x2_ll_encode_fns[] = {
  Opcode_ae_cvtp24a16x2_ll_Slot_inst_encode, 0, 0, 0, Opcode_ae_cvtp24a16x2_ll_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_cvtp24a16x2_lh_encode_fns[] = {
  Opcode_ae_cvtp24a16x2_lh_Slot_inst_encode, 0, 0, 0, Opcode_ae_cvtp24a16x2_lh_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_cvtp24a16x2_hl_encode_fns[] = {
  Opcode_ae_cvtp24a16x2_hl_Slot_inst_encode, 0, 0, 0, Opcode_ae_cvtp24a16x2_hl_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_cvtp24a16x2_hh_encode_fns[] = {
  Opcode_ae_cvtp24a16x2_hh_Slot_inst_encode, 0, 0, 0, Opcode_ae_cvtp24a16x2_hh_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_truncp24q48x2_encode_fns[] = {
  0, 0, 0, Opcode_ae_truncp24q48x2_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_truncp16_encode_fns[] = {
  0, 0, 0, Opcode_ae_truncp16_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_roundsp24q48sym_encode_fns[] = {
  0, 0, 0, Opcode_ae_roundsp24q48sym_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_roundsp24q48asym_encode_fns[] = {
  0, 0, 0, Opcode_ae_roundsp24q48asym_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_roundsp16q48sym_encode_fns[] = {
  0, 0, 0, Opcode_ae_roundsp16q48sym_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_roundsp16q48asym_encode_fns[] = {
  0, 0, 0, Opcode_ae_roundsp16q48asym_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_roundsp16sym_encode_fns[] = {
  0, 0, 0, Opcode_ae_roundsp16sym_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_roundsp16asym_encode_fns[] = {
  0, 0, 0, Opcode_ae_roundsp16asym_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_zeroq56_encode_fns[] = {
  0, 0, 0, Opcode_ae_zeroq56_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_movq56_encode_fns[] = {
  Opcode_ae_movq56_Slot_inst_encode, 0, 0, Opcode_ae_movq56_Slot_ae_slot1_encode, Opcode_ae_movq56_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_movtq56_encode_fns[] = {
  Opcode_ae_movtq56_Slot_inst_encode, 0, 0, 0, Opcode_ae_movtq56_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_movfq56_encode_fns[] = {
  Opcode_ae_movfq56_Slot_inst_encode, 0, 0, 0, Opcode_ae_movfq56_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_cvtq48a32s_encode_fns[] = {
  Opcode_ae_cvtq48a32s_Slot_inst_encode, 0, 0, 0, Opcode_ae_cvtq48a32s_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_cvtq48p24s_l_encode_fns[] = {
  0, 0, 0, Opcode_ae_cvtq48p24s_l_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_cvtq48p24s_h_encode_fns[] = {
  0, 0, 0, Opcode_ae_cvtq48p24s_h_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_satq48s_encode_fns[] = {
  0, 0, 0, Opcode_ae_satq48s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_truncq32_encode_fns[] = {
  0, 0, 0, Opcode_ae_truncq32_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_roundsq32sym_encode_fns[] = {
  0, 0, 0, Opcode_ae_roundsq32sym_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_roundsq32asym_encode_fns[] = {
  0, 0, 0, Opcode_ae_roundsq32asym_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_trunca32q48_encode_fns[] = {
  Opcode_ae_trunca32q48_Slot_inst_encode, 0, 0, 0, Opcode_ae_trunca32q48_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_movap24s_l_encode_fns[] = {
  Opcode_ae_movap24s_l_Slot_inst_encode, 0, 0, 0, Opcode_ae_movap24s_l_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_movap24s_h_encode_fns[] = {
  Opcode_ae_movap24s_h_Slot_inst_encode, 0, 0, 0, Opcode_ae_movap24s_h_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_trunca16p24s_l_encode_fns[] = {
  Opcode_ae_trunca16p24s_l_Slot_inst_encode, 0, 0, 0, Opcode_ae_trunca16p24s_l_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_trunca16p24s_h_encode_fns[] = {
  Opcode_ae_trunca16p24s_h_Slot_inst_encode, 0, 0, 0, Opcode_ae_trunca16p24s_h_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_addp24_encode_fns[] = {
  0, 0, 0, Opcode_ae_addp24_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_subp24_encode_fns[] = {
  0, 0, 0, Opcode_ae_subp24_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_negp24_encode_fns[] = {
  0, 0, 0, Opcode_ae_negp24_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_absp24_encode_fns[] = {
  0, 0, 0, Opcode_ae_absp24_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_maxp24s_encode_fns[] = {
  0, 0, 0, Opcode_ae_maxp24s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_minp24s_encode_fns[] = {
  0, 0, 0, Opcode_ae_minp24s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_maxbp24s_encode_fns[] = {
  0, 0, 0, Opcode_ae_maxbp24s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_minbp24s_encode_fns[] = {
  0, 0, 0, Opcode_ae_minbp24s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_addsp24s_encode_fns[] = {
  0, 0, 0, Opcode_ae_addsp24s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_subsp24s_encode_fns[] = {
  0, 0, 0, Opcode_ae_subsp24s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_negsp24s_encode_fns[] = {
  0, 0, 0, Opcode_ae_negsp24s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_abssp24s_encode_fns[] = {
  0, 0, 0, Opcode_ae_abssp24s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_andp48_encode_fns[] = {
  0, 0, 0, Opcode_ae_andp48_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_nandp48_encode_fns[] = {
  0, 0, 0, Opcode_ae_nandp48_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_orp48_encode_fns[] = {
  0, 0, 0, Opcode_ae_orp48_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_xorp48_encode_fns[] = {
  0, 0, 0, Opcode_ae_xorp48_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_ltp24s_encode_fns[] = {
  0, 0, 0, Opcode_ae_ltp24s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_lep24s_encode_fns[] = {
  0, 0, 0, Opcode_ae_lep24s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_eqp24_encode_fns[] = {
  0, 0, 0, Opcode_ae_eqp24_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_addq56_encode_fns[] = {
  0, 0, 0, Opcode_ae_addq56_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_subq56_encode_fns[] = {
  0, 0, 0, Opcode_ae_subq56_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_negq56_encode_fns[] = {
  0, 0, 0, Opcode_ae_negq56_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_absq56_encode_fns[] = {
  0, 0, 0, Opcode_ae_absq56_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_maxq56s_encode_fns[] = {
  0, 0, 0, Opcode_ae_maxq56s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_minq56s_encode_fns[] = {
  0, 0, 0, Opcode_ae_minq56s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_maxbq56s_encode_fns[] = {
  0, 0, 0, Opcode_ae_maxbq56s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_minbq56s_encode_fns[] = {
  0, 0, 0, Opcode_ae_minbq56s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_addsq56s_encode_fns[] = {
  0, 0, 0, Opcode_ae_addsq56s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_subsq56s_encode_fns[] = {
  0, 0, 0, Opcode_ae_subsq56s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_negsq56s_encode_fns[] = {
  0, 0, 0, Opcode_ae_negsq56s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_abssq56s_encode_fns[] = {
  0, 0, 0, Opcode_ae_abssq56s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_andq56_encode_fns[] = {
  0, 0, 0, Opcode_ae_andq56_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_nandq56_encode_fns[] = {
  0, 0, 0, Opcode_ae_nandq56_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_orq56_encode_fns[] = {
  0, 0, 0, Opcode_ae_orq56_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_xorq56_encode_fns[] = {
  0, 0, 0, Opcode_ae_xorq56_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_sllip24_encode_fns[] = {
  0, 0, 0, Opcode_ae_sllip24_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_srlip24_encode_fns[] = {
  0, 0, 0, Opcode_ae_srlip24_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_sraip24_encode_fns[] = {
  0, 0, 0, Opcode_ae_sraip24_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_sllsp24_encode_fns[] = {
  0, 0, 0, Opcode_ae_sllsp24_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_srlsp24_encode_fns[] = {
  0, 0, 0, Opcode_ae_srlsp24_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_srasp24_encode_fns[] = {
  0, 0, 0, Opcode_ae_srasp24_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_sllisp24s_encode_fns[] = {
  0, 0, 0, Opcode_ae_sllisp24s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_sllssp24s_encode_fns[] = {
  0, 0, 0, Opcode_ae_sllssp24s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_slliq56_encode_fns[] = {
  Opcode_ae_slliq56_Slot_inst_encode, 0, 0, 0, Opcode_ae_slliq56_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_srliq56_encode_fns[] = {
  Opcode_ae_srliq56_Slot_inst_encode, 0, 0, 0, Opcode_ae_srliq56_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sraiq56_encode_fns[] = {
  Opcode_ae_sraiq56_Slot_inst_encode, 0, 0, 0, Opcode_ae_sraiq56_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sllsq56_encode_fns[] = {
  Opcode_ae_sllsq56_Slot_inst_encode, 0, 0, 0, Opcode_ae_sllsq56_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_srlsq56_encode_fns[] = {
  Opcode_ae_srlsq56_Slot_inst_encode, 0, 0, 0, Opcode_ae_srlsq56_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_srasq56_encode_fns[] = {
  Opcode_ae_srasq56_Slot_inst_encode, 0, 0, 0, Opcode_ae_srasq56_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sllaq56_encode_fns[] = {
  Opcode_ae_sllaq56_Slot_inst_encode, 0, 0, 0, Opcode_ae_sllaq56_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_srlaq56_encode_fns[] = {
  Opcode_ae_srlaq56_Slot_inst_encode, 0, 0, 0, Opcode_ae_srlaq56_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sraaq56_encode_fns[] = {
  Opcode_ae_sraaq56_Slot_inst_encode, 0, 0, 0, Opcode_ae_sraaq56_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sllisq56s_encode_fns[] = {
  Opcode_ae_sllisq56s_Slot_inst_encode, 0, 0, 0, Opcode_ae_sllisq56s_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sllssq56s_encode_fns[] = {
  Opcode_ae_sllssq56s_Slot_inst_encode, 0, 0, 0, Opcode_ae_sllssq56s_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_sllasq56s_encode_fns[] = {
  Opcode_ae_sllasq56s_Slot_inst_encode, 0, 0, 0, Opcode_ae_sllasq56s_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_ltq56s_encode_fns[] = {
  0, 0, 0, Opcode_ae_ltq56s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_leq56s_encode_fns[] = {
  0, 0, 0, Opcode_ae_leq56s_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_eqq56_encode_fns[] = {
  0, 0, 0, Opcode_ae_eqq56_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_nsaq56s_encode_fns[] = {
  Opcode_ae_nsaq56s_Slot_inst_encode, 0, 0, 0, Opcode_ae_nsaq56s_Slot_ae_slot0_encode
};

xtensa_opcode_encode_fn Opcode_ae_mulfs32p16s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulfs32p16s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulfp24s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulfp24s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulp24s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulp24s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulfs32p16s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulfs32p16s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulfp24s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulfp24s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulp24s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulp24s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulfs32p16s_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulfs32p16s_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulfp24s_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulfp24s_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulp24s_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulp24s_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulfs32p16s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulfs32p16s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulfp24s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulfp24s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulp24s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulp24s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafs32p16s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafs32p16s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafp24s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafp24s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulap24s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulap24s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafs32p16s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafs32p16s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafp24s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafp24s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulap24s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulap24s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafs32p16s_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafs32p16s_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafp24s_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafp24s_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulap24s_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulap24s_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafs32p16s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafs32p16s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafp24s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafp24s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulap24s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulap24s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfs32p16s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfs32p16s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfp24s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfp24s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsp24s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsp24s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfs32p16s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfs32p16s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfp24s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfp24s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsp24s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsp24s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfs32p16s_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfs32p16s_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfp24s_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfp24s_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsp24s_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsp24s_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfs32p16s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfs32p16s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfp24s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfp24s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsp24s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsp24s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafs56p24s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafs56p24s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulas56p24s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulas56p24s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafs56p24s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafs56p24s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulas56p24s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulas56p24s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafs56p24s_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafs56p24s_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulas56p24s_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulas56p24s_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafs56p24s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafs56p24s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulas56p24s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulas56p24s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfs56p24s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfs56p24s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulss56p24s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulss56p24s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfs56p24s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfs56p24s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulss56p24s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulss56p24s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfs56p24s_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfs56p24s_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulss56p24s_hl_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulss56p24s_hl_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfs56p24s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfs56p24s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulss56p24s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulss56p24s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulfq32sp16s_l_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulfq32sp16s_l_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulfq32sp16s_h_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulfq32sp16s_h_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulfq32sp16u_l_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulfq32sp16u_l_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulfq32sp16u_h_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulfq32sp16u_h_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulq32sp16s_l_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulq32sp16s_l_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulq32sp16s_h_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulq32sp16s_h_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulq32sp16u_l_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulq32sp16u_l_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulq32sp16u_h_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulq32sp16u_h_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafq32sp16s_l_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafq32sp16s_l_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafq32sp16s_h_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafq32sp16s_h_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafq32sp16u_l_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafq32sp16u_l_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulafq32sp16u_h_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulafq32sp16u_h_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulaq32sp16s_l_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulaq32sp16s_l_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulaq32sp16s_h_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulaq32sp16s_h_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulaq32sp16u_l_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulaq32sp16u_l_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulaq32sp16u_h_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulaq32sp16u_h_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfq32sp16s_l_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfq32sp16s_l_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfq32sp16s_h_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfq32sp16s_h_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfq32sp16u_l_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfq32sp16u_l_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsfq32sp16u_h_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsfq32sp16u_h_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsq32sp16s_l_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsq32sp16s_l_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsq32sp16s_h_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsq32sp16s_h_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsq32sp16u_l_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsq32sp16u_l_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsq32sp16u_h_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsq32sp16u_h_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaaq32sp16s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaaq32sp16s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaafq32sp16s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaafq32sp16s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaaq32sp16u_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaaq32sp16u_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaafq32sp16u_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaafq32sp16u_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaaq32sp16s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaaq32sp16s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaafq32sp16s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaafq32sp16s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaaq32sp16u_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaaq32sp16u_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaafq32sp16u_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaafq32sp16u_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaaq32sp16s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaaq32sp16s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaafq32sp16s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaafq32sp16s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaaq32sp16u_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaaq32sp16u_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaafq32sp16u_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaafq32sp16u_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasq32sp16s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasq32sp16s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasfq32sp16s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasfq32sp16s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasq32sp16u_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasq32sp16u_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasfq32sp16u_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasfq32sp16u_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasq32sp16s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasq32sp16s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasfq32sp16s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasfq32sp16s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasq32sp16u_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasq32sp16u_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasfq32sp16u_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasfq32sp16u_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasq32sp16s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasq32sp16s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasfq32sp16s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasfq32sp16s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasq32sp16u_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasq32sp16u_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasfq32sp16u_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasfq32sp16u_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsaq32sp16s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsaq32sp16s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsafq32sp16s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsafq32sp16s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsaq32sp16u_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsaq32sp16u_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsafq32sp16u_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsafq32sp16u_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsaq32sp16s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsaq32sp16s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsafq32sp16s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsafq32sp16s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsaq32sp16u_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsaq32sp16u_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsafq32sp16u_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsafq32sp16u_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsaq32sp16s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsaq32sp16s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsafq32sp16s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsafq32sp16s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsaq32sp16u_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsaq32sp16u_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsafq32sp16u_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsafq32sp16u_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssq32sp16s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssq32sp16s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssfq32sp16s_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssfq32sp16s_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssq32sp16u_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssq32sp16u_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssfq32sp16u_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssfq32sp16u_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssq32sp16s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssq32sp16s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssfq32sp16s_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssfq32sp16s_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssq32sp16u_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssq32sp16u_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssfq32sp16u_hh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssfq32sp16u_hh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssq32sp16s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssq32sp16s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssfq32sp16s_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssfq32sp16s_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssq32sp16u_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssq32sp16u_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssfq32sp16u_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssfq32sp16u_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaafp24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaafp24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaap24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaap24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaafp24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaafp24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzaap24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzaap24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasfp24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasfp24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasp24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasp24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasfp24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasfp24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzasp24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzasp24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsafp24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsafp24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsap24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsap24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsafp24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsafp24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzsap24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzsap24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssfp24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssfp24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssp24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssp24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssfp24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssfp24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulzssp24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulzssp24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulaafp24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulaafp24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulaap24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulaap24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulaafp24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulaafp24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulaap24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulaap24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulasfp24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulasfp24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulasp24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulasp24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulasfp24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulasfp24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulasp24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulasp24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsafp24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsafp24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsap24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsap24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsafp24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsafp24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulsap24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulsap24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulssfp24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulssfp24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulssp24s_hh_ll_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulssp24s_hh_ll_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulssfp24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulssfp24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_mulssp24s_hl_lh_encode_fns[] = {
  0, 0, 0, Opcode_ae_mulssp24s_hl_lh_Slot_ae_slot1_encode, 0
};

xtensa_opcode_encode_fn Opcode_ae_sha32_encode_fns[] = {
  Opcode_ae_sha32_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_vldl32t_encode_fns[] = {
  Opcode_ae_vldl32t_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_vldl16t_encode_fns[] = {
  Opcode_ae_vldl16t_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_vldl16c_encode_fns[] = {
  Opcode_ae_vldl16c_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_vldsht_encode_fns[] = {
  Opcode_ae_vldsht_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_lb_encode_fns[] = {
  Opcode_ae_lb_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_lbi_encode_fns[] = {
  Opcode_ae_lbi_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_lbk_encode_fns[] = {
  Opcode_ae_lbk_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_lbki_encode_fns[] = {
  Opcode_ae_lbki_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_db_encode_fns[] = {
  Opcode_ae_db_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_dbi_encode_fns[] = {
  Opcode_ae_dbi_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_vlel32t_encode_fns[] = {
  Opcode_ae_vlel32t_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_vlel16t_encode_fns[] = {
  Opcode_ae_vlel16t_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_sb_encode_fns[] = {
  Opcode_ae_sb_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_sbi_encode_fns[] = {
  Opcode_ae_sbi_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_vles16c_encode_fns[] = {
  Opcode_ae_vles16c_Slot_inst_encode, 0, 0, 0, 0
};

xtensa_opcode_encode_fn Opcode_ae_sbf_encode_fns[] = {
  Opcode_ae_sbf_Slot_inst_encode, 0, 0, 0, 0
};


/* Opcode table.  */

static xtensa_funcUnit_use Opcode_ae_vldl32t_funcUnit_uses[] = {
  { FUNCUNIT_ae_add32, 3 }
};

static xtensa_funcUnit_use Opcode_ae_vldl16t_funcUnit_uses[] = {
  { FUNCUNIT_ae_add32, 3 }
};

static xtensa_funcUnit_use Opcode_ae_vldl16c_funcUnit_uses[] = {
  { FUNCUNIT_ae_shift32x4, 2 },
  { FUNCUNIT_ae_shift32x5, 3 },
  { FUNCUNIT_ae_add32, 3 }
};

static xtensa_funcUnit_use Opcode_ae_vldsht_funcUnit_uses[] = {
  { FUNCUNIT_ae_shift32x4, 2 },
  { FUNCUNIT_ae_shift32x5, 3 },
  { FUNCUNIT_ae_add32, 3 }
};

static xtensa_funcUnit_use Opcode_ae_lb_funcUnit_uses[] = {
  { FUNCUNIT_ae_subshift, 2 }
};

static xtensa_funcUnit_use Opcode_ae_lbi_funcUnit_uses[] = {
  { FUNCUNIT_ae_subshift, 2 }
};

static xtensa_funcUnit_use Opcode_ae_lbk_funcUnit_uses[] = {
  { FUNCUNIT_ae_subshift, 2 }
};

static xtensa_funcUnit_use Opcode_ae_lbki_funcUnit_uses[] = {
  { FUNCUNIT_ae_subshift, 2 }
};

static xtensa_funcUnit_use Opcode_ae_db_funcUnit_uses[] = {
  { FUNCUNIT_ae_shift32x4, 2 },
  { FUNCUNIT_ae_subshift, 2 }
};

static xtensa_funcUnit_use Opcode_ae_dbi_funcUnit_uses[] = {
  { FUNCUNIT_ae_shift32x4, 2 },
  { FUNCUNIT_ae_subshift, 2 }
};

static xtensa_funcUnit_use Opcode_ae_vlel32t_funcUnit_uses[] = {
  { FUNCUNIT_ae_add32, 3 }
};

static xtensa_funcUnit_use Opcode_ae_vlel16t_funcUnit_uses[] = {
  { FUNCUNIT_ae_add32, 3 }
};

static xtensa_funcUnit_use Opcode_ae_sb_funcUnit_uses[] = {
  { FUNCUNIT_ae_shift32x4, 2 },
  { FUNCUNIT_ae_subshift, 2 }
};

static xtensa_funcUnit_use Opcode_ae_sbi_funcUnit_uses[] = {
  { FUNCUNIT_ae_shift32x4, 2 },
  { FUNCUNIT_ae_subshift, 2 }
};

static xtensa_funcUnit_use Opcode_ae_vles16c_funcUnit_uses[] = {
  { FUNCUNIT_ae_shift32x4, 2 },
  { FUNCUNIT_ae_subshift, 2 }
};

static xtensa_funcUnit_use Opcode_ae_sbf_funcUnit_uses[] = {
  { FUNCUNIT_ae_shift32x4, 2 },
  { FUNCUNIT_ae_subshift, 2 }
};

static xtensa_opcode_internal opcodes[] = {
  { "excw", ICLASS_xt_iclass_excw,
    0,
    Opcode_excw_encode_fns, 0, 0 },
  { "rfe", ICLASS_xt_iclass_rfe,
    XTENSA_OPCODE_IS_JUMP,
    Opcode_rfe_encode_fns, 0, 0 },
  { "rfde", ICLASS_xt_iclass_rfde,
    XTENSA_OPCODE_IS_JUMP,
    Opcode_rfde_encode_fns, 0, 0 },
  { "syscall", ICLASS_xt_iclass_syscall,
    0,
    Opcode_syscall_encode_fns, 0, 0 },
  { "simcall", ICLASS_xt_iclass_simcall,
    0,
    Opcode_simcall_encode_fns, 0, 0 },
  { "call12", ICLASS_xt_iclass_call12,
    XTENSA_OPCODE_IS_CALL,
    Opcode_call12_encode_fns, 0, 0 },
  { "call8", ICLASS_xt_iclass_call8,
    XTENSA_OPCODE_IS_CALL,
    Opcode_call8_encode_fns, 0, 0 },
  { "call4", ICLASS_xt_iclass_call4,
    XTENSA_OPCODE_IS_CALL,
    Opcode_call4_encode_fns, 0, 0 },
  { "callx12", ICLASS_xt_iclass_callx12,
    XTENSA_OPCODE_IS_CALL,
    Opcode_callx12_encode_fns, 0, 0 },
  { "callx8", ICLASS_xt_iclass_callx8,
    XTENSA_OPCODE_IS_CALL,
    Opcode_callx8_encode_fns, 0, 0 },
  { "callx4", ICLASS_xt_iclass_callx4,
    XTENSA_OPCODE_IS_CALL,
    Opcode_callx4_encode_fns, 0, 0 },
  { "entry", ICLASS_xt_iclass_entry,
    0,
    Opcode_entry_encode_fns, 0, 0 },
  { "movsp", ICLASS_xt_iclass_movsp,
    0,
    Opcode_movsp_encode_fns, 0, 0 },
  { "rotw", ICLASS_xt_iclass_rotw,
    0,
    Opcode_rotw_encode_fns, 0, 0 },
  { "retw", ICLASS_xt_iclass_retw,
    XTENSA_OPCODE_IS_JUMP,
    Opcode_retw_encode_fns, 0, 0 },
  { "retw.n", ICLASS_xt_iclass_retw,
    XTENSA_OPCODE_IS_JUMP,
    Opcode_retw_n_encode_fns, 0, 0 },
  { "rfwo", ICLASS_xt_iclass_rfwou,
    XTENSA_OPCODE_IS_JUMP,
    Opcode_rfwo_encode_fns, 0, 0 },
  { "rfwu", ICLASS_xt_iclass_rfwou,
    XTENSA_OPCODE_IS_JUMP,
    Opcode_rfwu_encode_fns, 0, 0 },
  { "l32e", ICLASS_xt_iclass_l32e,
    0,
    Opcode_l32e_encode_fns, 0, 0 },
  { "s32e", ICLASS_xt_iclass_s32e,
    0,
    Opcode_s32e_encode_fns, 0, 0 },
  { "rsr.windowbase", ICLASS_xt_iclass_rsr_windowbase,
    0,
    Opcode_rsr_windowbase_encode_fns, 0, 0 },
  { "wsr.windowbase", ICLASS_xt_iclass_wsr_windowbase,
    0,
    Opcode_wsr_windowbase_encode_fns, 0, 0 },
  { "xsr.windowbase", ICLASS_xt_iclass_xsr_windowbase,
    0,
    Opcode_xsr_windowbase_encode_fns, 0, 0 },
  { "rsr.windowstart", ICLASS_xt_iclass_rsr_windowstart,
    0,
    Opcode_rsr_windowstart_encode_fns, 0, 0 },
  { "wsr.windowstart", ICLASS_xt_iclass_wsr_windowstart,
    0,
    Opcode_wsr_windowstart_encode_fns, 0, 0 },
  { "xsr.windowstart", ICLASS_xt_iclass_xsr_windowstart,
    0,
    Opcode_xsr_windowstart_encode_fns, 0, 0 },
  { "add.n", ICLASS_xt_iclass_add_n,
    0,
    Opcode_add_n_encode_fns, 0, 0 },
  { "addi.n", ICLASS_xt_iclass_addi_n,
    0,
    Opcode_addi_n_encode_fns, 0, 0 },
  { "beqz.n", ICLASS_xt_iclass_bz6,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_beqz_n_encode_fns, 0, 0 },
  { "bnez.n", ICLASS_xt_iclass_bz6,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bnez_n_encode_fns, 0, 0 },
  { "ill.n", ICLASS_xt_iclass_ill_n,
    0,
    Opcode_ill_n_encode_fns, 0, 0 },
  { "l32i.n", ICLASS_xt_iclass_loadi4,
    0,
    Opcode_l32i_n_encode_fns, 0, 0 },
  { "mov.n", ICLASS_xt_iclass_mov_n,
    0,
    Opcode_mov_n_encode_fns, 0, 0 },
  { "movi.n", ICLASS_xt_iclass_movi_n,
    0,
    Opcode_movi_n_encode_fns, 0, 0 },
  { "nop.n", ICLASS_xt_iclass_nopn,
    0,
    Opcode_nop_n_encode_fns, 0, 0 },
  { "ret.n", ICLASS_xt_iclass_retn,
    XTENSA_OPCODE_IS_JUMP,
    Opcode_ret_n_encode_fns, 0, 0 },
  { "s32i.n", ICLASS_xt_iclass_storei4,
    0,
    Opcode_s32i_n_encode_fns, 0, 0 },
  { "rur.threadptr", ICLASS_rur_threadptr,
    0,
    Opcode_rur_threadptr_encode_fns, 0, 0 },
  { "wur.threadptr", ICLASS_wur_threadptr,
    0,
    Opcode_wur_threadptr_encode_fns, 0, 0 },
  { "addi", ICLASS_xt_iclass_addi,
    0,
    Opcode_addi_encode_fns, 0, 0 },
  { "addmi", ICLASS_xt_iclass_addmi,
    0,
    Opcode_addmi_encode_fns, 0, 0 },
  { "add", ICLASS_xt_iclass_addsub,
    0,
    Opcode_add_encode_fns, 0, 0 },
  { "sub", ICLASS_xt_iclass_addsub,
    0,
    Opcode_sub_encode_fns, 0, 0 },
  { "addx2", ICLASS_xt_iclass_addsub,
    0,
    Opcode_addx2_encode_fns, 0, 0 },
  { "addx4", ICLASS_xt_iclass_addsub,
    0,
    Opcode_addx4_encode_fns, 0, 0 },
  { "addx8", ICLASS_xt_iclass_addsub,
    0,
    Opcode_addx8_encode_fns, 0, 0 },
  { "subx2", ICLASS_xt_iclass_addsub,
    0,
    Opcode_subx2_encode_fns, 0, 0 },
  { "subx4", ICLASS_xt_iclass_addsub,
    0,
    Opcode_subx4_encode_fns, 0, 0 },
  { "subx8", ICLASS_xt_iclass_addsub,
    0,
    Opcode_subx8_encode_fns, 0, 0 },
  { "and", ICLASS_xt_iclass_bit,
    0,
    Opcode_and_encode_fns, 0, 0 },
  { "or", ICLASS_xt_iclass_bit,
    0,
    Opcode_or_encode_fns, 0, 0 },
  { "xor", ICLASS_xt_iclass_bit,
    0,
    Opcode_xor_encode_fns, 0, 0 },
  { "beqi", ICLASS_xt_iclass_bsi8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_beqi_encode_fns, 0, 0 },
  { "bnei", ICLASS_xt_iclass_bsi8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bnei_encode_fns, 0, 0 },
  { "bgei", ICLASS_xt_iclass_bsi8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bgei_encode_fns, 0, 0 },
  { "blti", ICLASS_xt_iclass_bsi8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_blti_encode_fns, 0, 0 },
  { "bbci", ICLASS_xt_iclass_bsi8b,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bbci_encode_fns, 0, 0 },
  { "bbsi", ICLASS_xt_iclass_bsi8b,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bbsi_encode_fns, 0, 0 },
  { "bgeui", ICLASS_xt_iclass_bsi8u,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bgeui_encode_fns, 0, 0 },
  { "bltui", ICLASS_xt_iclass_bsi8u,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bltui_encode_fns, 0, 0 },
  { "beq", ICLASS_xt_iclass_bst8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_beq_encode_fns, 0, 0 },
  { "bne", ICLASS_xt_iclass_bst8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bne_encode_fns, 0, 0 },
  { "bge", ICLASS_xt_iclass_bst8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bge_encode_fns, 0, 0 },
  { "blt", ICLASS_xt_iclass_bst8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_blt_encode_fns, 0, 0 },
  { "bgeu", ICLASS_xt_iclass_bst8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bgeu_encode_fns, 0, 0 },
  { "bltu", ICLASS_xt_iclass_bst8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bltu_encode_fns, 0, 0 },
  { "bany", ICLASS_xt_iclass_bst8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bany_encode_fns, 0, 0 },
  { "bnone", ICLASS_xt_iclass_bst8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bnone_encode_fns, 0, 0 },
  { "ball", ICLASS_xt_iclass_bst8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_ball_encode_fns, 0, 0 },
  { "bnall", ICLASS_xt_iclass_bst8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bnall_encode_fns, 0, 0 },
  { "bbc", ICLASS_xt_iclass_bst8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bbc_encode_fns, 0, 0 },
  { "bbs", ICLASS_xt_iclass_bst8,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bbs_encode_fns, 0, 0 },
  { "beqz", ICLASS_xt_iclass_bsz12,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_beqz_encode_fns, 0, 0 },
  { "bnez", ICLASS_xt_iclass_bsz12,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bnez_encode_fns, 0, 0 },
  { "bgez", ICLASS_xt_iclass_bsz12,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bgez_encode_fns, 0, 0 },
  { "bltz", ICLASS_xt_iclass_bsz12,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bltz_encode_fns, 0, 0 },
  { "call0", ICLASS_xt_iclass_call0,
    XTENSA_OPCODE_IS_CALL,
    Opcode_call0_encode_fns, 0, 0 },
  { "callx0", ICLASS_xt_iclass_callx0,
    XTENSA_OPCODE_IS_CALL,
    Opcode_callx0_encode_fns, 0, 0 },
  { "extui", ICLASS_xt_iclass_exti,
    0,
    Opcode_extui_encode_fns, 0, 0 },
  { "ill", ICLASS_xt_iclass_ill,
    0,
    Opcode_ill_encode_fns, 0, 0 },
  { "j", ICLASS_xt_iclass_jump,
    XTENSA_OPCODE_IS_JUMP,
    Opcode_j_encode_fns, 0, 0 },
  { "jx", ICLASS_xt_iclass_jumpx,
    XTENSA_OPCODE_IS_JUMP,
    Opcode_jx_encode_fns, 0, 0 },
  { "l16ui", ICLASS_xt_iclass_l16ui,
    0,
    Opcode_l16ui_encode_fns, 0, 0 },
  { "l16si", ICLASS_xt_iclass_l16si,
    0,
    Opcode_l16si_encode_fns, 0, 0 },
  { "l32i", ICLASS_xt_iclass_l32i,
    0,
    Opcode_l32i_encode_fns, 0, 0 },
  { "l32r", ICLASS_xt_iclass_l32r,
    0,
    Opcode_l32r_encode_fns, 0, 0 },
  { "l8ui", ICLASS_xt_iclass_l8i,
    0,
    Opcode_l8ui_encode_fns, 0, 0 },
  { "loop", ICLASS_xt_iclass_loop,
    XTENSA_OPCODE_IS_LOOP,
    Opcode_loop_encode_fns, 0, 0 },
  { "loopnez", ICLASS_xt_iclass_loopz,
    XTENSA_OPCODE_IS_LOOP,
    Opcode_loopnez_encode_fns, 0, 0 },
  { "loopgtz", ICLASS_xt_iclass_loopz,
    XTENSA_OPCODE_IS_LOOP,
    Opcode_loopgtz_encode_fns, 0, 0 },
  { "movi", ICLASS_xt_iclass_movi,
    0,
    Opcode_movi_encode_fns, 0, 0 },
  { "moveqz", ICLASS_xt_iclass_movz,
    0,
    Opcode_moveqz_encode_fns, 0, 0 },
  { "movnez", ICLASS_xt_iclass_movz,
    0,
    Opcode_movnez_encode_fns, 0, 0 },
  { "movltz", ICLASS_xt_iclass_movz,
    0,
    Opcode_movltz_encode_fns, 0, 0 },
  { "movgez", ICLASS_xt_iclass_movz,
    0,
    Opcode_movgez_encode_fns, 0, 0 },
  { "neg", ICLASS_xt_iclass_neg,
    0,
    Opcode_neg_encode_fns, 0, 0 },
  { "abs", ICLASS_xt_iclass_neg,
    0,
    Opcode_abs_encode_fns, 0, 0 },
  { "nop", ICLASS_xt_iclass_nop,
    0,
    Opcode_nop_encode_fns, 0, 0 },
  { "ret", ICLASS_xt_iclass_return,
    XTENSA_OPCODE_IS_JUMP,
    Opcode_ret_encode_fns, 0, 0 },
  { "s16i", ICLASS_xt_iclass_s16i,
    0,
    Opcode_s16i_encode_fns, 0, 0 },
  { "s32i", ICLASS_xt_iclass_s32i,
    0,
    Opcode_s32i_encode_fns, 0, 0 },
  { "s8i", ICLASS_xt_iclass_s8i,
    0,
    Opcode_s8i_encode_fns, 0, 0 },
  { "ssr", ICLASS_xt_iclass_sar,
    0,
    Opcode_ssr_encode_fns, 0, 0 },
  { "ssl", ICLASS_xt_iclass_sar,
    0,
    Opcode_ssl_encode_fns, 0, 0 },
  { "ssa8l", ICLASS_xt_iclass_sar,
    0,
    Opcode_ssa8l_encode_fns, 0, 0 },
  { "ssa8b", ICLASS_xt_iclass_sar,
    0,
    Opcode_ssa8b_encode_fns, 0, 0 },
  { "ssai", ICLASS_xt_iclass_sari,
    0,
    Opcode_ssai_encode_fns, 0, 0 },
  { "sll", ICLASS_xt_iclass_shifts,
    0,
    Opcode_sll_encode_fns, 0, 0 },
  { "src", ICLASS_xt_iclass_shiftst,
    0,
    Opcode_src_encode_fns, 0, 0 },
  { "srl", ICLASS_xt_iclass_shiftt,
    0,
    Opcode_srl_encode_fns, 0, 0 },
  { "sra", ICLASS_xt_iclass_shiftt,
    0,
    Opcode_sra_encode_fns, 0, 0 },
  { "slli", ICLASS_xt_iclass_slli,
    0,
    Opcode_slli_encode_fns, 0, 0 },
  { "srai", ICLASS_xt_iclass_srai,
    0,
    Opcode_srai_encode_fns, 0, 0 },
  { "srli", ICLASS_xt_iclass_srli,
    0,
    Opcode_srli_encode_fns, 0, 0 },
  { "memw", ICLASS_xt_iclass_memw,
    0,
    Opcode_memw_encode_fns, 0, 0 },
  { "extw", ICLASS_xt_iclass_extw,
    0,
    Opcode_extw_encode_fns, 0, 0 },
  { "isync", ICLASS_xt_iclass_isync,
    0,
    Opcode_isync_encode_fns, 0, 0 },
  { "rsync", ICLASS_xt_iclass_sync,
    0,
    Opcode_rsync_encode_fns, 0, 0 },
  { "esync", ICLASS_xt_iclass_sync,
    0,
    Opcode_esync_encode_fns, 0, 0 },
  { "dsync", ICLASS_xt_iclass_sync,
    0,
    Opcode_dsync_encode_fns, 0, 0 },
  { "rsil", ICLASS_xt_iclass_rsil,
    0,
    Opcode_rsil_encode_fns, 0, 0 },
  { "rsr.lend", ICLASS_xt_iclass_rsr_lend,
    0,
    Opcode_rsr_lend_encode_fns, 0, 0 },
  { "wsr.lend", ICLASS_xt_iclass_wsr_lend,
    0,
    Opcode_wsr_lend_encode_fns, 0, 0 },
  { "xsr.lend", ICLASS_xt_iclass_xsr_lend,
    0,
    Opcode_xsr_lend_encode_fns, 0, 0 },
  { "rsr.lcount", ICLASS_xt_iclass_rsr_lcount,
    0,
    Opcode_rsr_lcount_encode_fns, 0, 0 },
  { "wsr.lcount", ICLASS_xt_iclass_wsr_lcount,
    0,
    Opcode_wsr_lcount_encode_fns, 0, 0 },
  { "xsr.lcount", ICLASS_xt_iclass_xsr_lcount,
    0,
    Opcode_xsr_lcount_encode_fns, 0, 0 },
  { "rsr.lbeg", ICLASS_xt_iclass_rsr_lbeg,
    0,
    Opcode_rsr_lbeg_encode_fns, 0, 0 },
  { "wsr.lbeg", ICLASS_xt_iclass_wsr_lbeg,
    0,
    Opcode_wsr_lbeg_encode_fns, 0, 0 },
  { "xsr.lbeg", ICLASS_xt_iclass_xsr_lbeg,
    0,
    Opcode_xsr_lbeg_encode_fns, 0, 0 },
  { "rsr.sar", ICLASS_xt_iclass_rsr_sar,
    0,
    Opcode_rsr_sar_encode_fns, 0, 0 },
  { "wsr.sar", ICLASS_xt_iclass_wsr_sar,
    0,
    Opcode_wsr_sar_encode_fns, 0, 0 },
  { "xsr.sar", ICLASS_xt_iclass_xsr_sar,
    0,
    Opcode_xsr_sar_encode_fns, 0, 0 },
  { "rsr.litbase", ICLASS_xt_iclass_rsr_litbase,
    0,
    Opcode_rsr_litbase_encode_fns, 0, 0 },
  { "wsr.litbase", ICLASS_xt_iclass_wsr_litbase,
    0,
    Opcode_wsr_litbase_encode_fns, 0, 0 },
  { "xsr.litbase", ICLASS_xt_iclass_xsr_litbase,
    0,
    Opcode_xsr_litbase_encode_fns, 0, 0 },
  { "rsr.176", ICLASS_xt_iclass_rsr_176,
    0,
    Opcode_rsr_176_encode_fns, 0, 0 },
  { "wsr.176", ICLASS_xt_iclass_wsr_176,
    0,
    Opcode_wsr_176_encode_fns, 0, 0 },
  { "rsr.208", ICLASS_xt_iclass_rsr_208,
    0,
    Opcode_rsr_208_encode_fns, 0, 0 },
  { "rsr.ps", ICLASS_xt_iclass_rsr_ps,
    0,
    Opcode_rsr_ps_encode_fns, 0, 0 },
  { "wsr.ps", ICLASS_xt_iclass_wsr_ps,
    0,
    Opcode_wsr_ps_encode_fns, 0, 0 },
  { "xsr.ps", ICLASS_xt_iclass_xsr_ps,
    0,
    Opcode_xsr_ps_encode_fns, 0, 0 },
  { "rsr.epc1", ICLASS_xt_iclass_rsr_epc1,
    0,
    Opcode_rsr_epc1_encode_fns, 0, 0 },
  { "wsr.epc1", ICLASS_xt_iclass_wsr_epc1,
    0,
    Opcode_wsr_epc1_encode_fns, 0, 0 },
  { "xsr.epc1", ICLASS_xt_iclass_xsr_epc1,
    0,
    Opcode_xsr_epc1_encode_fns, 0, 0 },
  { "rsr.excsave1", ICLASS_xt_iclass_rsr_excsave1,
    0,
    Opcode_rsr_excsave1_encode_fns, 0, 0 },
  { "wsr.excsave1", ICLASS_xt_iclass_wsr_excsave1,
    0,
    Opcode_wsr_excsave1_encode_fns, 0, 0 },
  { "xsr.excsave1", ICLASS_xt_iclass_xsr_excsave1,
    0,
    Opcode_xsr_excsave1_encode_fns, 0, 0 },
  { "rsr.epc2", ICLASS_xt_iclass_rsr_epc2,
    0,
    Opcode_rsr_epc2_encode_fns, 0, 0 },
  { "wsr.epc2", ICLASS_xt_iclass_wsr_epc2,
    0,
    Opcode_wsr_epc2_encode_fns, 0, 0 },
  { "xsr.epc2", ICLASS_xt_iclass_xsr_epc2,
    0,
    Opcode_xsr_epc2_encode_fns, 0, 0 },
  { "rsr.excsave2", ICLASS_xt_iclass_rsr_excsave2,
    0,
    Opcode_rsr_excsave2_encode_fns, 0, 0 },
  { "wsr.excsave2", ICLASS_xt_iclass_wsr_excsave2,
    0,
    Opcode_wsr_excsave2_encode_fns, 0, 0 },
  { "xsr.excsave2", ICLASS_xt_iclass_xsr_excsave2,
    0,
    Opcode_xsr_excsave2_encode_fns, 0, 0 },
  { "rsr.eps2", ICLASS_xt_iclass_rsr_eps2,
    0,
    Opcode_rsr_eps2_encode_fns, 0, 0 },
  { "wsr.eps2", ICLASS_xt_iclass_wsr_eps2,
    0,
    Opcode_wsr_eps2_encode_fns, 0, 0 },
  { "xsr.eps2", ICLASS_xt_iclass_xsr_eps2,
    0,
    Opcode_xsr_eps2_encode_fns, 0, 0 },
  { "rsr.excvaddr", ICLASS_xt_iclass_rsr_excvaddr,
    0,
    Opcode_rsr_excvaddr_encode_fns, 0, 0 },
  { "wsr.excvaddr", ICLASS_xt_iclass_wsr_excvaddr,
    0,
    Opcode_wsr_excvaddr_encode_fns, 0, 0 },
  { "xsr.excvaddr", ICLASS_xt_iclass_xsr_excvaddr,
    0,
    Opcode_xsr_excvaddr_encode_fns, 0, 0 },
  { "rsr.depc", ICLASS_xt_iclass_rsr_depc,
    0,
    Opcode_rsr_depc_encode_fns, 0, 0 },
  { "wsr.depc", ICLASS_xt_iclass_wsr_depc,
    0,
    Opcode_wsr_depc_encode_fns, 0, 0 },
  { "xsr.depc", ICLASS_xt_iclass_xsr_depc,
    0,
    Opcode_xsr_depc_encode_fns, 0, 0 },
  { "rsr.exccause", ICLASS_xt_iclass_rsr_exccause,
    0,
    Opcode_rsr_exccause_encode_fns, 0, 0 },
  { "wsr.exccause", ICLASS_xt_iclass_wsr_exccause,
    0,
    Opcode_wsr_exccause_encode_fns, 0, 0 },
  { "xsr.exccause", ICLASS_xt_iclass_xsr_exccause,
    0,
    Opcode_xsr_exccause_encode_fns, 0, 0 },
  { "rsr.misc0", ICLASS_xt_iclass_rsr_misc0,
    0,
    Opcode_rsr_misc0_encode_fns, 0, 0 },
  { "wsr.misc0", ICLASS_xt_iclass_wsr_misc0,
    0,
    Opcode_wsr_misc0_encode_fns, 0, 0 },
  { "xsr.misc0", ICLASS_xt_iclass_xsr_misc0,
    0,
    Opcode_xsr_misc0_encode_fns, 0, 0 },
  { "rsr.misc1", ICLASS_xt_iclass_rsr_misc1,
    0,
    Opcode_rsr_misc1_encode_fns, 0, 0 },
  { "wsr.misc1", ICLASS_xt_iclass_wsr_misc1,
    0,
    Opcode_wsr_misc1_encode_fns, 0, 0 },
  { "xsr.misc1", ICLASS_xt_iclass_xsr_misc1,
    0,
    Opcode_xsr_misc1_encode_fns, 0, 0 },
  { "rsr.prid", ICLASS_xt_iclass_rsr_prid,
    0,
    Opcode_rsr_prid_encode_fns, 0, 0 },
  { "rsr.vecbase", ICLASS_xt_iclass_rsr_vecbase,
    0,
    Opcode_rsr_vecbase_encode_fns, 0, 0 },
  { "wsr.vecbase", ICLASS_xt_iclass_wsr_vecbase,
    0,
    Opcode_wsr_vecbase_encode_fns, 0, 0 },
  { "xsr.vecbase", ICLASS_xt_iclass_xsr_vecbase,
    0,
    Opcode_xsr_vecbase_encode_fns, 0, 0 },
  { "mul16u", ICLASS_xt_mul16,
    0,
    Opcode_mul16u_encode_fns, 0, 0 },
  { "mul16s", ICLASS_xt_mul16,
    0,
    Opcode_mul16s_encode_fns, 0, 0 },
  { "mull", ICLASS_xt_mul32,
    0,
    Opcode_mull_encode_fns, 0, 0 },
  { "rfi", ICLASS_xt_iclass_rfi,
    XTENSA_OPCODE_IS_JUMP,
    Opcode_rfi_encode_fns, 0, 0 },
  { "waiti", ICLASS_xt_iclass_wait,
    0,
    Opcode_waiti_encode_fns, 0, 0 },
  { "rsr.interrupt", ICLASS_xt_iclass_rsr_interrupt,
    0,
    Opcode_rsr_interrupt_encode_fns, 0, 0 },
  { "wsr.intset", ICLASS_xt_iclass_wsr_intset,
    0,
    Opcode_wsr_intset_encode_fns, 0, 0 },
  { "wsr.intclear", ICLASS_xt_iclass_wsr_intclear,
    0,
    Opcode_wsr_intclear_encode_fns, 0, 0 },
  { "rsr.intenable", ICLASS_xt_iclass_rsr_intenable,
    0,
    Opcode_rsr_intenable_encode_fns, 0, 0 },
  { "wsr.intenable", ICLASS_xt_iclass_wsr_intenable,
    0,
    Opcode_wsr_intenable_encode_fns, 0, 0 },
  { "xsr.intenable", ICLASS_xt_iclass_xsr_intenable,
    0,
    Opcode_xsr_intenable_encode_fns, 0, 0 },
  { "break", ICLASS_xt_iclass_break,
    0,
    Opcode_break_encode_fns, 0, 0 },
  { "break.n", ICLASS_xt_iclass_break_n,
    0,
    Opcode_break_n_encode_fns, 0, 0 },
  { "rsr.debugcause", ICLASS_xt_iclass_rsr_debugcause,
    0,
    Opcode_rsr_debugcause_encode_fns, 0, 0 },
  { "wsr.debugcause", ICLASS_xt_iclass_wsr_debugcause,
    0,
    Opcode_wsr_debugcause_encode_fns, 0, 0 },
  { "xsr.debugcause", ICLASS_xt_iclass_xsr_debugcause,
    0,
    Opcode_xsr_debugcause_encode_fns, 0, 0 },
  { "rsr.icount", ICLASS_xt_iclass_rsr_icount,
    0,
    Opcode_rsr_icount_encode_fns, 0, 0 },
  { "wsr.icount", ICLASS_xt_iclass_wsr_icount,
    0,
    Opcode_wsr_icount_encode_fns, 0, 0 },
  { "xsr.icount", ICLASS_xt_iclass_xsr_icount,
    0,
    Opcode_xsr_icount_encode_fns, 0, 0 },
  { "rsr.icountlevel", ICLASS_xt_iclass_rsr_icountlevel,
    0,
    Opcode_rsr_icountlevel_encode_fns, 0, 0 },
  { "wsr.icountlevel", ICLASS_xt_iclass_wsr_icountlevel,
    0,
    Opcode_wsr_icountlevel_encode_fns, 0, 0 },
  { "xsr.icountlevel", ICLASS_xt_iclass_xsr_icountlevel,
    0,
    Opcode_xsr_icountlevel_encode_fns, 0, 0 },
  { "rsr.ddr", ICLASS_xt_iclass_rsr_ddr,
    0,
    Opcode_rsr_ddr_encode_fns, 0, 0 },
  { "wsr.ddr", ICLASS_xt_iclass_wsr_ddr,
    0,
    Opcode_wsr_ddr_encode_fns, 0, 0 },
  { "xsr.ddr", ICLASS_xt_iclass_xsr_ddr,
    0,
    Opcode_xsr_ddr_encode_fns, 0, 0 },
  { "rfdo", ICLASS_xt_iclass_rfdo,
    XTENSA_OPCODE_IS_JUMP,
    Opcode_rfdo_encode_fns, 0, 0 },
  { "rfdd", ICLASS_xt_iclass_rfdd,
    XTENSA_OPCODE_IS_JUMP,
    Opcode_rfdd_encode_fns, 0, 0 },
  { "andb", ICLASS_xt_iclass_bbool1,
    0,
    Opcode_andb_encode_fns, 0, 0 },
  { "andbc", ICLASS_xt_iclass_bbool1,
    0,
    Opcode_andbc_encode_fns, 0, 0 },
  { "orb", ICLASS_xt_iclass_bbool1,
    0,
    Opcode_orb_encode_fns, 0, 0 },
  { "orbc", ICLASS_xt_iclass_bbool1,
    0,
    Opcode_orbc_encode_fns, 0, 0 },
  { "xorb", ICLASS_xt_iclass_bbool1,
    0,
    Opcode_xorb_encode_fns, 0, 0 },
  { "any4", ICLASS_xt_iclass_bbool4,
    0,
    Opcode_any4_encode_fns, 0, 0 },
  { "all4", ICLASS_xt_iclass_bbool4,
    0,
    Opcode_all4_encode_fns, 0, 0 },
  { "any8", ICLASS_xt_iclass_bbool8,
    0,
    Opcode_any8_encode_fns, 0, 0 },
  { "all8", ICLASS_xt_iclass_bbool8,
    0,
    Opcode_all8_encode_fns, 0, 0 },
  { "bf", ICLASS_xt_iclass_bbranch,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bf_encode_fns, 0, 0 },
  { "bt", ICLASS_xt_iclass_bbranch,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_bt_encode_fns, 0, 0 },
  { "movf", ICLASS_xt_iclass_bmove,
    0,
    Opcode_movf_encode_fns, 0, 0 },
  { "movt", ICLASS_xt_iclass_bmove,
    0,
    Opcode_movt_encode_fns, 0, 0 },
  { "rsr.br", ICLASS_xt_iclass_RSR_BR,
    0,
    Opcode_rsr_br_encode_fns, 0, 0 },
  { "wsr.br", ICLASS_xt_iclass_WSR_BR,
    0,
    Opcode_wsr_br_encode_fns, 0, 0 },
  { "xsr.br", ICLASS_xt_iclass_XSR_BR,
    0,
    Opcode_xsr_br_encode_fns, 0, 0 },
  { "rsr.ccount", ICLASS_xt_iclass_rsr_ccount,
    0,
    Opcode_rsr_ccount_encode_fns, 0, 0 },
  { "wsr.ccount", ICLASS_xt_iclass_wsr_ccount,
    0,
    Opcode_wsr_ccount_encode_fns, 0, 0 },
  { "xsr.ccount", ICLASS_xt_iclass_xsr_ccount,
    0,
    Opcode_xsr_ccount_encode_fns, 0, 0 },
  { "rsr.ccompare0", ICLASS_xt_iclass_rsr_ccompare0,
    0,
    Opcode_rsr_ccompare0_encode_fns, 0, 0 },
  { "wsr.ccompare0", ICLASS_xt_iclass_wsr_ccompare0,
    0,
    Opcode_wsr_ccompare0_encode_fns, 0, 0 },
  { "xsr.ccompare0", ICLASS_xt_iclass_xsr_ccompare0,
    0,
    Opcode_xsr_ccompare0_encode_fns, 0, 0 },
  { "rsr.ccompare1", ICLASS_xt_iclass_rsr_ccompare1,
    0,
    Opcode_rsr_ccompare1_encode_fns, 0, 0 },
  { "wsr.ccompare1", ICLASS_xt_iclass_wsr_ccompare1,
    0,
    Opcode_wsr_ccompare1_encode_fns, 0, 0 },
  { "xsr.ccompare1", ICLASS_xt_iclass_xsr_ccompare1,
    0,
    Opcode_xsr_ccompare1_encode_fns, 0, 0 },
  { "ipf", ICLASS_xt_iclass_icache,
    0,
    Opcode_ipf_encode_fns, 0, 0 },
  { "ihi", ICLASS_xt_iclass_icache,
    0,
    Opcode_ihi_encode_fns, 0, 0 },
  { "iii", ICLASS_xt_iclass_icache_inv,
    0,
    Opcode_iii_encode_fns, 0, 0 },
  { "lict", ICLASS_xt_iclass_licx,
    0,
    Opcode_lict_encode_fns, 0, 0 },
  { "licw", ICLASS_xt_iclass_licx,
    0,
    Opcode_licw_encode_fns, 0, 0 },
  { "sict", ICLASS_xt_iclass_sicx,
    0,
    Opcode_sict_encode_fns, 0, 0 },
  { "sicw", ICLASS_xt_iclass_sicx,
    0,
    Opcode_sicw_encode_fns, 0, 0 },
  { "dhwb", ICLASS_xt_iclass_dcache,
    0,
    Opcode_dhwb_encode_fns, 0, 0 },
  { "dhwbi", ICLASS_xt_iclass_dcache,
    0,
    Opcode_dhwbi_encode_fns, 0, 0 },
  { "diwb", ICLASS_xt_iclass_dcache_ind,
    0,
    Opcode_diwb_encode_fns, 0, 0 },
  { "diwbi", ICLASS_xt_iclass_dcache_ind,
    0,
    Opcode_diwbi_encode_fns, 0, 0 },
  { "dhi", ICLASS_xt_iclass_dcache_inv,
    0,
    Opcode_dhi_encode_fns, 0, 0 },
  { "dii", ICLASS_xt_iclass_dcache_inv,
    0,
    Opcode_dii_encode_fns, 0, 0 },
  { "dpfr", ICLASS_xt_iclass_dpf,
    0,
    Opcode_dpfr_encode_fns, 0, 0 },
  { "dpfw", ICLASS_xt_iclass_dpf,
    0,
    Opcode_dpfw_encode_fns, 0, 0 },
  { "dpfro", ICLASS_xt_iclass_dpf,
    0,
    Opcode_dpfro_encode_fns, 0, 0 },
  { "dpfwo", ICLASS_xt_iclass_dpf,
    0,
    Opcode_dpfwo_encode_fns, 0, 0 },
  { "sdct", ICLASS_xt_iclass_sdct,
    0,
    Opcode_sdct_encode_fns, 0, 0 },
  { "ldct", ICLASS_xt_iclass_ldct,
    0,
    Opcode_ldct_encode_fns, 0, 0 },
  { "wsr.ptevaddr", ICLASS_xt_iclass_wsr_ptevaddr,
    0,
    Opcode_wsr_ptevaddr_encode_fns, 0, 0 },
  { "rsr.ptevaddr", ICLASS_xt_iclass_rsr_ptevaddr,
    0,
    Opcode_rsr_ptevaddr_encode_fns, 0, 0 },
  { "xsr.ptevaddr", ICLASS_xt_iclass_xsr_ptevaddr,
    0,
    Opcode_xsr_ptevaddr_encode_fns, 0, 0 },
  { "rsr.rasid", ICLASS_xt_iclass_rsr_rasid,
    0,
    Opcode_rsr_rasid_encode_fns, 0, 0 },
  { "wsr.rasid", ICLASS_xt_iclass_wsr_rasid,
    0,
    Opcode_wsr_rasid_encode_fns, 0, 0 },
  { "xsr.rasid", ICLASS_xt_iclass_xsr_rasid,
    0,
    Opcode_xsr_rasid_encode_fns, 0, 0 },
  { "rsr.itlbcfg", ICLASS_xt_iclass_rsr_itlbcfg,
    0,
    Opcode_rsr_itlbcfg_encode_fns, 0, 0 },
  { "wsr.itlbcfg", ICLASS_xt_iclass_wsr_itlbcfg,
    0,
    Opcode_wsr_itlbcfg_encode_fns, 0, 0 },
  { "xsr.itlbcfg", ICLASS_xt_iclass_xsr_itlbcfg,
    0,
    Opcode_xsr_itlbcfg_encode_fns, 0, 0 },
  { "rsr.dtlbcfg", ICLASS_xt_iclass_rsr_dtlbcfg,
    0,
    Opcode_rsr_dtlbcfg_encode_fns, 0, 0 },
  { "wsr.dtlbcfg", ICLASS_xt_iclass_wsr_dtlbcfg,
    0,
    Opcode_wsr_dtlbcfg_encode_fns, 0, 0 },
  { "xsr.dtlbcfg", ICLASS_xt_iclass_xsr_dtlbcfg,
    0,
    Opcode_xsr_dtlbcfg_encode_fns, 0, 0 },
  { "idtlb", ICLASS_xt_iclass_idtlb,
    0,
    Opcode_idtlb_encode_fns, 0, 0 },
  { "pdtlb", ICLASS_xt_iclass_rdtlb,
    0,
    Opcode_pdtlb_encode_fns, 0, 0 },
  { "rdtlb0", ICLASS_xt_iclass_rdtlb,
    0,
    Opcode_rdtlb0_encode_fns, 0, 0 },
  { "rdtlb1", ICLASS_xt_iclass_rdtlb,
    0,
    Opcode_rdtlb1_encode_fns, 0, 0 },
  { "wdtlb", ICLASS_xt_iclass_wdtlb,
    0,
    Opcode_wdtlb_encode_fns, 0, 0 },
  { "iitlb", ICLASS_xt_iclass_iitlb,
    0,
    Opcode_iitlb_encode_fns, 0, 0 },
  { "pitlb", ICLASS_xt_iclass_ritlb,
    0,
    Opcode_pitlb_encode_fns, 0, 0 },
  { "ritlb0", ICLASS_xt_iclass_ritlb,
    0,
    Opcode_ritlb0_encode_fns, 0, 0 },
  { "ritlb1", ICLASS_xt_iclass_ritlb,
    0,
    Opcode_ritlb1_encode_fns, 0, 0 },
  { "witlb", ICLASS_xt_iclass_witlb,
    0,
    Opcode_witlb_encode_fns, 0, 0 },
  { "ldpte", ICLASS_xt_iclass_ldpte,
    0,
    Opcode_ldpte_encode_fns, 0, 0 },
  { "hwwitlba", ICLASS_xt_iclass_hwwitlba,
    XTENSA_OPCODE_IS_BRANCH,
    Opcode_hwwitlba_encode_fns, 0, 0 },
  { "hwwdtlba", ICLASS_xt_iclass_hwwdtlba,
    0,
    Opcode_hwwdtlba_encode_fns, 0, 0 },
  { "rsr.cpenable", ICLASS_xt_iclass_rsr_cpenable,
    0,
    Opcode_rsr_cpenable_encode_fns, 0, 0 },
  { "wsr.cpenable", ICLASS_xt_iclass_wsr_cpenable,
    0,
    Opcode_wsr_cpenable_encode_fns, 0, 0 },
  { "xsr.cpenable", ICLASS_xt_iclass_xsr_cpenable,
    0,
    Opcode_xsr_cpenable_encode_fns, 0, 0 },
  { "clamps", ICLASS_xt_iclass_clamp,
    0,
    Opcode_clamps_encode_fns, 0, 0 },
  { "min", ICLASS_xt_iclass_minmax,
    0,
    Opcode_min_encode_fns, 0, 0 },
  { "max", ICLASS_xt_iclass_minmax,
    0,
    Opcode_max_encode_fns, 0, 0 },
  { "minu", ICLASS_xt_iclass_minmax,
    0,
    Opcode_minu_encode_fns, 0, 0 },
  { "maxu", ICLASS_xt_iclass_minmax,
    0,
    Opcode_maxu_encode_fns, 0, 0 },
  { "nsa", ICLASS_xt_iclass_nsa,
    0,
    Opcode_nsa_encode_fns, 0, 0 },
  { "nsau", ICLASS_xt_iclass_nsa,
    0,
    Opcode_nsau_encode_fns, 0, 0 },
  { "sext", ICLASS_xt_iclass_sx,
    0,
    Opcode_sext_encode_fns, 0, 0 },
  { "l32ai", ICLASS_xt_iclass_l32ai,
    0,
    Opcode_l32ai_encode_fns, 0, 0 },
  { "s32ri", ICLASS_xt_iclass_s32ri,
    0,
    Opcode_s32ri_encode_fns, 0, 0 },
  { "s32c1i", ICLASS_xt_iclass_s32c1i,
    0,
    Opcode_s32c1i_encode_fns, 0, 0 },
  { "rsr.scompare1", ICLASS_xt_iclass_rsr_scompare1,
    0,
    Opcode_rsr_scompare1_encode_fns, 0, 0 },
  { "wsr.scompare1", ICLASS_xt_iclass_wsr_scompare1,
    0,
    Opcode_wsr_scompare1_encode_fns, 0, 0 },
  { "xsr.scompare1", ICLASS_xt_iclass_xsr_scompare1,
    0,
    Opcode_xsr_scompare1_encode_fns, 0, 0 },
  { "rsr.atomctl", ICLASS_xt_iclass_rsr_atomctl,
    0,
    Opcode_rsr_atomctl_encode_fns, 0, 0 },
  { "wsr.atomctl", ICLASS_xt_iclass_wsr_atomctl,
    0,
    Opcode_wsr_atomctl_encode_fns, 0, 0 },
  { "xsr.atomctl", ICLASS_xt_iclass_xsr_atomctl,
    0,
    Opcode_xsr_atomctl_encode_fns, 0, 0 },
  { "rer", ICLASS_xt_iclass_rer,
    0,
    Opcode_rer_encode_fns, 0, 0 },
  { "wer", ICLASS_xt_iclass_wer,
    0,
    Opcode_wer_encode_fns, 0, 0 },
  { "rur.ae_ovf_sar", ICLASS_rur_ae_ovf_sar,
    0,
    Opcode_rur_ae_ovf_sar_encode_fns, 0, 0 },
  { "wur.ae_ovf_sar", ICLASS_wur_ae_ovf_sar,
    0,
    Opcode_wur_ae_ovf_sar_encode_fns, 0, 0 },
  { "rur.ae_bithead", ICLASS_rur_ae_bithead,
    0,
    Opcode_rur_ae_bithead_encode_fns, 0, 0 },
  { "wur.ae_bithead", ICLASS_wur_ae_bithead,
    0,
    Opcode_wur_ae_bithead_encode_fns, 0, 0 },
  { "rur.ae_ts_fts_bu_bp", ICLASS_rur_ae_ts_fts_bu_bp,
    0,
    Opcode_rur_ae_ts_fts_bu_bp_encode_fns, 0, 0 },
  { "wur.ae_ts_fts_bu_bp", ICLASS_wur_ae_ts_fts_bu_bp,
    0,
    Opcode_wur_ae_ts_fts_bu_bp_encode_fns, 0, 0 },
  { "rur.ae_sd_no", ICLASS_rur_ae_sd_no,
    0,
    Opcode_rur_ae_sd_no_encode_fns, 0, 0 },
  { "wur.ae_sd_no", ICLASS_wur_ae_sd_no,
    0,
    Opcode_wur_ae_sd_no_encode_fns, 0, 0 },
  { "rur.ae_overflow", ICLASS_ae_iclass_rur_ae_overflow,
    0,
    Opcode_rur_ae_overflow_encode_fns, 0, 0 },
  { "wur.ae_overflow", ICLASS_ae_iclass_wur_ae_overflow,
    0,
    Opcode_wur_ae_overflow_encode_fns, 0, 0 },
  { "rur.ae_sar", ICLASS_ae_iclass_rur_ae_sar,
    0,
    Opcode_rur_ae_sar_encode_fns, 0, 0 },
  { "wur.ae_sar", ICLASS_ae_iclass_wur_ae_sar,
    0,
    Opcode_wur_ae_sar_encode_fns, 0, 0 },
  { "rur.ae_bitptr", ICLASS_ae_iclass_rur_ae_bitptr,
    0,
    Opcode_rur_ae_bitptr_encode_fns, 0, 0 },
  { "wur.ae_bitptr", ICLASS_ae_iclass_wur_ae_bitptr,
    0,
    Opcode_wur_ae_bitptr_encode_fns, 0, 0 },
  { "rur.ae_bitsused", ICLASS_ae_iclass_rur_ae_bitsused,
    0,
    Opcode_rur_ae_bitsused_encode_fns, 0, 0 },
  { "wur.ae_bitsused", ICLASS_ae_iclass_wur_ae_bitsused,
    0,
    Opcode_wur_ae_bitsused_encode_fns, 0, 0 },
  { "rur.ae_tablesize", ICLASS_ae_iclass_rur_ae_tablesize,
    0,
    Opcode_rur_ae_tablesize_encode_fns, 0, 0 },
  { "wur.ae_tablesize", ICLASS_ae_iclass_wur_ae_tablesize,
    0,
    Opcode_wur_ae_tablesize_encode_fns, 0, 0 },
  { "rur.ae_first_ts", ICLASS_ae_iclass_rur_ae_first_ts,
    0,
    Opcode_rur_ae_first_ts_encode_fns, 0, 0 },
  { "wur.ae_first_ts", ICLASS_ae_iclass_wur_ae_first_ts,
    0,
    Opcode_wur_ae_first_ts_encode_fns, 0, 0 },
  { "rur.ae_nextoffset", ICLASS_ae_iclass_rur_ae_nextoffset,
    0,
    Opcode_rur_ae_nextoffset_encode_fns, 0, 0 },
  { "wur.ae_nextoffset", ICLASS_ae_iclass_wur_ae_nextoffset,
    0,
    Opcode_wur_ae_nextoffset_encode_fns, 0, 0 },
  { "rur.ae_searchdone", ICLASS_ae_iclass_rur_ae_searchdone,
    0,
    Opcode_rur_ae_searchdone_encode_fns, 0, 0 },
  { "wur.ae_searchdone", ICLASS_ae_iclass_wur_ae_searchdone,
    0,
    Opcode_wur_ae_searchdone_encode_fns, 0, 0 },
  { "ae_lp16f.i", ICLASS_ae_iclass_lp16f_i,
    0,
    Opcode_ae_lp16f_i_encode_fns, 0, 0 },
  { "ae_lp16f.iu", ICLASS_ae_iclass_lp16f_iu,
    0,
    Opcode_ae_lp16f_iu_encode_fns, 0, 0 },
  { "ae_lp16f.x", ICLASS_ae_iclass_lp16f_x,
    0,
    Opcode_ae_lp16f_x_encode_fns, 0, 0 },
  { "ae_lp16f.xu", ICLASS_ae_iclass_lp16f_xu,
    0,
    Opcode_ae_lp16f_xu_encode_fns, 0, 0 },
  { "ae_lp24.i", ICLASS_ae_iclass_lp24_i,
    0,
    Opcode_ae_lp24_i_encode_fns, 0, 0 },
  { "ae_lp24.iu", ICLASS_ae_iclass_lp24_iu,
    0,
    Opcode_ae_lp24_iu_encode_fns, 0, 0 },
  { "ae_lp24.x", ICLASS_ae_iclass_lp24_x,
    0,
    Opcode_ae_lp24_x_encode_fns, 0, 0 },
  { "ae_lp24.xu", ICLASS_ae_iclass_lp24_xu,
    0,
    Opcode_ae_lp24_xu_encode_fns, 0, 0 },
  { "ae_lp24f.i", ICLASS_ae_iclass_lp24f_i,
    0,
    Opcode_ae_lp24f_i_encode_fns, 0, 0 },
  { "ae_lp24f.iu", ICLASS_ae_iclass_lp24f_iu,
    0,
    Opcode_ae_lp24f_iu_encode_fns, 0, 0 },
  { "ae_lp24f.x", ICLASS_ae_iclass_lp24f_x,
    0,
    Opcode_ae_lp24f_x_encode_fns, 0, 0 },
  { "ae_lp24f.xu", ICLASS_ae_iclass_lp24f_xu,
    0,
    Opcode_ae_lp24f_xu_encode_fns, 0, 0 },
  { "ae_lp16x2f.i", ICLASS_ae_iclass_lp16x2f_i,
    0,
    Opcode_ae_lp16x2f_i_encode_fns, 0, 0 },
  { "ae_lp16x2f.iu", ICLASS_ae_iclass_lp16x2f_iu,
    0,
    Opcode_ae_lp16x2f_iu_encode_fns, 0, 0 },
  { "ae_lp16x2f.x", ICLASS_ae_iclass_lp16x2f_x,
    0,
    Opcode_ae_lp16x2f_x_encode_fns, 0, 0 },
  { "ae_lp16x2f.xu", ICLASS_ae_iclass_lp16x2f_xu,
    0,
    Opcode_ae_lp16x2f_xu_encode_fns, 0, 0 },
  { "ae_lp24x2f.i", ICLASS_ae_iclass_lp24x2f_i,
    0,
    Opcode_ae_lp24x2f_i_encode_fns, 0, 0 },
  { "ae_lp24x2f.iu", ICLASS_ae_iclass_lp24x2f_iu,
    0,
    Opcode_ae_lp24x2f_iu_encode_fns, 0, 0 },
  { "ae_lp24x2f.x", ICLASS_ae_iclass_lp24x2f_x,
    0,
    Opcode_ae_lp24x2f_x_encode_fns, 0, 0 },
  { "ae_lp24x2f.xu", ICLASS_ae_iclass_lp24x2f_xu,
    0,
    Opcode_ae_lp24x2f_xu_encode_fns, 0, 0 },
  { "ae_lp24x2.i", ICLASS_ae_iclass_lp24x2_i,
    0,
    Opcode_ae_lp24x2_i_encode_fns, 0, 0 },
  { "ae_lp24x2.iu", ICLASS_ae_iclass_lp24x2_iu,
    0,
    Opcode_ae_lp24x2_iu_encode_fns, 0, 0 },
  { "ae_lp24x2.x", ICLASS_ae_iclass_lp24x2_x,
    0,
    Opcode_ae_lp24x2_x_encode_fns, 0, 0 },
  { "ae_lp24x2.xu", ICLASS_ae_iclass_lp24x2_xu,
    0,
    Opcode_ae_lp24x2_xu_encode_fns, 0, 0 },
  { "ae_sp16x2f.i", ICLASS_ae_iclass_sp16x2f_i,
    0,
    Opcode_ae_sp16x2f_i_encode_fns, 0, 0 },
  { "ae_sp16x2f.iu", ICLASS_ae_iclass_sp16x2f_iu,
    0,
    Opcode_ae_sp16x2f_iu_encode_fns, 0, 0 },
  { "ae_sp16x2f.x", ICLASS_ae_iclass_sp16x2f_x,
    0,
    Opcode_ae_sp16x2f_x_encode_fns, 0, 0 },
  { "ae_sp16x2f.xu", ICLASS_ae_iclass_sp16x2f_xu,
    0,
    Opcode_ae_sp16x2f_xu_encode_fns, 0, 0 },
  { "ae_sp24x2s.i", ICLASS_ae_iclass_sp24x2s_i,
    0,
    Opcode_ae_sp24x2s_i_encode_fns, 0, 0 },
  { "ae_sp24x2s.iu", ICLASS_ae_iclass_sp24x2s_iu,
    0,
    Opcode_ae_sp24x2s_iu_encode_fns, 0, 0 },
  { "ae_sp24x2s.x", ICLASS_ae_iclass_sp24x2s_x,
    0,
    Opcode_ae_sp24x2s_x_encode_fns, 0, 0 },
  { "ae_sp24x2s.xu", ICLASS_ae_iclass_sp24x2s_xu,
    0,
    Opcode_ae_sp24x2s_xu_encode_fns, 0, 0 },
  { "ae_sp24x2f.i", ICLASS_ae_iclass_sp24x2f_i,
    0,
    Opcode_ae_sp24x2f_i_encode_fns, 0, 0 },
  { "ae_sp24x2f.iu", ICLASS_ae_iclass_sp24x2f_iu,
    0,
    Opcode_ae_sp24x2f_iu_encode_fns, 0, 0 },
  { "ae_sp24x2f.x", ICLASS_ae_iclass_sp24x2f_x,
    0,
    Opcode_ae_sp24x2f_x_encode_fns, 0, 0 },
  { "ae_sp24x2f.xu", ICLASS_ae_iclass_sp24x2f_xu,
    0,
    Opcode_ae_sp24x2f_xu_encode_fns, 0, 0 },
  { "ae_sp16f.l.i", ICLASS_ae_iclass_sp16f_l_i,
    0,
    Opcode_ae_sp16f_l_i_encode_fns, 0, 0 },
  { "ae_sp16f.l.iu", ICLASS_ae_iclass_sp16f_l_iu,
    0,
    Opcode_ae_sp16f_l_iu_encode_fns, 0, 0 },
  { "ae_sp16f.l.x", ICLASS_ae_iclass_sp16f_l_x,
    0,
    Opcode_ae_sp16f_l_x_encode_fns, 0, 0 },
  { "ae_sp16f.l.xu", ICLASS_ae_iclass_sp16f_l_xu,
    0,
    Opcode_ae_sp16f_l_xu_encode_fns, 0, 0 },
  { "ae_sp24s.l.i", ICLASS_ae_iclass_sp24s_l_i,
    0,
    Opcode_ae_sp24s_l_i_encode_fns, 0, 0 },
  { "ae_sp24s.l.iu", ICLASS_ae_iclass_sp24s_l_iu,
    0,
    Opcode_ae_sp24s_l_iu_encode_fns, 0, 0 },
  { "ae_sp24s.l.x", ICLASS_ae_iclass_sp24s_l_x,
    0,
    Opcode_ae_sp24s_l_x_encode_fns, 0, 0 },
  { "ae_sp24s.l.xu", ICLASS_ae_iclass_sp24s_l_xu,
    0,
    Opcode_ae_sp24s_l_xu_encode_fns, 0, 0 },
  { "ae_sp24f.l.i", ICLASS_ae_iclass_sp24f_l_i,
    0,
    Opcode_ae_sp24f_l_i_encode_fns, 0, 0 },
  { "ae_sp24f.l.iu", ICLASS_ae_iclass_sp24f_l_iu,
    0,
    Opcode_ae_sp24f_l_iu_encode_fns, 0, 0 },
  { "ae_sp24f.l.x", ICLASS_ae_iclass_sp24f_l_x,
    0,
    Opcode_ae_sp24f_l_x_encode_fns, 0, 0 },
  { "ae_sp24f.l.xu", ICLASS_ae_iclass_sp24f_l_xu,
    0,
    Opcode_ae_sp24f_l_xu_encode_fns, 0, 0 },
  { "ae_lq56.i", ICLASS_ae_iclass_lq56_i,
    0,
    Opcode_ae_lq56_i_encode_fns, 0, 0 },
  { "ae_lq56.iu", ICLASS_ae_iclass_lq56_iu,
    0,
    Opcode_ae_lq56_iu_encode_fns, 0, 0 },
  { "ae_lq56.x", ICLASS_ae_iclass_lq56_x,
    0,
    Opcode_ae_lq56_x_encode_fns, 0, 0 },
  { "ae_lq56.xu", ICLASS_ae_iclass_lq56_xu,
    0,
    Opcode_ae_lq56_xu_encode_fns, 0, 0 },
  { "ae_lq32f.i", ICLASS_ae_iclass_lq32f_i,
    0,
    Opcode_ae_lq32f_i_encode_fns, 0, 0 },
  { "ae_lq32f.iu", ICLASS_ae_iclass_lq32f_iu,
    0,
    Opcode_ae_lq32f_iu_encode_fns, 0, 0 },
  { "ae_lq32f.x", ICLASS_ae_iclass_lq32f_x,
    0,
    Opcode_ae_lq32f_x_encode_fns, 0, 0 },
  { "ae_lq32f.xu", ICLASS_ae_iclass_lq32f_xu,
    0,
    Opcode_ae_lq32f_xu_encode_fns, 0, 0 },
  { "ae_sq56s.i", ICLASS_ae_iclass_sq56s_i,
    0,
    Opcode_ae_sq56s_i_encode_fns, 0, 0 },
  { "ae_sq56s.iu", ICLASS_ae_iclass_sq56s_iu,
    0,
    Opcode_ae_sq56s_iu_encode_fns, 0, 0 },
  { "ae_sq56s.x", ICLASS_ae_iclass_sq56s_x,
    0,
    Opcode_ae_sq56s_x_encode_fns, 0, 0 },
  { "ae_sq56s.xu", ICLASS_ae_iclass_sq56s_xu,
    0,
    Opcode_ae_sq56s_xu_encode_fns, 0, 0 },
  { "ae_sq32f.i", ICLASS_ae_iclass_sq32f_i,
    0,
    Opcode_ae_sq32f_i_encode_fns, 0, 0 },
  { "ae_sq32f.iu", ICLASS_ae_iclass_sq32f_iu,
    0,
    Opcode_ae_sq32f_iu_encode_fns, 0, 0 },
  { "ae_sq32f.x", ICLASS_ae_iclass_sq32f_x,
    0,
    Opcode_ae_sq32f_x_encode_fns, 0, 0 },
  { "ae_sq32f.xu", ICLASS_ae_iclass_sq32f_xu,
    0,
    Opcode_ae_sq32f_xu_encode_fns, 0, 0 },
  { "ae_zerop48", ICLASS_ae_iclass_zerop48,
    0,
    Opcode_ae_zerop48_encode_fns, 0, 0 },
  { "ae_movp48", ICLASS_ae_iclass_movp48,
    0,
    Opcode_ae_movp48_encode_fns, 0, 0 },
  { "ae_selp24.ll", ICLASS_ae_iclass_selp24_ll,
    0,
    Opcode_ae_selp24_ll_encode_fns, 0, 0 },
  { "ae_selp24.lh", ICLASS_ae_iclass_selp24_lh,
    0,
    Opcode_ae_selp24_lh_encode_fns, 0, 0 },
  { "ae_selp24.hl", ICLASS_ae_iclass_selp24_hl,
    0,
    Opcode_ae_selp24_hl_encode_fns, 0, 0 },
  { "ae_selp24.hh", ICLASS_ae_iclass_selp24_hh,
    0,
    Opcode_ae_selp24_hh_encode_fns, 0, 0 },
  { "ae_movtp24x2", ICLASS_ae_iclass_movtp24x2,
    0,
    Opcode_ae_movtp24x2_encode_fns, 0, 0 },
  { "ae_movfp24x2", ICLASS_ae_iclass_movfp24x2,
    0,
    Opcode_ae_movfp24x2_encode_fns, 0, 0 },
  { "ae_movtp48", ICLASS_ae_iclass_movtp48,
    0,
    Opcode_ae_movtp48_encode_fns, 0, 0 },
  { "ae_movfp48", ICLASS_ae_iclass_movfp48,
    0,
    Opcode_ae_movfp48_encode_fns, 0, 0 },
  { "ae_movpa24x2", ICLASS_ae_iclass_movpa24x2,
    0,
    Opcode_ae_movpa24x2_encode_fns, 0, 0 },
  { "ae_truncp24a32x2", ICLASS_ae_iclass_truncp24a32x2,
    0,
    Opcode_ae_truncp24a32x2_encode_fns, 0, 0 },
  { "ae_cvta32p24.l", ICLASS_ae_iclass_cvta32p24_l,
    0,
    Opcode_ae_cvta32p24_l_encode_fns, 0, 0 },
  { "ae_cvta32p24.h", ICLASS_ae_iclass_cvta32p24_h,
    0,
    Opcode_ae_cvta32p24_h_encode_fns, 0, 0 },
  { "ae_cvtp24a16x2.ll", ICLASS_ae_iclass_cvtp24a16x2_ll,
    0,
    Opcode_ae_cvtp24a16x2_ll_encode_fns, 0, 0 },
  { "ae_cvtp24a16x2.lh", ICLASS_ae_iclass_cvtp24a16x2_lh,
    0,
    Opcode_ae_cvtp24a16x2_lh_encode_fns, 0, 0 },
  { "ae_cvtp24a16x2.hl", ICLASS_ae_iclass_cvtp24a16x2_hl,
    0,
    Opcode_ae_cvtp24a16x2_hl_encode_fns, 0, 0 },
  { "ae_cvtp24a16x2.hh", ICLASS_ae_iclass_cvtp24a16x2_hh,
    0,
    Opcode_ae_cvtp24a16x2_hh_encode_fns, 0, 0 },
  { "ae_truncp24q48x2", ICLASS_ae_iclass_truncp24q48x2,
    0,
    Opcode_ae_truncp24q48x2_encode_fns, 0, 0 },
  { "ae_truncp16", ICLASS_ae_iclass_truncp16,
    0,
    Opcode_ae_truncp16_encode_fns, 0, 0 },
  { "ae_roundsp24q48sym", ICLASS_ae_iclass_roundsp24q48sym,
    0,
    Opcode_ae_roundsp24q48sym_encode_fns, 0, 0 },
  { "ae_roundsp24q48asym", ICLASS_ae_iclass_roundsp24q48asym,
    0,
    Opcode_ae_roundsp24q48asym_encode_fns, 0, 0 },
  { "ae_roundsp16q48sym", ICLASS_ae_iclass_roundsp16q48sym,
    0,
    Opcode_ae_roundsp16q48sym_encode_fns, 0, 0 },
  { "ae_roundsp16q48asym", ICLASS_ae_iclass_roundsp16q48asym,
    0,
    Opcode_ae_roundsp16q48asym_encode_fns, 0, 0 },
  { "ae_roundsp16sym", ICLASS_ae_iclass_roundsp16sym,
    0,
    Opcode_ae_roundsp16sym_encode_fns, 0, 0 },
  { "ae_roundsp16asym", ICLASS_ae_iclass_roundsp16asym,
    0,
    Opcode_ae_roundsp16asym_encode_fns, 0, 0 },
  { "ae_zeroq56", ICLASS_ae_iclass_zeroq56,
    0,
    Opcode_ae_zeroq56_encode_fns, 0, 0 },
  { "ae_movq56", ICLASS_ae_iclass_movq56,
    0,
    Opcode_ae_movq56_encode_fns, 0, 0 },
  { "ae_movtq56", ICLASS_ae_iclass_movtq56,
    0,
    Opcode_ae_movtq56_encode_fns, 0, 0 },
  { "ae_movfq56", ICLASS_ae_iclass_movfq56,
    0,
    Opcode_ae_movfq56_encode_fns, 0, 0 },
  { "ae_cvtq48a32s", ICLASS_ae_iclass_cvtq48a32s,
    0,
    Opcode_ae_cvtq48a32s_encode_fns, 0, 0 },
  { "ae_cvtq48p24s.l", ICLASS_ae_iclass_cvtq48p24s_l,
    0,
    Opcode_ae_cvtq48p24s_l_encode_fns, 0, 0 },
  { "ae_cvtq48p24s.h", ICLASS_ae_iclass_cvtq48p24s_h,
    0,
    Opcode_ae_cvtq48p24s_h_encode_fns, 0, 0 },
  { "ae_satq48s", ICLASS_ae_iclass_satq48s,
    0,
    Opcode_ae_satq48s_encode_fns, 0, 0 },
  { "ae_truncq32", ICLASS_ae_iclass_truncq32,
    0,
    Opcode_ae_truncq32_encode_fns, 0, 0 },
  { "ae_roundsq32sym", ICLASS_ae_iclass_roundsq32sym,
    0,
    Opcode_ae_roundsq32sym_encode_fns, 0, 0 },
  { "ae_roundsq32asym", ICLASS_ae_iclass_roundsq32asym,
    0,
    Opcode_ae_roundsq32asym_encode_fns, 0, 0 },
  { "ae_trunca32q48", ICLASS_ae_iclass_trunca32q48,
    0,
    Opcode_ae_trunca32q48_encode_fns, 0, 0 },
  { "ae_movap24s.l", ICLASS_ae_iclass_movap24s_l,
    0,
    Opcode_ae_movap24s_l_encode_fns, 0, 0 },
  { "ae_movap24s.h", ICLASS_ae_iclass_movap24s_h,
    0,
    Opcode_ae_movap24s_h_encode_fns, 0, 0 },
  { "ae_trunca16p24s.l", ICLASS_ae_iclass_trunca16p24s_l,
    0,
    Opcode_ae_trunca16p24s_l_encode_fns, 0, 0 },
  { "ae_trunca16p24s.h", ICLASS_ae_iclass_trunca16p24s_h,
    0,
    Opcode_ae_trunca16p24s_h_encode_fns, 0, 0 },
  { "ae_addp24", ICLASS_ae_iclass_addp24,
    0,
    Opcode_ae_addp24_encode_fns, 0, 0 },
  { "ae_subp24", ICLASS_ae_iclass_subp24,
    0,
    Opcode_ae_subp24_encode_fns, 0, 0 },
  { "ae_negp24", ICLASS_ae_iclass_negp24,
    0,
    Opcode_ae_negp24_encode_fns, 0, 0 },
  { "ae_absp24", ICLASS_ae_iclass_absp24,
    0,
    Opcode_ae_absp24_encode_fns, 0, 0 },
  { "ae_maxp24s", ICLASS_ae_iclass_maxp24s,
    0,
    Opcode_ae_maxp24s_encode_fns, 0, 0 },
  { "ae_minp24s", ICLASS_ae_iclass_minp24s,
    0,
    Opcode_ae_minp24s_encode_fns, 0, 0 },
  { "ae_maxbp24s", ICLASS_ae_iclass_maxbp24s,
    0,
    Opcode_ae_maxbp24s_encode_fns, 0, 0 },
  { "ae_minbp24s", ICLASS_ae_iclass_minbp24s,
    0,
    Opcode_ae_minbp24s_encode_fns, 0, 0 },
  { "ae_addsp24s", ICLASS_ae_iclass_addsp24s,
    0,
    Opcode_ae_addsp24s_encode_fns, 0, 0 },
  { "ae_subsp24s", ICLASS_ae_iclass_subsp24s,
    0,
    Opcode_ae_subsp24s_encode_fns, 0, 0 },
  { "ae_negsp24s", ICLASS_ae_iclass_negsp24s,
    0,
    Opcode_ae_negsp24s_encode_fns, 0, 0 },
  { "ae_abssp24s", ICLASS_ae_iclass_abssp24s,
    0,
    Opcode_ae_abssp24s_encode_fns, 0, 0 },
  { "ae_andp48", ICLASS_ae_iclass_andp48,
    0,
    Opcode_ae_andp48_encode_fns, 0, 0 },
  { "ae_nandp48", ICLASS_ae_iclass_nandp48,
    0,
    Opcode_ae_nandp48_encode_fns, 0, 0 },
  { "ae_orp48", ICLASS_ae_iclass_orp48,
    0,
    Opcode_ae_orp48_encode_fns, 0, 0 },
  { "ae_xorp48", ICLASS_ae_iclass_xorp48,
    0,
    Opcode_ae_xorp48_encode_fns, 0, 0 },
  { "ae_ltp24s", ICLASS_ae_iclass_ltp24s,
    0,
    Opcode_ae_ltp24s_encode_fns, 0, 0 },
  { "ae_lep24s", ICLASS_ae_iclass_lep24s,
    0,
    Opcode_ae_lep24s_encode_fns, 0, 0 },
  { "ae_eqp24", ICLASS_ae_iclass_eqp24,
    0,
    Opcode_ae_eqp24_encode_fns, 0, 0 },
  { "ae_addq56", ICLASS_ae_iclass_addq56,
    0,
    Opcode_ae_addq56_encode_fns, 0, 0 },
  { "ae_subq56", ICLASS_ae_iclass_subq56,
    0,
    Opcode_ae_subq56_encode_fns, 0, 0 },
  { "ae_negq56", ICLASS_ae_iclass_negq56,
    0,
    Opcode_ae_negq56_encode_fns, 0, 0 },
  { "ae_absq56", ICLASS_ae_iclass_absq56,
    0,
    Opcode_ae_absq56_encode_fns, 0, 0 },
  { "ae_maxq56s", ICLASS_ae_iclass_maxq56s,
    0,
    Opcode_ae_maxq56s_encode_fns, 0, 0 },
  { "ae_minq56s", ICLASS_ae_iclass_minq56s,
    0,
    Opcode_ae_minq56s_encode_fns, 0, 0 },
  { "ae_maxbq56s", ICLASS_ae_iclass_maxbq56s,
    0,
    Opcode_ae_maxbq56s_encode_fns, 0, 0 },
  { "ae_minbq56s", ICLASS_ae_iclass_minbq56s,
    0,
    Opcode_ae_minbq56s_encode_fns, 0, 0 },
  { "ae_addsq56s", ICLASS_ae_iclass_addsq56s,
    0,
    Opcode_ae_addsq56s_encode_fns, 0, 0 },
  { "ae_subsq56s", ICLASS_ae_iclass_subsq56s,
    0,
    Opcode_ae_subsq56s_encode_fns, 0, 0 },
  { "ae_negsq56s", ICLASS_ae_iclass_negsq56s,
    0,
    Opcode_ae_negsq56s_encode_fns, 0, 0 },
  { "ae_abssq56s", ICLASS_ae_iclass_abssq56s,
    0,
    Opcode_ae_abssq56s_encode_fns, 0, 0 },
  { "ae_andq56", ICLASS_ae_iclass_andq56,
    0,
    Opcode_ae_andq56_encode_fns, 0, 0 },
  { "ae_nandq56", ICLASS_ae_iclass_nandq56,
    0,
    Opcode_ae_nandq56_encode_fns, 0, 0 },
  { "ae_orq56", ICLASS_ae_iclass_orq56,
    0,
    Opcode_ae_orq56_encode_fns, 0, 0 },
  { "ae_xorq56", ICLASS_ae_iclass_xorq56,
    0,
    Opcode_ae_xorq56_encode_fns, 0, 0 },
  { "ae_sllip24", ICLASS_ae_iclass_sllip24,
    0,
    Opcode_ae_sllip24_encode_fns, 0, 0 },
  { "ae_srlip24", ICLASS_ae_iclass_srlip24,
    0,
    Opcode_ae_srlip24_encode_fns, 0, 0 },
  { "ae_sraip24", ICLASS_ae_iclass_sraip24,
    0,
    Opcode_ae_sraip24_encode_fns, 0, 0 },
  { "ae_sllsp24", ICLASS_ae_iclass_sllsp24,
    0,
    Opcode_ae_sllsp24_encode_fns, 0, 0 },
  { "ae_srlsp24", ICLASS_ae_iclass_srlsp24,
    0,
    Opcode_ae_srlsp24_encode_fns, 0, 0 },
  { "ae_srasp24", ICLASS_ae_iclass_srasp24,
    0,
    Opcode_ae_srasp24_encode_fns, 0, 0 },
  { "ae_sllisp24s", ICLASS_ae_iclass_sllisp24s,
    0,
    Opcode_ae_sllisp24s_encode_fns, 0, 0 },
  { "ae_sllssp24s", ICLASS_ae_iclass_sllssp24s,
    0,
    Opcode_ae_sllssp24s_encode_fns, 0, 0 },
  { "ae_slliq56", ICLASS_ae_iclass_slliq56,
    0,
    Opcode_ae_slliq56_encode_fns, 0, 0 },
  { "ae_srliq56", ICLASS_ae_iclass_srliq56,
    0,
    Opcode_ae_srliq56_encode_fns, 0, 0 },
  { "ae_sraiq56", ICLASS_ae_iclass_sraiq56,
    0,
    Opcode_ae_sraiq56_encode_fns, 0, 0 },
  { "ae_sllsq56", ICLASS_ae_iclass_sllsq56,
    0,
    Opcode_ae_sllsq56_encode_fns, 0, 0 },
  { "ae_srlsq56", ICLASS_ae_iclass_srlsq56,
    0,
    Opcode_ae_srlsq56_encode_fns, 0, 0 },
  { "ae_srasq56", ICLASS_ae_iclass_srasq56,
    0,
    Opcode_ae_srasq56_encode_fns, 0, 0 },
  { "ae_sllaq56", ICLASS_ae_iclass_sllaq56,
    0,
    Opcode_ae_sllaq56_encode_fns, 0, 0 },
  { "ae_srlaq56", ICLASS_ae_iclass_srlaq56,
    0,
    Opcode_ae_srlaq56_encode_fns, 0, 0 },
  { "ae_sraaq56", ICLASS_ae_iclass_sraaq56,
    0,
    Opcode_ae_sraaq56_encode_fns, 0, 0 },
  { "ae_sllisq56s", ICLASS_ae_iclass_sllisq56s,
    0,
    Opcode_ae_sllisq56s_encode_fns, 0, 0 },
  { "ae_sllssq56s", ICLASS_ae_iclass_sllssq56s,
    0,
    Opcode_ae_sllssq56s_encode_fns, 0, 0 },
  { "ae_sllasq56s", ICLASS_ae_iclass_sllasq56s,
    0,
    Opcode_ae_sllasq56s_encode_fns, 0, 0 },
  { "ae_ltq56s", ICLASS_ae_iclass_ltq56s,
    0,
    Opcode_ae_ltq56s_encode_fns, 0, 0 },
  { "ae_leq56s", ICLASS_ae_iclass_leq56s,
    0,
    Opcode_ae_leq56s_encode_fns, 0, 0 },
  { "ae_eqq56", ICLASS_ae_iclass_eqq56,
    0,
    Opcode_ae_eqq56_encode_fns, 0, 0 },
  { "ae_nsaq56s", ICLASS_ae_iclass_nsaq56s,
    0,
    Opcode_ae_nsaq56s_encode_fns, 0, 0 },
  { "ae_mulfs32p16s.ll", ICLASS_ae_iclass_mulfs32p16s_ll,
    0,
    Opcode_ae_mulfs32p16s_ll_encode_fns, 0, 0 },
  { "ae_mulfp24s.ll", ICLASS_ae_iclass_mulfp24s_ll,
    0,
    Opcode_ae_mulfp24s_ll_encode_fns, 0, 0 },
  { "ae_mulp24s.ll", ICLASS_ae_iclass_mulp24s_ll,
    0,
    Opcode_ae_mulp24s_ll_encode_fns, 0, 0 },
  { "ae_mulfs32p16s.lh", ICLASS_ae_iclass_mulfs32p16s_lh,
    0,
    Opcode_ae_mulfs32p16s_lh_encode_fns, 0, 0 },
  { "ae_mulfp24s.lh", ICLASS_ae_iclass_mulfp24s_lh,
    0,
    Opcode_ae_mulfp24s_lh_encode_fns, 0, 0 },
  { "ae_mulp24s.lh", ICLASS_ae_iclass_mulp24s_lh,
    0,
    Opcode_ae_mulp24s_lh_encode_fns, 0, 0 },
  { "ae_mulfs32p16s.hl", ICLASS_ae_iclass_mulfs32p16s_hl,
    0,
    Opcode_ae_mulfs32p16s_hl_encode_fns, 0, 0 },
  { "ae_mulfp24s.hl", ICLASS_ae_iclass_mulfp24s_hl,
    0,
    Opcode_ae_mulfp24s_hl_encode_fns, 0, 0 },
  { "ae_mulp24s.hl", ICLASS_ae_iclass_mulp24s_hl,
    0,
    Opcode_ae_mulp24s_hl_encode_fns, 0, 0 },
  { "ae_mulfs32p16s.hh", ICLASS_ae_iclass_mulfs32p16s_hh,
    0,
    Opcode_ae_mulfs32p16s_hh_encode_fns, 0, 0 },
  { "ae_mulfp24s.hh", ICLASS_ae_iclass_mulfp24s_hh,
    0,
    Opcode_ae_mulfp24s_hh_encode_fns, 0, 0 },
  { "ae_mulp24s.hh", ICLASS_ae_iclass_mulp24s_hh,
    0,
    Opcode_ae_mulp24s_hh_encode_fns, 0, 0 },
  { "ae_mulafs32p16s.ll", ICLASS_ae_iclass_mulafs32p16s_ll,
    0,
    Opcode_ae_mulafs32p16s_ll_encode_fns, 0, 0 },
  { "ae_mulafp24s.ll", ICLASS_ae_iclass_mulafp24s_ll,
    0,
    Opcode_ae_mulafp24s_ll_encode_fns, 0, 0 },
  { "ae_mulap24s.ll", ICLASS_ae_iclass_mulap24s_ll,
    0,
    Opcode_ae_mulap24s_ll_encode_fns, 0, 0 },
  { "ae_mulafs32p16s.lh", ICLASS_ae_iclass_mulafs32p16s_lh,
    0,
    Opcode_ae_mulafs32p16s_lh_encode_fns, 0, 0 },
  { "ae_mulafp24s.lh", ICLASS_ae_iclass_mulafp24s_lh,
    0,
    Opcode_ae_mulafp24s_lh_encode_fns, 0, 0 },
  { "ae_mulap24s.lh", ICLASS_ae_iclass_mulap24s_lh,
    0,
    Opcode_ae_mulap24s_lh_encode_fns, 0, 0 },
  { "ae_mulafs32p16s.hl", ICLASS_ae_iclass_mulafs32p16s_hl,
    0,
    Opcode_ae_mulafs32p16s_hl_encode_fns, 0, 0 },
  { "ae_mulafp24s.hl", ICLASS_ae_iclass_mulafp24s_hl,
    0,
    Opcode_ae_mulafp24s_hl_encode_fns, 0, 0 },
  { "ae_mulap24s.hl", ICLASS_ae_iclass_mulap24s_hl,
    0,
    Opcode_ae_mulap24s_hl_encode_fns, 0, 0 },
  { "ae_mulafs32p16s.hh", ICLASS_ae_iclass_mulafs32p16s_hh,
    0,
    Opcode_ae_mulafs32p16s_hh_encode_fns, 0, 0 },
  { "ae_mulafp24s.hh", ICLASS_ae_iclass_mulafp24s_hh,
    0,
    Opcode_ae_mulafp24s_hh_encode_fns, 0, 0 },
  { "ae_mulap24s.hh", ICLASS_ae_iclass_mulap24s_hh,
    0,
    Opcode_ae_mulap24s_hh_encode_fns, 0, 0 },
  { "ae_mulsfs32p16s.ll", ICLASS_ae_iclass_mulsfs32p16s_ll,
    0,
    Opcode_ae_mulsfs32p16s_ll_encode_fns, 0, 0 },
  { "ae_mulsfp24s.ll", ICLASS_ae_iclass_mulsfp24s_ll,
    0,
    Opcode_ae_mulsfp24s_ll_encode_fns, 0, 0 },
  { "ae_mulsp24s.ll", ICLASS_ae_iclass_mulsp24s_ll,
    0,
    Opcode_ae_mulsp24s_ll_encode_fns, 0, 0 },
  { "ae_mulsfs32p16s.lh", ICLASS_ae_iclass_mulsfs32p16s_lh,
    0,
    Opcode_ae_mulsfs32p16s_lh_encode_fns, 0, 0 },
  { "ae_mulsfp24s.lh", ICLASS_ae_iclass_mulsfp24s_lh,
    0,
    Opcode_ae_mulsfp24s_lh_encode_fns, 0, 0 },
  { "ae_mulsp24s.lh", ICLASS_ae_iclass_mulsp24s_lh,
    0,
    Opcode_ae_mulsp24s_lh_encode_fns, 0, 0 },
  { "ae_mulsfs32p16s.hl", ICLASS_ae_iclass_mulsfs32p16s_hl,
    0,
    Opcode_ae_mulsfs32p16s_hl_encode_fns, 0, 0 },
  { "ae_mulsfp24s.hl", ICLASS_ae_iclass_mulsfp24s_hl,
    0,
    Opcode_ae_mulsfp24s_hl_encode_fns, 0, 0 },
  { "ae_mulsp24s.hl", ICLASS_ae_iclass_mulsp24s_hl,
    0,
    Opcode_ae_mulsp24s_hl_encode_fns, 0, 0 },
  { "ae_mulsfs32p16s.hh", ICLASS_ae_iclass_mulsfs32p16s_hh,
    0,
    Opcode_ae_mulsfs32p16s_hh_encode_fns, 0, 0 },
  { "ae_mulsfp24s.hh", ICLASS_ae_iclass_mulsfp24s_hh,
    0,
    Opcode_ae_mulsfp24s_hh_encode_fns, 0, 0 },
  { "ae_mulsp24s.hh", ICLASS_ae_iclass_mulsp24s_hh,
    0,
    Opcode_ae_mulsp24s_hh_encode_fns, 0, 0 },
  { "ae_mulafs56p24s.ll", ICLASS_ae_iclass_mulafs56p24s_ll,
    0,
    Opcode_ae_mulafs56p24s_ll_encode_fns, 0, 0 },
  { "ae_mulas56p24s.ll", ICLASS_ae_iclass_mulas56p24s_ll,
    0,
    Opcode_ae_mulas56p24s_ll_encode_fns, 0, 0 },
  { "ae_mulafs56p24s.lh", ICLASS_ae_iclass_mulafs56p24s_lh,
    0,
    Opcode_ae_mulafs56p24s_lh_encode_fns, 0, 0 },
  { "ae_mulas56p24s.lh", ICLASS_ae_iclass_mulas56p24s_lh,
    0,
    Opcode_ae_mulas56p24s_lh_encode_fns, 0, 0 },
  { "ae_mulafs56p24s.hl", ICLASS_ae_iclass_mulafs56p24s_hl,
    0,
    Opcode_ae_mulafs56p24s_hl_encode_fns, 0, 0 },
  { "ae_mulas56p24s.hl", ICLASS_ae_iclass_mulas56p24s_hl,
    0,
    Opcode_ae_mulas56p24s_hl_encode_fns, 0, 0 },
  { "ae_mulafs56p24s.hh", ICLASS_ae_iclass_mulafs56p24s_hh,
    0,
    Opcode_ae_mulafs56p24s_hh_encode_fns, 0, 0 },
  { "ae_mulas56p24s.hh", ICLASS_ae_iclass_mulas56p24s_hh,
    0,
    Opcode_ae_mulas56p24s_hh_encode_fns, 0, 0 },
  { "ae_mulsfs56p24s.ll", ICLASS_ae_iclass_mulsfs56p24s_ll,
    0,
    Opcode_ae_mulsfs56p24s_ll_encode_fns, 0, 0 },
  { "ae_mulss56p24s.ll", ICLASS_ae_iclass_mulss56p24s_ll,
    0,
    Opcode_ae_mulss56p24s_ll_encode_fns, 0, 0 },
  { "ae_mulsfs56p24s.lh", ICLASS_ae_iclass_mulsfs56p24s_lh,
    0,
    Opcode_ae_mulsfs56p24s_lh_encode_fns, 0, 0 },
  { "ae_mulss56p24s.lh", ICLASS_ae_iclass_mulss56p24s_lh,
    0,
    Opcode_ae_mulss56p24s_lh_encode_fns, 0, 0 },
  { "ae_mulsfs56p24s.hl", ICLASS_ae_iclass_mulsfs56p24s_hl,
    0,
    Opcode_ae_mulsfs56p24s_hl_encode_fns, 0, 0 },
  { "ae_mulss56p24s.hl", ICLASS_ae_iclass_mulss56p24s_hl,
    0,
    Opcode_ae_mulss56p24s_hl_encode_fns, 0, 0 },
  { "ae_mulsfs56p24s.hh", ICLASS_ae_iclass_mulsfs56p24s_hh,
    0,
    Opcode_ae_mulsfs56p24s_hh_encode_fns, 0, 0 },
  { "ae_mulss56p24s.hh", ICLASS_ae_iclass_mulss56p24s_hh,
    0,
    Opcode_ae_mulss56p24s_hh_encode_fns, 0, 0 },
  { "ae_mulfq32sp16s.l", ICLASS_ae_iclass_mulfq32sp16s_l,
    0,
    Opcode_ae_mulfq32sp16s_l_encode_fns, 0, 0 },
  { "ae_mulfq32sp16s.h", ICLASS_ae_iclass_mulfq32sp16s_h,
    0,
    Opcode_ae_mulfq32sp16s_h_encode_fns, 0, 0 },
  { "ae_mulfq32sp16u.l", ICLASS_ae_iclass_mulfq32sp16u_l,
    0,
    Opcode_ae_mulfq32sp16u_l_encode_fns, 0, 0 },
  { "ae_mulfq32sp16u.h", ICLASS_ae_iclass_mulfq32sp16u_h,
    0,
    Opcode_ae_mulfq32sp16u_h_encode_fns, 0, 0 },
  { "ae_mulq32sp16s.l", ICLASS_ae_iclass_mulq32sp16s_l,
    0,
    Opcode_ae_mulq32sp16s_l_encode_fns, 0, 0 },
  { "ae_mulq32sp16s.h", ICLASS_ae_iclass_mulq32sp16s_h,
    0,
    Opcode_ae_mulq32sp16s_h_encode_fns, 0, 0 },
  { "ae_mulq32sp16u.l", ICLASS_ae_iclass_mulq32sp16u_l,
    0,
    Opcode_ae_mulq32sp16u_l_encode_fns, 0, 0 },
  { "ae_mulq32sp16u.h", ICLASS_ae_iclass_mulq32sp16u_h,
    0,
    Opcode_ae_mulq32sp16u_h_encode_fns, 0, 0 },
  { "ae_mulafq32sp16s.l", ICLASS_ae_iclass_mulafq32sp16s_l,
    0,
    Opcode_ae_mulafq32sp16s_l_encode_fns, 0, 0 },
  { "ae_mulafq32sp16s.h", ICLASS_ae_iclass_mulafq32sp16s_h,
    0,
    Opcode_ae_mulafq32sp16s_h_encode_fns, 0, 0 },
  { "ae_mulafq32sp16u.l", ICLASS_ae_iclass_mulafq32sp16u_l,
    0,
    Opcode_ae_mulafq32sp16u_l_encode_fns, 0, 0 },
  { "ae_mulafq32sp16u.h", ICLASS_ae_iclass_mulafq32sp16u_h,
    0,
    Opcode_ae_mulafq32sp16u_h_encode_fns, 0, 0 },
  { "ae_mulaq32sp16s.l", ICLASS_ae_iclass_mulaq32sp16s_l,
    0,
    Opcode_ae_mulaq32sp16s_l_encode_fns, 0, 0 },
  { "ae_mulaq32sp16s.h", ICLASS_ae_iclass_mulaq32sp16s_h,
    0,
    Opcode_ae_mulaq32sp16s_h_encode_fns, 0, 0 },
  { "ae_mulaq32sp16u.l", ICLASS_ae_iclass_mulaq32sp16u_l,
    0,
    Opcode_ae_mulaq32sp16u_l_encode_fns, 0, 0 },
  { "ae_mulaq32sp16u.h", ICLASS_ae_iclass_mulaq32sp16u_h,
    0,
    Opcode_ae_mulaq32sp16u_h_encode_fns, 0, 0 },
  { "ae_mulsfq32sp16s.l", ICLASS_ae_iclass_mulsfq32sp16s_l,
    0,
    Opcode_ae_mulsfq32sp16s_l_encode_fns, 0, 0 },
  { "ae_mulsfq32sp16s.h", ICLASS_ae_iclass_mulsfq32sp16s_h,
    0,
    Opcode_ae_mulsfq32sp16s_h_encode_fns, 0, 0 },
  { "ae_mulsfq32sp16u.l", ICLASS_ae_iclass_mulsfq32sp16u_l,
    0,
    Opcode_ae_mulsfq32sp16u_l_encode_fns, 0, 0 },
  { "ae_mulsfq32sp16u.h", ICLASS_ae_iclass_mulsfq32sp16u_h,
    0,
    Opcode_ae_mulsfq32sp16u_h_encode_fns, 0, 0 },
  { "ae_mulsq32sp16s.l", ICLASS_ae_iclass_mulsq32sp16s_l,
    0,
    Opcode_ae_mulsq32sp16s_l_encode_fns, 0, 0 },
  { "ae_mulsq32sp16s.h", ICLASS_ae_iclass_mulsq32sp16s_h,
    0,
    Opcode_ae_mulsq32sp16s_h_encode_fns, 0, 0 },
  { "ae_mulsq32sp16u.l", ICLASS_ae_iclass_mulsq32sp16u_l,
    0,
    Opcode_ae_mulsq32sp16u_l_encode_fns, 0, 0 },
  { "ae_mulsq32sp16u.h", ICLASS_ae_iclass_mulsq32sp16u_h,
    0,
    Opcode_ae_mulsq32sp16u_h_encode_fns, 0, 0 },
  { "ae_mulzaaq32sp16s.ll", ICLASS_ae_iclass_mulzaaq32sp16s_ll,
    0,
    Opcode_ae_mulzaaq32sp16s_ll_encode_fns, 0, 0 },
  { "ae_mulzaafq32sp16s.ll", ICLASS_ae_iclass_mulzaafq32sp16s_ll,
    0,
    Opcode_ae_mulzaafq32sp16s_ll_encode_fns, 0, 0 },
  { "ae_mulzaaq32sp16u.ll", ICLASS_ae_iclass_mulzaaq32sp16u_ll,
    0,
    Opcode_ae_mulzaaq32sp16u_ll_encode_fns, 0, 0 },
  { "ae_mulzaafq32sp16u.ll", ICLASS_ae_iclass_mulzaafq32sp16u_ll,
    0,
    Opcode_ae_mulzaafq32sp16u_ll_encode_fns, 0, 0 },
  { "ae_mulzaaq32sp16s.hh", ICLASS_ae_iclass_mulzaaq32sp16s_hh,
    0,
    Opcode_ae_mulzaaq32sp16s_hh_encode_fns, 0, 0 },
  { "ae_mulzaafq32sp16s.hh", ICLASS_ae_iclass_mulzaafq32sp16s_hh,
    0,
    Opcode_ae_mulzaafq32sp16s_hh_encode_fns, 0, 0 },
  { "ae_mulzaaq32sp16u.hh", ICLASS_ae_iclass_mulzaaq32sp16u_hh,
    0,
    Opcode_ae_mulzaaq32sp16u_hh_encode_fns, 0, 0 },
  { "ae_mulzaafq32sp16u.hh", ICLASS_ae_iclass_mulzaafq32sp16u_hh,
    0,
    Opcode_ae_mulzaafq32sp16u_hh_encode_fns, 0, 0 },
  { "ae_mulzaaq32sp16s.lh", ICLASS_ae_iclass_mulzaaq32sp16s_lh,
    0,
    Opcode_ae_mulzaaq32sp16s_lh_encode_fns, 0, 0 },
  { "ae_mulzaafq32sp16s.lh", ICLASS_ae_iclass_mulzaafq32sp16s_lh,
    0,
    Opcode_ae_mulzaafq32sp16s_lh_encode_fns, 0, 0 },
  { "ae_mulzaaq32sp16u.lh", ICLASS_ae_iclass_mulzaaq32sp16u_lh,
    0,
    Opcode_ae_mulzaaq32sp16u_lh_encode_fns, 0, 0 },
  { "ae_mulzaafq32sp16u.lh", ICLASS_ae_iclass_mulzaafq32sp16u_lh,
    0,
    Opcode_ae_mulzaafq32sp16u_lh_encode_fns, 0, 0 },
  { "ae_mulzasq32sp16s.ll", ICLASS_ae_iclass_mulzasq32sp16s_ll,
    0,
    Opcode_ae_mulzasq32sp16s_ll_encode_fns, 0, 0 },
  { "ae_mulzasfq32sp16s.ll", ICLASS_ae_iclass_mulzasfq32sp16s_ll,
    0,
    Opcode_ae_mulzasfq32sp16s_ll_encode_fns, 0, 0 },
  { "ae_mulzasq32sp16u.ll", ICLASS_ae_iclass_mulzasq32sp16u_ll,
    0,
    Opcode_ae_mulzasq32sp16u_ll_encode_fns, 0, 0 },
  { "ae_mulzasfq32sp16u.ll", ICLASS_ae_iclass_mulzasfq32sp16u_ll,
    0,
    Opcode_ae_mulzasfq32sp16u_ll_encode_fns, 0, 0 },
  { "ae_mulzasq32sp16s.hh", ICLASS_ae_iclass_mulzasq32sp16s_hh,
    0,
    Opcode_ae_mulzasq32sp16s_hh_encode_fns, 0, 0 },
  { "ae_mulzasfq32sp16s.hh", ICLASS_ae_iclass_mulzasfq32sp16s_hh,
    0,
    Opcode_ae_mulzasfq32sp16s_hh_encode_fns, 0, 0 },
  { "ae_mulzasq32sp16u.hh", ICLASS_ae_iclass_mulzasq32sp16u_hh,
    0,
    Opcode_ae_mulzasq32sp16u_hh_encode_fns, 0, 0 },
  { "ae_mulzasfq32sp16u.hh", ICLASS_ae_iclass_mulzasfq32sp16u_hh,
    0,
    Opcode_ae_mulzasfq32sp16u_hh_encode_fns, 0, 0 },
  { "ae_mulzasq32sp16s.lh", ICLASS_ae_iclass_mulzasq32sp16s_lh,
    0,
    Opcode_ae_mulzasq32sp16s_lh_encode_fns, 0, 0 },
  { "ae_mulzasfq32sp16s.lh", ICLASS_ae_iclass_mulzasfq32sp16s_lh,
    0,
    Opcode_ae_mulzasfq32sp16s_lh_encode_fns, 0, 0 },
  { "ae_mulzasq32sp16u.lh", ICLASS_ae_iclass_mulzasq32sp16u_lh,
    0,
    Opcode_ae_mulzasq32sp16u_lh_encode_fns, 0, 0 },
  { "ae_mulzasfq32sp16u.lh", ICLASS_ae_iclass_mulzasfq32sp16u_lh,
    0,
    Opcode_ae_mulzasfq32sp16u_lh_encode_fns, 0, 0 },
  { "ae_mulzsaq32sp16s.ll", ICLASS_ae_iclass_mulzsaq32sp16s_ll,
    0,
    Opcode_ae_mulzsaq32sp16s_ll_encode_fns, 0, 0 },
  { "ae_mulzsafq32sp16s.ll", ICLASS_ae_iclass_mulzsafq32sp16s_ll,
    0,
    Opcode_ae_mulzsafq32sp16s_ll_encode_fns, 0, 0 },
  { "ae_mulzsaq32sp16u.ll", ICLASS_ae_iclass_mulzsaq32sp16u_ll,
    0,
    Opcode_ae_mulzsaq32sp16u_ll_encode_fns, 0, 0 },
  { "ae_mulzsafq32sp16u.ll", ICLASS_ae_iclass_mulzsafq32sp16u_ll,
    0,
    Opcode_ae_mulzsafq32sp16u_ll_encode_fns, 0, 0 },
  { "ae_mulzsaq32sp16s.hh", ICLASS_ae_iclass_mulzsaq32sp16s_hh,
    0,
    Opcode_ae_mulzsaq32sp16s_hh_encode_fns, 0, 0 },
  { "ae_mulzsafq32sp16s.hh", ICLASS_ae_iclass_mulzsafq32sp16s_hh,
    0,
    Opcode_ae_mulzsafq32sp16s_hh_encode_fns, 0, 0 },
  { "ae_mulzsaq32sp16u.hh", ICLASS_ae_iclass_mulzsaq32sp16u_hh,
    0,
    Opcode_ae_mulzsaq32sp16u_hh_encode_fns, 0, 0 },
  { "ae_mulzsafq32sp16u.hh", ICLASS_ae_iclass_mulzsafq32sp16u_hh,
    0,
    Opcode_ae_mulzsafq32sp16u_hh_encode_fns, 0, 0 },
  { "ae_mulzsaq32sp16s.lh", ICLASS_ae_iclass_mulzsaq32sp16s_lh,
    0,
    Opcode_ae_mulzsaq32sp16s_lh_encode_fns, 0, 0 },
  { "ae_mulzsafq32sp16s.lh", ICLASS_ae_iclass_mulzsafq32sp16s_lh,
    0,
    Opcode_ae_mulzsafq32sp16s_lh_encode_fns, 0, 0 },
  { "ae_mulzsaq32sp16u.lh", ICLASS_ae_iclass_mulzsaq32sp16u_lh,
    0,
    Opcode_ae_mulzsaq32sp16u_lh_encode_fns, 0, 0 },
  { "ae_mulzsafq32sp16u.lh", ICLASS_ae_iclass_mulzsafq32sp16u_lh,
    0,
    Opcode_ae_mulzsafq32sp16u_lh_encode_fns, 0, 0 },
  { "ae_mulzssq32sp16s.ll", ICLASS_ae_iclass_mulzssq32sp16s_ll,
    0,
    Opcode_ae_mulzssq32sp16s_ll_encode_fns, 0, 0 },
  { "ae_mulzssfq32sp16s.ll", ICLASS_ae_iclass_mulzssfq32sp16s_ll,
    0,
    Opcode_ae_mulzssfq32sp16s_ll_encode_fns, 0, 0 },
  { "ae_mulzssq32sp16u.ll", ICLASS_ae_iclass_mulzssq32sp16u_ll,
    0,
    Opcode_ae_mulzssq32sp16u_ll_encode_fns, 0, 0 },
  { "ae_mulzssfq32sp16u.ll", ICLASS_ae_iclass_mulzssfq32sp16u_ll,
    0,
    Opcode_ae_mulzssfq32sp16u_ll_encode_fns, 0, 0 },
  { "ae_mulzssq32sp16s.hh", ICLASS_ae_iclass_mulzssq32sp16s_hh,
    0,
    Opcode_ae_mulzssq32sp16s_hh_encode_fns, 0, 0 },
  { "ae_mulzssfq32sp16s.hh", ICLASS_ae_iclass_mulzssfq32sp16s_hh,
    0,
    Opcode_ae_mulzssfq32sp16s_hh_encode_fns, 0, 0 },
  { "ae_mulzssq32sp16u.hh", ICLASS_ae_iclass_mulzssq32sp16u_hh,
    0,
    Opcode_ae_mulzssq32sp16u_hh_encode_fns, 0, 0 },
  { "ae_mulzssfq32sp16u.hh", ICLASS_ae_iclass_mulzssfq32sp16u_hh,
    0,
    Opcode_ae_mulzssfq32sp16u_hh_encode_fns, 0, 0 },
  { "ae_mulzssq32sp16s.lh", ICLASS_ae_iclass_mulzssq32sp16s_lh,
    0,
    Opcode_ae_mulzssq32sp16s_lh_encode_fns, 0, 0 },
  { "ae_mulzssfq32sp16s.lh", ICLASS_ae_iclass_mulzssfq32sp16s_lh,
    0,
    Opcode_ae_mulzssfq32sp16s_lh_encode_fns, 0, 0 },
  { "ae_mulzssq32sp16u.lh", ICLASS_ae_iclass_mulzssq32sp16u_lh,
    0,
    Opcode_ae_mulzssq32sp16u_lh_encode_fns, 0, 0 },
  { "ae_mulzssfq32sp16u.lh", ICLASS_ae_iclass_mulzssfq32sp16u_lh,
    0,
    Opcode_ae_mulzssfq32sp16u_lh_encode_fns, 0, 0 },
  { "ae_mulzaafp24s.hh.ll", ICLASS_ae_iclass_mulzaafp24s_hh_ll,
    0,
    Opcode_ae_mulzaafp24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulzaap24s.hh.ll", ICLASS_ae_iclass_mulzaap24s_hh_ll,
    0,
    Opcode_ae_mulzaap24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulzaafp24s.hl.lh", ICLASS_ae_iclass_mulzaafp24s_hl_lh,
    0,
    Opcode_ae_mulzaafp24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulzaap24s.hl.lh", ICLASS_ae_iclass_mulzaap24s_hl_lh,
    0,
    Opcode_ae_mulzaap24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulzasfp24s.hh.ll", ICLASS_ae_iclass_mulzasfp24s_hh_ll,
    0,
    Opcode_ae_mulzasfp24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulzasp24s.hh.ll", ICLASS_ae_iclass_mulzasp24s_hh_ll,
    0,
    Opcode_ae_mulzasp24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulzasfp24s.hl.lh", ICLASS_ae_iclass_mulzasfp24s_hl_lh,
    0,
    Opcode_ae_mulzasfp24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulzasp24s.hl.lh", ICLASS_ae_iclass_mulzasp24s_hl_lh,
    0,
    Opcode_ae_mulzasp24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulzsafp24s.hh.ll", ICLASS_ae_iclass_mulzsafp24s_hh_ll,
    0,
    Opcode_ae_mulzsafp24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulzsap24s.hh.ll", ICLASS_ae_iclass_mulzsap24s_hh_ll,
    0,
    Opcode_ae_mulzsap24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulzsafp24s.hl.lh", ICLASS_ae_iclass_mulzsafp24s_hl_lh,
    0,
    Opcode_ae_mulzsafp24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulzsap24s.hl.lh", ICLASS_ae_iclass_mulzsap24s_hl_lh,
    0,
    Opcode_ae_mulzsap24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulzssfp24s.hh.ll", ICLASS_ae_iclass_mulzssfp24s_hh_ll,
    0,
    Opcode_ae_mulzssfp24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulzssp24s.hh.ll", ICLASS_ae_iclass_mulzssp24s_hh_ll,
    0,
    Opcode_ae_mulzssp24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulzssfp24s.hl.lh", ICLASS_ae_iclass_mulzssfp24s_hl_lh,
    0,
    Opcode_ae_mulzssfp24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulzssp24s.hl.lh", ICLASS_ae_iclass_mulzssp24s_hl_lh,
    0,
    Opcode_ae_mulzssp24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulaafp24s.hh.ll", ICLASS_ae_iclass_mulaafp24s_hh_ll,
    0,
    Opcode_ae_mulaafp24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulaap24s.hh.ll", ICLASS_ae_iclass_mulaap24s_hh_ll,
    0,
    Opcode_ae_mulaap24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulaafp24s.hl.lh", ICLASS_ae_iclass_mulaafp24s_hl_lh,
    0,
    Opcode_ae_mulaafp24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulaap24s.hl.lh", ICLASS_ae_iclass_mulaap24s_hl_lh,
    0,
    Opcode_ae_mulaap24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulasfp24s.hh.ll", ICLASS_ae_iclass_mulasfp24s_hh_ll,
    0,
    Opcode_ae_mulasfp24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulasp24s.hh.ll", ICLASS_ae_iclass_mulasp24s_hh_ll,
    0,
    Opcode_ae_mulasp24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulasfp24s.hl.lh", ICLASS_ae_iclass_mulasfp24s_hl_lh,
    0,
    Opcode_ae_mulasfp24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulasp24s.hl.lh", ICLASS_ae_iclass_mulasp24s_hl_lh,
    0,
    Opcode_ae_mulasp24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulsafp24s.hh.ll", ICLASS_ae_iclass_mulsafp24s_hh_ll,
    0,
    Opcode_ae_mulsafp24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulsap24s.hh.ll", ICLASS_ae_iclass_mulsap24s_hh_ll,
    0,
    Opcode_ae_mulsap24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulsafp24s.hl.lh", ICLASS_ae_iclass_mulsafp24s_hl_lh,
    0,
    Opcode_ae_mulsafp24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulsap24s.hl.lh", ICLASS_ae_iclass_mulsap24s_hl_lh,
    0,
    Opcode_ae_mulsap24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulssfp24s.hh.ll", ICLASS_ae_iclass_mulssfp24s_hh_ll,
    0,
    Opcode_ae_mulssfp24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulssp24s.hh.ll", ICLASS_ae_iclass_mulssp24s_hh_ll,
    0,
    Opcode_ae_mulssp24s_hh_ll_encode_fns, 0, 0 },
  { "ae_mulssfp24s.hl.lh", ICLASS_ae_iclass_mulssfp24s_hl_lh,
    0,
    Opcode_ae_mulssfp24s_hl_lh_encode_fns, 0, 0 },
  { "ae_mulssp24s.hl.lh", ICLASS_ae_iclass_mulssp24s_hl_lh,
    0,
    Opcode_ae_mulssp24s_hl_lh_encode_fns, 0, 0 },
  { "ae_sha32", ICLASS_ae_iclass_sha32,
    0,
    Opcode_ae_sha32_encode_fns, 0, 0 },
  { "ae_vldl32t", ICLASS_ae_iclass_vldl32t,
    0,
    Opcode_ae_vldl32t_encode_fns, 1, Opcode_ae_vldl32t_funcUnit_uses },
  { "ae_vldl16t", ICLASS_ae_iclass_vldl16t,
    0,
    Opcode_ae_vldl16t_encode_fns, 1, Opcode_ae_vldl16t_funcUnit_uses },
  { "ae_vldl16c", ICLASS_ae_iclass_vldl16c,
    0,
    Opcode_ae_vldl16c_encode_fns, 3, Opcode_ae_vldl16c_funcUnit_uses },
  { "ae_vldsht", ICLASS_ae_iclass_vldsht,
    0,
    Opcode_ae_vldsht_encode_fns, 3, Opcode_ae_vldsht_funcUnit_uses },
  { "ae_lb", ICLASS_ae_iclass_lb,
    0,
    Opcode_ae_lb_encode_fns, 1, Opcode_ae_lb_funcUnit_uses },
  { "ae_lbi", ICLASS_ae_iclass_lbi,
    0,
    Opcode_ae_lbi_encode_fns, 1, Opcode_ae_lbi_funcUnit_uses },
  { "ae_lbk", ICLASS_ae_iclass_lbk,
    0,
    Opcode_ae_lbk_encode_fns, 1, Opcode_ae_lbk_funcUnit_uses },
  { "ae_lbki", ICLASS_ae_iclass_lbki,
    0,
    Opcode_ae_lbki_encode_fns, 1, Opcode_ae_lbki_funcUnit_uses },
  { "ae_db", ICLASS_ae_iclass_db,
    0,
    Opcode_ae_db_encode_fns, 2, Opcode_ae_db_funcUnit_uses },
  { "ae_dbi", ICLASS_ae_iclass_dbi,
    0,
    Opcode_ae_dbi_encode_fns, 2, Opcode_ae_dbi_funcUnit_uses },
  { "ae_vlel32t", ICLASS_ae_iclass_vlel32t,
    0,
    Opcode_ae_vlel32t_encode_fns, 1, Opcode_ae_vlel32t_funcUnit_uses },
  { "ae_vlel16t", ICLASS_ae_iclass_vlel16t,
    0,
    Opcode_ae_vlel16t_encode_fns, 1, Opcode_ae_vlel16t_funcUnit_uses },
  { "ae_sb", ICLASS_ae_iclass_sb,
    0,
    Opcode_ae_sb_encode_fns, 2, Opcode_ae_sb_funcUnit_uses },
  { "ae_sbi", ICLASS_ae_iclass_sbi,
    0,
    Opcode_ae_sbi_encode_fns, 2, Opcode_ae_sbi_funcUnit_uses },
  { "ae_vles16c", ICLASS_ae_iclass_vles16c,
    0,
    Opcode_ae_vles16c_encode_fns, 2, Opcode_ae_vles16c_funcUnit_uses },
  { "ae_sbf", ICLASS_ae_iclass_sbf,
    0,
    Opcode_ae_sbf_encode_fns, 2, Opcode_ae_sbf_funcUnit_uses }
};

enum xtensa_opcode_id {
  OPCODE_EXCW,
  OPCODE_RFE,
  OPCODE_RFDE,
  OPCODE_SYSCALL,
  OPCODE_SIMCALL,
  OPCODE_CALL12,
  OPCODE_CALL8,
  OPCODE_CALL4,
  OPCODE_CALLX12,
  OPCODE_CALLX8,
  OPCODE_CALLX4,
  OPCODE_ENTRY,
  OPCODE_MOVSP,
  OPCODE_ROTW,
  OPCODE_RETW,
  OPCODE_RETW_N,
  OPCODE_RFWO,
  OPCODE_RFWU,
  OPCODE_L32E,
  OPCODE_S32E,
  OPCODE_RSR_WINDOWBASE,
  OPCODE_WSR_WINDOWBASE,
  OPCODE_XSR_WINDOWBASE,
  OPCODE_RSR_WINDOWSTART,
  OPCODE_WSR_WINDOWSTART,
  OPCODE_XSR_WINDOWSTART,
  OPCODE_ADD_N,
  OPCODE_ADDI_N,
  OPCODE_BEQZ_N,
  OPCODE_BNEZ_N,
  OPCODE_ILL_N,
  OPCODE_L32I_N,
  OPCODE_MOV_N,
  OPCODE_MOVI_N,
  OPCODE_NOP_N,
  OPCODE_RET_N,
  OPCODE_S32I_N,
  OPCODE_RUR_THREADPTR,
  OPCODE_WUR_THREADPTR,
  OPCODE_ADDI,
  OPCODE_ADDMI,
  OPCODE_ADD,
  OPCODE_SUB,
  OPCODE_ADDX2,
  OPCODE_ADDX4,
  OPCODE_ADDX8,
  OPCODE_SUBX2,
  OPCODE_SUBX4,
  OPCODE_SUBX8,
  OPCODE_AND,
  OPCODE_OR,
  OPCODE_XOR,
  OPCODE_BEQI,
  OPCODE_BNEI,
  OPCODE_BGEI,
  OPCODE_BLTI,
  OPCODE_BBCI,
  OPCODE_BBSI,
  OPCODE_BGEUI,
  OPCODE_BLTUI,
  OPCODE_BEQ,
  OPCODE_BNE,
  OPCODE_BGE,
  OPCODE_BLT,
  OPCODE_BGEU,
  OPCODE_BLTU,
  OPCODE_BANY,
  OPCODE_BNONE,
  OPCODE_BALL,
  OPCODE_BNALL,
  OPCODE_BBC,
  OPCODE_BBS,
  OPCODE_BEQZ,
  OPCODE_BNEZ,
  OPCODE_BGEZ,
  OPCODE_BLTZ,
  OPCODE_CALL0,
  OPCODE_CALLX0,
  OPCODE_EXTUI,
  OPCODE_ILL,
  OPCODE_J,
  OPCODE_JX,
  OPCODE_L16UI,
  OPCODE_L16SI,
  OPCODE_L32I,
  OPCODE_L32R,
  OPCODE_L8UI,
  OPCODE_LOOP,
  OPCODE_LOOPNEZ,
  OPCODE_LOOPGTZ,
  OPCODE_MOVI,
  OPCODE_MOVEQZ,
  OPCODE_MOVNEZ,
  OPCODE_MOVLTZ,
  OPCODE_MOVGEZ,
  OPCODE_NEG,
  OPCODE_ABS,
  OPCODE_NOP,
  OPCODE_RET,
  OPCODE_S16I,
  OPCODE_S32I,
  OPCODE_S8I,
  OPCODE_SSR,
  OPCODE_SSL,
  OPCODE_SSA8L,
  OPCODE_SSA8B,
  OPCODE_SSAI,
  OPCODE_SLL,
  OPCODE_SRC,
  OPCODE_SRL,
  OPCODE_SRA,
  OPCODE_SLLI,
  OPCODE_SRAI,
  OPCODE_SRLI,
  OPCODE_MEMW,
  OPCODE_EXTW,
  OPCODE_ISYNC,
  OPCODE_RSYNC,
  OPCODE_ESYNC,
  OPCODE_DSYNC,
  OPCODE_RSIL,
  OPCODE_RSR_LEND,
  OPCODE_WSR_LEND,
  OPCODE_XSR_LEND,
  OPCODE_RSR_LCOUNT,
  OPCODE_WSR_LCOUNT,
  OPCODE_XSR_LCOUNT,
  OPCODE_RSR_LBEG,
  OPCODE_WSR_LBEG,
  OPCODE_XSR_LBEG,
  OPCODE_RSR_SAR,
  OPCODE_WSR_SAR,
  OPCODE_XSR_SAR,
  OPCODE_RSR_LITBASE,
  OPCODE_WSR_LITBASE,
  OPCODE_XSR_LITBASE,
  OPCODE_RSR_176,
  OPCODE_WSR_176,
  OPCODE_RSR_208,
  OPCODE_RSR_PS,
  OPCODE_WSR_PS,
  OPCODE_XSR_PS,
  OPCODE_RSR_EPC1,
  OPCODE_WSR_EPC1,
  OPCODE_XSR_EPC1,
  OPCODE_RSR_EXCSAVE1,
  OPCODE_WSR_EXCSAVE1,
  OPCODE_XSR_EXCSAVE1,
  OPCODE_RSR_EPC2,
  OPCODE_WSR_EPC2,
  OPCODE_XSR_EPC2,
  OPCODE_RSR_EXCSAVE2,
  OPCODE_WSR_EXCSAVE2,
  OPCODE_XSR_EXCSAVE2,
  OPCODE_RSR_EPS2,
  OPCODE_WSR_EPS2,
  OPCODE_XSR_EPS2,
  OPCODE_RSR_EXCVADDR,
  OPCODE_WSR_EXCVADDR,
  OPCODE_XSR_EXCVADDR,
  OPCODE_RSR_DEPC,
  OPCODE_WSR_DEPC,
  OPCODE_XSR_DEPC,
  OPCODE_RSR_EXCCAUSE,
  OPCODE_WSR_EXCCAUSE,
  OPCODE_XSR_EXCCAUSE,
  OPCODE_RSR_MISC0,
  OPCODE_WSR_MISC0,
  OPCODE_XSR_MISC0,
  OPCODE_RSR_MISC1,
  OPCODE_WSR_MISC1,
  OPCODE_XSR_MISC1,
  OPCODE_RSR_PRID,
  OPCODE_RSR_VECBASE,
  OPCODE_WSR_VECBASE,
  OPCODE_XSR_VECBASE,
  OPCODE_MUL16U,
  OPCODE_MUL16S,
  OPCODE_MULL,
  OPCODE_RFI,
  OPCODE_WAITI,
  OPCODE_RSR_INTERRUPT,
  OPCODE_WSR_INTSET,
  OPCODE_WSR_INTCLEAR,
  OPCODE_RSR_INTENABLE,
  OPCODE_WSR_INTENABLE,
  OPCODE_XSR_INTENABLE,
  OPCODE_BREAK,
  OPCODE_BREAK_N,
  OPCODE_RSR_DEBUGCAUSE,
  OPCODE_WSR_DEBUGCAUSE,
  OPCODE_XSR_DEBUGCAUSE,
  OPCODE_RSR_ICOUNT,
  OPCODE_WSR_ICOUNT,
  OPCODE_XSR_ICOUNT,
  OPCODE_RSR_ICOUNTLEVEL,
  OPCODE_WSR_ICOUNTLEVEL,
  OPCODE_XSR_ICOUNTLEVEL,
  OPCODE_RSR_DDR,
  OPCODE_WSR_DDR,
  OPCODE_XSR_DDR,
  OPCODE_RFDO,
  OPCODE_RFDD,
  OPCODE_ANDB,
  OPCODE_ANDBC,
  OPCODE_ORB,
  OPCODE_ORBC,
  OPCODE_XORB,
  OPCODE_ANY4,
  OPCODE_ALL4,
  OPCODE_ANY8,
  OPCODE_ALL8,
  OPCODE_BF,
  OPCODE_BT,
  OPCODE_MOVF,
  OPCODE_MOVT,
  OPCODE_RSR_BR,
  OPCODE_WSR_BR,
  OPCODE_XSR_BR,
  OPCODE_RSR_CCOUNT,
  OPCODE_WSR_CCOUNT,
  OPCODE_XSR_CCOUNT,
  OPCODE_RSR_CCOMPARE0,
  OPCODE_WSR_CCOMPARE0,
  OPCODE_XSR_CCOMPARE0,
  OPCODE_RSR_CCOMPARE1,
  OPCODE_WSR_CCOMPARE1,
  OPCODE_XSR_CCOMPARE1,
  OPCODE_IPF,
  OPCODE_IHI,
  OPCODE_III,
  OPCODE_LICT,
  OPCODE_LICW,
  OPCODE_SICT,
  OPCODE_SICW,
  OPCODE_DHWB,
  OPCODE_DHWBI,
  OPCODE_DIWB,
  OPCODE_DIWBI,
  OPCODE_DHI,
  OPCODE_DII,
  OPCODE_DPFR,
  OPCODE_DPFW,
  OPCODE_DPFRO,
  OPCODE_DPFWO,
  OPCODE_SDCT,
  OPCODE_LDCT,
  OPCODE_WSR_PTEVADDR,
  OPCODE_RSR_PTEVADDR,
  OPCODE_XSR_PTEVADDR,
  OPCODE_RSR_RASID,
  OPCODE_WSR_RASID,
  OPCODE_XSR_RASID,
  OPCODE_RSR_ITLBCFG,
  OPCODE_WSR_ITLBCFG,
  OPCODE_XSR_ITLBCFG,
  OPCODE_RSR_DTLBCFG,
  OPCODE_WSR_DTLBCFG,
  OPCODE_XSR_DTLBCFG,
  OPCODE_IDTLB,
  OPCODE_PDTLB,
  OPCODE_RDTLB0,
  OPCODE_RDTLB1,
  OPCODE_WDTLB,
  OPCODE_IITLB,
  OPCODE_PITLB,
  OPCODE_RITLB0,
  OPCODE_RITLB1,
  OPCODE_WITLB,
  OPCODE_LDPTE,
  OPCODE_HWWITLBA,
  OPCODE_HWWDTLBA,
  OPCODE_RSR_CPENABLE,
  OPCODE_WSR_CPENABLE,
  OPCODE_XSR_CPENABLE,
  OPCODE_CLAMPS,
  OPCODE_MIN,
  OPCODE_MAX,
  OPCODE_MINU,
  OPCODE_MAXU,
  OPCODE_NSA,
  OPCODE_NSAU,
  OPCODE_SEXT,
  OPCODE_L32AI,
  OPCODE_S32RI,
  OPCODE_S32C1I,
  OPCODE_RSR_SCOMPARE1,
  OPCODE_WSR_SCOMPARE1,
  OPCODE_XSR_SCOMPARE1,
  OPCODE_RSR_ATOMCTL,
  OPCODE_WSR_ATOMCTL,
  OPCODE_XSR_ATOMCTL,
  OPCODE_RER,
  OPCODE_WER,
  OPCODE_RUR_AE_OVF_SAR,
  OPCODE_WUR_AE_OVF_SAR,
  OPCODE_RUR_AE_BITHEAD,
  OPCODE_WUR_AE_BITHEAD,
  OPCODE_RUR_AE_TS_FTS_BU_BP,
  OPCODE_WUR_AE_TS_FTS_BU_BP,
  OPCODE_RUR_AE_SD_NO,
  OPCODE_WUR_AE_SD_NO,
  OPCODE_RUR_AE_OVERFLOW,
  OPCODE_WUR_AE_OVERFLOW,
  OPCODE_RUR_AE_SAR,
  OPCODE_WUR_AE_SAR,
  OPCODE_RUR_AE_BITPTR,
  OPCODE_WUR_AE_BITPTR,
  OPCODE_RUR_AE_BITSUSED,
  OPCODE_WUR_AE_BITSUSED,
  OPCODE_RUR_AE_TABLESIZE,
  OPCODE_WUR_AE_TABLESIZE,
  OPCODE_RUR_AE_FIRST_TS,
  OPCODE_WUR_AE_FIRST_TS,
  OPCODE_RUR_AE_NEXTOFFSET,
  OPCODE_WUR_AE_NEXTOFFSET,
  OPCODE_RUR_AE_SEARCHDONE,
  OPCODE_WUR_AE_SEARCHDONE,
  OPCODE_AE_LP16F_I,
  OPCODE_AE_LP16F_IU,
  OPCODE_AE_LP16F_X,
  OPCODE_AE_LP16F_XU,
  OPCODE_AE_LP24_I,
  OPCODE_AE_LP24_IU,
  OPCODE_AE_LP24_X,
  OPCODE_AE_LP24_XU,
  OPCODE_AE_LP24F_I,
  OPCODE_AE_LP24F_IU,
  OPCODE_AE_LP24F_X,
  OPCODE_AE_LP24F_XU,
  OPCODE_AE_LP16X2F_I,
  OPCODE_AE_LP16X2F_IU,
  OPCODE_AE_LP16X2F_X,
  OPCODE_AE_LP16X2F_XU,
  OPCODE_AE_LP24X2F_I,
  OPCODE_AE_LP24X2F_IU,
  OPCODE_AE_LP24X2F_X,
  OPCODE_AE_LP24X2F_XU,
  OPCODE_AE_LP24X2_I,
  OPCODE_AE_LP24X2_IU,
  OPCODE_AE_LP24X2_X,
  OPCODE_AE_LP24X2_XU,
  OPCODE_AE_SP16X2F_I,
  OPCODE_AE_SP16X2F_IU,
  OPCODE_AE_SP16X2F_X,
  OPCODE_AE_SP16X2F_XU,
  OPCODE_AE_SP24X2S_I,
  OPCODE_AE_SP24X2S_IU,
  OPCODE_AE_SP24X2S_X,
  OPCODE_AE_SP24X2S_XU,
  OPCODE_AE_SP24X2F_I,
  OPCODE_AE_SP24X2F_IU,
  OPCODE_AE_SP24X2F_X,
  OPCODE_AE_SP24X2F_XU,
  OPCODE_AE_SP16F_L_I,
  OPCODE_AE_SP16F_L_IU,
  OPCODE_AE_SP16F_L_X,
  OPCODE_AE_SP16F_L_XU,
  OPCODE_AE_SP24S_L_I,
  OPCODE_AE_SP24S_L_IU,
  OPCODE_AE_SP24S_L_X,
  OPCODE_AE_SP24S_L_XU,
  OPCODE_AE_SP24F_L_I,
  OPCODE_AE_SP24F_L_IU,
  OPCODE_AE_SP24F_L_X,
  OPCODE_AE_SP24F_L_XU,
  OPCODE_AE_LQ56_I,
  OPCODE_AE_LQ56_IU,
  OPCODE_AE_LQ56_X,
  OPCODE_AE_LQ56_XU,
  OPCODE_AE_LQ32F_I,
  OPCODE_AE_LQ32F_IU,
  OPCODE_AE_LQ32F_X,
  OPCODE_AE_LQ32F_XU,
  OPCODE_AE_SQ56S_I,
  OPCODE_AE_SQ56S_IU,
  OPCODE_AE_SQ56S_X,
  OPCODE_AE_SQ56S_XU,
  OPCODE_AE_SQ32F_I,
  OPCODE_AE_SQ32F_IU,
  OPCODE_AE_SQ32F_X,
  OPCODE_AE_SQ32F_XU,
  OPCODE_AE_ZEROP48,
  OPCODE_AE_MOVP48,
  OPCODE_AE_SELP24_LL,
  OPCODE_AE_SELP24_LH,
  OPCODE_AE_SELP24_HL,
  OPCODE_AE_SELP24_HH,
  OPCODE_AE_MOVTP24X2,
  OPCODE_AE_MOVFP24X2,
  OPCODE_AE_MOVTP48,
  OPCODE_AE_MOVFP48,
  OPCODE_AE_MOVPA24X2,
  OPCODE_AE_TRUNCP24A32X2,
  OPCODE_AE_CVTA32P24_L,
  OPCODE_AE_CVTA32P24_H,
  OPCODE_AE_CVTP24A16X2_LL,
  OPCODE_AE_CVTP24A16X2_LH,
  OPCODE_AE_CVTP24A16X2_HL,
  OPCODE_AE_CVTP24A16X2_HH,
  OPCODE_AE_TRUNCP24Q48X2,
  OPCODE_AE_TRUNCP16,
  OPCODE_AE_ROUNDSP24Q48SYM,
  OPCODE_AE_ROUNDSP24Q48ASYM,
  OPCODE_AE_ROUNDSP16Q48SYM,
  OPCODE_AE_ROUNDSP16Q48ASYM,
  OPCODE_AE_ROUNDSP16SYM,
  OPCODE_AE_ROUNDSP16ASYM,
  OPCODE_AE_ZEROQ56,
  OPCODE_AE_MOVQ56,
  OPCODE_AE_MOVTQ56,
  OPCODE_AE_MOVFQ56,
  OPCODE_AE_CVTQ48A32S,
  OPCODE_AE_CVTQ48P24S_L,
  OPCODE_AE_CVTQ48P24S_H,
  OPCODE_AE_SATQ48S,
  OPCODE_AE_TRUNCQ32,
  OPCODE_AE_ROUNDSQ32SYM,
  OPCODE_AE_ROUNDSQ32ASYM,
  OPCODE_AE_TRUNCA32Q48,
  OPCODE_AE_MOVAP24S_L,
  OPCODE_AE_MOVAP24S_H,
  OPCODE_AE_TRUNCA16P24S_L,
  OPCODE_AE_TRUNCA16P24S_H,
  OPCODE_AE_ADDP24,
  OPCODE_AE_SUBP24,
  OPCODE_AE_NEGP24,
  OPCODE_AE_ABSP24,
  OPCODE_AE_MAXP24S,
  OPCODE_AE_MINP24S,
  OPCODE_AE_MAXBP24S,
  OPCODE_AE_MINBP24S,
  OPCODE_AE_ADDSP24S,
  OPCODE_AE_SUBSP24S,
  OPCODE_AE_NEGSP24S,
  OPCODE_AE_ABSSP24S,
  OPCODE_AE_ANDP48,
  OPCODE_AE_NANDP48,
  OPCODE_AE_ORP48,
  OPCODE_AE_XORP48,
  OPCODE_AE_LTP24S,
  OPCODE_AE_LEP24S,
  OPCODE_AE_EQP24,
  OPCODE_AE_ADDQ56,
  OPCODE_AE_SUBQ56,
  OPCODE_AE_NEGQ56,
  OPCODE_AE_ABSQ56,
  OPCODE_AE_MAXQ56S,
  OPCODE_AE_MINQ56S,
  OPCODE_AE_MAXBQ56S,
  OPCODE_AE_MINBQ56S,
  OPCODE_AE_ADDSQ56S,
  OPCODE_AE_SUBSQ56S,
  OPCODE_AE_NEGSQ56S,
  OPCODE_AE_ABSSQ56S,
  OPCODE_AE_ANDQ56,
  OPCODE_AE_NANDQ56,
  OPCODE_AE_ORQ56,
  OPCODE_AE_XORQ56,
  OPCODE_AE_SLLIP24,
  OPCODE_AE_SRLIP24,
  OPCODE_AE_SRAIP24,
  OPCODE_AE_SLLSP24,
  OPCODE_AE_SRLSP24,
  OPCODE_AE_SRASP24,
  OPCODE_AE_SLLISP24S,
  OPCODE_AE_SLLSSP24S,
  OPCODE_AE_SLLIQ56,
  OPCODE_AE_SRLIQ56,
  OPCODE_AE_SRAIQ56,
  OPCODE_AE_SLLSQ56,
  OPCODE_AE_SRLSQ56,
  OPCODE_AE_SRASQ56,
  OPCODE_AE_SLLAQ56,
  OPCODE_AE_SRLAQ56,
  OPCODE_AE_SRAAQ56,
  OPCODE_AE_SLLISQ56S,
  OPCODE_AE_SLLSSQ56S,
  OPCODE_AE_SLLASQ56S,
  OPCODE_AE_LTQ56S,
  OPCODE_AE_LEQ56S,
  OPCODE_AE_EQQ56,
  OPCODE_AE_NSAQ56S,
  OPCODE_AE_MULFS32P16S_LL,
  OPCODE_AE_MULFP24S_LL,
  OPCODE_AE_MULP24S_LL,
  OPCODE_AE_MULFS32P16S_LH,
  OPCODE_AE_MULFP24S_LH,
  OPCODE_AE_MULP24S_LH,
  OPCODE_AE_MULFS32P16S_HL,
  OPCODE_AE_MULFP24S_HL,
  OPCODE_AE_MULP24S_HL,
  OPCODE_AE_MULFS32P16S_HH,
  OPCODE_AE_MULFP24S_HH,
  OPCODE_AE_MULP24S_HH,
  OPCODE_AE_MULAFS32P16S_LL,
  OPCODE_AE_MULAFP24S_LL,
  OPCODE_AE_MULAP24S_LL,
  OPCODE_AE_MULAFS32P16S_LH,
  OPCODE_AE_MULAFP24S_LH,
  OPCODE_AE_MULAP24S_LH,
  OPCODE_AE_MULAFS32P16S_HL,
  OPCODE_AE_MULAFP24S_HL,
  OPCODE_AE_MULAP24S_HL,
  OPCODE_AE_MULAFS32P16S_HH,
  OPCODE_AE_MULAFP24S_HH,
  OPCODE_AE_MULAP24S_HH,
  OPCODE_AE_MULSFS32P16S_LL,
  OPCODE_AE_MULSFP24S_LL,
  OPCODE_AE_MULSP24S_LL,
  OPCODE_AE_MULSFS32P16S_LH,
  OPCODE_AE_MULSFP24S_LH,
  OPCODE_AE_MULSP24S_LH,
  OPCODE_AE_MULSFS32P16S_HL,
  OPCODE_AE_MULSFP24S_HL,
  OPCODE_AE_MULSP24S_HL,
  OPCODE_AE_MULSFS32P16S_HH,
  OPCODE_AE_MULSFP24S_HH,
  OPCODE_AE_MULSP24S_HH,
  OPCODE_AE_MULAFS56P24S_LL,
  OPCODE_AE_MULAS56P24S_LL,
  OPCODE_AE_MULAFS56P24S_LH,
  OPCODE_AE_MULAS56P24S_LH,
  OPCODE_AE_MULAFS56P24S_HL,
  OPCODE_AE_MULAS56P24S_HL,
  OPCODE_AE_MULAFS56P24S_HH,
  OPCODE_AE_MULAS56P24S_HH,
  OPCODE_AE_MULSFS56P24S_LL,
  OPCODE_AE_MULSS56P24S_LL,
  OPCODE_AE_MULSFS56P24S_LH,
  OPCODE_AE_MULSS56P24S_LH,
  OPCODE_AE_MULSFS56P24S_HL,
  OPCODE_AE_MULSS56P24S_HL,
  OPCODE_AE_MULSFS56P24S_HH,
  OPCODE_AE_MULSS56P24S_HH,
  OPCODE_AE_MULFQ32SP16S_L,
  OPCODE_AE_MULFQ32SP16S_H,
  OPCODE_AE_MULFQ32SP16U_L,
  OPCODE_AE_MULFQ32SP16U_H,
  OPCODE_AE_MULQ32SP16S_L,
  OPCODE_AE_MULQ32SP16S_H,
  OPCODE_AE_MULQ32SP16U_L,
  OPCODE_AE_MULQ32SP16U_H,
  OPCODE_AE_MULAFQ32SP16S_L,
  OPCODE_AE_MULAFQ32SP16S_H,
  OPCODE_AE_MULAFQ32SP16U_L,
  OPCODE_AE_MULAFQ32SP16U_H,
  OPCODE_AE_MULAQ32SP16S_L,
  OPCODE_AE_MULAQ32SP16S_H,
  OPCODE_AE_MULAQ32SP16U_L,
  OPCODE_AE_MULAQ32SP16U_H,
  OPCODE_AE_MULSFQ32SP16S_L,
  OPCODE_AE_MULSFQ32SP16S_H,
  OPCODE_AE_MULSFQ32SP16U_L,
  OPCODE_AE_MULSFQ32SP16U_H,
  OPCODE_AE_MULSQ32SP16S_L,
  OPCODE_AE_MULSQ32SP16S_H,
  OPCODE_AE_MULSQ32SP16U_L,
  OPCODE_AE_MULSQ32SP16U_H,
  OPCODE_AE_MULZAAQ32SP16S_LL,
  OPCODE_AE_MULZAAFQ32SP16S_LL,
  OPCODE_AE_MULZAAQ32SP16U_LL,
  OPCODE_AE_MULZAAFQ32SP16U_LL,
  OPCODE_AE_MULZAAQ32SP16S_HH,
  OPCODE_AE_MULZAAFQ32SP16S_HH,
  OPCODE_AE_MULZAAQ32SP16U_HH,
  OPCODE_AE_MULZAAFQ32SP16U_HH,
  OPCODE_AE_MULZAAQ32SP16S_LH,
  OPCODE_AE_MULZAAFQ32SP16S_LH,
  OPCODE_AE_MULZAAQ32SP16U_LH,
  OPCODE_AE_MULZAAFQ32SP16U_LH,
  OPCODE_AE_MULZASQ32SP16S_LL,
  OPCODE_AE_MULZASFQ32SP16S_LL,
  OPCODE_AE_MULZASQ32SP16U_LL,
  OPCODE_AE_MULZASFQ32SP16U_LL,
  OPCODE_AE_MULZASQ32SP16S_HH,
  OPCODE_AE_MULZASFQ32SP16S_HH,
  OPCODE_AE_MULZASQ32SP16U_HH,
  OPCODE_AE_MULZASFQ32SP16U_HH,
  OPCODE_AE_MULZASQ32SP16S_LH,
  OPCODE_AE_MULZASFQ32SP16S_LH,
  OPCODE_AE_MULZASQ32SP16U_LH,
  OPCODE_AE_MULZASFQ32SP16U_LH,
  OPCODE_AE_MULZSAQ32SP16S_LL,
  OPCODE_AE_MULZSAFQ32SP16S_LL,
  OPCODE_AE_MULZSAQ32SP16U_LL,
  OPCODE_AE_MULZSAFQ32SP16U_LL,
  OPCODE_AE_MULZSAQ32SP16S_HH,
  OPCODE_AE_MULZSAFQ32SP16S_HH,
  OPCODE_AE_MULZSAQ32SP16U_HH,
  OPCODE_AE_MULZSAFQ32SP16U_HH,
  OPCODE_AE_MULZSAQ32SP16S_LH,
  OPCODE_AE_MULZSAFQ32SP16S_LH,
  OPCODE_AE_MULZSAQ32SP16U_LH,
  OPCODE_AE_MULZSAFQ32SP16U_LH,
  OPCODE_AE_MULZSSQ32SP16S_LL,
  OPCODE_AE_MULZSSFQ32SP16S_LL,
  OPCODE_AE_MULZSSQ32SP16U_LL,
  OPCODE_AE_MULZSSFQ32SP16U_LL,
  OPCODE_AE_MULZSSQ32SP16S_HH,
  OPCODE_AE_MULZSSFQ32SP16S_HH,
  OPCODE_AE_MULZSSQ32SP16U_HH,
  OPCODE_AE_MULZSSFQ32SP16U_HH,
  OPCODE_AE_MULZSSQ32SP16S_LH,
  OPCODE_AE_MULZSSFQ32SP16S_LH,
  OPCODE_AE_MULZSSQ32SP16U_LH,
  OPCODE_AE_MULZSSFQ32SP16U_LH,
  OPCODE_AE_MULZAAFP24S_HH_LL,
  OPCODE_AE_MULZAAP24S_HH_LL,
  OPCODE_AE_MULZAAFP24S_HL_LH,
  OPCODE_AE_MULZAAP24S_HL_LH,
  OPCODE_AE_MULZASFP24S_HH_LL,
  OPCODE_AE_MULZASP24S_HH_LL,
  OPCODE_AE_MULZASFP24S_HL_LH,
  OPCODE_AE_MULZASP24S_HL_LH,
  OPCODE_AE_MULZSAFP24S_HH_LL,
  OPCODE_AE_MULZSAP24S_HH_LL,
  OPCODE_AE_MULZSAFP24S_HL_LH,
  OPCODE_AE_MULZSAP24S_HL_LH,
  OPCODE_AE_MULZSSFP24S_HH_LL,
  OPCODE_AE_MULZSSP24S_HH_LL,
  OPCODE_AE_MULZSSFP24S_HL_LH,
  OPCODE_AE_MULZSSP24S_HL_LH,
  OPCODE_AE_MULAAFP24S_HH_LL,
  OPCODE_AE_MULAAP24S_HH_LL,
  OPCODE_AE_MULAAFP24S_HL_LH,
  OPCODE_AE_MULAAP24S_HL_LH,
  OPCODE_AE_MULASFP24S_HH_LL,
  OPCODE_AE_MULASP24S_HH_LL,
  OPCODE_AE_MULASFP24S_HL_LH,
  OPCODE_AE_MULASP24S_HL_LH,
  OPCODE_AE_MULSAFP24S_HH_LL,
  OPCODE_AE_MULSAP24S_HH_LL,
  OPCODE_AE_MULSAFP24S_HL_LH,
  OPCODE_AE_MULSAP24S_HL_LH,
  OPCODE_AE_MULSSFP24S_HH_LL,
  OPCODE_AE_MULSSP24S_HH_LL,
  OPCODE_AE_MULSSFP24S_HL_LH,
  OPCODE_AE_MULSSP24S_HL_LH,
  OPCODE_AE_SHA32,
  OPCODE_AE_VLDL32T,
  OPCODE_AE_VLDL16T,
  OPCODE_AE_VLDL16C,
  OPCODE_AE_VLDSHT,
  OPCODE_AE_LB,
  OPCODE_AE_LBI,
  OPCODE_AE_LBK,
  OPCODE_AE_LBKI,
  OPCODE_AE_DB,
  OPCODE_AE_DBI,
  OPCODE_AE_VLEL32T,
  OPCODE_AE_VLEL16T,
  OPCODE_AE_SB,
  OPCODE_AE_SBI,
  OPCODE_AE_VLES16C,
  OPCODE_AE_SBF
};


/* Slot-specific opcode decode functions.  */

static int
Slot_inst_decode (const xtensa_insnbuf insn)
{
  switch (Field_op0_Slot_inst_get (insn))
    {
    case 0:
      switch (Field_op1_Slot_inst_get (insn))
	{
	case 0:
	  switch (Field_op2_Slot_inst_get (insn))
	    {
	    case 0:
	      switch (Field_r_Slot_inst_get (insn))
		{
		case 0:
		  switch (Field_m_Slot_inst_get (insn))
		    {
		    case 0:
		      if (Field_s_Slot_inst_get (insn) == 0 &&
			  Field_n_Slot_inst_get (insn) == 0)
			return OPCODE_ILL;
		      break;
		    case 2:
		      switch (Field_n_Slot_inst_get (insn))
			{
			case 0:
			  return OPCODE_RET;
			case 1:
			  return OPCODE_RETW;
			case 2:
			  return OPCODE_JX;
			}
		      break;
		    case 3:
		      switch (Field_n_Slot_inst_get (insn))
			{
			case 0:
			  return OPCODE_CALLX0;
			case 1:
			  return OPCODE_CALLX4;
			case 2:
			  return OPCODE_CALLX8;
			case 3:
			  return OPCODE_CALLX12;
			}
		      break;
		    }
		  break;
		case 1:
		  return OPCODE_MOVSP;
		case 2:
		  if (Field_s_Slot_inst_get (insn) == 0)
		    {
		      switch (Field_t_Slot_inst_get (insn))
			{
			case 0:
			  return OPCODE_ISYNC;
			case 1:
			  return OPCODE_RSYNC;
			case 2:
			  return OPCODE_ESYNC;
			case 3:
			  return OPCODE_DSYNC;
			case 8:
			  return OPCODE_EXCW;
			case 12:
			  return OPCODE_MEMW;
			case 13:
			  return OPCODE_EXTW;
			case 15:
			  return OPCODE_NOP;
			}
		    }
		  break;
		case 3:
		  switch (Field_t_Slot_inst_get (insn))
		    {
		    case 0:
		      switch (Field_s_Slot_inst_get (insn))
			{
			case 0:
			  return OPCODE_RFE;
			case 2:
			  return OPCODE_RFDE;
			case 4:
			  return OPCODE_RFWO;
			case 5:
			  return OPCODE_RFWU;
			}
		      break;
		    case 1:
		      return OPCODE_RFI;
		    }
		  break;
		case 4:
		  return OPCODE_BREAK;
		case 5:
		  switch (Field_s_Slot_inst_get (insn))
		    {
		    case 0:
		      if (Field_t_Slot_inst_get (insn) == 0)
			return OPCODE_SYSCALL;
		      break;
		    case 1:
		      if (Field_t_Slot_inst_get (insn) == 0)
			return OPCODE_SIMCALL;
		      break;
		    }
		  break;
		case 6:
		  return OPCODE_RSIL;
		case 7:
		  if (Field_t_Slot_inst_get (insn) == 0)
		    return OPCODE_WAITI;
		  break;
		case 8:
		  return OPCODE_ANY4;
		case 9:
		  return OPCODE_ALL4;
		case 10:
		  return OPCODE_ANY8;
		case 11:
		  return OPCODE_ALL8;
		}
	      break;
	    case 1:
	      return OPCODE_AND;
	    case 2:
	      return OPCODE_OR;
	    case 3:
	      return OPCODE_XOR;
	    case 4:
	      switch (Field_r_Slot_inst_get (insn))
		{
		case 0:
		  if (Field_t_Slot_inst_get (insn) == 0)
		    return OPCODE_SSR;
		  break;
		case 1:
		  if (Field_t_Slot_inst_get (insn) == 0)
		    return OPCODE_SSL;
		  break;
		case 2:
		  if (Field_t_Slot_inst_get (insn) == 0)
		    return OPCODE_SSA8L;
		  break;
		case 3:
		  if (Field_t_Slot_inst_get (insn) == 0)
		    return OPCODE_SSA8B;
		  break;
		case 4:
		  if (Field_thi3_Slot_inst_get (insn) == 0)
		    return OPCODE_SSAI;
		  break;
		case 6:
		  return OPCODE_RER;
		case 7:
		  return OPCODE_WER;
		case 8:
		  if (Field_s_Slot_inst_get (insn) == 0)
		    return OPCODE_ROTW;
		  break;
		case 14:
		  return OPCODE_NSA;
		case 15:
		  return OPCODE_NSAU;
		}
	      break;
	    case 5:
	      switch (Field_r_Slot_inst_get (insn))
		{
		case 1:
		  return OPCODE_HWWITLBA;
		case 3:
		  return OPCODE_RITLB0;
		case 4:
		  if (Field_t_Slot_inst_get (insn) == 0)
		    return OPCODE_IITLB;
		  break;
		case 5:
		  return OPCODE_PITLB;
		case 6:
		  return OPCODE_WITLB;
		case 7:
		  return OPCODE_RITLB1;
		case 9:
		  return OPCODE_HWWDTLBA;
		case 11:
		  return OPCODE_RDTLB0;
		case 12:
		  if (Field_t_Slot_inst_get (insn) == 0)
		    return OPCODE_IDTLB;
		  break;
		case 13:
		  return OPCODE_PDTLB;
		case 14:
		  return OPCODE_WDTLB;
		case 15:
		  return OPCODE_RDTLB1;
		}
	      break;
	    case 6:
	      switch (Field_s_Slot_inst_get (insn))
		{
		case 0:
		  return OPCODE_NEG;
		case 1:
		  return OPCODE_ABS;
		}
	      break;
	    case 8:
	      return OPCODE_ADD;
	    case 9:
	      return OPCODE_ADDX2;
	    case 10:
	      return OPCODE_ADDX4;
	    case 11:
	      return OPCODE_ADDX8;
	    case 12:
	      return OPCODE_SUB;
	    case 13:
	      return OPCODE_SUBX2;
	    case 14:
	      return OPCODE_SUBX4;
	    case 15:
	      return OPCODE_SUBX8;
	    }
	  break;
	case 1:
	  switch (Field_op2_Slot_inst_get (insn))
	    {
	    case 0:
	    case 1:
	      return OPCODE_SLLI;
	    case 2:
	    case 3:
	      return OPCODE_SRAI;
	    case 4:
	      return OPCODE_SRLI;
	    case 6:
	      switch (Field_sr_Slot_inst_get (insn))
		{
		case 0:
		  return OPCODE_XSR_LBEG;
		case 1:
		  return OPCODE_XSR_LEND;
		case 2:
		  return OPCODE_XSR_LCOUNT;
		case 3:
		  return OPCODE_XSR_SAR;
		case 4:
		  return OPCODE_XSR_BR;
		case 5:
		  return OPCODE_XSR_LITBASE;
		case 12:
		  return OPCODE_XSR_SCOMPARE1;
		case 72:
		  return OPCODE_XSR_WINDOWBASE;
		case 73:
		  return OPCODE_XSR_WINDOWSTART;
		case 83:
		  return OPCODE_XSR_PTEVADDR;
		case 90:
		  return OPCODE_XSR_RASID;
		case 91:
		  return OPCODE_XSR_ITLBCFG;
		case 92:
		  return OPCODE_XSR_DTLBCFG;
		case 99:
		  return OPCODE_XSR_ATOMCTL;
		case 104:
		  return OPCODE_XSR_DDR;
		case 177:
		  return OPCODE_XSR_EPC1;
		case 178:
		  return OPCODE_XSR_EPC2;
		case 192:
		  return OPCODE_XSR_DEPC;
		case 194:
		  return OPCODE_XSR_EPS2;
		case 209:
		  return OPCODE_XSR_EXCSAVE1;
		case 210:
		  return OPCODE_XSR_EXCSAVE2;
		case 224:
		  return OPCODE_XSR_CPENABLE;
		case 228:
		  return OPCODE_XSR_INTENABLE;
		case 230:
		  return OPCODE_XSR_PS;
		case 231:
		  return OPCODE_XSR_VECBASE;
		case 232:
		  return OPCODE_XSR_EXCCAUSE;
		case 233:
		  return OPCODE_XSR_DEBUGCAUSE;
		case 234:
		  return OPCODE_XSR_CCOUNT;
		case 236:
		  return OPCODE_XSR_ICOUNT;
		case 237:
		  return OPCODE_XSR_ICOUNTLEVEL;
		case 238:
		  return OPCODE_XSR_EXCVADDR;
		case 240:
		  return OPCODE_XSR_CCOMPARE0;
		case 241:
		  return OPCODE_XSR_CCOMPARE1;
		case 244:
		  return OPCODE_XSR_MISC0;
		case 245:
		  return OPCODE_XSR_MISC1;
		}
	      break;
	    case 8:
	      return OPCODE_SRC;
	    case 9:
	      if (Field_s_Slot_inst_get (insn) == 0)
		return OPCODE_SRL;
	      break;
	    case 10:
	      if (Field_t_Slot_inst_get (insn) == 0)
		return OPCODE_SLL;
	      break;
	    case 11:
	      if (Field_s_Slot_inst_get (insn) == 0)
		return OPCODE_SRA;
	      break;
	    case 12:
	      return OPCODE_MUL16U;
	    case 13:
	      return OPCODE_MUL16S;
	    case 15:
	      switch (Field_r_Slot_inst_get (insn))
		{
		case 0:
		  return OPCODE_LICT;
		case 1:
		  return OPCODE_SICT;
		case 2:
		  return OPCODE_LICW;
		case 3:
		  return OPCODE_SICW;
		case 8:
		  return OPCODE_LDCT;
		case 9:
		  return OPCODE_SDCT;
		case 14:
		  if (Field_t_Slot_inst_get (insn) == 0)
		    return OPCODE_RFDO;
		  if (Field_t_Slot_inst_get (insn) == 1)
		    return OPCODE_RFDD;
		  break;
		case 15:
		  return OPCODE_LDPTE;
		}
	      break;
	    }
	  break;
	case 2:
	  switch (Field_op2_Slot_inst_get (insn))
	    {
	    case 0:
	      return OPCODE_ANDB;
	    case 1:
	      return OPCODE_ANDBC;
	    case 2:
	      return OPCODE_ORB;
	    case 3:
	      return OPCODE_ORBC;
	    case 4:
	      return OPCODE_XORB;
	    case 8:
	      return OPCODE_MULL;
	    }
	  break;
	case 3:
	  switch (Field_op2_Slot_inst_get (insn))
	    {
	    case 0:
	      switch (Field_sr_Slot_inst_get (insn))
		{
		case 0:
		  return OPCODE_RSR_LBEG;
		case 1:
		  return OPCODE_RSR_LEND;
		case 2:
		  return OPCODE_RSR_LCOUNT;
		case 3:
		  return OPCODE_RSR_SAR;
		case 4:
		  return OPCODE_RSR_BR;
		case 5:
		  return OPCODE_RSR_LITBASE;
		case 12:
		  return OPCODE_RSR_SCOMPARE1;
		case 72:
		  return OPCODE_RSR_WINDOWBASE;
		case 73:
		  return OPCODE_RSR_WINDOWSTART;
		case 83:
		  return OPCODE_RSR_PTEVADDR;
		case 90:
		  return OPCODE_RSR_RASID;
		case 91:
		  return OPCODE_RSR_ITLBCFG;
		case 92:
		  return OPCODE_RSR_DTLBCFG;
		case 99:
		  return OPCODE_RSR_ATOMCTL;
		case 104:
		  return OPCODE_RSR_DDR;
		case 176:
		  return OPCODE_RSR_176;
		case 177:
		  return OPCODE_RSR_EPC1;
		case 178:
		  return OPCODE_RSR_EPC2;
		case 192:
		  return OPCODE_RSR_DEPC;
		case 194:
		  return OPCODE_RSR_EPS2;
		case 208:
		  return OPCODE_RSR_208;
		case 209:
		  return OPCODE_RSR_EXCSAVE1;
		case 210:
		  return OPCODE_RSR_EXCSAVE2;
		case 224:
		  return OPCODE_RSR_CPENABLE;
		case 226:
		  return OPCODE_RSR_INTERRUPT;
		case 228:
		  return OPCODE_RSR_INTENABLE;
		case 230:
		  return OPCODE_RSR_PS;
		case 231:
		  return OPCODE_RSR_VECBASE;
		case 232:
		  return OPCODE_RSR_EXCCAUSE;
		case 233:
		  return OPCODE_RSR_DEBUGCAUSE;
		case 234:
		  return OPCODE_RSR_CCOUNT;
		case 235:
		  return OPCODE_RSR_PRID;
		case 236:
		  return OPCODE_RSR_ICOUNT;
		case 237:
		  return OPCODE_RSR_ICOUNTLEVEL;
		case 238:
		  return OPCODE_RSR_EXCVADDR;
		case 240:
		  return OPCODE_RSR_CCOMPARE0;
		case 241:
		  return OPCODE_RSR_CCOMPARE1;
		case 244:
		  return OPCODE_RSR_MISC0;
		case 245:
		  return OPCODE_RSR_MISC1;
		}
	      break;
	    case 1:
	      switch (Field_sr_Slot_inst_get (insn))
		{
		case 0:
		  return OPCODE_WSR_LBEG;
		case 1:
		  return OPCODE_WSR_LEND;
		case 2:
		  return OPCODE_WSR_LCOUNT;
		case 3:
		  return OPCODE_WSR_SAR;
		case 4:
		  return OPCODE_WSR_BR;
		case 5:
		  return OPCODE_WSR_LITBASE;
		case 12:
		  return OPCODE_WSR_SCOMPARE1;
		case 72:
		  return OPCODE_WSR_WINDOWBASE;
		case 73:
		  return OPCODE_WSR_WINDOWSTART;
		case 83:
		  return OPCODE_WSR_PTEVADDR;
		case 90:
		  return OPCODE_WSR_RASID;
		case 91:
		  return OPCODE_WSR_ITLBCFG;
		case 92:
		  return OPCODE_WSR_DTLBCFG;
		case 99:
		  return OPCODE_WSR_ATOMCTL;
		case 104:
		  return OPCODE_WSR_DDR;
		case 176:
		  return OPCODE_WSR_176;
		case 177:
		  return OPCODE_WSR_EPC1;
		case 178:
		  return OPCODE_WSR_EPC2;
		case 192:
		  return OPCODE_WSR_DEPC;
		case 194:
		  return OPCODE_WSR_EPS2;
		case 209:
		  return OPCODE_WSR_EXCSAVE1;
		case 210:
		  return OPCODE_WSR_EXCSAVE2;
		case 224:
		  return OPCODE_WSR_CPENABLE;
		case 226:
		  return OPCODE_WSR_INTSET;
		case 227:
		  return OPCODE_WSR_INTCLEAR;
		case 228:
		  return OPCODE_WSR_INTENABLE;
		case 230:
		  return OPCODE_WSR_PS;
		case 231:
		  return OPCODE_WSR_VECBASE;
		case 232:
		  return OPCODE_WSR_EXCCAUSE;
		case 233:
		  return OPCODE_WSR_DEBUGCAUSE;
		case 234:
		  return OPCODE_WSR_CCOUNT;
		case 236:
		  return OPCODE_WSR_ICOUNT;
		case 237:
		  return OPCODE_WSR_ICOUNTLEVEL;
		case 238:
		  return OPCODE_WSR_EXCVADDR;
		case 240:
		  return OPCODE_WSR_CCOMPARE0;
		case 241:
		  return OPCODE_WSR_CCOMPARE1;
		case 244:
		  return OPCODE_WSR_MISC0;
		case 245:
		  return OPCODE_WSR_MISC1;
		}
	      break;
	    case 2:
	      return OPCODE_SEXT;
	    case 3:
	      return OPCODE_CLAMPS;
	    case 4:
	      return OPCODE_MIN;
	    case 5:
	      return OPCODE_MAX;
	    case 6:
	      return OPCODE_MINU;
	    case 7:
	      return OPCODE_MAXU;
	    case 8:
	      return OPCODE_MOVEQZ;
	    case 9:
	      return OPCODE_MOVNEZ;
	    case 10:
	      return OPCODE_MOVLTZ;
	    case 11:
	      return OPCODE_MOVGEZ;
	    case 12:
	      return OPCODE_MOVF;
	    case 13:
	      return OPCODE_MOVT;
	    case 14:
	      switch (Field_st_Slot_inst_get (insn))
		{
		case 231:
		  return OPCODE_RUR_THREADPTR;
		case 240:
		  return OPCODE_RUR_AE_OVF_SAR;
		case 241:
		  return OPCODE_RUR_AE_BITHEAD;
		case 242:
		  return OPCODE_RUR_AE_TS_FTS_BU_BP;
		case 243:
		  return OPCODE_RUR_AE_SD_NO;
		}
	      break;
	    case 15:
	      switch (Field_sr_Slot_inst_get (insn))
		{
		case 231:
		  return OPCODE_WUR_THREADPTR;
		case 240:
		  return OPCODE_WUR_AE_OVF_SAR;
		case 241:
		  return OPCODE_WUR_AE_BITHEAD;
		case 242:
		  return OPCODE_WUR_AE_TS_FTS_BU_BP;
		case 243:
		  return OPCODE_WUR_AE_SD_NO;
		}
	      break;
	    }
	  break;
	case 4:
	case 5:
	  return OPCODE_EXTUI;
	case 9:
	  switch (Field_op2_Slot_inst_get (insn))
	    {
	    case 0:
	      return OPCODE_L32E;
	    case 4:
	      return OPCODE_S32E;
	    }
	  break;
	}
      break;
    case 1:
      return OPCODE_L32R;
    case 2:
      switch (Field_r_Slot_inst_get (insn))
	{
	case 0:
	  return OPCODE_L8UI;
	case 1:
	  return OPCODE_L16UI;
	case 2:
	  return OPCODE_L32I;
	case 4:
	  return OPCODE_S8I;
	case 5:
	  return OPCODE_S16I;
	case 6:
	  return OPCODE_S32I;
	case 7:
	  switch (Field_t_Slot_inst_get (insn))
	    {
	    case 0:
	      return OPCODE_DPFR;
	    case 1:
	      return OPCODE_DPFW;
	    case 2:
	      return OPCODE_DPFRO;
	    case 3:
	      return OPCODE_DPFWO;
	    case 4:
	      return OPCODE_DHWB;
	    case 5:
	      return OPCODE_DHWBI;
	    case 6:
	      return OPCODE_DHI;
	    case 7:
	      return OPCODE_DII;
	    case 8:
	      switch (Field_op1_Slot_inst_get (insn))
		{
		case 4:
		  return OPCODE_DIWB;
		case 5:
		  return OPCODE_DIWBI;
		}
	      break;
	    case 12:
	      return OPCODE_IPF;
	    case 14:
	      return OPCODE_IHI;
	    case 15:
	      return OPCODE_III;
	    }
	  break;
	case 9:
	  return OPCODE_L16SI;
	case 10:
	  return OPCODE_MOVI;
	case 11:
	  return OPCODE_L32AI;
	case 12:
	  return OPCODE_ADDI;
	case 13:
	  return OPCODE_ADDMI;
	case 14:
	  return OPCODE_S32C1I;
	case 15:
	  return OPCODE_S32RI;
	}
      break;
    case 4:
      switch (Field_ae_r10_Slot_inst_get (insn))
	{
	case 0:
	  if (Field_op1_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_LQ56_I;
	  if (Field_op1_Slot_inst_get (insn) == 2 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_LQ56_X;
	  break;
	case 1:
	  if (Field_op1_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_LQ32F_I;
	  if (Field_op1_Slot_inst_get (insn) == 2 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_LQ32F_X;
	  break;
	case 2:
	  if (Field_op1_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_LQ56_IU;
	  if (Field_op1_Slot_inst_get (insn) == 2 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_LQ56_XU;
	  if (Field_op1_Slot_inst_get (insn) == 7 &&
	      Field_t_Slot_inst_get (insn) == 3 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_CVTQ48A32S;
	  break;
	case 3:
	  if (Field_op1_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_LQ32F_IU;
	  if (Field_op1_Slot_inst_get (insn) == 2 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_LQ32F_XU;
	  break;
	}
      switch (Field_ae_r3_Slot_inst_get (insn))
	{
	case 0:
	  if (Field_op1_Slot_inst_get (insn) == 5 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP16F_I;
	  if (Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP16F_IU;
	  if (Field_op1_Slot_inst_get (insn) == 12 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP16F_X;
	  if (Field_op1_Slot_inst_get (insn) == 15 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP16F_XU;
	  if (Field_op1_Slot_inst_get (insn) == 6 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP24F_I;
	  if (Field_op1_Slot_inst_get (insn) == 10 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP24F_IU;
	  if (Field_op1_Slot_inst_get (insn) == 13 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP24F_X;
	  if (Field_op1_Slot_inst_get (insn) == 0 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_LP24F_XU;
	  if (Field_op1_Slot_inst_get (insn) == 7 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP24X2F_I;
	  if (Field_op1_Slot_inst_get (insn) == 11 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP24X2F_IU;
	  if (Field_op1_Slot_inst_get (insn) == 14 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP24X2F_X;
	  if (Field_op1_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_LP24X2F_XU;
	  if (Field_op1_Slot_inst_get (insn) == 2 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP16X2F_I;
	  if (Field_op1_Slot_inst_get (insn) == 5 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP16X2F_IU;
	  if (Field_op1_Slot_inst_get (insn) == 8 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP16X2F_X;
	  if (Field_op1_Slot_inst_get (insn) == 11 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP16X2F_XU;
	  if (Field_op1_Slot_inst_get (insn) == 3 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24X2F_I;
	  if (Field_op1_Slot_inst_get (insn) == 6 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24X2F_IU;
	  if (Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24X2F_X;
	  if (Field_op1_Slot_inst_get (insn) == 12 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24X2F_XU;
	  if (Field_op1_Slot_inst_get (insn) == 4 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24S_L_I;
	  if (Field_op1_Slot_inst_get (insn) == 7 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24S_L_IU;
	  if (Field_op1_Slot_inst_get (insn) == 10 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24S_L_X;
	  if (Field_op1_Slot_inst_get (insn) == 13 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24S_L_XU;
	  if (Field_ae_s3_Slot_inst_get (insn) == 0 &&
	      Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_MOVP48;
	  if (Field_op1_Slot_inst_get (insn) == 0 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_MOVPA24X2;
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 11 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_CVTA32P24_L;
	  if (Field_op1_Slot_inst_get (insn) == 14 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_CVTP24A16X2_LL;
	  if (Field_op1_Slot_inst_get (insn) == 15 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_CVTP24A16X2_HL;
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 7 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_MOVAP24S_L;
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 8 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_TRUNCA16P24S_L;
	  break;
	case 1:
	  if (Field_op1_Slot_inst_get (insn) == 5 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP24_I;
	  if (Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP24_IU;
	  if (Field_op1_Slot_inst_get (insn) == 12 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP24_X;
	  if (Field_op1_Slot_inst_get (insn) == 15 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP24_XU;
	  if (Field_op1_Slot_inst_get (insn) == 6 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP16X2F_I;
	  if (Field_op1_Slot_inst_get (insn) == 10 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP16X2F_IU;
	  if (Field_op1_Slot_inst_get (insn) == 13 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP16X2F_X;
	  if (Field_op1_Slot_inst_get (insn) == 0 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_LP16X2F_XU;
	  if (Field_op1_Slot_inst_get (insn) == 7 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP24X2_I;
	  if (Field_op1_Slot_inst_get (insn) == 11 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP24X2_IU;
	  if (Field_op1_Slot_inst_get (insn) == 14 &&
	      Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LP24X2_X;
	  if (Field_op1_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_LP24X2_XU;
	  if (Field_op1_Slot_inst_get (insn) == 2 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24X2S_I;
	  if (Field_op1_Slot_inst_get (insn) == 5 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24X2S_IU;
	  if (Field_op1_Slot_inst_get (insn) == 8 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24X2S_X;
	  if (Field_op1_Slot_inst_get (insn) == 11 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24X2S_XU;
	  if (Field_op1_Slot_inst_get (insn) == 3 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP16F_L_I;
	  if (Field_op1_Slot_inst_get (insn) == 6 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP16F_L_IU;
	  if (Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP16F_L_X;
	  if (Field_op1_Slot_inst_get (insn) == 12 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP16F_L_XU;
	  if (Field_op1_Slot_inst_get (insn) == 4 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24F_L_I;
	  if (Field_op1_Slot_inst_get (insn) == 7 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24F_L_IU;
	  if (Field_op1_Slot_inst_get (insn) == 10 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24F_L_X;
	  if (Field_op1_Slot_inst_get (insn) == 13 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_SP24F_L_XU;
	  if (Field_op1_Slot_inst_get (insn) == 0 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_TRUNCP24A32X2;
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 11 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_CVTA32P24_H;
	  if (Field_op1_Slot_inst_get (insn) == 14 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_CVTP24A16X2_LH;
	  if (Field_op1_Slot_inst_get (insn) == 15 &&
	      Field_op2_Slot_inst_get (insn) == 11)
	    return OPCODE_AE_CVTP24A16X2_HH;
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 7 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_MOVAP24S_H;
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 8 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_TRUNCA16P24S_H;
	  break;
	}
      switch (Field_ae_r32_Slot_inst_get (insn))
	{
	case 0:
	  if (Field_op1_Slot_inst_get (insn) == 3 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SQ56S_I;
	  if (Field_op1_Slot_inst_get (insn) == 4 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SQ56S_X;
	  if (Field_op1_Slot_inst_get (insn) == 7 &&
	      Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_TRUNCA32Q48;
	  break;
	case 1:
	  if (Field_op1_Slot_inst_get (insn) == 3 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SQ32F_I;
	  if (Field_op1_Slot_inst_get (insn) == 4 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SQ32F_X;
	  if (Field_op1_Slot_inst_get (insn) == 7 &&
	      Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_NSAQ56S;
	  break;
	case 2:
	  if (Field_op1_Slot_inst_get (insn) == 3 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SQ56S_IU;
	  if (Field_op1_Slot_inst_get (insn) == 4 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SQ56S_XU;
	  break;
	case 3:
	  if (Field_op1_Slot_inst_get (insn) == 3 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SQ32F_IU;
	  if (Field_op1_Slot_inst_get (insn) == 4 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SQ32F_XU;
	  break;
	}
      switch (Field_ae_s_non_samt_Slot_inst_get (insn))
	{
	case 0:
	  if (Field_op1_Slot_inst_get (insn) == 5 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SLLIQ56;
	  break;
	case 1:
	  if (Field_op1_Slot_inst_get (insn) == 5 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SRLIQ56;
	  break;
	case 2:
	  if (Field_op1_Slot_inst_get (insn) == 5 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SRAIQ56;
	  break;
	case 3:
	  if (Field_op1_Slot_inst_get (insn) == 5 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SLLISQ56S;
	  break;
	}
      switch (Field_op1_Slot_inst_get (insn))
	{
	case 0:
	  if (Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_SHA32;
	  if (Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_VLDL32T;
	  break;
	case 1:
	  if (Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_SLLAQ56;
	  if (Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_VLDL16T;
	  break;
	case 2:
	  if (Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_SRLAQ56;
	  if (Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_LBK;
	  break;
	case 3:
	  if (Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_SRAAQ56;
	  if (Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_VLEL32T;
	  break;
	case 4:
	  if (Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_SLLASQ56S;
	  if (Field_op2_Slot_inst_get (insn) == 10)
	    return OPCODE_AE_VLEL16T;
	  break;
	case 5:
	  if (Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_MOVTQ56;
	  break;
	case 6:
	  if (Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_MOVFQ56;
	  break;
	}
      switch (Field_r_Slot_inst_get (insn))
	{
	case 0:
	  if (Field_s_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 10 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_WUR_AE_OVERFLOW;
	  if (Field_op2_Slot_inst_get (insn) == 15)
	    return OPCODE_AE_SBI;
	  break;
	case 1:
	  if (Field_s_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 10 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_WUR_AE_SAR;
	  if (Field_op1_Slot_inst_get (insn) == 0 &&
	      Field_op2_Slot_inst_get (insn) == 15)
	    return OPCODE_AE_DB;
	  if (Field_op1_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 15)
	    return OPCODE_AE_SB;
	  break;
	case 2:
	  if (Field_s_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 10 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_WUR_AE_BITPTR;
	  break;
	case 3:
	  if (Field_s_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 10 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_WUR_AE_BITSUSED;
	  break;
	case 4:
	  if (Field_s_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 10 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_WUR_AE_TABLESIZE;
	  break;
	case 5:
	  if (Field_s_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 10 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_WUR_AE_FIRST_TS;
	  break;
	case 6:
	  if (Field_s_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 10 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_WUR_AE_NEXTOFFSET;
	  break;
	case 7:
	  if (Field_s_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 10 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_WUR_AE_SEARCHDONE;
	  break;
	case 8:
	  if (Field_s_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 10 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_VLDSHT;
	  break;
	case 12:
	  if (Field_op1_Slot_inst_get (insn) == 7 &&
	      Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_VLES16C;
	  break;
	case 13:
	  if (Field_op1_Slot_inst_get (insn) == 7 &&
	      Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_SBF;
	  break;
	case 14:
	  if (Field_op1_Slot_inst_get (insn) == 7 &&
	      Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_VLDL16C;
	  break;
	}
      switch (Field_s_Slot_inst_get (insn))
	{
	case 0:
	  if (Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SLLSQ56;
	  if (Field_op1_Slot_inst_get (insn) == 6 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_LB;
	  break;
	case 1:
	  if (Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SRLSQ56;
	  break;
	case 2:
	  if (Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SRASQ56;
	  break;
	case 3:
	  if (Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_SLLSSQ56S;
	  break;
	case 4:
	  if (Field_t_Slot_inst_get (insn) == 1 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_AE_MOVQ56;
	  break;
	case 8:
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_RUR_AE_OVERFLOW;
	  break;
	case 9:
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_RUR_AE_SAR;
	  break;
	case 10:
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_RUR_AE_BITPTR;
	  break;
	case 11:
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_RUR_AE_BITSUSED;
	  break;
	case 12:
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_RUR_AE_TABLESIZE;
	  break;
	case 13:
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_RUR_AE_FIRST_TS;
	  break;
	case 14:
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_RUR_AE_NEXTOFFSET;
	  break;
	case 15:
	  if (Field_t_Slot_inst_get (insn) == 0 &&
	      Field_op1_Slot_inst_get (insn) == 9 &&
	      Field_op2_Slot_inst_get (insn) == 12)
	    return OPCODE_RUR_AE_SEARCHDONE;
	  break;
	}
      switch (Field_t_Slot_inst_get (insn))
	{
	case 0:
	  if (Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_LBKI;
	  if (Field_r_Slot_inst_get (insn) == 2 &&
	      Field_op2_Slot_inst_get (insn) == 15)
	    return OPCODE_AE_DBI;
	  break;
	case 2:
	  if (Field_s_Slot_inst_get (insn) == 0 &&
	      Field_op2_Slot_inst_get (insn) == 14)
	    return OPCODE_AE_LBI;
	  break;
	}
      break;
    case 5:
      switch (Field_n_Slot_inst_get (insn))
	{
	case 0:
	  return OPCODE_CALL0;
	case 1:
	  return OPCODE_CALL4;
	case 2:
	  return OPCODE_CALL8;
	case 3:
	  return OPCODE_CALL12;
	}
      break;
    case 6:
      switch (Field_n_Slot_inst_get (insn))
	{
	case 0:
	  return OPCODE_J;
	case 1:
	  switch (Field_m_Slot_inst_get (insn))
	    {
	    case 0:
	      return OPCODE_BEQZ;
	    case 1:
	      return OPCODE_BNEZ;
	    case 2:
	      return OPCODE_BLTZ;
	    case 3:
	      return OPCODE_BGEZ;
	    }
	  break;
	case 2:
	  switch (Field_m_Slot_inst_get (insn))
	    {
	    case 0:
	      return OPCODE_BEQI;
	    case 1:
	      return OPCODE_BNEI;
	    case 2:
	      return OPCODE_BLTI;
	    case 3:
	      return OPCODE_BGEI;
	    }
	  break;
	case 3:
	  switch (Field_m_Slot_inst_get (insn))
	    {
	    case 0:
	      return OPCODE_ENTRY;
	    case 1:
	      switch (Field_r_Slot_inst_get (insn))
		{
		case 0:
		  return OPCODE_BF;
		case 1:
		  return OPCODE_BT;
		case 8:
		  return OPCODE_LOOP;
		case 9:
		  return OPCODE_LOOPNEZ;
		case 10:
		  return OPCODE_LOOPGTZ;
		}
	      break;
	    case 2:
	      return OPCODE_BLTUI;
	    case 3:
	      return OPCODE_BGEUI;
	    }
	  break;
	}
      break;
    case 7:
      switch (Field_r_Slot_inst_get (insn))
	{
	case 0:
	  return OPCODE_BNONE;
	case 1:
	  return OPCODE_BEQ;
	case 2:
	  return OPCODE_BLT;
	case 3:
	  return OPCODE_BLTU;
	case 4:
	  return OPCODE_BALL;
	case 5:
	  return OPCODE_BBC;
	case 6:
	case 7:
	  return OPCODE_BBCI;
	case 8:
	  return OPCODE_BANY;
	case 9:
	  return OPCODE_BNE;
	case 10:
	  return OPCODE_BGE;
	case 11:
	  return OPCODE_BGEU;
	case 12:
	  return OPCODE_BNALL;
	case 13:
	  return OPCODE_BBS;
	case 14:
	case 15:
	  return OPCODE_BBSI;
	}
      break;
    }
  return 0;
}

static int
Slot_inst16b_decode (const xtensa_insnbuf insn)
{
  switch (Field_op0_Slot_inst16b_get (insn))
    {
    case 12:
      switch (Field_i_Slot_inst16b_get (insn))
	{
	case 0:
	  return OPCODE_MOVI_N;
	case 1:
	  switch (Field_z_Slot_inst16b_get (insn))
	    {
	    case 0:
	      return OPCODE_BEQZ_N;
	    case 1:
	      return OPCODE_BNEZ_N;
	    }
	  break;
	}
      break;
    case 13:
      switch (Field_r_Slot_inst16b_get (insn))
	{
	case 0:
	  return OPCODE_MOV_N;
	case 15:
	  switch (Field_t_Slot_inst16b_get (insn))
	    {
	    case 0:
	      return OPCODE_RET_N;
	    case 1:
	      return OPCODE_RETW_N;
	    case 2:
	      return OPCODE_BREAK_N;
	    case 3:
	      if (Field_s_Slot_inst16b_get (insn) == 0)
		return OPCODE_NOP_N;
	      break;
	    case 6:
	      if (Field_s_Slot_inst16b_get (insn) == 0)
		return OPCODE_ILL_N;
	      break;
	    }
	  break;
	}
      break;
    }
  return 0;
}

static int
Slot_inst16a_decode (const xtensa_insnbuf insn)
{
  switch (Field_op0_Slot_inst16a_get (insn))
    {
    case 8:
      return OPCODE_L32I_N;
    case 9:
      return OPCODE_S32I_N;
    case 10:
      return OPCODE_ADD_N;
    case 11:
      return OPCODE_ADDI_N;
    }
  return 0;
}

static int
Slot_ae_slot0_decode (const xtensa_insnbuf insn)
{
  if (Field_ftsf212ae_slot0_Slot_ae_slot0_get (insn) == 0 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_J;
  if (Field_ftsf213ae_slot0_Slot_ae_slot0_get (insn) == 2 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_EXTUI;
  switch (Field_ftsf214ae_slot0_Slot_ae_slot0_get (insn))
    {
    case 6:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_BGEZ;
      break;
    case 7:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_BLTZ;
      break;
    case 8:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_BEQZ;
      break;
    case 9:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_BNEZ;
      break;
    case 10:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_MOVI;
      break;
    }
  switch (Field_ftsf215ae_slot0_Slot_ae_slot0_get (insn))
    {
    case 88:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_SRAI;
      break;
    case 96:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_SLLI;
      break;
    case 123:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
	  Field_ftsf364ae_slot0_Slot_ae_slot0_get (insn) == 0)
	return OPCODE_AE_MOVTQ56;
      break;
    }
  if (Field_ftsf216ae_slot0_Slot_ae_slot0_get (insn) == 418 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_CVTP24A16X2_HH;
  if (Field_ftsf217_Slot_ae_slot0_get (insn) == 1 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 4 &&
      Field_ae_r20_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_L32I;
  if (Field_ftsf218ae_slot0_Slot_ae_slot0_get (insn) == 419 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP16F_I;
  if (Field_ftsf219ae_slot0_Slot_ae_slot0_get (insn) == 420 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_CVTP24A16X2_HL;
  if (Field_ftsf220ae_slot0_Slot_ae_slot0_get (insn) == 421 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP16F_IU;
  if (Field_ftsf221ae_slot0_Slot_ae_slot0_get (insn) == 422 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP16F_X;
  if (Field_ftsf222ae_slot0_Slot_ae_slot0_get (insn) == 423 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP16F_XU;
  if (Field_ftsf223ae_slot0_Slot_ae_slot0_get (insn) == 424 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_CVTP24A16X2_LH;
  if (Field_ftsf224ae_slot0_Slot_ae_slot0_get (insn) == 425 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP16X2F_I;
  if (Field_ftsf225ae_slot0_Slot_ae_slot0_get (insn) == 426 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP16X2F_IU;
  if (Field_ftsf226ae_slot0_Slot_ae_slot0_get (insn) == 427 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP16X2F_XU;
  if (Field_ftsf227ae_slot0_Slot_ae_slot0_get (insn) == 428 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP16X2F_X;
  if (Field_ftsf228ae_slot0_Slot_ae_slot0_get (insn) == 429 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24_I;
  if (Field_ftsf229ae_slot0_Slot_ae_slot0_get (insn) == 430 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24_IU;
  if (Field_ftsf230ae_slot0_Slot_ae_slot0_get (insn) == 431 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24_X;
  if (Field_ftsf231ae_slot0_Slot_ae_slot0_get (insn) == 432 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_CVTP24A16X2_LL;
  if (Field_ftsf232ae_slot0_Slot_ae_slot0_get (insn) == 433 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24_XU;
  if (Field_ftsf233ae_slot0_Slot_ae_slot0_get (insn) == 434 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24F_I;
  if (Field_ftsf234ae_slot0_Slot_ae_slot0_get (insn) == 435 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24F_XU;
  if (Field_ftsf235ae_slot0_Slot_ae_slot0_get (insn) == 436 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24F_IU;
  if (Field_ftsf236ae_slot0_Slot_ae_slot0_get (insn) == 437 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24X2_I;
  if (Field_ftsf237ae_slot0_Slot_ae_slot0_get (insn) == 438 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24X2_IU;
  if (Field_ftsf238ae_slot0_Slot_ae_slot0_get (insn) == 439 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24X2_X;
  if (Field_ftsf239ae_slot0_Slot_ae_slot0_get (insn) == 440 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24F_X;
  if (Field_ftsf240ae_slot0_Slot_ae_slot0_get (insn) == 441 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24X2_XU;
  if (Field_ftsf241ae_slot0_Slot_ae_slot0_get (insn) == 442 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24X2F_I;
  if (Field_ftsf242ae_slot0_Slot_ae_slot0_get (insn) == 443 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24X2F_X;
  if (Field_ftsf243ae_slot0_Slot_ae_slot0_get (insn) == 444 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24X2F_IU;
  if (Field_ftsf244ae_slot0_Slot_ae_slot0_get (insn) == 445 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LP24X2F_XU;
  if (Field_ftsf245ae_slot0_Slot_ae_slot0_get (insn) == 446 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_MOVPA24X2;
  if (Field_ftsf246ae_slot0_Slot_ae_slot0_get (insn) == 447 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP16F_L_I;
  if (Field_ftsf247ae_slot0_Slot_ae_slot0_get (insn) == 450 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP16F_L_IU;
  if (Field_ftsf248ae_slot0_Slot_ae_slot0_get (insn) == 451 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP16X2F_X;
  if (Field_ftsf249ae_slot0_Slot_ae_slot0_get (insn) == 452 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP16F_L_X;
  if (Field_ftsf250ae_slot0_Slot_ae_slot0_get (insn) == 453 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP16X2F_XU;
  if (Field_ftsf251ae_slot0_Slot_ae_slot0_get (insn) == 454 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24F_L_I;
  if (Field_ftsf252ae_slot0_Slot_ae_slot0_get (insn) == 455 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24F_L_IU;
  if (Field_ftsf253ae_slot0_Slot_ae_slot0_get (insn) == 456 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP16F_L_XU;
  if (Field_ftsf254ae_slot0_Slot_ae_slot0_get (insn) == 457 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24F_L_X;
  if (Field_ftsf255ae_slot0_Slot_ae_slot0_get (insn) == 458 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24F_L_XU;
  if (Field_ftsf256ae_slot0_Slot_ae_slot0_get (insn) == 459 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24S_L_IU;
  if (Field_ftsf257ae_slot0_Slot_ae_slot0_get (insn) == 460 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24S_L_I;
  if (Field_ftsf258ae_slot0_Slot_ae_slot0_get (insn) == 461 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24S_L_X;
  if (Field_ftsf259ae_slot0_Slot_ae_slot0_get (insn) == 462 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24S_L_XU;
  if (Field_ftsf260ae_slot0_Slot_ae_slot0_get (insn) == 463 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24X2F_I;
  if (Field_ftsf261ae_slot0_Slot_ae_slot0_get (insn) == 464 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP16X2F_I;
  if (Field_ftsf262ae_slot0_Slot_ae_slot0_get (insn) == 465 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24X2F_IU;
  if (Field_ftsf263ae_slot0_Slot_ae_slot0_get (insn) == 466 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24X2F_X;
  if (Field_ftsf264ae_slot0_Slot_ae_slot0_get (insn) == 467 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24X2S_IU;
  if (Field_ftsf265ae_slot0_Slot_ae_slot0_get (insn) == 468 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24X2F_XU;
  if (Field_ftsf266ae_slot0_Slot_ae_slot0_get (insn) == 469 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24X2S_X;
  if (Field_ftsf267ae_slot0_Slot_ae_slot0_get (insn) == 470 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24X2S_XU;
  if (Field_ftsf268ae_slot0_Slot_ae_slot0_get (insn) == 471 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_TRUNCP24A32X2;
  if (Field_ftsf269ae_slot0_Slot_ae_slot0_get (insn) == 472 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP24X2S_I;
  if (Field_ftsf270ae_slot0_Slot_ae_slot0_get (insn) == 946 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SQ32F_I;
  if (Field_ftsf271ae_slot0_Slot_ae_slot0_get (insn) == 947 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SQ32F_IU;
  if (Field_ftsf272ae_slot0_Slot_ae_slot0_get (insn) == 948 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LQ32F_I;
  if (Field_ftsf273ae_slot0_Slot_ae_slot0_get (insn) == 949 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LQ32F_X;
  if (Field_ftsf274ae_slot0_Slot_ae_slot0_get (insn) == 950 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LQ32F_XU;
  if (Field_ftsf275ae_slot0_Slot_ae_slot0_get (insn) == 951 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LQ56_I;
  if (Field_ftsf276ae_slot0_Slot_ae_slot0_get (insn) == 952 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LQ32F_IU;
  if (Field_ftsf277ae_slot0_Slot_ae_slot0_get (insn) == 953 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LQ56_IU;
  if (Field_ftsf278ae_slot0_Slot_ae_slot0_get (insn) == 954 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_LQ56_X;
  if (Field_ftsf279ae_slot0_Slot_ae_slot0_get (insn) == 15280 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_CVTQ48A32S;
  if (Field_ftsf281ae_slot0_Slot_ae_slot0_get (insn) == 60977 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_JX;
  if (Field_ftsf282ae_slot0_Slot_ae_slot0_get (insn) == 61041 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_SSR;
  if (Field_ftsf283ae_slot0_Slot_ae_slot0_get (insn) == 30577 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf352ae_slot0_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_NOP;
  if (Field_ftsf284ae_slot0_Slot_ae_slot0_get (insn) == 7641 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf354ae_slot0_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_SSA8B;
  if (Field_ftsf286ae_slot0_Slot_ae_slot0_get (insn) == 3821 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf356ae_slot0_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_SSA8L;
  if (Field_ftsf288ae_slot0_Slot_ae_slot0_get (insn) == 1911 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf359ae_slot0_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_SSL;
  if (Field_ftsf290ae_slot0_Slot_ae_slot0_get (insn) == 478 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_s8_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_AE_LQ56_XU;
  if (Field_ftsf292ae_slot0_Slot_ae_slot0_get (insn) == 1913 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_s_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_ALL8;
  switch (Field_ftsf293_Slot_ae_slot0_get (insn))
    {
    case 0:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_BBCI;
      break;
    case 1:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_BBSI;
      break;
    }
  if (Field_ftsf294ae_slot0_Slot_ae_slot0_get (insn) == 1915 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_s_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_ANY8;
  if (Field_ftsf295ae_slot0_Slot_ae_slot0_get (insn) == 959 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf358ae_slot0_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_SSAI;
  if (Field_ftsf296ae_slot0_Slot_ae_slot0_get (insn) == 480 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SP16X2F_IU;
  if (Field_ftsf297ae_slot0_Slot_ae_slot0_get (insn) == 962 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SQ56S_I;
  if (Field_ftsf298ae_slot0_Slot_ae_slot0_get (insn) == 963 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SQ56S_IU;
  switch (Field_ftsf299ae_slot0_Slot_ae_slot0_get (insn))
    {
    case 964:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_AE_SLLIQ56;
      break;
    case 965:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_AE_SRAIQ56;
      break;
    case 966:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_AE_SRLIQ56;
      break;
    case 968:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_AE_SLLISQ56S;
      break;
    }
  switch (Field_ftsf300ae_slot0_Slot_ae_slot0_get (insn))
    {
    case 3868:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_ABS;
      break;
    case 3869:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_NEG;
      break;
    case 3870:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_SRA;
      break;
    case 3871:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_SRL;
      break;
    }
  switch (Field_ftsf301ae_slot0_Slot_ae_slot0_get (insn))
    {
    case 7752:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
	  Field_ftsf321_Slot_ae_slot0_get (insn) == 0)
	return OPCODE_AE_MOVP48;
      break;
    case 7753:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
	  Field_ftsf353_Slot_ae_slot0_get (insn) == 0)
	return OPCODE_ANY4;
      break;
    }
  if (Field_ftsf302ae_slot0_Slot_ae_slot0_get (insn) == 31016 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf321_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_AE_MOVQ56;
  if (Field_ftsf303ae_slot0_Slot_ae_slot0_get (insn) == 31017 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf321_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_AE_SLLSSQ56S;
  if (Field_ftsf304ae_slot0_Slot_ae_slot0_get (insn) == 15509 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf369ae_slot0_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_AE_SRASQ56;
  if (Field_ftsf306ae_slot0_Slot_ae_slot0_get (insn) == 7755 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf368ae_slot0_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_AE_SRLSQ56;
  if (Field_ftsf308ae_slot0_Slot_ae_slot0_get (insn) == 1939 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf366ae_slot0_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_AE_SLLSQ56;
  if (Field_ftsf309ae_slot0_Slot_ae_slot0_get (insn) == 485 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf360ae_slot0_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_ALL4;
  if (Field_ftsf310ae_slot0_Slot_ae_slot0_get (insn) == 972 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SQ56S_X;
  if (Field_ftsf311ae_slot0_Slot_ae_slot0_get (insn) == 973 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SQ56S_XU;
  if (Field_ftsf312ae_slot0_Slot_ae_slot0_get (insn) == 7792 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_CVTA32P24_H;
  if (Field_ftsf313ae_slot0_Slot_ae_slot0_get (insn) == 7793 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_CVTA32P24_L;
  if (Field_ftsf314ae_slot0_Slot_ae_slot0_get (insn) == 7794 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_MOVAP24S_H;
  if (Field_ftsf315ae_slot0_Slot_ae_slot0_get (insn) == 7795 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_TRUNCA16P24S_L;
  if (Field_ftsf316ae_slot0_Slot_ae_slot0_get (insn) == 7796 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_MOVAP24S_L;
  if (Field_ftsf317ae_slot0_Slot_ae_slot0_get (insn) == 7797 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf353_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_AE_NSAQ56S;
  if (Field_ftsf318ae_slot0_Slot_ae_slot0_get (insn) == 3899 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf365ae_slot0_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_AE_TRUNCA32Q48;
  if (Field_ftsf319_Slot_ae_slot0_get (insn) == 3 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 3 &&
      Field_ftsf361ae_slot0_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_BT;
  if (Field_ftsf320ae_slot0_Slot_ae_slot0_get (insn) == 975 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ae_s20_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_AE_TRUNCA16P24S_H;
  if (Field_ftsf321_Slot_ae_slot0_get (insn) == 1 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 3 &&
      Field_ae_s20_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_BLTUI;
  if (Field_ftsf322ae_slot0_Slot_ae_slot0_get (insn) == 3920 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_MOVFQ56;
  if (Field_ftsf323ae_slot0_Slot_ae_slot0_get (insn) == 3921 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SLLAQ56;
  if (Field_ftsf324ae_slot0_Slot_ae_slot0_get (insn) == 3922 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_AE_SLLASQ56S;
  if (Field_ftsf325ae_slot0_Slot_ae_slot0_get (insn) == 3923 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
    return OPCODE_SLL;
  if (Field_ftsf326ae_slot0_Slot_ae_slot0_get (insn) == 981 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf357_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_AE_SRAAQ56;
  if (Field_ftsf328ae_slot0_Slot_ae_slot0_get (insn) == 491 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ae_s20_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_AE_SRLAQ56;
  if (Field_ftsf329ae_slot0_Slot_ae_slot0_get (insn) == 31 &&
      Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
      Field_ftsf362ae_slot0_Slot_ae_slot0_get (insn) == 0)
    return OPCODE_AE_SQ32F_XU;
  switch (Field_imm8_Slot_ae_slot0_get (insn))
    {
    case 178:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_ADD;
      break;
    case 179:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_ADDX8;
      break;
    case 180:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_ADDX2;
      break;
    case 181:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_AND;
      break;
    case 182:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_ANDB;
      break;
    case 183:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_ANDBC;
      break;
    case 184:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_ADDX4;
      break;
    case 185:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_CLAMPS;
      break;
    case 186:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_MAX;
      break;
    case 187:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_MIN;
      break;
    case 188:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_MAXU;
      break;
    case 189:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_MINU;
      break;
    case 190:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_MOVEQZ;
      break;
    case 191:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_MOVF;
      break;
    case 194:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_MOVGEZ;
      break;
    case 195:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_ORB;
      break;
    case 196:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_MOVLTZ;
      break;
    case 197:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_ORBC;
      break;
    case 198:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_SEXT;
      break;
    case 199:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_SRC;
      break;
    case 200:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_MOVNEZ;
      break;
    case 201:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_SRLI;
      break;
    case 202:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_SUB;
      break;
    case 203:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_SUBX4;
      break;
    case 204:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_SUBX2;
      break;
    case 205:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_SUBX8;
      break;
    case 206:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_XOR;
      break;
    case 207:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_XORB;
      break;
    case 208:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_MOVT;
      break;
    case 224:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1)
	return OPCODE_OR;
      break;
    case 244:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 1 &&
	  Field_ae_r32_Slot_ae_slot0_get (insn) == 0)
	return OPCODE_AE_SQ32F_X;
      break;
    }
  if (Field_op0_s4_Slot_ae_slot0_get (insn) == 5)
    return OPCODE_L32R;
  switch (Field_r_Slot_ae_slot0_get (insn))
    {
    case 0:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 4)
	return OPCODE_BNE;
      break;
    case 1:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 4)
	return OPCODE_BNONE;
      break;
    case 2:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 4)
	return OPCODE_L16SI;
      break;
    case 3:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 4)
	return OPCODE_L8UI;
      break;
    case 4:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_ADDI;
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 4)
	return OPCODE_L16UI;
      break;
    case 5:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_BALL;
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 4)
	return OPCODE_S16I;
      break;
    case 6:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_BANY;
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 4)
	return OPCODE_S32I;
      break;
    case 7:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_BBC;
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 4)
	return OPCODE_S8I;
      break;
    case 8:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_ADDMI;
      break;
    case 9:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_BBS;
      break;
    case 10:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_BEQ;
      break;
    case 11:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_BGEU;
      break;
    case 12:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_BGE;
      break;
    case 13:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_BLT;
      break;
    case 14:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_BLTU;
      break;
    case 15:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 2)
	return OPCODE_BNALL;
      break;
    }
  switch (Field_t_Slot_ae_slot0_get (insn))
    {
    case 0:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 3)
	return OPCODE_BEQI;
      break;
    case 1:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 3)
	return OPCODE_BGEI;
      break;
    case 2:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 3)
	return OPCODE_BGEUI;
      break;
    case 3:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 3)
	return OPCODE_BNEI;
      break;
    case 4:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 3)
	return OPCODE_BLTI;
      break;
    case 5:
      if (Field_op0_s4_Slot_ae_slot0_get (insn) == 3 &&
	  Field_r_Slot_ae_slot0_get (insn) == 0)
	return OPCODE_BF;
      break;
    }
  return 0;
}

static int
Slot_ae_slot1_decode (const xtensa_insnbuf insn)
{
  if (Field_ftsf100ae_slot1_Slot_ae_slot1_get (insn) == 115 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ae_r20_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_NEGSP24S;
  if (Field_ftsf101ae_slot1_Slot_ae_slot1_get (insn) == 29 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ftsf348ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_ABSSP24S;
  if (Field_ftsf103ae_slot1_Slot_ae_slot1_get (insn) == 15 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ftsf349ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_NEGP24;
  if (Field_ftsf104ae_slot1_Slot_ae_slot1_get (insn) == 0 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5)
    return OPCODE_AE_MAXBQ56S;
  if (Field_ftsf105ae_slot1_Slot_ae_slot1_get (insn) == 1 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5)
    return OPCODE_AE_MINBQ56S;
  if (Field_ftsf106ae_slot1_Slot_ae_slot1_get (insn) == 2 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5 &&
      Field_ae_r32_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_EQQ56;
  if (Field_ftsf107ae_slot1_Slot_ae_slot1_get (insn) == 48 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5)
    return OPCODE_AE_ADDSQ56S;
  if (Field_ftsf108ae_slot1_Slot_ae_slot1_get (insn) == 49 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5)
    return OPCODE_AE_ANDQ56;
  if (Field_ftsf109ae_slot1_Slot_ae_slot1_get (insn) == 50 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5)
    return OPCODE_AE_MAXQ56S;
  if (Field_ftsf110ae_slot1_Slot_ae_slot1_get (insn) == 51 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5)
    return OPCODE_AE_ORQ56;
  if (Field_ftsf111ae_slot1_Slot_ae_slot1_get (insn) == 52 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5)
    return OPCODE_AE_MINQ56S;
  if (Field_ftsf112ae_slot1_Slot_ae_slot1_get (insn) == 53 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5)
    return OPCODE_AE_SUBQ56;
  if (Field_ftsf113ae_slot1_Slot_ae_slot1_get (insn) == 54 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5)
    return OPCODE_AE_SUBSQ56S;
  if (Field_ftsf114ae_slot1_Slot_ae_slot1_get (insn) == 55 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5)
    return OPCODE_AE_XORQ56;
  if (Field_ftsf115ae_slot1_Slot_ae_slot1_get (insn) == 56 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5)
    return OPCODE_AE_NANDQ56;
  if (Field_ftsf116ae_slot1_Slot_ae_slot1_get (insn) == 57 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5)
    return OPCODE_AE_ABSQ56;
  if (Field_ftsf118ae_slot1_Slot_ae_slot1_get (insn) == 185 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5)
    return OPCODE_AE_NEGSQ56S;
  if (Field_ftsf119ae_slot1_Slot_ae_slot1_get (insn) == 185 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5 &&
      Field_ftsf338_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_SATQ48S;
  if (Field_ftsf12_Slot_ae_slot1_get (insn) == 1 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5 &&
      Field_ftsf341ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_LTQ56S;
  if (Field_ftsf120ae_slot1_Slot_ae_slot1_get (insn) == 29 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5 &&
      Field_ftsf343ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_ABSSQ56S;
  if (Field_ftsf122ae_slot1_Slot_ae_slot1_get (insn) == 15 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5 &&
      Field_ftsf346ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_NEGQ56;
  if (Field_ftsf124ae_slot1_Slot_ae_slot1_get (insn) == 1 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5 &&
      Field_ftsf339ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_LEQ56S;
  if (Field_ftsf125ae_slot1_Slot_ae_slot1_get (insn) == 1 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5 &&
      Field_ftsf350ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_TRUNCP24Q48X2;
  if (Field_ftsf126ae_slot1_Slot_ae_slot1_get (insn) == 1 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 5 &&
      Field_ftsf344ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_ADDQ56;
  if (Field_ftsf127ae_slot1_Slot_ae_slot1_get (insn) == 0 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAAFP24S_HH_LL;
  if (Field_ftsf128ae_slot1_Slot_ae_slot1_get (insn) == 1 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAAFP24S_HL_LH;
  if (Field_ftsf129ae_slot1_Slot_ae_slot1_get (insn) == 2 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAAP24S_HH_LL;
  if (Field_ftsf13_Slot_ae_slot1_get (insn) == 2 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ftsf12_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_SLLISP24S;
  if (Field_ftsf130ae_slot1_Slot_ae_slot1_get (insn) == 3 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAFS32P16S_HL;
  if (Field_ftsf131ae_slot1_Slot_ae_slot1_get (insn) == 4 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAAP24S_HL_LH;
  if (Field_ftsf132ae_slot1_Slot_ae_slot1_get (insn) == 5 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAFS32P16S_LH;
  if (Field_ftsf133ae_slot1_Slot_ae_slot1_get (insn) == 6 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAFS32P16S_LL;
  if (Field_ftsf134ae_slot1_Slot_ae_slot1_get (insn) == 7 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAFS56P24S_HH;
  if (Field_ftsf135ae_slot1_Slot_ae_slot1_get (insn) == 8 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAFP24S_HH;
  if (Field_ftsf136ae_slot1_Slot_ae_slot1_get (insn) == 9 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAFS56P24S_HL;
  if (Field_ftsf137ae_slot1_Slot_ae_slot1_get (insn) == 10 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAFS56P24S_LH;
  if (Field_ftsf138ae_slot1_Slot_ae_slot1_get (insn) == 11 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAP24S_HH;
  if (Field_ftsf139ae_slot1_Slot_ae_slot1_get (insn) == 12 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAFS56P24S_LL;
  if (Field_ftsf140ae_slot1_Slot_ae_slot1_get (insn) == 13 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAP24S_HL;
  if (Field_ftsf141ae_slot1_Slot_ae_slot1_get (insn) == 14 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAP24S_LH;
  if (Field_ftsf142ae_slot1_Slot_ae_slot1_get (insn) == 15 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAP24S_LL;
  if (Field_ftsf143ae_slot1_Slot_ae_slot1_get (insn) == 16 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAFP24S_HL;
  if (Field_ftsf144ae_slot1_Slot_ae_slot1_get (insn) == 17 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAS56P24S_HH;
  if (Field_ftsf145ae_slot1_Slot_ae_slot1_get (insn) == 18 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAS56P24S_HL;
  if (Field_ftsf146ae_slot1_Slot_ae_slot1_get (insn) == 19 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULASFP24S_HH_LL;
  if (Field_ftsf147ae_slot1_Slot_ae_slot1_get (insn) == 20 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAS56P24S_LH;
  if (Field_ftsf148ae_slot1_Slot_ae_slot1_get (insn) == 21 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULASFP24S_HL_LH;
  if (Field_ftsf149ae_slot1_Slot_ae_slot1_get (insn) == 22 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULASP24S_HH_LL;
  if (Field_ftsf150ae_slot1_Slot_ae_slot1_get (insn) == 23 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULASP24S_HL_LH;
  if (Field_ftsf151ae_slot1_Slot_ae_slot1_get (insn) == 24 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAS56P24S_LL;
  if (Field_ftsf152ae_slot1_Slot_ae_slot1_get (insn) == 25 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULFP24S_HH;
  if (Field_ftsf153ae_slot1_Slot_ae_slot1_get (insn) == 26 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULFP24S_HL;
  if (Field_ftsf154ae_slot1_Slot_ae_slot1_get (insn) == 27 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULFP24S_LL;
  if (Field_ftsf155ae_slot1_Slot_ae_slot1_get (insn) == 28 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULFP24S_LH;
  if (Field_ftsf156ae_slot1_Slot_ae_slot1_get (insn) == 29 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULFS32P16S_HH;
  if (Field_ftsf157ae_slot1_Slot_ae_slot1_get (insn) == 30 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULFS32P16S_HL;
  if (Field_ftsf158ae_slot1_Slot_ae_slot1_get (insn) == 31 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULFS32P16S_LH;
  if (Field_ftsf159ae_slot1_Slot_ae_slot1_get (insn) == 32 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAFP24S_LH;
  if (Field_ftsf160ae_slot1_Slot_ae_slot1_get (insn) == 33 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULFS32P16S_LL;
  if (Field_ftsf161ae_slot1_Slot_ae_slot1_get (insn) == 34 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULP24S_HH;
  if (Field_ftsf162ae_slot1_Slot_ae_slot1_get (insn) == 35 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSAFP24S_HH_LL;
  if (Field_ftsf163ae_slot1_Slot_ae_slot1_get (insn) == 36 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULP24S_HL;
  if (Field_ftsf164ae_slot1_Slot_ae_slot1_get (insn) == 37 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSAFP24S_HL_LH;
  if (Field_ftsf165ae_slot1_Slot_ae_slot1_get (insn) == 38 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSAP24S_HH_LL;
  if (Field_ftsf166ae_slot1_Slot_ae_slot1_get (insn) == 39 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSAP24S_HL_LH;
  if (Field_ftsf167ae_slot1_Slot_ae_slot1_get (insn) == 40 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULP24S_LH;
  if (Field_ftsf168ae_slot1_Slot_ae_slot1_get (insn) == 41 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSFP24S_HH;
  if (Field_ftsf169ae_slot1_Slot_ae_slot1_get (insn) == 42 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSFP24S_HL;
  if (Field_ftsf170ae_slot1_Slot_ae_slot1_get (insn) == 43 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSFP24S_LL;
  if (Field_ftsf171ae_slot1_Slot_ae_slot1_get (insn) == 44 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSFP24S_LH;
  if (Field_ftsf172ae_slot1_Slot_ae_slot1_get (insn) == 45 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSFS32P16S_HH;
  if (Field_ftsf173ae_slot1_Slot_ae_slot1_get (insn) == 46 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSFS32P16S_HL;
  if (Field_ftsf174ae_slot1_Slot_ae_slot1_get (insn) == 47 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSFS32P16S_LH;
  if (Field_ftsf175ae_slot1_Slot_ae_slot1_get (insn) == 48 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULP24S_LL;
  if (Field_ftsf176ae_slot1_Slot_ae_slot1_get (insn) == 49 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSFS32P16S_LL;
  if (Field_ftsf177ae_slot1_Slot_ae_slot1_get (insn) == 50 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSFS56P24S_HH;
  if (Field_ftsf178ae_slot1_Slot_ae_slot1_get (insn) == 51 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSFS56P24S_LL;
  if (Field_ftsf179ae_slot1_Slot_ae_slot1_get (insn) == 52 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSFS56P24S_HL;
  if (Field_ftsf180ae_slot1_Slot_ae_slot1_get (insn) == 53 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSP24S_HH;
  if (Field_ftsf181ae_slot1_Slot_ae_slot1_get (insn) == 54 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSP24S_HL;
  if (Field_ftsf182ae_slot1_Slot_ae_slot1_get (insn) == 55 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSP24S_LH;
  if (Field_ftsf183ae_slot1_Slot_ae_slot1_get (insn) == 56 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSFS56P24S_LH;
  if (Field_ftsf184ae_slot1_Slot_ae_slot1_get (insn) == 57 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSP24S_LL;
  if (Field_ftsf185ae_slot1_Slot_ae_slot1_get (insn) == 58 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSS56P24S_HH;
  if (Field_ftsf186ae_slot1_Slot_ae_slot1_get (insn) == 59 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSS56P24S_LH;
  if (Field_ftsf187ae_slot1_Slot_ae_slot1_get (insn) == 60 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSS56P24S_HL;
  if (Field_ftsf188ae_slot1_Slot_ae_slot1_get (insn) == 61 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSS56P24S_LL;
  if (Field_ftsf189ae_slot1_Slot_ae_slot1_get (insn) == 62 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSSFP24S_HH_LL;
  if (Field_ftsf190ae_slot1_Slot_ae_slot1_get (insn) == 63 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSSFP24S_HL_LH;
  if (Field_ftsf191ae_slot1_Slot_ae_slot1_get (insn) == 64 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULAFP24S_LL;
  if (Field_ftsf192ae_slot1_Slot_ae_slot1_get (insn) == 65 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSSP24S_HH_LL;
  if (Field_ftsf193ae_slot1_Slot_ae_slot1_get (insn) == 66 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULSSP24S_HL_LH;
  if (Field_ftsf194ae_slot1_Slot_ae_slot1_get (insn) == 67 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULZASFP24S_HH_LL;
  if (Field_ftsf195ae_slot1_Slot_ae_slot1_get (insn) == 68 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULZAAFP24S_HH_LL;
  if (Field_ftsf196ae_slot1_Slot_ae_slot1_get (insn) == 69 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULZASFP24S_HL_LH;
  if (Field_ftsf197ae_slot1_Slot_ae_slot1_get (insn) == 70 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULZASP24S_HH_LL;
  if (Field_ftsf198ae_slot1_Slot_ae_slot1_get (insn) == 71 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULZASP24S_HL_LH;
  if (Field_ftsf199ae_slot1_Slot_ae_slot1_get (insn) == 72 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULZAAFP24S_HL_LH;
  if (Field_ftsf200ae_slot1_Slot_ae_slot1_get (insn) == 73 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULZSAFP24S_HH_LL;
  if (Field_ftsf201ae_slot1_Slot_ae_slot1_get (insn) == 74 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULZSAFP24S_HL_LH;
  if (Field_ftsf202ae_slot1_Slot_ae_slot1_get (insn) == 75 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULZSAP24S_HL_LH;
  if (Field_ftsf203ae_slot1_Slot_ae_slot1_get (insn) == 76 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULZSAP24S_HH_LL;
  if (Field_ftsf204ae_slot1_Slot_ae_slot1_get (insn) == 77 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULZSSFP24S_HH_LL;
  if (Field_ftsf205ae_slot1_Slot_ae_slot1_get (insn) == 78 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULZSSFP24S_HL_LH;
  if (Field_ftsf206ae_slot1_Slot_ae_slot1_get (insn) == 79 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6)
    return OPCODE_AE_MULZSSP24S_HH_LL;
  if (Field_ftsf207ae_slot1_Slot_ae_slot1_get (insn) == 10 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6 &&
      Field_ftsf336ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_MULZAAP24S_HH_LL;
  if (Field_ftsf209ae_slot1_Slot_ae_slot1_get (insn) == 11 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6 &&
      Field_ftsf336ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_MULZSSP24S_HL_LH;
  if (Field_ftsf210ae_slot1_Slot_ae_slot1_get (insn) == 3 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6 &&
      Field_ftsf337ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_MULZAAP24S_HL_LH;
  if (Field_ftsf211ae_slot1_Slot_ae_slot1_get (insn) == 1 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 6 &&
      Field_ftsf332ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_MULAFS32P16S_HH;
  if (Field_ftsf21ae_slot1_Slot_ae_slot1_get (insn) == 0 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MAXBP24S;
  if (Field_ftsf22ae_slot1_Slot_ae_slot1_get (insn) == 1 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MINBP24S;
  if (Field_ftsf23ae_slot1_Slot_ae_slot1_get (insn) == 8 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MOVFP48;
  if (Field_ftsf24ae_slot1_Slot_ae_slot1_get (insn) == 9 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MOVTP48;
  if (Field_ftsf25ae_slot1_Slot_ae_slot1_get (insn) == 20 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_ADDP24;
  if (Field_ftsf26ae_slot1_Slot_ae_slot1_get (insn) == 21 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_ANDP48;
  if (Field_ftsf27ae_slot1_Slot_ae_slot1_get (insn) == 22 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MAXP24S;
  if (Field_ftsf28ae_slot1_Slot_ae_slot1_get (insn) == 23 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MINP24S;
  if (Field_ftsf29ae_slot1_Slot_ae_slot1_get (insn) == 24 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_ADDSP24S;
  if (Field_ftsf30ae_slot1_Slot_ae_slot1_get (insn) == 25 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_NANDP48;
  if (Field_ftsf31ae_slot1_Slot_ae_slot1_get (insn) == 26 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_ORP48;
  if (Field_ftsf32ae_slot1_Slot_ae_slot1_get (insn) == 27 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_SELP24_HL;
  if (Field_ftsf33ae_slot1_Slot_ae_slot1_get (insn) == 28 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_SELP24_HH;
  if (Field_ftsf34ae_slot1_Slot_ae_slot1_get (insn) == 29 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_SELP24_LH;
  if (Field_ftsf35ae_slot1_Slot_ae_slot1_get (insn) == 30 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_SELP24_LL;
  if (Field_ftsf36ae_slot1_Slot_ae_slot1_get (insn) == 31 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_SUBP24;
  switch (Field_ftsf37ae_slot1_Slot_ae_slot1_get (insn))
    {
    case 8:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
	return OPCODE_AE_SLLIP24;
      break;
    case 9:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
	return OPCODE_AE_SRAIP24;
      break;
    case 10:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
	return OPCODE_AE_SRLIP24;
      break;
    }
  if (Field_ftsf38ae_slot1_Slot_ae_slot1_get (insn) == 176 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULAFQ32SP16S_L;
  if (Field_ftsf39ae_slot1_Slot_ae_slot1_get (insn) == 177 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULAFQ32SP16U_H;
  if (Field_ftsf40ae_slot1_Slot_ae_slot1_get (insn) == 178 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULAFQ32SP16U_L;
  if (Field_ftsf41ae_slot1_Slot_ae_slot1_get (insn) == 179 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULAQ32SP16U_H;
  if (Field_ftsf42ae_slot1_Slot_ae_slot1_get (insn) == 180 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULAQ32SP16S_H;
  if (Field_ftsf43ae_slot1_Slot_ae_slot1_get (insn) == 181 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULAQ32SP16U_L;
  if (Field_ftsf44ae_slot1_Slot_ae_slot1_get (insn) == 182 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULFQ32SP16S_H;
  if (Field_ftsf45ae_slot1_Slot_ae_slot1_get (insn) == 183 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULFQ32SP16S_L;
  if (Field_ftsf46ae_slot1_Slot_ae_slot1_get (insn) == 184 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULAQ32SP16S_L;
  if (Field_ftsf47ae_slot1_Slot_ae_slot1_get (insn) == 185 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULFQ32SP16U_H;
  if (Field_ftsf48ae_slot1_Slot_ae_slot1_get (insn) == 186 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULFQ32SP16U_L;
  if (Field_ftsf49ae_slot1_Slot_ae_slot1_get (insn) == 187 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULQ32SP16S_L;
  if (Field_ftsf50ae_slot1_Slot_ae_slot1_get (insn) == 188 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULQ32SP16S_H;
  if (Field_ftsf51ae_slot1_Slot_ae_slot1_get (insn) == 189 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULQ32SP16U_H;
  if (Field_ftsf52ae_slot1_Slot_ae_slot1_get (insn) == 190 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULQ32SP16U_L;
  if (Field_ftsf53ae_slot1_Slot_ae_slot1_get (insn) == 191 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULSFQ32SP16S_H;
  if (Field_ftsf54ae_slot1_Slot_ae_slot1_get (insn) == 192 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULAFQ32SP16S_H;
  if (Field_ftsf55ae_slot1_Slot_ae_slot1_get (insn) == 193 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULSFQ32SP16S_L;
  if (Field_ftsf56ae_slot1_Slot_ae_slot1_get (insn) == 194 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULSFQ32SP16U_H;
  if (Field_ftsf57ae_slot1_Slot_ae_slot1_get (insn) == 195 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULSQ32SP16U_L;
  if (Field_ftsf58ae_slot1_Slot_ae_slot1_get (insn) == 196 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MULSFQ32SP16U_L;
  if (Field_ftsf59ae_slot1_Slot_ae_slot1_get (insn) == 773 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_CVTQ48P24S_H;
  if (Field_ftsf60ae_slot1_Slot_ae_slot1_get (insn) == 789 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ae_r20_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_ZEROQ56;
  if (Field_ftsf61ae_slot1_Slot_ae_slot1_get (insn) == 405 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ftsf330ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_NOP;
  if (Field_ftsf63ae_slot1_Slot_ae_slot1_get (insn) == 198 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ae_r10_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_CVTQ48P24S_L;
  if (Field_ftsf64ae_slot1_Slot_ae_slot1_get (insn) == 1543 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MOVQ56;
  if (Field_ftsf66ae_slot1_Slot_ae_slot1_get (insn) == 1559 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_ROUNDSQ32ASYM;
  if (Field_ftsf67ae_slot1_Slot_ae_slot1_get (insn) == 791 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ftsf342ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_ROUNDSQ32SYM;
  if (Field_ftsf69ae_slot1_Slot_ae_slot1_get (insn) == 407 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ftsf340_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_TRUNCQ32;
  if (Field_ftsf71ae_slot1_Slot_ae_slot1_get (insn) == 25 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ae_s20_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_MULSQ32SP16S_H;
  if (Field_ftsf72ae_slot1_Slot_ae_slot1_get (insn) == 26 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ae_s20_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_MULSQ32SP16S_L;
  if (Field_ftsf73ae_slot1_Slot_ae_slot1_get (insn) == 417 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_MOVP48;
  if (Field_ftsf75ae_slot1_Slot_ae_slot1_get (insn) == 419 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_ROUNDSP16ASYM;
  if (Field_ftsf76ae_slot1_Slot_ae_slot1_get (insn) == 421 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_ROUNDSP16SYM;
  if (Field_ftsf77ae_slot1_Slot_ae_slot1_get (insn) == 423 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_SRASP24;
  if (Field_ftsf78ae_slot1_Slot_ae_slot1_get (insn) == 425 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_SLLSP24;
  if (Field_ftsf79ae_slot1_Slot_ae_slot1_get (insn) == 427 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_SRLSP24;
  if (Field_ftsf80ae_slot1_Slot_ae_slot1_get (insn) == 429 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_TRUNCP16;
  if (Field_ftsf81ae_slot1_Slot_ae_slot1_get (insn) == 431 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ae_r20_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_ZEROP48;
  if (Field_ftsf82ae_slot1_Slot_ae_slot1_get (insn) == 109 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ae_r10_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_SLLSSP24S;
  if (Field_ftsf84ae_slot1_Slot_ae_slot1_get (insn) == 881 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_ROUNDSP16Q48ASYM;
  if (Field_ftsf86ae_slot1_Slot_ae_slot1_get (insn) == 883 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_ROUNDSP16Q48SYM;
  if (Field_ftsf87ae_slot1_Slot_ae_slot1_get (insn) == 443 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ftsf342ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_ROUNDSP24Q48ASYM;
  if (Field_ftsf88ae_slot1_Slot_ae_slot1_get (insn) == 223 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ftsf340_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_ROUNDSP24Q48SYM;
  if (Field_ftsf89ae_slot1_Slot_ae_slot1_get (insn) == 7 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ftsf334ae_slot1_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_MULSQ32SP16U_H;
  if (Field_ftsf90ae_slot1_Slot_ae_slot1_get (insn) == 96 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_EQP24;
  if (Field_ftsf91ae_slot1_Slot_ae_slot1_get (insn) == 97 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_LEP24S;
  if (Field_ftsf92ae_slot1_Slot_ae_slot1_get (insn) == 49 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ftsf208_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_LTP24S;
  if (Field_ftsf94ae_slot1_Slot_ae_slot1_get (insn) == 25 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ftsf347_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_MOVFP24X2;
  if (Field_ftsf96ae_slot1_Slot_ae_slot1_get (insn) == 13 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ae_s20_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_MOVTP24X2;
  if (Field_ftsf97ae_slot1_Slot_ae_slot1_get (insn) == 112 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_SUBSP24S;
  if (Field_ftsf98ae_slot1_Slot_ae_slot1_get (insn) == 113 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1)
    return OPCODE_AE_XORP48;
  if (Field_ftsf99ae_slot1_Slot_ae_slot1_get (insn) == 114 &&
      Field_op0_s3_Slot_ae_slot1_get (insn) == 1 &&
      Field_ae_r20_Slot_ae_slot1_get (insn) == 0)
    return OPCODE_AE_ABSP24;
  switch (Field_t_Slot_ae_slot1_get (insn))
    {
    case 0:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZAAFQ32SP16S_HH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZASFQ32SP16U_LH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSAQ32SP16S_LL;
      break;
    case 1:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZAAFQ32SP16S_LH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZASFQ32SP16U_LL;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSAQ32SP16U_HH;
      break;
    case 2:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZAAFQ32SP16S_LL;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZASQ32SP16S_HH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSAQ32SP16U_LH;
      break;
    case 3:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZAAFQ32SP16U_LL;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZASQ32SP16U_HH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSSFQ32SP16S_LH;
      break;
    case 4:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZAAFQ32SP16U_HH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZASQ32SP16S_LH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSAQ32SP16U_LL;
      break;
    case 5:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZAAQ32SP16S_HH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZASQ32SP16U_LH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSSFQ32SP16S_LL;
      break;
    case 6:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZAAQ32SP16S_LH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZASQ32SP16U_LL;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSSFQ32SP16U_HH;
      break;
    case 7:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZAAQ32SP16S_LL;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZSAFQ32SP16S_HH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSSFQ32SP16U_LH;
      break;
    case 8:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZAAFQ32SP16U_LH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZASQ32SP16S_LL;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSSFQ32SP16S_HH;
      break;
    case 9:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZAAQ32SP16U_HH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZSAFQ32SP16S_LH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSSFQ32SP16U_LL;
      break;
    case 10:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZAAQ32SP16U_LH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZSAFQ32SP16S_LL;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSSQ32SP16S_HH;
      break;
    case 11:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZASFQ32SP16S_HH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZSAFQ32SP16U_LH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSSQ32SP16S_LL;
      break;
    case 12:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZAAQ32SP16U_LL;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZSAFQ32SP16U_HH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSSQ32SP16S_LH;
      break;
    case 13:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZASFQ32SP16S_LH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZSAFQ32SP16U_LL;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSSQ32SP16U_HH;
      break;
    case 14:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZASFQ32SP16S_LL;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZSAQ32SP16S_HH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSSQ32SP16U_LH;
      break;
    case 15:
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 2)
	return OPCODE_AE_MULZASFQ32SP16U_HH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 3)
	return OPCODE_AE_MULZSAQ32SP16S_LH;
      if (Field_op0_s3_Slot_ae_slot1_get (insn) == 4)
	return OPCODE_AE_MULZSSQ32SP16U_LL;
      break;
    }
  return 0;
}


/* Instruction slots.  */

static void
Slot_x24_Format_inst_0_get (const xtensa_insnbuf insn,
			    xtensa_insnbuf slotbuf)
{
  slotbuf[1] = 0;
  slotbuf[0] = (insn[0] & 0xffffff);
}

static void
Slot_x24_Format_inst_0_set (xtensa_insnbuf insn,
			    const xtensa_insnbuf slotbuf)
{
  insn[0] = (insn[0] & ~0xffffff) | (slotbuf[0] & 0xffffff);
}

static void
Slot_x16a_Format_inst16a_0_get (const xtensa_insnbuf insn,
				xtensa_insnbuf slotbuf)
{
  slotbuf[1] = 0;
  slotbuf[0] = (insn[0] & 0xffff);
}

static void
Slot_x16a_Format_inst16a_0_set (xtensa_insnbuf insn,
				const xtensa_insnbuf slotbuf)
{
  insn[0] = (insn[0] & ~0xffff) | (slotbuf[0] & 0xffff);
}

static void
Slot_x16b_Format_inst16b_0_get (const xtensa_insnbuf insn,
				xtensa_insnbuf slotbuf)
{
  slotbuf[1] = 0;
  slotbuf[0] = (insn[0] & 0xffff);
}

static void
Slot_x16b_Format_inst16b_0_set (xtensa_insnbuf insn,
				const xtensa_insnbuf slotbuf)
{
  insn[0] = (insn[0] & ~0xffff) | (slotbuf[0] & 0xffff);
}

static void
Slot_ae_format_Format_ae_slot1_31_get (const xtensa_insnbuf insn,
				      xtensa_insnbuf slotbuf)
{
  slotbuf[1] = 0;
  slotbuf[0] = ((insn[0] & 0x80000000) >> 31);
  slotbuf[0] = (slotbuf[0] & ~0x7ffffe) | ((insn[1] & 0x3fffff) << 1);
}

static void
Slot_ae_format_Format_ae_slot1_31_set (xtensa_insnbuf insn,
				      const xtensa_insnbuf slotbuf)
{
  insn[0] = (insn[0] & ~0x80000000) | ((slotbuf[0] & 0x1) << 31);
  insn[1] = (insn[1] & ~0x3fffff) | ((slotbuf[0] & 0x7ffffe) >> 1);
}

static void
Slot_ae_format_Format_ae_slot0_4_get (const xtensa_insnbuf insn,
				      xtensa_insnbuf slotbuf)
{
  slotbuf[1] = 0;
  slotbuf[0] = ((insn[0] & 0x7ffffff0) >> 4);
}

static void
Slot_ae_format_Format_ae_slot0_4_set (xtensa_insnbuf insn,
				      const xtensa_insnbuf slotbuf)
{
  insn[0] = (insn[0] & ~0x7ffffff0) | ((slotbuf[0] & 0x7ffffff) << 4);
}

static xtensa_get_field_fn
Slot_inst_get_field_fns[] = {
  Field_t_Slot_inst_get,
  Field_bbi4_Slot_inst_get,
  Field_bbi_Slot_inst_get,
  Field_imm12_Slot_inst_get,
  Field_imm8_Slot_inst_get,
  Field_s_Slot_inst_get,
  Field_imm12b_Slot_inst_get,
  Field_imm16_Slot_inst_get,
  Field_m_Slot_inst_get,
  Field_n_Slot_inst_get,
  Field_offset_Slot_inst_get,
  Field_op0_Slot_inst_get,
  Field_op1_Slot_inst_get,
  Field_op2_Slot_inst_get,
  Field_r_Slot_inst_get,
  Field_sa4_Slot_inst_get,
  Field_sae4_Slot_inst_get,
  Field_sae_Slot_inst_get,
  Field_sal_Slot_inst_get,
  Field_sargt_Slot_inst_get,
  Field_sas4_Slot_inst_get,
  Field_sas_Slot_inst_get,
  Field_sr_Slot_inst_get,
  Field_st_Slot_inst_get,
  Field_thi3_Slot_inst_get,
  Field_imm4_Slot_inst_get,
  Field_mn_Slot_inst_get,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_t2_Slot_inst_get,
  Field_s2_Slot_inst_get,
  Field_r2_Slot_inst_get,
  Field_t4_Slot_inst_get,
  Field_s4_Slot_inst_get,
  Field_r4_Slot_inst_get,
  Field_t8_Slot_inst_get,
  Field_s8_Slot_inst_get,
  Field_r8_Slot_inst_get,
  Field_xt_wbr15_imm_Slot_inst_get,
  Field_xt_wbr18_imm_Slot_inst_get,
  Field_ae_r3_Slot_inst_get,
  Field_ae_s_non_samt_Slot_inst_get,
  Field_ae_s3_Slot_inst_get,
  Field_ae_r32_Slot_inst_get,
  Field_ae_samt_s_t_Slot_inst_get,
  Field_ae_r20_Slot_inst_get,
  Field_ae_r10_Slot_inst_get,
  Field_ae_s20_Slot_inst_get,
  0,
  Field_ftsf12_Slot_inst_get,
  Field_ftsf13_Slot_inst_get,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Implicit_Field_ar0_get,
  Implicit_Field_ar4_get,
  Implicit_Field_ar8_get,
  Implicit_Field_ar12_get,
  Implicit_Field_bt16_get,
  Implicit_Field_bs16_get,
  Implicit_Field_br16_get,
  Implicit_Field_brall_get
};

static xtensa_set_field_fn
Slot_inst_set_field_fns[] = {
  Field_t_Slot_inst_set,
  Field_bbi4_Slot_inst_set,
  Field_bbi_Slot_inst_set,
  Field_imm12_Slot_inst_set,
  Field_imm8_Slot_inst_set,
  Field_s_Slot_inst_set,
  Field_imm12b_Slot_inst_set,
  Field_imm16_Slot_inst_set,
  Field_m_Slot_inst_set,
  Field_n_Slot_inst_set,
  Field_offset_Slot_inst_set,
  Field_op0_Slot_inst_set,
  Field_op1_Slot_inst_set,
  Field_op2_Slot_inst_set,
  Field_r_Slot_inst_set,
  Field_sa4_Slot_inst_set,
  Field_sae4_Slot_inst_set,
  Field_sae_Slot_inst_set,
  Field_sal_Slot_inst_set,
  Field_sargt_Slot_inst_set,
  Field_sas4_Slot_inst_set,
  Field_sas_Slot_inst_set,
  Field_sr_Slot_inst_set,
  Field_st_Slot_inst_set,
  Field_thi3_Slot_inst_set,
  Field_imm4_Slot_inst_set,
  Field_mn_Slot_inst_set,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_t2_Slot_inst_set,
  Field_s2_Slot_inst_set,
  Field_r2_Slot_inst_set,
  Field_t4_Slot_inst_set,
  Field_s4_Slot_inst_set,
  Field_r4_Slot_inst_set,
  Field_t8_Slot_inst_set,
  Field_s8_Slot_inst_set,
  Field_r8_Slot_inst_set,
  Field_xt_wbr15_imm_Slot_inst_set,
  Field_xt_wbr18_imm_Slot_inst_set,
  Field_ae_r3_Slot_inst_set,
  Field_ae_s_non_samt_Slot_inst_set,
  Field_ae_s3_Slot_inst_set,
  Field_ae_r32_Slot_inst_set,
  Field_ae_samt_s_t_Slot_inst_set,
  Field_ae_r20_Slot_inst_set,
  Field_ae_r10_Slot_inst_set,
  Field_ae_s20_Slot_inst_set,
  0,
  Field_ftsf12_Slot_inst_set,
  Field_ftsf13_Slot_inst_set,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set
};

static xtensa_get_field_fn
Slot_inst16a_get_field_fns[] = {
  Field_t_Slot_inst16a_get,
  0,
  0,
  0,
  0,
  Field_s_Slot_inst16a_get,
  0,
  0,
  0,
  0,
  0,
  Field_op0_Slot_inst16a_get,
  0,
  0,
  Field_r_Slot_inst16a_get,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_sr_Slot_inst16a_get,
  Field_st_Slot_inst16a_get,
  0,
  Field_imm4_Slot_inst16a_get,
  0,
  Field_i_Slot_inst16a_get,
  Field_imm6lo_Slot_inst16a_get,
  Field_imm6hi_Slot_inst16a_get,
  Field_imm7lo_Slot_inst16a_get,
  Field_imm7hi_Slot_inst16a_get,
  Field_z_Slot_inst16a_get,
  Field_imm6_Slot_inst16a_get,
  Field_imm7_Slot_inst16a_get,
  Field_t2_Slot_inst16a_get,
  Field_s2_Slot_inst16a_get,
  Field_r2_Slot_inst16a_get,
  Field_t4_Slot_inst16a_get,
  Field_s4_Slot_inst16a_get,
  Field_r4_Slot_inst16a_get,
  Field_t8_Slot_inst16a_get,
  Field_s8_Slot_inst16a_get,
  Field_r8_Slot_inst16a_get,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Implicit_Field_ar0_get,
  Implicit_Field_ar4_get,
  Implicit_Field_ar8_get,
  Implicit_Field_ar12_get,
  Implicit_Field_bt16_get,
  Implicit_Field_bs16_get,
  Implicit_Field_br16_get,
  Implicit_Field_brall_get
};

static xtensa_set_field_fn
Slot_inst16a_set_field_fns[] = {
  Field_t_Slot_inst16a_set,
  0,
  0,
  0,
  0,
  Field_s_Slot_inst16a_set,
  0,
  0,
  0,
  0,
  0,
  Field_op0_Slot_inst16a_set,
  0,
  0,
  Field_r_Slot_inst16a_set,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_sr_Slot_inst16a_set,
  Field_st_Slot_inst16a_set,
  0,
  Field_imm4_Slot_inst16a_set,
  0,
  Field_i_Slot_inst16a_set,
  Field_imm6lo_Slot_inst16a_set,
  Field_imm6hi_Slot_inst16a_set,
  Field_imm7lo_Slot_inst16a_set,
  Field_imm7hi_Slot_inst16a_set,
  Field_z_Slot_inst16a_set,
  Field_imm6_Slot_inst16a_set,
  Field_imm7_Slot_inst16a_set,
  Field_t2_Slot_inst16a_set,
  Field_s2_Slot_inst16a_set,
  Field_r2_Slot_inst16a_set,
  Field_t4_Slot_inst16a_set,
  Field_s4_Slot_inst16a_set,
  Field_r4_Slot_inst16a_set,
  Field_t8_Slot_inst16a_set,
  Field_s8_Slot_inst16a_set,
  Field_r8_Slot_inst16a_set,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set
};

static xtensa_get_field_fn
Slot_inst16b_get_field_fns[] = {
  Field_t_Slot_inst16b_get,
  0,
  0,
  0,
  0,
  Field_s_Slot_inst16b_get,
  0,
  0,
  0,
  0,
  0,
  Field_op0_Slot_inst16b_get,
  0,
  0,
  Field_r_Slot_inst16b_get,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_sr_Slot_inst16b_get,
  Field_st_Slot_inst16b_get,
  0,
  Field_imm4_Slot_inst16b_get,
  0,
  Field_i_Slot_inst16b_get,
  Field_imm6lo_Slot_inst16b_get,
  Field_imm6hi_Slot_inst16b_get,
  Field_imm7lo_Slot_inst16b_get,
  Field_imm7hi_Slot_inst16b_get,
  Field_z_Slot_inst16b_get,
  Field_imm6_Slot_inst16b_get,
  Field_imm7_Slot_inst16b_get,
  Field_t2_Slot_inst16b_get,
  Field_s2_Slot_inst16b_get,
  Field_r2_Slot_inst16b_get,
  Field_t4_Slot_inst16b_get,
  Field_s4_Slot_inst16b_get,
  Field_r4_Slot_inst16b_get,
  Field_t8_Slot_inst16b_get,
  Field_s8_Slot_inst16b_get,
  Field_r8_Slot_inst16b_get,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Implicit_Field_ar0_get,
  Implicit_Field_ar4_get,
  Implicit_Field_ar8_get,
  Implicit_Field_ar12_get,
  Implicit_Field_bt16_get,
  Implicit_Field_bs16_get,
  Implicit_Field_br16_get,
  Implicit_Field_brall_get
};

static xtensa_set_field_fn
Slot_inst16b_set_field_fns[] = {
  Field_t_Slot_inst16b_set,
  0,
  0,
  0,
  0,
  Field_s_Slot_inst16b_set,
  0,
  0,
  0,
  0,
  0,
  Field_op0_Slot_inst16b_set,
  0,
  0,
  Field_r_Slot_inst16b_set,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_sr_Slot_inst16b_set,
  Field_st_Slot_inst16b_set,
  0,
  Field_imm4_Slot_inst16b_set,
  0,
  Field_i_Slot_inst16b_set,
  Field_imm6lo_Slot_inst16b_set,
  Field_imm6hi_Slot_inst16b_set,
  Field_imm7lo_Slot_inst16b_set,
  Field_imm7hi_Slot_inst16b_set,
  Field_z_Slot_inst16b_set,
  Field_imm6_Slot_inst16b_set,
  Field_imm7_Slot_inst16b_set,
  Field_t2_Slot_inst16b_set,
  Field_s2_Slot_inst16b_set,
  Field_r2_Slot_inst16b_set,
  Field_t4_Slot_inst16b_set,
  Field_s4_Slot_inst16b_set,
  Field_r4_Slot_inst16b_set,
  Field_t8_Slot_inst16b_set,
  Field_s8_Slot_inst16b_set,
  Field_r8_Slot_inst16b_set,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set
};

static xtensa_get_field_fn
Slot_ae_slot1_get_field_fns[] = {
  Field_t_Slot_ae_slot1_get,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_t2_Slot_ae_slot1_get,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_ae_r32_Slot_ae_slot1_get,
  0,
  Field_ae_r20_Slot_ae_slot1_get,
  Field_ae_r10_Slot_ae_slot1_get,
  Field_ae_s20_Slot_ae_slot1_get,
  Field_op0_s3_Slot_ae_slot1_get,
  Field_ftsf12_Slot_ae_slot1_get,
  Field_ftsf13_Slot_ae_slot1_get,
  Field_ftsf14_Slot_ae_slot1_get,
  Field_ftsf21ae_slot1_Slot_ae_slot1_get,
  Field_ftsf22ae_slot1_Slot_ae_slot1_get,
  Field_ftsf23ae_slot1_Slot_ae_slot1_get,
  Field_ftsf24ae_slot1_Slot_ae_slot1_get,
  Field_ftsf25ae_slot1_Slot_ae_slot1_get,
  Field_ftsf26ae_slot1_Slot_ae_slot1_get,
  Field_ftsf27ae_slot1_Slot_ae_slot1_get,
  Field_ftsf28ae_slot1_Slot_ae_slot1_get,
  Field_ftsf29ae_slot1_Slot_ae_slot1_get,
  Field_ftsf30ae_slot1_Slot_ae_slot1_get,
  Field_ftsf31ae_slot1_Slot_ae_slot1_get,
  Field_ftsf32ae_slot1_Slot_ae_slot1_get,
  Field_ftsf33ae_slot1_Slot_ae_slot1_get,
  Field_ftsf34ae_slot1_Slot_ae_slot1_get,
  Field_ftsf35ae_slot1_Slot_ae_slot1_get,
  Field_ftsf36ae_slot1_Slot_ae_slot1_get,
  Field_ftsf37ae_slot1_Slot_ae_slot1_get,
  Field_ftsf38ae_slot1_Slot_ae_slot1_get,
  Field_ftsf39ae_slot1_Slot_ae_slot1_get,
  Field_ftsf40ae_slot1_Slot_ae_slot1_get,
  Field_ftsf41ae_slot1_Slot_ae_slot1_get,
  Field_ftsf42ae_slot1_Slot_ae_slot1_get,
  Field_ftsf43ae_slot1_Slot_ae_slot1_get,
  Field_ftsf44ae_slot1_Slot_ae_slot1_get,
  Field_ftsf45ae_slot1_Slot_ae_slot1_get,
  Field_ftsf46ae_slot1_Slot_ae_slot1_get,
  Field_ftsf47ae_slot1_Slot_ae_slot1_get,
  Field_ftsf48ae_slot1_Slot_ae_slot1_get,
  Field_ftsf49ae_slot1_Slot_ae_slot1_get,
  Field_ftsf50ae_slot1_Slot_ae_slot1_get,
  Field_ftsf51ae_slot1_Slot_ae_slot1_get,
  Field_ftsf52ae_slot1_Slot_ae_slot1_get,
  Field_ftsf53ae_slot1_Slot_ae_slot1_get,
  Field_ftsf54ae_slot1_Slot_ae_slot1_get,
  Field_ftsf55ae_slot1_Slot_ae_slot1_get,
  Field_ftsf56ae_slot1_Slot_ae_slot1_get,
  Field_ftsf57ae_slot1_Slot_ae_slot1_get,
  Field_ftsf58ae_slot1_Slot_ae_slot1_get,
  Field_ftsf59ae_slot1_Slot_ae_slot1_get,
  Field_ftsf60ae_slot1_Slot_ae_slot1_get,
  Field_ftsf61ae_slot1_Slot_ae_slot1_get,
  Field_ftsf63ae_slot1_Slot_ae_slot1_get,
  Field_ftsf64ae_slot1_Slot_ae_slot1_get,
  Field_ftsf66ae_slot1_Slot_ae_slot1_get,
  Field_ftsf67ae_slot1_Slot_ae_slot1_get,
  Field_ftsf69ae_slot1_Slot_ae_slot1_get,
  Field_ftsf71ae_slot1_Slot_ae_slot1_get,
  Field_ftsf72ae_slot1_Slot_ae_slot1_get,
  Field_ftsf73ae_slot1_Slot_ae_slot1_get,
  Field_ftsf75ae_slot1_Slot_ae_slot1_get,
  Field_ftsf76ae_slot1_Slot_ae_slot1_get,
  Field_ftsf77ae_slot1_Slot_ae_slot1_get,
  Field_ftsf78ae_slot1_Slot_ae_slot1_get,
  Field_ftsf79ae_slot1_Slot_ae_slot1_get,
  Field_ftsf80ae_slot1_Slot_ae_slot1_get,
  Field_ftsf81ae_slot1_Slot_ae_slot1_get,
  Field_ftsf82ae_slot1_Slot_ae_slot1_get,
  Field_ftsf84ae_slot1_Slot_ae_slot1_get,
  Field_ftsf86ae_slot1_Slot_ae_slot1_get,
  Field_ftsf87ae_slot1_Slot_ae_slot1_get,
  Field_ftsf88ae_slot1_Slot_ae_slot1_get,
  Field_ftsf89ae_slot1_Slot_ae_slot1_get,
  Field_ftsf90ae_slot1_Slot_ae_slot1_get,
  Field_ftsf91ae_slot1_Slot_ae_slot1_get,
  Field_ftsf92ae_slot1_Slot_ae_slot1_get,
  Field_ftsf94ae_slot1_Slot_ae_slot1_get,
  Field_ftsf96ae_slot1_Slot_ae_slot1_get,
  Field_ftsf97ae_slot1_Slot_ae_slot1_get,
  Field_ftsf98ae_slot1_Slot_ae_slot1_get,
  Field_ftsf99ae_slot1_Slot_ae_slot1_get,
  Field_ftsf100ae_slot1_Slot_ae_slot1_get,
  Field_ftsf101ae_slot1_Slot_ae_slot1_get,
  Field_ftsf103ae_slot1_Slot_ae_slot1_get,
  Field_ftsf104ae_slot1_Slot_ae_slot1_get,
  Field_ftsf105ae_slot1_Slot_ae_slot1_get,
  Field_ftsf106ae_slot1_Slot_ae_slot1_get,
  Field_ftsf107ae_slot1_Slot_ae_slot1_get,
  Field_ftsf108ae_slot1_Slot_ae_slot1_get,
  Field_ftsf109ae_slot1_Slot_ae_slot1_get,
  Field_ftsf110ae_slot1_Slot_ae_slot1_get,
  Field_ftsf111ae_slot1_Slot_ae_slot1_get,
  Field_ftsf112ae_slot1_Slot_ae_slot1_get,
  Field_ftsf113ae_slot1_Slot_ae_slot1_get,
  Field_ftsf114ae_slot1_Slot_ae_slot1_get,
  Field_ftsf115ae_slot1_Slot_ae_slot1_get,
  Field_ftsf116ae_slot1_Slot_ae_slot1_get,
  Field_ftsf118ae_slot1_Slot_ae_slot1_get,
  Field_ftsf119ae_slot1_Slot_ae_slot1_get,
  Field_ftsf120ae_slot1_Slot_ae_slot1_get,
  Field_ftsf122ae_slot1_Slot_ae_slot1_get,
  Field_ftsf124ae_slot1_Slot_ae_slot1_get,
  Field_ftsf125ae_slot1_Slot_ae_slot1_get,
  Field_ftsf126ae_slot1_Slot_ae_slot1_get,
  Field_ftsf127ae_slot1_Slot_ae_slot1_get,
  Field_ftsf128ae_slot1_Slot_ae_slot1_get,
  Field_ftsf129ae_slot1_Slot_ae_slot1_get,
  Field_ftsf130ae_slot1_Slot_ae_slot1_get,
  Field_ftsf131ae_slot1_Slot_ae_slot1_get,
  Field_ftsf132ae_slot1_Slot_ae_slot1_get,
  Field_ftsf133ae_slot1_Slot_ae_slot1_get,
  Field_ftsf134ae_slot1_Slot_ae_slot1_get,
  Field_ftsf135ae_slot1_Slot_ae_slot1_get,
  Field_ftsf136ae_slot1_Slot_ae_slot1_get,
  Field_ftsf137ae_slot1_Slot_ae_slot1_get,
  Field_ftsf138ae_slot1_Slot_ae_slot1_get,
  Field_ftsf139ae_slot1_Slot_ae_slot1_get,
  Field_ftsf140ae_slot1_Slot_ae_slot1_get,
  Field_ftsf141ae_slot1_Slot_ae_slot1_get,
  Field_ftsf142ae_slot1_Slot_ae_slot1_get,
  Field_ftsf143ae_slot1_Slot_ae_slot1_get,
  Field_ftsf144ae_slot1_Slot_ae_slot1_get,
  Field_ftsf145ae_slot1_Slot_ae_slot1_get,
  Field_ftsf146ae_slot1_Slot_ae_slot1_get,
  Field_ftsf147ae_slot1_Slot_ae_slot1_get,
  Field_ftsf148ae_slot1_Slot_ae_slot1_get,
  Field_ftsf149ae_slot1_Slot_ae_slot1_get,
  Field_ftsf150ae_slot1_Slot_ae_slot1_get,
  Field_ftsf151ae_slot1_Slot_ae_slot1_get,
  Field_ftsf152ae_slot1_Slot_ae_slot1_get,
  Field_ftsf153ae_slot1_Slot_ae_slot1_get,
  Field_ftsf154ae_slot1_Slot_ae_slot1_get,
  Field_ftsf155ae_slot1_Slot_ae_slot1_get,
  Field_ftsf156ae_slot1_Slot_ae_slot1_get,
  Field_ftsf157ae_slot1_Slot_ae_slot1_get,
  Field_ftsf158ae_slot1_Slot_ae_slot1_get,
  Field_ftsf159ae_slot1_Slot_ae_slot1_get,
  Field_ftsf160ae_slot1_Slot_ae_slot1_get,
  Field_ftsf161ae_slot1_Slot_ae_slot1_get,
  Field_ftsf162ae_slot1_Slot_ae_slot1_get,
  Field_ftsf163ae_slot1_Slot_ae_slot1_get,
  Field_ftsf164ae_slot1_Slot_ae_slot1_get,
  Field_ftsf165ae_slot1_Slot_ae_slot1_get,
  Field_ftsf166ae_slot1_Slot_ae_slot1_get,
  Field_ftsf167ae_slot1_Slot_ae_slot1_get,
  Field_ftsf168ae_slot1_Slot_ae_slot1_get,
  Field_ftsf169ae_slot1_Slot_ae_slot1_get,
  Field_ftsf170ae_slot1_Slot_ae_slot1_get,
  Field_ftsf171ae_slot1_Slot_ae_slot1_get,
  Field_ftsf172ae_slot1_Slot_ae_slot1_get,
  Field_ftsf173ae_slot1_Slot_ae_slot1_get,
  Field_ftsf174ae_slot1_Slot_ae_slot1_get,
  Field_ftsf175ae_slot1_Slot_ae_slot1_get,
  Field_ftsf176ae_slot1_Slot_ae_slot1_get,
  Field_ftsf177ae_slot1_Slot_ae_slot1_get,
  Field_ftsf178ae_slot1_Slot_ae_slot1_get,
  Field_ftsf179ae_slot1_Slot_ae_slot1_get,
  Field_ftsf180ae_slot1_Slot_ae_slot1_get,
  Field_ftsf181ae_slot1_Slot_ae_slot1_get,
  Field_ftsf182ae_slot1_Slot_ae_slot1_get,
  Field_ftsf183ae_slot1_Slot_ae_slot1_get,
  Field_ftsf184ae_slot1_Slot_ae_slot1_get,
  Field_ftsf185ae_slot1_Slot_ae_slot1_get,
  Field_ftsf186ae_slot1_Slot_ae_slot1_get,
  Field_ftsf187ae_slot1_Slot_ae_slot1_get,
  Field_ftsf188ae_slot1_Slot_ae_slot1_get,
  Field_ftsf189ae_slot1_Slot_ae_slot1_get,
  Field_ftsf190ae_slot1_Slot_ae_slot1_get,
  Field_ftsf191ae_slot1_Slot_ae_slot1_get,
  Field_ftsf192ae_slot1_Slot_ae_slot1_get,
  Field_ftsf193ae_slot1_Slot_ae_slot1_get,
  Field_ftsf194ae_slot1_Slot_ae_slot1_get,
  Field_ftsf195ae_slot1_Slot_ae_slot1_get,
  Field_ftsf196ae_slot1_Slot_ae_slot1_get,
  Field_ftsf197ae_slot1_Slot_ae_slot1_get,
  Field_ftsf198ae_slot1_Slot_ae_slot1_get,
  Field_ftsf199ae_slot1_Slot_ae_slot1_get,
  Field_ftsf200ae_slot1_Slot_ae_slot1_get,
  Field_ftsf201ae_slot1_Slot_ae_slot1_get,
  Field_ftsf202ae_slot1_Slot_ae_slot1_get,
  Field_ftsf203ae_slot1_Slot_ae_slot1_get,
  Field_ftsf204ae_slot1_Slot_ae_slot1_get,
  Field_ftsf205ae_slot1_Slot_ae_slot1_get,
  Field_ftsf206ae_slot1_Slot_ae_slot1_get,
  Field_ftsf207ae_slot1_Slot_ae_slot1_get,
  Field_ftsf208_Slot_ae_slot1_get,
  Field_ftsf209ae_slot1_Slot_ae_slot1_get,
  Field_ftsf210ae_slot1_Slot_ae_slot1_get,
  Field_ftsf211ae_slot1_Slot_ae_slot1_get,
  Field_ftsf330ae_slot1_Slot_ae_slot1_get,
  Field_ftsf332ae_slot1_Slot_ae_slot1_get,
  Field_ftsf334ae_slot1_Slot_ae_slot1_get,
  Field_ftsf336ae_slot1_Slot_ae_slot1_get,
  Field_ftsf337ae_slot1_Slot_ae_slot1_get,
  Field_ftsf338_Slot_ae_slot1_get,
  Field_ftsf339ae_slot1_Slot_ae_slot1_get,
  Field_ftsf340_Slot_ae_slot1_get,
  Field_ftsf341ae_slot1_Slot_ae_slot1_get,
  Field_ftsf342ae_slot1_Slot_ae_slot1_get,
  Field_ftsf343ae_slot1_Slot_ae_slot1_get,
  Field_ftsf344ae_slot1_Slot_ae_slot1_get,
  Field_ftsf346ae_slot1_Slot_ae_slot1_get,
  Field_ftsf347_Slot_ae_slot1_get,
  Field_ftsf348ae_slot1_Slot_ae_slot1_get,
  Field_ftsf349ae_slot1_Slot_ae_slot1_get,
  Field_ftsf350ae_slot1_Slot_ae_slot1_get,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Implicit_Field_ar0_get,
  Implicit_Field_ar4_get,
  Implicit_Field_ar8_get,
  Implicit_Field_ar12_get,
  Implicit_Field_bt16_get,
  Implicit_Field_bs16_get,
  Implicit_Field_br16_get,
  Implicit_Field_brall_get
};

static xtensa_set_field_fn
Slot_ae_slot1_set_field_fns[] = {
  Field_t_Slot_ae_slot1_set,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_t2_Slot_ae_slot1_set,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_ae_r32_Slot_ae_slot1_set,
  0,
  Field_ae_r20_Slot_ae_slot1_set,
  Field_ae_r10_Slot_ae_slot1_set,
  Field_ae_s20_Slot_ae_slot1_set,
  Field_op0_s3_Slot_ae_slot1_set,
  Field_ftsf12_Slot_ae_slot1_set,
  Field_ftsf13_Slot_ae_slot1_set,
  Field_ftsf14_Slot_ae_slot1_set,
  Field_ftsf21ae_slot1_Slot_ae_slot1_set,
  Field_ftsf22ae_slot1_Slot_ae_slot1_set,
  Field_ftsf23ae_slot1_Slot_ae_slot1_set,
  Field_ftsf24ae_slot1_Slot_ae_slot1_set,
  Field_ftsf25ae_slot1_Slot_ae_slot1_set,
  Field_ftsf26ae_slot1_Slot_ae_slot1_set,
  Field_ftsf27ae_slot1_Slot_ae_slot1_set,
  Field_ftsf28ae_slot1_Slot_ae_slot1_set,
  Field_ftsf29ae_slot1_Slot_ae_slot1_set,
  Field_ftsf30ae_slot1_Slot_ae_slot1_set,
  Field_ftsf31ae_slot1_Slot_ae_slot1_set,
  Field_ftsf32ae_slot1_Slot_ae_slot1_set,
  Field_ftsf33ae_slot1_Slot_ae_slot1_set,
  Field_ftsf34ae_slot1_Slot_ae_slot1_set,
  Field_ftsf35ae_slot1_Slot_ae_slot1_set,
  Field_ftsf36ae_slot1_Slot_ae_slot1_set,
  Field_ftsf37ae_slot1_Slot_ae_slot1_set,
  Field_ftsf38ae_slot1_Slot_ae_slot1_set,
  Field_ftsf39ae_slot1_Slot_ae_slot1_set,
  Field_ftsf40ae_slot1_Slot_ae_slot1_set,
  Field_ftsf41ae_slot1_Slot_ae_slot1_set,
  Field_ftsf42ae_slot1_Slot_ae_slot1_set,
  Field_ftsf43ae_slot1_Slot_ae_slot1_set,
  Field_ftsf44ae_slot1_Slot_ae_slot1_set,
  Field_ftsf45ae_slot1_Slot_ae_slot1_set,
  Field_ftsf46ae_slot1_Slot_ae_slot1_set,
  Field_ftsf47ae_slot1_Slot_ae_slot1_set,
  Field_ftsf48ae_slot1_Slot_ae_slot1_set,
  Field_ftsf49ae_slot1_Slot_ae_slot1_set,
  Field_ftsf50ae_slot1_Slot_ae_slot1_set,
  Field_ftsf51ae_slot1_Slot_ae_slot1_set,
  Field_ftsf52ae_slot1_Slot_ae_slot1_set,
  Field_ftsf53ae_slot1_Slot_ae_slot1_set,
  Field_ftsf54ae_slot1_Slot_ae_slot1_set,
  Field_ftsf55ae_slot1_Slot_ae_slot1_set,
  Field_ftsf56ae_slot1_Slot_ae_slot1_set,
  Field_ftsf57ae_slot1_Slot_ae_slot1_set,
  Field_ftsf58ae_slot1_Slot_ae_slot1_set,
  Field_ftsf59ae_slot1_Slot_ae_slot1_set,
  Field_ftsf60ae_slot1_Slot_ae_slot1_set,
  Field_ftsf61ae_slot1_Slot_ae_slot1_set,
  Field_ftsf63ae_slot1_Slot_ae_slot1_set,
  Field_ftsf64ae_slot1_Slot_ae_slot1_set,
  Field_ftsf66ae_slot1_Slot_ae_slot1_set,
  Field_ftsf67ae_slot1_Slot_ae_slot1_set,
  Field_ftsf69ae_slot1_Slot_ae_slot1_set,
  Field_ftsf71ae_slot1_Slot_ae_slot1_set,
  Field_ftsf72ae_slot1_Slot_ae_slot1_set,
  Field_ftsf73ae_slot1_Slot_ae_slot1_set,
  Field_ftsf75ae_slot1_Slot_ae_slot1_set,
  Field_ftsf76ae_slot1_Slot_ae_slot1_set,
  Field_ftsf77ae_slot1_Slot_ae_slot1_set,
  Field_ftsf78ae_slot1_Slot_ae_slot1_set,
  Field_ftsf79ae_slot1_Slot_ae_slot1_set,
  Field_ftsf80ae_slot1_Slot_ae_slot1_set,
  Field_ftsf81ae_slot1_Slot_ae_slot1_set,
  Field_ftsf82ae_slot1_Slot_ae_slot1_set,
  Field_ftsf84ae_slot1_Slot_ae_slot1_set,
  Field_ftsf86ae_slot1_Slot_ae_slot1_set,
  Field_ftsf87ae_slot1_Slot_ae_slot1_set,
  Field_ftsf88ae_slot1_Slot_ae_slot1_set,
  Field_ftsf89ae_slot1_Slot_ae_slot1_set,
  Field_ftsf90ae_slot1_Slot_ae_slot1_set,
  Field_ftsf91ae_slot1_Slot_ae_slot1_set,
  Field_ftsf92ae_slot1_Slot_ae_slot1_set,
  Field_ftsf94ae_slot1_Slot_ae_slot1_set,
  Field_ftsf96ae_slot1_Slot_ae_slot1_set,
  Field_ftsf97ae_slot1_Slot_ae_slot1_set,
  Field_ftsf98ae_slot1_Slot_ae_slot1_set,
  Field_ftsf99ae_slot1_Slot_ae_slot1_set,
  Field_ftsf100ae_slot1_Slot_ae_slot1_set,
  Field_ftsf101ae_slot1_Slot_ae_slot1_set,
  Field_ftsf103ae_slot1_Slot_ae_slot1_set,
  Field_ftsf104ae_slot1_Slot_ae_slot1_set,
  Field_ftsf105ae_slot1_Slot_ae_slot1_set,
  Field_ftsf106ae_slot1_Slot_ae_slot1_set,
  Field_ftsf107ae_slot1_Slot_ae_slot1_set,
  Field_ftsf108ae_slot1_Slot_ae_slot1_set,
  Field_ftsf109ae_slot1_Slot_ae_slot1_set,
  Field_ftsf110ae_slot1_Slot_ae_slot1_set,
  Field_ftsf111ae_slot1_Slot_ae_slot1_set,
  Field_ftsf112ae_slot1_Slot_ae_slot1_set,
  Field_ftsf113ae_slot1_Slot_ae_slot1_set,
  Field_ftsf114ae_slot1_Slot_ae_slot1_set,
  Field_ftsf115ae_slot1_Slot_ae_slot1_set,
  Field_ftsf116ae_slot1_Slot_ae_slot1_set,
  Field_ftsf118ae_slot1_Slot_ae_slot1_set,
  Field_ftsf119ae_slot1_Slot_ae_slot1_set,
  Field_ftsf120ae_slot1_Slot_ae_slot1_set,
  Field_ftsf122ae_slot1_Slot_ae_slot1_set,
  Field_ftsf124ae_slot1_Slot_ae_slot1_set,
  Field_ftsf125ae_slot1_Slot_ae_slot1_set,
  Field_ftsf126ae_slot1_Slot_ae_slot1_set,
  Field_ftsf127ae_slot1_Slot_ae_slot1_set,
  Field_ftsf128ae_slot1_Slot_ae_slot1_set,
  Field_ftsf129ae_slot1_Slot_ae_slot1_set,
  Field_ftsf130ae_slot1_Slot_ae_slot1_set,
  Field_ftsf131ae_slot1_Slot_ae_slot1_set,
  Field_ftsf132ae_slot1_Slot_ae_slot1_set,
  Field_ftsf133ae_slot1_Slot_ae_slot1_set,
  Field_ftsf134ae_slot1_Slot_ae_slot1_set,
  Field_ftsf135ae_slot1_Slot_ae_slot1_set,
  Field_ftsf136ae_slot1_Slot_ae_slot1_set,
  Field_ftsf137ae_slot1_Slot_ae_slot1_set,
  Field_ftsf138ae_slot1_Slot_ae_slot1_set,
  Field_ftsf139ae_slot1_Slot_ae_slot1_set,
  Field_ftsf140ae_slot1_Slot_ae_slot1_set,
  Field_ftsf141ae_slot1_Slot_ae_slot1_set,
  Field_ftsf142ae_slot1_Slot_ae_slot1_set,
  Field_ftsf143ae_slot1_Slot_ae_slot1_set,
  Field_ftsf144ae_slot1_Slot_ae_slot1_set,
  Field_ftsf145ae_slot1_Slot_ae_slot1_set,
  Field_ftsf146ae_slot1_Slot_ae_slot1_set,
  Field_ftsf147ae_slot1_Slot_ae_slot1_set,
  Field_ftsf148ae_slot1_Slot_ae_slot1_set,
  Field_ftsf149ae_slot1_Slot_ae_slot1_set,
  Field_ftsf150ae_slot1_Slot_ae_slot1_set,
  Field_ftsf151ae_slot1_Slot_ae_slot1_set,
  Field_ftsf152ae_slot1_Slot_ae_slot1_set,
  Field_ftsf153ae_slot1_Slot_ae_slot1_set,
  Field_ftsf154ae_slot1_Slot_ae_slot1_set,
  Field_ftsf155ae_slot1_Slot_ae_slot1_set,
  Field_ftsf156ae_slot1_Slot_ae_slot1_set,
  Field_ftsf157ae_slot1_Slot_ae_slot1_set,
  Field_ftsf158ae_slot1_Slot_ae_slot1_set,
  Field_ftsf159ae_slot1_Slot_ae_slot1_set,
  Field_ftsf160ae_slot1_Slot_ae_slot1_set,
  Field_ftsf161ae_slot1_Slot_ae_slot1_set,
  Field_ftsf162ae_slot1_Slot_ae_slot1_set,
  Field_ftsf163ae_slot1_Slot_ae_slot1_set,
  Field_ftsf164ae_slot1_Slot_ae_slot1_set,
  Field_ftsf165ae_slot1_Slot_ae_slot1_set,
  Field_ftsf166ae_slot1_Slot_ae_slot1_set,
  Field_ftsf167ae_slot1_Slot_ae_slot1_set,
  Field_ftsf168ae_slot1_Slot_ae_slot1_set,
  Field_ftsf169ae_slot1_Slot_ae_slot1_set,
  Field_ftsf170ae_slot1_Slot_ae_slot1_set,
  Field_ftsf171ae_slot1_Slot_ae_slot1_set,
  Field_ftsf172ae_slot1_Slot_ae_slot1_set,
  Field_ftsf173ae_slot1_Slot_ae_slot1_set,
  Field_ftsf174ae_slot1_Slot_ae_slot1_set,
  Field_ftsf175ae_slot1_Slot_ae_slot1_set,
  Field_ftsf176ae_slot1_Slot_ae_slot1_set,
  Field_ftsf177ae_slot1_Slot_ae_slot1_set,
  Field_ftsf178ae_slot1_Slot_ae_slot1_set,
  Field_ftsf179ae_slot1_Slot_ae_slot1_set,
  Field_ftsf180ae_slot1_Slot_ae_slot1_set,
  Field_ftsf181ae_slot1_Slot_ae_slot1_set,
  Field_ftsf182ae_slot1_Slot_ae_slot1_set,
  Field_ftsf183ae_slot1_Slot_ae_slot1_set,
  Field_ftsf184ae_slot1_Slot_ae_slot1_set,
  Field_ftsf185ae_slot1_Slot_ae_slot1_set,
  Field_ftsf186ae_slot1_Slot_ae_slot1_set,
  Field_ftsf187ae_slot1_Slot_ae_slot1_set,
  Field_ftsf188ae_slot1_Slot_ae_slot1_set,
  Field_ftsf189ae_slot1_Slot_ae_slot1_set,
  Field_ftsf190ae_slot1_Slot_ae_slot1_set,
  Field_ftsf191ae_slot1_Slot_ae_slot1_set,
  Field_ftsf192ae_slot1_Slot_ae_slot1_set,
  Field_ftsf193ae_slot1_Slot_ae_slot1_set,
  Field_ftsf194ae_slot1_Slot_ae_slot1_set,
  Field_ftsf195ae_slot1_Slot_ae_slot1_set,
  Field_ftsf196ae_slot1_Slot_ae_slot1_set,
  Field_ftsf197ae_slot1_Slot_ae_slot1_set,
  Field_ftsf198ae_slot1_Slot_ae_slot1_set,
  Field_ftsf199ae_slot1_Slot_ae_slot1_set,
  Field_ftsf200ae_slot1_Slot_ae_slot1_set,
  Field_ftsf201ae_slot1_Slot_ae_slot1_set,
  Field_ftsf202ae_slot1_Slot_ae_slot1_set,
  Field_ftsf203ae_slot1_Slot_ae_slot1_set,
  Field_ftsf204ae_slot1_Slot_ae_slot1_set,
  Field_ftsf205ae_slot1_Slot_ae_slot1_set,
  Field_ftsf206ae_slot1_Slot_ae_slot1_set,
  Field_ftsf207ae_slot1_Slot_ae_slot1_set,
  Field_ftsf208_Slot_ae_slot1_set,
  Field_ftsf209ae_slot1_Slot_ae_slot1_set,
  Field_ftsf210ae_slot1_Slot_ae_slot1_set,
  Field_ftsf211ae_slot1_Slot_ae_slot1_set,
  Field_ftsf330ae_slot1_Slot_ae_slot1_set,
  Field_ftsf332ae_slot1_Slot_ae_slot1_set,
  Field_ftsf334ae_slot1_Slot_ae_slot1_set,
  Field_ftsf336ae_slot1_Slot_ae_slot1_set,
  Field_ftsf337ae_slot1_Slot_ae_slot1_set,
  Field_ftsf338_Slot_ae_slot1_set,
  Field_ftsf339ae_slot1_Slot_ae_slot1_set,
  Field_ftsf340_Slot_ae_slot1_set,
  Field_ftsf341ae_slot1_Slot_ae_slot1_set,
  Field_ftsf342ae_slot1_Slot_ae_slot1_set,
  Field_ftsf343ae_slot1_Slot_ae_slot1_set,
  Field_ftsf344ae_slot1_Slot_ae_slot1_set,
  Field_ftsf346ae_slot1_Slot_ae_slot1_set,
  Field_ftsf347_Slot_ae_slot1_set,
  Field_ftsf348ae_slot1_Slot_ae_slot1_set,
  Field_ftsf349ae_slot1_Slot_ae_slot1_set,
  Field_ftsf350ae_slot1_Slot_ae_slot1_set,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set
};

static xtensa_get_field_fn
Slot_ae_slot0_get_field_fns[] = {
  Field_t_Slot_ae_slot0_get,
  0,
  Field_bbi_Slot_ae_slot0_get,
  Field_imm12_Slot_ae_slot0_get,
  Field_imm8_Slot_ae_slot0_get,
  Field_s_Slot_ae_slot0_get,
  Field_imm12b_Slot_ae_slot0_get,
  Field_imm16_Slot_ae_slot0_get,
  0,
  0,
  Field_offset_Slot_ae_slot0_get,
  0,
  0,
  Field_op2_Slot_ae_slot0_get,
  Field_r_Slot_ae_slot0_get,
  0,
  0,
  Field_sae_Slot_ae_slot0_get,
  Field_sal_Slot_ae_slot0_get,
  Field_sargt_Slot_ae_slot0_get,
  0,
  Field_sas_Slot_ae_slot0_get,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_s4_Slot_ae_slot0_get,
  0,
  0,
  Field_s8_Slot_ae_slot0_get,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_ae_r32_Slot_ae_slot0_get,
  Field_ae_samt_s_t_Slot_ae_slot0_get,
  Field_ae_r20_Slot_ae_slot0_get,
  Field_ae_r10_Slot_ae_slot0_get,
  Field_ae_s20_Slot_ae_slot0_get,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_op0_s4_Slot_ae_slot0_get,
  Field_ftsf212ae_slot0_Slot_ae_slot0_get,
  Field_ftsf213ae_slot0_Slot_ae_slot0_get,
  Field_ftsf214ae_slot0_Slot_ae_slot0_get,
  Field_ftsf215ae_slot0_Slot_ae_slot0_get,
  Field_ftsf216ae_slot0_Slot_ae_slot0_get,
  Field_ftsf217_Slot_ae_slot0_get,
  Field_ftsf218ae_slot0_Slot_ae_slot0_get,
  Field_ftsf219ae_slot0_Slot_ae_slot0_get,
  Field_ftsf220ae_slot0_Slot_ae_slot0_get,
  Field_ftsf221ae_slot0_Slot_ae_slot0_get,
  Field_ftsf222ae_slot0_Slot_ae_slot0_get,
  Field_ftsf223ae_slot0_Slot_ae_slot0_get,
  Field_ftsf224ae_slot0_Slot_ae_slot0_get,
  Field_ftsf225ae_slot0_Slot_ae_slot0_get,
  Field_ftsf226ae_slot0_Slot_ae_slot0_get,
  Field_ftsf227ae_slot0_Slot_ae_slot0_get,
  Field_ftsf228ae_slot0_Slot_ae_slot0_get,
  Field_ftsf229ae_slot0_Slot_ae_slot0_get,
  Field_ftsf230ae_slot0_Slot_ae_slot0_get,
  Field_ftsf231ae_slot0_Slot_ae_slot0_get,
  Field_ftsf232ae_slot0_Slot_ae_slot0_get,
  Field_ftsf233ae_slot0_Slot_ae_slot0_get,
  Field_ftsf234ae_slot0_Slot_ae_slot0_get,
  Field_ftsf235ae_slot0_Slot_ae_slot0_get,
  Field_ftsf236ae_slot0_Slot_ae_slot0_get,
  Field_ftsf237ae_slot0_Slot_ae_slot0_get,
  Field_ftsf238ae_slot0_Slot_ae_slot0_get,
  Field_ftsf239ae_slot0_Slot_ae_slot0_get,
  Field_ftsf240ae_slot0_Slot_ae_slot0_get,
  Field_ftsf241ae_slot0_Slot_ae_slot0_get,
  Field_ftsf242ae_slot0_Slot_ae_slot0_get,
  Field_ftsf243ae_slot0_Slot_ae_slot0_get,
  Field_ftsf244ae_slot0_Slot_ae_slot0_get,
  Field_ftsf245ae_slot0_Slot_ae_slot0_get,
  Field_ftsf246ae_slot0_Slot_ae_slot0_get,
  Field_ftsf247ae_slot0_Slot_ae_slot0_get,
  Field_ftsf248ae_slot0_Slot_ae_slot0_get,
  Field_ftsf249ae_slot0_Slot_ae_slot0_get,
  Field_ftsf250ae_slot0_Slot_ae_slot0_get,
  Field_ftsf251ae_slot0_Slot_ae_slot0_get,
  Field_ftsf252ae_slot0_Slot_ae_slot0_get,
  Field_ftsf253ae_slot0_Slot_ae_slot0_get,
  Field_ftsf254ae_slot0_Slot_ae_slot0_get,
  Field_ftsf255ae_slot0_Slot_ae_slot0_get,
  Field_ftsf256ae_slot0_Slot_ae_slot0_get,
  Field_ftsf257ae_slot0_Slot_ae_slot0_get,
  Field_ftsf258ae_slot0_Slot_ae_slot0_get,
  Field_ftsf259ae_slot0_Slot_ae_slot0_get,
  Field_ftsf260ae_slot0_Slot_ae_slot0_get,
  Field_ftsf261ae_slot0_Slot_ae_slot0_get,
  Field_ftsf262ae_slot0_Slot_ae_slot0_get,
  Field_ftsf263ae_slot0_Slot_ae_slot0_get,
  Field_ftsf264ae_slot0_Slot_ae_slot0_get,
  Field_ftsf265ae_slot0_Slot_ae_slot0_get,
  Field_ftsf266ae_slot0_Slot_ae_slot0_get,
  Field_ftsf267ae_slot0_Slot_ae_slot0_get,
  Field_ftsf268ae_slot0_Slot_ae_slot0_get,
  Field_ftsf269ae_slot0_Slot_ae_slot0_get,
  Field_ftsf270ae_slot0_Slot_ae_slot0_get,
  Field_ftsf271ae_slot0_Slot_ae_slot0_get,
  Field_ftsf272ae_slot0_Slot_ae_slot0_get,
  Field_ftsf273ae_slot0_Slot_ae_slot0_get,
  Field_ftsf274ae_slot0_Slot_ae_slot0_get,
  Field_ftsf275ae_slot0_Slot_ae_slot0_get,
  Field_ftsf276ae_slot0_Slot_ae_slot0_get,
  Field_ftsf277ae_slot0_Slot_ae_slot0_get,
  Field_ftsf278ae_slot0_Slot_ae_slot0_get,
  Field_ftsf279ae_slot0_Slot_ae_slot0_get,
  Field_ftsf281ae_slot0_Slot_ae_slot0_get,
  Field_ftsf282ae_slot0_Slot_ae_slot0_get,
  Field_ftsf283ae_slot0_Slot_ae_slot0_get,
  Field_ftsf284ae_slot0_Slot_ae_slot0_get,
  Field_ftsf286ae_slot0_Slot_ae_slot0_get,
  Field_ftsf288ae_slot0_Slot_ae_slot0_get,
  Field_ftsf290ae_slot0_Slot_ae_slot0_get,
  Field_ftsf292ae_slot0_Slot_ae_slot0_get,
  Field_ftsf293_Slot_ae_slot0_get,
  Field_ftsf294ae_slot0_Slot_ae_slot0_get,
  Field_ftsf295ae_slot0_Slot_ae_slot0_get,
  Field_ftsf296ae_slot0_Slot_ae_slot0_get,
  Field_ftsf297ae_slot0_Slot_ae_slot0_get,
  Field_ftsf298ae_slot0_Slot_ae_slot0_get,
  Field_ftsf299ae_slot0_Slot_ae_slot0_get,
  Field_ftsf300ae_slot0_Slot_ae_slot0_get,
  Field_ftsf301ae_slot0_Slot_ae_slot0_get,
  Field_ftsf302ae_slot0_Slot_ae_slot0_get,
  Field_ftsf303ae_slot0_Slot_ae_slot0_get,
  Field_ftsf304ae_slot0_Slot_ae_slot0_get,
  Field_ftsf306ae_slot0_Slot_ae_slot0_get,
  Field_ftsf308ae_slot0_Slot_ae_slot0_get,
  Field_ftsf309ae_slot0_Slot_ae_slot0_get,
  Field_ftsf310ae_slot0_Slot_ae_slot0_get,
  Field_ftsf311ae_slot0_Slot_ae_slot0_get,
  Field_ftsf312ae_slot0_Slot_ae_slot0_get,
  Field_ftsf313ae_slot0_Slot_ae_slot0_get,
  Field_ftsf314ae_slot0_Slot_ae_slot0_get,
  Field_ftsf315ae_slot0_Slot_ae_slot0_get,
  Field_ftsf316ae_slot0_Slot_ae_slot0_get,
  Field_ftsf317ae_slot0_Slot_ae_slot0_get,
  Field_ftsf318ae_slot0_Slot_ae_slot0_get,
  Field_ftsf319_Slot_ae_slot0_get,
  Field_ftsf320ae_slot0_Slot_ae_slot0_get,
  Field_ftsf321_Slot_ae_slot0_get,
  Field_ftsf322ae_slot0_Slot_ae_slot0_get,
  Field_ftsf323ae_slot0_Slot_ae_slot0_get,
  Field_ftsf324ae_slot0_Slot_ae_slot0_get,
  Field_ftsf325ae_slot0_Slot_ae_slot0_get,
  Field_ftsf326ae_slot0_Slot_ae_slot0_get,
  Field_ftsf328ae_slot0_Slot_ae_slot0_get,
  Field_ftsf329ae_slot0_Slot_ae_slot0_get,
  Field_ftsf352ae_slot0_Slot_ae_slot0_get,
  Field_ftsf353_Slot_ae_slot0_get,
  Field_ftsf354ae_slot0_Slot_ae_slot0_get,
  Field_ftsf356ae_slot0_Slot_ae_slot0_get,
  Field_ftsf357_Slot_ae_slot0_get,
  Field_ftsf358ae_slot0_Slot_ae_slot0_get,
  Field_ftsf359ae_slot0_Slot_ae_slot0_get,
  Field_ftsf360ae_slot0_Slot_ae_slot0_get,
  Field_ftsf361ae_slot0_Slot_ae_slot0_get,
  Field_ftsf362ae_slot0_Slot_ae_slot0_get,
  Field_ftsf364ae_slot0_Slot_ae_slot0_get,
  Field_ftsf365ae_slot0_Slot_ae_slot0_get,
  Field_ftsf366ae_slot0_Slot_ae_slot0_get,
  Field_ftsf368ae_slot0_Slot_ae_slot0_get,
  Field_ftsf369ae_slot0_Slot_ae_slot0_get,
  Implicit_Field_ar0_get,
  Implicit_Field_ar4_get,
  Implicit_Field_ar8_get,
  Implicit_Field_ar12_get,
  Implicit_Field_bt16_get,
  Implicit_Field_bs16_get,
  Implicit_Field_br16_get,
  Implicit_Field_brall_get
};

static xtensa_set_field_fn
Slot_ae_slot0_set_field_fns[] = {
  Field_t_Slot_ae_slot0_set,
  0,
  Field_bbi_Slot_ae_slot0_set,
  Field_imm12_Slot_ae_slot0_set,
  Field_imm8_Slot_ae_slot0_set,
  Field_s_Slot_ae_slot0_set,
  Field_imm12b_Slot_ae_slot0_set,
  Field_imm16_Slot_ae_slot0_set,
  0,
  0,
  Field_offset_Slot_ae_slot0_set,
  0,
  0,
  Field_op2_Slot_ae_slot0_set,
  Field_r_Slot_ae_slot0_set,
  0,
  0,
  Field_sae_Slot_ae_slot0_set,
  Field_sal_Slot_ae_slot0_set,
  Field_sargt_Slot_ae_slot0_set,
  0,
  Field_sas_Slot_ae_slot0_set,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_s4_Slot_ae_slot0_set,
  0,
  0,
  Field_s8_Slot_ae_slot0_set,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_ae_r32_Slot_ae_slot0_set,
  Field_ae_samt_s_t_Slot_ae_slot0_set,
  Field_ae_r20_Slot_ae_slot0_set,
  Field_ae_r10_Slot_ae_slot0_set,
  Field_ae_s20_Slot_ae_slot0_set,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  Field_op0_s4_Slot_ae_slot0_set,
  Field_ftsf212ae_slot0_Slot_ae_slot0_set,
  Field_ftsf213ae_slot0_Slot_ae_slot0_set,
  Field_ftsf214ae_slot0_Slot_ae_slot0_set,
  Field_ftsf215ae_slot0_Slot_ae_slot0_set,
  Field_ftsf216ae_slot0_Slot_ae_slot0_set,
  Field_ftsf217_Slot_ae_slot0_set,
  Field_ftsf218ae_slot0_Slot_ae_slot0_set,
  Field_ftsf219ae_slot0_Slot_ae_slot0_set,
  Field_ftsf220ae_slot0_Slot_ae_slot0_set,
  Field_ftsf221ae_slot0_Slot_ae_slot0_set,
  Field_ftsf222ae_slot0_Slot_ae_slot0_set,
  Field_ftsf223ae_slot0_Slot_ae_slot0_set,
  Field_ftsf224ae_slot0_Slot_ae_slot0_set,
  Field_ftsf225ae_slot0_Slot_ae_slot0_set,
  Field_ftsf226ae_slot0_Slot_ae_slot0_set,
  Field_ftsf227ae_slot0_Slot_ae_slot0_set,
  Field_ftsf228ae_slot0_Slot_ae_slot0_set,
  Field_ftsf229ae_slot0_Slot_ae_slot0_set,
  Field_ftsf230ae_slot0_Slot_ae_slot0_set,
  Field_ftsf231ae_slot0_Slot_ae_slot0_set,
  Field_ftsf232ae_slot0_Slot_ae_slot0_set,
  Field_ftsf233ae_slot0_Slot_ae_slot0_set,
  Field_ftsf234ae_slot0_Slot_ae_slot0_set,
  Field_ftsf235ae_slot0_Slot_ae_slot0_set,
  Field_ftsf236ae_slot0_Slot_ae_slot0_set,
  Field_ftsf237ae_slot0_Slot_ae_slot0_set,
  Field_ftsf238ae_slot0_Slot_ae_slot0_set,
  Field_ftsf239ae_slot0_Slot_ae_slot0_set,
  Field_ftsf240ae_slot0_Slot_ae_slot0_set,
  Field_ftsf241ae_slot0_Slot_ae_slot0_set,
  Field_ftsf242ae_slot0_Slot_ae_slot0_set,
  Field_ftsf243ae_slot0_Slot_ae_slot0_set,
  Field_ftsf244ae_slot0_Slot_ae_slot0_set,
  Field_ftsf245ae_slot0_Slot_ae_slot0_set,
  Field_ftsf246ae_slot0_Slot_ae_slot0_set,
  Field_ftsf247ae_slot0_Slot_ae_slot0_set,
  Field_ftsf248ae_slot0_Slot_ae_slot0_set,
  Field_ftsf249ae_slot0_Slot_ae_slot0_set,
  Field_ftsf250ae_slot0_Slot_ae_slot0_set,
  Field_ftsf251ae_slot0_Slot_ae_slot0_set,
  Field_ftsf252ae_slot0_Slot_ae_slot0_set,
  Field_ftsf253ae_slot0_Slot_ae_slot0_set,
  Field_ftsf254ae_slot0_Slot_ae_slot0_set,
  Field_ftsf255ae_slot0_Slot_ae_slot0_set,
  Field_ftsf256ae_slot0_Slot_ae_slot0_set,
  Field_ftsf257ae_slot0_Slot_ae_slot0_set,
  Field_ftsf258ae_slot0_Slot_ae_slot0_set,
  Field_ftsf259ae_slot0_Slot_ae_slot0_set,
  Field_ftsf260ae_slot0_Slot_ae_slot0_set,
  Field_ftsf261ae_slot0_Slot_ae_slot0_set,
  Field_ftsf262ae_slot0_Slot_ae_slot0_set,
  Field_ftsf263ae_slot0_Slot_ae_slot0_set,
  Field_ftsf264ae_slot0_Slot_ae_slot0_set,
  Field_ftsf265ae_slot0_Slot_ae_slot0_set,
  Field_ftsf266ae_slot0_Slot_ae_slot0_set,
  Field_ftsf267ae_slot0_Slot_ae_slot0_set,
  Field_ftsf268ae_slot0_Slot_ae_slot0_set,
  Field_ftsf269ae_slot0_Slot_ae_slot0_set,
  Field_ftsf270ae_slot0_Slot_ae_slot0_set,
  Field_ftsf271ae_slot0_Slot_ae_slot0_set,
  Field_ftsf272ae_slot0_Slot_ae_slot0_set,
  Field_ftsf273ae_slot0_Slot_ae_slot0_set,
  Field_ftsf274ae_slot0_Slot_ae_slot0_set,
  Field_ftsf275ae_slot0_Slot_ae_slot0_set,
  Field_ftsf276ae_slot0_Slot_ae_slot0_set,
  Field_ftsf277ae_slot0_Slot_ae_slot0_set,
  Field_ftsf278ae_slot0_Slot_ae_slot0_set,
  Field_ftsf279ae_slot0_Slot_ae_slot0_set,
  Field_ftsf281ae_slot0_Slot_ae_slot0_set,
  Field_ftsf282ae_slot0_Slot_ae_slot0_set,
  Field_ftsf283ae_slot0_Slot_ae_slot0_set,
  Field_ftsf284ae_slot0_Slot_ae_slot0_set,
  Field_ftsf286ae_slot0_Slot_ae_slot0_set,
  Field_ftsf288ae_slot0_Slot_ae_slot0_set,
  Field_ftsf290ae_slot0_Slot_ae_slot0_set,
  Field_ftsf292ae_slot0_Slot_ae_slot0_set,
  Field_ftsf293_Slot_ae_slot0_set,
  Field_ftsf294ae_slot0_Slot_ae_slot0_set,
  Field_ftsf295ae_slot0_Slot_ae_slot0_set,
  Field_ftsf296ae_slot0_Slot_ae_slot0_set,
  Field_ftsf297ae_slot0_Slot_ae_slot0_set,
  Field_ftsf298ae_slot0_Slot_ae_slot0_set,
  Field_ftsf299ae_slot0_Slot_ae_slot0_set,
  Field_ftsf300ae_slot0_Slot_ae_slot0_set,
  Field_ftsf301ae_slot0_Slot_ae_slot0_set,
  Field_ftsf302ae_slot0_Slot_ae_slot0_set,
  Field_ftsf303ae_slot0_Slot_ae_slot0_set,
  Field_ftsf304ae_slot0_Slot_ae_slot0_set,
  Field_ftsf306ae_slot0_Slot_ae_slot0_set,
  Field_ftsf308ae_slot0_Slot_ae_slot0_set,
  Field_ftsf309ae_slot0_Slot_ae_slot0_set,
  Field_ftsf310ae_slot0_Slot_ae_slot0_set,
  Field_ftsf311ae_slot0_Slot_ae_slot0_set,
  Field_ftsf312ae_slot0_Slot_ae_slot0_set,
  Field_ftsf313ae_slot0_Slot_ae_slot0_set,
  Field_ftsf314ae_slot0_Slot_ae_slot0_set,
  Field_ftsf315ae_slot0_Slot_ae_slot0_set,
  Field_ftsf316ae_slot0_Slot_ae_slot0_set,
  Field_ftsf317ae_slot0_Slot_ae_slot0_set,
  Field_ftsf318ae_slot0_Slot_ae_slot0_set,
  Field_ftsf319_Slot_ae_slot0_set,
  Field_ftsf320ae_slot0_Slot_ae_slot0_set,
  Field_ftsf321_Slot_ae_slot0_set,
  Field_ftsf322ae_slot0_Slot_ae_slot0_set,
  Field_ftsf323ae_slot0_Slot_ae_slot0_set,
  Field_ftsf324ae_slot0_Slot_ae_slot0_set,
  Field_ftsf325ae_slot0_Slot_ae_slot0_set,
  Field_ftsf326ae_slot0_Slot_ae_slot0_set,
  Field_ftsf328ae_slot0_Slot_ae_slot0_set,
  Field_ftsf329ae_slot0_Slot_ae_slot0_set,
  Field_ftsf352ae_slot0_Slot_ae_slot0_set,
  Field_ftsf353_Slot_ae_slot0_set,
  Field_ftsf354ae_slot0_Slot_ae_slot0_set,
  Field_ftsf356ae_slot0_Slot_ae_slot0_set,
  Field_ftsf357_Slot_ae_slot0_set,
  Field_ftsf358ae_slot0_Slot_ae_slot0_set,
  Field_ftsf359ae_slot0_Slot_ae_slot0_set,
  Field_ftsf360ae_slot0_Slot_ae_slot0_set,
  Field_ftsf361ae_slot0_Slot_ae_slot0_set,
  Field_ftsf362ae_slot0_Slot_ae_slot0_set,
  Field_ftsf364ae_slot0_Slot_ae_slot0_set,
  Field_ftsf365ae_slot0_Slot_ae_slot0_set,
  Field_ftsf366ae_slot0_Slot_ae_slot0_set,
  Field_ftsf368ae_slot0_Slot_ae_slot0_set,
  Field_ftsf369ae_slot0_Slot_ae_slot0_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set,
  Implicit_Field_set
};

static xtensa_slot_internal slots[] = {
  { "Inst", "x24", 0,
    Slot_x24_Format_inst_0_get, Slot_x24_Format_inst_0_set,
    Slot_inst_get_field_fns, Slot_inst_set_field_fns,
    Slot_inst_decode, "nop" },
  { "Inst16a", "x16a", 0,
    Slot_x16a_Format_inst16a_0_get, Slot_x16a_Format_inst16a_0_set,
    Slot_inst16a_get_field_fns, Slot_inst16a_set_field_fns,
    Slot_inst16a_decode, "" },
  { "Inst16b", "x16b", 0,
    Slot_x16b_Format_inst16b_0_get, Slot_x16b_Format_inst16b_0_set,
    Slot_inst16b_get_field_fns, Slot_inst16b_set_field_fns,
    Slot_inst16b_decode, "nop.n" },
  { "ae_slot1", "ae_format", 1,
    Slot_ae_format_Format_ae_slot1_31_get, Slot_ae_format_Format_ae_slot1_31_set,
    Slot_ae_slot1_get_field_fns, Slot_ae_slot1_set_field_fns,
    Slot_ae_slot1_decode, "nop" },
  { "ae_slot0", "ae_format", 0,
    Slot_ae_format_Format_ae_slot0_4_get, Slot_ae_format_Format_ae_slot0_4_set,
    Slot_ae_slot0_get_field_fns, Slot_ae_slot0_set_field_fns,
    Slot_ae_slot0_decode, "nop" }
};


/* Instruction formats.  */

static void
Format_x24_encode (xtensa_insnbuf insn)
{
  insn[0] = 0;
  insn[1] = 0;
}

static void
Format_x16a_encode (xtensa_insnbuf insn)
{
  insn[0] = 0x8;
  insn[1] = 0;
}

static void
Format_x16b_encode (xtensa_insnbuf insn)
{
  insn[0] = 0xc;
  insn[1] = 0;
}

static void
Format_ae_format_encode (xtensa_insnbuf insn)
{
  insn[0] = 0xf;
  insn[1] = 0;
}

static int Format_x24_slots[] = { 0 };

static int Format_x16a_slots[] = { 1 };

static int Format_x16b_slots[] = { 2 };

static int Format_ae_format_slots[] = { 4, 3 };

static xtensa_format_internal formats[] = {
  { "x24", 3, Format_x24_encode, 1, Format_x24_slots },
  { "x16a", 2, Format_x16a_encode, 1, Format_x16a_slots },
  { "x16b", 2, Format_x16b_encode, 1, Format_x16b_slots },
  { "ae_format", 8, Format_ae_format_encode, 2, Format_ae_format_slots }
};


static int
format_decoder (const xtensa_insnbuf insn)
{
  if ((insn[0] & 0x8) == 0 && (insn[1] & 0) == 0)
    return 0; /* x24 */
  if ((insn[0] & 0xc) == 0x8 && (insn[1] & 0) == 0)
    return 1; /* x16a */
  if ((insn[0] & 0xe) == 0xc && (insn[1] & 0) == 0)
    return 2; /* x16b */
  if ((insn[0] & 0xf) == 0xf && (insn[1] & 0xffc00000) == 0)
    return 3; /* ae_format */
  return -1;
}

static int length_table[16] = {
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  2,
  2,
  2,
  2,
  2,
  2,
  -1,
  8
};

static int
length_decoder (const unsigned char *insn)
{
  int op0 = insn[0] & 0xf;
  return length_table[op0];
}


/* Top-level ISA structure.  */

xtensa_isa_internal xtensa_modules = {
  0 /* little-endian */,
  8 /* insn_size */, 0,
  4, formats, format_decoder, length_decoder,
  5, slots,
  387 /* num_fields */,
  445, operands,
  588, iclasses,
  656, opcodes, 0,
  8, regfiles,
  NUM_STATES, states, 0,
  NUM_SYSREGS, sysregs, 0,
  { MAX_SPECIAL_REG, MAX_USER_REG }, { 0, 0 },
  2, interfaces, 0,
  4, funcUnits, 0
};
