/*
 *  PowerPC emulation helpers header for qemu.
 * 
 *  Copyright (c) 2003-2007 Jocelyn Mayer
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#if defined(MEMSUFFIX)

/* Memory load/store helpers */
void glue(do_lsw, MEMSUFFIX) (int dst);
void glue(do_lsw_le, MEMSUFFIX) (int dst);
void glue(do_stsw, MEMSUFFIX) (int src);
void glue(do_stsw_le, MEMSUFFIX) (int src);
void glue(do_lmw, MEMSUFFIX) (int dst);
void glue(do_lmw_le, MEMSUFFIX) (int dst);
void glue(do_stmw, MEMSUFFIX) (int src);
void glue(do_stmw_le, MEMSUFFIX) (int src);
void glue(do_POWER_lscbx, MEMSUFFIX) (int dest, int ra, int rb);
void glue(do_POWER2_lfq, MEMSUFFIX) (void);
void glue(do_POWER2_lfq_le, MEMSUFFIX) (void);
void glue(do_POWER2_stfq, MEMSUFFIX) (void);
void glue(do_POWER2_stfq_le, MEMSUFFIX) (void);

#else

/* Registers load and stores */
void do_load_cr (void);
void do_store_cr (uint32_t mask);
void do_load_xer (void);
void do_store_xer (void);
void do_load_fpscr (void);
void do_store_fpscr (uint32_t mask);

/* Integer arithmetic helpers */
void do_addo (void);
void do_addco (void);
void do_adde (void);
void do_addeo (void);
void do_addmeo (void);
void do_addzeo (void);
void do_divwo (void);
void do_divwuo (void);
void do_mullwo (void);
void do_nego (void);
void do_subfo (void);
void do_subfco (void);
void do_subfe (void);
void do_subfeo (void);
void do_subfmeo (void);
void do_subfzeo (void);
void do_sraw(void);

/* Floating-point arithmetic helpers */
void do_fsqrt (void);
void do_fres (void);
void do_frsqrte (void);
void do_fsel (void);
void do_fnmadd (void);
void do_fnmsub (void);
void do_fctiw (void);
void do_fctiwz (void);
void do_fcmpu (void);
void do_fcmpo (void);

void do_tw (int flags);
void do_icbi (void);

#if !defined(CONFIG_USER_ONLY)
void do_rfi (void);
void do_tlbia (void);
void do_tlbie (void);
void do_load_6xx_tlb (int is_code);
#endif

/* POWER / PowerPC 601 specific helpers */
void do_store_601_batu (int nr);
void do_POWER_abso (void);
void do_POWER_clcs (void);
void do_POWER_div (void);
void do_POWER_divo (void);
void do_POWER_divs (void);
void do_POWER_divso (void);
void do_POWER_dozo (void);
void do_POWER_maskg (void);
void do_POWER_mulo (void);
#if !defined(CONFIG_USER_ONLY)
void do_POWER_rac (void);
void do_POWER_rfsvc (void);
#endif

/* PowerPC 602 specific helper */
#if !defined(CONFIG_USER_ONLY)
void do_op_602_mfrom (void);
#endif

/* PowerPC 4xx specific helpers */
void do_405_check_ov (void);
void do_405_check_sat (void);
#if !defined(CONFIG_USER_ONLY)
void do_4xx_load_dcr (int dcrn);
void do_4xx_store_dcr (int dcrn);
void do_4xx_rfci (void);
void do_4xx_tlbre_lo (void);
void do_4xx_tlbre_hi (void);
void do_4xx_tlbsx (void);
void do_4xx_tlbsx_ (void);
void do_4xx_tlbwe_lo (void);
void do_4xx_tlbwe_hi (void);
#endif

void do_440_dlmzb (void);

#if !defined(CONFIG_USER_ONLY)
void do_load_403_pb (int num);
void do_store_403_pb (int num);
#endif

#endif
