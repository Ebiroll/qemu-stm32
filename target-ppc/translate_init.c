/*
 *  PowerPC CPU initialization for qemu.
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

/* A lot of PowerPC definition have been included here.
 * Most of them are not usable for now but have been kept
 * inside "#if defined(TODO) ... #endif" statements to make tests easier.
 */

#include "dis-asm.h"

//#define PPC_DUMP_CPU
//#define PPC_DEBUG_SPR
//#define PPC_DEBUG_IRQ

struct ppc_def_t {
    const unsigned char *name;
    uint32_t pvr;
    uint32_t pvr_mask;
    uint64_t insns_flags;
    uint64_t msr_mask;
    uint8_t mmu_model;
    uint8_t excp_model;
    uint8_t bus_model;
    uint8_t pad;
    int bfd_mach;
    void (*init_proc)(CPUPPCState *env);
};

/* For user-mode emulation, we don't emulate any IRQ controller */
#if defined(CONFIG_USER_ONLY)
#define PPC_IRQ_INIT_FN(name)                                                 \
static inline void glue(glue(ppc, name),_irq_init) (CPUPPCState *env)         \
{                                                                             \
}
#else
#define PPC_IRQ_INIT_FN(name)                                                 \
void glue(glue(ppc, name),_irq_init) (CPUPPCState *env);
#endif

PPC_IRQ_INIT_FN(40x);
PPC_IRQ_INIT_FN(6xx);
PPC_IRQ_INIT_FN(970);

/* Generic callbacks:
 * do nothing but store/retrieve spr value
 */
#ifdef PPC_DUMP_SPR_ACCESSES
static void spr_read_generic (void *opaque, int sprn)
{
    gen_op_load_dump_spr(sprn);
}

static void spr_write_generic (void *opaque, int sprn)
{
    gen_op_store_dump_spr(sprn);
}
#else
static void spr_read_generic (void *opaque, int sprn)
{
    gen_op_load_spr(sprn);
}

static void spr_write_generic (void *opaque, int sprn)
{
    gen_op_store_spr(sprn);
}
#endif

#if !defined(CONFIG_USER_ONLY)
static void spr_write_clear (void *opaque, int sprn)
{
    gen_op_mask_spr(sprn);
}
#endif

/* SPR common to all PowerPC */
/* XER */
static void spr_read_xer (void *opaque, int sprn)
{
    gen_op_load_xer();
}

static void spr_write_xer (void *opaque, int sprn)
{
    gen_op_store_xer();
}

/* LR */
static void spr_read_lr (void *opaque, int sprn)
{
    gen_op_load_lr();
}

static void spr_write_lr (void *opaque, int sprn)
{
    gen_op_store_lr();
}

/* CTR */
static void spr_read_ctr (void *opaque, int sprn)
{
    gen_op_load_ctr();
}

static void spr_write_ctr (void *opaque, int sprn)
{
    gen_op_store_ctr();
}

/* User read access to SPR */
/* USPRx */
/* UMMCRx */
/* UPMCx */
/* USIA */
/* UDECR */
static void spr_read_ureg (void *opaque, int sprn)
{
    gen_op_load_spr(sprn + 0x10);
}

/* SPR common to all non-embedded PowerPC */
/* DECR */
#if !defined(CONFIG_USER_ONLY)
static void spr_read_decr (void *opaque, int sprn)
{
    gen_op_load_decr();
}

static void spr_write_decr (void *opaque, int sprn)
{
    gen_op_store_decr();
}
#endif

/* SPR common to all non-embedded PowerPC, except 601 */
/* Time base */
static void spr_read_tbl (void *opaque, int sprn)
{
    gen_op_load_tbl();
}

static void spr_read_tbu (void *opaque, int sprn)
{
    gen_op_load_tbu();
}

__attribute__ (( unused ))
static void spr_read_atbl (void *opaque, int sprn)
{
    gen_op_load_atbl();
}

__attribute__ (( unused ))
static void spr_read_atbu (void *opaque, int sprn)
{
    gen_op_load_atbu();
}

#if !defined(CONFIG_USER_ONLY)
static void spr_write_tbl (void *opaque, int sprn)
{
    gen_op_store_tbl();
}

static void spr_write_tbu (void *opaque, int sprn)
{
    gen_op_store_tbu();
}

__attribute__ (( unused ))
static void spr_write_atbl (void *opaque, int sprn)
{
    gen_op_store_atbl();
}

__attribute__ (( unused ))
static void spr_write_atbu (void *opaque, int sprn)
{
    gen_op_store_atbu();
}
#endif

#if !defined(CONFIG_USER_ONLY)
/* IBAT0U...IBAT0U */
/* IBAT0L...IBAT7L */
static void spr_read_ibat (void *opaque, int sprn)
{
    gen_op_load_ibat(sprn & 1, (sprn - SPR_IBAT0U) / 2);
}

static void spr_read_ibat_h (void *opaque, int sprn)
{
    gen_op_load_ibat(sprn & 1, (sprn - SPR_IBAT4U) / 2);
}

static void spr_write_ibatu (void *opaque, int sprn)
{
    gen_op_store_ibatu((sprn - SPR_IBAT0U) / 2);
}

static void spr_write_ibatu_h (void *opaque, int sprn)
{
    gen_op_store_ibatu((sprn - SPR_IBAT4U) / 2);
}

static void spr_write_ibatl (void *opaque, int sprn)
{
    gen_op_store_ibatl((sprn - SPR_IBAT0L) / 2);
}

static void spr_write_ibatl_h (void *opaque, int sprn)
{
    gen_op_store_ibatl((sprn - SPR_IBAT4L) / 2);
}

/* DBAT0U...DBAT7U */
/* DBAT0L...DBAT7L */
static void spr_read_dbat (void *opaque, int sprn)
{
    gen_op_load_dbat(sprn & 1, (sprn - SPR_DBAT0U) / 2);
}

static void spr_read_dbat_h (void *opaque, int sprn)
{
    gen_op_load_dbat(sprn & 1, (sprn - SPR_DBAT4U) / 2);
}

static void spr_write_dbatu (void *opaque, int sprn)
{
    gen_op_store_dbatu((sprn - SPR_DBAT0U) / 2);
}

static void spr_write_dbatu_h (void *opaque, int sprn)
{
    gen_op_store_dbatu((sprn - SPR_DBAT4U) / 2);
}

static void spr_write_dbatl (void *opaque, int sprn)
{
    gen_op_store_dbatl((sprn - SPR_DBAT0L) / 2);
}

static void spr_write_dbatl_h (void *opaque, int sprn)
{
    gen_op_store_dbatl((sprn - SPR_DBAT4L) / 2);
}

/* SDR1 */
static void spr_read_sdr1 (void *opaque, int sprn)
{
    gen_op_load_sdr1();
}

static void spr_write_sdr1 (void *opaque, int sprn)
{
    gen_op_store_sdr1();
}

/* 64 bits PowerPC specific SPRs */
/* ASR */
/* Currently unused */
#if 0 && defined(TARGET_PPC64)
static void spr_read_asr (void *opaque, int sprn)
{
    gen_op_load_asr();
}

static void spr_write_asr (void *opaque, int sprn)
{
    DisasContext *ctx = opaque;

    gen_op_store_asr();
}
#endif
#endif

/* PowerPC 601 specific registers */
/* RTC */
static void spr_read_601_rtcl (void *opaque, int sprn)
{
    gen_op_load_601_rtcl();
}

static void spr_read_601_rtcu (void *opaque, int sprn)
{
    gen_op_load_601_rtcu();
}

#if !defined(CONFIG_USER_ONLY)
static void spr_write_601_rtcu (void *opaque, int sprn)
{
    gen_op_store_601_rtcu();
}

static void spr_write_601_rtcl (void *opaque, int sprn)
{
    gen_op_store_601_rtcl();
}
#endif

/* Unified bats */
#if !defined(CONFIG_USER_ONLY)
static void spr_read_601_ubat (void *opaque, int sprn)
{
    gen_op_load_601_bat(sprn & 1, (sprn - SPR_IBAT0U) / 2);
}

static void spr_write_601_ubatu (void *opaque, int sprn)
{
    gen_op_store_601_batu((sprn - SPR_IBAT0U) / 2);
}

static void spr_write_601_ubatl (void *opaque, int sprn)
{
    gen_op_store_601_batl((sprn - SPR_IBAT0L) / 2);
}
#endif

/* PowerPC 40x specific registers */
#if !defined(CONFIG_USER_ONLY)
static void spr_read_40x_pit (void *opaque, int sprn)
{
    gen_op_load_40x_pit();
}

static void spr_write_40x_pit (void *opaque, int sprn)
{
    gen_op_store_40x_pit();
}

static void spr_write_40x_dbcr0 (void *opaque, int sprn)
{
    DisasContext *ctx = opaque;

    gen_op_store_40x_dbcr0();
    /* We must stop translation as we may have rebooted */
    GEN_STOP(ctx);
}

static void spr_write_40x_sler (void *opaque, int sprn)
{
    gen_op_store_40x_sler();
}

static void spr_write_booke_tcr (void *opaque, int sprn)
{
    gen_op_store_booke_tcr();
}

static void spr_write_booke_tsr (void *opaque, int sprn)
{
    gen_op_store_booke_tsr();
}
#endif

/* PowerPC 403 specific registers */
/* PBL1 / PBU1 / PBL2 / PBU2 */
#if !defined(CONFIG_USER_ONLY)
static void spr_read_403_pbr (void *opaque, int sprn)
{
    gen_op_load_403_pb(sprn - SPR_403_PBL1);
}

static void spr_write_403_pbr (void *opaque, int sprn)
{
    gen_op_store_403_pb(sprn - SPR_403_PBL1);
}

static void spr_write_pir (void *opaque, int sprn)
{
    gen_op_store_pir();
}
#endif

#if !defined(CONFIG_USER_ONLY)
/* Callback used to write the exception vector base */
static void spr_write_excp_prefix (void *opaque, int sprn)
{
    gen_op_store_excp_prefix();
    gen_op_store_spr(sprn);
}

static void spr_write_excp_vector (void *opaque, int sprn)
{
    DisasContext *ctx = opaque;

    if (sprn >= SPR_BOOKE_IVOR0 && sprn <= SPR_BOOKE_IVOR15) {
        gen_op_store_excp_vector(sprn - SPR_BOOKE_IVOR0);
        gen_op_store_spr(sprn);
    } else if (sprn >= SPR_BOOKE_IVOR32 && sprn <= SPR_BOOKE_IVOR37) {
        gen_op_store_excp_vector(sprn - SPR_BOOKE_IVOR32 + 32);
        gen_op_store_spr(sprn);
    } else {
        printf("Trying to write an unknown exception vector %d %03x\n",
               sprn, sprn);
        GEN_EXCP_PRIVREG(ctx);
    }
}
#endif

#if defined(CONFIG_USER_ONLY)
#define spr_register(env, num, name, uea_read, uea_write,                     \
                     oea_read, oea_write, initial_value)                      \
do {                                                                          \
     _spr_register(env, num, name, uea_read, uea_write, initial_value);       \
} while (0)
static inline void _spr_register (CPUPPCState *env, int num,
                                  const unsigned char *name,
                                  void (*uea_read)(void *opaque, int sprn),
                                  void (*uea_write)(void *opaque, int sprn),
                                  target_ulong initial_value)
#else
static inline void spr_register (CPUPPCState *env, int num,
                                 const unsigned char *name,
                                 void (*uea_read)(void *opaque, int sprn),
                                 void (*uea_write)(void *opaque, int sprn),
                                 void (*oea_read)(void *opaque, int sprn),
                                 void (*oea_write)(void *opaque, int sprn),
                                 target_ulong initial_value)
#endif
{
    ppc_spr_t *spr;

    spr = &env->spr_cb[num];
    if (spr->name != NULL ||env-> spr[num] != 0x00000000 ||
#if !defined(CONFIG_USER_ONLY)
        spr->oea_read != NULL || spr->oea_write != NULL ||
#endif
        spr->uea_read != NULL || spr->uea_write != NULL) {
        printf("Error: Trying to register SPR %d (%03x) twice !\n", num, num);
        exit(1);
    }
#if defined(PPC_DEBUG_SPR)
    printf("*** register spr %d (%03x) %s val " ADDRX "\n", num, num, name,
           initial_value);
#endif
    spr->name = name;
    spr->uea_read = uea_read;
    spr->uea_write = uea_write;
#if !defined(CONFIG_USER_ONLY)
    spr->oea_read = oea_read;
    spr->oea_write = oea_write;
#endif
    env->spr[num] = initial_value;
}

/* Generic PowerPC SPRs */
static void gen_spr_generic (CPUPPCState *env)
{
    /* Integer processing */
    spr_register(env, SPR_XER, "XER",
                 &spr_read_xer, &spr_write_xer,
                 &spr_read_xer, &spr_write_xer,
                 0x00000000);
    /* Branch contol */
    spr_register(env, SPR_LR, "LR",
                 &spr_read_lr, &spr_write_lr,
                 &spr_read_lr, &spr_write_lr,
                 0x00000000);
    spr_register(env, SPR_CTR, "CTR",
                 &spr_read_ctr, &spr_write_ctr,
                 &spr_read_ctr, &spr_write_ctr,
                 0x00000000);
    /* Interrupt processing */
    spr_register(env, SPR_SRR0, "SRR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_SRR1, "SRR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Processor control */
    spr_register(env, SPR_SPRG0, "SPRG0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_SPRG1, "SPRG1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_SPRG2, "SPRG2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_SPRG3, "SPRG3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

/* SPR common to all non-embedded PowerPC, including 601 */
static void gen_spr_ne_601 (CPUPPCState *env)
{
    /* Exception processing */
    spr_register(env, SPR_DSISR, "DSISR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_DAR, "DAR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Timer */
    spr_register(env, SPR_DECR, "DECR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_decr, &spr_write_decr,
                 0x00000000);
    /* Memory management */
    spr_register(env, SPR_SDR1, "SDR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_sdr1, &spr_write_sdr1,
                 0x00000000);
}

/* BATs 0-3 */
static void gen_low_BATs (CPUPPCState *env)
{
    spr_register(env, SPR_IBAT0U, "IBAT0U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat, &spr_write_ibatu,
                 0x00000000);
    spr_register(env, SPR_IBAT0L, "IBAT0L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat, &spr_write_ibatl,
                 0x00000000);
    spr_register(env, SPR_IBAT1U, "IBAT1U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat, &spr_write_ibatu,
                 0x00000000);
    spr_register(env, SPR_IBAT1L, "IBAT1L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat, &spr_write_ibatl,
                 0x00000000);
    spr_register(env, SPR_IBAT2U, "IBAT2U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat, &spr_write_ibatu,
                 0x00000000);
    spr_register(env, SPR_IBAT2L, "IBAT2L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat, &spr_write_ibatl,
                 0x00000000);
    spr_register(env, SPR_IBAT3U, "IBAT3U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat, &spr_write_ibatu,
                 0x00000000);
    spr_register(env, SPR_IBAT3L, "IBAT3L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat, &spr_write_ibatl,
                 0x00000000);
    spr_register(env, SPR_DBAT0U, "DBAT0U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat, &spr_write_dbatu,
                 0x00000000);
    spr_register(env, SPR_DBAT0L, "DBAT0L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat, &spr_write_dbatl,
                 0x00000000);
    spr_register(env, SPR_DBAT1U, "DBAT1U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat, &spr_write_dbatu,
                 0x00000000);
    spr_register(env, SPR_DBAT1L, "DBAT1L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat, &spr_write_dbatl,
                 0x00000000);
    spr_register(env, SPR_DBAT2U, "DBAT2U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat, &spr_write_dbatu,
                 0x00000000);
    spr_register(env, SPR_DBAT2L, "DBAT2L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat, &spr_write_dbatl,
                 0x00000000);
    spr_register(env, SPR_DBAT3U, "DBAT3U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat, &spr_write_dbatu,
                 0x00000000);
    spr_register(env, SPR_DBAT3L, "DBAT3L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat, &spr_write_dbatl,
                 0x00000000);
    env->nb_BATs += 4;
}

/* BATs 4-7 */
static void gen_high_BATs (CPUPPCState *env)
{
    spr_register(env, SPR_IBAT4U, "IBAT4U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat_h, &spr_write_ibatu_h,
                 0x00000000);
    spr_register(env, SPR_IBAT4L, "IBAT4L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat_h, &spr_write_ibatl_h,
                 0x00000000);
    spr_register(env, SPR_IBAT5U, "IBAT5U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat_h, &spr_write_ibatu_h,
                 0x00000000);
    spr_register(env, SPR_IBAT5L, "IBAT5L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat_h, &spr_write_ibatl_h,
                 0x00000000);
    spr_register(env, SPR_IBAT6U, "IBAT6U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat_h, &spr_write_ibatu_h,
                 0x00000000);
    spr_register(env, SPR_IBAT6L, "IBAT6L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat_h, &spr_write_ibatl_h,
                 0x00000000);
    spr_register(env, SPR_IBAT7U, "IBAT7U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat_h, &spr_write_ibatu_h,
                 0x00000000);
    spr_register(env, SPR_IBAT7L, "IBAT7L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_ibat_h, &spr_write_ibatl_h,
                 0x00000000);
    spr_register(env, SPR_DBAT4U, "DBAT4U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat_h, &spr_write_dbatu_h,
                 0x00000000);
    spr_register(env, SPR_DBAT4L, "DBAT4L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat_h, &spr_write_dbatl_h,
                 0x00000000);
    spr_register(env, SPR_DBAT5U, "DBAT5U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat_h, &spr_write_dbatu_h,
                 0x00000000);
    spr_register(env, SPR_DBAT5L, "DBAT5L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat_h, &spr_write_dbatl_h,
                 0x00000000);
    spr_register(env, SPR_DBAT6U, "DBAT6U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat_h, &spr_write_dbatu_h,
                 0x00000000);
    spr_register(env, SPR_DBAT6L, "DBAT6L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat_h, &spr_write_dbatl_h,
                 0x00000000);
    spr_register(env, SPR_DBAT7U, "DBAT7U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat_h, &spr_write_dbatu_h,
                 0x00000000);
    spr_register(env, SPR_DBAT7L, "DBAT7L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_dbat_h, &spr_write_dbatl_h,
                 0x00000000);
    env->nb_BATs += 4;
}

/* Generic PowerPC time base */
static void gen_tbl (CPUPPCState *env)
{
    spr_register(env, SPR_VTBL,  "TBL",
                 &spr_read_tbl, SPR_NOACCESS,
                 &spr_read_tbl, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_TBL,   "TBL",
                 SPR_NOACCESS, SPR_NOACCESS,
                 SPR_NOACCESS, &spr_write_tbl,
                 0x00000000);
    spr_register(env, SPR_VTBU,  "TBU",
                 &spr_read_tbu, SPR_NOACCESS,
                 &spr_read_tbu, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_TBU,   "TBU",
                 SPR_NOACCESS, SPR_NOACCESS,
                 SPR_NOACCESS, &spr_write_tbu,
                 0x00000000);
}

/* Softare table search registers */
static void gen_6xx_7xx_soft_tlb (CPUPPCState *env, int nb_tlbs, int nb_ways)
{
    env->nb_tlb = nb_tlbs;
    env->nb_ways = nb_ways;
    env->id_tlbs = 1;
    spr_register(env, SPR_DMISS, "DMISS",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_DCMP, "DCMP",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_HASH1, "HASH1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_HASH2, "HASH2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_IMISS, "IMISS",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_ICMP, "ICMP",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_RPA, "RPA",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

/* SPR common to MPC755 and G2 */
static void gen_spr_G2_755 (CPUPPCState *env)
{
    /* SGPRs */
    spr_register(env, SPR_SPRG4, "SPRG4",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_SPRG5, "SPRG5",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_SPRG6, "SPRG6",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_SPRG7, "SPRG7",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* External access control */
    /* XXX : not implemented */
    spr_register(env, SPR_EAR, "EAR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

/* SPR common to all 7xx PowerPC implementations */
static void gen_spr_7xx (CPUPPCState *env)
{
    /* Breakpoints */
    /* XXX : not implemented */
    spr_register(env, SPR_DABR, "DABR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_IABR, "IABR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Cache management */
    /* XXX : not implemented */
    spr_register(env, SPR_ICTC, "ICTC",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_L2CR, "L2CR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Performance monitors */
    /* XXX : not implemented */
    spr_register(env, SPR_MMCR0, "MMCR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_MMCR1, "MMCR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_PMC1, "PMC1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_PMC2, "PMC2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_PMC3, "PMC3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_PMC4, "PMC4",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_SIAR, "SIAR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_UMMCR0, "UMMCR0",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_UMMCR1, "UMMCR1",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_UPMC1, "UPMC1",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_UPMC2, "UPMC2",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_UPMC3, "UPMC3",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_UPMC4, "UPMC4",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_USIAR, "USIAR",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    /* External access control */
    /* XXX : not implemented */
    spr_register(env, SPR_EAR, "EAR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

static void gen_spr_thrm (CPUPPCState *env)
{
    /* Thermal management */
    /* XXX : not implemented */
    spr_register(env, SPR_THRM1, "THRM1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_THRM2, "THRM2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_THRM3, "THRM3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

/* SPR specific to PowerPC 604 implementation */
static void gen_spr_604 (CPUPPCState *env)
{
    /* Processor identification */
    spr_register(env, SPR_PIR, "PIR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_pir,
                 0x00000000);
    /* Breakpoints */
    /* XXX : not implemented */
    spr_register(env, SPR_IABR, "IABR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_DABR, "DABR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Performance counters */
    /* XXX : not implemented */
    spr_register(env, SPR_MMCR0, "MMCR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_MMCR1, "MMCR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_PMC1, "PMC1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_PMC2, "PMC2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_PMC3, "PMC3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_PMC4, "PMC4",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_SIAR, "SIAR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_SDA, "SDA",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    /* External access control */
    /* XXX : not implemented */
    spr_register(env, SPR_EAR, "EAR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

/* SPR specific to PowerPC 603 implementation */
static void gen_spr_603 (CPUPPCState *env)
{
    /* External access control */
    /* XXX : not implemented */
    spr_register(env, SPR_EAR, "EAR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

/* SPR specific to PowerPC G2 implementation */
static void gen_spr_G2 (CPUPPCState *env)
{
    /* Memory base address */
    /* MBAR */
    spr_register(env, SPR_MBAR, "MBAR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* System version register */
    /* SVR */
    spr_register(env, SPR_SVR, "SVR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    /* Exception processing */
    spr_register(env, SPR_BOOKE_CSRR0, "CSRR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_CSRR1, "CSRR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Breakpoints */
    /* XXX : not implemented */
    spr_register(env, SPR_DABR, "DABR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_DABR2, "DABR2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_IABR, "IABR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_IABR2, "IABR2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_IBCR, "IBCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_DBCR, "DBCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

/* SPR specific to PowerPC 602 implementation */
static void gen_spr_602 (CPUPPCState *env)
{
    /* ESA registers */
    /* XXX : not implemented */
    spr_register(env, SPR_SER, "SER",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_SEBR, "SEBR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_ESASRR, "ESASRR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Floating point status */
    /* XXX : not implemented */
    spr_register(env, SPR_SP, "SP",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_LT, "LT",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Watchdog timer */
    /* XXX : not implemented */
    spr_register(env, SPR_TCR, "TCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Interrupt base */
    spr_register(env, SPR_IBR, "IBR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_IABR, "IABR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

/* SPR specific to PowerPC 601 implementation */
static void gen_spr_601 (CPUPPCState *env)
{
    /* Multiplication/division register */
    /* MQ */
    spr_register(env, SPR_MQ, "MQ",
                 &spr_read_generic, &spr_write_generic,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* RTC registers */
    spr_register(env, SPR_601_RTCU, "RTCU",
                 SPR_NOACCESS, SPR_NOACCESS,
                 SPR_NOACCESS, &spr_write_601_rtcu,
                 0x00000000);
    spr_register(env, SPR_601_VRTCU, "RTCU",
                 &spr_read_601_rtcu, SPR_NOACCESS,
                 &spr_read_601_rtcu, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_601_RTCL, "RTCL",
                 SPR_NOACCESS, SPR_NOACCESS,
                 SPR_NOACCESS, &spr_write_601_rtcl,
                 0x00000000);
    spr_register(env, SPR_601_VRTCL, "RTCL",
                 &spr_read_601_rtcl, SPR_NOACCESS,
                 &spr_read_601_rtcl, SPR_NOACCESS,
                 0x00000000);
    /* Timer */
#if 0 /* ? */
    spr_register(env, SPR_601_UDECR, "UDECR",
                 &spr_read_decr, SPR_NOACCESS,
                 &spr_read_decr, SPR_NOACCESS,
                 0x00000000);
#endif
    /* External access control */
    /* XXX : not implemented */
    spr_register(env, SPR_EAR, "EAR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    spr_register(env, SPR_IBAT0U, "IBAT0U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_601_ubat, &spr_write_601_ubatu,
                 0x00000000);
    spr_register(env, SPR_IBAT0L, "IBAT0L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_601_ubat, &spr_write_601_ubatl,
                 0x00000000);
    spr_register(env, SPR_IBAT1U, "IBAT1U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_601_ubat, &spr_write_601_ubatu,
                 0x00000000);
    spr_register(env, SPR_IBAT1L, "IBAT1L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_601_ubat, &spr_write_601_ubatl,
                 0x00000000);
    spr_register(env, SPR_IBAT2U, "IBAT2U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_601_ubat, &spr_write_601_ubatu,
                 0x00000000);
    spr_register(env, SPR_IBAT2L, "IBAT2L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_601_ubat, &spr_write_601_ubatl,
                 0x00000000);
    spr_register(env, SPR_IBAT3U, "IBAT3U",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_601_ubat, &spr_write_601_ubatu,
                 0x00000000);
    spr_register(env, SPR_IBAT3L, "IBAT3L",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_601_ubat, &spr_write_601_ubatl,
                 0x00000000);
    env->nb_BATs = 4;
}

static void gen_spr_74xx (CPUPPCState *env)
{
    /* Processor identification */
    spr_register(env, SPR_PIR, "PIR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_pir,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_MMCR2, "MMCR2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_UMMCR2, "UMMCR2",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    /* XXX: not implemented */
    spr_register(env, SPR_BAMR, "BAMR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_UBAMR, "UBAMR",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_MSSCR0, "MSSCR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Altivec */
    spr_register(env, SPR_VRSAVE, "VRSAVE",
                 &spr_read_generic, &spr_write_generic,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

#if defined (TODO)
static void gen_l3_ctrl (CPUPPCState *env)
{
    /* L3CR */
    /* XXX : not implemented */
    spr_register(env, SPR_L3CR, "L3CR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* L3ITCR0 */
    spr_register(env, SPR_L3ITCR0, "L3ITCR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* L3ITCR1 */
    spr_register(env, SPR_L3ITCR1, "L3ITCR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* L3ITCR2 */
    spr_register(env, SPR_L3ITCR2, "L3ITCR2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* L3ITCR3 */
    spr_register(env, SPR_L3ITCR3, "L3ITCR3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* L3OHCR */
    spr_register(env, SPR_L3OHCR, "L3OHCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* L3PM */
    spr_register(env, SPR_L3PM, "L3PM",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}
#endif /* TODO */

#if defined (TODO)
static void gen_74xx_soft_tlb (CPUPPCState *env)
{
    /* XXX: TODO */
    spr_register(env, SPR_PTEHI, "PTEHI",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_PTELO, "PTELO",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_TLBMISS, "TLBMISS",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}
#endif /* TODO */

/* PowerPC BookE SPR */
static void gen_spr_BookE (CPUPPCState *env)
{
    /* Processor identification */
    spr_register(env, SPR_BOOKE_PIR, "PIR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_pir,
                 0x00000000);
    /* Interrupt processing */
    spr_register(env, SPR_BOOKE_CSRR0, "CSRR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_CSRR1, "CSRR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
#if 0
    spr_register(env, SPR_BOOKE_DSRR0, "DSRR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_DSRR1, "DSRR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
#endif
    /* Debug */
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_IAC1, "IAC1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_IAC2, "IAC2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_IAC3, "IAC3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_IAC4, "IAC4",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_DAC1, "DAC1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_DAC2, "DAC2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_DVC1, "DVC1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_DVC2, "DVC2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_DBCR0, "DBCR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_DBCR1, "DBCR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_DBCR2, "DBCR2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_DBSR, "DBSR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_clear,
                 0x00000000);
    spr_register(env, SPR_BOOKE_DEAR, "DEAR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_ESR, "ESR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVPR, "IVPR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_prefix,
                 0x00000000);
    /* Exception vectors */
    spr_register(env, SPR_BOOKE_IVOR0, "IVOR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR1, "IVOR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR2, "IVOR2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR3, "IVOR3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR4, "IVOR4",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR5, "IVOR5",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR6, "IVOR6",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR7, "IVOR7",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR8, "IVOR8",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR9, "IVOR9",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR10, "IVOR10",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR11, "IVOR11",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR12, "IVOR12",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR13, "IVOR13",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR14, "IVOR14",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR15, "IVOR15",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
#if 0
    spr_register(env, SPR_BOOKE_IVOR32, "IVOR32",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR33, "IVOR33",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR34, "IVOR34",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR35, "IVOR35",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR36, "IVOR36",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
    spr_register(env, SPR_BOOKE_IVOR37, "IVOR37",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_vector,
                 0x00000000);
#endif
    spr_register(env, SPR_BOOKE_PID, "PID",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_TCR, "TCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_booke_tcr,
                 0x00000000);
    spr_register(env, SPR_BOOKE_TSR, "TSR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_booke_tsr,
                 0x00000000);
    /* Timer */
    spr_register(env, SPR_DECR, "DECR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_decr, &spr_write_decr,
                 0x00000000);
    spr_register(env, SPR_BOOKE_DECAR, "DECAR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 SPR_NOACCESS, &spr_write_generic,
                 0x00000000);
    /* SPRGs */
    spr_register(env, SPR_USPRG0, "USPRG0",
                 &spr_read_generic, &spr_write_generic,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_SPRG4, "SPRG4",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG4, "USPRG4",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_SPRG5, "SPRG5",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG5, "USPRG5",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_SPRG6, "SPRG6",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG6, "USPRG6",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_SPRG7, "SPRG7",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG7, "USPRG7",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
}

/* FSL storage control registers */
#if defined(TODO)
static void gen_spr_BookE_FSL (CPUPPCState *env)
{
    /* TLB assist registers */
    spr_register(env, SPR_BOOKE_MAS0, "MAS0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MAS1, "MAS2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MAS2, "MAS3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MAS3, "MAS4",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MAS4, "MAS5",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MAS6, "MAS6",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MAS7, "MAS7",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    if (env->nb_pids > 1) {
        spr_register(env, SPR_BOOKE_PID1, "PID1",
                     SPR_NOACCESS, SPR_NOACCESS,
                     &spr_read_generic, &spr_write_generic,
                     0x00000000);
    }
    if (env->nb_pids > 2) {
        spr_register(env, SPR_BOOKE_PID2, "PID2",
                     SPR_NOACCESS, SPR_NOACCESS,
                     &spr_read_generic, &spr_write_generic,
                     0x00000000);
    }
    spr_register(env, SPR_BOOKE_MMUCFG, "MMUCFG",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000); /* TOFIX */
    spr_register(env, SPR_BOOKE_MMUCSR0, "MMUCSR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000); /* TOFIX */
    switch (env->nb_ways) {
    case 4:
        spr_register(env, SPR_BOOKE_TLB3CFG, "TLB3CFG",
                     SPR_NOACCESS, SPR_NOACCESS,
                     &spr_read_generic, SPR_NOACCESS,
                     0x00000000); /* TOFIX */
        /* Fallthru */
    case 3:
        spr_register(env, SPR_BOOKE_TLB2CFG, "TLB2CFG",
                     SPR_NOACCESS, SPR_NOACCESS,
                     &spr_read_generic, SPR_NOACCESS,
                     0x00000000); /* TOFIX */
        /* Fallthru */
    case 2:
        spr_register(env, SPR_BOOKE_TLB1CFG, "TLB1CFG",
                     SPR_NOACCESS, SPR_NOACCESS,
                     &spr_read_generic, SPR_NOACCESS,
                     0x00000000); /* TOFIX */
        /* Fallthru */
    case 1:
        spr_register(env, SPR_BOOKE_TLB0CFG, "TLB0CFG",
                     SPR_NOACCESS, SPR_NOACCESS,
                     &spr_read_generic, SPR_NOACCESS,
                     0x00000000); /* TOFIX */
        /* Fallthru */
    case 0:
    default:
        break;
    }
}
#endif

/* SPR specific to PowerPC 440 implementation */
static void gen_spr_440 (CPUPPCState *env)
{
    /* Cache control */
    /* XXX : not implemented */
    spr_register(env, SPR_440_DNV0, "DNV0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_DNV1, "DNV1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_DNV2, "DNV2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_DNV3, "DNV3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_DTV0, "DTV0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_DTV1, "DTV1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_DTV2, "DTV2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_DTV3, "DTV3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_DVLIM, "DVLIM",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_INV0, "INV0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_INV1, "INV1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_INV2, "INV2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_INV3, "INV3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_ITV0, "ITV0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_ITV1, "ITV1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_ITV2, "ITV2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_ITV3, "ITV3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_IVLIM, "IVLIM",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Cache debug */
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_DCDBTRH, "DCDBTRH",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_DCDBTRL, "DCDBTRL",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_ICDBDR, "ICDBDR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_ICDBTRH, "ICDBTRH",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_ICDBTRL, "ICDBTRL",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_440_DBDR, "DBDR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Processor control */
    spr_register(env, SPR_4xx_CCR0, "CCR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_440_RSTCFG, "RSTCFG",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    /* Storage control */
    spr_register(env, SPR_440_MMUCR, "MMUCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

/* SPR shared between PowerPC 40x implementations */
static void gen_spr_40x (CPUPPCState *env)
{
    /* Cache */
    /* not emulated, as Qemu do not emulate caches */
    spr_register(env, SPR_40x_DCCR, "DCCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* not emulated, as Qemu do not emulate caches */
    spr_register(env, SPR_40x_ICCR, "ICCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_BOOKE_ICDBDR, "ICDBDR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 0x00000000);
    /* Exception */
    spr_register(env, SPR_40x_DEAR, "DEAR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_40x_ESR, "ESR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_40x_EVPR, "EVPR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_excp_prefix,
                 0x00000000);
    spr_register(env, SPR_40x_SRR2, "SRR2",
                 &spr_read_generic, &spr_write_generic,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_40x_SRR3, "SRR3",
                 &spr_read_generic, &spr_write_generic,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Timers */
    spr_register(env, SPR_40x_PIT, "PIT",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_40x_pit, &spr_write_40x_pit,
                 0x00000000);
    spr_register(env, SPR_40x_TCR, "TCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_booke_tcr,
                 0x00000000);
    spr_register(env, SPR_40x_TSR, "TSR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_booke_tsr,
                 0x00000000);
}

/* SPR specific to PowerPC 405 implementation */
static void gen_spr_405 (CPUPPCState *env)
{
    /* MMU */
    spr_register(env, SPR_40x_PID, "PID",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_4xx_CCR0, "CCR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00700000);
    /* Debug interface */
    /* XXX : not implemented */
    spr_register(env, SPR_40x_DBCR0, "DBCR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_40x_dbcr0,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_405_DBCR1, "DBCR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_40x_DBSR, "DBSR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_clear,
                 /* Last reset was system reset */
                 0x00000300);
    /* XXX : not implemented */
    spr_register(env, SPR_40x_DAC1, "DAC1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_40x_DAC2, "DAC2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_405_DVC1, "DVC1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_405_DVC2, "DVC2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_40x_IAC1, "IAC1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_40x_IAC2, "IAC2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_405_IAC3, "IAC3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_405_IAC4, "IAC4",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Storage control */
    /* XXX: TODO: not implemented */
    spr_register(env, SPR_405_SLER, "SLER",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_40x_sler,
                 0x00000000);
    spr_register(env, SPR_40x_ZPR, "ZPR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_405_SU0R, "SU0R",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* SPRG */
    spr_register(env, SPR_USPRG0, "USPRG0",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_SPRG4, "SPRG4",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG4, "USPRG4",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_SPRG5, "SPRG5",
                 SPR_NOACCESS, SPR_NOACCESS,
                 spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG5, "USPRG5",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_SPRG6, "SPRG6",
                 SPR_NOACCESS, SPR_NOACCESS,
                 spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG6, "USPRG6",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_SPRG7, "SPRG7",
                 SPR_NOACCESS, SPR_NOACCESS,
                 spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG7, "USPRG7",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
}

/* SPR shared between PowerPC 401 & 403 implementations */
static void gen_spr_401_403 (CPUPPCState *env)
{
    /* Time base */
    spr_register(env, SPR_403_VTBL,  "TBL",
                 &spr_read_tbl, SPR_NOACCESS,
                 &spr_read_tbl, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_403_TBL,   "TBL",
                 SPR_NOACCESS, SPR_NOACCESS,
                 SPR_NOACCESS, &spr_write_tbl,
                 0x00000000);
    spr_register(env, SPR_403_VTBU,  "TBU",
                 &spr_read_tbu, SPR_NOACCESS,
                 &spr_read_tbu, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_403_TBU,   "TBU",
                 SPR_NOACCESS, SPR_NOACCESS,
                 SPR_NOACCESS, &spr_write_tbu,
                 0x00000000);
    /* Debug */
    /* XXX: not implemented */
    spr_register(env, SPR_403_CDBCR, "CDBCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

/* SPR specific to PowerPC 401 implementation */
static void gen_spr_401 (CPUPPCState *env)
{
    /* Debug interface */
    /* XXX : not implemented */
    spr_register(env, SPR_40x_DBCR0, "DBCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_40x_dbcr0,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_40x_DBSR, "DBSR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_clear,
                 /* Last reset was system reset */
                 0x00000300);
    /* XXX : not implemented */
    spr_register(env, SPR_40x_DAC1, "DAC",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_40x_IAC1, "IAC",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Storage control */
    /* XXX: TODO: not implemented */
    spr_register(env, SPR_405_SLER, "SLER",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_40x_sler,
                 0x00000000);
    /* not emulated, as Qemu never does speculative access */
    spr_register(env, SPR_40x_SGR, "SGR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0xFFFFFFFF);
    /* not emulated, as Qemu do not emulate caches */
    spr_register(env, SPR_40x_DCWR, "DCWR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

static void gen_spr_401x2 (CPUPPCState *env)
{
    gen_spr_401(env);
    spr_register(env, SPR_40x_PID, "PID",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_40x_ZPR, "ZPR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

/* SPR specific to PowerPC 403 implementation */
static void gen_spr_403 (CPUPPCState *env)
{
    /* Debug interface */
    /* XXX : not implemented */
    spr_register(env, SPR_40x_DBCR0, "DBCR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_40x_dbcr0,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_40x_DBSR, "DBSR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_clear,
                 /* Last reset was system reset */
                 0x00000300);
    /* XXX : not implemented */
    spr_register(env, SPR_40x_DAC1, "DAC1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_40x_DAC2, "DAC2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_40x_IAC1, "IAC1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_40x_IAC2, "IAC2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

static void gen_spr_403_real (CPUPPCState *env)
{
    spr_register(env, SPR_403_PBL1,  "PBL1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_403_pbr, &spr_write_403_pbr,
                 0x00000000);
    spr_register(env, SPR_403_PBU1,  "PBU1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_403_pbr, &spr_write_403_pbr,
                 0x00000000);
    spr_register(env, SPR_403_PBL2,  "PBL2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_403_pbr, &spr_write_403_pbr,
                 0x00000000);
    spr_register(env, SPR_403_PBU2,  "PBU2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_403_pbr, &spr_write_403_pbr,
                 0x00000000);
}

static void gen_spr_403_mmu (CPUPPCState *env)
{
    /* MMU */
    spr_register(env, SPR_40x_PID, "PID",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_40x_ZPR, "ZPR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

/* SPR specific to PowerPC compression coprocessor extension */
static void gen_spr_compress (CPUPPCState *env)
{
    spr_register(env, SPR_401_SKR, "SKR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}

#if defined (TARGET_PPC64)
#if defined (TODO)
/* SPR specific to PowerPC 620 */
static void gen_spr_620 (CPUPPCState *env)
{
    spr_register(env, SPR_620_PMR0, "PMR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMR1, "PMR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMR2, "PMR2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMR3, "PMR3",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMR4, "PMR4",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMR5, "PMR5",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMR6, "PMR6",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMR7, "PMR7",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMR8, "PMR8",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMR9, "PMR9",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMRA, "PMR10",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMRB, "PMR11",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMRC, "PMR12",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMRD, "PMR13",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMRE, "PMR14",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_PMRF, "PMR15",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_HID8, "HID8",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_620_HID9, "HID9",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
}
#endif
#endif /* defined (TARGET_PPC64) */

// XXX: TODO
/*
 * AMR     => SPR 29 (Power 2.04)
 * CTRL    => SPR 136 (Power 2.04)
 * CTRL    => SPR 152 (Power 2.04)
 * SCOMC   => SPR 276 (64 bits ?)
 * SCOMD   => SPR 277 (64 bits ?)
 * ASR     => SPR 280 (64 bits)
 * TBU40   => SPR 286 (Power 2.04 hypv)
 * HSPRG0  => SPR 304 (Power 2.04 hypv)
 * HSPRG1  => SPR 305 (Power 2.04 hypv)
 * HDSISR  => SPR 306 (Power 2.04 hypv)
 * HDAR    => SPR 307 (Power 2.04 hypv)
 * PURR    => SPR 309 (Power 2.04 hypv)
 * HDEC    => SPR 310 (Power 2.04 hypv)
 * HIOR    => SPR 311 (hypv)
 * RMOR    => SPR 312 (970)
 * HRMOR   => SPR 313 (Power 2.04 hypv)
 * HSRR0   => SPR 314 (Power 2.04 hypv)
 * HSRR1   => SPR 315 (Power 2.04 hypv)
 * LPCR    => SPR 316 (970)
 * LPIDR   => SPR 317 (970)
 * SPEFSCR => SPR 512 (Power 2.04 emb)
 * ATBL    => SPR 526 (Power 2.04 emb)
 * ATBU    => SPR 527 (Power 2.04 emb)
 * EPR     => SPR 702 (Power 2.04 emb)
 * perf    => 768-783 (Power 2.04)
 * perf    => 784-799 (Power 2.04)
 * PPR     => SPR 896 (Power 2.04)
 * EPLC    => SPR 947 (Power 2.04 emb)
 * EPSC    => SPR 948 (Power 2.04 emb)
 * DABRX   => 1015    (Power 2.04 hypv)
 * FPECR   => SPR 1022 (?)
 * ... and more (thermal management, performance counters, ...)
 */

/*****************************************************************************/
/* Exception vectors models                                                  */
static void init_excp_4xx_real (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_CRITICAL] = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_PIT]      = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_FIT]      = 0x00001010;
    env->excp_vectors[POWERPC_EXCP_WDT]      = 0x00001020;
    env->excp_vectors[POWERPC_EXCP_DEBUG]    = 0x00002000;
    env->excp_prefix = 0x00000000;
    env->ivor_mask = 0x0000FFF0;
    env->ivpr_mask = 0xFFFF0000;
#endif
}

static void init_excp_4xx_softmmu (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_CRITICAL] = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_PIT]      = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_FIT]      = 0x00001010;
    env->excp_vectors[POWERPC_EXCP_WDT]      = 0x00001020;
    env->excp_vectors[POWERPC_EXCP_DTLB]     = 0x00001100;
    env->excp_vectors[POWERPC_EXCP_ITLB]     = 0x00001200;
    env->excp_vectors[POWERPC_EXCP_DEBUG]    = 0x00002000;
    env->excp_prefix = 0x00000000;
    env->ivor_mask = 0x0000FFF0;
    env->ivpr_mask = 0xFFFF0000;
#endif
}

static void init_excp_BookE (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_CRITICAL] = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_APU]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_FIT]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_WDT]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_DTLB]     = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_ITLB]     = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_DEBUG]    = 0x00000000;
    env->excp_prefix = 0x00000000;
    env->ivor_mask = 0x0000FFE0;
    env->ivpr_mask = 0xFFFF0000;
#endif
}

static void init_excp_601 (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_IO]       = 0x00000A00;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_RUNM]     = 0x00002000;
    env->excp_prefix = 0xFFF00000;
#endif
}

static void init_excp_602 (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_FPA]      = 0x00000E00;
    env->excp_vectors[POWERPC_EXCP_IFTLB]    = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_DLTLB]    = 0x00001100;
    env->excp_vectors[POWERPC_EXCP_DSTLB]    = 0x00001200;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
    env->excp_vectors[POWERPC_EXCP_WDT]      = 0x00001500;
    env->excp_vectors[POWERPC_EXCP_EMUL]     = 0x00001600;
    env->excp_prefix = 0xFFF00000;
#endif
}

static void init_excp_603 (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_IFTLB]    = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_DLTLB]    = 0x00001100;
    env->excp_vectors[POWERPC_EXCP_DSTLB]    = 0x00001200;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
#endif
}

static void init_excp_G2 (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_CRITICAL] = 0x00000A00;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_IFTLB]    = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_DLTLB]    = 0x00001100;
    env->excp_vectors[POWERPC_EXCP_DSTLB]    = 0x00001200;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
#endif
}

static void init_excp_604 (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
#endif
}

#if defined (TODO)
static void init_excp_620 (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_FPA]      = 0x00000E00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
#endif
}
#endif /* defined (TODO) */

static void init_excp_7x0 (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_THERM]    = 0x00001700;
#endif
}

static void init_excp_750FX (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
    env->excp_vectors[POWERPC_EXCP_THERM]    = 0x00001700;
#endif
}

static void init_excp_7400 (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_VPU]      = 0x00000F20;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
    env->excp_vectors[POWERPC_EXCP_VPUA]     = 0x00001600;
    env->excp_vectors[POWERPC_EXCP_THERM]    = 0x00001700;
#endif
}

#if defined (TODO)
static void init_excp_7450 (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_VPU]      = 0x00000F20;
    env->excp_vectors[POWERPC_EXCP_IFTLB]    = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_DLTLB]    = 0x00001100;
    env->excp_vectors[POWERPC_EXCP_DSTLB]    = 0x00001200;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
    env->excp_vectors[POWERPC_EXCP_VPUA]     = 0x00001600;
#endif
}
#endif /* defined (TODO) */

#if defined (TARGET_PPC64)
static void init_excp_970 (CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_DSEG]     = 0x00000380;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_ISEG]     = 0x00000480;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
#if defined(TARGET_PPC64H) /* PowerPC 64 with hypervisor mode support */
    env->excp_vectors[POWERPC_EXCP_HDECR]    = 0x00000980;
#endif
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_VPU]      = 0x00000F20;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_MAINT]    = 0x00001600;
    env->excp_vectors[POWERPC_EXCP_VPUA]     = 0x00001700;
    env->excp_vectors[POWERPC_EXCP_THERM]    = 0x00001800;
#endif
}
#endif

/*****************************************************************************/
/* PowerPC implementations definitions                                       */

/* PowerPC 40x instruction set                                               */
#define POWERPC_INSNS_EMB    (PPC_INSNS_BASE | PPC_EMB_COMMON)

/* PowerPC 401                                                               */
#define POWERPC_INSNS_401    (POWERPC_INSNS_EMB |                             \
                              PPC_MEM_SYNC | PPC_MEM_EIEIO |                  \
                              PPC_4xx_COMMON | PPC_40x_EXCP | PPC_40x_ICBT)
#define POWERPC_MSRM_401     (0x00000000000FD201ULL)
#define POWERPC_MMU_401      (POWERPC_MMU_REAL_4xx)
#define POWERPC_EXCP_401     (POWERPC_EXCP_40x)
#define POWERPC_INPUT_401    (PPC_FLAGS_INPUT_401)
#define POWERPC_BFDM_401     (bfd_mach_ppc_403)

static void init_proc_401 (CPUPPCState *env)
{
    gen_spr_40x(env);
    gen_spr_401_403(env);
    gen_spr_401(env);
    init_excp_4xx_real(env);
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env);
}

/* PowerPC 401x2                                                             */
#define POWERPC_INSNS_401x2  (POWERPC_INSNS_EMB |                             \
                              PPC_MEM_SYNC | PPC_MEM_EIEIO |                  \
                              PPC_40x_TLB | PPC_MEM_TLBIA | PPC_MEM_TLBSYNC | \
                              PPC_CACHE_DCBA | PPC_MFTB |                     \
                              PPC_4xx_COMMON | PPC_40x_EXCP | PPC_40x_ICBT)
#define POWERPC_MSRM_401x2   (0x00000000001FD231ULL)
#define POWERPC_MMU_401x2    (POWERPC_MMU_SOFT_4xx_Z)
#define POWERPC_EXCP_401x2   (POWERPC_EXCP_40x)
#define POWERPC_INPUT_401x2  (PPC_FLAGS_INPUT_401)
#define POWERPC_BFDM_401x2   (bfd_mach_ppc_403)

static void init_proc_401x2 (CPUPPCState *env)
{
    gen_spr_40x(env);
    gen_spr_401_403(env);
    gen_spr_401x2(env);
    gen_spr_compress(env);
    /* Memory management */
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    init_excp_4xx_softmmu(env);
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env);
}

/* PowerPC 401x3                                                             */
#if defined(TODO)
#define POWERPC_INSNS_401x3  (POWERPC_INSNS_EMB |                             \
                              PPC_MEM_SYNC | PPC_MEM_EIEIO |                  \
                              PPC_40x_TLB | PPC_MEM_TLBIA | PPC_MEM_TLBSYNC | \
                              PPC_CACHE_DCBA | PPC_MFTB |                     \
                              PPC_4xx_COMMON | PPC_40x_EXCP | PPC_40x_ICBT)
#define POWERPC_MSRM_401x3   (0x00000000001FD631ULL)
#define POWERPC_MMU_401x3    (POWERPC_MMU_SOFT_4xx_Z)
#define POWERPC_EXCP_401x3   (POWERPC_EXCP_40x)
#define POWERPC_INPUT_401x3  (PPC_FLAGS_INPUT_401)
#define POWERPC_BFDM_401x3   (bfd_mach_ppc_403)

static void init_proc_401x3 (CPUPPCState *env)
{
    gen_spr_40x(env);
    gen_spr_401_403(env);
    gen_spr_401(env);
    gen_spr_401x2(env);
    gen_spr_compress(env);
    init_excp_4xx_softmmu(env);
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env);
}
#endif /* TODO */

/* IOP480                                                                    */
#define POWERPC_INSNS_IOP480 (POWERPC_INSNS_EMB |                             \
                              PPC_MEM_SYNC | PPC_MEM_EIEIO |                  \
                              PPC_40x_TLB | PPC_MEM_TLBIA | PPC_MEM_TLBSYNC | \
                              PPC_CACHE_DCBA |                                \
                              PPC_4xx_COMMON | PPC_40x_EXCP |  PPC_40x_ICBT)
#define POWERPC_MSRM_IOP480  (0x00000000001FD231ULL)
#define POWERPC_MMU_IOP480   (POWERPC_MMU_SOFT_4xx_Z)
#define POWERPC_EXCP_IOP480  (POWERPC_EXCP_40x)
#define POWERPC_INPUT_IOP480 (PPC_FLAGS_INPUT_401)
#define POWERPC_BFDM_IOP480  (bfd_mach_ppc_403)

static void init_proc_IOP480 (CPUPPCState *env)
{
    gen_spr_40x(env);
    gen_spr_401_403(env);
    gen_spr_401x2(env);
    gen_spr_compress(env);
    /* Memory management */
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    init_excp_4xx_softmmu(env);
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env);
}

/* PowerPC 403                                                               */
#define POWERPC_INSNS_403    (POWERPC_INSNS_EMB |                             \
                              PPC_MEM_SYNC | PPC_MEM_EIEIO |                  \
                              PPC_40x_TLB | PPC_MEM_TLBIA | PPC_MEM_TLBSYNC | \
                              PPC_4xx_COMMON | PPC_40x_EXCP | PPC_40x_ICBT)
#define POWERPC_MSRM_403     (0x000000000007D00DULL)
#define POWERPC_MMU_403      (POWERPC_MMU_REAL_4xx)
#define POWERPC_EXCP_403     (POWERPC_EXCP_40x)
#define POWERPC_INPUT_403    (PPC_FLAGS_INPUT_401)
#define POWERPC_BFDM_403     (bfd_mach_ppc_403)

static void init_proc_403 (CPUPPCState *env)
{
    gen_spr_40x(env);
    gen_spr_401_403(env);
    gen_spr_403(env);
    gen_spr_403_real(env);
    init_excp_4xx_real(env);
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env);
}

/* PowerPC 403 GCX                                                           */
#define POWERPC_INSNS_403GCX (POWERPC_INSNS_EMB |                             \
                              PPC_MEM_SYNC | PPC_MEM_EIEIO |                  \
                              PPC_40x_TLB | PPC_MEM_TLBIA | PPC_MEM_TLBSYNC | \
                              PPC_4xx_COMMON | PPC_40x_EXCP | PPC_40x_ICBT)
#define POWERPC_MSRM_403GCX  (0x000000000007D00DULL)
#define POWERPC_MMU_403GCX   (POWERPC_MMU_SOFT_4xx_Z)
#define POWERPC_EXCP_403GCX  (POWERPC_EXCP_40x)
#define POWERPC_INPUT_403GCX (PPC_FLAGS_INPUT_401)
#define POWERPC_BFDM_403GCX  (bfd_mach_ppc_403)

static void init_proc_403GCX (CPUPPCState *env)
{
    gen_spr_40x(env);
    gen_spr_401_403(env);
    gen_spr_403(env);
    gen_spr_403_real(env);
    gen_spr_403_mmu(env);
    /* Bus access control */
    /* not emulated, as Qemu never does speculative access */
    spr_register(env, SPR_40x_SGR, "SGR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0xFFFFFFFF);
    /* not emulated, as Qemu do not emulate caches */
    spr_register(env, SPR_40x_DCWR, "DCWR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    init_excp_4xx_softmmu(env);
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env);
}

/* PowerPC 405                                                               */
#define POWERPC_INSNS_405    (POWERPC_INSNS_EMB | PPC_MFTB |                  \
                              PPC_MEM_SYNC | PPC_MEM_EIEIO | PPC_CACHE_DCBA | \
                              PPC_40x_TLB | PPC_MEM_TLBIA | PPC_MEM_TLBSYNC | \
                              PPC_4xx_COMMON | PPC_40x_EXCP | PPC_40x_ICBT |  \
                              PPC_405_MAC)
#define POWERPC_MSRM_405     (0x000000000006E630ULL)
#define POWERPC_MMU_405      (POWERPC_MMU_SOFT_4xx)
#define POWERPC_EXCP_405     (POWERPC_EXCP_40x)
#define POWERPC_INPUT_405    (PPC_FLAGS_INPUT_405)
#define POWERPC_BFDM_405     (bfd_mach_ppc_403)

static void init_proc_405 (CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_40x(env);
    gen_spr_405(env);
    /* Bus access control */
    /* not emulated, as Qemu never does speculative access */
    spr_register(env, SPR_40x_SGR, "SGR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0xFFFFFFFF);
    /* not emulated, as Qemu do not emulate caches */
    spr_register(env, SPR_40x_DCWR, "DCWR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    init_excp_4xx_softmmu(env);
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env);
}

/* PowerPC 440 EP                                                            */
#define POWERPC_INSNS_440EP  (POWERPC_INSNS_EMB |                             \
                              PPC_CACHE_DCBA | PPC_MEM_TLBSYNC |              \
                              PPC_BOOKE | PPC_4xx_COMMON | PPC_405_MAC |      \
                              PPC_440_SPEC | PPC_RFMCI)
#define POWERPC_MSRM_440EP   (0x000000000006D630ULL)
#define POWERPC_MMU_440EP    (POWERPC_MMU_BOOKE)
#define POWERPC_EXCP_440EP   (POWERPC_EXCP_BOOKE)
#define POWERPC_INPUT_440EP  (PPC_FLAGS_INPUT_BookE)
#define POWERPC_BFDM_440EP   (bfd_mach_ppc_403)

static void init_proc_440EP (CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_BookE(env);
    gen_spr_440(env);
    spr_register(env, SPR_BOOKE_MCSR, "MCSR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MCSRR0, "MCSRR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MCSRR1, "MCSRR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_440_CCR1, "CCR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    init_excp_BookE(env);
    /* XXX: TODO: allocate internal IRQ controller */
}

/* PowerPC 440 GP                                                            */
#define POWERPC_INSNS_440GP  (POWERPC_INSNS_EMB |                             \
                              PPC_CACHE_DCBA | PPC_MEM_TLBSYNC |              \
                              PPC_BOOKE | PPC_BOOKE_EXT | PPC_4xx_COMMON |    \
                              PPC_405_MAC | PPC_440_SPEC)
#define POWERPC_MSRM_440GP   (0x000000000006FF30ULL)
#define POWERPC_MMU_440GP    (POWERPC_MMU_BOOKE)
#define POWERPC_EXCP_440GP   (POWERPC_EXCP_BOOKE)
#define POWERPC_INPUT_440GP  (PPC_FLAGS_INPUT_BookE)
#define POWERPC_BFDM_440GP   (bfd_mach_ppc_403)

static void init_proc_440GP (CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_BookE(env);
    gen_spr_440(env);
    /* Memory management */
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    init_excp_BookE(env);
    /* XXX: TODO: allocate internal IRQ controller */
}

/* PowerPC 440x4                                                             */
#if defined(TODO)
#define POWERPC_INSNS_440x4  (POWERPC_INSNS_EMB |                             \
                              PPC_CACHE_DCBA | PPC_MEM_TLBSYNC |              \
                              PPC_BOOKE | PPC_4xx_COMMON | PPC_405_MAC |      \
                              PPC_440_SPEC)
#define POWERPC_MSRM_440x4   (0x000000000006FF30ULL)
#define POWERPC_MMU_440x4    (POWERPC_MMU_BOOKE)
#define POWERPC_EXCP_440x4   (POWERPC_EXCP_BOOKE)
#define POWERPC_INPUT_440x4  (PPC_FLAGS_INPUT_BookE)
#define POWERPC_BFDM_440x4   (bfd_mach_ppc_403)

static void init_proc_440x4 (CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_BookE(env);
    gen_spr_440(env);
    /* Memory management */
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    init_excp_BookE(env);
    /* XXX: TODO: allocate internal IRQ controller */
}
#endif /* TODO */

/* PowerPC 440x5                                                             */
#define POWERPC_INSNS_440x5  (POWERPC_INSNS_EMB |                             \
                              PPC_CACHE_DCBA | PPC_MEM_TLBSYNC |              \
                              PPC_BOOKE | PPC_4xx_COMMON | PPC_405_MAC |      \
                              PPC_440_SPEC | PPC_RFMCI)
#define POWERPC_MSRM_440x5   (0x000000000006FF30ULL)
#define POWERPC_MMU_440x5    (POWERPC_MMU_BOOKE)
#define POWERPC_EXCP_440x5   (POWERPC_EXCP_BOOKE)
#define POWERPC_INPUT_440x5  (PPC_FLAGS_INPUT_BookE)
#define POWERPC_BFDM_440x5   (bfd_mach_ppc_403)

static void init_proc_440x5 (CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_BookE(env);
    gen_spr_440(env);
    spr_register(env, SPR_BOOKE_MCSR, "MCSR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MCSRR0, "MCSRR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MCSRR1, "MCSRR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_440_CCR1, "CCR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    init_excp_BookE(env);
    /* XXX: TODO: allocate internal IRQ controller */
}

/* PowerPC 460 (guessed)                                                     */
#if defined(TODO)
#define POWERPC_INSNS_460F   (POWERPC_INSNS_EMB |                             \
                              PPC_CACHE_DCBA | PPC_MEM_TLBSYNC |              \
                              PPC_BOOKE | PPC_BOOKE_EXT | PPC_4xx_COMMON |    \
                              PPC_405_MAC | PPC_440_SPEC | PPC_DCRUX)
#define POWERPC_MSRM_460     (0x000000000006FF30ULL)
#define POWERPC_MMU_460      (POWERPC_MMU_BOOKE)
#define POWERPC_EXCP_460     (POWERPC_EXCP_BOOKE)
#define POWERPC_INPUT_460    (PPC_FLAGS_INPUT_BookE)
#define POWERPC_BFDM_460     (bfd_mach_ppc_403)

static void init_proc_460 (CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_BookE(env);
    gen_spr_440(env);
    spr_register(env, SPR_BOOKE_MCSR, "MCSR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MCSRR0, "MCSRR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MCSRR1, "MCSRR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_440_CCR1, "CCR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_DCRIPR, "SPR_DCRIPR",
                 &spr_read_generic, &spr_write_generic,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    init_excp_BookE(env);
    /* XXX: TODO: allocate internal IRQ controller */
}
#endif /* TODO */

/* PowerPC 460F (guessed)                                                    */
#if defined(TODO)
#define POWERPC_INSNS_460F   (POWERPC_INSNS_EMB |                             \
                              PPC_CACHE_DCBA | PPC_MEM_TLBSYNC |              \
                              PPC_FLOAT | PPC_FLOAT_FSQRT | PPC_FLOAT_FRES |  \
                              PPC_FLOAT_FRSQRTE | PPC_FLOAT_FSEL |            \
                              PPC_FLOAT_STFIWX |                              \
                              PPC_BOOKE | PPC_BOOKE_EXT | PPC_4xx_COMMON |    \
                              PPC_405_MAC | PPC_440_SPEC | PPC_DCRUX)
#define POWERPC_MSRM_460     (0x000000000006FF30ULL)
#define POWERPC_MMU_460F     (POWERPC_MMU_BOOKE)
#define POWERPC_EXCP_460F    (POWERPC_EXCP_BOOKE)
#define POWERPC_INPUT_460F   (PPC_FLAGS_INPUT_BookE)
#define POWERPC_BFDM_460F    (bfd_mach_ppc_403)

static void init_proc_460F (CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_BookE(env);
    gen_spr_440(env);
    spr_register(env, SPR_BOOKE_MCSR, "MCSR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MCSRR0, "MCSRR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_BOOKE_MCSRR1, "MCSRR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_440_CCR1, "CCR1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_DCRIPR, "SPR_DCRIPR",
                 &spr_read_generic, &spr_write_generic,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    init_excp_BookE(env);
    /* XXX: TODO: allocate internal IRQ controller */
}
#endif /* TODO */

/* Generic BookE PowerPC                                                     */
#if defined(TODO)
#define POWERPC_INSNS_BookE  (POWERPC_INSNS_EMB |                             \
                              PPC_MEM_EIEIO | PPC_MEM_TLBSYNC |               \
                              PPC_CACHE_DCBA |                                \
                              PPC_FLOAT | PPC_FLOAT_FSQRT |                   \
                              PPC_FLOAT_FRES | PPC_FLOAT_FRSQRTE |            \
                              PPC_FLOAT_FSEL | PPC_FLOAT_STFIW |              \
                              PPC_BOOKE)
#define POWERPC_MSRM_BookE   (0x000000000006D630ULL)
#define POWERPC_MMU_BookE    (POWERPC_MMU_BOOKE)
#define POWERPC_EXCP_BookE   (POWERPC_EXCP_BOOKE)
#define POWERPC_INPUT_BookE  (PPC_FLAGS_INPUT_BookE)
#define POWERPC_BFDM_BookE   (bfd_mach_ppc_403)

static void init_proc_BookE (CPUPPCState *env)
{
    init_excp_BookE(env);
}
#endif /* TODO */

/* e200 core                                                                 */
#if defined(TODO)
#endif /* TODO */

/* e300 core                                                                 */
#if defined(TODO)
#endif /* TODO */

/* e500 core                                                                 */
#if defined(TODO)
#define POWERPC_INSNS_e500   (POWERPC_INSNS_EMB |                             \
                              PPC_MEM_EIEIO | PPC_MEM_TLBSYNC |               \
                              PPC_CACHE_DCBA |                                \
                              PPC_BOOKE | PPC_E500_VECTOR)
#define POWERPC_MMU_e500     (POWERPC_MMU_SOFT_4xx)
#define POWERPC_EXCP_e500    (POWERPC_EXCP_40x)
#define POWERPC_INPUT_e500   (PPC_FLAGS_INPUT_BookE)
#define POWERPC_BFDM_e500    (bfd_mach_ppc_403)

static void init_proc_e500 (CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_BookE(env);
    /* Memory management */
    gen_spr_BookE_FSL(env);
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    init_excp_BookE(env);
    /* XXX: TODO: allocate internal IRQ controller */
}
#endif /* TODO */

/* e600 core                                                                 */
#if defined(TODO)
#endif /* TODO */

/* Non-embedded PowerPC                                                      */
/* Base instructions set for all 6xx/7xx/74xx/970 PowerPC                    */
#define POWERPC_INSNS_6xx    (PPC_INSNS_BASE | PPC_FLOAT | PPC_MEM_SYNC |     \
                              PPC_MEM_EIEIO | PPC_SEGMENT | PPC_MEM_TLBIE)
/* Instructions common to all 6xx/7xx/74xx/970 PowerPC except 601 & 602      */
#define POWERPC_INSNS_WORKS  (POWERPC_INSNS_6xx | PPC_FLOAT_FSQRT |           \
                              PPC_FLOAT_FRES | PPC_FLOAT_FRSQRTE |            \
                              PPC_FLOAT_FSEL | PPC_FLOAT_STFIWX |             \
                              PPC_MEM_TLBSYNC | PPC_MFTB)

/* POWER : same as 601, without mfmsr, mfsr                                  */
#if defined(TODO)
#define POWERPC_INSNS_POWER  (XXX_TODO)
/* POWER RSC (from RAD6000) */
#define POWERPC_MSRM_POWER   (0x00000000FEF0ULL)
#endif /* TODO */

/* PowerPC 601                                                               */
#define POWERPC_INSNS_601    (POWERPC_INSNS_6xx | PPC_EXTERN | PPC_POWER_BR)
#define POWERPC_MSRM_601     (0x000000000000FE70ULL)
//#define POWERPC_MMU_601      (POWERPC_MMU_601)
//#define POWERPC_EXCP_601     (POWERPC_EXCP_601)
#define POWERPC_INPUT_601    (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_601     (bfd_mach_ppc_601)

static void init_proc_601 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_601(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_601_HID2, "HID2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_601_HID5, "HID5",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_601_HID15, "HID15",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    env->nb_tlb = 64;
    env->nb_ways = 2;
    env->id_tlbs = 0;
    env->id_tlbs = 0;
    init_excp_601(env);
    /* XXX: TODO: allocate internal IRQ controller */
}

/* PowerPC 602                                                               */
#define POWERPC_INSNS_602    (POWERPC_INSNS_6xx | PPC_MFTB |                  \
                              PPC_FLOAT_FRES | PPC_FLOAT_FRSQRTE |            \
                              PPC_FLOAT_FSEL | PPC_FLOAT_STFIWX |             \
                              PPC_6xx_TLB | PPC_MEM_TLBSYNC | PPC_602_SPEC)
#define POWERPC_MSRM_602     (0x000000000033FF73ULL)
#define POWERPC_MMU_602      (POWERPC_MMU_SOFT_6xx)
//#define POWERPC_EXCP_602     (POWERPC_EXCP_602)
#define POWERPC_INPUT_602    (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_602     (bfd_mach_ppc_602)

static void init_proc_602 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_602(env);
    /* Time base */
    gen_tbl(env);
    /* hardware implementation registers */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    init_excp_602(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}

/* PowerPC 603                                                               */
#define POWERPC_INSNS_603    (POWERPC_INSNS_WORKS | PPC_6xx_TLB | PPC_EXTERN)
#define POWERPC_MSRM_603     (0x000000000001FF73ULL)
#define POWERPC_MMU_603      (POWERPC_MMU_SOFT_6xx)
//#define POWERPC_EXCP_603     (POWERPC_EXCP_603)
#define POWERPC_INPUT_603    (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_603     (bfd_mach_ppc_603)

static void init_proc_603 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_603(env);
    /* Time base */
    gen_tbl(env);
    /* hardware implementation registers */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    init_excp_603(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}

/* PowerPC 603e                                                              */
#define POWERPC_INSNS_603E   (POWERPC_INSNS_WORKS | PPC_6xx_TLB | PPC_EXTERN)
#define POWERPC_MSRM_603E    (0x000000000007FF73ULL)
#define POWERPC_MMU_603E     (POWERPC_MMU_SOFT_6xx)
//#define POWERPC_EXCP_603E    (POWERPC_EXCP_603E)
#define POWERPC_INPUT_603E   (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_603E    (bfd_mach_ppc_ec603e)

static void init_proc_603E (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_603(env);
    /* Time base */
    gen_tbl(env);
    /* hardware implementation registers */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_IABR, "IABR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    init_excp_603(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}

/* PowerPC G2                                                                */
#define POWERPC_INSNS_G2     (POWERPC_INSNS_WORKS | PPC_6xx_TLB | PPC_EXTERN)
#define POWERPC_MSRM_G2      (0x000000000006FFF2ULL)
#define POWERPC_MMU_G2       (POWERPC_MMU_SOFT_6xx)
//#define POWERPC_EXCP_G2      (POWERPC_EXCP_G2)
#define POWERPC_INPUT_G2     (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_G2      (bfd_mach_ppc_ec603e)

static void init_proc_G2 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_G2_755(env);
    gen_spr_G2(env);
    /* Time base */
    gen_tbl(env);
    /* Hardware implementation register */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID2, "HID2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    init_excp_G2(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}

/* PowerPC G2LE                                                              */
#define POWERPC_INSNS_G2LE   (POWERPC_INSNS_WORKS | PPC_6xx_TLB | PPC_EXTERN)
#define POWERPC_MSRM_G2LE    (0x000000000007FFF3ULL)
#define POWERPC_MMU_G2LE     (POWERPC_MMU_SOFT_6xx)
#define POWERPC_EXCP_G2LE    (POWERPC_EXCP_G2)
#define POWERPC_INPUT_G2LE   (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_G2LE    (bfd_mach_ppc_ec603e)

static void init_proc_G2LE (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_G2_755(env);
    gen_spr_G2(env);
    /* Time base */
    gen_tbl(env);
    /* Hardware implementation register */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID2, "HID2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    init_excp_G2(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}

/* PowerPC 604                                                               */
#define POWERPC_INSNS_604    (POWERPC_INSNS_WORKS | PPC_EXTERN)
#define POWERPC_MSRM_604     (0x000000000005FF77ULL)
#define POWERPC_MMU_604      (POWERPC_MMU_32B)
//#define POWERPC_EXCP_604     (POWERPC_EXCP_604)
#define POWERPC_INPUT_604    (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_604     (bfd_mach_ppc_604)

static void init_proc_604 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_604(env);
    /* Time base */
    gen_tbl(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    init_excp_604(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}

/* PowerPC 740/750 (aka G3)                                                  */
#define POWERPC_INSNS_7x0    (POWERPC_INSNS_WORKS | PPC_EXTERN)
#define POWERPC_MSRM_7x0     (0x000000000007FF77ULL)
#define POWERPC_MMU_7x0      (POWERPC_MMU_32B)
//#define POWERPC_EXCP_7x0     (POWERPC_EXCP_7x0)
#define POWERPC_INPUT_7x0    (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_7x0     (bfd_mach_ppc_750)

static void init_proc_7x0 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* Thermal management */
    gen_spr_thrm(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    init_excp_7x0(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}

/* PowerPC 750FX/GX                                                          */
#define POWERPC_INSNS_750fx  (POWERPC_INSNS_WORKS | PPC_EXTERN)
#define POWERPC_MSRM_750fx   (0x000000000007FF77ULL)
#define POWERPC_MMU_750fx    (POWERPC_MMU_32B)
#define POWERPC_EXCP_750fx   (POWERPC_EXCP_7x0)
#define POWERPC_INPUT_750fx  (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_750fx   (bfd_mach_ppc_750)

static void init_proc_750fx (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* Thermal management */
    gen_spr_thrm(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_750_HID2, "HID2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    /* PowerPC 750fx & 750gx has 8 DBATs and 8 IBATs */
    gen_high_BATs(env);
    init_excp_750FX(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}

/* PowerPC 745/755                                                           */
#define POWERPC_INSNS_7x5    (POWERPC_INSNS_WORKS | PPC_EXTERN | PPC_6xx_TLB)
#define POWERPC_MSRM_7x5     (0x000000000007FF77ULL)
#define POWERPC_MMU_7x5      (POWERPC_MMU_SOFT_6xx)
//#define POWERPC_EXCP_7x5     (POWERPC_EXCP_7x5)
#define POWERPC_INPUT_7x5    (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_7x5     (bfd_mach_ppc_750)

static void init_proc_7x5 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_G2_755(env);
    /* Time base */
    gen_tbl(env);
    /* L2 cache control */
    /* XXX : not implemented */
    spr_register(env, SPR_ICTC, "ICTC",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_L2PMCR, "L2PMCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID2, "HID2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}

/* PowerPC 7400 (aka G4)                                                     */
#define POWERPC_INSNS_7400   (POWERPC_INSNS_WORKS | PPC_CACHE_DCBA |          \
                              PPC_EXTERN | PPC_MEM_TLBIA |                    \
                              PPC_ALTIVEC)
#define POWERPC_MSRM_7400    (0x000000000205FF77ULL)
#define POWERPC_MMU_7400     (POWERPC_MMU_32B)
#define POWERPC_EXCP_7400    (POWERPC_EXCP_74xx)
#define POWERPC_INPUT_7400   (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_7400    (bfd_mach_ppc_7400)

static void init_proc_7400 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    /* Thermal management */
    gen_spr_thrm(env);
    /* Memory management */
    gen_low_BATs(env);
    init_excp_7400(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}

/* PowerPC 7410 (aka G4)                                                     */
#define POWERPC_INSNS_7410   (POWERPC_INSNS_WORKS | PPC_CACHE_DCBA |          \
                              PPC_EXTERN | PPC_MEM_TLBIA |                    \
                              PPC_ALTIVEC)
#define POWERPC_MSRM_7410    (0x000000000205FF77ULL)
#define POWERPC_MMU_7410     (POWERPC_MMU_32B)
#define POWERPC_EXCP_7410    (POWERPC_EXCP_74xx)
#define POWERPC_INPUT_7410   (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_7410    (bfd_mach_ppc_7400)

static void init_proc_7410 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    /* Thermal management */
    gen_spr_thrm(env);
    /* L2PMCR */
    /* XXX : not implemented */
    spr_register(env, SPR_L2PMCR, "L2PMCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* LDSTDB */
    /* XXX : not implemented */
    spr_register(env, SPR_LDSTDB, "LDSTDB",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    init_excp_7400(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}

/* PowerPC 7440 (aka G4)                                                     */
#if defined (TODO)
#define POWERPC_INSNS_7440   (POWERPC_INSNS_WORKS | PPC_CACHE_DCBA |          \
                              PPC_EXTERN | PPC_74xx_TLB | PPC_MEM_TLBIA |     \
                              PPC_ALTIVEC)
#define POWERPC_MSRM_7440    (0x000000000205FF77ULL)
#define POWERPC_MMU_7440     (POWERPC_MMU_SOFT_74xx)
#define POWERPC_EXCP_7440    (POWERPC_EXCP_74xx)
#define POWERPC_INPUT_7440   (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_7440    (bfd_mach_ppc_7400)

static void init_proc_7440 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    /* LDSTCR */
    /* XXX : not implemented */
    spr_register(env, SPR_LDSTCR, "LDSTCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* ICTRL */
    /* XXX : not implemented */
    spr_register(env, SPR_ICTRL, "ICTRL",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* MSSSR0 */
    spr_register(env, SPR_MSSSR0, "MSSSR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* PMC */
    /* XXX : not implemented */
    spr_register(env, SPR_PMC5, "PMC5",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_UPMC5, "UPMC5",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_PMC6, "PMC6",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_UPMC6, "UPMC6",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    gen_74xx_soft_tlb(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}
#endif /* TODO */

/* PowerPC 7450 (aka G4)                                                     */
#if defined (TODO)
#define POWERPC_INSNS_7450   (POWERPC_INSNS_WORKS | PPC_CACHE_DCBA |          \
                              PPC_EXTERN | PPC_74xx_TLB | PPC_MEM_TLBIA |     \
                              PPC_ALTIVEC)
#define POWERPC_MSRM_7450    (0x000000000205FF77ULL)
#define POWERPC_MMU_7450     (POWERPC_MMU_SOFT_74xx)
#define POWERPC_EXCP_7450    (POWERPC_EXCP_74xx)
#define POWERPC_INPUT_7450   (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_7450    (bfd_mach_ppc_7400)

static void init_proc_7450 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    /* Level 3 cache control */
    gen_l3_ctrl(env);
    /* LDSTCR */
    /* XXX : not implemented */
    spr_register(env, SPR_LDSTCR, "LDSTCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* ICTRL */
    /* XXX : not implemented */
    spr_register(env, SPR_ICTRL, "ICTRL",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* MSSSR0 */
    spr_register(env, SPR_MSSSR0, "MSSSR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* PMC */
    /* XXX : not implemented */
    spr_register(env, SPR_PMC5, "PMC5",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_UPMC5, "UPMC5",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_PMC6, "PMC6",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_UPMC6, "UPMC6",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    gen_74xx_soft_tlb(env);
    init_excp_7450(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}
#endif /* TODO */

/* PowerPC 7445 (aka G4)                                                     */
#if defined (TODO)
#define POWERPC_INSNS_7445   (POWERPC_INSNS_WORKS | PPC_CACHE_DCBA |          \
                              PPC_EXTERN | PPC_74xx_TLB | PPC_MEM_TLBIA |     \
                              PPC_ALTIVEC)
#define POWERPC_MSRM_7445    (0x000000000205FF77ULL)
#define POWERPC_MMU_7445     (POWERPC_MMU_SOFT_74xx)
#define POWERPC_EXCP_7445    (POWERPC_EXCP_74xx)
#define POWERPC_INPUT_7445   (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_7445    (bfd_mach_ppc_7400)

static void init_proc_7445 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    /* LDSTCR */
    /* XXX : not implemented */
    spr_register(env, SPR_LDSTCR, "LDSTCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* ICTRL */
    /* XXX : not implemented */
    spr_register(env, SPR_ICTRL, "ICTRL",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* MSSSR0 */
    spr_register(env, SPR_MSSSR0, "MSSSR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* PMC */
    /* XXX : not implemented */
    spr_register(env, SPR_PMC5, "PMC5",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_UPMC5, "UPMC5",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_PMC6, "PMC6",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_UPMC6, "UPMC6",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    /* SPRGs */
    spr_register(env, SPR_SPRG4, "SPRG4",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG4, "USPRG4",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_SPRG5, "SPRG5",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG5, "USPRG5",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_SPRG6, "SPRG6",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG6, "USPRG6",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_SPRG7, "SPRG7",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG7, "USPRG7",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_74xx_soft_tlb(env);
    init_excp_7450(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}
#endif /* TODO */

/* PowerPC 7455 (aka G4)                                                     */
#if defined (TODO)
#define POWERPC_INSNS_7455   (POWERPC_INSNS_WORKS | PPC_CACHE_DCBA |          \
                              PPC_EXTERN | PPC_74xx_TLB | PPC_MEM_TLBIA |     \
                              PPC_ALTIVEC)
#define POWERPC_MSRM_7455    (0x000000000205FF77ULL)
#define POWERPC_MMU_7455     (POWERPC_MMU_SOFT_74xx)
#define POWERPC_EXCP_7455    (POWERPC_EXCP_74xx)
#define POWERPC_INPUT_7455   (PPC_FLAGS_INPUT_6xx)
#define POWERPC_BFDM_7455    (bfd_mach_ppc_7400)

static void init_proc_7455 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    /* Level 3 cache control */
    gen_l3_ctrl(env);
    /* LDSTCR */
    /* XXX : not implemented */
    spr_register(env, SPR_LDSTCR, "LDSTCR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* ICTRL */
    /* XXX : not implemented */
    spr_register(env, SPR_ICTRL, "ICTRL",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* MSSSR0 */
    spr_register(env, SPR_MSSSR0, "MSSSR0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* PMC */
    /* XXX : not implemented */
    spr_register(env, SPR_PMC5, "PMC5",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_UPMC5, "UPMC5",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_PMC6, "PMC6",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_UPMC6, "UPMC6",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    /* SPRGs */
    spr_register(env, SPR_SPRG4, "SPRG4",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG4, "USPRG4",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_SPRG5, "SPRG5",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG5, "USPRG5",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_SPRG6, "SPRG6",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG6, "USPRG6",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    spr_register(env, SPR_SPRG7, "SPRG7",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    spr_register(env, SPR_USPRG7, "USPRG7",
                 &spr_read_ureg, SPR_NOACCESS,
                 &spr_read_ureg, SPR_NOACCESS,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_74xx_soft_tlb(env);
    init_excp_7450(env);
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env);
}
#endif /* TODO */

#if defined (TARGET_PPC64)
/* PowerPC 970                                                               */
#define POWERPC_INSNS_970    (POWERPC_INSNS_WORKS | PPC_FLOAT_FSQRT |         \
                              PPC_64B | PPC_ALTIVEC |                         \
                              PPC_64_BRIDGE | PPC_SLBI)
#define POWERPC_MSRM_970     (0x900000000204FF36ULL)
#define POWERPC_MMU_970      (POWERPC_MMU_64BRIDGE)
//#define POWERPC_EXCP_970     (POWERPC_EXCP_970)
#define POWERPC_INPUT_970    (PPC_FLAGS_INPUT_970)
#define POWERPC_BFDM_970     (bfd_mach_ppc64)

static void init_proc_970 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_750_HID2, "HID2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    /* XXX: not correct */
    gen_low_BATs(env);
#if 0 // TODO
    env->slb_nr = 32;
#endif
    init_excp_970(env);
    /* Allocate hardware IRQ controller */
    ppc970_irq_init(env);
}

/* PowerPC 970FX (aka G5)                                                    */
#define POWERPC_INSNS_970FX  (POWERPC_INSNS_WORKS | PPC_FLOAT_FSQRT |         \
                              PPC_64B | PPC_ALTIVEC |                         \
                              PPC_64_BRIDGE | PPC_SLBI)
#define POWERPC_MSRM_970FX   (0x800000000204FF36ULL)
#define POWERPC_MMU_970FX    (POWERPC_MMU_64BRIDGE)
#define POWERPC_EXCP_970FX   (POWERPC_EXCP_970)
#define POWERPC_INPUT_970FX  (PPC_FLAGS_INPUT_970)
#define POWERPC_BFDM_970FX   (bfd_mach_ppc64)

static void init_proc_970FX (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_750_HID2, "HID2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    /* XXX: not correct */
    gen_low_BATs(env);
#if 0 // TODO
    env->slb_nr = 32;
#endif
    init_excp_970(env);
    /* Allocate hardware IRQ controller */
    ppc970_irq_init(env);
}

/* PowerPC 970 GX                                                            */
#define POWERPC_INSNS_970GX  (POWERPC_INSNS_WORKS | PPC_FLOAT_FSQRT |         \
                              PPC_64B | PPC_ALTIVEC |                         \
                              PPC_64_BRIDGE | PPC_SLBI)
#define POWERPC_MSRM_970GX   (0x800000000204FF36ULL)
#define POWERPC_MMU_970GX    (POWERPC_MMU_64BRIDGE)
#define POWERPC_EXCP_970GX   (POWERPC_EXCP_970)
#define POWERPC_INPUT_970GX  (PPC_FLAGS_INPUT_970)
#define POWERPC_BFDM_970GX   (bfd_mach_ppc64)

static void init_proc_970GX (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_HID1, "HID1",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* XXX : not implemented */
    spr_register(env, SPR_750_HID2, "HID2",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    /* XXX: not correct */
    gen_low_BATs(env);
#if 0 // TODO
    env->slb_nr = 32;
#endif
    init_excp_970(env);
    /* Allocate hardware IRQ controller */
    ppc970_irq_init(env);
}

/* PowerPC 620                                                               */
#if defined (TODO)
#define POWERPC_INSNS_620    (POWERPC_INSNS_WORKS | PPC_FLOAT_FSQRT |         \
                              PPC_64B | PPC_SLBI)
#define POWERPC_MSRM_620     (0x800000000005FF73ULL)
#define POWERPC_MMU_620      (POWERPC_MMU_64B)
#define POWERPC_EXCP_620     (POWERPC_EXCP_970)
#define POWERPC_INPUT_620    (PPC_FLAGS_INPUT_970)
#define POWERPC_BFDM_620     (bfd_mach_ppc64)

static void init_proc_620 (CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_620(env);
    /* Time base */
    gen_tbl(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    spr_register(env, SPR_HID0, "HID0",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, &spr_write_generic,
                 0x00000000);
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    init_excp_620(env);
    /* XXX: TODO: initialize internal interrupt controller */
}
#endif /* TODO */
#endif /* defined (TARGET_PPC64) */

/* Default 32 bits PowerPC target will be 604 */
#define CPU_POWERPC_PPC32     CPU_POWERPC_604
#define POWERPC_INSNS_PPC32   POWERPC_INSNS_604
#define POWERPC_MSRM_PPC32    POWERPC_MSRM_604
#define POWERPC_MMU_PPC32     POWERPC_MMU_604
#define POWERPC_EXCP_PPC32    POWERPC_EXCP_604
#define POWERPC_INPUT_PPC32   POWERPC_INPUT_604
#define init_proc_PPC32       init_proc_604
#define POWERPC_BFDM_PPC32    POWERPC_BFDM_604

/* Default 64 bits PowerPC target will be 970 FX */
#define CPU_POWERPC_PPC64     CPU_POWERPC_970FX
#define POWERPC_INSNS_PPC64   POWERPC_INSNS_970FX
#define POWERPC_MSRM_PPC64    POWERPC_MSRM_970FX
#define POWERPC_MMU_PPC64     POWERPC_MMU_970FX
#define POWERPC_EXCP_PPC64    POWERPC_EXCP_970FX
#define POWERPC_INPUT_PPC64   POWERPC_INPUT_970FX
#define init_proc_PPC64       init_proc_970FX
#define POWERPC_BFDM_PPC64    POWERPC_BFDM_970FX

/* Default PowerPC target will be PowerPC 32 */
#if defined (TARGET_PPC64) && 0 // XXX: TODO
#define CPU_POWERPC_DEFAULT   CPU_POWERPC_PPC64
#define POWERPC_INSNS_DEFAULT POWERPC_INSNS_PPC64
#define POWERPC_MSRM_DEFAULT  POWERPC_MSRM_PPC64
#define POWERPC_MMU_DEFAULT   POWERPC_MMU_PPC64
#define POWERPC_EXCP_DEFAULT  POWERPC_EXCP_PPC64
#define POWERPC_INPUT_DEFAULT POWERPC_INPUT_PPC64
#define init_proc_DEFAULT     init_proc_PPC64
#define POWERPC_BFDM_DEFAULT  POWERPC_BFDM_PPC64
#else
#define CPU_POWERPC_DEFAULT   CPU_POWERPC_PPC32
#define POWERPC_INSNS_DEFAULT POWERPC_INSNS_PPC32
#define POWERPC_MSRM_DEFAULT  POWERPC_MSRM_PPC32
#define POWERPC_MMU_DEFAULT   POWERPC_MMU_PPC32
#define POWERPC_EXCP_DEFAULT  POWERPC_EXCP_PPC32
#define POWERPC_INPUT_DEFAULT POWERPC_INPUT_PPC32
#define init_proc_DEFAULT     init_proc_PPC32
#define POWERPC_BFDM_DEFAULT  POWERPC_BFDM_PPC32
#endif

/*****************************************************************************/
/* PVR definitions for most known PowerPC                                    */
enum {
    /* PowerPC 401 family */
    /* Generic PowerPC 401 */
#define CPU_POWERPC_401       CPU_POWERPC_401G2
    /* PowerPC 401 cores */
    CPU_POWERPC_401A1       = 0x00210000,
    CPU_POWERPC_401B2       = 0x00220000,
#if 0
    CPU_POWERPC_401B3       = xxx,
#endif
    CPU_POWERPC_401C2       = 0x00230000,
    CPU_POWERPC_401D2       = 0x00240000,
    CPU_POWERPC_401E2       = 0x00250000,
    CPU_POWERPC_401F2       = 0x00260000,
    CPU_POWERPC_401G2       = 0x00270000,
    /* PowerPC 401 microcontrolers */
#if 0
    CPU_POWERPC_401GF       = xxx,
#endif
#define CPU_POWERPC_IOP480    CPU_POWERPC_401B2
    /* IBM Processor for Network Resources */
    CPU_POWERPC_COBRA       = 0x10100000, /* XXX: 405 ? */
#if 0
    CPU_POWERPC_XIPCHIP     = xxx,
#endif
    /* PowerPC 403 family */
    /* Generic PowerPC 403 */
#define CPU_POWERPC_403       CPU_POWERPC_403GC
    /* PowerPC 403 microcontrollers */
    CPU_POWERPC_403GA       = 0x00200011,
    CPU_POWERPC_403GB       = 0x00200100,
    CPU_POWERPC_403GC       = 0x00200200,
    CPU_POWERPC_403GCX      = 0x00201400,
#if 0
    CPU_POWERPC_403GP       = xxx,
#endif
    /* PowerPC 405 family */
    /* Generic PowerPC 405 */
#define CPU_POWERPC_405       CPU_POWERPC_405D4
    /* PowerPC 405 cores */
#if 0
    CPU_POWERPC_405A3       = xxx,
#endif
#if 0
    CPU_POWERPC_405A4       = xxx,
#endif
#if 0
    CPU_POWERPC_405B3       = xxx,
#endif
#if 0
    CPU_POWERPC_405B4       = xxx,
#endif
#if 0
    CPU_POWERPC_405C3       = xxx,
#endif
#if 0
    CPU_POWERPC_405C4       = xxx,
#endif
    CPU_POWERPC_405D2       = 0x20010000,
#if 0
    CPU_POWERPC_405D3       = xxx,
#endif
    CPU_POWERPC_405D4       = 0x41810000,
#if 0
    CPU_POWERPC_405D5       = xxx,
#endif
#if 0
    CPU_POWERPC_405E4       = xxx,
#endif
#if 0
    CPU_POWERPC_405F4       = xxx,
#endif
#if 0
    CPU_POWERPC_405F5       = xxx,
#endif
#if 0
    CPU_POWERPC_405F6       = xxx,
#endif
    /* PowerPC 405 microcontrolers */
    /* XXX: missing 0x200108a0 */
#define CPU_POWERPC_405CR     CPU_POWERPC_405CRc
    CPU_POWERPC_405CRa      = 0x40110041,
    CPU_POWERPC_405CRb      = 0x401100C5,
    CPU_POWERPC_405CRc      = 0x40110145,
    CPU_POWERPC_405EP       = 0x51210950,
#if 0
    CPU_POWERPC_405EXr      = xxx,
#endif
    CPU_POWERPC_405EZ       = 0x41511460, /* 0x51210950 ? */
#if 0
    CPU_POWERPC_405FX       = xxx,
#endif
#define CPU_POWERPC_405GP     CPU_POWERPC_405GPd
    CPU_POWERPC_405GPa      = 0x40110000,
    CPU_POWERPC_405GPb      = 0x40110040,
    CPU_POWERPC_405GPc      = 0x40110082,
    CPU_POWERPC_405GPd      = 0x401100C4,
#define CPU_POWERPC_405GPe    CPU_POWERPC_405CRc
    CPU_POWERPC_405GPR      = 0x50910951,
#if 0
    CPU_POWERPC_405H        = xxx,
#endif
#if 0
    CPU_POWERPC_405L        = xxx,
#endif
    CPU_POWERPC_405LP       = 0x41F10000,
#if 0
    CPU_POWERPC_405PM       = xxx,
#endif
#if 0
    CPU_POWERPC_405PS       = xxx,
#endif
#if 0
    CPU_POWERPC_405S        = xxx,
#endif
    /* IBM network processors */
    CPU_POWERPC_NPE405H     = 0x414100C0,
    CPU_POWERPC_NPE405H2    = 0x41410140,
    CPU_POWERPC_NPE405L     = 0x416100C0,
    CPU_POWERPC_NPE4GS3     = 0x40B10000,
#if 0
    CPU_POWERPC_NPCxx1      = xxx,
#endif
#if 0
    CPU_POWERPC_NPR161      = xxx,
#endif
#if 0
    CPU_POWERPC_LC77700     = xxx,
#endif
    /* IBM STBxxx (PowerPC 401/403/405 core based microcontrollers) */
#if 0
    CPU_POWERPC_STB01000    = xxx,
#endif
#if 0
    CPU_POWERPC_STB01010    = xxx,
#endif
#if 0
    CPU_POWERPC_STB0210     = xxx, /* 401B3 */
#endif
    CPU_POWERPC_STB03       = 0x40310000, /* 0x40130000 ? */
#if 0
    CPU_POWERPC_STB043      = xxx,
#endif
#if 0
    CPU_POWERPC_STB045      = xxx,
#endif
    CPU_POWERPC_STB04       = 0x41810000,
    CPU_POWERPC_STB25       = 0x51510950,
#if 0
    CPU_POWERPC_STB130      = xxx,
#endif
    /* Xilinx cores */
    CPU_POWERPC_X2VP4       = 0x20010820,
#define CPU_POWERPC_X2VP7     CPU_POWERPC_X2VP4
    CPU_POWERPC_X2VP20      = 0x20010860,
#define CPU_POWERPC_X2VP50    CPU_POWERPC_X2VP20
#if 0
    CPU_POWERPC_ZL10310     = xxx,
#endif
#if 0
    CPU_POWERPC_ZL10311     = xxx,
#endif
#if 0
    CPU_POWERPC_ZL10320     = xxx,
#endif
#if 0
    CPU_POWERPC_ZL10321     = xxx,
#endif
    /* PowerPC 440 family */
    /* Generic PowerPC 440 */
#define CPU_POWERPC_440       CPU_POWERPC_440GXf
    /* PowerPC 440 cores */
#if 0
    CPU_POWERPC_440A4       = xxx,
#endif
#if 0
    CPU_POWERPC_440A5       = xxx,
#endif
#if 0
    CPU_POWERPC_440B4       = xxx,
#endif
#if 0
    CPU_POWERPC_440F5       = xxx,
#endif
#if 0
    CPU_POWERPC_440G5       = xxx,
#endif
#if 0
    CPU_POWERPC_440H4       = xxx,
#endif
#if 0
    CPU_POWERPC_440H6       = xxx,
#endif
    /* PowerPC 440 microcontrolers */
#define CPU_POWERPC_440EP     CPU_POWERPC_440EPb
    CPU_POWERPC_440EPa      = 0x42221850,
    CPU_POWERPC_440EPb      = 0x422218D3,
#define CPU_POWERPC_440GP     CPU_POWERPC_440GPc
    CPU_POWERPC_440GPb      = 0x40120440,
    CPU_POWERPC_440GPc      = 0x40120481,
#define CPU_POWERPC_440GR     CPU_POWERPC_440GRa
#define CPU_POWERPC_440GRa    CPU_POWERPC_440EPb
    CPU_POWERPC_440GRX      = 0x200008D0,
#define CPU_POWERPC_440EPX    CPU_POWERPC_440GRX
#define CPU_POWERPC_440GX     CPU_POWERPC_440GXf
    CPU_POWERPC_440GXa      = 0x51B21850,
    CPU_POWERPC_440GXb      = 0x51B21851,
    CPU_POWERPC_440GXc      = 0x51B21892,
    CPU_POWERPC_440GXf      = 0x51B21894,
#if 0
    CPU_POWERPC_440S        = xxx,
#endif
    CPU_POWERPC_440SP       = 0x53221850,
    CPU_POWERPC_440SP2      = 0x53221891,
    CPU_POWERPC_440SPE      = 0x53421890,
    /* PowerPC 460 family */
#if 0
    /* Generic PowerPC 464 */
#define CPU_POWERPC_464       CPU_POWERPC_464H90
#endif
    /* PowerPC 464 microcontrolers */
#if 0
    CPU_POWERPC_464H90      = xxx,
#endif
#if 0
    CPU_POWERPC_464H90FP    = xxx,
#endif
    /* Freescale embedded PowerPC cores */
    /* e200 family */
#define CPU_POWERPC_e200      CPU_POWERPC_e200z6
#if 0
    CPU_POWERPC_e200z0      = xxx,
#endif
#if 0
    CPU_POWERPC_e200z3      = xxx,
#endif
    CPU_POWERPC_e200z5      = 0x81000000,
    CPU_POWERPC_e200z6      = 0x81120000,
    /* e300 family */
#define CPU_POWERPC_e300      CPU_POWERPC_e300c3
    CPU_POWERPC_e300c1      = 0x00830000,
    CPU_POWERPC_e300c2      = 0x00840000,
    CPU_POWERPC_e300c3      = 0x00850000,
    /* e500 family */
#define CPU_POWERPC_e500      CPU_POWERPC_e500_v22
    CPU_POWERPC_e500_v11    = 0x80200010,
    CPU_POWERPC_e500_v12    = 0x80200020,
    CPU_POWERPC_e500_v21    = 0x80210010,
    CPU_POWERPC_e500_v22    = 0x80210020,
#if 0
    CPU_POWERPC_e500mc      = xxx,
#endif
    /* e600 family */
    CPU_POWERPC_e600        = 0x80040010,
    /* PowerPC MPC 5xx cores */
    CPU_POWERPC_5xx         = 0x00020020,
    /* PowerPC MPC 8xx cores (aka PowerQUICC) */
    CPU_POWERPC_8xx         = 0x00500000,
    /* PowerPC MPC 8xxx cores (aka PowerQUICC-II) */
    CPU_POWERPC_82xx_HIP3   = 0x00810101,
    CPU_POWERPC_82xx_HIP4   = 0x80811014,
    CPU_POWERPC_827x        = 0x80822013,
    /* PowerPC 6xx cores */
    CPU_POWERPC_601         = 0x00010001,
    CPU_POWERPC_601a        = 0x00010002,
    CPU_POWERPC_602         = 0x00050100,
    CPU_POWERPC_603         = 0x00030100,
#define CPU_POWERPC_603E      CPU_POWERPC_603E_v41
    CPU_POWERPC_603E_v11    = 0x00060101,
    CPU_POWERPC_603E_v12    = 0x00060102,
    CPU_POWERPC_603E_v13    = 0x00060103,
    CPU_POWERPC_603E_v14    = 0x00060104,
    CPU_POWERPC_603E_v22    = 0x00060202,
    CPU_POWERPC_603E_v3     = 0x00060300,
    CPU_POWERPC_603E_v4     = 0x00060400,
    CPU_POWERPC_603E_v41    = 0x00060401,
    CPU_POWERPC_603E7t      = 0x00071201,
    CPU_POWERPC_603E7v      = 0x00070100,
    CPU_POWERPC_603E7v1     = 0x00070101,
    CPU_POWERPC_603E7v2     = 0x00070201,
    CPU_POWERPC_603E7       = 0x00070200,
    CPU_POWERPC_603P        = 0x00070000,
#define CPU_POWERPC_603R      CPU_POWERPC_603E7t
    CPU_POWERPC_G2          = 0x00810011,
#if 0 // Linux pretends the MSB is zero...
    CPU_POWERPC_G2H4        = 0x80811010,
    CPU_POWERPC_G2gp        = 0x80821010,
    CPU_POWERPC_G2ls        = 0x90810010,
    CPU_POWERPC_G2LE        = 0x80820010,
    CPU_POWERPC_G2LEgp      = 0x80822010,
    CPU_POWERPC_G2LEls      = 0xA0822010,
#else
    CPU_POWERPC_G2H4        = 0x00811010,
    CPU_POWERPC_G2gp        = 0x00821010,
    CPU_POWERPC_G2ls        = 0x10810010,
    CPU_POWERPC_G2LE        = 0x00820010,
    CPU_POWERPC_G2LEgp      = 0x00822010,
    CPU_POWERPC_G2LEls      = 0x20822010,
#endif
    CPU_POWERPC_604         = 0x00040103,
#define CPU_POWERPC_604E      CPU_POWERPC_604E_v24
    CPU_POWERPC_604E_v10    = 0x00090100, /* Also 2110 & 2120 */
    CPU_POWERPC_604E_v22    = 0x00090202,
    CPU_POWERPC_604E_v24    = 0x00090204,
    CPU_POWERPC_604R        = 0x000a0101, /* Also 0x00093102 */
#if 0
    CPU_POWERPC_604EV       = xxx,
#endif
    /* PowerPC 740/750 cores (aka G3) */
    /* XXX: missing 0x00084202 */
#define CPU_POWERPC_7x0       CPU_POWERPC_7x0_v31
    CPU_POWERPC_7x0_v20     = 0x00080200,
    CPU_POWERPC_7x0_v21     = 0x00080201,
    CPU_POWERPC_7x0_v22     = 0x00080202,
    CPU_POWERPC_7x0_v30     = 0x00080300,
    CPU_POWERPC_7x0_v31     = 0x00080301,
    CPU_POWERPC_740E        = 0x00080100,
    CPU_POWERPC_7x0P        = 0x10080000,
    /* XXX: missing 0x00087010 (CL ?) */
    CPU_POWERPC_750CL       = 0x00087200,
#define CPU_POWERPC_750CX     CPU_POWERPC_750CX_v22
    CPU_POWERPC_750CX_v21   = 0x00082201,
    CPU_POWERPC_750CX_v22   = 0x00082202,
#define CPU_POWERPC_750CXE    CPU_POWERPC_750CXE_v31b
    CPU_POWERPC_750CXE_v21  = 0x00082211,
    CPU_POWERPC_750CXE_v22  = 0x00082212,
    CPU_POWERPC_750CXE_v23  = 0x00082213,
    CPU_POWERPC_750CXE_v24  = 0x00082214,
    CPU_POWERPC_750CXE_v24b = 0x00083214,
    CPU_POWERPC_750CXE_v31  = 0x00083211,
    CPU_POWERPC_750CXE_v31b = 0x00083311,
    CPU_POWERPC_750CXR      = 0x00083410,
    CPU_POWERPC_750E        = 0x00080200,
    CPU_POWERPC_750FL       = 0x700A0203,
#define CPU_POWERPC_750FX     CPU_POWERPC_750FX_v23
    CPU_POWERPC_750FX_v10   = 0x70000100,
    CPU_POWERPC_750FX_v20   = 0x70000200,
    CPU_POWERPC_750FX_v21   = 0x70000201,
    CPU_POWERPC_750FX_v22   = 0x70000202,
    CPU_POWERPC_750FX_v23   = 0x70000203,
    CPU_POWERPC_750GL       = 0x70020102,
#define CPU_POWERPC_750GX     CPU_POWERPC_750GX_v12
    CPU_POWERPC_750GX_v10   = 0x70020100,
    CPU_POWERPC_750GX_v11   = 0x70020101,
    CPU_POWERPC_750GX_v12   = 0x70020102,
#define CPU_POWERPC_750L      CPU_POWERPC_750L_v32 /* Aka LoneStar */
    CPU_POWERPC_750L_v22    = 0x00088202,
    CPU_POWERPC_750L_v30    = 0x00088300,
    CPU_POWERPC_750L_v32    = 0x00088302,
    /* PowerPC 745/755 cores */
#define CPU_POWERPC_7x5       CPU_POWERPC_7x5_v28
    CPU_POWERPC_7x5_v10     = 0x00083100,
    CPU_POWERPC_7x5_v11     = 0x00083101,
    CPU_POWERPC_7x5_v20     = 0x00083200,
    CPU_POWERPC_7x5_v21     = 0x00083201,
    CPU_POWERPC_7x5_v22     = 0x00083202, /* aka D */
    CPU_POWERPC_7x5_v23     = 0x00083203, /* aka E */
    CPU_POWERPC_7x5_v24     = 0x00083204,
    CPU_POWERPC_7x5_v25     = 0x00083205,
    CPU_POWERPC_7x5_v26     = 0x00083206,
    CPU_POWERPC_7x5_v27     = 0x00083207,
    CPU_POWERPC_7x5_v28     = 0x00083208,
#if 0
    CPU_POWERPC_7x5P        = xxx,
#endif
    /* PowerPC 74xx cores (aka G4) */
    /* XXX: missing 0x000C1101 */
#define CPU_POWERPC_7400      CPU_POWERPC_7400_v29
    CPU_POWERPC_7400_v10    = 0x000C0100,
    CPU_POWERPC_7400_v11    = 0x000C0101,
    CPU_POWERPC_7400_v20    = 0x000C0200,
    CPU_POWERPC_7400_v22    = 0x000C0202,
    CPU_POWERPC_7400_v26    = 0x000C0206,
    CPU_POWERPC_7400_v27    = 0x000C0207,
    CPU_POWERPC_7400_v28    = 0x000C0208,
    CPU_POWERPC_7400_v29    = 0x000C0209,
#define CPU_POWERPC_7410      CPU_POWERPC_7410_v14
    CPU_POWERPC_7410_v10    = 0x800C1100,
    CPU_POWERPC_7410_v11    = 0x800C1101,
    CPU_POWERPC_7410_v12    = 0x800C1102, /* aka C */
    CPU_POWERPC_7410_v13    = 0x800C1103, /* aka D */
    CPU_POWERPC_7410_v14    = 0x800C1104, /* aka E */
#define CPU_POWERPC_7448      CPU_POWERPC_7448_v21
    CPU_POWERPC_7448_v10    = 0x80040100,
    CPU_POWERPC_7448_v11    = 0x80040101,
    CPU_POWERPC_7448_v20    = 0x80040200,
    CPU_POWERPC_7448_v21    = 0x80040201,
#define CPU_POWERPC_7450      CPU_POWERPC_7450_v21
    CPU_POWERPC_7450_v10    = 0x80000100,
    CPU_POWERPC_7450_v11    = 0x80000101,
    CPU_POWERPC_7450_v12    = 0x80000102,
    CPU_POWERPC_7450_v20    = 0x80000200, /* aka D: 2.04 */
    CPU_POWERPC_7450_v21    = 0x80000201, /* aka E */
    CPU_POWERPC_74x1        = 0x80000203,
    CPU_POWERPC_74x1G       = 0x80000210, /* aka G: 2.3 */
    /* XXX: missing 0x80010200 */
#define CPU_POWERPC_74x5      CPU_POWERPC_74x5_v32
    CPU_POWERPC_74x5_v10    = 0x80010100,
    CPU_POWERPC_74x5_v21    = 0x80010201, /* aka C: 2.1 */
    CPU_POWERPC_74x5_v32    = 0x80010302,
    CPU_POWERPC_74x5_v33    = 0x80010303, /* aka F: 3.3 */
    CPU_POWERPC_74x5_v34    = 0x80010304, /* aka G: 3.4 */
#define CPU_POWERPC_74x7      CPU_POWERPC_74x7_v12
    CPU_POWERPC_74x7_v10    = 0x80020100, /* aka A: 1.0 */
    CPU_POWERPC_74x7_v11    = 0x80030101, /* aka B: 1.1 */
    CPU_POWERPC_74x7_v12    = 0x80020102, /* aka C: 1.2 */
    /* 64 bits PowerPC */
    CPU_POWERPC_620         = 0x00140000,
    CPU_POWERPC_630         = 0x00400000,
    CPU_POWERPC_631         = 0x00410104,
    CPU_POWERPC_POWER4      = 0x00350000,
    CPU_POWERPC_POWER4P     = 0x00380000,
    CPU_POWERPC_POWER5      = 0x003A0203,
#define CPU_POWERPC_POWER5GR  CPU_POWERPC_POWER5
    CPU_POWERPC_POWER5P     = 0x003B0000,
#define CPU_POWERPC_POWER5GS  CPU_POWERPC_POWER5P
    CPU_POWERPC_POWER6      = 0x003E0000,
    CPU_POWERPC_POWER6_5    = 0x0F000001, /* POWER6 running POWER5 mode */
    CPU_POWERPC_POWER6A     = 0x0F000002,
    CPU_POWERPC_970         = 0x00390202,
#define CPU_POWERPC_970FX     CPU_POWERPC_970FX_v31
    CPU_POWERPC_970FX_v10   = 0x00391100,
    CPU_POWERPC_970FX_v20   = 0x003C0200,
    CPU_POWERPC_970FX_v21   = 0x003C0201,
    CPU_POWERPC_970FX_v30   = 0x003C0300,
    CPU_POWERPC_970FX_v31   = 0x003C0301,
    CPU_POWERPC_970GX       = 0x00450000,
#define CPU_POWERPC_970MP     CPU_POWERPC_970MP_v11
    CPU_POWERPC_970MP_v10   = 0x00440100,
    CPU_POWERPC_970MP_v11   = 0x00440101,
#define CPU_POWERPC_CELL      CPU_POWERPC_CELL_v32
    CPU_POWERPC_CELL_v10    = 0x00700100,
    CPU_POWERPC_CELL_v20    = 0x00700400,
    CPU_POWERPC_CELL_v30    = 0x00700500,
    CPU_POWERPC_CELL_v31    = 0x00700501,
#define CPU_POWERPC_CELL_v32  CPU_POWERPC_CELL_v31
    CPU_POWERPC_RS64        = 0x00330000,
    CPU_POWERPC_RS64II      = 0x00340000,
    CPU_POWERPC_RS64III     = 0x00360000,
    CPU_POWERPC_RS64IV      = 0x00370000,
    /* Original POWER */
    /* XXX: should be POWER (RIOS), RSC3308, RSC4608,
     * POWER2 (RIOS2) & RSC2 (P2SC) here
     */
#if 0
    CPU_POWER           = xxx, /* 0x20000 ? 0x30000 for RSC ? */
#endif
#if 0
    CPU_POWER2          = xxx, /* 0x40000 ? */
#endif
    /* PA Semi core */
    CPU_POWERPC_PA6T        = 0x00900000,
};

/* System version register (used on MPC 8xxx)                                */
enum {
    PPC_SVR_8540      = 0x80300000,
    PPC_SVR_8541E     = 0x807A0010,
    PPC_SVR_8543v10   = 0x80320010,
    PPC_SVR_8543v11   = 0x80320011,
    PPC_SVR_8543v20   = 0x80320020,
    PPC_SVR_8543Ev10  = 0x803A0010,
    PPC_SVR_8543Ev11  = 0x803A0011,
    PPC_SVR_8543Ev20  = 0x803A0020,
    PPC_SVR_8545      = 0x80310220,
    PPC_SVR_8545E     = 0x80390220,
    PPC_SVR_8547E     = 0x80390120,
    PPC_SCR_8548v10   = 0x80310010,
    PPC_SCR_8548v11   = 0x80310011,
    PPC_SCR_8548v20   = 0x80310020,
    PPC_SVR_8548Ev10  = 0x80390010,
    PPC_SVR_8548Ev11  = 0x80390011,
    PPC_SVR_8548Ev20  = 0x80390020,
    PPC_SVR_8555E     = 0x80790010,
    PPC_SVR_8560v10   = 0x80700010,
    PPC_SVR_8560v20   = 0x80700020,
};

/*****************************************************************************/
/* PowerPC CPU definitions                                                   */
#define POWERPC_DEF(_name, _pvr, _pvr_mask, _type)                            \
    {                                                                         \
        .name        = _name,                                                 \
        .pvr         = _pvr,                                                  \
        .pvr_mask    = _pvr_mask,                                             \
        .insns_flags = glue(POWERPC_INSNS_,_type),                            \
        .msr_mask    = glue(POWERPC_MSRM_,_type),                             \
        .mmu_model   = glue(POWERPC_MMU_,_type),                              \
        .excp_model  = glue(POWERPC_EXCP_,_type),                             \
        .bus_model   = glue(POWERPC_INPUT_,_type),                            \
        .bfd_mach    = glue(POWERPC_BFDM_,_type),                             \
        .init_proc   = &glue(init_proc_,_type),                               \
    }

static ppc_def_t ppc_defs[] = {
    /* Embedded PowerPC                                                      */
    /* PowerPC 401 family                                                    */
    /* Generic PowerPC 401 */
    POWERPC_DEF("401",         CPU_POWERPC_401,         0xFFFF0000, 401),
    /* PowerPC 401 cores                                                     */
    /* PowerPC 401A1 */
    POWERPC_DEF("401A1",       CPU_POWERPC_401A1,       0xFFFFFFFF, 401),
    /* PowerPC 401B2                                                         */
    POWERPC_DEF("401B2",       CPU_POWERPC_401B2,       0xFFFFFFFF, 401x2),
#if defined (TODO)
    /* PowerPC 401B3                                                         */
    POWERPC_DEF("401B3",       CPU_POWERPC_401B3,       0xFFFFFFFF, 401x3),
#endif
    /* PowerPC 401C2                                                         */
    POWERPC_DEF("401C2",       CPU_POWERPC_401C2,       0xFFFFFFFF, 401x2),
    /* PowerPC 401D2                                                         */
    POWERPC_DEF("401D2",       CPU_POWERPC_401D2,       0xFFFFFFFF, 401x2),
    /* PowerPC 401E2                                                         */
    POWERPC_DEF("401E2",       CPU_POWERPC_401E2,       0xFFFFFFFF, 401x2),
    /* PowerPC 401F2                                                         */
    POWERPC_DEF("401F2",       CPU_POWERPC_401F2,       0xFFFFFFFF, 401x2),
    /* PowerPC 401G2                                                         */
    /* XXX: to be checked */
    POWERPC_DEF("401G2",       CPU_POWERPC_401G2,       0xFFFFFFFF, 401x2),
    /* PowerPC 401 microcontrolers                                           */
#if defined (TODO)
    /* PowerPC 401GF                                                         */
    POWERPC_DEF("401GF",       CPU_POWERPC_401GF,       0xFFFFFFFF, 401),
#endif
    /* IOP480 (401 microcontroler)                                           */
    POWERPC_DEF("IOP480",      CPU_POWERPC_IOP480,      0xFFFFFFFF, IOP480),
    /* IBM Processor for Network Resources                                   */
    POWERPC_DEF("Cobra",       CPU_POWERPC_COBRA,       0xFFFFFFFF, 401),
#if defined (TODO)
    POWERPC_DEF("Xipchip",     CPU_POWERPC_XIPCHIP,     0xFFFFFFFF, 401),
#endif
    /* PowerPC 403 family                                                    */
    /* Generic PowerPC 403                                                   */
    POWERPC_DEF("403",         CPU_POWERPC_403,         0xFFFF0000, 403),
    /* PowerPC 403 microcontrolers                                           */
    /* PowerPC 403 GA                                                        */
    POWERPC_DEF("403GA",       CPU_POWERPC_403GA,       0xFFFFFFFF, 403),
    /* PowerPC 403 GB                                                        */
    POWERPC_DEF("403GB",       CPU_POWERPC_403GB,       0xFFFFFFFF, 403),
    /* PowerPC 403 GC                                                        */
    POWERPC_DEF("403GC",       CPU_POWERPC_403GC,       0xFFFFFFFF, 403),
    /* PowerPC 403 GCX                                                       */
    POWERPC_DEF("403GCX",      CPU_POWERPC_403GCX,      0xFFFFFFFF, 403GCX),
#if defined (TODO)
    /* PowerPC 403 GP                                                        */
    POWERPC_DEF("403GP",       CPU_POWERPC_403GP,       0xFFFFFFFF, 403),
#endif
    /* PowerPC 405 family                                                    */
    /* Generic PowerPC 405                                                   */
    POWERPC_DEF("405",         CPU_POWERPC_405,         0xFFFF0000, 405),
    /* PowerPC 405 cores                                                     */
#if defined (TODO)
    /* PowerPC 405 A3                                                        */
    POWERPC_DEF("405A3",       CPU_POWERPC_405A3,       0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* PowerPC 405 A4                                                        */
    POWERPC_DEF("405A4",       CPU_POWERPC_405A4,       0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* PowerPC 405 B3                                                        */
    POWERPC_DEF("405B3",       CPU_POWERPC_405B3,       0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* PowerPC 405 B4                                                        */
    POWERPC_DEF("405B4",       CPU_POWERPC_405B4,       0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* PowerPC 405 C3                                                        */
    POWERPC_DEF("405C3",       CPU_POWERPC_405C3,       0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* PowerPC 405 C4                                                        */
    POWERPC_DEF("405C4",       CPU_POWERPC_405C4,       0xFFFFFFFF, 405),
#endif
    /* PowerPC 405 D2                                                        */
    POWERPC_DEF("405D2",       CPU_POWERPC_405D2,       0xFFFFFFFF, 405),
#if defined (TODO)
    /* PowerPC 405 D3                                                        */
    POWERPC_DEF("405D3",       CPU_POWERPC_405D3,       0xFFFFFFFF, 405),
#endif
    /* PowerPC 405 D4                                                        */
    POWERPC_DEF("405D4",       CPU_POWERPC_405D4,       0xFFFFFFFF, 405),
#if defined (TODO)
    /* PowerPC 405 D5                                                        */
    POWERPC_DEF("405D5",       CPU_POWERPC_405D5,       0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* PowerPC 405 E4                                                        */
    POWERPC_DEF("405E4",       CPU_POWERPC_405E4,       0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* PowerPC 405 F4                                                        */
    POWERPC_DEF("405F4",       CPU_POWERPC_405F4,       0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* PowerPC 405 F5                                                        */
    POWERPC_DEF("405F5",       CPU_POWERPC_405F5,       0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* PowerPC 405 F6                                                        */
    POWERPC_DEF("405F6",       CPU_POWERPC_405F6,       0xFFFFFFFF, 405),
#endif
    /* PowerPC 405 microcontrolers                                           */
    /* PowerPC 405 CR                                                        */
    POWERPC_DEF("405CR",       CPU_POWERPC_405CR,       0xFFFFFFFF, 405),
    /* PowerPC 405 CRa                                                       */
    POWERPC_DEF("405CRa",      CPU_POWERPC_405CRa,      0xFFFFFFFF, 405),
    /* PowerPC 405 CRb                                                       */
    POWERPC_DEF("405CRb",      CPU_POWERPC_405CRb,      0xFFFFFFFF, 405),
    /* PowerPC 405 CRc                                                       */
    POWERPC_DEF("405CRc",      CPU_POWERPC_405CRc,      0xFFFFFFFF, 405),
    /* PowerPC 405 EP                                                        */
    POWERPC_DEF("405EP",       CPU_POWERPC_405EP,       0xFFFFFFFF, 405),
#if defined(TODO)
    /* PowerPC 405 EXr                                                       */
    POWERPC_DEF("405EXr",      CPU_POWERPC_405EXr,      0xFFFFFFFF, 405),
#endif
    /* PowerPC 405 EZ                                                        */
    POWERPC_DEF("405EZ",       CPU_POWERPC_405EZ,       0xFFFFFFFF, 405),
#if defined(TODO)
    /* PowerPC 405 FX                                                        */
    POWERPC_DEF("405FX",       CPU_POWERPC_405FX,       0xFFFFFFFF, 405),
#endif
    /* PowerPC 405 GP                                                        */
    POWERPC_DEF("405GP",       CPU_POWERPC_405GP,       0xFFFFFFFF, 405),
    /* PowerPC 405 GPa                                                       */
    POWERPC_DEF("405GPa",      CPU_POWERPC_405GPa,      0xFFFFFFFF, 405),
    /* PowerPC 405 GPb                                                       */
    POWERPC_DEF("405GPb",      CPU_POWERPC_405GPb,      0xFFFFFFFF, 405),
    /* PowerPC 405 GPc                                                       */
    POWERPC_DEF("405GPc",      CPU_POWERPC_405GPc,      0xFFFFFFFF, 405),
    /* PowerPC 405 GPd                                                       */
    POWERPC_DEF("405GPd",      CPU_POWERPC_405GPd,      0xFFFFFFFF, 405),
    /* PowerPC 405 GPe                                                       */
    POWERPC_DEF("405GPe",      CPU_POWERPC_405GPe,      0xFFFFFFFF, 405),
    /* PowerPC 405 GPR                                                       */
    POWERPC_DEF("405GPR",      CPU_POWERPC_405GPR,      0xFFFFFFFF, 405),
#if defined(TODO)
    /* PowerPC 405 H                                                         */
    POWERPC_DEF("405H",        CPU_POWERPC_405H,        0xFFFFFFFF, 405),
#endif
#if defined(TODO)
    /* PowerPC 405 L                                                         */
    POWERPC_DEF("405L",        CPU_POWERPC_405L,        0xFFFFFFFF, 405),
#endif
    /* PowerPC 405 LP                                                        */
    POWERPC_DEF("405LP",       CPU_POWERPC_405LP,       0xFFFFFFFF, 405),
#if defined(TODO)
    /* PowerPC 405 PM                                                        */
    POWERPC_DEF("405PM",       CPU_POWERPC_405PM,       0xFFFFFFFF, 405),
#endif
#if defined(TODO)
    /* PowerPC 405 PS                                                        */
    POWERPC_DEF("405PS",       CPU_POWERPC_405PS,       0xFFFFFFFF, 405),
#endif
#if defined(TODO)
    /* PowerPC 405 S                                                         */
    POWERPC_DEF("405S",        CPU_POWERPC_405S,        0xFFFFFFFF, 405),
#endif
    /* Npe405 H                                                              */
    POWERPC_DEF("Npe405H",     CPU_POWERPC_NPE405H,     0xFFFFFFFF, 405),
    /* Npe405 H2                                                             */
    POWERPC_DEF("Npe405H2",    CPU_POWERPC_NPE405H2,    0xFFFFFFFF, 405),
    /* Npe405 L                                                              */
    POWERPC_DEF("Npe405L",     CPU_POWERPC_NPE405L,     0xFFFFFFFF, 405),
    /* Npe4GS3                                                               */
    POWERPC_DEF("Npe4GS3",     CPU_POWERPC_NPE4GS3,     0xFFFFFFFF, 405),
#if defined (TODO)
    POWERPC_DEF("Npcxx1",      CPU_POWERPC_NPCxx1,      0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    POWERPC_DEF("Npr161",      CPU_POWERPC_NPR161,      0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* PowerPC LC77700 (Sanyo)                                               */
    POWERPC_DEF("LC77700",     CPU_POWERPC_LC77700,     0xFFFFFFFF, 405),
#endif
    /* PowerPC 401/403/405 based set-top-box microcontrolers                 */
#if defined (TODO)
    /* STB010000                                                             */
    POWERPC_DEF("STB01000",    CPU_POWERPC_STB01000,    0xFFFFFFFF, 401x2),
#endif
#if defined (TODO)
    /* STB01010                                                              */
    POWERPC_DEF("STB01010",    CPU_POWERPC_STB01010,    0xFFFFFFFF, 401x2),
#endif
#if defined (TODO)
    /* STB0210                                                               */
    POWERPC_DEF("STB0210",     CPU_POWERPC_STB0210,     0xFFFFFFFF, 401x3),
#endif
    /* STB03xx                                                               */
    POWERPC_DEF("STB03",       CPU_POWERPC_STB03,       0xFFFFFFFF, 405),
#if defined (TODO)
    /* STB043x                                                               */
    POWERPC_DEF("STB043",      CPU_POWERPC_STB043,      0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* STB045x                                                               */
    POWERPC_DEF("STB045",      CPU_POWERPC_STB045,      0xFFFFFFFF, 405),
#endif
    /* STB04xx                                                               */
    POWERPC_DEF("STB04",       CPU_POWERPC_STB04,       0xFFFF0000, 405),
    /* STB25xx                                                               */
    POWERPC_DEF("STB25",       CPU_POWERPC_STB25,       0xFFFFFFFF, 405),
#if defined (TODO)
    /* STB130                                                                */
    POWERPC_DEF("STB130",      CPU_POWERPC_STB130,      0xFFFFFFFF, 405),
#endif
    /* Xilinx PowerPC 405 cores                                              */
    POWERPC_DEF("x2vp4",       CPU_POWERPC_X2VP4,       0xFFFFFFFF, 405),
    POWERPC_DEF("x2vp7",       CPU_POWERPC_X2VP7,       0xFFFFFFFF, 405),
    POWERPC_DEF("x2vp20",      CPU_POWERPC_X2VP20,      0xFFFFFFFF, 405),
    POWERPC_DEF("x2vp50",      CPU_POWERPC_X2VP50,      0xFFFFFFFF, 405),
#if defined (TODO)
    /* Zarlink ZL10310                                                       */
    POWERPC_DEF("zl10310",     CPU_POWERPC_ZL10310,     0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* Zarlink ZL10311                                                       */
    POWERPC_DEF("zl10311",     CPU_POWERPC_ZL10311,     0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* Zarlink ZL10320                                                       */
    POWERPC_DEF("zl10320",     CPU_POWERPC_ZL10320,     0xFFFFFFFF, 405),
#endif
#if defined (TODO)
    /* Zarlink ZL10321                                                       */
    POWERPC_DEF("zl10321",     CPU_POWERPC_ZL10321,     0xFFFFFFFF, 405),
#endif
    /* PowerPC 440 family                                                    */
    /* Generic PowerPC 440                                                   */
    POWERPC_DEF("440",         CPU_POWERPC_440,         0xFFFFFFFF, 440GP),
    /* PowerPC 440 cores                                                     */
#if defined (TODO)
    /* PowerPC 440 A4                                                        */
    POWERPC_DEF("440A4",       CPU_POWERPC_440A4,       0xFFFFFFFF, 440x4),
#endif
#if defined (TODO)
    /* PowerPC 440 A5                                                        */
    POWERPC_DEF("440A5",       CPU_POWERPC_440A5,       0xFFFFFFFF, 440x5),
#endif
#if defined (TODO)
    /* PowerPC 440 B4                                                        */
    POWERPC_DEF("440B4",       CPU_POWERPC_440B4,       0xFFFFFFFF, 440x4),
#endif
#if defined (TODO)
    /* PowerPC 440 G4                                                        */
    POWERPC_DEF("440G4",       CPU_POWERPC_440G4,       0xFFFFFFFF, 440x4),
#endif
#if defined (TODO)
    /* PowerPC 440 F5                                                        */
    POWERPC_DEF("440F5",       CPU_POWERPC_440F5,       0xFFFFFFFF, 440x5),
#endif
#if defined (TODO)
    /* PowerPC 440 G5                                                        */
    POWERPC_DEF("440G5",       CPU_POWERPC_440G5,       0xFFFFFFFF, 440x5),
#endif
#if defined (TODO)
    /* PowerPC 440H4                                                         */
    POWERPC_DEF("440H4",       CPU_POWERPC_440H4,       0xFFFFFFFF, 440x4),
#endif
#if defined (TODO)
    /* PowerPC 440H6                                                         */
    POWERPC_DEF("440H6",       CPU_POWERPC_440H6,       0xFFFFFFFF, 440Gx5),
#endif
    /* PowerPC 440 microcontrolers                                           */
    /* PowerPC 440 EP                                                        */
    POWERPC_DEF("440EP",       CPU_POWERPC_440EP,       0xFFFFFFFF, 440EP),
    /* PowerPC 440 EPa                                                       */
    POWERPC_DEF("440EPa",      CPU_POWERPC_440EPa,      0xFFFFFFFF, 440EP),
    /* PowerPC 440 EPb                                                       */
    POWERPC_DEF("440EPb",      CPU_POWERPC_440EPb,      0xFFFFFFFF, 440EP),
    /* PowerPC 440 EPX                                                       */
    POWERPC_DEF("440EPX",      CPU_POWERPC_440EPX,      0xFFFFFFFF, 440EP),
    /* PowerPC 440 GP                                                        */
    POWERPC_DEF("440GP",       CPU_POWERPC_440GP,       0xFFFFFFFF, 440GP),
    /* PowerPC 440 GPb                                                       */
    POWERPC_DEF("440GPb",      CPU_POWERPC_440GPb,      0xFFFFFFFF, 440GP),
    /* PowerPC 440 GPc                                                       */
    POWERPC_DEF("440GPc",      CPU_POWERPC_440GPc,      0xFFFFFFFF, 440GP),
    /* PowerPC 440 GR                                                        */
    POWERPC_DEF("440GR",       CPU_POWERPC_440GR,       0xFFFFFFFF, 440x5),
    /* PowerPC 440 GRa                                                       */
    POWERPC_DEF("440GRa",      CPU_POWERPC_440GRa,      0xFFFFFFFF, 440x5),
    /* PowerPC 440 GRX                                                       */
    POWERPC_DEF("440GRX",      CPU_POWERPC_440GRX,      0xFFFFFFFF, 440x5),
    /* PowerPC 440 GX                                                        */
    POWERPC_DEF("440GX",       CPU_POWERPC_440GX,       0xFFFFFFFF, 440EP),
    /* PowerPC 440 GXa                                                       */
    POWERPC_DEF("440GXa",      CPU_POWERPC_440GXa,      0xFFFFFFFF, 440EP),
    /* PowerPC 440 GXb                                                       */
    POWERPC_DEF("440GXb",      CPU_POWERPC_440GXb,      0xFFFFFFFF, 440EP),
    /* PowerPC 440 GXc                                                       */
    POWERPC_DEF("440GXc",      CPU_POWERPC_440GXc,      0xFFFFFFFF, 440EP),
    /* PowerPC 440 GXf                                                       */
    POWERPC_DEF("440GXf",      CPU_POWERPC_440GXf,      0xFFFFFFFF, 440EP),
#if defined(TODO)
    /* PowerPC 440 S                                                         */
    POWERPC_DEF("440S",        CPU_POWERPC_440S,        0xFFFFFFFF, 440),
#endif
    /* PowerPC 440 SP                                                        */
    POWERPC_DEF("440SP",       CPU_POWERPC_440SP,       0xFFFFFFFF, 440EP),
    /* PowerPC 440 SP2                                                       */
    POWERPC_DEF("440SP2",      CPU_POWERPC_440SP2,      0xFFFFFFFF, 440EP),
    /* PowerPC 440 SPE                                                       */
    POWERPC_DEF("440SPE",      CPU_POWERPC_440SPE,      0xFFFFFFFF, 440EP),
    /* PowerPC 460 family                                                    */
#if defined (TODO)
    /* Generic PowerPC 464                                                   */
    POWERPC_DEF("464",         CPU_POWERPC_464,         0xFFFFFFFF, 460),
#endif
    /* PowerPC 464 microcontrolers                                           */
#if defined (TODO)
    /* PowerPC 464H90                                                        */
    POWERPC_DEF("464H90",      CPU_POWERPC_464H90,      0xFFFFFFFF, 460),
#endif
#if defined (TODO)
    /* PowerPC 464H90F                                                       */
    POWERPC_DEF("464H90F",     CPU_POWERPC_464H90F,     0xFFFFFFFF, 460F),
#endif
    /* Freescale embedded PowerPC cores                                      */
    /* e200 family                                                           */
#if defined (TODO)
    /* Generic PowerPC e200 core                                             */
    POWERPC_DEF("e200",        CPU_POWERPC_e200,        0xFFFFFFFF, e200),
#endif
#if defined (TODO)
    /* PowerPC e200z5 core                                                   */
    POWERPC_DEF("e200z5",      CPU_POWERPC_e200z5,      0xFFFFFFFF, e200),
#endif
#if defined (TODO)
    /* PowerPC e200z6 core                                                   */
    POWERPC_DEF("e200z6",      CPU_POWERPC_e200z6,      0xFFFFFFFF, e200),
#endif
    /* e300 family                                                           */
#if defined (TODO)
    /* Generic PowerPC e300 core                                             */
    POWERPC_DEF("e300",        CPU_POWERPC_e300,        0xFFFFFFFF, e300),
#endif
#if defined (TODO)
    /* PowerPC e300c1 core                                                   */
    POWERPC_DEF("e300c1",      CPU_POWERPC_e300c1,      0xFFFFFFFF, e300),
#endif
#if defined (TODO)
    /* PowerPC e300c2 core                                                   */
    POWERPC_DEF("e300c2",      CPU_POWERPC_e300c2,      0xFFFFFFFF, e300),
#endif
#if defined (TODO)
    /* PowerPC e300c3 core                                                   */
    POWERPC_DEF("e300c3",      CPU_POWERPC_e300c3,      0xFFFFFFFF, e300),
#endif
    /* e500 family                                                           */
#if defined (TODO)
    /* PowerPC e500 core                                                     */
    POWERPC_DEF("e500",        CPU_POWERPC_e500,        0xFFFFFFFF, e500),
#endif
#if defined (TODO)
    /* PowerPC e500 v1.1 core                                                */
    POWERPC_DEF("e500v1.1",    CPU_POWERPC_e500_v11,    0xFFFFFFFF, e500),
#endif
#if defined (TODO)
    /* PowerPC e500 v1.2 core                                                */
    POWERPC_DEF("e500v1.2",    CPU_POWERPC_e500_v12,    0xFFFFFFFF, e500),
#endif
#if defined (TODO)
    /* PowerPC e500 v2.1 core                                                */
    POWERPC_DEF("e500v2.1",    CPU_POWERPC_e500_v21,    0xFFFFFFFF, e500),
#endif
#if defined (TODO)
    /* PowerPC e500 v2.2 core                                                */
    POWERPC_DEF("e500v2.2",    CPU_POWERPC_e500_v22,    0xFFFFFFFF, e500),
#endif
    /* e600 family                                                           */
#if defined (TODO)
    /* PowerPC e600 core                                                     */
    POWERPC_DEF("e600",        CPU_POWERPC_e600,        0xFFFFFFFF, e600),
#endif
    /* PowerPC MPC 5xx cores                                                 */
#if defined (TODO)
    /* PowerPC MPC 5xx                                                       */
    POWERPC_DEF("mpc5xx",      CPU_POWERPC_5xx,         0xFFFFFFFF, 5xx),
#endif
    /* PowerPC MPC 8xx cores                                                 */
#if defined (TODO)
    /* PowerPC MPC 8xx                                                       */
    POWERPC_DEF("mpc8xx",      CPU_POWERPC_8xx,         0xFFFFFFFF, 8xx),
#endif
    /* PowerPC MPC 8xxx cores                                                */
#if defined (TODO)
    /* PowerPC MPC 82xx HIP3                                                 */
    POWERPC_DEF("mpc82xxhip3", CPU_POWERPC_82xx_HIP3,   0xFFFFFFFF, 82xx),
#endif
#if defined (TODO)
    /* PowerPC MPC 82xx HIP4                                                 */
    POWERPC_DEF("mpc82xxhip4", CPU_POWERPC_82xx_HIP4,   0xFFFFFFFF, 82xx),
#endif
#if defined (TODO)
    /* PowerPC MPC 827x                                                      */
    POWERPC_DEF("mpc827x",     CPU_POWERPC_827x,        0xFFFFFFFF, 827x),
#endif

    /* 32 bits "classic" PowerPC                                             */
    /* PowerPC 6xx family                                                    */
    /* PowerPC 601                                                           */
    POWERPC_DEF("601",         CPU_POWERPC_601,         0xFFFFFFFF, 601),
    /* PowerPC 601v2                                                         */
    POWERPC_DEF("601a",        CPU_POWERPC_601a,        0xFFFFFFFF, 601),
    /* PowerPC 602                                                           */
    POWERPC_DEF("602",         CPU_POWERPC_602,         0xFFFFFFFF, 602),
    /* PowerPC 603                                                           */
    POWERPC_DEF("603",         CPU_POWERPC_603,         0xFFFFFFFF, 603),
    /* Code name for PowerPC 603                                             */
    POWERPC_DEF("Vanilla",     CPU_POWERPC_603,         0xFFFFFFFF, 603),
    /* PowerPC 603e                                                          */
    POWERPC_DEF("603e",        CPU_POWERPC_603E,        0xFFFFFFFF, 603E),
    /* Code name for PowerPC 603e                                            */
    POWERPC_DEF("Stretch",     CPU_POWERPC_603E,        0xFFFFFFFF, 603E),
    /* PowerPC 603e v1.1                                                     */
    POWERPC_DEF("603e1.1",     CPU_POWERPC_603E_v11,    0xFFFFFFFF, 603E),
    /* PowerPC 603e v1.2                                                     */
    POWERPC_DEF("603e1.2",     CPU_POWERPC_603E_v12,    0xFFFFFFFF, 603E),
    /* PowerPC 603e v1.3                                                     */
    POWERPC_DEF("603e1.3",     CPU_POWERPC_603E_v13,    0xFFFFFFFF, 603E),
    /* PowerPC 603e v1.4                                                     */
    POWERPC_DEF("603e1.4",     CPU_POWERPC_603E_v14,    0xFFFFFFFF, 603E),
    /* PowerPC 603e v2.2                                                     */
    POWERPC_DEF("603e2.2",     CPU_POWERPC_603E_v22,    0xFFFFFFFF, 603E),
    /* PowerPC 603e v3                                                       */
    POWERPC_DEF("603e3",       CPU_POWERPC_603E_v3,     0xFFFFFFFF, 603E),
    /* PowerPC 603e v4                                                       */
    POWERPC_DEF("603e4",       CPU_POWERPC_603E_v4,     0xFFFFFFFF, 603E),
    /* PowerPC 603e v4.1                                                     */
    POWERPC_DEF("603e4.1",     CPU_POWERPC_603E_v41,    0xFFFFFFFF, 603E),
    /* PowerPC 603e                                                          */
    POWERPC_DEF("603e7",       CPU_POWERPC_603E7,       0xFFFFFFFF, 603E),
    /* PowerPC 603e7t                                                        */
    POWERPC_DEF("603e7t",      CPU_POWERPC_603E7t,      0xFFFFFFFF, 603E),
    /* PowerPC 603e7v                                                        */
    POWERPC_DEF("603e7v",      CPU_POWERPC_603E7v,      0xFFFFFFFF, 603E),
    /* Code name for PowerPC 603ev                                           */
    POWERPC_DEF("Vaillant",    CPU_POWERPC_603E7v,      0xFFFFFFFF, 603E),
    /* PowerPC 603e7v1                                                       */
    POWERPC_DEF("603e7v1",     CPU_POWERPC_603E7v1,     0xFFFFFFFF, 603E),
    /* PowerPC 603e7v2                                                       */
    POWERPC_DEF("603e7v2",     CPU_POWERPC_603E7v2,     0xFFFFFFFF, 603E),
    /* PowerPC 603p                                                          */
    /* to be checked */
    POWERPC_DEF("603p",        CPU_POWERPC_603P,        0xFFFFFFFF, 603),
    /* PowerPC 603r                                                          */
    POWERPC_DEF("603r",        CPU_POWERPC_603R,        0xFFFFFFFF, 603E),
    /* Code name for PowerPC 603r                                            */
    POWERPC_DEF("Goldeneye",   CPU_POWERPC_603R,        0xFFFFFFFF, 603E),
    /* PowerPC G2 core                                                       */
    POWERPC_DEF("G2",          CPU_POWERPC_G2,          0xFFFFFFFF, G2),
    /* PowerPC G2 H4                                                         */
    POWERPC_DEF("G2H4",        CPU_POWERPC_G2H4,        0xFFFFFFFF, G2),
    /* PowerPC G2 GP                                                         */
    POWERPC_DEF("G2GP",        CPU_POWERPC_G2gp,        0xFFFFFFFF, G2),
    /* PowerPC G2 LS                                                         */
    POWERPC_DEF("G2LS",        CPU_POWERPC_G2ls,        0xFFFFFFFF, G2),
    /* PowerPC G2LE                                                          */
    /* Same as G2, with little-endian mode support                           */
    POWERPC_DEF("G2le",        CPU_POWERPC_G2LE,        0xFFFFFFFF, G2LE),
    /* PowerPC G2LE GP                                                       */
    POWERPC_DEF("G2leGP",      CPU_POWERPC_G2LEgp,      0xFFFFFFFF, G2LE),
    /* PowerPC G2LE LS                                                       */
    POWERPC_DEF("G2leLS",      CPU_POWERPC_G2LEls,      0xFFFFFFFF, G2LE),
    /* PowerPC 604                                                           */
    POWERPC_DEF("604",         CPU_POWERPC_604,         0xFFFFFFFF, 604),
    /* PowerPC 604e                                                          */
    POWERPC_DEF("604e",        CPU_POWERPC_604E,        0xFFFFFFFF, 604),
    /* PowerPC 604e v1.0                                                     */
    POWERPC_DEF("604e1.0",     CPU_POWERPC_604E_v10,    0xFFFFFFFF, 604),
    /* PowerPC 604e v2.2                                                     */
    POWERPC_DEF("604e2.2",     CPU_POWERPC_604E_v22,    0xFFFFFFFF, 604),
    /* PowerPC 604e v2.4                                                     */
    POWERPC_DEF("604e2.4",     CPU_POWERPC_604E_v24,    0xFFFFFFFF, 604),
    /* PowerPC 604r                                                          */
    POWERPC_DEF("604r",        CPU_POWERPC_604R,        0xFFFFFFFF, 604),
#if defined(TODO)
    /* PowerPC 604ev                                                         */
    POWERPC_DEF("604ev",       CPU_POWERPC_604EV,       0xFFFFFFFF, 604),
#endif
    /* PowerPC 7xx family                                                    */
    /* Generic PowerPC 740 (G3)                                              */
    POWERPC_DEF("740",         CPU_POWERPC_7x0,         0xFFFFFFFF, 7x0),
    /* Generic PowerPC 750 (G3)                                              */
    POWERPC_DEF("750",         CPU_POWERPC_7x0,         0xFFFFFFFF, 7x0),
    /* Code name for generic PowerPC 740/750 (G3)                            */
    POWERPC_DEF("Arthur",      CPU_POWERPC_7x0,         0xFFFFFFFF, 7x0),
    /* PowerPC 740/750 is also known as G3                                   */
    POWERPC_DEF("G3",          CPU_POWERPC_7x0,         0xFFFFFFFF, 7x0),
    /* PowerPC 740 v2.0 (G3)                                                 */
    POWERPC_DEF("740v2.0",     CPU_POWERPC_7x0_v20,     0xFFFFFFFF, 7x0),
    /* PowerPC 750 v2.0 (G3)                                                 */
    POWERPC_DEF("750v2.0",     CPU_POWERPC_7x0_v20,     0xFFFFFFFF, 7x0),
    /* PowerPC 740 v2.1 (G3)                                                 */
    POWERPC_DEF("740v2.1",     CPU_POWERPC_7x0_v21,     0xFFFFFFFF, 7x0),
    /* PowerPC 750 v2.1 (G3)                                                 */
    POWERPC_DEF("750v2.1",     CPU_POWERPC_7x0_v21,     0xFFFFFFFF, 7x0),
    /* PowerPC 740 v2.2 (G3)                                                 */
    POWERPC_DEF("740v2.2",     CPU_POWERPC_7x0_v22,     0xFFFFFFFF, 7x0),
    /* PowerPC 750 v2.2 (G3)                                                 */
    POWERPC_DEF("750v2.2",     CPU_POWERPC_7x0_v22,     0xFFFFFFFF, 7x0),
    /* PowerPC 740 v3.0 (G3)                                                 */
    POWERPC_DEF("740v3.0",     CPU_POWERPC_7x0_v30,     0xFFFFFFFF, 7x0),
    /* PowerPC 750 v3.0 (G3)                                                 */
    POWERPC_DEF("750v3.0",     CPU_POWERPC_7x0_v30,     0xFFFFFFFF, 7x0),
    /* PowerPC 740 v3.1 (G3)                                                 */
    POWERPC_DEF("740v3.1",     CPU_POWERPC_7x0_v31,     0xFFFFFFFF, 7x0),
    /* PowerPC 750 v3.1 (G3)                                                 */
    POWERPC_DEF("750v3.1",     CPU_POWERPC_7x0_v31,     0xFFFFFFFF, 7x0),
    /* PowerPC 740E (G3)                                                     */
    POWERPC_DEF("740e",        CPU_POWERPC_740E,        0xFFFFFFFF, 7x0),
    /* PowerPC 740P (G3)                                                     */
    POWERPC_DEF("740p",        CPU_POWERPC_7x0P,        0xFFFFFFFF, 7x0),
    /* PowerPC 750P (G3)                                                     */
    POWERPC_DEF("750p",        CPU_POWERPC_7x0P,        0xFFFFFFFF, 7x0),
    /* Code name for PowerPC 740P/750P (G3)                                  */
    POWERPC_DEF("Conan/Doyle", CPU_POWERPC_7x0P,        0xFFFFFFFF, 7x0),
    /* PowerPC 750CL (G3 embedded)                                           */
    POWERPC_DEF("750cl",       CPU_POWERPC_750CL,       0xFFFFFFFF, 7x0),
    /* PowerPC 750CX (G3 embedded)                                           */
    POWERPC_DEF("750cx",       CPU_POWERPC_750CX,       0xFFFFFFFF, 7x0),
    /* PowerPC 750CX v2.1 (G3 embedded)                                      */
    POWERPC_DEF("750cx2.1",    CPU_POWERPC_750CX_v21,   0xFFFFFFFF, 7x0),
    /* PowerPC 750CX v2.2 (G3 embedded)                                      */
    POWERPC_DEF("750cx2.2",    CPU_POWERPC_750CX_v22,   0xFFFFFFFF, 7x0),
    /* PowerPC 750CXe (G3 embedded)                                          */
    POWERPC_DEF("750cxe",      CPU_POWERPC_750CXE,      0xFFFFFFFF, 7x0),
    /* PowerPC 750CXe v2.1 (G3 embedded)                                     */
    POWERPC_DEF("750cxe21",    CPU_POWERPC_750CXE_v21,  0xFFFFFFFF, 7x0),
    /* PowerPC 750CXe v2.2 (G3 embedded)                                     */
    POWERPC_DEF("750cxe22",    CPU_POWERPC_750CXE_v22,  0xFFFFFFFF, 7x0),
    /* PowerPC 750CXe v2.3 (G3 embedded)                                     */
    POWERPC_DEF("750cxe23",    CPU_POWERPC_750CXE_v23,  0xFFFFFFFF, 7x0),
    /* PowerPC 750CXe v2.4 (G3 embedded)                                     */
    POWERPC_DEF("750cxe24",    CPU_POWERPC_750CXE_v24,  0xFFFFFFFF, 7x0),
    /* PowerPC 750CXe v2.4b (G3 embedded)                                    */
    POWERPC_DEF("750cxe24b",   CPU_POWERPC_750CXE_v24b, 0xFFFFFFFF, 7x0),
    /* PowerPC 750CXe v3.1 (G3 embedded)                                     */
    POWERPC_DEF("750cxe31",    CPU_POWERPC_750CXE_v31,  0xFFFFFFFF, 7x0),
    /* PowerPC 750CXe v3.1b (G3 embedded)                                    */
    POWERPC_DEF("750cxe3.1b",  CPU_POWERPC_750CXE_v31b, 0xFFFFFFFF, 7x0),
    /* PowerPC 750CXr (G3 embedded)                                          */
    POWERPC_DEF("750cxr",      CPU_POWERPC_750CXR,      0xFFFFFFFF, 7x0),
    /* PowerPC 750E (G3)                                                     */
    POWERPC_DEF("750e",        CPU_POWERPC_750E,        0xFFFFFFFF, 7x0),
    /* PowerPC 750FL (G3 embedded)                                           */
    POWERPC_DEF("750fl",       CPU_POWERPC_750FL,       0xFFFFFFFF, 750fx),
    /* PowerPC 750FX (G3 embedded)                                           */
    POWERPC_DEF("750fx",       CPU_POWERPC_750FX,       0xFFFFFFFF, 750fx),
    /* PowerPC 750FX v1.0 (G3 embedded)                                      */
    POWERPC_DEF("750fx1.0",    CPU_POWERPC_750FX_v10,   0xFFFFFFFF, 750fx),
    /* PowerPC 750FX v2.0 (G3 embedded)                                      */
    POWERPC_DEF("750fx2.0",    CPU_POWERPC_750FX_v20,   0xFFFFFFFF, 750fx),
    /* PowerPC 750FX v2.1 (G3 embedded)                                      */
    POWERPC_DEF("750fx2.1",    CPU_POWERPC_750FX_v21,   0xFFFFFFFF, 750fx),
    /* PowerPC 750FX v2.2 (G3 embedded)                                      */
    POWERPC_DEF("750fx2.2",    CPU_POWERPC_750FX_v22,   0xFFFFFFFF, 750fx),
    /* PowerPC 750FX v2.3 (G3 embedded)                                      */
    POWERPC_DEF("750fx2.3",    CPU_POWERPC_750FX_v23,   0xFFFFFFFF, 750fx),
    /* PowerPC 750GL (G3 embedded)                                           */
    POWERPC_DEF("750gl",       CPU_POWERPC_750GL,       0xFFFFFFFF, 750fx),
    /* PowerPC 750GX (G3 embedded)                                           */
    POWERPC_DEF("750gx",       CPU_POWERPC_750GX,       0xFFFFFFFF, 750fx),
    /* PowerPC 750GX v1.0 (G3 embedded)                                      */
    POWERPC_DEF("750gx1.0",    CPU_POWERPC_750GX_v10,   0xFFFFFFFF, 750fx),
    /* PowerPC 750GX v1.1 (G3 embedded)                                      */
    POWERPC_DEF("750gx1.1",    CPU_POWERPC_750GX_v11,   0xFFFFFFFF, 750fx),
    /* PowerPC 750GX v1.2 (G3 embedded)                                      */
    POWERPC_DEF("750gx1.2",    CPU_POWERPC_750GX_v12,   0xFFFFFFFF, 750fx),
    /* PowerPC 750L (G3 embedded)                                            */
    POWERPC_DEF("750l",        CPU_POWERPC_750L,        0xFFFFFFFF, 7x0),
    /* Code name for PowerPC 750L (G3 embedded)                              */
    POWERPC_DEF("LoneStar",    CPU_POWERPC_750L,        0xFFFFFFFF, 7x0),
    /* PowerPC 750L v2.2 (G3 embedded)                                       */
    POWERPC_DEF("750l2.2",     CPU_POWERPC_750L_v22,    0xFFFFFFFF, 7x0),
    /* PowerPC 750L v3.0 (G3 embedded)                                       */
    POWERPC_DEF("750l3.0",     CPU_POWERPC_750L_v30,    0xFFFFFFFF, 7x0),
    /* PowerPC 750L v3.2 (G3 embedded)                                       */
    POWERPC_DEF("750l3.2",     CPU_POWERPC_750L_v32,    0xFFFFFFFF, 7x0),
    /* Generic PowerPC 745                                                   */
    POWERPC_DEF("745",         CPU_POWERPC_7x5,         0xFFFFFFFF, 7x5),
    /* Generic PowerPC 755                                                   */
    POWERPC_DEF("755",         CPU_POWERPC_7x5,         0xFFFFFFFF, 7x5),
    /* Code name for PowerPC 745/755                                         */
    POWERPC_DEF("Goldfinger",  CPU_POWERPC_7x5,         0xFFFFFFFF, 7x5),
    /* PowerPC 745 v1.0                                                      */
    POWERPC_DEF("745v1.0",     CPU_POWERPC_7x5_v10,     0xFFFFFFFF, 7x5),
    /* PowerPC 755 v1.0                                                      */
    POWERPC_DEF("755v1.0",     CPU_POWERPC_7x5_v10,     0xFFFFFFFF, 7x5),
    /* PowerPC 745 v1.1                                                      */
    POWERPC_DEF("745v1.1",     CPU_POWERPC_7x5_v11,     0xFFFFFFFF, 7x5),
    /* PowerPC 755 v1.1                                                      */
    POWERPC_DEF("755v1.1",     CPU_POWERPC_7x5_v11,     0xFFFFFFFF, 7x5),
    /* PowerPC 745 v2.0                                                      */
    POWERPC_DEF("745v2.0",     CPU_POWERPC_7x5_v20,     0xFFFFFFFF, 7x5),
    /* PowerPC 755 v2.0                                                      */
    POWERPC_DEF("755v2.0",     CPU_POWERPC_7x5_v20,     0xFFFFFFFF, 7x5),
    /* PowerPC 745 v2.1                                                      */
    POWERPC_DEF("745v2.1",     CPU_POWERPC_7x5_v21,     0xFFFFFFFF, 7x5),
    /* PowerPC 755 v2.1                                                      */
    POWERPC_DEF("755v2.1",     CPU_POWERPC_7x5_v21,     0xFFFFFFFF, 7x5),
    /* PowerPC 745 v2.2                                                      */
    POWERPC_DEF("745v2.2",     CPU_POWERPC_7x5_v22,     0xFFFFFFFF, 7x5),
    /* PowerPC 755 v2.2                                                      */
    POWERPC_DEF("755v2.2",     CPU_POWERPC_7x5_v22,     0xFFFFFFFF, 7x5),
    /* PowerPC 745 v2.3                                                      */
    POWERPC_DEF("745v2.3",     CPU_POWERPC_7x5_v23,     0xFFFFFFFF, 7x5),
    /* PowerPC 755 v2.3                                                      */
    POWERPC_DEF("755v2.3",     CPU_POWERPC_7x5_v23,     0xFFFFFFFF, 7x5),
    /* PowerPC 745 v2.4                                                      */
    POWERPC_DEF("745v2.4",     CPU_POWERPC_7x5_v24,     0xFFFFFFFF, 7x5),
    /* PowerPC 755 v2.4                                                      */
    POWERPC_DEF("755v2.4",     CPU_POWERPC_7x5_v24,     0xFFFFFFFF, 7x5),
    /* PowerPC 745 v2.5                                                      */
    POWERPC_DEF("745v2.5",     CPU_POWERPC_7x5_v25,     0xFFFFFFFF, 7x5),
    /* PowerPC 755 v2.5                                                      */
    POWERPC_DEF("755v2.5",     CPU_POWERPC_7x5_v25,     0xFFFFFFFF, 7x5),
    /* PowerPC 745 v2.6                                                      */
    POWERPC_DEF("745v2.6",     CPU_POWERPC_7x5_v26,     0xFFFFFFFF, 7x5),
    /* PowerPC 755 v2.6                                                      */
    POWERPC_DEF("755v2.6",     CPU_POWERPC_7x5_v26,     0xFFFFFFFF, 7x5),
    /* PowerPC 745 v2.7                                                      */
    POWERPC_DEF("745v2.7",     CPU_POWERPC_7x5_v27,     0xFFFFFFFF, 7x5),
    /* PowerPC 755 v2.7                                                      */
    POWERPC_DEF("755v2.7",     CPU_POWERPC_7x5_v27,     0xFFFFFFFF, 7x5),
    /* PowerPC 745 v2.8                                                      */
    POWERPC_DEF("745v2.8",     CPU_POWERPC_7x5_v28,     0xFFFFFFFF, 7x5),
    /* PowerPC 755 v2.8                                                      */
    POWERPC_DEF("755v2.8",     CPU_POWERPC_7x5_v28,     0xFFFFFFFF, 7x5),
#if defined (TODO)
    /* PowerPC 745P (G3)                                                     */
    POWERPC_DEF("745p",        CPU_POWERPC_7x5P,        0xFFFFFFFF, 7x5),
    /* PowerPC 755P (G3)                                                     */
    POWERPC_DEF("755p",        CPU_POWERPC_7x5P,        0xFFFFFFFF, 7x5),
#endif
    /* PowerPC 74xx family                                                   */
    /* PowerPC 7400 (G4)                                                     */
    POWERPC_DEF("7400",        CPU_POWERPC_7400,        0xFFFFFFFF, 7400),
    /* Code name for PowerPC 7400                                            */
    POWERPC_DEF("Max",         CPU_POWERPC_7400,        0xFFFFFFFF, 7400),
    /* PowerPC 74xx is also well known as G4                                 */
    POWERPC_DEF("G4",          CPU_POWERPC_7400,        0xFFFFFFFF, 7400),
    /* PowerPC 7400 v1.0 (G4)                                                */
    POWERPC_DEF("7400v1.0",    CPU_POWERPC_7400_v10,    0xFFFFFFFF, 7400),
    /* PowerPC 7400 v1.1 (G4)                                                */
    POWERPC_DEF("7400v1.1",    CPU_POWERPC_7400_v11,    0xFFFFFFFF, 7400),
    /* PowerPC 7400 v2.0 (G4)                                                */
    POWERPC_DEF("7400v2.0",    CPU_POWERPC_7400_v20,    0xFFFFFFFF, 7400),
    /* PowerPC 7400 v2.2 (G4)                                                */
    POWERPC_DEF("7400v2.2",    CPU_POWERPC_7400_v22,    0xFFFFFFFF, 7400),
    /* PowerPC 7400 v2.6 (G4)                                                */
    POWERPC_DEF("7400v2.6",    CPU_POWERPC_7400_v26,    0xFFFFFFFF, 7400),
    /* PowerPC 7400 v2.7 (G4)                                                */
    POWERPC_DEF("7400v2.7",    CPU_POWERPC_7400_v27,    0xFFFFFFFF, 7400),
    /* PowerPC 7400 v2.8 (G4)                                                */
    POWERPC_DEF("7400v2.8",    CPU_POWERPC_7400_v28,    0xFFFFFFFF, 7400),
    /* PowerPC 7400 v2.9 (G4)                                                */
    POWERPC_DEF("7400v2.9",    CPU_POWERPC_7400_v29,    0xFFFFFFFF, 7400),
    /* PowerPC 7410 (G4)                                                     */
    POWERPC_DEF("7410",        CPU_POWERPC_7410,        0xFFFFFFFF, 7410),
    /* Code name for PowerPC 7410                                            */
    POWERPC_DEF("Nitro",       CPU_POWERPC_7410,        0xFFFFFFFF, 7410),
    /* PowerPC 7410 v1.0 (G4)                                                */
    POWERPC_DEF("7410v1.0",    CPU_POWERPC_7410_v10,    0xFFFFFFFF, 7410),
    /* PowerPC 7410 v1.1 (G4)                                                */
    POWERPC_DEF("7410v1.1",    CPU_POWERPC_7410_v11,    0xFFFFFFFF, 7410),
    /* PowerPC 7410 v1.2 (G4)                                                */
    POWERPC_DEF("7410v1.2",    CPU_POWERPC_7410_v12,    0xFFFFFFFF, 7410),
    /* PowerPC 7410 v1.3 (G4)                                                */
    POWERPC_DEF("7410v1.3",    CPU_POWERPC_7410_v13,    0xFFFFFFFF, 7410),
    /* PowerPC 7410 v1.4 (G4)                                                */
    POWERPC_DEF("7410v1.4",    CPU_POWERPC_7410_v14,    0xFFFFFFFF, 7410),
    /* PowerPC 7448 (G4)                                                     */
    POWERPC_DEF("7448",        CPU_POWERPC_7448,        0xFFFFFFFF, 7400),
    /* PowerPC 7448 v1.0 (G4)                                                */
    POWERPC_DEF("7448v1.0",    CPU_POWERPC_7448_v10,    0xFFFFFFFF, 7400),
    /* PowerPC 7448 v1.1 (G4)                                                */
    POWERPC_DEF("7448v1.1",    CPU_POWERPC_7448_v11,    0xFFFFFFFF, 7400),
    /* PowerPC 7448 v2.0 (G4)                                                */
    POWERPC_DEF("7448v2.0",    CPU_POWERPC_7448_v20,    0xFFFFFFFF, 7400),
    /* PowerPC 7448 v2.1 (G4)                                                */
    POWERPC_DEF("7448v2.1",    CPU_POWERPC_7448_v21,    0xFFFFFFFF, 7400),
#if defined (TODO)
    /* PowerPC 7450 (G4)                                                     */
    POWERPC_DEF("7450",        CPU_POWERPC_7450,        0xFFFFFFFF, 7450),
    /* Code name for PowerPC 7450                                            */
    POWERPC_DEF("Vger",        CPU_POWERPC_7450,        0xFFFFFFFF, 7450),
#endif
#if defined (TODO)
    /* PowerPC 7450 v1.0 (G4)                                                */
    POWERPC_DEF("7450v1.0",    CPU_POWERPC_7450_v10,    0xFFFFFFFF, 7450),
#endif
#if defined (TODO)
    /* PowerPC 7450 v1.1 (G4)                                                */
    POWERPC_DEF("7450v1.1",    CPU_POWERPC_7450_v11,    0xFFFFFFFF, 7450),
#endif
#if defined (TODO)
    /* PowerPC 7450 v1.2 (G4)                                                */
    POWERPC_DEF("7450v1.2",    CPU_POWERPC_7450_v12,    0xFFFFFFFF, 7450),
#endif
#if defined (TODO)
    /* PowerPC 7450 v2.0 (G4)                                                */
    POWERPC_DEF("7450v2.0",    CPU_POWERPC_7450_v20,    0xFFFFFFFF, 7450),
#endif
#if defined (TODO)
    /* PowerPC 7450 v2.1 (G4)                                                */
    POWERPC_DEF("7450v2.1",    CPU_POWERPC_7450_v21,    0xFFFFFFFF, 7450),
#endif
#if defined (TODO)
    /* PowerPC 7441 (G4)                                                     */
    POWERPC_DEF("7441",        CPU_POWERPC_74x1,        0xFFFFFFFF, 7440),
    /* PowerPC 7451 (G4)                                                     */
    POWERPC_DEF("7451",        CPU_POWERPC_74x1,        0xFFFFFFFF, 7450),
#endif
#if defined (TODO)
    /* PowerPC 7441g (G4)                                                    */
    POWERPC_DEF("7441g",       CPU_POWERPC_74x1G,       0xFFFFFFFF, 7440),
    /* PowerPC 7451g (G4)                                                    */
    POWERPC_DEF("7451g",       CPU_POWERPC_74x1G,       0xFFFFFFFF, 7450),
#endif
#if defined (TODO)
    /* PowerPC 7445 (G4)                                                     */
    POWERPC_DEF("7445",        CPU_POWERPC_74x5,        0xFFFFFFFF, 7445),
    /* PowerPC 7455 (G4)                                                     */
    POWERPC_DEF("7455",        CPU_POWERPC_74x5,        0xFFFFFFFF, 7455),
    /* Code name for PowerPC 7445/7455                                       */
    POWERPC_DEF("Apollo6",     CPU_POWERPC_74x5,        0xFFFFFFFF, 7455),
#endif
#if defined (TODO)
    /* PowerPC 7445 v1.0 (G4)                                                */
    POWERPC_DEF("7445v1.0",    CPU_POWERPC_74x5_v10,    0xFFFFFFFF, 7445),
    /* PowerPC 7455 v1.0 (G4)                                                */
    POWERPC_DEF("7455v1.0",    CPU_POWERPC_74x5_v10,    0xFFFFFFFF, 7455),
#endif
#if defined (TODO)
    /* PowerPC 7445 v2.1 (G4)                                                */
    POWERPC_DEF("7445v2.1",    CPU_POWERPC_74x5_v21,    0xFFFFFFFF, 7445),
    /* PowerPC 7455 v2.1 (G4)                                                */
    POWERPC_DEF("7455v2.1",    CPU_POWERPC_74x5_v21,    0xFFFFFFFF, 7455),
#endif
#if defined (TODO)
    /* PowerPC 7445 v3.2 (G4)                                                */
    POWERPC_DEF("7445v3.2",    CPU_POWERPC_74x5_v32,    0xFFFFFFFF, 7445),
    /* PowerPC 7455 v3.2 (G4)                                                */
    POWERPC_DEF("7455v3.2",    CPU_POWERPC_74x5_v32,    0xFFFFFFFF, 7455),
#endif
#if defined (TODO)
    /* PowerPC 7445 v3.3 (G4)                                                */
    POWERPC_DEF("7445v3.3",    CPU_POWERPC_74x5_v33,    0xFFFFFFFF, 7445),
    /* PowerPC 7455 v3.3 (G4)                                                */
    POWERPC_DEF("7455v3.3",    CPU_POWERPC_74x5_v33,    0xFFFFFFFF, 7455),
#endif
#if defined (TODO)
    /* PowerPC 7445 v3.4 (G4)                                                */
    POWERPC_DEF("7445v3.4",    CPU_POWERPC_74x5_v34,    0xFFFFFFFF, 7445),
    /* PowerPC 7455 v3.4 (G4)                                                */
    POWERPC_DEF("7455v3.4",    CPU_POWERPC_74x5_v34,    0xFFFFFFFF, 7455),
#endif
#if defined (TODO)
    /* PowerPC 7447 (G4)                                                     */
    POWERPC_DEF("7447",        CPU_POWERPC_74x7,        0xFFFFFFFF, 7445),
    /* PowerPC 7457 (G4)                                                     */
    POWERPC_DEF("7457",        CPU_POWERPC_74x7,        0xFFFFFFFF, 7455),
    /* Code name for PowerPC 7447/7457                                       */
    POWERPC_DEF("Apollo7",     CPU_POWERPC_74x7,        0xFFFFFFFF, 7455),
#endif
#if defined (TODO)
    /* PowerPC 7447 v1.0 (G4)                                                */
    POWERPC_DEF("7447v1.0",    CPU_POWERPC_74x7_v10,    0xFFFFFFFF, 7445),
    /* PowerPC 7457 v1.0 (G4)                                                */
    POWERPC_DEF("7457v1.0",    CPU_POWERPC_74x7_v10,    0xFFFFFFFF, 7455),
    /* Code name for PowerPC 7447A/7457A                                     */
    POWERPC_DEF("Apollo7PM",   CPU_POWERPC_74x7_v10,    0xFFFFFFFF, 7455),
#endif
#if defined (TODO)
    /* PowerPC 7447 v1.1 (G4)                                                */
    POWERPC_DEF("7447v1.1",    CPU_POWERPC_74x7_v11,    0xFFFFFFFF, 7445),
    /* PowerPC 7457 v1.1 (G4)                                                */
    POWERPC_DEF("7457v1.1",    CPU_POWERPC_74x7_v11,    0xFFFFFFFF, 7455),
#endif
#if defined (TODO)
    /* PowerPC 7447 v1.2 (G4)                                                */
    POWERPC_DEF("7447v1.2",    CPU_POWERPC_74x7_v12,    0xFFFFFFFF, 7445),
    /* PowerPC 7457 v1.2 (G4)                                                */
    POWERPC_DEF("7457v1.2",    CPU_POWERPC_74x7_v12,    0xFFFFFFFF, 7455),
#endif
    /* 64 bits PowerPC                                                       */
#if defined (TARGET_PPC64)
#if defined (TODO)
    /* PowerPC 620                                                           */
    POWERPC_DEF("620",         CPU_POWERPC_620,         0xFFFFFFFF, 620),
#endif
#if defined (TODO)
    /* PowerPC 630 (POWER3)                                                  */
    POWERPC_DEF("630",         CPU_POWERPC_630,         0xFFFFFFFF, 630),
    POWERPC_DEF("POWER3",      CPU_POWERPC_630,         0xFFFFFFFF, 630),
#endif
#if defined (TODO)
    /* PowerPC 631 (Power 3+)                                                */
    POWERPC_DEF("631",         CPU_POWERPC_631,         0xFFFFFFFF, 631),
    POWERPC_DEF("POWER3+",     CPU_POWERPC_631,         0xFFFFFFFF, 631),
#endif
#if defined (TODO)
    /* POWER4                                                                */
    POWERPC_DEF("POWER4",      CPU_POWERPC_POWER4,      0xFFFFFFFF, POWER4),
#endif
#if defined (TODO)
    /* POWER4p                                                               */
    POWERPC_DEF("POWER4+",     CPU_POWERPC_POWER4P,     0xFFFFFFFF, POWER4P),
#endif
#if defined (TODO)
    /* POWER5                                                                */
    POWERPC_DEF("POWER5",      CPU_POWERPC_POWER5,      0xFFFFFFFF, POWER5),
    /* POWER5GR                                                              */
    POWERPC_DEF("POWER5gr",    CPU_POWERPC_POWER5GR,    0xFFFFFFFF, POWER5),
#endif
#if defined (TODO)
    /* POWER5+                                                               */
    POWERPC_DEF("POWER5+",     CPU_POWERPC_POWER5P,     0xFFFFFFFF, POWER5P),
    /* POWER5GS                                                              */
    POWERPC_DEF("POWER5gs",    CPU_POWERPC_POWER5GS,    0xFFFFFFFF, POWER5P),
#endif
#if defined (TODO)
    /* POWER6                                                                */
    POWERPC_DEF("POWER6",      CPU_POWERPC_POWER6,      0xFFFFFFFF, POWER6),
    /* POWER6 running in POWER5 mode                                         */
    POWERPC_DEF("POWER6_5",    CPU_POWERPC_POWER6_5,    0xFFFFFFFF, POWER5),
    /* POWER6A                                                               */
    POWERPC_DEF("POWER6A",     CPU_POWERPC_POWER6A,     0xFFFFFFFF, POWER6),
#endif
    /* PowerPC 970                                                           */
    POWERPC_DEF("970",         CPU_POWERPC_970,         0xFFFFFFFF, 970),
    /* PowerPC 970FX (G5)                                                    */
    POWERPC_DEF("970fx",       CPU_POWERPC_970FX,       0xFFFFFFFF, 970FX),
    /* PowerPC 970FX v1.0 (G5)                                               */
    POWERPC_DEF("970fx1.0",    CPU_POWERPC_970FX_v10,   0xFFFFFFFF, 970FX),
    /* PowerPC 970FX v2.0 (G5)                                               */
    POWERPC_DEF("970fx2.0",    CPU_POWERPC_970FX_v20,   0xFFFFFFFF, 970FX),
    /* PowerPC 970FX v2.1 (G5)                                               */
    POWERPC_DEF("970fx2.1",    CPU_POWERPC_970FX_v21,   0xFFFFFFFF, 970FX),
    /* PowerPC 970FX v3.0 (G5)                                               */
    POWERPC_DEF("970fx3.0",    CPU_POWERPC_970FX_v30,   0xFFFFFFFF, 970FX),
    /* PowerPC 970FX v3.1 (G5)                                               */
    POWERPC_DEF("970fx3.1",    CPU_POWERPC_970FX_v31,   0xFFFFFFFF, 970FX),
    /* PowerPC 970GX (G5)                                                    */
    POWERPC_DEF("970gx",       CPU_POWERPC_970GX,       0xFFFFFFFF, 970GX),
    /* PowerPC 970MP                                                         */
    POWERPC_DEF("970mp",       CPU_POWERPC_970MP,       0xFFFFFFFF, 970),
    /* PowerPC 970MP v1.0                                                    */
    POWERPC_DEF("970mp1.0",    CPU_POWERPC_970MP_v10,   0xFFFFFFFF, 970),
    /* PowerPC 970MP v1.1                                                    */
    POWERPC_DEF("970mp1.1",    CPU_POWERPC_970MP_v11,   0xFFFFFFFF, 970),
#if defined (TODO)
    /* PowerPC Cell                                                          */
    POWERPC_DEF("Cell",        CPU_POWERPC_CELL,        0xFFFFFFFF, 970),
#endif
#if defined (TODO)
    /* PowerPC Cell v1.0                                                     */
    POWERPC_DEF("Cell1.0",     CPU_POWERPC_CELL_v10,    0xFFFFFFFF, 970),
#endif
#if defined (TODO)
    /* PowerPC Cell v2.0                                                     */
    POWERPC_DEF("Cell2.0",     CPU_POWERPC_CELL_v20,    0xFFFFFFFF, 970),
#endif
#if defined (TODO)
    /* PowerPC Cell v3.0                                                     */
    POWERPC_DEF("Cell3.0",     CPU_POWERPC_CELL_v30,    0xFFFFFFFF, 970),
#endif
#if defined (TODO)
    /* PowerPC Cell v3.1                                                     */
    POWERPC_DEF("Cell3.1",     CPU_POWERPC_CELL_v31,    0xFFFFFFFF, 970),
#endif
#if defined (TODO)
    /* PowerPC Cell v3.2                                                     */
    POWERPC_DEF("Cell3.2",     CPU_POWERPC_CELL_v32,    0xFFFFFFFF, 970),
#endif
#if defined (TODO)
    /* RS64 (Apache/A35)                                                     */
    /* This one seems to support the whole POWER2 instruction set
     * and the PowerPC 64 one.
     */
    /* What about A10 & A30 ? */
    POWERPC_DEF("RS64",        CPU_POWERPC_RS64,        0xFFFFFFFF, RS64),
    POWERPC_DEF("Apache",      CPU_POWERPC_RS64,        0xFFFFFFFF, RS64),
    POWERPC_DEF("A35",         CPU_POWERPC_RS64,        0xFFFFFFFF, RS64),
#endif
#if defined (TODO)
    /* RS64-II (NorthStar/A50)                                               */
    POWERPC_DEF("RS64-II",     CPU_POWERPC_RS64II,      0xFFFFFFFF, RS64),
    POWERPC_DEF("NorthStar",   CPU_POWERPC_RS64II,      0xFFFFFFFF, RS64),
    POWERPC_DEF("A50",         CPU_POWERPC_RS64II,      0xFFFFFFFF, RS64),
#endif
#if defined (TODO)
    /* RS64-III (Pulsar)                                                     */
    POWERPC_DEF("RS64-III",    CPU_POWERPC_RS64III,     0xFFFFFFFF, RS64),
    POWERPC_DEF("Pulsar",      CPU_POWERPC_RS64III,     0xFFFFFFFF, RS64),
#endif
#if defined (TODO)
    /* RS64-IV (IceStar/IStar/SStar)                                         */
    POWERPC_DEF("RS64-IV",     CPU_POWERPC_RS64IV,      0xFFFFFFFF, RS64),
    POWERPC_DEF("IceStar",     CPU_POWERPC_RS64IV,      0xFFFFFFFF, RS64),
    POWERPC_DEF("IStar",       CPU_POWERPC_RS64IV,      0xFFFFFFFF, RS64),
    POWERPC_DEF("SStar",       CPU_POWERPC_RS64IV,      0xFFFFFFFF, RS64),
#endif
#endif /* defined (TARGET_PPC64) */
    /* POWER                                                                 */
#if defined (TODO)
    /* Original POWER                                                        */
    POWERPC_DEF("POWER",       CPU_POWERPC_POWER,       0xFFFFFFFF, POWER),
    POWERPC_DEF("RIOS",        CPU_POWERPC_POWER,       0xFFFFFFFF, POWER),
    POWERPC_DEF("RSC",         CPU_POWERPC_POWER,       0xFFFFFFFF, POWER),
    POWERPC_DEF("RSC3308",     CPU_POWERPC_POWER,       0xFFFFFFFF, POWER),
    POWERPC_DEF("RSC4608",     CPU_POWERPC_POWER,       0xFFFFFFFF, POWER),
#endif
#if defined (TODO)
    /* POWER2                                                                */
    POWERPC_DEF("POWER2",      CPU_POWERPC_POWER2,      0xFFFFFFFF, POWER),
    POWERPC_DEF("RSC2",        CPU_POWERPC_POWER2,      0xFFFFFFFF, POWER),
    POWERPC_DEF("P2SC",        CPU_POWERPC_POWER2,      0xFFFFFFFF, POWER),
#endif
    /* PA semi cores                                                         */
#if defined (TODO)
    /* PA PA6T */
    POWERPC_DEF("PA6T",        CPU_POWERPC_PA6T,        0xFFFFFFFF, PA6T),
#endif
    /* Generic PowerPCs                                                      */
#if defined (TARGET_PPC64)
#if defined (TODO)
    POWERPC_DEF("ppc64",       CPU_POWERPC_PPC64,       0xFFFFFFFF, PPC64),
#endif
#endif
    POWERPC_DEF("ppc32",       CPU_POWERPC_PPC32,       0xFFFFFFFF, PPC32),
    POWERPC_DEF("ppc",         CPU_POWERPC_DEFAULT,     0xFFFFFFFF, DEFAULT),
    /* Fallback                                                              */
    POWERPC_DEF("default",     CPU_POWERPC_DEFAULT,     0xFFFFFFFF, DEFAULT),
};

/*****************************************************************************/
/* Generic CPU instanciation routine                                         */
static void init_ppc_proc (CPUPPCState *env, ppc_def_t *def)
{
#if !defined(CONFIG_USER_ONLY)
    int i;

    env->irq_inputs = NULL;
    /* Set all exception vectors to an invalid address */
    for (i = 0; i < POWERPC_EXCP_NB; i++)
        env->excp_vectors[i] = (target_ulong)(-1ULL);
    env->excp_prefix = 0x00000000;
    env->ivor_mask = 0x00000000;
    env->ivpr_mask = 0x00000000;
#endif
    /* Default MMU definitions */
    env->nb_BATs = 0;
    env->nb_tlb = 0;
    env->nb_ways = 0;
    /* Register SPR common to all PowerPC implementations */
    gen_spr_generic(env);
    spr_register(env, SPR_PVR, "PVR",
                 SPR_NOACCESS, SPR_NOACCESS,
                 &spr_read_generic, SPR_NOACCESS,
                 def->pvr);
    /* PowerPC implementation specific initialisations (SPRs, timers, ...) */
    (*def->init_proc)(env);
    /* Allocate TLBs buffer when needed */
    if (env->nb_tlb != 0) {
        int nb_tlb = env->nb_tlb;
        if (env->id_tlbs != 0)
            nb_tlb *= 2;
        env->tlb = qemu_mallocz(nb_tlb * sizeof(ppc_tlb_t));
        /* Pre-compute some useful values */
        env->tlb_per_way = env->nb_tlb / env->nb_ways;
    }
#if !defined(CONFIG_USER_ONLY)
    if (env->irq_inputs == NULL) {
        fprintf(stderr, "WARNING: no internal IRQ controller registered.\n"
                " Attempt Qemu to crash very soon !\n");
    }
#endif
}

#if defined(PPC_DUMP_CPU)
static void dump_ppc_sprs (CPUPPCState *env)
{
    ppc_spr_t *spr;
#if !defined(CONFIG_USER_ONLY)
    uint32_t sr, sw;
#endif
    uint32_t ur, uw;
    int i, j, n;

    printf("Special purpose registers:\n");
    for (i = 0; i < 32; i++) {
        for (j = 0; j < 32; j++) {
            n = (i << 5) | j;
            spr = &env->spr_cb[n];
            uw = spr->uea_write != NULL && spr->uea_write != SPR_NOACCESS;
            ur = spr->uea_read != NULL && spr->uea_read != SPR_NOACCESS;
#if !defined(CONFIG_USER_ONLY)
            sw = spr->oea_write != NULL && spr->oea_write != SPR_NOACCESS;
            sr = spr->oea_read != NULL && spr->oea_read != SPR_NOACCESS;
            if (sw || sr || uw || ur) {
                printf("SPR: %4d (%03x) %-8s s%c%c u%c%c\n",
                       (i << 5) | j, (i << 5) | j, spr->name,
                       sw ? 'w' : '-', sr ? 'r' : '-',
                       uw ? 'w' : '-', ur ? 'r' : '-');
            }
#else
            if (uw || ur) {
                printf("SPR: %4d (%03x) %-8s u%c%c\n",
                       (i << 5) | j, (i << 5) | j, spr->name,
                       uw ? 'w' : '-', ur ? 'r' : '-');
            }
#endif
        }
    }
    fflush(stdout);
    fflush(stderr);
}
#endif

/*****************************************************************************/
#include <stdlib.h>
#include <string.h>

int fflush (FILE *stream);

/* Opcode types */
enum {
    PPC_DIRECT   = 0, /* Opcode routine        */
    PPC_INDIRECT = 1, /* Indirect opcode table */
};

static inline int is_indirect_opcode (void *handler)
{
    return ((unsigned long)handler & 0x03) == PPC_INDIRECT;
}

static inline opc_handler_t **ind_table(void *handler)
{
    return (opc_handler_t **)((unsigned long)handler & ~3);
}

/* Instruction table creation */
/* Opcodes tables creation */
static void fill_new_table (opc_handler_t **table, int len)
{
    int i;

    for (i = 0; i < len; i++)
        table[i] = &invalid_handler;
}

static int create_new_table (opc_handler_t **table, unsigned char idx)
{
    opc_handler_t **tmp;

    tmp = malloc(0x20 * sizeof(opc_handler_t));
    if (tmp == NULL)
        return -1;
    fill_new_table(tmp, 0x20);
    table[idx] = (opc_handler_t *)((unsigned long)tmp | PPC_INDIRECT);

    return 0;
}

static int insert_in_table (opc_handler_t **table, unsigned char idx,
                            opc_handler_t *handler)
{
    if (table[idx] != &invalid_handler)
        return -1;
    table[idx] = handler;

    return 0;
}

static int register_direct_insn (opc_handler_t **ppc_opcodes,
                                 unsigned char idx, opc_handler_t *handler)
{
    if (insert_in_table(ppc_opcodes, idx, handler) < 0) {
        printf("*** ERROR: opcode %02x already assigned in main "
               "opcode table\n", idx);
        return -1;
    }

    return 0;
}

static int register_ind_in_table (opc_handler_t **table,
                                  unsigned char idx1, unsigned char idx2,
                                  opc_handler_t *handler)
{
    if (table[idx1] == &invalid_handler) {
        if (create_new_table(table, idx1) < 0) {
            printf("*** ERROR: unable to create indirect table "
                   "idx=%02x\n", idx1);
            return -1;
        }
    } else {
        if (!is_indirect_opcode(table[idx1])) {
            printf("*** ERROR: idx %02x already assigned to a direct "
                   "opcode\n", idx1);
            return -1;
        }
    }
    if (handler != NULL &&
        insert_in_table(ind_table(table[idx1]), idx2, handler) < 0) {
        printf("*** ERROR: opcode %02x already assigned in "
               "opcode table %02x\n", idx2, idx1);
        return -1;
    }

    return 0;
}

static int register_ind_insn (opc_handler_t **ppc_opcodes,
                              unsigned char idx1, unsigned char idx2,
                              opc_handler_t *handler)
{
    int ret;

    ret = register_ind_in_table(ppc_opcodes, idx1, idx2, handler);

    return ret;
}

static int register_dblind_insn (opc_handler_t **ppc_opcodes,
                                 unsigned char idx1, unsigned char idx2,
                                 unsigned char idx3, opc_handler_t *handler)
{
    if (register_ind_in_table(ppc_opcodes, idx1, idx2, NULL) < 0) {
        printf("*** ERROR: unable to join indirect table idx "
               "[%02x-%02x]\n", idx1, idx2);
        return -1;
    }
    if (register_ind_in_table(ind_table(ppc_opcodes[idx1]), idx2, idx3,
                              handler) < 0) {
        printf("*** ERROR: unable to insert opcode "
               "[%02x-%02x-%02x]\n", idx1, idx2, idx3);
        return -1;
    }

    return 0;
}

static int register_insn (opc_handler_t **ppc_opcodes, opcode_t *insn)
{
    if (insn->opc2 != 0xFF) {
        if (insn->opc3 != 0xFF) {
            if (register_dblind_insn(ppc_opcodes, insn->opc1, insn->opc2,
                                     insn->opc3, &insn->handler) < 0)
                return -1;
        } else {
            if (register_ind_insn(ppc_opcodes, insn->opc1,
                                  insn->opc2, &insn->handler) < 0)
                return -1;
        }
    } else {
        if (register_direct_insn(ppc_opcodes, insn->opc1, &insn->handler) < 0)
            return -1;
    }

    return 0;
}

static int test_opcode_table (opc_handler_t **table, int len)
{
    int i, count, tmp;

    for (i = 0, count = 0; i < len; i++) {
        /* Consistency fixup */
        if (table[i] == NULL)
            table[i] = &invalid_handler;
        if (table[i] != &invalid_handler) {
            if (is_indirect_opcode(table[i])) {
                tmp = test_opcode_table(ind_table(table[i]), 0x20);
                if (tmp == 0) {
                    free(table[i]);
                    table[i] = &invalid_handler;
                } else {
                    count++;
                }
            } else {
                count++;
            }
        }
    }

    return count;
}

static void fix_opcode_tables (opc_handler_t **ppc_opcodes)
{
    if (test_opcode_table(ppc_opcodes, 0x40) == 0)
        printf("*** WARNING: no opcode defined !\n");
}

/*****************************************************************************/
static int create_ppc_opcodes (CPUPPCState *env, ppc_def_t *def)
{
    opcode_t *opc, *start, *end;

    fill_new_table(env->opcodes, 0x40);
    if (&opc_start < &opc_end) {
        start = &opc_start;
        end = &opc_end;
    } else {
        start = &opc_end;
        end = &opc_start;
    }
    for (opc = start + 1; opc != end; opc++) {
        if ((opc->handler.type & def->insns_flags) != 0) {
            if (register_insn(env->opcodes, opc) < 0) {
                printf("*** ERROR initializing PowerPC instruction "
                       "0x%02x 0x%02x 0x%02x\n", opc->opc1, opc->opc2,
                       opc->opc3);
                return -1;
            }
        }
    }
    fix_opcode_tables(env->opcodes);
    fflush(stdout);
    fflush(stderr);

    return 0;
}

#if defined(PPC_DUMP_CPU)
static int dump_ppc_insns (CPUPPCState *env)
{
    opc_handler_t **table, *handler;
    uint8_t opc1, opc2, opc3;

    printf("Instructions set:\n");
    /* opc1 is 6 bits long */
    for (opc1 = 0x00; opc1 < 0x40; opc1++) {
        table = env->opcodes;
        handler = table[opc1];
        if (is_indirect_opcode(handler)) {
            /* opc2 is 5 bits long */
            for (opc2 = 0; opc2 < 0x20; opc2++) {
                table = env->opcodes;
                handler = env->opcodes[opc1];
                table = ind_table(handler);
                handler = table[opc2];
                if (is_indirect_opcode(handler)) {
                    table = ind_table(handler);
                    /* opc3 is 5 bits long */
                    for (opc3 = 0; opc3 < 0x20; opc3++) {
                        handler = table[opc3];
                        if (handler->handler != &gen_invalid) {
                            printf("INSN: %02x %02x %02x (%02d %04d) : %s\n",
                                   opc1, opc2, opc3, opc1, (opc3 << 5) | opc2,
                                   handler->oname);
                        }
                    }
                } else {
                    if (handler->handler != &gen_invalid) {
                        printf("INSN: %02x %02x -- (%02d %04d) : %s\n",
                               opc1, opc2, opc1, opc2, handler->oname);
                    }
                }
            }
        } else {
            if (handler->handler != &gen_invalid) {
                printf("INSN: %02x -- -- (%02d ----) : %s\n",
                       opc1, opc1, handler->oname);
            }
        }
    }
}
#endif

int cpu_ppc_register (CPUPPCState *env, ppc_def_t *def)
{
    env->msr_mask = def->msr_mask;
    env->mmu_model = def->mmu_model;
    env->excp_model = def->excp_model;
    env->bus_model = def->bus_model;
    env->bfd_mach = def->bfd_mach;
    if (create_ppc_opcodes(env, def) < 0)
        return -1;
    init_ppc_proc(env, def);
#if defined(PPC_DUMP_CPU)
    {
        const unsigned char *mmu_model, *excp_model, *bus_model;
        switch (env->mmu_model) {
        case POWERPC_MMU_32B:
            mmu_model = "PowerPC 32";
            break;
        case POWERPC_MMU_64B:
            mmu_model = "PowerPC 64";
            break;
        case POWERPC_MMU_601:
            mmu_model = "PowerPC 601";
            break;
        case POWERPC_MMU_SOFT_6xx:
            mmu_model = "PowerPC 6xx/7xx with software driven TLBs";
            break;
        case POWERPC_MMU_SOFT_74xx:
            mmu_model = "PowerPC 74xx with software driven TLBs";
            break;
        case POWERPC_MMU_SOFT_4xx:
            mmu_model = "PowerPC 4xx with software driven TLBs";
            break;
        case POWERPC_MMU_SOFT_4xx_Z:
            mmu_model = "PowerPC 4xx with software driven TLBs "
                "and zones protections";
            break;
        case POWERPC_MMU_REAL_4xx:
            mmu_model = "PowerPC 4xx real mode only";
            break;
        case POWERPC_MMU_BOOKE:
            mmu_model = "PowerPC BookE";
            break;
        case POWERPC_MMU_BOOKE_FSL:
            mmu_model = "PowerPC BookE FSL";
            break;
        case POWERPC_MMU_64BRIDGE:
            mmu_model = "PowerPC 64 bridge";
            break;
        default:
            mmu_model = "Unknown or invalid";
            break;
        }
        switch (env->excp_model) {
        case POWERPC_EXCP_STD:
            excp_model = "PowerPC";
            break;
        case POWERPC_EXCP_40x:
            excp_model = "PowerPC 40x";
            break;
        case POWERPC_EXCP_601:
            excp_model = "PowerPC 601";
            break;
        case POWERPC_EXCP_602:
            excp_model = "PowerPC 602";
            break;
        case POWERPC_EXCP_603:
            excp_model = "PowerPC 603";
            break;
        case POWERPC_EXCP_603E:
            excp_model = "PowerPC 603e";
            break;
        case POWERPC_EXCP_604:
            excp_model = "PowerPC 604";
            break;
        case POWERPC_EXCP_7x0:
            excp_model = "PowerPC 740/750";
            break;
        case POWERPC_EXCP_7x5:
            excp_model = "PowerPC 745/755";
            break;
        case POWERPC_EXCP_74xx:
            excp_model = "PowerPC 74xx";
            break;
        case POWERPC_EXCP_970:
            excp_model = "PowerPC 970";
            break;
        case POWERPC_EXCP_BOOKE:
            excp_model = "PowerPC BookE";
            break;
        default:
            excp_model = "Unknown or invalid";
            break;
        }
        switch (env->bus_model) {
        case PPC_FLAGS_INPUT_6xx:
            bus_model = "PowerPC 6xx";
            break;
        case PPC_FLAGS_INPUT_BookE:
            bus_model = "PowerPC BookE";
            break;
        case PPC_FLAGS_INPUT_405:
            bus_model = "PowerPC 405";
            break;
        case PPC_FLAGS_INPUT_970:
            bus_model = "PowerPC 970";
            break;
        case PPC_FLAGS_INPUT_401:
            bus_model = "PowerPC 401/403";
            break;
        default:
            bus_model = "Unknown or invalid";
            break;
        }
        printf("PowerPC %-12s : PVR %08x MSR %016" PRIx64 "\n"
               "    MMU model        : %s\n",
               def->name, def->pvr, def->msr_mask, mmu_model);
        if (env->tlb != NULL) {
            printf("                       %d %s TLB in %d ways\n",
                   env->nb_tlb, env->id_tlbs ? "splitted" : "merged",
                   env->nb_ways);
        }
        printf("    Exceptions model : %s\n"
               "    Bus model        : %s\n",
               excp_model, bus_model);
    }
    dump_ppc_insns(env);
    dump_ppc_sprs(env);
    fflush(stdout);
#endif

    return 0;
}

int ppc_find_by_name (const unsigned char *name, ppc_def_t **def)
{
    int i, max, ret;

    ret = -1;
    *def = NULL;
    max = sizeof(ppc_defs) / sizeof(ppc_def_t);
    for (i = 0; i < max; i++) {
        if (strcasecmp(name, ppc_defs[i].name) == 0) {
            *def = &ppc_defs[i];
            ret = 0;
            break;
        }
    }

    return ret;
}

int ppc_find_by_pvr (uint32_t pvr, ppc_def_t **def)
{
    int i, max, ret;

    ret = -1;
    *def = NULL;
    max = sizeof(ppc_defs) / sizeof(ppc_def_t);
    for (i = 0; i < max; i++) {
        if ((pvr & ppc_defs[i].pvr_mask) ==
            (ppc_defs[i].pvr & ppc_defs[i].pvr_mask)) {
            *def = &ppc_defs[i];
            ret = 0;
            break;
        }
    }

    return ret;
}

void ppc_cpu_list (FILE *f, int (*cpu_fprintf)(FILE *f, const char *fmt, ...))
{
    int i, max;

    max = sizeof(ppc_defs) / sizeof(ppc_def_t);
    for (i = 0; i < max; i++) {
        (*cpu_fprintf)(f, "PowerPC %-16s PVR %08x\n",
                       ppc_defs[i].name, ppc_defs[i].pvr);
    }
}
