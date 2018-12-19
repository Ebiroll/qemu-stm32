/*
 * Tiny Code Generator for QEMU
 *
 * Copyright (c) 2018 SiFive, Inc
 * Copyright (c) 2008-2009 Arnaud Patard <arnaud.patard@rtp-net.org>
 * Copyright (c) 2009 Aurelien Jarno <aurelien@aurel32.net>
 * Copyright (c) 2008 Fabrice Bellard
 *
 * Based on i386/tcg-target.c and mips/tcg-target.c
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

#include "tcg-pool.inc.c"

#ifdef CONFIG_DEBUG_TCG
static const char * const tcg_target_reg_names[TCG_TARGET_NB_REGS] = {
    "zero",
    "ra",
    "sp",
    "gp",
    "tp",
    "t0",
    "t1",
    "t2",
    "s0",
    "s1",
    "a0",
    "a1",
    "a2",
    "a3",
    "a4",
    "a5",
    "a6",
    "a7",
    "s2",
    "s3",
    "s4",
    "s5",
    "s6",
    "s7",
    "s8",
    "s9",
    "s10",
    "s11",
    "t3",
    "t4",
    "t5",
    "t6"
};
#endif

static const int tcg_target_reg_alloc_order[] = {
    /* Call saved registers */
    /* TCG_REG_S0 reservered for TCG_AREG0 */
    TCG_REG_S1,
    TCG_REG_S2,
    TCG_REG_S3,
    TCG_REG_S4,
    TCG_REG_S5,
    TCG_REG_S6,
    TCG_REG_S7,
    TCG_REG_S8,
    TCG_REG_S9,
    TCG_REG_S10,
    TCG_REG_S11,

    /* Call clobbered registers */
    TCG_REG_T0,
    TCG_REG_T1,
    TCG_REG_T2,
    TCG_REG_T3,
    TCG_REG_T4,
    TCG_REG_T5,
    TCG_REG_T6,

    /* Argument registers */
    TCG_REG_A0,
    TCG_REG_A1,
    TCG_REG_A2,
    TCG_REG_A3,
    TCG_REG_A4,
    TCG_REG_A5,
    TCG_REG_A6,
    TCG_REG_A7,
};

static const int tcg_target_call_iarg_regs[] = {
    TCG_REG_A0,
    TCG_REG_A1,
    TCG_REG_A2,
    TCG_REG_A3,
    TCG_REG_A4,
    TCG_REG_A5,
    TCG_REG_A6,
    TCG_REG_A7,
};

static const int tcg_target_call_oarg_regs[] = {
    TCG_REG_A0,
    TCG_REG_A1,
};

#define TCG_CT_CONST_ZERO  0x100
#define TCG_CT_CONST_S12   0x200
#define TCG_CT_CONST_N12   0x400
#define TCG_CT_CONST_M12   0x800

static inline tcg_target_long sextreg(tcg_target_long val, int pos, int len)
{
    if (TCG_TARGET_REG_BITS == 32) {
        return sextract32(val, pos, len);
    } else {
        return sextract64(val, pos, len);
    }
}

/* parse target specific constraints */
static const char *target_parse_constraint(TCGArgConstraint *ct,
                                           const char *ct_str, TCGType type)
{
    switch (*ct_str++) {
    case 'r':
        ct->ct |= TCG_CT_REG;
        ct->u.regs = 0xffffffff;
        break;
    case 'L':
        /* qemu_ld/qemu_st constraint */
        ct->ct |= TCG_CT_REG;
        ct->u.regs = 0xffffffff;
        /* qemu_ld/qemu_st uses TCG_REG_TMP0 */
#if defined(CONFIG_SOFTMMU)
        tcg_regset_reset_reg(ct->u.regs, tcg_target_call_iarg_regs[0]);
        tcg_regset_reset_reg(ct->u.regs, tcg_target_call_iarg_regs[1]);
        tcg_regset_reset_reg(ct->u.regs, tcg_target_call_iarg_regs[2]);
        tcg_regset_reset_reg(ct->u.regs, tcg_target_call_iarg_regs[3]);
        tcg_regset_reset_reg(ct->u.regs, tcg_target_call_iarg_regs[4]);
#endif
        break;
    case 'I':
        ct->ct |= TCG_CT_CONST_S12;
        break;
    case 'N':
        ct->ct |= TCG_CT_CONST_N12;
        break;
    case 'M':
        ct->ct |= TCG_CT_CONST_M12;
        break;
    case 'Z':
        /* we can use a zero immediate as a zero register argument. */
        ct->ct |= TCG_CT_CONST_ZERO;
        break;
    default:
        return NULL;
    }
    return ct_str;
}

/* test if a constant matches the constraint */
static int tcg_target_const_match(tcg_target_long val, TCGType type,
                                  const TCGArgConstraint *arg_ct)
{
    int ct = arg_ct->ct;
    if (ct & TCG_CT_CONST) {
        return 1;
    }
    if ((ct & TCG_CT_CONST_ZERO) && val == 0) {
        return 1;
    }
    if ((ct & TCG_CT_CONST_S12) && val == sextreg(val, 0, 12)) {
        return 1;
    }
    if ((ct & TCG_CT_CONST_N12) && -val == sextreg(-val, 0, 12)) {
        return 1;
    }
    if ((ct & TCG_CT_CONST_M12) && val >= -0xfff && val <= 0xfff) {
        return 1;
    }
    return 0;
}

/*
 * RISC-V Base ISA opcodes (IM)
 */

typedef enum {
    OPC_ADD = 0x33,
    OPC_ADDI = 0x13,
    OPC_AND = 0x7033,
    OPC_ANDI = 0x7013,
    OPC_AUIPC = 0x17,
    OPC_BEQ = 0x63,
    OPC_BGE = 0x5063,
    OPC_BGEU = 0x7063,
    OPC_BLT = 0x4063,
    OPC_BLTU = 0x6063,
    OPC_BNE = 0x1063,
    OPC_DIV = 0x2004033,
    OPC_DIVU = 0x2005033,
    OPC_JAL = 0x6f,
    OPC_JALR = 0x67,
    OPC_LB = 0x3,
    OPC_LBU = 0x4003,
    OPC_LD = 0x3003,
    OPC_LH = 0x1003,
    OPC_LHU = 0x5003,
    OPC_LUI = 0x37,
    OPC_LW = 0x2003,
    OPC_LWU = 0x6003,
    OPC_MUL = 0x2000033,
    OPC_MULH = 0x2001033,
    OPC_MULHSU = 0x2002033,
    OPC_MULHU = 0x2003033,
    OPC_OR = 0x6033,
    OPC_ORI = 0x6013,
    OPC_REM = 0x2006033,
    OPC_REMU = 0x2007033,
    OPC_SB = 0x23,
    OPC_SD = 0x3023,
    OPC_SH = 0x1023,
    OPC_SLL = 0x1033,
    OPC_SLLI = 0x1013,
    OPC_SLT = 0x2033,
    OPC_SLTI = 0x2013,
    OPC_SLTIU = 0x3013,
    OPC_SLTU = 0x3033,
    OPC_SRA = 0x40005033,
    OPC_SRAI = 0x40005013,
    OPC_SRL = 0x5033,
    OPC_SRLI = 0x5013,
    OPC_SUB = 0x40000033,
    OPC_SW = 0x2023,
    OPC_XOR = 0x4033,
    OPC_XORI = 0x4013,

#if TCG_TARGET_REG_BITS == 64
    OPC_ADDIW = 0x1b,
    OPC_ADDW = 0x3b,
    OPC_DIVUW = 0x200503b,
    OPC_DIVW = 0x200403b,
    OPC_MULW = 0x200003b,
    OPC_REMUW = 0x200703b,
    OPC_REMW = 0x200603b,
    OPC_SLLIW = 0x101b,
    OPC_SLLW = 0x103b,
    OPC_SRAIW = 0x4000501b,
    OPC_SRAW = 0x4000503b,
    OPC_SRLIW = 0x501b,
    OPC_SRLW = 0x503b,
    OPC_SUBW = 0x4000003b,
#else
    /* Simplify code throughout by defining aliases for RV32.  */
    OPC_ADDIW = OPC_ADDI,
    OPC_ADDW = OPC_ADD,
    OPC_DIVUW = OPC_DIVU,
    OPC_DIVW = OPC_DIV,
    OPC_MULW = OPC_MUL,
    OPC_REMUW = OPC_REMU,
    OPC_REMW = OPC_REM,
    OPC_SLLIW = OPC_SLLI,
    OPC_SLLW = OPC_SLL,
    OPC_SRAIW = OPC_SRAI,
    OPC_SRAW = OPC_SRA,
    OPC_SRLIW = OPC_SRLI,
    OPC_SRLW = OPC_SRL,
    OPC_SUBW = OPC_SUB,
#endif

    OPC_FENCE = 0x0000000f,
} RISCVInsn;

/*
 * RISC-V immediate and instruction encoders (excludes 16-bit RVC)
 */

/* Type-R */

static int32_t encode_r(RISCVInsn opc, TCGReg rd, TCGReg rs1, TCGReg rs2)
{
    return opc | (rd & 0x1f) << 7 | (rs1 & 0x1f) << 15 | (rs2 & 0x1f) << 20;
}

/* Type-I */

static int32_t encode_imm12(uint32_t imm)
{
    return (imm & 0xfff) << 20;
}

static int32_t encode_i(RISCVInsn opc, TCGReg rd, TCGReg rs1, uint32_t imm)
{
    return opc | (rd & 0x1f) << 7 | (rs1 & 0x1f) << 15 | encode_imm12(imm);
}

/* Type-S */

static int32_t encode_simm12(uint32_t imm)
{
    int32_t ret = 0;

    ret |= (imm & 0xFE0) << 20;
    ret |= (imm & 0x1F) << 7;

    return ret;
}

static int32_t encode_s(RISCVInsn opc, TCGReg rs1, TCGReg rs2, uint32_t imm)
{
    return opc | (rs1 & 0x1f) << 15 | (rs2 & 0x1f) << 20 | encode_simm12(imm);
}

/* Type-SB */

static int32_t encode_sbimm12(uint32_t imm)
{
    int32_t ret = 0;

    ret |= (imm & 0x1000) << 19;
    ret |= (imm & 0x7e0) << 20;
    ret |= (imm & 0x1e) << 7;
    ret |= (imm & 0x800) >> 4;

    return ret;
}

static int32_t encode_sb(RISCVInsn opc, TCGReg rs1, TCGReg rs2, uint32_t imm)
{
    return opc | (rs1 & 0x1f) << 15 | (rs2 & 0x1f) << 20 | encode_sbimm12(imm);
}

/* Type-U */

static int32_t encode_uimm20(uint32_t imm)
{
    return imm & 0xfffff000;
}

static int32_t encode_u(RISCVInsn opc, TCGReg rd, uint32_t imm)
{
    return opc | (rd & 0x1f) << 7 | encode_uimm20(imm);
}

/* Type-UJ */

static int32_t encode_ujimm20(uint32_t imm)
{
    int32_t ret = 0;

    ret |= (imm & 0x0007fe) << (21 - 1);
    ret |= (imm & 0x000800) << (20 - 11);
    ret |= (imm & 0x0ff000) << (12 - 12);
    ret |= (imm & 0x100000) << (31 - 20);

    return ret;
}

static int32_t encode_uj(RISCVInsn opc, TCGReg rd, uint32_t imm)
{
    return opc | (rd & 0x1f) << 7 | encode_ujimm20(imm);
}

/*
 * RISC-V instruction emitters
 */

static void tcg_out_opc_reg(TCGContext *s, RISCVInsn opc,
                            TCGReg rd, TCGReg rs1, TCGReg rs2)
{
    tcg_out32(s, encode_r(opc, rd, rs1, rs2));
}

static void tcg_out_opc_imm(TCGContext *s, RISCVInsn opc,
                            TCGReg rd, TCGReg rs1, TCGArg imm)
{
    tcg_out32(s, encode_i(opc, rd, rs1, imm));
}

static void tcg_out_opc_store(TCGContext *s, RISCVInsn opc,
                              TCGReg rs1, TCGReg rs2, uint32_t imm)
{
    tcg_out32(s, encode_s(opc, rs1, rs2, imm));
}

static void tcg_out_opc_branch(TCGContext *s, RISCVInsn opc,
                               TCGReg rs1, TCGReg rs2, uint32_t imm)
{
    tcg_out32(s, encode_sb(opc, rs1, rs2, imm));
}

static void tcg_out_opc_upper(TCGContext *s, RISCVInsn opc,
                              TCGReg rd, uint32_t imm)
{
    tcg_out32(s, encode_u(opc, rd, imm));
}

static void tcg_out_opc_jump(TCGContext *s, RISCVInsn opc,
                             TCGReg rd, uint32_t imm)
{
    tcg_out32(s, encode_uj(opc, rd, imm));
}

static void tcg_out_nop_fill(tcg_insn_unit *p, int count)
{
    int i;
    for (i = 0; i < count; ++i) {
        p[i] = encode_i(OPC_ADDI, TCG_REG_ZERO, TCG_REG_ZERO, 0);
    }
}

/*
 * Relocations
 */

static bool reloc_sbimm12(tcg_insn_unit *code_ptr, tcg_insn_unit *target)
{
    intptr_t offset = (intptr_t)target - (intptr_t)code_ptr;

    if (offset == sextreg(offset, 1, 12) << 1) {
        code_ptr[0] |= encode_sbimm12(offset);
        return true;
    }

    return false;
}

static bool reloc_jimm20(tcg_insn_unit *code_ptr, tcg_insn_unit *target)
{
    intptr_t offset = (intptr_t)target - (intptr_t)code_ptr;

    if (offset == sextreg(offset, 1, 20) << 1) {
        code_ptr[0] |= encode_ujimm20(offset);
        return true;
    }

    return false;
}

static bool reloc_call(tcg_insn_unit *code_ptr, tcg_insn_unit *target)
{
    intptr_t offset = (intptr_t)target - (intptr_t)code_ptr;
    int32_t lo = sextreg(offset, 0, 12);
    int32_t hi = offset - lo;

    if (offset == hi + lo) {
        code_ptr[0] |= encode_uimm20(hi);
        code_ptr[1] |= encode_imm12(lo);
        return true;
    }

    return false;
}

static bool patch_reloc(tcg_insn_unit *code_ptr, int type,
                        intptr_t value, intptr_t addend)
{
    uint32_t insn = *code_ptr;
    intptr_t diff;
    bool short_jmp;

    tcg_debug_assert(addend == 0);

    switch (type) {
    case R_RISCV_BRANCH:
        diff = value - (uintptr_t)code_ptr;
        short_jmp = diff == sextreg(diff, 0, 12);
        if (short_jmp) {
            return reloc_sbimm12(code_ptr, (tcg_insn_unit *)value);
        } else {
            /* Invert the condition */
            insn = insn ^ (1 << 12);
            /* Clear the offset */
            insn &= 0x01fff07f;
            /* Set the offset to the PC + 8 */
            insn |= encode_sbimm12(8);

            /* Move forward */
            code_ptr[0] = insn;

            /* Overwrite the NOP with jal x0,value */
            diff = value - (uintptr_t)(code_ptr + 1);
            insn = encode_uj(OPC_JAL, TCG_REG_ZERO, diff);
            code_ptr[1] = insn;

            return true;
        }
        break;
    case R_RISCV_JAL:
        return reloc_jimm20(code_ptr, (tcg_insn_unit *)value);
        break;
    case R_RISCV_CALL:
        return reloc_call(code_ptr, (tcg_insn_unit *)value);
        break;
    default:
        tcg_abort();
    }
}

/*
 * TCG intrinsics
 */

static void tcg_out_mov(TCGContext *s, TCGType type, TCGReg ret, TCGReg arg)
{
    if (ret == arg) {
        return;
    }
    switch (type) {
    case TCG_TYPE_I32:
    case TCG_TYPE_I64:
        tcg_out_opc_imm(s, OPC_ADDI, ret, arg, 0);
        break;
    default:
        g_assert_not_reached();
    }
}

static void tcg_out_movi(TCGContext *s, TCGType type, TCGReg rd,
                         tcg_target_long val)
{
    tcg_target_long lo, hi, tmp;
    int shift, ret;

    if (TCG_TARGET_REG_BITS == 64 && type == TCG_TYPE_I32) {
        val = (int32_t)val;
    }

    lo = sextreg(val, 0, 12);
    if (val == lo) {
        tcg_out_opc_imm(s, OPC_ADDI, rd, TCG_REG_ZERO, lo);
        return;
    }

    hi = val - lo;
    if (TCG_TARGET_REG_BITS == 32 || val == (int32_t)val) {
        tcg_out_opc_upper(s, OPC_LUI, rd, hi);
        if (lo != 0) {
            tcg_out_opc_imm(s, OPC_ADDIW, rd, rd, lo);
        }
        return;
    }

    /* We can only be here if TCG_TARGET_REG_BITS != 32 */
    tmp = tcg_pcrel_diff(s, (void *)val);
    if (tmp == (int32_t)tmp) {
        tcg_out_opc_upper(s, OPC_AUIPC, rd, 0);
        tcg_out_opc_imm(s, OPC_ADDI, rd, rd, 0);
        ret = reloc_call(s->code_ptr - 2, (tcg_insn_unit *)val);
        tcg_debug_assert(ret == true);
        return;
    }

    /* Look for a single 20-bit section.  */
    shift = ctz64(val);
    tmp = val >> shift;
    if (tmp == sextreg(tmp, 0, 20)) {
        tcg_out_opc_upper(s, OPC_LUI, rd, tmp << 12);
        if (shift > 12) {
            tcg_out_opc_imm(s, OPC_SLLI, rd, rd, shift - 12);
        } else {
            tcg_out_opc_imm(s, OPC_SRAI, rd, rd, 12 - shift);
        }
        return;
    }

    /* Look for a few high zero bits, with lots of bits set in the middle.  */
    shift = clz64(val);
    tmp = val << shift;
    if (tmp == sextreg(tmp, 12, 20) << 12) {
        tcg_out_opc_upper(s, OPC_LUI, rd, tmp);
        tcg_out_opc_imm(s, OPC_SRLI, rd, rd, shift);
        return;
    } else if (tmp == sextreg(tmp, 0, 12)) {
        tcg_out_opc_imm(s, OPC_ADDI, rd, TCG_REG_ZERO, tmp);
        tcg_out_opc_imm(s, OPC_SRLI, rd, rd, shift);
        return;
    }

    /* Drop into the constant pool.  */
    new_pool_label(s, val, R_RISCV_CALL, s->code_ptr, 0);
    tcg_out_opc_upper(s, OPC_AUIPC, rd, 0);
    tcg_out_opc_imm(s, OPC_LD, rd, rd, 0);
}

static void tcg_out_ext8u(TCGContext *s, TCGReg ret, TCGReg arg)
{
    tcg_out_opc_imm(s, OPC_ANDI, ret, arg, 0xff);
}

static void tcg_out_ext16u(TCGContext *s, TCGReg ret, TCGReg arg)
{
    tcg_out_opc_imm(s, OPC_SLLIW, ret, arg, 16);
    tcg_out_opc_imm(s, OPC_SRLIW, ret, ret, 16);
}

static void tcg_out_ext32u(TCGContext *s, TCGReg ret, TCGReg arg)
{
    tcg_out_opc_imm(s, OPC_SLLI, ret, arg, 32);
    tcg_out_opc_imm(s, OPC_SRLI, ret, ret, 32);
}

static void tcg_out_ext8s(TCGContext *s, TCGReg ret, TCGReg arg)
{
    tcg_out_opc_imm(s, OPC_SLLIW, ret, arg, 24);
    tcg_out_opc_imm(s, OPC_SRAIW, ret, ret, 24);
}

static void tcg_out_ext16s(TCGContext *s, TCGReg ret, TCGReg arg)
{
    tcg_out_opc_imm(s, OPC_SLLIW, ret, arg, 16);
    tcg_out_opc_imm(s, OPC_SRAIW, ret, ret, 16);
}

static void tcg_out_ext32s(TCGContext *s, TCGReg ret, TCGReg arg)
{
    tcg_out_opc_imm(s, OPC_ADDIW, ret, arg, 0);
}

static void tcg_out_ldst(TCGContext *s, RISCVInsn opc, TCGReg data,
                         TCGReg addr, intptr_t offset)
{
    intptr_t imm12 = sextreg(offset, 0, 12);

    if (offset != imm12) {
        intptr_t diff = offset - (uintptr_t)s->code_ptr;

        if (addr == TCG_REG_ZERO && diff == (int32_t)diff) {
            imm12 = sextreg(diff, 0, 12);
            tcg_out_opc_upper(s, OPC_AUIPC, TCG_REG_TMP2, diff - imm12);
        } else {
            tcg_out_movi(s, TCG_TYPE_PTR, TCG_REG_TMP2, offset - imm12);
            if (addr != TCG_REG_ZERO) {
                tcg_out_opc_reg(s, OPC_ADD, TCG_REG_TMP2, TCG_REG_TMP2, addr);
            }
        }
        addr = TCG_REG_TMP2;
    }

    switch (opc) {
    case OPC_SB:
    case OPC_SH:
    case OPC_SW:
    case OPC_SD:
        tcg_out_opc_store(s, opc, addr, data, imm12);
        break;
    case OPC_LB:
    case OPC_LBU:
    case OPC_LH:
    case OPC_LHU:
    case OPC_LW:
    case OPC_LWU:
    case OPC_LD:
        tcg_out_opc_imm(s, opc, data, addr, imm12);
        break;
    default:
        g_assert_not_reached();
    }
}

static void tcg_out_ld(TCGContext *s, TCGType type, TCGReg arg,
                       TCGReg arg1, intptr_t arg2)
{
    bool is32bit = (TCG_TARGET_REG_BITS == 32 || type == TCG_TYPE_I32);
    tcg_out_ldst(s, is32bit ? OPC_LW : OPC_LD, arg, arg1, arg2);
}

static void tcg_out_st(TCGContext *s, TCGType type, TCGReg arg,
                       TCGReg arg1, intptr_t arg2)
{
    bool is32bit = (TCG_TARGET_REG_BITS == 32 || type == TCG_TYPE_I32);
    tcg_out_ldst(s, is32bit ? OPC_SW : OPC_SD, arg, arg1, arg2);
}

static bool tcg_out_sti(TCGContext *s, TCGType type, TCGArg val,
                        TCGReg base, intptr_t ofs)
{
    if (val == 0) {
        tcg_out_st(s, type, TCG_REG_ZERO, base, ofs);
        return true;
    }
    return false;
}

static void tcg_out_addsub2(TCGContext *s,
                            TCGReg rl, TCGReg rh,
                            TCGReg al, TCGReg ah,
                            TCGArg bl, TCGArg bh,
                            bool cbl, bool cbh, bool is_sub, bool is32bit)
{
    const RISCVInsn opc_add = is32bit ? OPC_ADDW : OPC_ADD;
    const RISCVInsn opc_addi = is32bit ? OPC_ADDIW : OPC_ADDI;
    const RISCVInsn opc_sub = is32bit ? OPC_SUBW : OPC_SUB;
    TCGReg th = TCG_REG_TMP1;

    /* If we have a negative constant such that negating it would
       make the high part zero, we can (usually) eliminate one insn.  */
    if (cbl && cbh && bh == -1 && bl != 0) {
        bl = -bl;
        bh = 0;
        is_sub = !is_sub;
    }

    /* By operating on the high part first, we get to use the final
       carry operation to move back from the temporary.  */
    if (!cbh) {
        tcg_out_opc_reg(s, (is_sub ? opc_sub : opc_add), th, ah, bh);
    } else if (bh != 0 || ah == rl) {
        tcg_out_opc_imm(s, opc_addi, th, ah, (is_sub ? -bh : bh));
    } else {
        th = ah;
    }

    /* Note that tcg optimization should eliminate the bl == 0 case.  */
    if (is_sub) {
        if (cbl) {
            tcg_out_opc_imm(s, OPC_SLTIU, TCG_REG_TMP0, al, bl);
            tcg_out_opc_imm(s, opc_addi, rl, al, -bl);
        } else {
            tcg_out_opc_reg(s, OPC_SLTU, TCG_REG_TMP0, al, bl);
            tcg_out_opc_reg(s, opc_sub, rl, al, bl);
        }
        tcg_out_opc_reg(s, opc_sub, rh, th, TCG_REG_TMP0);
    } else {
        if (cbl) {
            tcg_out_opc_imm(s, opc_addi, rl, al, bl);
            tcg_out_opc_imm(s, OPC_SLTIU, TCG_REG_TMP0, rl, bl);
        } else if (rl == al && rl == bl) {
            tcg_out_opc_imm(s, OPC_SLTI, TCG_REG_TMP0, al, 0);
            tcg_out_opc_reg(s, opc_addi, rl, al, bl);
        } else {
            tcg_out_opc_reg(s, opc_add, rl, al, bl);
            tcg_out_opc_reg(s, OPC_SLTU, TCG_REG_TMP0,
                            rl, (rl == bl ? al : bl));
        }
        tcg_out_opc_reg(s, opc_add, rh, th, TCG_REG_TMP0);
    }
}

static const struct {
    RISCVInsn op;
    bool swap;
} tcg_brcond_to_riscv[] = {
    [TCG_COND_EQ] =  { OPC_BEQ,  false },
    [TCG_COND_NE] =  { OPC_BNE,  false },
    [TCG_COND_LT] =  { OPC_BLT,  false },
    [TCG_COND_GE] =  { OPC_BGE,  false },
    [TCG_COND_LE] =  { OPC_BGE,  true  },
    [TCG_COND_GT] =  { OPC_BLT,  true  },
    [TCG_COND_LTU] = { OPC_BLTU, false },
    [TCG_COND_GEU] = { OPC_BGEU, false },
    [TCG_COND_LEU] = { OPC_BGEU, true  },
    [TCG_COND_GTU] = { OPC_BLTU, true  }
};

static void tcg_out_brcond(TCGContext *s, TCGCond cond, TCGReg arg1,
                           TCGReg arg2, TCGLabel *l)
{
    RISCVInsn op = tcg_brcond_to_riscv[cond].op;

    tcg_debug_assert(op != 0);

    if (tcg_brcond_to_riscv[cond].swap) {
        TCGReg t = arg1;
        arg1 = arg2;
        arg2 = t;
    }

    if (l->has_value) {
        intptr_t diff = tcg_pcrel_diff(s, l->u.value_ptr);
        if (diff == sextreg(diff, 0, 12)) {
            tcg_out_opc_branch(s, op, arg1, arg2, diff);
        } else {
            /* Invert the conditional branch.  */
            tcg_out_opc_branch(s, op ^ (1 << 12), arg1, arg2, 8);
            tcg_out_opc_jump(s, OPC_JAL, TCG_REG_ZERO, diff - 4);
        }
    } else {
        tcg_out_reloc(s, s->code_ptr, R_RISCV_BRANCH, l, 0);
        tcg_out_opc_branch(s, op, arg1, arg2, 0);
        /* NOP to allow patching later */
        tcg_out_opc_imm(s, OPC_ADDI, TCG_REG_ZERO, TCG_REG_ZERO, 0);
    }
}

static void tcg_out_setcond(TCGContext *s, TCGCond cond, TCGReg ret,
                            TCGReg arg1, TCGReg arg2)
{
    switch (cond) {
    case TCG_COND_EQ:
        tcg_out_opc_reg(s, OPC_SUB, ret, arg1, arg2);
        tcg_out_opc_imm(s, OPC_SLTIU, ret, ret, 1);
        break;
    case TCG_COND_NE:
        tcg_out_opc_reg(s, OPC_SUB, ret, arg1, arg2);
        tcg_out_opc_reg(s, OPC_SLTU, ret, TCG_REG_ZERO, ret);
        break;
    case TCG_COND_LT:
        tcg_out_opc_reg(s, OPC_SLT, ret, arg1, arg2);
        break;
    case TCG_COND_GE:
        tcg_out_opc_reg(s, OPC_SLT, ret, arg1, arg2);
        tcg_out_opc_imm(s, OPC_XORI, ret, ret, 1);
        break;
    case TCG_COND_LE:
        tcg_out_opc_reg(s, OPC_SLT, ret, arg2, arg1);
        tcg_out_opc_imm(s, OPC_XORI, ret, ret, 1);
        break;
    case TCG_COND_GT:
        tcg_out_opc_reg(s, OPC_SLT, ret, arg2, arg1);
        break;
    case TCG_COND_LTU:
        tcg_out_opc_reg(s, OPC_SLTU, ret, arg1, arg2);
        break;
    case TCG_COND_GEU:
        tcg_out_opc_reg(s, OPC_SLTU, ret, arg1, arg2);
        tcg_out_opc_imm(s, OPC_XORI, ret, ret, 1);
        break;
    case TCG_COND_LEU:
        tcg_out_opc_reg(s, OPC_SLTU, ret, arg2, arg1);
        tcg_out_opc_imm(s, OPC_XORI, ret, ret, 1);
        break;
    case TCG_COND_GTU:
        tcg_out_opc_reg(s, OPC_SLTU, ret, arg2, arg1);
        break;
    default:
         g_assert_not_reached();
         break;
     }
}

static void tcg_out_brcond2(TCGContext *s, TCGCond cond, TCGReg al, TCGReg ah,
                            TCGReg bl, TCGReg bh, TCGLabel *l)
{
    /* todo */
    g_assert_not_reached();
}

static void tcg_out_setcond2(TCGContext *s, TCGCond cond, TCGReg ret,
                             TCGReg al, TCGReg ah, TCGReg bl, TCGReg bh)
{
    /* todo */
    g_assert_not_reached();
}

static inline void tcg_out_goto(TCGContext *s, tcg_insn_unit *target)
{
    ptrdiff_t offset = tcg_pcrel_diff(s, target);
    tcg_debug_assert(offset == sextreg(offset, 1, 20) << 1);
    tcg_out_opc_jump(s, OPC_JAL, TCG_REG_ZERO, offset);
}

static void tcg_out_call_int(TCGContext *s, tcg_insn_unit *arg, bool tail)
{
    TCGReg link = tail ? TCG_REG_ZERO : TCG_REG_RA;
    ptrdiff_t offset = tcg_pcrel_diff(s, arg);
    int ret;

    if (offset == sextreg(offset, 1, 20) << 1) {
        /* short jump: -2097150 to 2097152 */
        tcg_out_opc_jump(s, OPC_JAL, link, offset);
    } else if (TCG_TARGET_REG_BITS == 32 ||
        offset == sextreg(offset, 1, 31) << 1) {
        /* long jump: -2147483646 to 2147483648 */
        tcg_out_opc_upper(s, OPC_AUIPC, TCG_REG_TMP0, 0);
        tcg_out_opc_imm(s, OPC_JALR, link, TCG_REG_TMP0, 0);
        ret = reloc_call(s->code_ptr - 2, arg);\
        tcg_debug_assert(ret == true);
    } else if (TCG_TARGET_REG_BITS == 64) {
        /* far jump: 64-bit */
        tcg_target_long imm = sextreg((tcg_target_long)arg, 0, 12);
        tcg_target_long base = (tcg_target_long)arg - imm;
        tcg_out_movi(s, TCG_TYPE_PTR, TCG_REG_TMP0, base);
        tcg_out_opc_imm(s, OPC_JALR, link, TCG_REG_TMP0, imm);
    } else {
        g_assert_not_reached();
    }
}

static void tcg_out_call(TCGContext *s, tcg_insn_unit *arg)
{
    tcg_out_call_int(s, arg, false);
}

static void tcg_out_mb(TCGContext *s, TCGArg a0)
{
    tcg_insn_unit insn = OPC_FENCE;

    if (a0 & TCG_MO_LD_LD) {
        insn |= 0x02200000;
    }
    if (a0 & TCG_MO_ST_LD) {
        insn |= 0x01200000;
    }
    if (a0 & TCG_MO_LD_ST) {
        insn |= 0x02100000;
    }
    if (a0 & TCG_MO_ST_ST) {
        insn |= 0x02200000;
    }
    tcg_out32(s, insn);
}

/*
 * Load/store and TLB
 */

#if defined(CONFIG_SOFTMMU)
#include "tcg-ldst.inc.c"

/* helper signature: helper_ret_ld_mmu(CPUState *env, target_ulong addr,
 *                                     TCGMemOpIdx oi, uintptr_t ra)
 */
static void * const qemu_ld_helpers[16] = {
    [MO_UB]   = helper_ret_ldub_mmu,
    [MO_SB]   = helper_ret_ldsb_mmu,
    [MO_LEUW] = helper_le_lduw_mmu,
    [MO_LESW] = helper_le_ldsw_mmu,
    [MO_LEUL] = helper_le_ldul_mmu,
#if TCG_TARGET_REG_BITS == 64
    [MO_LESL] = helper_le_ldsl_mmu,
#endif
    [MO_LEQ]  = helper_le_ldq_mmu,
    [MO_BEUW] = helper_be_lduw_mmu,
    [MO_BESW] = helper_be_ldsw_mmu,
    [MO_BEUL] = helper_be_ldul_mmu,
#if TCG_TARGET_REG_BITS == 64
    [MO_BESL] = helper_be_ldsl_mmu,
#endif
    [MO_BEQ]  = helper_be_ldq_mmu,
};

/* helper signature: helper_ret_st_mmu(CPUState *env, target_ulong addr,
 *                                     uintxx_t val, TCGMemOpIdx oi,
 *                                     uintptr_t ra)
 */
static void * const qemu_st_helpers[16] = {
    [MO_UB]   = helper_ret_stb_mmu,
    [MO_LEUW] = helper_le_stw_mmu,
    [MO_LEUL] = helper_le_stl_mmu,
    [MO_LEQ]  = helper_le_stq_mmu,
    [MO_BEUW] = helper_be_stw_mmu,
    [MO_BEUL] = helper_be_stl_mmu,
    [MO_BEQ]  = helper_be_stq_mmu,
};

static void tcg_out_tlb_load(TCGContext *s, TCGReg addrl,
                             TCGReg addrh, TCGMemOpIdx oi,
                             tcg_insn_unit **label_ptr, bool is_load)
{
    TCGMemOp opc = get_memop(oi);
    unsigned s_bits = opc & MO_SIZE;
    unsigned a_bits = get_alignment_bits(opc);
    target_ulong mask;
    int mem_index = get_mmuidx(oi);
    int cmp_off
        = (is_load
           ? offsetof(CPUArchState, tlb_table[mem_index][0].addr_read)
           : offsetof(CPUArchState, tlb_table[mem_index][0].addr_write));
    int add_off = offsetof(CPUArchState, tlb_table[mem_index][0].addend);
    RISCVInsn load_cmp_op = (TARGET_LONG_BITS == 64 ? OPC_LD :
                             TCG_TARGET_REG_BITS == 64 ? OPC_LWU : OPC_LW);
    RISCVInsn load_add_op = TCG_TARGET_REG_BITS == 64 ? OPC_LD : OPC_LW;
    TCGReg base = TCG_AREG0;

    /* We don't support oversize guests */
    if (TCG_TARGET_REG_BITS < TARGET_LONG_BITS) {
        g_assert_not_reached();
    }

    /* We don't support unaligned accesses. */
    if (a_bits < s_bits) {
        a_bits = s_bits;
    }
    mask = (target_ulong)TARGET_PAGE_MASK | ((1 << a_bits) - 1);


    /* Compensate for very large offsets.  */
    if (add_off >= 0x1000) {
        int adj;
        base = TCG_REG_TMP2;
        if (cmp_off <= 2 * 0xfff) {
            adj = 0xfff;
            tcg_out_opc_imm(s, OPC_ADDI, base, TCG_AREG0, adj);
        } else {
            adj = cmp_off - sextreg(cmp_off, 0, 12);
            tcg_debug_assert(add_off - adj >= -0x1000
                             && add_off - adj < 0x1000);

            tcg_out_opc_upper(s, OPC_LUI, base, adj);
            tcg_out_opc_reg(s, OPC_ADD, base, base, TCG_AREG0);
        }
        add_off -= adj;
        cmp_off -= adj;
    }

    /* Extract the page index.  */
    if (CPU_TLB_BITS + CPU_TLB_ENTRY_BITS < 12) {
        tcg_out_opc_imm(s, OPC_SRLI, TCG_REG_TMP0, addrl,
                        TARGET_PAGE_BITS - CPU_TLB_ENTRY_BITS);
        tcg_out_opc_imm(s, OPC_ANDI, TCG_REG_TMP0, TCG_REG_TMP0,
                        MAKE_64BIT_MASK(CPU_TLB_ENTRY_BITS, CPU_TLB_BITS));
    } else if (TARGET_PAGE_BITS >= 12) {
        tcg_out_opc_upper(s, OPC_LUI, TCG_REG_TMP0,
                          MAKE_64BIT_MASK(TARGET_PAGE_BITS, CPU_TLB_BITS));
        tcg_out_opc_reg(s, OPC_AND, TCG_REG_TMP0, TCG_REG_TMP0, addrl);
        tcg_out_opc_imm(s, OPC_SRLI, TCG_REG_TMP0, TCG_REG_TMP0,
                        CPU_TLB_BITS - CPU_TLB_ENTRY_BITS);
    } else {
        tcg_out_opc_imm(s, OPC_SRLI, TCG_REG_TMP0, addrl, TARGET_PAGE_BITS);
        tcg_out_opc_imm(s, OPC_ANDI, TCG_REG_TMP0, TCG_REG_TMP0,
                        MAKE_64BIT_MASK(0, CPU_TLB_BITS));
        tcg_out_opc_imm(s, OPC_SLLI, TCG_REG_TMP0, TCG_REG_TMP0,
                        CPU_TLB_ENTRY_BITS);
    }

    /* Add that to the base address to index the tlb.  */
    tcg_out_opc_reg(s, OPC_ADD, TCG_REG_TMP2, base, TCG_REG_TMP0);
    base = TCG_REG_TMP2;

    /* Load the tlb comparator and the addend.  */
    tcg_out_ldst(s, load_cmp_op, TCG_REG_TMP0, base, cmp_off);
    tcg_out_ldst(s, load_add_op, TCG_REG_TMP2, base, add_off);

    /* Clear the non-page, non-alignment bits from the address.  */
    if (mask == sextreg(mask, 0, 12)) {
        tcg_out_opc_imm(s, OPC_ANDI, TCG_REG_TMP1, addrl, mask);
    } else {
        tcg_out_movi(s, TCG_TYPE_REG, TCG_REG_TMP1, mask);
        tcg_out_opc_reg(s, OPC_AND, TCG_REG_TMP1, TCG_REG_TMP1, addrl);
     }

    /* Compare masked address with the TLB entry. */
    label_ptr[0] = s->code_ptr;
    tcg_out_opc_branch(s, OPC_BNE, TCG_REG_TMP0, TCG_REG_TMP1, 0);
    /* NOP to allow patching later */
    tcg_out_opc_imm(s, OPC_ADDI, TCG_REG_ZERO, TCG_REG_ZERO, 0);
    /* TODO: Move this out of line
     * see:
     *   https://lists.nongnu.org/archive/html/qemu-devel/2018-11/msg02234.html
     */

    /* TLB Hit - translate address using addend.  */
    if (TCG_TARGET_REG_BITS > TARGET_LONG_BITS) {
        tcg_out_ext32u(s, TCG_REG_TMP0, addrl);
        addrl = TCG_REG_TMP0;
    }
    tcg_out_opc_reg(s, OPC_ADD, TCG_REG_TMP0, TCG_REG_TMP2, addrl);
}

static void add_qemu_ldst_label(TCGContext *s, int is_ld, TCGMemOpIdx oi,
                                TCGType ext,
                                TCGReg datalo, TCGReg datahi,
                                TCGReg addrlo, TCGReg addrhi,
                                void *raddr, tcg_insn_unit **label_ptr)
{
    TCGLabelQemuLdst *label = new_ldst_label(s);

    label->is_ld = is_ld;
    label->oi = oi;
    label->type = ext;
    label->datalo_reg = datalo;
    label->datahi_reg = datahi;
    label->addrlo_reg = addrlo;
    label->addrhi_reg = addrhi;
    label->raddr = raddr;
    label->label_ptr[0] = label_ptr[0];
}

static void tcg_out_qemu_ld_slow_path(TCGContext *s, TCGLabelQemuLdst *l)
{
    TCGMemOpIdx oi = l->oi;
    TCGMemOp opc = get_memop(oi);
    TCGReg a0 = tcg_target_call_iarg_regs[0];
    TCGReg a1 = tcg_target_call_iarg_regs[1];
    TCGReg a2 = tcg_target_call_iarg_regs[2];
    TCGReg a3 = tcg_target_call_iarg_regs[3];

    /* We don't support oversize guests */
    if (TCG_TARGET_REG_BITS < TARGET_LONG_BITS) {
        g_assert_not_reached();
    }

    /* resolve label address */
    patch_reloc(l->label_ptr[0], R_RISCV_BRANCH, (intptr_t) s->code_ptr, 0);

    /* call load helper */
    tcg_out_mov(s, TCG_TYPE_PTR, a0, TCG_AREG0);
    tcg_out_mov(s, TCG_TYPE_PTR, a1, l->addrlo_reg);
    tcg_out_movi(s, TCG_TYPE_PTR, a2, oi);
    tcg_out_movi(s, TCG_TYPE_PTR, a3, (tcg_target_long)l->raddr);

    tcg_out_call(s, qemu_ld_helpers[opc & (MO_BSWAP | MO_SSIZE)]);
    tcg_out_mov(s, (opc & MO_SIZE) == MO_64, l->datalo_reg, a0);

    tcg_out_goto(s, l->raddr);
}

static void tcg_out_qemu_st_slow_path(TCGContext *s, TCGLabelQemuLdst *l)
{
    TCGMemOpIdx oi = l->oi;
    TCGMemOp opc = get_memop(oi);
    TCGMemOp s_bits = opc & MO_SIZE;
    TCGReg a0 = tcg_target_call_iarg_regs[0];
    TCGReg a1 = tcg_target_call_iarg_regs[1];
    TCGReg a2 = tcg_target_call_iarg_regs[2];
    TCGReg a3 = tcg_target_call_iarg_regs[3];
    TCGReg a4 = tcg_target_call_iarg_regs[4];

    /* We don't support oversize guests */
    if (TCG_TARGET_REG_BITS < TARGET_LONG_BITS) {
        g_assert_not_reached();
    }

    /* resolve label address */
    patch_reloc(l->label_ptr[0], R_RISCV_BRANCH, (intptr_t) s->code_ptr, 0);

    /* call store helper */
    tcg_out_mov(s, TCG_TYPE_PTR, a0, TCG_AREG0);
    tcg_out_mov(s, TCG_TYPE_PTR, a1, l->addrlo_reg);
    tcg_out_mov(s, TCG_TYPE_PTR, a2, l->datalo_reg);
    switch (s_bits) {
    case MO_8:
        tcg_out_ext8u(s, a2, a2);
        break;
    case MO_16:
        tcg_out_ext16u(s, a2, a2);
        break;
    default:
        break;
    }
    tcg_out_movi(s, TCG_TYPE_PTR, a3, oi);
    tcg_out_movi(s, TCG_TYPE_PTR, a4, (tcg_target_long)l->raddr);

    tcg_out_call(s, qemu_st_helpers[opc & (MO_BSWAP | MO_SSIZE)]);

    tcg_out_goto(s, l->raddr);
}
#endif /* CONFIG_SOFTMMU */
