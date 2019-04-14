/*
 * Generic vector operation expansion
 *
 * Copyright (c) 2018 Linaro
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

/*
 * "Generic" vectors.  All operands are given as offsets from ENV,
 * and therefore cannot also be allocated via tcg_global_mem_new_*.
 * OPRSZ is the byte size of the vector upon which the operation is performed.
 * MAXSZ is the byte size of the full vector; bytes beyond OPSZ are cleared.
 *
 * All sizes must be 8 or any multiple of 16.
 * When OPRSZ is 8, the alignment may be 8, otherwise must be 16.
 * Operands may completely, but not partially, overlap.
 */

/* Expand a call to a gvec-style helper, with pointers to two vector
   operands, and a descriptor (see tcg-gvec-desc.h).  */
typedef void gen_helper_gvec_2(TCGv_ptr, TCGv_ptr, TCGv_i32);
void tcg_gen_gvec_2_ool(uint32_t dofs, uint32_t aofs,
                        uint32_t oprsz, uint32_t maxsz, int32_t data,
                        gen_helper_gvec_2 *fn);

/* Similarly, passing an extra data value.  */
typedef void gen_helper_gvec_2i(TCGv_ptr, TCGv_ptr, TCGv_i64, TCGv_i32);
void tcg_gen_gvec_2i_ool(uint32_t dofs, uint32_t aofs, TCGv_i64 c,
                         uint32_t oprsz, uint32_t maxsz, int32_t data,
                         gen_helper_gvec_2i *fn);

/* Similarly, passing an extra pointer (e.g. env or float_status).  */
typedef void gen_helper_gvec_2_ptr(TCGv_ptr, TCGv_ptr, TCGv_ptr, TCGv_i32);
void tcg_gen_gvec_2_ptr(uint32_t dofs, uint32_t aofs,
                        TCGv_ptr ptr, uint32_t oprsz, uint32_t maxsz,
                        int32_t data, gen_helper_gvec_2_ptr *fn);

/* Similarly, with three vector operands.  */
typedef void gen_helper_gvec_3(TCGv_ptr, TCGv_ptr, TCGv_ptr, TCGv_i32);
void tcg_gen_gvec_3_ool(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                        uint32_t oprsz, uint32_t maxsz, int32_t data,
                        gen_helper_gvec_3 *fn);

/* Similarly, with four vector operands.  */
typedef void gen_helper_gvec_4(TCGv_ptr, TCGv_ptr, TCGv_ptr,
                               TCGv_ptr, TCGv_i32);
void tcg_gen_gvec_4_ool(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                        uint32_t cofs, uint32_t oprsz, uint32_t maxsz,
                        int32_t data, gen_helper_gvec_4 *fn);

/* Similarly, with five vector operands.  */
typedef void gen_helper_gvec_5(TCGv_ptr, TCGv_ptr, TCGv_ptr, TCGv_ptr,
                               TCGv_ptr, TCGv_i32);
void tcg_gen_gvec_5_ool(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                        uint32_t cofs, uint32_t xofs, uint32_t oprsz,
                        uint32_t maxsz, int32_t data, gen_helper_gvec_5 *fn);

typedef void gen_helper_gvec_3_ptr(TCGv_ptr, TCGv_ptr, TCGv_ptr,
                                   TCGv_ptr, TCGv_i32);
void tcg_gen_gvec_3_ptr(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                        TCGv_ptr ptr, uint32_t oprsz, uint32_t maxsz,
                        int32_t data, gen_helper_gvec_3_ptr *fn);

typedef void gen_helper_gvec_4_ptr(TCGv_ptr, TCGv_ptr, TCGv_ptr,
                                   TCGv_ptr, TCGv_ptr, TCGv_i32);
void tcg_gen_gvec_4_ptr(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                        uint32_t cofs, TCGv_ptr ptr, uint32_t oprsz,
                        uint32_t maxsz, int32_t data,
                        gen_helper_gvec_4_ptr *fn);

/* Expand a gvec operation.  Either inline or out-of-line depending on
   the actual vector size and the operations supported by the host.  */
typedef struct {
    /* Expand inline as a 64-bit or 32-bit integer.
       Only one of these will be non-NULL.  */
    void (*fni8)(TCGv_i64, TCGv_i64);
    void (*fni4)(TCGv_i32, TCGv_i32);
    /* Expand inline with a host vector type.  */
    void (*fniv)(unsigned, TCGv_vec, TCGv_vec);
    /* Expand out-of-line helper w/descriptor.  */
    gen_helper_gvec_2 *fno;
    /* The optional opcodes, if any, utilized by .fniv.  */
    const TCGOpcode *opt_opc;
    /* The data argument to the out-of-line helper.  */
    int32_t data;
    /* The vector element size, if applicable.  */
    uint8_t vece;
    /* Prefer i64 to v64.  */
    bool prefer_i64;
} GVecGen2;

typedef struct {
    /* Expand inline as a 64-bit or 32-bit integer.
       Only one of these will be non-NULL.  */
    void (*fni8)(TCGv_i64, TCGv_i64, int64_t);
    void (*fni4)(TCGv_i32, TCGv_i32, int32_t);
    /* Expand inline with a host vector type.  */
    void (*fniv)(unsigned, TCGv_vec, TCGv_vec, int64_t);
    /* Expand out-of-line helper w/descriptor, data in descriptor.  */
    gen_helper_gvec_2 *fno;
    /* Expand out-of-line helper w/descriptor, data as argument.  */
    gen_helper_gvec_2i *fnoi;
    /* The optional opcodes, if any, utilized by .fniv.  */
    const TCGOpcode *opt_opc;
    /* The vector element size, if applicable.  */
    uint8_t vece;
    /* Prefer i64 to v64.  */
    bool prefer_i64;
    /* Load dest as a 3rd source operand.  */
    bool load_dest;
} GVecGen2i;

typedef struct {
    /* Expand inline as a 64-bit or 32-bit integer.
       Only one of these will be non-NULL.  */
    void (*fni8)(TCGv_i64, TCGv_i64, TCGv_i64);
    void (*fni4)(TCGv_i32, TCGv_i32, TCGv_i32);
    /* Expand inline with a host vector type.  */
    void (*fniv)(unsigned, TCGv_vec, TCGv_vec, TCGv_vec);
    /* Expand out-of-line helper w/descriptor.  */
    gen_helper_gvec_2i *fno;
    /* The optional opcodes, if any, utilized by .fniv.  */
    const TCGOpcode *opt_opc;
    /* The data argument to the out-of-line helper.  */
    uint32_t data;
    /* The vector element size, if applicable.  */
    uint8_t vece;
    /* Prefer i64 to v64.  */
    bool prefer_i64;
    /* Load scalar as 1st source operand.  */
    bool scalar_first;
} GVecGen2s;

typedef struct {
    /* Expand inline as a 64-bit or 32-bit integer.
       Only one of these will be non-NULL.  */
    void (*fni8)(TCGv_i64, TCGv_i64, TCGv_i64);
    void (*fni4)(TCGv_i32, TCGv_i32, TCGv_i32);
    /* Expand inline with a host vector type.  */
    void (*fniv)(unsigned, TCGv_vec, TCGv_vec, TCGv_vec);
    /* Expand out-of-line helper w/descriptor.  */
    gen_helper_gvec_3 *fno;
    /* The optional opcodes, if any, utilized by .fniv.  */
    const TCGOpcode *opt_opc;
    /* The data argument to the out-of-line helper.  */
    int32_t data;
    /* The vector element size, if applicable.  */
    uint8_t vece;
    /* Prefer i64 to v64.  */
    bool prefer_i64;
    /* Load dest as a 3rd source operand.  */
    bool load_dest;
} GVecGen3;

typedef struct {
    /*
     * Expand inline as a 64-bit or 32-bit integer. Only one of these will be
     * non-NULL.
     */
    void (*fni8)(TCGv_i64, TCGv_i64, TCGv_i64, int64_t);
    void (*fni4)(TCGv_i32, TCGv_i32, TCGv_i32, int32_t);
    /* Expand inline with a host vector type.  */
    void (*fniv)(unsigned, TCGv_vec, TCGv_vec, TCGv_vec, int64_t);
    /* Expand out-of-line helper w/descriptor, data in descriptor.  */
    gen_helper_gvec_3 *fno;
    /* The optional opcodes, if any, utilized by .fniv.  */
    const TCGOpcode *opt_opc;
    /* The vector element size, if applicable.  */
    uint8_t vece;
    /* Prefer i64 to v64.  */
    bool prefer_i64;
    /* Load dest as a 3rd source operand.  */
    bool load_dest;
} GVecGen3i;

typedef struct {
    /* Expand inline as a 64-bit or 32-bit integer.
       Only one of these will be non-NULL.  */
    void (*fni8)(TCGv_i64, TCGv_i64, TCGv_i64, TCGv_i64);
    void (*fni4)(TCGv_i32, TCGv_i32, TCGv_i32, TCGv_i32);
    /* Expand inline with a host vector type.  */
    void (*fniv)(unsigned, TCGv_vec, TCGv_vec, TCGv_vec, TCGv_vec);
    /* Expand out-of-line helper w/descriptor.  */
    gen_helper_gvec_4 *fno;
    /* The optional opcodes, if any, utilized by .fniv.  */
    const TCGOpcode *opt_opc;
    /* The data argument to the out-of-line helper.  */
    int32_t data;
    /* The vector element size, if applicable.  */
    uint8_t vece;
    /* Prefer i64 to v64.  */
    bool prefer_i64;
    /* Write aofs as a 2nd dest operand.  */
    bool write_aofs;
} GVecGen4;

void tcg_gen_gvec_2(uint32_t dofs, uint32_t aofs,
                    uint32_t oprsz, uint32_t maxsz, const GVecGen2 *);
void tcg_gen_gvec_2i(uint32_t dofs, uint32_t aofs, uint32_t oprsz,
                     uint32_t maxsz, int64_t c, const GVecGen2i *);
void tcg_gen_gvec_2s(uint32_t dofs, uint32_t aofs, uint32_t oprsz,
                     uint32_t maxsz, TCGv_i64 c, const GVecGen2s *);
void tcg_gen_gvec_3(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                    uint32_t oprsz, uint32_t maxsz, const GVecGen3 *);
void tcg_gen_gvec_3i(uint32_t dofs, uint32_t aofs, uint32_t bofs,
                     uint32_t oprsz, uint32_t maxsz, int64_t c,
                     const GVecGen3i *);
void tcg_gen_gvec_4(uint32_t dofs, uint32_t aofs, uint32_t bofs, uint32_t cofs,
                    uint32_t oprsz, uint32_t maxsz, const GVecGen4 *);

/* Expand a specific vector operation.  */

void tcg_gen_gvec_mov(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_not(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_neg(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t oprsz, uint32_t maxsz);

void tcg_gen_gvec_add(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_sub(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_mul(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t bofs, uint32_t oprsz, uint32_t maxsz);

void tcg_gen_gvec_addi(unsigned vece, uint32_t dofs, uint32_t aofs,
                       int64_t c, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_muli(unsigned vece, uint32_t dofs, uint32_t aofs,
                       int64_t c, uint32_t oprsz, uint32_t maxsz);

void tcg_gen_gvec_adds(unsigned vece, uint32_t dofs, uint32_t aofs,
                       TCGv_i64 c, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_subs(unsigned vece, uint32_t dofs, uint32_t aofs,
                       TCGv_i64 c, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_muls(unsigned vece, uint32_t dofs, uint32_t aofs,
                       TCGv_i64 c, uint32_t oprsz, uint32_t maxsz);

/* Saturated arithmetic.  */
void tcg_gen_gvec_ssadd(unsigned vece, uint32_t dofs, uint32_t aofs,
                        uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_sssub(unsigned vece, uint32_t dofs, uint32_t aofs,
                        uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_usadd(unsigned vece, uint32_t dofs, uint32_t aofs,
                        uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_ussub(unsigned vece, uint32_t dofs, uint32_t aofs,
                        uint32_t bofs, uint32_t oprsz, uint32_t maxsz);

/* Min/max.  */
void tcg_gen_gvec_smin(unsigned vece, uint32_t dofs, uint32_t aofs,
                       uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_umin(unsigned vece, uint32_t dofs, uint32_t aofs,
                       uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_smax(unsigned vece, uint32_t dofs, uint32_t aofs,
                       uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_umax(unsigned vece, uint32_t dofs, uint32_t aofs,
                       uint32_t bofs, uint32_t oprsz, uint32_t maxsz);

void tcg_gen_gvec_and(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_or(unsigned vece, uint32_t dofs, uint32_t aofs,
                     uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_xor(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_andc(unsigned vece, uint32_t dofs, uint32_t aofs,
                       uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_orc(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_nand(unsigned vece, uint32_t dofs, uint32_t aofs,
                       uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_nor(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_eqv(unsigned vece, uint32_t dofs, uint32_t aofs,
                      uint32_t bofs, uint32_t oprsz, uint32_t maxsz);

void tcg_gen_gvec_andi(unsigned vece, uint32_t dofs, uint32_t aofs,
                       int64_t c, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_xori(unsigned vece, uint32_t dofs, uint32_t aofs,
                       int64_t c, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_ori(unsigned vece, uint32_t dofs, uint32_t aofs,
                      int64_t c, uint32_t oprsz, uint32_t maxsz);

void tcg_gen_gvec_ands(unsigned vece, uint32_t dofs, uint32_t aofs,
                       TCGv_i64 c, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_xors(unsigned vece, uint32_t dofs, uint32_t aofs,
                       TCGv_i64 c, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_ors(unsigned vece, uint32_t dofs, uint32_t aofs,
                      TCGv_i64 c, uint32_t oprsz, uint32_t maxsz);

void tcg_gen_gvec_dup_mem(unsigned vece, uint32_t dofs, uint32_t aofs,
                          uint32_t s, uint32_t m);
void tcg_gen_gvec_dup_i32(unsigned vece, uint32_t dofs, uint32_t s,
                          uint32_t m, TCGv_i32);
void tcg_gen_gvec_dup_i64(unsigned vece, uint32_t dofs, uint32_t s,
                          uint32_t m, TCGv_i64);

void tcg_gen_gvec_dup8i(uint32_t dofs, uint32_t s, uint32_t m, uint8_t x);
void tcg_gen_gvec_dup16i(uint32_t dofs, uint32_t s, uint32_t m, uint16_t x);
void tcg_gen_gvec_dup32i(uint32_t dofs, uint32_t s, uint32_t m, uint32_t x);
void tcg_gen_gvec_dup64i(uint32_t dofs, uint32_t s, uint32_t m, uint64_t x);

void tcg_gen_gvec_shli(unsigned vece, uint32_t dofs, uint32_t aofs,
                       int64_t shift, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_shri(unsigned vece, uint32_t dofs, uint32_t aofs,
                       int64_t shift, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_sari(unsigned vece, uint32_t dofs, uint32_t aofs,
                       int64_t shift, uint32_t oprsz, uint32_t maxsz);

/*
 * Perform vector shift by vector element, modulo the element size.
 * E.g.  D[i] = A[i] << (B[i] % (8 << vece)).
 */
void tcg_gen_gvec_shlv(unsigned vece, uint32_t dofs, uint32_t aofs,
                       uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_shrv(unsigned vece, uint32_t dofs, uint32_t aofs,
                       uint32_t bofs, uint32_t oprsz, uint32_t maxsz);
void tcg_gen_gvec_sarv(unsigned vece, uint32_t dofs, uint32_t aofs,
                       uint32_t bofs, uint32_t oprsz, uint32_t maxsz);

void tcg_gen_gvec_cmp(TCGCond cond, unsigned vece, uint32_t dofs,
                      uint32_t aofs, uint32_t bofs,
                      uint32_t oprsz, uint32_t maxsz);

/*
 * 64-bit vector operations.  Use these when the register has been allocated
 * with tcg_global_mem_new_i64, and so we cannot also address it via pointer.
 * OPRSZ = MAXSZ = 8.
 */

void tcg_gen_vec_neg8_i64(TCGv_i64 d, TCGv_i64 a);
void tcg_gen_vec_neg16_i64(TCGv_i64 d, TCGv_i64 a);
void tcg_gen_vec_neg32_i64(TCGv_i64 d, TCGv_i64 a);

void tcg_gen_vec_add8_i64(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b);
void tcg_gen_vec_add16_i64(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b);
void tcg_gen_vec_add32_i64(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b);

void tcg_gen_vec_sub8_i64(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b);
void tcg_gen_vec_sub16_i64(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b);
void tcg_gen_vec_sub32_i64(TCGv_i64 d, TCGv_i64 a, TCGv_i64 b);

void tcg_gen_vec_shl8i_i64(TCGv_i64 d, TCGv_i64 a, int64_t);
void tcg_gen_vec_shl16i_i64(TCGv_i64 d, TCGv_i64 a, int64_t);
void tcg_gen_vec_shr8i_i64(TCGv_i64 d, TCGv_i64 a, int64_t);
void tcg_gen_vec_shr16i_i64(TCGv_i64 d, TCGv_i64 a, int64_t);
void tcg_gen_vec_sar8i_i64(TCGv_i64 d, TCGv_i64 a, int64_t);
void tcg_gen_vec_sar16i_i64(TCGv_i64 d, TCGv_i64 a, int64_t);
