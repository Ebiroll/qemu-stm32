/*
 * dyngen helpers
 * 
 *  Copyright (c) 2003 Fabrice Bellard
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

#ifdef __alpha__

register int gp asm("$29");

static inline void immediate_ldah(void *p, int val) {
    uint32_t *dest = p;
    long high = ((val >> 16) + ((val >> 15) & 1)) & 0xffff;

    *dest &= ~0xffff;
    *dest |= high;
    *dest |= 31 << 16;
}
static inline void immediate_lda(void *dest, int val) {
    *(uint16_t *) dest = val;
}
void fix_bsr(void *p, int offset) {
    uint32_t *dest = p;
    *dest &= ~((1 << 21) - 1);
    *dest |= (offset >> 2) & ((1 << 21) - 1);
}

#endif /* __alpha__ */

#ifdef __arm__

#define MAX_OP_SIZE    (128 * 4) /* in bytes */
/* max size of the code that can be generated without calling arm_flush_ldr */
#define MAX_FRAG_SIZE  (1024 * 4) 
//#define MAX_FRAG_SIZE  (135 * 4) /* for testing */ 

typedef struct LDREntry {
    uint8_t *ptr;
    uint32_t *data_ptr;
} LDREntry;

static LDREntry arm_ldr_table[1024];
static uint32_t arm_data_table[1024];

extern char exec_loop;

static inline void arm_reloc_pc24(uint32_t *ptr, uint32_t insn, int val)
{
    *ptr = (insn & ~0xffffff) | ((insn + ((val - (int)ptr) >> 2)) & 0xffffff);
}

static uint8_t *arm_flush_ldr(uint8_t *gen_code_ptr,
                              LDREntry *ldr_start, LDREntry *ldr_end, 
                              uint32_t *data_start, uint32_t *data_end, 
                              int gen_jmp)
{
    LDREntry *le;
    uint32_t *ptr;
    int offset, data_size, target;
    uint8_t *data_ptr;
    uint32_t insn;
 
    data_size = (uint8_t *)data_end - (uint8_t *)data_start;

    if (!gen_jmp) {
        /* b exec_loop */
        arm_reloc_pc24((uint32_t *)gen_code_ptr, 0xeafffffe, (long)(&exec_loop)); 
        gen_code_ptr += 4;
    } else {
        /* generate branch to skip the data */
        if (data_size == 0)
            return gen_code_ptr;
        target = (long)gen_code_ptr + data_size + 4;
        arm_reloc_pc24((uint32_t *)gen_code_ptr, 0xeafffffe, target);
        gen_code_ptr += 4;
    }
   
    /* copy the data */
    data_ptr = gen_code_ptr;
    memcpy(gen_code_ptr, data_start, data_size);
    gen_code_ptr += data_size;
    
    /* patch the ldr to point to the data */
    for(le = ldr_start; le < ldr_end; le++) {
        ptr = (uint32_t *)le->ptr;
        offset = ((unsigned long)(le->data_ptr) - (unsigned long)data_start) + 
            (unsigned long)data_ptr - 
            (unsigned long)ptr - 8;
        insn = *ptr & ~(0xfff | 0x00800000);
        if (offset < 0) {
            offset = - offset;
        } else {
            insn |= 0x00800000;
        }
        if (offset > 0xfff) {
            fprintf(stderr, "Error ldr offset\n");
            abort();
        }
        insn |= offset;
        *ptr = insn;
    }
    return gen_code_ptr;
}

#endif /* __arm__ */

                       
                   
