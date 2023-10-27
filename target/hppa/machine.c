/*
 *  HPPA interrupt helper routines
 *
 *  Copyright (c) 2017 Richard Henderson
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

#include "qemu/osdep.h"
#include "cpu.h"
#include "migration/cpu.h"

#if TARGET_REGISTER_BITS == 64
#define qemu_put_betr   qemu_put_be64
#define qemu_get_betr   qemu_get_be64
#define VMSTATE_UINTTL_V(_f, _s, _v) \
    VMSTATE_UINT64_V(_f, _s, _v)
#define VMSTATE_UINTTL_ARRAY_V(_f, _s, _n, _v) \
    VMSTATE_UINT64_ARRAY_V(_f, _s, _n, _v)
#else
#define qemu_put_betr   qemu_put_be32
#define qemu_get_betr   qemu_get_be32
#define VMSTATE_UINTTR_V(_f, _s, _v) \
    VMSTATE_UINT32_V(_f, _s, _v)
#define VMSTATE_UINTTR_ARRAY_V(_f, _s, _n, _v) \
    VMSTATE_UINT32_ARRAY_V(_f, _s, _n, _v)
#endif

#define VMSTATE_UINTTR(_f, _s) \
    VMSTATE_UINTTR_V(_f, _s, 0)
#define VMSTATE_UINTTR_ARRAY(_f, _s, _n) \
    VMSTATE_UINTTR_ARRAY_V(_f, _s, _n, 0)


static int get_psw(QEMUFile *f, void *opaque, size_t size,
                   const VMStateField *field)
{
    CPUHPPAState *env = opaque;
    cpu_hppa_put_psw(env, qemu_get_betr(f));
    return 0;
}

static int put_psw(QEMUFile *f, void *opaque, size_t size,
                   const VMStateField *field, JSONWriter *vmdesc)
{
    CPUHPPAState *env = opaque;
    qemu_put_betr(f, cpu_hppa_get_psw(env));
    return 0;
}

static const VMStateInfo vmstate_psw = {
    .name = "psw",
    .get = get_psw,
    .put = put_psw,
};

/* FIXME: Use the PA2.0 format, which is a superset of the PA1.1 format.  */
static int get_tlb(QEMUFile *f, void *opaque, size_t size,
                   const VMStateField *field)
{
    HPPATLBEntry *ent = opaque;
    uint32_t val;

    ent->itree.start = qemu_get_be64(f);
    ent->pa = qemu_get_betr(f);
    val = qemu_get_be32(f);

    ent->entry_valid = extract32(val, 0, 1);
    ent->access_id = extract32(val, 1, 18);
    ent->u = extract32(val, 19, 1);
    ent->ar_pl2 = extract32(val, 20, 2);
    ent->ar_pl1 = extract32(val, 22, 2);
    ent->ar_type = extract32(val, 24, 3);
    ent->b = extract32(val, 27, 1);
    ent->d = extract32(val, 28, 1);
    ent->t = extract32(val, 29, 1);

    ent->itree.last = ent->itree.start + TARGET_PAGE_SIZE - 1;
    return 0;
}

static int put_tlb(QEMUFile *f, void *opaque, size_t size,
                   const VMStateField *field, JSONWriter *vmdesc)
{
    HPPATLBEntry *ent = opaque;
    uint32_t val = 0;

    if (ent->entry_valid) {
        val = 1;
        val = deposit32(val, 1, 18, ent->access_id);
        val = deposit32(val, 19, 1, ent->u);
        val = deposit32(val, 20, 2, ent->ar_pl2);
        val = deposit32(val, 22, 2, ent->ar_pl1);
        val = deposit32(val, 24, 3, ent->ar_type);
        val = deposit32(val, 27, 1, ent->b);
        val = deposit32(val, 28, 1, ent->d);
        val = deposit32(val, 29, 1, ent->t);
    }

    qemu_put_be64(f, ent->itree.start);
    qemu_put_betr(f, ent->pa);
    qemu_put_be32(f, val);
    return 0;
}

static const VMStateInfo vmstate_tlb = {
    .name = "tlb entry",
    .get = get_tlb,
    .put = put_tlb,
};

static int tlb_pre_load(void *opaque)
{
    CPUHPPAState *env = opaque;

    /*
     * Zap the entire tlb, on-the-side data structures and all.
     * Each tlb entry will have data re-filled by put_tlb.
     */
    memset(env->tlb, 0, sizeof(env->tlb));
    memset(&env->tlb_root, 0, sizeof(env->tlb_root));
    env->tlb_unused = NULL;
    env->tlb_partial = NULL;

    return 0;
}

static int tlb_post_load(void *opaque, int version_id)
{
    CPUHPPAState *env = opaque;
    HPPATLBEntry **unused = &env->tlb_unused;
    HPPATLBEntry *partial = NULL;

    /*
     * Re-create the interval tree from the valid entries.
     * Truely invalid entries should have start == end == 0.
     * Otherwise it should be the in-flight tlb_partial entry.
     */
    for (uint32_t i = 0; i < ARRAY_SIZE(env->tlb); ++i) {
        HPPATLBEntry *e = &env->tlb[i];

        if (e->entry_valid) {
            interval_tree_insert(&e->itree, &env->tlb_root);
        } else if (i < HPPA_BTLB_ENTRIES) {
            /* btlb not in unused list */
        } else if (partial == NULL && e->itree.start < e->itree.last) {
            partial = e;
        } else {
            *unused = e;
            unused = &e->unused_next;
        }
    }
    env->tlb_partial = partial;
    *unused = NULL;

    return 0;
}

static VMStateField vmstate_env_fields[] = {
    VMSTATE_UINTTR_ARRAY(gr, CPUHPPAState, 32),
    VMSTATE_UINT64_ARRAY(fr, CPUHPPAState, 32),
    VMSTATE_UINT64_ARRAY(sr, CPUHPPAState, 8),
    VMSTATE_UINTTR_ARRAY(cr, CPUHPPAState, 32),
    VMSTATE_UINTTR_ARRAY(cr_back, CPUHPPAState, 2),
    VMSTATE_UINTTR_ARRAY(shadow, CPUHPPAState, 7),

    /* Save the architecture value of the psw, not the internally
       expanded version.  Since this architecture value does not
       exist in memory to be stored, this requires a but of hoop
       jumping.  We want OFFSET=0 so that we effectively pass ENV
       to the helper functions, and we need to fill in the name by
       hand since there's no field of that name.  */
    {
        .name = "psw",
        .version_id = 0,
        .size = sizeof(uint64_t),
        .info = &vmstate_psw,
        .flags = VMS_SINGLE,
        .offset = 0
    },

    VMSTATE_UINTTR(iaoq_f, CPUHPPAState),
    VMSTATE_UINTTR(iaoq_b, CPUHPPAState),
    VMSTATE_UINT64(iasq_f, CPUHPPAState),
    VMSTATE_UINT64(iasq_b, CPUHPPAState),

    VMSTATE_UINT32(fr0_shadow, CPUHPPAState),

    VMSTATE_ARRAY(tlb, CPUHPPAState, ARRAY_SIZE(((CPUHPPAState *)0)->tlb),
                  0, vmstate_tlb, HPPATLBEntry),
    VMSTATE_UINT32(tlb_last, CPUHPPAState),

    VMSTATE_END_OF_LIST()
};

static const VMStateDescription vmstate_env = {
    .name = "env",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = vmstate_env_fields,
    .pre_load = tlb_pre_load,
    .post_load = tlb_post_load,
};

static VMStateField vmstate_cpu_fields[] = {
    VMSTATE_CPU(),
    VMSTATE_STRUCT(env, HPPACPU, 1, vmstate_env, CPUHPPAState),
    VMSTATE_END_OF_LIST()
};

const VMStateDescription vmstate_hppa_cpu = {
    .name = "cpu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = vmstate_cpu_fields,
};
