/*
 * Helpers for CWP and PSTATE handling
 *
 *  Copyright (c) 2003-2005 Fabrice Bellard
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
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "cpu.h"
#include "helper.h"

//#define DEBUG_PSTATE

#ifdef DEBUG_PSTATE
#define DPRINTF_PSTATE(fmt, ...)                                \
    do { printf("PSTATE: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF_PSTATE(fmt, ...) do {} while (0)
#endif

static inline void memcpy32(target_ulong *dst, const target_ulong *src)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
    dst[4] = src[4];
    dst[5] = src[5];
    dst[6] = src[6];
    dst[7] = src[7];
}

void cpu_set_cwp(CPUState *env, int new_cwp)
{
    /* put the modified wrap registers at their proper location */
    if (env->cwp == env->nwindows - 1) {
        memcpy32(env->regbase, env->regbase + env->nwindows * 16);
    }
    env->cwp = new_cwp;

    /* put the wrap registers at their temporary location */
    if (new_cwp == env->nwindows - 1) {
        memcpy32(env->regbase + env->nwindows * 16, env->regbase);
    }
    env->regwptr = env->regbase + (new_cwp * 16);
}

target_ulong cpu_get_psr(CPUState *env)
{
    helper_compute_psr(env);

#if !defined(TARGET_SPARC64)
    return env->version | (env->psr & PSR_ICC) |
        (env->psref ? PSR_EF : 0) |
        (env->psrpil << 8) |
        (env->psrs ? PSR_S : 0) |
        (env->psrps ? PSR_PS : 0) |
        (env->psret ? PSR_ET : 0) | env->cwp;
#else
    return env->psr & PSR_ICC;
#endif
}

void cpu_put_psr(CPUState *env, target_ulong val)
{
    env->psr = val & PSR_ICC;
#if !defined(TARGET_SPARC64)
    env->psref = (val & PSR_EF) ? 1 : 0;
    env->psrpil = (val & PSR_PIL) >> 8;
#endif
#if ((!defined(TARGET_SPARC64)) && !defined(CONFIG_USER_ONLY))
    cpu_check_irqs(env);
#endif
#if !defined(TARGET_SPARC64)
    env->psrs = (val & PSR_S) ? 1 : 0;
    env->psrps = (val & PSR_PS) ? 1 : 0;
    env->psret = (val & PSR_ET) ? 1 : 0;
    cpu_set_cwp(env, val & PSR_CWP);
#endif
    env->cc_op = CC_OP_FLAGS;
}

int cpu_cwp_inc(CPUState *env, int cwp)
{
    if (unlikely(cwp >= env->nwindows)) {
        cwp -= env->nwindows;
    }
    return cwp;
}

int cpu_cwp_dec(CPUState *env, int cwp)
{
    if (unlikely(cwp < 0)) {
        cwp += env->nwindows;
    }
    return cwp;
}

#ifndef TARGET_SPARC64
void helper_rett(CPUState *env)
{
    unsigned int cwp;

    if (env->psret == 1) {
        helper_raise_exception(env, TT_ILL_INSN);
    }

    env->psret = 1;
    cwp = cpu_cwp_inc(env, env->cwp + 1) ;
    if (env->wim & (1 << cwp)) {
        helper_raise_exception(env, TT_WIN_UNF);
    }
    cpu_set_cwp(env, cwp);
    env->psrs = env->psrps;
}

/* XXX: use another pointer for %iN registers to avoid slow wrapping
   handling ? */
void helper_save(CPUState *env)
{
    uint32_t cwp;

    cwp = cpu_cwp_dec(env, env->cwp - 1);
    if (env->wim & (1 << cwp)) {
        helper_raise_exception(env, TT_WIN_OVF);
    }
    cpu_set_cwp(env, cwp);
}

void helper_restore(CPUState *env)
{
    uint32_t cwp;

    cwp = cpu_cwp_inc(env, env->cwp + 1);
    if (env->wim & (1 << cwp)) {
        helper_raise_exception(env, TT_WIN_UNF);
    }
    cpu_set_cwp(env, cwp);
}

void helper_wrpsr(CPUState *env, target_ulong new_psr)
{
    if ((new_psr & PSR_CWP) >= env->nwindows) {
        helper_raise_exception(env, TT_ILL_INSN);
    } else {
        cpu_put_psr(env, new_psr);
    }
}

target_ulong helper_rdpsr(CPUState *env)
{
    return cpu_get_psr(env);
}

#else
/* XXX: use another pointer for %iN registers to avoid slow wrapping
   handling ? */
void helper_save(CPUState *env)
{
    uint32_t cwp;

    cwp = cpu_cwp_dec(env, env->cwp - 1);
    if (env->cansave == 0) {
        helper_raise_exception(env, TT_SPILL | (env->otherwin != 0 ?
                                                (TT_WOTHER |
                                                 ((env->wstate & 0x38) >> 1)) :
                                                ((env->wstate & 0x7) << 2)));
    } else {
        if (env->cleanwin - env->canrestore == 0) {
            /* XXX Clean windows without trap */
            helper_raise_exception(env, TT_CLRWIN);
        } else {
            env->cansave--;
            env->canrestore++;
            cpu_set_cwp(env, cwp);
        }
    }
}

void helper_restore(CPUState *env)
{
    uint32_t cwp;

    cwp = cpu_cwp_inc(env, env->cwp + 1);
    if (env->canrestore == 0) {
        helper_raise_exception(env, TT_FILL | (env->otherwin != 0 ?
                                               (TT_WOTHER |
                                                ((env->wstate & 0x38) >> 1)) :
                                               ((env->wstate & 0x7) << 2)));
    } else {
        env->cansave++;
        env->canrestore--;
        cpu_set_cwp(env, cwp);
    }
}

void helper_flushw(CPUState *env)
{
    if (env->cansave != env->nwindows - 2) {
        helper_raise_exception(env, TT_SPILL | (env->otherwin != 0 ?
                                                (TT_WOTHER |
                                                 ((env->wstate & 0x38) >> 1)) :
                                                ((env->wstate & 0x7) << 2)));
    }
}

void helper_saved(CPUState *env)
{
    env->cansave++;
    if (env->otherwin == 0) {
        env->canrestore--;
    } else {
        env->otherwin--;
    }
}

void helper_restored(CPUState *env)
{
    env->canrestore++;
    if (env->cleanwin < env->nwindows - 1) {
        env->cleanwin++;
    }
    if (env->otherwin == 0) {
        env->cansave--;
    } else {
        env->otherwin--;
    }
}

target_ulong cpu_get_ccr(CPUState *env)
{
    target_ulong psr;

    psr = cpu_get_psr(env);

    return ((env->xcc >> 20) << 4) | ((psr & PSR_ICC) >> 20);
}

void cpu_put_ccr(CPUState *env, target_ulong val)
{
    env->xcc = (val >> 4) << 20;
    env->psr = (val & 0xf) << 20;
    CC_OP = CC_OP_FLAGS;
}

target_ulong cpu_get_cwp64(CPUState *env)
{
    return env->nwindows - 1 - env->cwp;
}

void cpu_put_cwp64(CPUState *env, int cwp)
{
    if (unlikely(cwp >= env->nwindows || cwp < 0)) {
        cwp %= env->nwindows;
    }
    cpu_set_cwp(env, env->nwindows - 1 - cwp);
}

target_ulong helper_rdccr(CPUState *env)
{
    return cpu_get_ccr(env);
}

void helper_wrccr(CPUState *env, target_ulong new_ccr)
{
    cpu_put_ccr(env, new_ccr);
}

/* CWP handling is reversed in V9, but we still use the V8 register
   order. */
target_ulong helper_rdcwp(CPUState *env)
{
    return cpu_get_cwp64(env);
}

void helper_wrcwp(CPUState *env, target_ulong new_cwp)
{
    cpu_put_cwp64(env, new_cwp);
}

static inline uint64_t *get_gregset(CPUState *env, uint32_t pstate)
{
    switch (pstate) {
    default:
        DPRINTF_PSTATE("ERROR in get_gregset: active pstate bits=%x%s%s%s\n",
                       pstate,
                       (pstate & PS_IG) ? " IG" : "",
                       (pstate & PS_MG) ? " MG" : "",
                       (pstate & PS_AG) ? " AG" : "");
        /* pass through to normal set of global registers */
    case 0:
        return env->bgregs;
    case PS_AG:
        return env->agregs;
    case PS_MG:
        return env->mgregs;
    case PS_IG:
        return env->igregs;
    }
}

void cpu_change_pstate(CPUState *env, uint32_t new_pstate)
{
    uint32_t pstate_regs, new_pstate_regs;
    uint64_t *src, *dst;

    if (env->def->features & CPU_FEATURE_GL) {
        /* PS_AG is not implemented in this case */
        new_pstate &= ~PS_AG;
    }

    pstate_regs = env->pstate & 0xc01;
    new_pstate_regs = new_pstate & 0xc01;

    if (new_pstate_regs != pstate_regs) {
        DPRINTF_PSTATE("change_pstate: switching regs old=%x new=%x\n",
                       pstate_regs, new_pstate_regs);
        /* Switch global register bank */
        src = get_gregset(env, new_pstate_regs);
        dst = get_gregset(env, pstate_regs);
        memcpy32(dst, env->gregs);
        memcpy32(env->gregs, src);
    } else {
        DPRINTF_PSTATE("change_pstate: regs new=%x (unchanged)\n",
                       new_pstate_regs);
    }
    env->pstate = new_pstate;
}

void helper_wrpstate(CPUState *env, target_ulong new_state)
{
    cpu_change_pstate(env, new_state & 0xf3f);

#if !defined(CONFIG_USER_ONLY)
    if (cpu_interrupts_enabled(env)) {
        cpu_check_irqs(env);
    }
#endif
}

void helper_wrpil(CPUState *env, target_ulong new_pil)
{
#if !defined(CONFIG_USER_ONLY)
    DPRINTF_PSTATE("helper_wrpil old=%x new=%x\n",
                   env->psrpil, (uint32_t)new_pil);

    env->psrpil = new_pil;

    if (cpu_interrupts_enabled(env)) {
        cpu_check_irqs(env);
    }
#endif
}

void helper_done(CPUState *env)
{
    trap_state *tsptr = cpu_tsptr(env);

    env->pc = tsptr->tnpc;
    env->npc = tsptr->tnpc + 4;
    cpu_put_ccr(env, tsptr->tstate >> 32);
    env->asi = (tsptr->tstate >> 24) & 0xff;
    cpu_change_pstate(env, (tsptr->tstate >> 8) & 0xf3f);
    cpu_put_cwp64(env, tsptr->tstate & 0xff);
    env->tl--;

    DPRINTF_PSTATE("... helper_done tl=%d\n", env->tl);

#if !defined(CONFIG_USER_ONLY)
    if (cpu_interrupts_enabled(env)) {
        cpu_check_irqs(env);
    }
#endif
}

void helper_retry(CPUState *env)
{
    trap_state *tsptr = cpu_tsptr(env);

    env->pc = tsptr->tpc;
    env->npc = tsptr->tnpc;
    cpu_put_ccr(env, tsptr->tstate >> 32);
    env->asi = (tsptr->tstate >> 24) & 0xff;
    cpu_change_pstate(env, (tsptr->tstate >> 8) & 0xf3f);
    cpu_put_cwp64(env, tsptr->tstate & 0xff);
    env->tl--;

    DPRINTF_PSTATE("... helper_retry tl=%d\n", env->tl);

#if !defined(CONFIG_USER_ONLY)
    if (cpu_interrupts_enabled(env)) {
        cpu_check_irqs(env);
    }
#endif
}
#endif
