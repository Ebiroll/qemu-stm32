/* External helpers */
void glue(do_lsw, MEMSUFFIX) (int dst);
void glue(do_stsw, MEMSUFFIX) (int src);

/* Internal helpers for sign extension and byte-reverse */
static inline uint32_t glue(_ld16x, MEMSUFFIX) (void *EA, int type)
{
    return s_ext16(glue(_lduw, MEMSUFFIX)(EA, type));
}

static inline uint16_t glue(_ld16r, MEMSUFFIX) (void *EA, int type)
{
    uint16_t tmp = glue(_lduw, MEMSUFFIX)(EA, type);
    return ((tmp & 0xFF00) >> 8) | ((tmp & 0x00FF) << 8);
}

static inline uint32_t glue(_ld32r, MEMSUFFIX) (void *EA, int type)
{
    uint32_t tmp = glue(_ldl, MEMSUFFIX)(EA, type);
    return ((tmp & 0xFF000000) >> 24) | ((tmp & 0x00FF0000) >> 8) |
        ((tmp & 0x0000FF00) << 8) | ((tmp & 0x000000FF) << 24);
}

static inline void glue(_st16r, MEMSUFFIX) (void *EA, uint16_t data, int type)
{
    uint16_t tmp = ((data & 0xFF00) >> 8) | ((data & 0x00FF) << 8);
    glue(_stw, MEMSUFFIX)(EA, tmp, type);
}

static inline void glue(_st32r, MEMSUFFIX) (void *EA, uint32_t data, int type)
{
    uint32_t tmp = ((data & 0xFF000000) >> 24) | ((data & 0x00FF0000) >> 8) |
        ((data & 0x0000FF00) << 8) | ((data & 0x000000FF) << 24);
    glue(_stl, MEMSUFFIX)(EA, tmp, type);
}

/***                             Integer load                              ***/
#define PPC_LD_OP(name, op)                                                   \
PPC_OP(glue(glue(l, name), MEMSUFFIX))                                        \
{                                                                             \
    T1 = glue(op, MEMSUFFIX)((void *)T0, ACCESS_INT);                         \
    RETURN();                                                                 \
}

#define PPC_ST_OP(name, op)                                                   \
PPC_OP(glue(glue(st, name), MEMSUFFIX))                                       \
{                                                                             \
    glue(op, MEMSUFFIX)((void *)T0, T1, ACCESS_INT);                          \
    RETURN();                                                                 \
}

PPC_LD_OP(bz, _ldub);
PPC_LD_OP(ha, _ld16x);
PPC_LD_OP(hz, _lduw);
PPC_LD_OP(wz, _ldl);

/***                              Integer store                            ***/
PPC_ST_OP(b, _stb);
PPC_ST_OP(h, _stw);
PPC_ST_OP(w, _stl);

/***                Integer load and store with byte reverse               ***/
PPC_LD_OP(hbr, _ld16r);
PPC_LD_OP(wbr, _ld32r);
PPC_ST_OP(hbr, _st16r);
PPC_ST_OP(wbr, _st32r);

/***                    Integer load and store multiple                    ***/
PPC_OP(glue(lmw, MEMSUFFIX))
{
    int dst = PARAM(1);

    for (; dst < 32; dst++, T0 += 4) {
        ugpr(dst) = glue(_ldl, MEMSUFFIX)((void *)T0, ACCESS_INT);
    }
    RETURN();
}

PPC_OP(glue(stmw, MEMSUFFIX))
{
    int src = PARAM(1);

    for (; src < 32; src++, T0 += 4) {
        glue(_stl, MEMSUFFIX)((void *)T0, ugpr(src), ACCESS_INT);
    }
    RETURN();
}

/***                    Integer load and store strings                     ***/
PPC_OP(glue(lswi, MEMSUFFIX))
{
    glue(do_lsw, MEMSUFFIX)(PARAM(1));
    RETURN();
}

/* PPC32 specification says we must generate an exception if
 * rA is in the range of registers to be loaded.
 * In an other hand, IBM says this is valid, but rA won't be loaded.
 * For now, I'll follow the spec...
 */
PPC_OP(glue(lswx, MEMSUFFIX))
{
    if (T1 > 0) {
        if ((PARAM(1) < PARAM(2) && (PARAM(1) + T1) > PARAM(2)) ||
            (PARAM(1) < PARAM(3) && (PARAM(1) + T1) > PARAM(3))) {
            do_queue_exception_err(EXCP_PROGRAM, EXCP_INVAL | EXCP_INVAL_LSWX);
            do_process_exceptions();
        } else {
            glue(do_lsw, MEMSUFFIX)(PARAM(1));
        }
    }
    RETURN();
}

PPC_OP(glue(stsw, MEMSUFFIX))
{
    glue(do_stsw, MEMSUFFIX)(PARAM(1));
    RETURN();
}

/***                         Floating-point store                          ***/
#define PPC_STF_OP(name, op)                                                  \
PPC_OP(glue(glue(st, name), MEMSUFFIX))                                       \
{                                                                             \
    glue(op, MEMSUFFIX)((void *)T0, FT1);                                     \
    RETURN();                                                                 \
}

PPC_STF_OP(fd, stfq);
PPC_STF_OP(fs, stfl);

/***                         Floating-point load                           ***/
#define PPC_LDF_OP(name, op)                                                  \
PPC_OP(glue(glue(l, name), MEMSUFFIX))                                        \
{                                                                             \
    FT1 = glue(op, MEMSUFFIX)((void *)T0);                                    \
    RETURN();                                                                 \
}

PPC_LDF_OP(fd, ldfq);
PPC_LDF_OP(fs, ldfl);

/* Store with reservation */
PPC_OP(glue(stwcx, MEMSUFFIX))
{
    if (T0 & 0x03) {
        do_queue_exception(EXCP_ALIGN);
        do_process_exceptions();
    } else {
        if (regs->reserve != T0) {
            env->crf[0] = xer_ov;
        } else {
            glue(_stl, MEMSUFFIX)((void *)T0, T1, ACCESS_RES);
            env->crf[0] = xer_ov | 0x02;
        }
    }
    regs->reserve = 0;
    RETURN();
}

PPC_OP(glue(dcbz, MEMSUFFIX))
{
    glue(_stl, MEMSUFFIX)((void *)(T0 + 0x00), 0, ACCESS_INT);
    glue(_stl, MEMSUFFIX)((void *)(T0 + 0x04), 0, ACCESS_INT);
    glue(_stl, MEMSUFFIX)((void *)(T0 + 0x08), 0, ACCESS_INT);
    glue(_stl, MEMSUFFIX)((void *)(T0 + 0x0C), 0, ACCESS_INT);
    glue(_stl, MEMSUFFIX)((void *)(T0 + 0x10), 0, ACCESS_INT);
    glue(_stl, MEMSUFFIX)((void *)(T0 + 0x14), 0, ACCESS_INT);
    glue(_stl, MEMSUFFIX)((void *)(T0 + 0x18), 0, ACCESS_INT);
    glue(_stl, MEMSUFFIX)((void *)(T0 + 0x1C), 0, ACCESS_INT);
    RETURN();
}

/* External access */
PPC_OP(glue(eciwx, MEMSUFFIX))
{
    T1 = glue(_ldl, MEMSUFFIX)((void *)T0, ACCESS_EXT);
    RETURN();
}

PPC_OP(glue(ecowx, MEMSUFFIX))
{
    glue(_stl, MEMSUFFIX)((void *)T0, T1, ACCESS_EXT);
    RETURN();
}

#undef MEMSUFFIX
