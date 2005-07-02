#if !defined(__QEMU_MIPS_EXEC_H__)
#define __QEMU_MIPS_EXEC_H__

#define DEBUG_OP

#include "mips-defs.h"
#include "dyngen-exec.h"

register struct CPUMIPSState *env asm(AREG0);

#if defined (USE_64BITS_REGS)
typedef int64_t host_int_t;
typedef uint64_t host_uint_t;
#else
typedef int32_t host_int_t;
typedef uint32_t host_uint_t;
#endif

register host_uint_t T0 asm(AREG1);
register host_uint_t T1 asm(AREG2);
register host_uint_t T2 asm(AREG3);
register host_int_t Ts0 asm(AREG1);
register host_int_t Ts1 asm(AREG2);
register host_int_t Ts2 asm(AREG3);

#define PARAM(n) ((uint32_t)PARAM##n)
#define SPARAM(n) ((int32_t)PARAM##n)

#if defined (USE_HOST_FLOAT_REGS)
register double FT0 asm(FREG0);
register double FT1 asm(FREG1);
register double FT2 asm(FREG2);
register float FTS0 asm(FREG0);
register float FTS1 asm(FREG1);
register float FTS2 asm(FREG2);
#else
#define FT0 (env->ft0.d)
#define FT1 (env->ft1.d)
#define FT2 (env->ft2.d)
#define FTS0 (env->ft0.f)
#define FTS1 (env->ft1.f)
#define FTS2 (env->ft2.f)
#endif

#if defined (DEBUG_OP)
#define RETURN() __asm__ __volatile__("nop");
#else
#define RETURN() __asm__ __volatile__("");
#endif

#include "cpu.h"
#include "exec-all.h"

#if !defined(CONFIG_USER_ONLY)

#define ldul_user ldl_user
#define ldul_kernel ldl_kernel

#define ACCESS_TYPE 0
#define MEMSUFFIX _kernel
#define DATA_SIZE 1
#include "softmmu_header.h"

#define DATA_SIZE 2
#include "softmmu_header.h"

#define DATA_SIZE 4
#include "softmmu_header.h"

#define DATA_SIZE 8
#include "softmmu_header.h"
#undef ACCESS_TYPE
#undef MEMSUFFIX

#define ACCESS_TYPE 1
#define MEMSUFFIX _user
#define DATA_SIZE 1
#include "softmmu_header.h"

#define DATA_SIZE 2
#include "softmmu_header.h"

#define DATA_SIZE 4
#include "softmmu_header.h"

#define DATA_SIZE 8
#include "softmmu_header.h"
#undef ACCESS_TYPE
#undef MEMSUFFIX

/* these access are slower, they must be as rare as possible */
#define ACCESS_TYPE 2
#define MEMSUFFIX _data
#define DATA_SIZE 1
#include "softmmu_header.h"

#define DATA_SIZE 2
#include "softmmu_header.h"

#define DATA_SIZE 4
#include "softmmu_header.h"

#define DATA_SIZE 8
#include "softmmu_header.h"
#undef ACCESS_TYPE
#undef MEMSUFFIX

#define ldub(p) ldub_data(p)
#define ldsb(p) ldsb_data(p)
#define lduw(p) lduw_data(p)
#define ldsw(p) ldsw_data(p)
#define ldl(p) ldl_data(p)
#define ldq(p) ldq_data(p)

#define stb(p, v) stb_data(p, v)
#define stw(p, v) stw_data(p, v)
#define stl(p, v) stl_data(p, v)
#define stq(p, v) stq_data(p, v)

#endif /* !defined(CONFIG_USER_ONLY) */

static inline void env_to_regs(void)
{
}

static inline void regs_to_env(void)
{
}

#if (HOST_LONG_BITS == 32)
void do_mult (void);
void do_multu (void);
void do_madd (void);
void do_maddu (void);
void do_msub (void);
void do_msubu (void);
#endif
__attribute__ (( regparm(2) ))
void do_mfc0(int reg, int sel);
__attribute__ (( regparm(2) ))
void do_mtc0(int reg, int sel);
void do_tlbwi (void);
void do_tlbwr (void);
void do_tlbp (void);
void do_tlbr (void);
void do_lwl_raw (void);
void do_lwr_raw (void);
void do_swl_raw (void);
void do_swr_raw (void);
#if !defined(CONFIG_USER_ONLY)
void do_lwl_user (void);
void do_lwl_kernel (void);
void do_lwr_user (void);
void do_lwr_kernel (void);
void do_swl_user (void);
void do_swl_kernel (void);
void do_swr_user (void);
void do_swr_kernel (void);
#endif
__attribute__ (( regparm(1) ))
void do_pmon (int function);

int cpu_mips_handle_mmu_fault (CPUState *env, target_ulong address, int rw,
                               int is_user, int is_softmmu);
void do_interrupt (CPUState *env);

void cpu_loop_exit(void);
__attribute__ (( regparm(2) ))
void do_raise_exception_err (uint32_t exception, int error_code);
__attribute__ (( regparm(1) ))
void do_raise_exception (uint32_t exception);

void cpu_dump_state(CPUState *env, FILE *f, 
                    int (*cpu_fprintf)(FILE *f, const char *fmt, ...),
                    int flags);
void cpu_mips_irqctrl_init (void);
uint32_t cpu_mips_get_random (CPUState *env);
uint32_t cpu_mips_get_count (CPUState *env);
void cpu_mips_store_count (CPUState *env, uint32_t value);
void cpu_mips_store_compare (CPUState *env, uint32_t value);
void cpu_mips_clock_init (CPUState *env);

#endif /* !defined(__QEMU_MIPS_EXEC_H__) */
