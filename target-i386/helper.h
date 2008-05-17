#define TCG_HELPER_PROTO

void helper_lock(void);
void helper_unlock(void);
void helper_divb_AL(target_ulong t0);
void helper_idivb_AL(target_ulong t0);
void helper_divw_AX(target_ulong t0);
void helper_idivw_AX(target_ulong t0);
void helper_divl_EAX(target_ulong t0);
void helper_idivl_EAX(target_ulong t0);
#ifdef TARGET_X86_64
void helper_mulq_EAX_T0(target_ulong t0);
void helper_imulq_EAX_T0(target_ulong t0);
target_ulong helper_imulq_T0_T1(target_ulong t0, target_ulong t1);
void helper_divq_EAX(target_ulong t0);
void helper_idivq_EAX(target_ulong t0);
#endif

void helper_aam(int base);
void helper_aad(int base);
void helper_aaa(void);
void helper_aas(void);
void helper_daa(void);
void helper_das(void);

uint32_t helper_lsl(uint32_t selector);
uint32_t helper_lar(uint32_t selector);
void helper_verr(uint32_t selector);
void helper_verw(uint32_t selector);
void helper_lldt(int selector);
void helper_ltr(int selector);
void helper_load_seg(int seg_reg, int selector);
void helper_ljmp_protected(int new_cs, target_ulong new_eip,
                           int next_eip_addend);
void helper_lcall_real(int new_cs, target_ulong new_eip1,
                       int shift, int next_eip);
void helper_lcall_protected(int new_cs, target_ulong new_eip, 
                            int shift, int next_eip_addend);
void helper_iret_real(int shift);
void helper_iret_protected(int shift, int next_eip);
void helper_lret_protected(int shift, int addend);
void helper_movl_crN_T0(int reg, target_ulong t0);
void helper_lmsw(target_ulong t0);
void helper_clts(void);
#if !defined(CONFIG_USER_ONLY)
target_ulong helper_movtl_T0_cr8(void);
#endif
void helper_movl_drN_T0(int reg, target_ulong t0);
void helper_invlpg(target_ulong addr);

void helper_enter_level(int level, int data32, target_ulong t1);
#ifdef TARGET_X86_64
void helper_enter64_level(int level, int data64, target_ulong t1);
#endif
void helper_sysenter(void);
void helper_sysexit(void);
#ifdef TARGET_X86_64
void helper_syscall(int next_eip_addend);
void helper_sysret(int dflag);
#endif
void helper_hlt(void);
void helper_monitor(target_ulong ptr);
void helper_mwait(void);
void helper_debug(void);
void helper_raise_interrupt(int intno, int next_eip_addend);
void helper_raise_exception(int exception_index);
void helper_cli(void);
void helper_sti(void);
void helper_set_inhibit_irq(void);
void helper_reset_inhibit_irq(void);
void helper_boundw(target_ulong a0, int v);
void helper_boundl(target_ulong a0, int v);
void helper_rsm(void);
void helper_cmpxchg8b(target_ulong a0);
void helper_single_step(void);
void helper_cpuid(void);
void helper_rdtsc(void);
void helper_rdpmc(void);
void helper_rdmsr(void);
void helper_wrmsr(void);

void helper_check_iob(uint32_t t0);
void helper_check_iow(uint32_t t0);
void helper_check_iol(uint32_t t0);
void helper_outb(uint32_t port, uint32_t data);
target_ulong helper_inb(uint32_t port);
void helper_outw(uint32_t port, uint32_t data);
target_ulong helper_inw(uint32_t port);
void helper_outl(uint32_t port, uint32_t data);
target_ulong helper_inl(uint32_t port);

void helper_svm_check_intercept_param(uint32_t type, uint64_t param);
void helper_vmexit(uint32_t exit_code, uint64_t exit_info_1);
void helper_svm_check_io(uint32_t port, uint32_t param, 
                         uint32_t next_eip_addend);
void helper_vmrun(void);
void helper_vmmcall(void);
void helper_vmload(void);
void helper_vmsave(void);
void helper_stgi(void);
void helper_clgi(void);
void helper_skinit(void);
void helper_invlpga(void);

/* x86 FPU */

void helper_flds_FT0(uint32_t val);
void helper_fldl_FT0(uint64_t val);
void helper_fildl_FT0(int32_t val);
void helper_flds_ST0(uint32_t val);
void helper_fldl_ST0(uint64_t val);
void helper_fildl_ST0(int32_t val);
void helper_fildll_ST0(int64_t val);
uint32_t helper_fsts_ST0(void);
uint64_t helper_fstl_ST0(void);
int32_t helper_fist_ST0(void);
int32_t helper_fistl_ST0(void);
int64_t helper_fistll_ST0(void);
int32_t helper_fistt_ST0(void);
int32_t helper_fisttl_ST0(void);
int64_t helper_fisttll_ST0(void);
void helper_fldt_ST0(target_ulong ptr);
void helper_fstt_ST0(target_ulong ptr);
void helper_fpush(void);
void helper_fpop(void);
void helper_fdecstp(void);
void helper_fincstp(void);
void helper_ffree_STN(int st_index);
void helper_fmov_ST0_FT0(void);
void helper_fmov_FT0_STN(int st_index);
void helper_fmov_ST0_STN(int st_index);
void helper_fmov_STN_ST0(int st_index);
void helper_fxchg_ST0_STN(int st_index);
void helper_fcom_ST0_FT0(void);
void helper_fucom_ST0_FT0(void);
void helper_fcomi_ST0_FT0(void);
void helper_fucomi_ST0_FT0(void);
void helper_fadd_ST0_FT0(void);
void helper_fmul_ST0_FT0(void);
void helper_fsub_ST0_FT0(void);
void helper_fsubr_ST0_FT0(void);
void helper_fdiv_ST0_FT0(void);
void helper_fdivr_ST0_FT0(void);
void helper_fadd_STN_ST0(int st_index);
void helper_fmul_STN_ST0(int st_index);
void helper_fsub_STN_ST0(int st_index);
void helper_fsubr_STN_ST0(int st_index);
void helper_fdiv_STN_ST0(int st_index);
void helper_fdivr_STN_ST0(int st_index);
void helper_fchs_ST0(void);
void helper_fabs_ST0(void);
void helper_fxam_ST0(void);
void helper_fld1_ST0(void);
void helper_fldl2t_ST0(void);
void helper_fldl2e_ST0(void);
void helper_fldpi_ST0(void);
void helper_fldlg2_ST0(void);
void helper_fldln2_ST0(void);
void helper_fldz_ST0(void);
void helper_fldz_FT0(void);
uint32_t helper_fnstsw(void);
uint32_t helper_fnstcw(void);
void helper_fldcw(uint32_t val);
void helper_fclex(void);
void helper_fwait(void);
void helper_fninit(void);
void helper_fbld_ST0(target_ulong ptr);
void helper_fbst_ST0(target_ulong ptr);
void helper_f2xm1(void);
void helper_fyl2x(void);
void helper_fptan(void);
void helper_fpatan(void);
void helper_fxtract(void);
void helper_fprem1(void);
void helper_fprem(void);
void helper_fyl2xp1(void);
void helper_fsqrt(void);
void helper_fsincos(void);
void helper_frndint(void);
void helper_fscale(void);
void helper_fsin(void);
void helper_fcos(void);
void helper_fxam_ST0(void);
void helper_fstenv(target_ulong ptr, int data32);
void helper_fldenv(target_ulong ptr, int data32);
void helper_fsave(target_ulong ptr, int data32);
void helper_frstor(target_ulong ptr, int data32);
void helper_fxsave(target_ulong ptr, int data64);
void helper_fxrstor(target_ulong ptr, int data64);

/* MMX/SSE */

void TCG_HELPER_PROTO helper_enter_mmx(void);
void TCG_HELPER_PROTO helper_emms(void);
void TCG_HELPER_PROTO helper_movq(uint64_t *d, uint64_t *s);

#define SHIFT 0
#include "ops_sse_header.h"
#define SHIFT 1
#include "ops_sse_header.h"

target_ulong helper_rclb(target_ulong t0, target_ulong t1);
target_ulong helper_rclw(target_ulong t0, target_ulong t1);
target_ulong helper_rcll(target_ulong t0, target_ulong t1);
target_ulong helper_rcrb(target_ulong t0, target_ulong t1);
target_ulong helper_rcrw(target_ulong t0, target_ulong t1);
target_ulong helper_rcrl(target_ulong t0, target_ulong t1);
#ifdef TARGET_X86_64
target_ulong helper_rclq(target_ulong t0, target_ulong t1);
target_ulong helper_rcrq(target_ulong t0, target_ulong t1);
#endif

