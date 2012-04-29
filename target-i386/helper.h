#include "def-helper.h"

DEF_HELPER_FLAGS_2(cc_compute_all, TCG_CALL_PURE, i32, env, int)
DEF_HELPER_FLAGS_2(cc_compute_c, TCG_CALL_PURE, i32, env, int)

DEF_HELPER_0(lock, void)
DEF_HELPER_0(unlock, void)
DEF_HELPER_3(write_eflags, void, env, tl, i32)
DEF_HELPER_1(read_eflags, tl, env)
DEF_HELPER_2(divb_AL, void, env, tl)
DEF_HELPER_2(idivb_AL, void, env, tl)
DEF_HELPER_2(divw_AX, void, env, tl)
DEF_HELPER_2(idivw_AX, void, env, tl)
DEF_HELPER_2(divl_EAX, void, env, tl)
DEF_HELPER_2(idivl_EAX, void, env, tl)
#ifdef TARGET_X86_64
DEF_HELPER_2(mulq_EAX_T0, void, env, tl)
DEF_HELPER_2(imulq_EAX_T0, void, env, tl)
DEF_HELPER_3(imulq_T0_T1, tl, env, tl, tl)
DEF_HELPER_2(divq_EAX, void, env, tl)
DEF_HELPER_2(idivq_EAX, void, env, tl)
#endif

DEF_HELPER_2(aam, void, env, int)
DEF_HELPER_2(aad, void, env, int)
DEF_HELPER_1(aaa, void, env)
DEF_HELPER_1(aas, void, env)
DEF_HELPER_1(daa, void, env)
DEF_HELPER_1(das, void, env)

DEF_HELPER_1(lsl, tl, tl)
DEF_HELPER_1(lar, tl, tl)
DEF_HELPER_1(verr, void, tl)
DEF_HELPER_1(verw, void, tl)
DEF_HELPER_1(lldt, void, int)
DEF_HELPER_1(ltr, void, int)
DEF_HELPER_2(load_seg, void, int, int)
DEF_HELPER_3(ljmp_protected, void, int, tl, int)
DEF_HELPER_4(lcall_real, void, int, tl, int, int)
DEF_HELPER_4(lcall_protected, void, int, tl, int, int)
DEF_HELPER_1(iret_real, void, int)
DEF_HELPER_2(iret_protected, void, int, int)
DEF_HELPER_2(lret_protected, void, int, int)
DEF_HELPER_1(read_crN, tl, int)
DEF_HELPER_2(write_crN, void, int, tl)
DEF_HELPER_1(lmsw, void, tl)
DEF_HELPER_1(clts, void, env)
DEF_HELPER_2(movl_drN_T0, void, int, tl)
DEF_HELPER_1(invlpg, void, tl)

DEF_HELPER_3(enter_level, void, int, int, tl)
#ifdef TARGET_X86_64
DEF_HELPER_3(enter64_level, void, int, int, tl)
#endif
DEF_HELPER_0(sysenter, void)
DEF_HELPER_1(sysexit, void, int)
#ifdef TARGET_X86_64
DEF_HELPER_1(syscall, void, int)
DEF_HELPER_1(sysret, void, int)
#endif
DEF_HELPER_1(hlt, void, int)
DEF_HELPER_1(monitor, void, tl)
DEF_HELPER_1(mwait, void, int)
DEF_HELPER_0(debug, void)
DEF_HELPER_1(reset_rf, void, env)
DEF_HELPER_3(raise_interrupt, void, env, int, int)
DEF_HELPER_2(raise_exception, void, env, int)
DEF_HELPER_1(cli, void, env)
DEF_HELPER_1(sti, void, env)
DEF_HELPER_1(set_inhibit_irq, void, env)
DEF_HELPER_1(reset_inhibit_irq, void, env)
DEF_HELPER_2(boundw, void, tl, int)
DEF_HELPER_2(boundl, void, tl, int)
DEF_HELPER_0(rsm, void)
DEF_HELPER_1(into, void, int)
DEF_HELPER_1(cmpxchg8b, void, tl)
#ifdef TARGET_X86_64
DEF_HELPER_1(cmpxchg16b, void, tl)
#endif
DEF_HELPER_0(single_step, void)
DEF_HELPER_0(cpuid, void)
DEF_HELPER_0(rdtsc, void)
DEF_HELPER_0(rdtscp, void)
DEF_HELPER_0(rdpmc, void)
DEF_HELPER_0(rdmsr, void)
DEF_HELPER_0(wrmsr, void)

DEF_HELPER_1(check_iob, void, i32)
DEF_HELPER_1(check_iow, void, i32)
DEF_HELPER_1(check_iol, void, i32)
DEF_HELPER_2(outb, void, i32, i32)
DEF_HELPER_1(inb, tl, i32)
DEF_HELPER_2(outw, void, i32, i32)
DEF_HELPER_1(inw, tl, i32)
DEF_HELPER_2(outl, void, i32, i32)
DEF_HELPER_1(inl, tl, i32)

DEF_HELPER_3(svm_check_intercept_param, void, env, i32, i64)
DEF_HELPER_3(vmexit, void, env, i32, i64)
DEF_HELPER_4(svm_check_io, void, env, i32, i32, i32)
DEF_HELPER_3(vmrun, void, env, int, int)
DEF_HELPER_1(vmmcall, void, env)
DEF_HELPER_2(vmload, void, env, int)
DEF_HELPER_2(vmsave, void, env, int)
DEF_HELPER_1(stgi, void, env)
DEF_HELPER_1(clgi, void, env)
DEF_HELPER_1(skinit, void, env)
DEF_HELPER_2(invlpga, void, env, int)

/* x86 FPU */

DEF_HELPER_2(flds_FT0, void, env, i32)
DEF_HELPER_2(fldl_FT0, void, env, i64)
DEF_HELPER_2(fildl_FT0, void, env, s32)
DEF_HELPER_2(flds_ST0, void, env, i32)
DEF_HELPER_2(fldl_ST0, void, env, i64)
DEF_HELPER_2(fildl_ST0, void, env, s32)
DEF_HELPER_2(fildll_ST0, void, env, s64)
DEF_HELPER_1(fsts_ST0, i32, env)
DEF_HELPER_1(fstl_ST0, i64, env)
DEF_HELPER_1(fist_ST0, s32, env)
DEF_HELPER_1(fistl_ST0, s32, env)
DEF_HELPER_1(fistll_ST0, s64, env)
DEF_HELPER_1(fistt_ST0, s32, env)
DEF_HELPER_1(fisttl_ST0, s32, env)
DEF_HELPER_1(fisttll_ST0, s64, env)
DEF_HELPER_2(fldt_ST0, void, env, tl)
DEF_HELPER_2(fstt_ST0, void, env, tl)
DEF_HELPER_1(fpush, void, env)
DEF_HELPER_1(fpop, void, env)
DEF_HELPER_1(fdecstp, void, env)
DEF_HELPER_1(fincstp, void, env)
DEF_HELPER_2(ffree_STN, void, env, int)
DEF_HELPER_1(fmov_ST0_FT0, void, env)
DEF_HELPER_2(fmov_FT0_STN, void, env, int)
DEF_HELPER_2(fmov_ST0_STN, void, env, int)
DEF_HELPER_2(fmov_STN_ST0, void, env, int)
DEF_HELPER_2(fxchg_ST0_STN, void, env, int)
DEF_HELPER_1(fcom_ST0_FT0, void, env)
DEF_HELPER_1(fucom_ST0_FT0, void, env)
DEF_HELPER_1(fcomi_ST0_FT0, void, env)
DEF_HELPER_1(fucomi_ST0_FT0, void, env)
DEF_HELPER_1(fadd_ST0_FT0, void, env)
DEF_HELPER_1(fmul_ST0_FT0, void, env)
DEF_HELPER_1(fsub_ST0_FT0, void, env)
DEF_HELPER_1(fsubr_ST0_FT0, void, env)
DEF_HELPER_1(fdiv_ST0_FT0, void, env)
DEF_HELPER_1(fdivr_ST0_FT0, void, env)
DEF_HELPER_2(fadd_STN_ST0, void, env, int)
DEF_HELPER_2(fmul_STN_ST0, void, env, int)
DEF_HELPER_2(fsub_STN_ST0, void, env, int)
DEF_HELPER_2(fsubr_STN_ST0, void, env, int)
DEF_HELPER_2(fdiv_STN_ST0, void, env, int)
DEF_HELPER_2(fdivr_STN_ST0, void, env, int)
DEF_HELPER_1(fchs_ST0, void, env)
DEF_HELPER_1(fabs_ST0, void, env)
DEF_HELPER_1(fxam_ST0, void, env)
DEF_HELPER_1(fld1_ST0, void, env)
DEF_HELPER_1(fldl2t_ST0, void, env)
DEF_HELPER_1(fldl2e_ST0, void, env)
DEF_HELPER_1(fldpi_ST0, void, env)
DEF_HELPER_1(fldlg2_ST0, void, env)
DEF_HELPER_1(fldln2_ST0, void, env)
DEF_HELPER_1(fldz_ST0, void, env)
DEF_HELPER_1(fldz_FT0, void, env)
DEF_HELPER_1(fnstsw, i32, env)
DEF_HELPER_1(fnstcw, i32, env)
DEF_HELPER_2(fldcw, void, env, i32)
DEF_HELPER_1(fclex, void, env)
DEF_HELPER_1(fwait, void, env)
DEF_HELPER_1(fninit, void, env)
DEF_HELPER_2(fbld_ST0, void, env, tl)
DEF_HELPER_2(fbst_ST0, void, env, tl)
DEF_HELPER_1(f2xm1, void, env)
DEF_HELPER_1(fyl2x, void, env)
DEF_HELPER_1(fptan, void, env)
DEF_HELPER_1(fpatan, void, env)
DEF_HELPER_1(fxtract, void, env)
DEF_HELPER_1(fprem1, void, env)
DEF_HELPER_1(fprem, void, env)
DEF_HELPER_1(fyl2xp1, void, env)
DEF_HELPER_1(fsqrt, void, env)
DEF_HELPER_1(fsincos, void, env)
DEF_HELPER_1(frndint, void, env)
DEF_HELPER_1(fscale, void, env)
DEF_HELPER_1(fsin, void, env)
DEF_HELPER_1(fcos, void, env)
DEF_HELPER_3(fstenv, void, env, tl, int)
DEF_HELPER_3(fldenv, void, env, tl, int)
DEF_HELPER_3(fsave, void, env, tl, int)
DEF_HELPER_3(frstor, void, env, tl, int)
DEF_HELPER_3(fxsave, void, env, tl, int)
DEF_HELPER_3(fxrstor, void, env, tl, int)
DEF_HELPER_1(bsf, tl, tl)
DEF_HELPER_1(bsr, tl, tl)
DEF_HELPER_2(lzcnt, tl, tl, int)

/* MMX/SSE */

DEF_HELPER_2(ldmxcsr, void, env, i32)
DEF_HELPER_1(enter_mmx, void, env)
DEF_HELPER_1(emms, void, env)
DEF_HELPER_3(movq, void, env, ptr, ptr)

#define SHIFT 0
#include "ops_sse_header.h"
#define SHIFT 1
#include "ops_sse_header.h"

DEF_HELPER_3(rclb, tl, env, tl, tl)
DEF_HELPER_3(rclw, tl, env, tl, tl)
DEF_HELPER_3(rcll, tl, env, tl, tl)
DEF_HELPER_3(rcrb, tl, env, tl, tl)
DEF_HELPER_3(rcrw, tl, env, tl, tl)
DEF_HELPER_3(rcrl, tl, env, tl, tl)
#ifdef TARGET_X86_64
DEF_HELPER_3(rclq, tl, env, tl, tl)
DEF_HELPER_3(rcrq, tl, env, tl, tl)
#endif

#include "def-helper.h"
