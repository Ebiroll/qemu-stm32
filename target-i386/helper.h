#include "def-helper.h"

DEF_HELPER_FLAGS_1(cc_compute_all, TCG_CALL_PURE, i32, int)
DEF_HELPER_FLAGS_1(cc_compute_c, TCG_CALL_PURE, i32, int)

DEF_HELPER_0(lock, void)
DEF_HELPER_0(unlock, void)
DEF_HELPER_2(write_eflags, void, tl, i32)
DEF_HELPER_0(read_eflags, tl)
DEF_HELPER_1(divb_AL, void, tl)
DEF_HELPER_1(idivb_AL, void, tl)
DEF_HELPER_1(divw_AX, void, tl)
DEF_HELPER_1(idivw_AX, void, tl)
DEF_HELPER_1(divl_EAX, void, tl)
DEF_HELPER_1(idivl_EAX, void, tl)
#ifdef TARGET_X86_64
DEF_HELPER_1(mulq_EAX_T0, void, tl)
DEF_HELPER_1(imulq_EAX_T0, void, tl)
DEF_HELPER_2(imulq_T0_T1, tl, tl, tl)
DEF_HELPER_1(divq_EAX, void, tl)
DEF_HELPER_1(idivq_EAX, void, tl)
#endif

DEF_HELPER_1(aam, void, int)
DEF_HELPER_1(aad, void, int)
DEF_HELPER_0(aaa, void)
DEF_HELPER_0(aas, void)
DEF_HELPER_0(daa, void)
DEF_HELPER_0(das, void)

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
DEF_HELPER_0(clts, void)
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
DEF_HELPER_0(reset_rf, void)
DEF_HELPER_2(raise_interrupt, void, int, int)
DEF_HELPER_1(raise_exception, void, int)
DEF_HELPER_0(cli, void)
DEF_HELPER_0(sti, void)
DEF_HELPER_0(set_inhibit_irq, void)
DEF_HELPER_0(reset_inhibit_irq, void)
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

DEF_HELPER_2(svm_check_intercept_param, void, i32, i64)
DEF_HELPER_2(vmexit, void, i32, i64)
DEF_HELPER_3(svm_check_io, void, i32, i32, i32)
DEF_HELPER_2(vmrun, void, int, int)
DEF_HELPER_0(vmmcall, void)
DEF_HELPER_1(vmload, void, int)
DEF_HELPER_1(vmsave, void, int)
DEF_HELPER_0(stgi, void)
DEF_HELPER_0(clgi, void)
DEF_HELPER_0(skinit, void)
DEF_HELPER_1(invlpga, void, int)

/* x86 FPU */

DEF_HELPER_1(flds_FT0, void, i32)
DEF_HELPER_1(fldl_FT0, void, i64)
DEF_HELPER_1(fildl_FT0, void, s32)
DEF_HELPER_1(flds_ST0, void, i32)
DEF_HELPER_1(fldl_ST0, void, i64)
DEF_HELPER_1(fildl_ST0, void, s32)
DEF_HELPER_1(fildll_ST0, void, s64)
DEF_HELPER_0(fsts_ST0, i32)
DEF_HELPER_0(fstl_ST0, i64)
DEF_HELPER_0(fist_ST0, s32)
DEF_HELPER_0(fistl_ST0, s32)
DEF_HELPER_0(fistll_ST0, s64)
DEF_HELPER_0(fistt_ST0, s32)
DEF_HELPER_0(fisttl_ST0, s32)
DEF_HELPER_0(fisttll_ST0, s64)
DEF_HELPER_1(fldt_ST0, void, tl)
DEF_HELPER_1(fstt_ST0, void, tl)
DEF_HELPER_0(fpush, void)
DEF_HELPER_0(fpop, void)
DEF_HELPER_0(fdecstp, void)
DEF_HELPER_0(fincstp, void)
DEF_HELPER_1(ffree_STN, void, int)
DEF_HELPER_0(fmov_ST0_FT0, void)
DEF_HELPER_1(fmov_FT0_STN, void, int)
DEF_HELPER_1(fmov_ST0_STN, void, int)
DEF_HELPER_1(fmov_STN_ST0, void, int)
DEF_HELPER_1(fxchg_ST0_STN, void, int)
DEF_HELPER_0(fcom_ST0_FT0, void)
DEF_HELPER_0(fucom_ST0_FT0, void)
DEF_HELPER_0(fcomi_ST0_FT0, void)
DEF_HELPER_0(fucomi_ST0_FT0, void)
DEF_HELPER_0(fadd_ST0_FT0, void)
DEF_HELPER_0(fmul_ST0_FT0, void)
DEF_HELPER_0(fsub_ST0_FT0, void)
DEF_HELPER_0(fsubr_ST0_FT0, void)
DEF_HELPER_0(fdiv_ST0_FT0, void)
DEF_HELPER_0(fdivr_ST0_FT0, void)
DEF_HELPER_1(fadd_STN_ST0, void, int)
DEF_HELPER_1(fmul_STN_ST0, void, int)
DEF_HELPER_1(fsub_STN_ST0, void, int)
DEF_HELPER_1(fsubr_STN_ST0, void, int)
DEF_HELPER_1(fdiv_STN_ST0, void, int)
DEF_HELPER_1(fdivr_STN_ST0, void, int)
DEF_HELPER_0(fchs_ST0, void)
DEF_HELPER_0(fabs_ST0, void)
DEF_HELPER_0(fxam_ST0, void)
DEF_HELPER_0(fld1_ST0, void)
DEF_HELPER_0(fldl2t_ST0, void)
DEF_HELPER_0(fldl2e_ST0, void)
DEF_HELPER_0(fldpi_ST0, void)
DEF_HELPER_0(fldlg2_ST0, void)
DEF_HELPER_0(fldln2_ST0, void)
DEF_HELPER_0(fldz_ST0, void)
DEF_HELPER_0(fldz_FT0, void)
DEF_HELPER_0(fnstsw, i32)
DEF_HELPER_0(fnstcw, i32)
DEF_HELPER_1(fldcw, void, i32)
DEF_HELPER_0(fclex, void)
DEF_HELPER_0(fwait, void)
DEF_HELPER_0(fninit, void)
DEF_HELPER_1(fbld_ST0, void, tl)
DEF_HELPER_1(fbst_ST0, void, tl)
DEF_HELPER_0(f2xm1, void)
DEF_HELPER_0(fyl2x, void)
DEF_HELPER_0(fptan, void)
DEF_HELPER_0(fpatan, void)
DEF_HELPER_0(fxtract, void)
DEF_HELPER_0(fprem1, void)
DEF_HELPER_0(fprem, void)
DEF_HELPER_0(fyl2xp1, void)
DEF_HELPER_0(fsqrt, void)
DEF_HELPER_0(fsincos, void)
DEF_HELPER_0(frndint, void)
DEF_HELPER_0(fscale, void)
DEF_HELPER_0(fsin, void)
DEF_HELPER_0(fcos, void)
DEF_HELPER_2(fstenv, void, tl, int)
DEF_HELPER_2(fldenv, void, tl, int)
DEF_HELPER_2(fsave, void, tl, int)
DEF_HELPER_2(frstor, void, tl, int)
DEF_HELPER_2(fxsave, void, tl, int)
DEF_HELPER_2(fxrstor, void, tl, int)
DEF_HELPER_1(bsf, tl, tl)
DEF_HELPER_1(bsr, tl, tl)

/* MMX/SSE */

DEF_HELPER_0(enter_mmx, void)
DEF_HELPER_0(emms, void)
DEF_HELPER_2(movq, void, ptr, ptr)

#define SHIFT 0
#include "ops_sse_header.h"
#define SHIFT 1
#include "ops_sse_header.h"

DEF_HELPER_2(rclb, tl, tl, tl)
DEF_HELPER_2(rclw, tl, tl, tl)
DEF_HELPER_2(rcll, tl, tl, tl)
DEF_HELPER_2(rcrb, tl, tl, tl)
DEF_HELPER_2(rcrw, tl, tl, tl)
DEF_HELPER_2(rcrl, tl, tl, tl)
#ifdef TARGET_X86_64
DEF_HELPER_2(rclq, tl, tl, tl)
DEF_HELPER_2(rcrq, tl, tl, tl)
#endif

#include "def-helper.h"
