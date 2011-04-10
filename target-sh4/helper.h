#include "def-helper.h"

DEF_HELPER_0(ldtlb, void)
DEF_HELPER_0(raise_illegal_instruction, void)
DEF_HELPER_0(raise_slot_illegal_instruction, void)
DEF_HELPER_0(raise_fpu_disable, void)
DEF_HELPER_0(raise_slot_fpu_disable, void)
DEF_HELPER_0(debug, void)
DEF_HELPER_1(sleep, void, i32)
DEF_HELPER_1(trapa, void, i32)

DEF_HELPER_2(movcal, void, i32, i32)
DEF_HELPER_0(discard_movcal_backup, void)
DEF_HELPER_1(ocbi, void, i32)

DEF_HELPER_2(addv, i32, i32, i32)
DEF_HELPER_2(addc, i32, i32, i32)
DEF_HELPER_2(subv, i32, i32, i32)
DEF_HELPER_2(subc, i32, i32, i32)
DEF_HELPER_2(div1, i32, i32, i32)
DEF_HELPER_2(macl, void, i32, i32)
DEF_HELPER_2(macw, void, i32, i32)

DEF_HELPER_1(ld_fpscr, void, i32)

DEF_HELPER_1(fabs_FT, f32, f32)
DEF_HELPER_1(fabs_DT, f64, f64)
DEF_HELPER_2(fadd_FT, f32, f32, f32)
DEF_HELPER_2(fadd_DT, f64, f64, f64)
DEF_HELPER_1(fcnvsd_FT_DT, f64, f32)
DEF_HELPER_1(fcnvds_DT_FT, f32, f64)

DEF_HELPER_2(fcmp_eq_FT, void, f32, f32)
DEF_HELPER_2(fcmp_eq_DT, void, f64, f64)
DEF_HELPER_2(fcmp_gt_FT, void, f32, f32)
DEF_HELPER_2(fcmp_gt_DT, void, f64, f64)
DEF_HELPER_2(fdiv_FT, f32, f32, f32)
DEF_HELPER_2(fdiv_DT, f64, f64, f64)
DEF_HELPER_1(float_FT, f32, i32)
DEF_HELPER_1(float_DT, f64, i32)
DEF_HELPER_3(fmac_FT, f32, f32, f32, f32)
DEF_HELPER_2(fmul_FT, f32, f32, f32)
DEF_HELPER_2(fmul_DT, f64, f64, f64)
DEF_HELPER_1(fneg_T, f32, f32)
DEF_HELPER_2(fsub_FT, f32, f32, f32)
DEF_HELPER_2(fsub_DT, f64, f64, f64)
DEF_HELPER_1(fsqrt_FT, f32, f32)
DEF_HELPER_1(fsqrt_DT, f64, f64)
DEF_HELPER_1(ftrc_FT, i32, f32)
DEF_HELPER_1(ftrc_DT, i32, f64)
DEF_HELPER_2(fipr, void, i32, i32)
DEF_HELPER_1(ftrv, void, i32)

#include "def-helper.h"
