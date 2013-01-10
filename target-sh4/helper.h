#include "exec/def-helper.h"

DEF_HELPER_1(ldtlb, void, env)
DEF_HELPER_1(raise_illegal_instruction, noreturn, env)
DEF_HELPER_1(raise_slot_illegal_instruction, noreturn, env)
DEF_HELPER_1(raise_fpu_disable, noreturn, env)
DEF_HELPER_1(raise_slot_fpu_disable, noreturn, env)
DEF_HELPER_1(debug, noreturn, env)
DEF_HELPER_1(sleep, noreturn, env)
DEF_HELPER_2(trapa, noreturn, env, i32)

DEF_HELPER_3(movcal, void, env, i32, i32)
DEF_HELPER_1(discard_movcal_backup, void, env)
DEF_HELPER_2(ocbi, void, env, i32)

DEF_HELPER_3(div1, i32, env, i32, i32)
DEF_HELPER_3(macl, void, env, i32, i32)
DEF_HELPER_3(macw, void, env, i32, i32)

DEF_HELPER_2(ld_fpscr, void, env, i32)

DEF_HELPER_FLAGS_1(fabs_FT, TCG_CALL_NO_RWG_SE, f32, f32)
DEF_HELPER_FLAGS_1(fabs_DT, TCG_CALL_NO_RWG_SE, f64, f64)
DEF_HELPER_3(fadd_FT, f32, env, f32, f32)
DEF_HELPER_3(fadd_DT, f64, env, f64, f64)
DEF_HELPER_2(fcnvsd_FT_DT, f64, env, f32)
DEF_HELPER_2(fcnvds_DT_FT, f32, env, f64)

DEF_HELPER_3(fcmp_eq_FT, void, env, f32, f32)
DEF_HELPER_3(fcmp_eq_DT, void, env, f64, f64)
DEF_HELPER_3(fcmp_gt_FT, void, env, f32, f32)
DEF_HELPER_3(fcmp_gt_DT, void, env, f64, f64)
DEF_HELPER_3(fdiv_FT, f32, env, f32, f32)
DEF_HELPER_3(fdiv_DT, f64, env, f64, f64)
DEF_HELPER_2(float_FT, f32, env, i32)
DEF_HELPER_2(float_DT, f64, env, i32)
DEF_HELPER_4(fmac_FT, f32, env, f32, f32, f32)
DEF_HELPER_3(fmul_FT, f32, env, f32, f32)
DEF_HELPER_3(fmul_DT, f64, env, f64, f64)
DEF_HELPER_FLAGS_1(fneg_T, TCG_CALL_NO_RWG_SE, f32, f32)
DEF_HELPER_3(fsub_FT, f32, env, f32, f32)
DEF_HELPER_3(fsub_DT, f64, env, f64, f64)
DEF_HELPER_2(fsqrt_FT, f32, env, f32)
DEF_HELPER_2(fsqrt_DT, f64, env, f64)
DEF_HELPER_2(ftrc_FT, i32, env, f32)
DEF_HELPER_2(ftrc_DT, i32, env, f64)
DEF_HELPER_3(fipr, void, env, i32, i32)
DEF_HELPER_2(ftrv, void, env, i32)

#include "exec/def-helper.h"
