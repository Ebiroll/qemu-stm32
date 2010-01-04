#include "def-helper.h"

DEF_HELPER_2(excp, void, int, int)
DEF_HELPER_0(load_pcc, i64)
DEF_HELPER_0(rc, i64)
DEF_HELPER_0(rs, i64)

DEF_HELPER_2(addqv, i64, i64, i64)
DEF_HELPER_2(addlv, i64, i64, i64)
DEF_HELPER_2(subqv, i64, i64, i64)
DEF_HELPER_2(sublv, i64, i64, i64)
DEF_HELPER_2(mullv, i64, i64, i64)
DEF_HELPER_2(mulqv, i64, i64, i64)
DEF_HELPER_2(umulh, i64, i64, i64)

DEF_HELPER_1(ctpop, i64, i64)
DEF_HELPER_1(ctlz, i64, i64)
DEF_HELPER_1(cttz, i64, i64)

DEF_HELPER_2(zap, i64, i64, i64)
DEF_HELPER_2(zapnot, i64, i64, i64)

DEF_HELPER_2(cmpbge, i64, i64, i64)

DEF_HELPER_2(minub8, i64, i64, i64)
DEF_HELPER_2(minsb8, i64, i64, i64)
DEF_HELPER_2(minuw4, i64, i64, i64)
DEF_HELPER_2(minsw4, i64, i64, i64)
DEF_HELPER_2(maxub8, i64, i64, i64)
DEF_HELPER_2(maxsb8, i64, i64, i64)
DEF_HELPER_2(maxuw4, i64, i64, i64)
DEF_HELPER_2(maxsw4, i64, i64, i64)
DEF_HELPER_2(perr, i64, i64, i64)
DEF_HELPER_1(pklb, i64, i64)
DEF_HELPER_1(pkwb, i64, i64)
DEF_HELPER_1(unpkbl, i64, i64)
DEF_HELPER_1(unpkbw, i64, i64)

DEF_HELPER_0(load_fpcr, i64)
DEF_HELPER_1(store_fpcr, void, i64)

DEF_HELPER_1(f_to_memory, i32, i64)
DEF_HELPER_1(memory_to_f, i64, i32)
DEF_HELPER_2(addf, i64, i64, i64)
DEF_HELPER_2(subf, i64, i64, i64)
DEF_HELPER_2(mulf, i64, i64, i64)
DEF_HELPER_2(divf, i64, i64, i64)
DEF_HELPER_1(sqrtf, i64, i64)

DEF_HELPER_1(g_to_memory, i64, i64)
DEF_HELPER_1(memory_to_g, i64, i64)
DEF_HELPER_2(addg, i64, i64, i64)
DEF_HELPER_2(subg, i64, i64, i64)
DEF_HELPER_2(mulg, i64, i64, i64)
DEF_HELPER_2(divg, i64, i64, i64)
DEF_HELPER_1(sqrtg, i64, i64)

DEF_HELPER_1(s_to_memory, i32, i64)
DEF_HELPER_1(memory_to_s, i64, i32)
DEF_HELPER_2(adds, i64, i64, i64)
DEF_HELPER_2(subs, i64, i64, i64)
DEF_HELPER_2(muls, i64, i64, i64)
DEF_HELPER_2(divs, i64, i64, i64)
DEF_HELPER_1(sqrts, i64, i64)

DEF_HELPER_2(addt, i64, i64, i64)
DEF_HELPER_2(subt, i64, i64, i64)
DEF_HELPER_2(mult, i64, i64, i64)
DEF_HELPER_2(divt, i64, i64, i64)
DEF_HELPER_1(sqrtt, i64, i64)

DEF_HELPER_2(cmptun, i64, i64, i64)
DEF_HELPER_2(cmpteq, i64, i64, i64)
DEF_HELPER_2(cmptle, i64, i64, i64)
DEF_HELPER_2(cmptlt, i64, i64, i64)
DEF_HELPER_2(cmpgeq, i64, i64, i64)
DEF_HELPER_2(cmpgle, i64, i64, i64)
DEF_HELPER_2(cmpglt, i64, i64, i64)

DEF_HELPER_2(cpys, i64, i64, i64)
DEF_HELPER_2(cpysn, i64, i64, i64)
DEF_HELPER_2(cpyse, i64, i64, i64)

DEF_HELPER_1(cvtts, i64, i64)
DEF_HELPER_1(cvtst, i64, i64)
DEF_HELPER_1(cvtqs, i64, i64)
DEF_HELPER_1(cvtqt, i64, i64)
DEF_HELPER_1(cvtqf, i64, i64)
DEF_HELPER_1(cvtgf, i64, i64)
DEF_HELPER_1(cvtgq, i64, i64)
DEF_HELPER_1(cvtqg, i64, i64)
DEF_HELPER_1(cvtlq, i64, i64)

DEF_HELPER_1(cvttq, i64, i64)
DEF_HELPER_1(cvttq_c, i64, i64)
DEF_HELPER_1(cvttq_svic, i64, i64)

DEF_HELPER_1(cvtql, i64, i64)
DEF_HELPER_1(cvtql_v, i64, i64)
DEF_HELPER_1(cvtql_sv, i64, i64)

DEF_HELPER_1(setroundmode, void, i32)
DEF_HELPER_1(setflushzero, void, i32)
DEF_HELPER_0(fp_exc_clear, void)
DEF_HELPER_0(fp_exc_get, i32)
DEF_HELPER_2(fp_exc_raise, void, i32, i32)
DEF_HELPER_2(fp_exc_raise_s, void, i32, i32)

DEF_HELPER_1(ieee_input, i64, i64)
DEF_HELPER_1(ieee_input_cmp, i64, i64)
DEF_HELPER_1(ieee_input_s, i64, i64)

#if !defined (CONFIG_USER_ONLY)
DEF_HELPER_0(hw_rei, void)
DEF_HELPER_1(hw_ret, void, i64)
DEF_HELPER_2(mfpr, i64, int, i64)
DEF_HELPER_2(mtpr, void, int, i64)
DEF_HELPER_0(set_alt_mode, void)
DEF_HELPER_0(restore_mode, void)

DEF_HELPER_1(ld_virt_to_phys, i64, i64)
DEF_HELPER_1(st_virt_to_phys, i64, i64)
DEF_HELPER_2(ldl_raw, void, i64, i64)
DEF_HELPER_2(ldq_raw, void, i64, i64)
DEF_HELPER_2(ldl_l_raw, void, i64, i64)
DEF_HELPER_2(ldq_l_raw, void, i64, i64)
DEF_HELPER_2(ldl_kernel, void, i64, i64)
DEF_HELPER_2(ldq_kernel, void, i64, i64)
DEF_HELPER_2(ldl_data, void, i64, i64)
DEF_HELPER_2(ldq_data, void, i64, i64)
DEF_HELPER_2(stl_raw, void, i64, i64)
DEF_HELPER_2(stq_raw, void, i64, i64)
DEF_HELPER_2(stl_c_raw, i64, i64, i64)
DEF_HELPER_2(stq_c_raw, i64, i64, i64)
#endif

#include "def-helper.h"
