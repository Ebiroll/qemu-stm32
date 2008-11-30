#include "def-helper.h"

DEF_HELPER_2(raise_exception_err, void, i32, i32)
DEF_HELPER_0(raise_debug, void)
DEF_HELPER_3(tw, void, tl, tl, i32)
#if defined(TARGET_PPC64)
DEF_HELPER_3(td, void, tl, tl, i32)
#endif

DEF_HELPER_2(fcmpo, i32, i64, i64)
DEF_HELPER_2(fcmpu, i32, i64, i64)

DEF_HELPER_0(load_cr, tl)
DEF_HELPER_2(store_cr, void, tl, i32)

#if defined(TARGET_PPC64)
DEF_HELPER_2(mulhd, i64, i64, i64)
DEF_HELPER_2(mulhdu, i64, i64, i64)
DEF_HELPER_2(mulldo, i64, i64, i64)
#endif

DEF_HELPER_1(cntlzw, tl, tl)
DEF_HELPER_1(popcntb, tl, tl)
DEF_HELPER_2(sraw, tl, tl, tl)
#if defined(TARGET_PPC64)
DEF_HELPER_1(cntlzd, tl, tl)
DEF_HELPER_1(popcntb_64, tl, tl)
DEF_HELPER_2(srad, tl, tl, tl)
#endif

DEF_HELPER_1(cntlsw32, i32, i32)
DEF_HELPER_1(cntlzw32, i32, i32)
DEF_HELPER_2(brinc, tl, tl, tl)

DEF_HELPER_0(float_check_status, void)
#ifdef CONFIG_SOFTFLOAT
DEF_HELPER_0(reset_fpstatus, void)
#endif
DEF_HELPER_2(compute_fprf, i32, i64, i32)
DEF_HELPER_2(store_fpscr, void, i64, i32)
DEF_HELPER_1(fpscr_setbit, void, i32)
DEF_HELPER_1(float64_to_float32, i32, i64)
DEF_HELPER_1(float32_to_float64, i64, i32)

DEF_HELPER_1(fctiw, i64, i64)
DEF_HELPER_1(fctiwz, i64, i64)
#if defined(TARGET_PPC64)
DEF_HELPER_1(fcfid, i64, i64)
DEF_HELPER_1(fctid, i64, i64)
DEF_HELPER_1(fctidz, i64, i64)
#endif
DEF_HELPER_1(frsp, i64, i64)
DEF_HELPER_1(frin, i64, i64)
DEF_HELPER_1(friz, i64, i64)
DEF_HELPER_1(frip, i64, i64)
DEF_HELPER_1(frim, i64, i64)

DEF_HELPER_2(fadd, i64, i64, i64)
DEF_HELPER_2(fsub, i64, i64, i64)
DEF_HELPER_2(fmul, i64, i64, i64)
DEF_HELPER_2(fdiv, i64, i64, i64)
DEF_HELPER_3(fmadd, i64, i64, i64, i64)
DEF_HELPER_3(fmsub, i64, i64, i64, i64)
DEF_HELPER_3(fnmadd, i64, i64, i64, i64)
DEF_HELPER_3(fnmsub, i64, i64, i64, i64)
DEF_HELPER_1(fabs, i64, i64)
DEF_HELPER_1(fnabs, i64, i64)
DEF_HELPER_1(fneg, i64, i64)
DEF_HELPER_1(fsqrt, i64, i64)
DEF_HELPER_1(fre, i64, i64)
DEF_HELPER_1(fres, i64, i64)
DEF_HELPER_1(frsqrte, i64, i64)
DEF_HELPER_3(fsel, i64, i64, i64, i64)

DEF_HELPER_1(efscfsi, i32, i32)
DEF_HELPER_1(efscfui, i32, i32)
DEF_HELPER_1(efscfuf, i32, i32)
DEF_HELPER_1(efscfsf, i32, i32)
DEF_HELPER_1(efsctsi, i32, i32)
DEF_HELPER_1(efsctui, i32, i32)
DEF_HELPER_1(efsctsiz, i32, i32)
DEF_HELPER_1(efsctuiz, i32, i32)
DEF_HELPER_1(efsctsf, i32, i32)
DEF_HELPER_1(efsctuf, i32, i32)
DEF_HELPER_1(evfscfsi, i64, i64)
DEF_HELPER_1(evfscfui, i64, i64)
DEF_HELPER_1(evfscfuf, i64, i64)
DEF_HELPER_1(evfscfsf, i64, i64)
DEF_HELPER_1(evfsctsi, i64, i64)
DEF_HELPER_1(evfsctui, i64, i64)
DEF_HELPER_1(evfsctsiz, i64, i64)
DEF_HELPER_1(evfsctuiz, i64, i64)
DEF_HELPER_1(evfsctsf, i64, i64)
DEF_HELPER_1(evfsctuf, i64, i64)
DEF_HELPER_2(efsadd, i32, i32, i32)
DEF_HELPER_2(efssub, i32, i32, i32)
DEF_HELPER_2(efsmul, i32, i32, i32)
DEF_HELPER_2(efsdiv, i32, i32, i32)
DEF_HELPER_2(evfsadd, i64, i64, i64)
DEF_HELPER_2(evfssub, i64, i64, i64)
DEF_HELPER_2(evfsmul, i64, i64, i64)
DEF_HELPER_2(evfsdiv, i64, i64, i64)
DEF_HELPER_2(efststlt, i32, i32, i32)
DEF_HELPER_2(efststgt, i32, i32, i32)
DEF_HELPER_2(efststeq, i32, i32, i32)
DEF_HELPER_2(efscmplt, i32, i32, i32)
DEF_HELPER_2(efscmpgt, i32, i32, i32)
DEF_HELPER_2(efscmpeq, i32, i32, i32)
DEF_HELPER_2(evfststlt, i32, i64, i64)
DEF_HELPER_2(evfststgt, i32, i64, i64)
DEF_HELPER_2(evfststeq, i32, i64, i64)
DEF_HELPER_2(evfscmplt, i32, i64, i64)
DEF_HELPER_2(evfscmpgt, i32, i64, i64)
DEF_HELPER_2(evfscmpeq, i32, i64, i64)
DEF_HELPER_1(efdcfsi, i64, i32)
DEF_HELPER_1(efdcfsid, i64, i64)
DEF_HELPER_1(efdcfui, i64, i32)
DEF_HELPER_1(efdcfuid, i64, i64)
DEF_HELPER_1(efdctsi, i32, i64)
DEF_HELPER_1(efdctui, i32, i64)
DEF_HELPER_1(efdctsiz, i32, i64)
DEF_HELPER_1(efdctsidz, i64, i64)
DEF_HELPER_1(efdctuiz, i32, i64)
DEF_HELPER_1(efdctuidz, i64, i64)
DEF_HELPER_1(efdcfsf, i64, i32)
DEF_HELPER_1(efdcfuf, i64, i32)
DEF_HELPER_1(efdctsf, i32, i64)
DEF_HELPER_1(efdctuf, i32, i64)
DEF_HELPER_1(efscfd, i32, i64)
DEF_HELPER_1(efdcfs, i64, i32)
DEF_HELPER_2(efdadd, i64, i64, i64)
DEF_HELPER_2(efdsub, i64, i64, i64)
DEF_HELPER_2(efdmul, i64, i64, i64)
DEF_HELPER_2(efddiv, i64, i64, i64)
DEF_HELPER_2(efdtstlt, i32, i64, i64)
DEF_HELPER_2(efdtstgt, i32, i64, i64)
DEF_HELPER_2(efdtsteq, i32, i64, i64)
DEF_HELPER_2(efdcmplt, i32, i64, i64)
DEF_HELPER_2(efdcmpgt, i32, i64, i64)
DEF_HELPER_2(efdcmpeq, i32, i64, i64)

#if !defined(CONFIG_USER_ONLY)
DEF_HELPER_1(load_6xx_tlbd, void, tl)
DEF_HELPER_1(load_6xx_tlbi, void, tl)
DEF_HELPER_1(load_74xx_tlbd, void, tl)
DEF_HELPER_1(load_74xx_tlbi, void, tl)

DEF_HELPER_1(602_mfrom, tl, tl)
#endif

#include "def-helper.h"
