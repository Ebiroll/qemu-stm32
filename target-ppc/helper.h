#include "def-helper.h"

DEF_HELPER_2(raise_exception_err, void, i32, i32)
DEF_HELPER_1(raise_exception, void, i32)
DEF_HELPER_3(tw, void, tl, tl, i32)
#if defined(TARGET_PPC64)
DEF_HELPER_3(td, void, tl, tl, i32)
#endif
#if !defined(CONFIG_USER_ONLY)
DEF_HELPER_1(store_msr, void, tl)
DEF_HELPER_0(rfi, void)
DEF_HELPER_0(rfsvc, void)
DEF_HELPER_0(40x_rfci, void)
DEF_HELPER_0(rfci, void)
DEF_HELPER_0(rfdi, void)
DEF_HELPER_0(rfmci, void)
#if defined(TARGET_PPC64)
DEF_HELPER_0(rfid, void)
DEF_HELPER_0(hrfid, void)
#endif
#endif

DEF_HELPER_2(lmw, void, tl, i32)
DEF_HELPER_2(stmw, void, tl, i32)
DEF_HELPER_3(lsw, void, tl, i32, i32)
DEF_HELPER_4(lswx, void, tl, i32, i32, i32)
DEF_HELPER_3(stsw, void, tl, i32, i32)
DEF_HELPER_1(dcbz, void, tl)
DEF_HELPER_1(dcbz_970, void, tl)
DEF_HELPER_1(icbi, void, tl)
DEF_HELPER_4(lscbx, tl, tl, i32, i32, i32)

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
DEF_HELPER_1(fpscr_clrbit, void, i32)
DEF_HELPER_1(fpscr_setbit, void, i32)
DEF_HELPER_1(float64_to_float32, i32, i64)
DEF_HELPER_1(float32_to_float64, i64, i32)

DEF_HELPER_3(fcmpo, void, i64, i64, i32)
DEF_HELPER_3(fcmpu, void, i64, i64, i32)

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

#define dh_alias_avr ptr
#define dh_ctype_avr ppc_avr_t *

DEF_HELPER_3(vaddubm, void, avr, avr, avr)
DEF_HELPER_3(vadduhm, void, avr, avr, avr)
DEF_HELPER_3(vadduwm, void, avr, avr, avr)
DEF_HELPER_3(vsububm, void, avr, avr, avr)
DEF_HELPER_3(vsubuhm, void, avr, avr, avr)
DEF_HELPER_3(vsubuwm, void, avr, avr, avr)
DEF_HELPER_3(vavgub, void, avr, avr, avr)
DEF_HELPER_3(vavguh, void, avr, avr, avr)
DEF_HELPER_3(vavguw, void, avr, avr, avr)
DEF_HELPER_3(vavgsb, void, avr, avr, avr)
DEF_HELPER_3(vavgsh, void, avr, avr, avr)
DEF_HELPER_3(vavgsw, void, avr, avr, avr)
DEF_HELPER_3(vminsb, void, avr, avr, avr)
DEF_HELPER_3(vminsh, void, avr, avr, avr)
DEF_HELPER_3(vminsw, void, avr, avr, avr)
DEF_HELPER_3(vmaxsb, void, avr, avr, avr)
DEF_HELPER_3(vmaxsh, void, avr, avr, avr)
DEF_HELPER_3(vmaxsw, void, avr, avr, avr)
DEF_HELPER_3(vminub, void, avr, avr, avr)
DEF_HELPER_3(vminuh, void, avr, avr, avr)
DEF_HELPER_3(vminuw, void, avr, avr, avr)
DEF_HELPER_3(vmaxub, void, avr, avr, avr)
DEF_HELPER_3(vmaxuh, void, avr, avr, avr)
DEF_HELPER_3(vmaxuw, void, avr, avr, avr)
DEF_HELPER_3(vmrglb, void, avr, avr, avr)
DEF_HELPER_3(vmrglh, void, avr, avr, avr)
DEF_HELPER_3(vmrglw, void, avr, avr, avr)
DEF_HELPER_3(vmrghb, void, avr, avr, avr)
DEF_HELPER_3(vmrghh, void, avr, avr, avr)
DEF_HELPER_3(vmrghw, void, avr, avr, avr)
DEF_HELPER_3(vmulesb, void, avr, avr, avr)
DEF_HELPER_3(vmulesh, void, avr, avr, avr)
DEF_HELPER_3(vmuleub, void, avr, avr, avr)
DEF_HELPER_3(vmuleuh, void, avr, avr, avr)
DEF_HELPER_3(vmulosb, void, avr, avr, avr)
DEF_HELPER_3(vmulosh, void, avr, avr, avr)
DEF_HELPER_3(vmuloub, void, avr, avr, avr)
DEF_HELPER_3(vmulouh, void, avr, avr, avr)
DEF_HELPER_3(vsrab, void, avr, avr, avr)
DEF_HELPER_3(vsrah, void, avr, avr, avr)
DEF_HELPER_3(vsraw, void, avr, avr, avr)
DEF_HELPER_3(vsrb, void, avr, avr, avr)
DEF_HELPER_3(vsrh, void, avr, avr, avr)
DEF_HELPER_3(vsrw, void, avr, avr, avr)
DEF_HELPER_3(vslb, void, avr, avr, avr)
DEF_HELPER_3(vslh, void, avr, avr, avr)
DEF_HELPER_3(vslw, void, avr, avr, avr)
DEF_HELPER_3(vslo, void, avr, avr, avr)
DEF_HELPER_3(vsro, void, avr, avr, avr)
DEF_HELPER_3(vaddcuw, void, avr, avr, avr)
DEF_HELPER_3(vsubcuw, void, avr, avr, avr)
DEF_HELPER_2(lvsl, void, avr, tl);
DEF_HELPER_2(lvsr, void, avr, tl);
DEF_HELPER_3(vrlb, void, avr, avr, avr)
DEF_HELPER_3(vrlh, void, avr, avr, avr)
DEF_HELPER_3(vrlw, void, avr, avr, avr)
DEF_HELPER_4(vsldoi, void, avr, avr, avr, i32)

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
DEF_HELPER_1(4xx_tlbre_hi, tl, tl)
DEF_HELPER_1(4xx_tlbre_lo, tl, tl)
DEF_HELPER_2(4xx_tlbwe_hi, void, tl, tl)
DEF_HELPER_2(4xx_tlbwe_lo, void, tl, tl)
DEF_HELPER_1(4xx_tlbsx, tl, tl)
DEF_HELPER_2(440_tlbre, tl, i32, tl)
DEF_HELPER_3(440_tlbwe, void, i32, tl, tl)
DEF_HELPER_1(440_tlbsx, tl, tl)
DEF_HELPER_1(6xx_tlbd, void, tl)
DEF_HELPER_1(6xx_tlbi, void, tl)
DEF_HELPER_1(74xx_tlbd, void, tl)
DEF_HELPER_1(74xx_tlbi, void, tl)
DEF_HELPER_0(tlbia, void)
DEF_HELPER_1(tlbie, void, tl)
#if defined(TARGET_PPC64)
DEF_HELPER_1(load_slb, tl, tl)
DEF_HELPER_2(store_slb, void, tl, tl)
DEF_HELPER_0(slbia, void)
DEF_HELPER_1(slbie, void, tl)
#endif
DEF_HELPER_1(load_sr, tl, tl);
DEF_HELPER_2(store_sr, void, tl, tl)

DEF_HELPER_1(602_mfrom, tl, tl)
#endif

DEF_HELPER_3(dlmzb, tl, tl, tl, i32)
DEF_HELPER_1(clcs, tl, i32)
#if !defined(CONFIG_USER_ONLY)
DEF_HELPER_1(rac, tl, tl)
#endif
DEF_HELPER_2(div, tl, tl, tl)
DEF_HELPER_2(divo, tl, tl, tl)
DEF_HELPER_2(divs, tl, tl, tl)
DEF_HELPER_2(divso, tl, tl, tl)

DEF_HELPER_1(load_dcr, tl, tl);
DEF_HELPER_2(store_dcr, void, tl, tl)

DEF_HELPER_1(load_dump_spr, void, i32)
DEF_HELPER_1(store_dump_spr, void, i32)
DEF_HELPER_0(load_tbl, tl)
DEF_HELPER_0(load_tbu, tl)
DEF_HELPER_0(load_atbl, tl)
DEF_HELPER_0(load_atbu, tl)
DEF_HELPER_0(load_601_rtcl, tl)
DEF_HELPER_0(load_601_rtcu, tl)
#if !defined(CONFIG_USER_ONLY)
#if defined(TARGET_PPC64)
DEF_HELPER_1(store_asr, void, tl)
#endif
DEF_HELPER_1(store_sdr1, void, tl)
DEF_HELPER_1(store_tbl, void, tl)
DEF_HELPER_1(store_tbu, void, tl)
DEF_HELPER_1(store_atbl, void, tl)
DEF_HELPER_1(store_atbu, void, tl)
DEF_HELPER_1(store_601_rtcl, void, tl)
DEF_HELPER_1(store_601_rtcu, void, tl)
DEF_HELPER_0(load_decr, tl)
DEF_HELPER_1(store_decr, void, tl)
DEF_HELPER_1(store_hid0_601, void, tl)
DEF_HELPER_2(store_403_pbr, void, i32, tl)
DEF_HELPER_0(load_40x_pit, tl)
DEF_HELPER_1(store_40x_pit, void, tl)
DEF_HELPER_1(store_40x_dbcr0, void, tl)
DEF_HELPER_1(store_40x_sler, void, tl)
DEF_HELPER_1(store_booke_tcr, void, tl)
DEF_HELPER_1(store_booke_tsr, void, tl)
DEF_HELPER_2(store_ibatl, void, i32, tl)
DEF_HELPER_2(store_ibatu, void, i32, tl)
DEF_HELPER_2(store_dbatl, void, i32, tl)
DEF_HELPER_2(store_dbatu, void, i32, tl)
DEF_HELPER_2(store_601_batl, void, i32, tl)
DEF_HELPER_2(store_601_batu, void, i32, tl)
#endif

#include "def-helper.h"
