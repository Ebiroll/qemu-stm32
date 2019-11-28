DEF_HELPER_FLAGS_3(raise_exception_err, TCG_CALL_NO_WG, void, env, i32, i32)
DEF_HELPER_FLAGS_2(raise_exception, TCG_CALL_NO_WG, void, env, i32)
DEF_HELPER_FLAGS_4(tw, TCG_CALL_NO_WG, void, env, tl, tl, i32)
#if defined(TARGET_PPC64)
DEF_HELPER_FLAGS_4(td, TCG_CALL_NO_WG, void, env, tl, tl, i32)
#endif
#if !defined(CONFIG_USER_ONLY)
DEF_HELPER_2(store_msr, void, env, tl)
DEF_HELPER_1(rfi, void, env)
DEF_HELPER_1(rfsvc, void, env)
DEF_HELPER_1(40x_rfci, void, env)
DEF_HELPER_1(rfci, void, env)
DEF_HELPER_1(rfdi, void, env)
DEF_HELPER_1(rfmci, void, env)
#if defined(TARGET_PPC64)
DEF_HELPER_2(pminsn, void, env, i32)
DEF_HELPER_1(rfid, void, env)
DEF_HELPER_1(hrfid, void, env)
DEF_HELPER_2(store_lpcr, void, env, tl)
DEF_HELPER_2(store_pcr, void, env, tl)
#endif
DEF_HELPER_1(check_tlb_flush_local, void, env)
DEF_HELPER_1(check_tlb_flush_global, void, env)
#endif

DEF_HELPER_3(lmw, void, env, tl, i32)
DEF_HELPER_FLAGS_3(stmw, TCG_CALL_NO_WG, void, env, tl, i32)
DEF_HELPER_4(lsw, void, env, tl, i32, i32)
DEF_HELPER_5(lswx, void, env, tl, i32, i32, i32)
DEF_HELPER_FLAGS_4(stsw, TCG_CALL_NO_WG, void, env, tl, i32, i32)
DEF_HELPER_FLAGS_3(dcbz, TCG_CALL_NO_WG, void, env, tl, i32)
DEF_HELPER_FLAGS_3(dcbzep, TCG_CALL_NO_WG, void, env, tl, i32)
DEF_HELPER_FLAGS_2(icbi, TCG_CALL_NO_WG, void, env, tl)
DEF_HELPER_FLAGS_2(icbiep, TCG_CALL_NO_WG, void, env, tl)
DEF_HELPER_5(lscbx, tl, env, tl, i32, i32, i32)

#if defined(TARGET_PPC64)
DEF_HELPER_4(divdeu, i64, env, i64, i64, i32)
DEF_HELPER_4(divde, i64, env, i64, i64, i32)
#endif
DEF_HELPER_4(divweu, tl, env, tl, tl, i32)
DEF_HELPER_4(divwe, tl, env, tl, tl, i32)

DEF_HELPER_FLAGS_1(popcntb, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_2(cmpb, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_3(sraw, tl, env, tl, tl)
#if defined(TARGET_PPC64)
DEF_HELPER_FLAGS_2(cmpeqb, TCG_CALL_NO_RWG_SE, i32, tl, tl)
DEF_HELPER_FLAGS_1(popcntw, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_2(bpermd, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_3(srad, tl, env, tl, tl)
DEF_HELPER_0(darn32, tl)
DEF_HELPER_0(darn64, tl)
#endif

DEF_HELPER_FLAGS_1(cntlsw32, TCG_CALL_NO_RWG_SE, i32, i32)
DEF_HELPER_FLAGS_1(cntlzw32, TCG_CALL_NO_RWG_SE, i32, i32)
DEF_HELPER_FLAGS_2(brinc, TCG_CALL_NO_RWG_SE, tl, tl, tl)

DEF_HELPER_1(float_check_status, void, env)
DEF_HELPER_1(reset_fpstatus, void, env)
DEF_HELPER_2(compute_fprf_float64, void, env, i64)
DEF_HELPER_3(store_fpscr, void, env, i64, i32)
DEF_HELPER_2(fpscr_clrbit, void, env, i32)
DEF_HELPER_2(fpscr_setbit, void, env, i32)
DEF_HELPER_FLAGS_1(todouble, TCG_CALL_NO_RWG_SE, i64, i32)
DEF_HELPER_FLAGS_1(tosingle, TCG_CALL_NO_RWG_SE, i32, i64)

DEF_HELPER_4(fcmpo, void, env, i64, i64, i32)
DEF_HELPER_4(fcmpu, void, env, i64, i64, i32)

DEF_HELPER_2(fctiw, i64, env, i64)
DEF_HELPER_2(fctiwu, i64, env, i64)
DEF_HELPER_2(fctiwz, i64, env, i64)
DEF_HELPER_2(fctiwuz, i64, env, i64)
DEF_HELPER_2(fcfid, i64, env, i64)
DEF_HELPER_2(fcfidu, i64, env, i64)
DEF_HELPER_2(fcfids, i64, env, i64)
DEF_HELPER_2(fcfidus, i64, env, i64)
DEF_HELPER_2(fctid, i64, env, i64)
DEF_HELPER_2(fctidu, i64, env, i64)
DEF_HELPER_2(fctidz, i64, env, i64)
DEF_HELPER_2(fctiduz, i64, env, i64)
DEF_HELPER_2(frsp, i64, env, i64)
DEF_HELPER_2(frin, i64, env, i64)
DEF_HELPER_2(friz, i64, env, i64)
DEF_HELPER_2(frip, i64, env, i64)
DEF_HELPER_2(frim, i64, env, i64)

DEF_HELPER_3(fadd, f64, env, f64, f64)
DEF_HELPER_3(fsub, f64, env, f64, f64)
DEF_HELPER_3(fmul, f64, env, f64, f64)
DEF_HELPER_3(fdiv, f64, env, f64, f64)
DEF_HELPER_4(fmadd, i64, env, i64, i64, i64)
DEF_HELPER_4(fmsub, i64, env, i64, i64, i64)
DEF_HELPER_4(fnmadd, i64, env, i64, i64, i64)
DEF_HELPER_4(fnmsub, i64, env, i64, i64, i64)
DEF_HELPER_2(fsqrt, f64, env, f64)
DEF_HELPER_2(fre, i64, env, i64)
DEF_HELPER_2(fres, i64, env, i64)
DEF_HELPER_2(frsqrte, i64, env, i64)
DEF_HELPER_4(fsel, i64, env, i64, i64, i64)

DEF_HELPER_FLAGS_2(ftdiv, TCG_CALL_NO_RWG_SE, i32, i64, i64)
DEF_HELPER_FLAGS_1(ftsqrt, TCG_CALL_NO_RWG_SE, i32, i64)

#define dh_alias_avr ptr
#define dh_ctype_avr ppc_avr_t *
#define dh_is_signed_avr dh_is_signed_ptr

#define dh_alias_vsr ptr
#define dh_ctype_vsr ppc_vsr_t *
#define dh_is_signed_vsr dh_is_signed_ptr

DEF_HELPER_3(vavgub, void, avr, avr, avr)
DEF_HELPER_3(vavguh, void, avr, avr, avr)
DEF_HELPER_3(vavguw, void, avr, avr, avr)
DEF_HELPER_3(vabsdub, void, avr, avr, avr)
DEF_HELPER_3(vabsduh, void, avr, avr, avr)
DEF_HELPER_3(vabsduw, void, avr, avr, avr)
DEF_HELPER_3(vavgsb, void, avr, avr, avr)
DEF_HELPER_3(vavgsh, void, avr, avr, avr)
DEF_HELPER_3(vavgsw, void, avr, avr, avr)
DEF_HELPER_4(vcmpequb, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpequh, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpequw, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpequd, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpneb, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpneh, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpnew, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpnezb, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpnezh, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpnezw, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtub, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtuh, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtuw, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtud, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtsb, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtsh, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtsw, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtsd, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpeqfp, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgefp, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtfp, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpbfp, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpequb_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpequh_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpequw_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpequd_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpneb_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpneh_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpnew_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpnezb_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpnezh_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpnezw_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtub_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtuh_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtuw_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtud_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtsb_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtsh_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtsw_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtsd_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpeqfp_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgefp_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtfp_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpbfp_dot, void, env, avr, avr, avr)
DEF_HELPER_3(vmrglb, void, avr, avr, avr)
DEF_HELPER_3(vmrglh, void, avr, avr, avr)
DEF_HELPER_3(vmrglw, void, avr, avr, avr)
DEF_HELPER_3(vmrghb, void, avr, avr, avr)
DEF_HELPER_3(vmrghh, void, avr, avr, avr)
DEF_HELPER_3(vmrghw, void, avr, avr, avr)
DEF_HELPER_3(vmulesb, void, avr, avr, avr)
DEF_HELPER_3(vmulesh, void, avr, avr, avr)
DEF_HELPER_3(vmulesw, void, avr, avr, avr)
DEF_HELPER_3(vmuleub, void, avr, avr, avr)
DEF_HELPER_3(vmuleuh, void, avr, avr, avr)
DEF_HELPER_3(vmuleuw, void, avr, avr, avr)
DEF_HELPER_3(vmulosb, void, avr, avr, avr)
DEF_HELPER_3(vmulosh, void, avr, avr, avr)
DEF_HELPER_3(vmulosw, void, avr, avr, avr)
DEF_HELPER_3(vmuloub, void, avr, avr, avr)
DEF_HELPER_3(vmulouh, void, avr, avr, avr)
DEF_HELPER_3(vmulouw, void, avr, avr, avr)
DEF_HELPER_3(vmuluwm, void, avr, avr, avr)
DEF_HELPER_3(vslo, void, avr, avr, avr)
DEF_HELPER_3(vsro, void, avr, avr, avr)
DEF_HELPER_3(vsrv, void, avr, avr, avr)
DEF_HELPER_3(vslv, void, avr, avr, avr)
DEF_HELPER_3(vaddcuw, void, avr, avr, avr)
DEF_HELPER_2(vprtybw, void, avr, avr)
DEF_HELPER_2(vprtybd, void, avr, avr)
DEF_HELPER_2(vprtybq, void, avr, avr)
DEF_HELPER_3(vsubcuw, void, avr, avr, avr)
DEF_HELPER_FLAGS_5(vaddsbs, TCG_CALL_NO_RWG, void, avr, avr, avr, avr, i32)
DEF_HELPER_FLAGS_5(vaddshs, TCG_CALL_NO_RWG, void, avr, avr, avr, avr, i32)
DEF_HELPER_FLAGS_5(vaddsws, TCG_CALL_NO_RWG, void, avr, avr, avr, avr, i32)
DEF_HELPER_FLAGS_5(vsubsbs, TCG_CALL_NO_RWG, void, avr, avr, avr, avr, i32)
DEF_HELPER_FLAGS_5(vsubshs, TCG_CALL_NO_RWG, void, avr, avr, avr, avr, i32)
DEF_HELPER_FLAGS_5(vsubsws, TCG_CALL_NO_RWG, void, avr, avr, avr, avr, i32)
DEF_HELPER_FLAGS_5(vaddubs, TCG_CALL_NO_RWG, void, avr, avr, avr, avr, i32)
DEF_HELPER_FLAGS_5(vadduhs, TCG_CALL_NO_RWG, void, avr, avr, avr, avr, i32)
DEF_HELPER_FLAGS_5(vadduws, TCG_CALL_NO_RWG, void, avr, avr, avr, avr, i32)
DEF_HELPER_FLAGS_5(vsububs, TCG_CALL_NO_RWG, void, avr, avr, avr, avr, i32)
DEF_HELPER_FLAGS_5(vsubuhs, TCG_CALL_NO_RWG, void, avr, avr, avr, avr, i32)
DEF_HELPER_FLAGS_5(vsubuws, TCG_CALL_NO_RWG, void, avr, avr, avr, avr, i32)
DEF_HELPER_3(vadduqm, void, avr, avr, avr)
DEF_HELPER_4(vaddecuq, void, avr, avr, avr, avr)
DEF_HELPER_4(vaddeuqm, void, avr, avr, avr, avr)
DEF_HELPER_3(vaddcuq, void, avr, avr, avr)
DEF_HELPER_3(vsubuqm, void, avr, avr, avr)
DEF_HELPER_4(vsubecuq, void, avr, avr, avr, avr)
DEF_HELPER_4(vsubeuqm, void, avr, avr, avr, avr)
DEF_HELPER_3(vsubcuq, void, avr, avr, avr)
DEF_HELPER_3(vrlb, void, avr, avr, avr)
DEF_HELPER_3(vrlh, void, avr, avr, avr)
DEF_HELPER_3(vrlw, void, avr, avr, avr)
DEF_HELPER_3(vrld, void, avr, avr, avr)
DEF_HELPER_4(vsldoi, void, avr, avr, avr, i32)
DEF_HELPER_3(vextractub, void, avr, avr, i32)
DEF_HELPER_3(vextractuh, void, avr, avr, i32)
DEF_HELPER_3(vextractuw, void, avr, avr, i32)
DEF_HELPER_3(vextractd, void, avr, avr, i32)
DEF_HELPER_3(vinsertb, void, avr, avr, i32)
DEF_HELPER_3(vinserth, void, avr, avr, i32)
DEF_HELPER_3(vinsertw, void, avr, avr, i32)
DEF_HELPER_3(vinsertd, void, avr, avr, i32)
DEF_HELPER_2(vextsb2w, void, avr, avr)
DEF_HELPER_2(vextsh2w, void, avr, avr)
DEF_HELPER_2(vextsb2d, void, avr, avr)
DEF_HELPER_2(vextsh2d, void, avr, avr)
DEF_HELPER_2(vextsw2d, void, avr, avr)
DEF_HELPER_2(vnegw, void, avr, avr)
DEF_HELPER_2(vnegd, void, avr, avr)
DEF_HELPER_2(vupkhpx, void, avr, avr)
DEF_HELPER_2(vupklpx, void, avr, avr)
DEF_HELPER_2(vupkhsb, void, avr, avr)
DEF_HELPER_2(vupkhsh, void, avr, avr)
DEF_HELPER_2(vupkhsw, void, avr, avr)
DEF_HELPER_2(vupklsb, void, avr, avr)
DEF_HELPER_2(vupklsh, void, avr, avr)
DEF_HELPER_2(vupklsw, void, avr, avr)
DEF_HELPER_5(vmsumubm, void, env, avr, avr, avr, avr)
DEF_HELPER_5(vmsummbm, void, env, avr, avr, avr, avr)
DEF_HELPER_5(vsel, void, env, avr, avr, avr, avr)
DEF_HELPER_5(vperm, void, env, avr, avr, avr, avr)
DEF_HELPER_5(vpermr, void, env, avr, avr, avr, avr)
DEF_HELPER_4(vpkshss, void, env, avr, avr, avr)
DEF_HELPER_4(vpkshus, void, env, avr, avr, avr)
DEF_HELPER_4(vpkswss, void, env, avr, avr, avr)
DEF_HELPER_4(vpkswus, void, env, avr, avr, avr)
DEF_HELPER_4(vpksdss, void, env, avr, avr, avr)
DEF_HELPER_4(vpksdus, void, env, avr, avr, avr)
DEF_HELPER_4(vpkuhus, void, env, avr, avr, avr)
DEF_HELPER_4(vpkuwus, void, env, avr, avr, avr)
DEF_HELPER_4(vpkudus, void, env, avr, avr, avr)
DEF_HELPER_4(vpkuhum, void, env, avr, avr, avr)
DEF_HELPER_4(vpkuwum, void, env, avr, avr, avr)
DEF_HELPER_4(vpkudum, void, env, avr, avr, avr)
DEF_HELPER_3(vpkpx, void, avr, avr, avr)
DEF_HELPER_5(vmhaddshs, void, env, avr, avr, avr, avr)
DEF_HELPER_5(vmhraddshs, void, env, avr, avr, avr, avr)
DEF_HELPER_5(vmsumuhm, void, env, avr, avr, avr, avr)
DEF_HELPER_5(vmsumuhs, void, env, avr, avr, avr, avr)
DEF_HELPER_5(vmsumshm, void, env, avr, avr, avr, avr)
DEF_HELPER_5(vmsumshs, void, env, avr, avr, avr, avr)
DEF_HELPER_4(vmladduhm, void, avr, avr, avr, avr)
DEF_HELPER_FLAGS_2(mtvscr, TCG_CALL_NO_RWG, void, env, i32)
DEF_HELPER_FLAGS_1(mfvscr, TCG_CALL_NO_RWG, i32, env)
DEF_HELPER_3(lvebx, void, env, avr, tl)
DEF_HELPER_3(lvehx, void, env, avr, tl)
DEF_HELPER_3(lvewx, void, env, avr, tl)
DEF_HELPER_3(stvebx, void, env, avr, tl)
DEF_HELPER_3(stvehx, void, env, avr, tl)
DEF_HELPER_3(stvewx, void, env, avr, tl)
#if defined(TARGET_PPC64)
DEF_HELPER_4(lxvl, void, env, tl, vsr, tl)
DEF_HELPER_4(lxvll, void, env, tl, vsr, tl)
DEF_HELPER_4(stxvl, void, env, tl, vsr, tl)
DEF_HELPER_4(stxvll, void, env, tl, vsr, tl)
#endif
DEF_HELPER_4(vsumsws, void, env, avr, avr, avr)
DEF_HELPER_4(vsum2sws, void, env, avr, avr, avr)
DEF_HELPER_4(vsum4sbs, void, env, avr, avr, avr)
DEF_HELPER_4(vsum4shs, void, env, avr, avr, avr)
DEF_HELPER_4(vsum4ubs, void, env, avr, avr, avr)
DEF_HELPER_4(vaddfp, void, env, avr, avr, avr)
DEF_HELPER_4(vsubfp, void, env, avr, avr, avr)
DEF_HELPER_4(vmaxfp, void, env, avr, avr, avr)
DEF_HELPER_4(vminfp, void, env, avr, avr, avr)
DEF_HELPER_3(vrefp, void, env, avr, avr)
DEF_HELPER_3(vrsqrtefp, void, env, avr, avr)
DEF_HELPER_3(vrlwmi, void, avr, avr, avr)
DEF_HELPER_3(vrldmi, void, avr, avr, avr)
DEF_HELPER_3(vrldnm, void, avr, avr, avr)
DEF_HELPER_3(vrlwnm, void, avr, avr, avr)
DEF_HELPER_5(vmaddfp, void, env, avr, avr, avr, avr)
DEF_HELPER_5(vnmsubfp, void, env, avr, avr, avr, avr)
DEF_HELPER_3(vexptefp, void, env, avr, avr)
DEF_HELPER_3(vlogefp, void, env, avr, avr)
DEF_HELPER_3(vrfim, void, env, avr, avr)
DEF_HELPER_3(vrfin, void, env, avr, avr)
DEF_HELPER_3(vrfip, void, env, avr, avr)
DEF_HELPER_3(vrfiz, void, env, avr, avr)
DEF_HELPER_4(vcfux, void, env, avr, avr, i32)
DEF_HELPER_4(vcfsx, void, env, avr, avr, i32)
DEF_HELPER_4(vctuxs, void, env, avr, avr, i32)
DEF_HELPER_4(vctsxs, void, env, avr, avr, i32)

DEF_HELPER_2(vclzb, void, avr, avr)
DEF_HELPER_2(vclzh, void, avr, avr)
DEF_HELPER_2(vctzb, void, avr, avr)
DEF_HELPER_2(vctzh, void, avr, avr)
DEF_HELPER_2(vctzw, void, avr, avr)
DEF_HELPER_2(vctzd, void, avr, avr)
DEF_HELPER_2(vpopcntb, void, avr, avr)
DEF_HELPER_2(vpopcnth, void, avr, avr)
DEF_HELPER_2(vpopcntw, void, avr, avr)
DEF_HELPER_2(vpopcntd, void, avr, avr)
DEF_HELPER_1(vclzlsbb, tl, avr)
DEF_HELPER_1(vctzlsbb, tl, avr)
DEF_HELPER_3(vbpermd, void, avr, avr, avr)
DEF_HELPER_3(vbpermq, void, avr, avr, avr)
DEF_HELPER_3(vpmsumb, void, avr, avr, avr)
DEF_HELPER_3(vpmsumh, void, avr, avr, avr)
DEF_HELPER_3(vpmsumw, void, avr, avr, avr)
DEF_HELPER_3(vpmsumd, void, avr, avr, avr)
DEF_HELPER_2(vextublx, tl, tl, avr)
DEF_HELPER_2(vextuhlx, tl, tl, avr)
DEF_HELPER_2(vextuwlx, tl, tl, avr)
DEF_HELPER_2(vextubrx, tl, tl, avr)
DEF_HELPER_2(vextuhrx, tl, tl, avr)
DEF_HELPER_2(vextuwrx, tl, tl, avr)

DEF_HELPER_2(vsbox, void, avr, avr)
DEF_HELPER_3(vcipher, void, avr, avr, avr)
DEF_HELPER_3(vcipherlast, void, avr, avr, avr)
DEF_HELPER_3(vncipher, void, avr, avr, avr)
DEF_HELPER_3(vncipherlast, void, avr, avr, avr)
DEF_HELPER_3(vshasigmaw, void, avr, avr, i32)
DEF_HELPER_3(vshasigmad, void, avr, avr, i32)
DEF_HELPER_4(vpermxor, void, avr, avr, avr, avr)

DEF_HELPER_4(bcdadd, i32, avr, avr, avr, i32)
DEF_HELPER_4(bcdsub, i32, avr, avr, avr, i32)
DEF_HELPER_3(bcdcfn, i32, avr, avr, i32)
DEF_HELPER_3(bcdctn, i32, avr, avr, i32)
DEF_HELPER_3(bcdcfz, i32, avr, avr, i32)
DEF_HELPER_3(bcdctz, i32, avr, avr, i32)
DEF_HELPER_3(bcdcfsq, i32, avr, avr, i32)
DEF_HELPER_3(bcdctsq, i32, avr, avr, i32)
DEF_HELPER_4(bcdcpsgn, i32, avr, avr, avr, i32)
DEF_HELPER_3(bcdsetsgn, i32, avr, avr, i32)
DEF_HELPER_4(bcds, i32, avr, avr, avr, i32)
DEF_HELPER_4(bcdus, i32, avr, avr, avr, i32)
DEF_HELPER_4(bcdsr, i32, avr, avr, avr, i32)
DEF_HELPER_4(bcdtrunc, i32, avr, avr, avr, i32)
DEF_HELPER_4(bcdutrunc, i32, avr, avr, avr, i32)

DEF_HELPER_4(xsadddp, void, env, vsr, vsr, vsr)
DEF_HELPER_5(xsaddqp, void, env, i32, vsr, vsr, vsr)
DEF_HELPER_4(xssubdp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xsmuldp, void, env, vsr, vsr, vsr)
DEF_HELPER_5(xsmulqp, void, env, i32, vsr, vsr, vsr)
DEF_HELPER_4(xsdivdp, void, env, vsr, vsr, vsr)
DEF_HELPER_5(xsdivqp, void, env, i32, vsr, vsr, vsr)
DEF_HELPER_3(xsredp, void, env, vsr, vsr)
DEF_HELPER_3(xssqrtdp, void, env, vsr, vsr)
DEF_HELPER_3(xsrsqrtedp, void, env, vsr, vsr)
DEF_HELPER_4(xstdivdp, void, env, i32, vsr, vsr)
DEF_HELPER_3(xstsqrtdp, void, env, i32, vsr)
DEF_HELPER_5(xsmadddp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(xsmsubdp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(xsnmadddp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(xsnmsubdp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_4(xscmpeqdp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xscmpgtdp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xscmpgedp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xscmpnedp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xscmpexpdp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscmpexpqp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscmpodp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscmpudp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscmpoqp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscmpuqp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xsmaxdp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xsmindp, void, env, vsr, vsr, vsr)
DEF_HELPER_5(xsmaxcdp, void, env, i32, vsr, vsr, vsr)
DEF_HELPER_5(xsmincdp, void, env, i32, vsr, vsr, vsr)
DEF_HELPER_5(xsmaxjdp, void, env, i32, vsr, vsr, vsr)
DEF_HELPER_5(xsminjdp, void, env, i32, vsr, vsr, vsr)
DEF_HELPER_3(xscvdphp, void, env, vsr, vsr)
DEF_HELPER_4(xscvdpqp, void, env, i32, vsr, vsr)
DEF_HELPER_3(xscvdpsp, void, env, vsr, vsr)
DEF_HELPER_2(xscvdpspn, i64, env, i64)
DEF_HELPER_4(xscvqpdp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscvqpsdz, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscvqpswz, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscvqpudz, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscvqpuwz, void, env, i32, vsr, vsr)
DEF_HELPER_3(xscvhpdp, void, env, vsr, vsr)
DEF_HELPER_4(xscvsdqp, void, env, i32, vsr, vsr)
DEF_HELPER_3(xscvspdp, void, env, vsr, vsr)
DEF_HELPER_2(xscvspdpn, i64, env, i64)
DEF_HELPER_3(xscvdpsxds, void, env, vsr, vsr)
DEF_HELPER_3(xscvdpsxws, void, env, vsr, vsr)
DEF_HELPER_3(xscvdpuxds, void, env, vsr, vsr)
DEF_HELPER_3(xscvdpuxws, void, env, vsr, vsr)
DEF_HELPER_3(xscvsxddp, void, env, vsr, vsr)
DEF_HELPER_3(xscvuxdsp, void, env, vsr, vsr)
DEF_HELPER_3(xscvsxdsp, void, env, vsr, vsr)
DEF_HELPER_4(xscvudqp, void, env, i32, vsr, vsr)
DEF_HELPER_3(xscvuxddp, void, env, vsr, vsr)
DEF_HELPER_3(xststdcsp, void, env, i32, vsr)
DEF_HELPER_2(xststdcdp, void, env, i32)
DEF_HELPER_2(xststdcqp, void, env, i32)
DEF_HELPER_3(xsrdpi, void, env, vsr, vsr)
DEF_HELPER_3(xsrdpic, void, env, vsr, vsr)
DEF_HELPER_3(xsrdpim, void, env, vsr, vsr)
DEF_HELPER_3(xsrdpip, void, env, vsr, vsr)
DEF_HELPER_3(xsrdpiz, void, env, vsr, vsr)
DEF_HELPER_4(xsrqpi, void, env, i32, vsr, vsr)
DEF_HELPER_4(xsrqpxp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xssqrtqp, void, env, i32, vsr, vsr)
DEF_HELPER_5(xssubqp, void, env, i32, vsr, vsr, vsr)

DEF_HELPER_4(xsaddsp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xssubsp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xsmulsp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xsdivsp, void, env, vsr, vsr, vsr)
DEF_HELPER_3(xsresp, void, env, vsr, vsr)
DEF_HELPER_2(xsrsp, i64, env, i64)
DEF_HELPER_3(xssqrtsp, void, env, vsr, vsr)
DEF_HELPER_3(xsrsqrtesp, void, env, vsr, vsr)
DEF_HELPER_5(xsmaddsp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(xsmsubsp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(xsnmaddsp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(xsnmsubsp, void, env, vsr, vsr, vsr, vsr)

DEF_HELPER_4(xvadddp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xvsubdp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xvmuldp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xvdivdp, void, env, vsr, vsr, vsr)
DEF_HELPER_3(xvredp, void, env, vsr, vsr)
DEF_HELPER_3(xvsqrtdp, void, env, vsr, vsr)
DEF_HELPER_3(xvrsqrtedp, void, env, vsr, vsr)
DEF_HELPER_4(xvtdivdp, void, env, i32, vsr, vsr)
DEF_HELPER_3(xvtsqrtdp, void, env, i32, vsr)
DEF_HELPER_5(xvmadddp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(xvmsubdp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(xvnmadddp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(xvnmsubdp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_4(xvmaxdp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xvmindp, void, env, vsr, vsr, vsr)
DEF_HELPER_FLAGS_4(xvcmpeqdp, TCG_CALL_NO_RWG, i32, env, vsr, vsr, vsr)
DEF_HELPER_FLAGS_4(xvcmpgedp, TCG_CALL_NO_RWG, i32, env, vsr, vsr, vsr)
DEF_HELPER_FLAGS_4(xvcmpgtdp, TCG_CALL_NO_RWG, i32, env, vsr, vsr, vsr)
DEF_HELPER_FLAGS_4(xvcmpnedp, TCG_CALL_NO_RWG, i32, env, vsr, vsr, vsr)
DEF_HELPER_3(xvcvdpsp, void, env, vsr, vsr)
DEF_HELPER_3(xvcvdpsxds, void, env, vsr, vsr)
DEF_HELPER_3(xvcvdpsxws, void, env, vsr, vsr)
DEF_HELPER_3(xvcvdpuxds, void, env, vsr, vsr)
DEF_HELPER_3(xvcvdpuxws, void, env, vsr, vsr)
DEF_HELPER_3(xvcvsxddp, void, env, vsr, vsr)
DEF_HELPER_3(xvcvuxddp, void, env, vsr, vsr)
DEF_HELPER_3(xvcvsxwdp, void, env, vsr, vsr)
DEF_HELPER_3(xvcvuxwdp, void, env, vsr, vsr)
DEF_HELPER_3(xvrdpi, void, env, vsr, vsr)
DEF_HELPER_3(xvrdpic, void, env, vsr, vsr)
DEF_HELPER_3(xvrdpim, void, env, vsr, vsr)
DEF_HELPER_3(xvrdpip, void, env, vsr, vsr)
DEF_HELPER_3(xvrdpiz, void, env, vsr, vsr)

DEF_HELPER_4(xvaddsp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xvsubsp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xvmulsp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xvdivsp, void, env, vsr, vsr, vsr)
DEF_HELPER_3(xvresp, void, env, vsr, vsr)
DEF_HELPER_3(xvsqrtsp, void, env, vsr, vsr)
DEF_HELPER_3(xvrsqrtesp, void, env, vsr, vsr)
DEF_HELPER_4(xvtdivsp, void, env, i32, vsr, vsr)
DEF_HELPER_3(xvtsqrtsp, void, env, i32, vsr)
DEF_HELPER_5(xvmaddsp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(xvmsubsp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(xvnmaddsp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(xvnmsubsp, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_4(xvmaxsp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xvminsp, void, env, vsr, vsr, vsr)
DEF_HELPER_FLAGS_4(xvcmpeqsp, TCG_CALL_NO_RWG, i32, env, vsr, vsr, vsr)
DEF_HELPER_FLAGS_4(xvcmpgesp, TCG_CALL_NO_RWG, i32, env, vsr, vsr, vsr)
DEF_HELPER_FLAGS_4(xvcmpgtsp, TCG_CALL_NO_RWG, i32, env, vsr, vsr, vsr)
DEF_HELPER_FLAGS_4(xvcmpnesp, TCG_CALL_NO_RWG, i32, env, vsr, vsr, vsr)
DEF_HELPER_3(xvcvspdp, void, env, vsr, vsr)
DEF_HELPER_3(xvcvsphp, void, env, vsr, vsr)
DEF_HELPER_3(xvcvhpsp, void, env, vsr, vsr)
DEF_HELPER_3(xvcvspsxds, void, env, vsr, vsr)
DEF_HELPER_3(xvcvspsxws, void, env, vsr, vsr)
DEF_HELPER_3(xvcvspuxds, void, env, vsr, vsr)
DEF_HELPER_3(xvcvspuxws, void, env, vsr, vsr)
DEF_HELPER_3(xvcvsxdsp, void, env, vsr, vsr)
DEF_HELPER_3(xvcvuxdsp, void, env, vsr, vsr)
DEF_HELPER_3(xvcvsxwsp, void, env, vsr, vsr)
DEF_HELPER_3(xvcvuxwsp, void, env, vsr, vsr)
DEF_HELPER_2(xvtstdcsp, void, env, i32)
DEF_HELPER_2(xvtstdcdp, void, env, i32)
DEF_HELPER_3(xvrspi, void, env, vsr, vsr)
DEF_HELPER_3(xvrspic, void, env, vsr, vsr)
DEF_HELPER_3(xvrspim, void, env, vsr, vsr)
DEF_HELPER_3(xvrspip, void, env, vsr, vsr)
DEF_HELPER_3(xvrspiz, void, env, vsr, vsr)
DEF_HELPER_4(xxperm, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xxpermr, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xxextractuw, void, env, vsr, vsr, i32)
DEF_HELPER_4(xxinsertw, void, env, vsr, vsr, i32)
DEF_HELPER_3(xvxsigsp, void, env, vsr, vsr)

DEF_HELPER_2(efscfsi, i32, env, i32)
DEF_HELPER_2(efscfui, i32, env, i32)
DEF_HELPER_2(efscfuf, i32, env, i32)
DEF_HELPER_2(efscfsf, i32, env, i32)
DEF_HELPER_2(efsctsi, i32, env, i32)
DEF_HELPER_2(efsctui, i32, env, i32)
DEF_HELPER_2(efsctsiz, i32, env, i32)
DEF_HELPER_2(efsctuiz, i32, env, i32)
DEF_HELPER_2(efsctsf, i32, env, i32)
DEF_HELPER_2(efsctuf, i32, env, i32)
DEF_HELPER_2(evfscfsi, i64, env, i64)
DEF_HELPER_2(evfscfui, i64, env, i64)
DEF_HELPER_2(evfscfuf, i64, env, i64)
DEF_HELPER_2(evfscfsf, i64, env, i64)
DEF_HELPER_2(evfsctsi, i64, env, i64)
DEF_HELPER_2(evfsctui, i64, env, i64)
DEF_HELPER_2(evfsctsiz, i64, env, i64)
DEF_HELPER_2(evfsctuiz, i64, env, i64)
DEF_HELPER_2(evfsctsf, i64, env, i64)
DEF_HELPER_2(evfsctuf, i64, env, i64)
DEF_HELPER_3(efsadd, i32, env, i32, i32)
DEF_HELPER_3(efssub, i32, env, i32, i32)
DEF_HELPER_3(efsmul, i32, env, i32, i32)
DEF_HELPER_3(efsdiv, i32, env, i32, i32)
DEF_HELPER_3(evfsadd, i64, env, i64, i64)
DEF_HELPER_3(evfssub, i64, env, i64, i64)
DEF_HELPER_3(evfsmul, i64, env, i64, i64)
DEF_HELPER_3(evfsdiv, i64, env, i64, i64)
DEF_HELPER_3(efststlt, i32, env, i32, i32)
DEF_HELPER_3(efststgt, i32, env, i32, i32)
DEF_HELPER_3(efststeq, i32, env, i32, i32)
DEF_HELPER_3(efscmplt, i32, env, i32, i32)
DEF_HELPER_3(efscmpgt, i32, env, i32, i32)
DEF_HELPER_3(efscmpeq, i32, env, i32, i32)
DEF_HELPER_3(evfststlt, i32, env, i64, i64)
DEF_HELPER_3(evfststgt, i32, env, i64, i64)
DEF_HELPER_3(evfststeq, i32, env, i64, i64)
DEF_HELPER_3(evfscmplt, i32, env, i64, i64)
DEF_HELPER_3(evfscmpgt, i32, env, i64, i64)
DEF_HELPER_3(evfscmpeq, i32, env, i64, i64)
DEF_HELPER_2(efdcfsi, i64, env, i32)
DEF_HELPER_2(efdcfsid, i64, env, i64)
DEF_HELPER_2(efdcfui, i64, env, i32)
DEF_HELPER_2(efdcfuid, i64, env, i64)
DEF_HELPER_2(efdctsi, i32, env, i64)
DEF_HELPER_2(efdctui, i32, env, i64)
DEF_HELPER_2(efdctsiz, i32, env, i64)
DEF_HELPER_2(efdctsidz, i64, env, i64)
DEF_HELPER_2(efdctuiz, i32, env, i64)
DEF_HELPER_2(efdctuidz, i64, env, i64)
DEF_HELPER_2(efdcfsf, i64, env, i32)
DEF_HELPER_2(efdcfuf, i64, env, i32)
DEF_HELPER_2(efdctsf, i32, env, i64)
DEF_HELPER_2(efdctuf, i32, env, i64)
DEF_HELPER_2(efscfd, i32, env, i64)
DEF_HELPER_2(efdcfs, i64, env, i32)
DEF_HELPER_3(efdadd, i64, env, i64, i64)
DEF_HELPER_3(efdsub, i64, env, i64, i64)
DEF_HELPER_3(efdmul, i64, env, i64, i64)
DEF_HELPER_3(efddiv, i64, env, i64, i64)
DEF_HELPER_3(efdtstlt, i32, env, i64, i64)
DEF_HELPER_3(efdtstgt, i32, env, i64, i64)
DEF_HELPER_3(efdtsteq, i32, env, i64, i64)
DEF_HELPER_3(efdcmplt, i32, env, i64, i64)
DEF_HELPER_3(efdcmpgt, i32, env, i64, i64)
DEF_HELPER_3(efdcmpeq, i32, env, i64, i64)

#if !defined(CONFIG_USER_ONLY)
DEF_HELPER_2(4xx_tlbre_hi, tl, env, tl)
DEF_HELPER_2(4xx_tlbre_lo, tl, env, tl)
DEF_HELPER_3(4xx_tlbwe_hi, void, env, tl, tl)
DEF_HELPER_3(4xx_tlbwe_lo, void, env, tl, tl)
DEF_HELPER_2(4xx_tlbsx, tl, env, tl)
DEF_HELPER_3(440_tlbre, tl, env, i32, tl)
DEF_HELPER_4(440_tlbwe, void, env, i32, tl, tl)
DEF_HELPER_2(440_tlbsx, tl, env, tl)
DEF_HELPER_1(booke206_tlbre, void, env)
DEF_HELPER_1(booke206_tlbwe, void, env)
DEF_HELPER_2(booke206_tlbsx, void, env, tl)
DEF_HELPER_2(booke206_tlbivax, void, env, tl)
DEF_HELPER_2(booke206_tlbilx0, void, env, tl)
DEF_HELPER_2(booke206_tlbilx1, void, env, tl)
DEF_HELPER_2(booke206_tlbilx3, void, env, tl)
DEF_HELPER_2(booke206_tlbflush, void, env, tl)
DEF_HELPER_3(booke_setpid, void, env, i32, tl)
DEF_HELPER_2(booke_set_eplc, void, env, tl)
DEF_HELPER_2(booke_set_epsc, void, env, tl)
DEF_HELPER_2(6xx_tlbd, void, env, tl)
DEF_HELPER_2(6xx_tlbi, void, env, tl)
DEF_HELPER_2(74xx_tlbd, void, env, tl)
DEF_HELPER_2(74xx_tlbi, void, env, tl)
DEF_HELPER_FLAGS_1(tlbia, TCG_CALL_NO_RWG, void, env)
DEF_HELPER_FLAGS_2(tlbie, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(tlbiva, TCG_CALL_NO_RWG, void, env, tl)
#if defined(TARGET_PPC64)
DEF_HELPER_FLAGS_3(store_slb, TCG_CALL_NO_RWG, void, env, tl, tl)
DEF_HELPER_2(load_slb_esid, tl, env, tl)
DEF_HELPER_2(load_slb_vsid, tl, env, tl)
DEF_HELPER_2(find_slb_vsid, tl, env, tl)
DEF_HELPER_FLAGS_1(slbia, TCG_CALL_NO_RWG, void, env)
DEF_HELPER_FLAGS_2(slbie, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(slbieg, TCG_CALL_NO_RWG, void, env, tl)
#endif
DEF_HELPER_FLAGS_2(load_sr, TCG_CALL_NO_RWG, tl, env, tl)
DEF_HELPER_FLAGS_3(store_sr, TCG_CALL_NO_RWG, void, env, tl, tl)

DEF_HELPER_FLAGS_1(602_mfrom, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_1(msgsnd, void, tl)
DEF_HELPER_2(msgclr, void, env, tl)
DEF_HELPER_1(book3s_msgsnd, void, tl)
DEF_HELPER_2(book3s_msgclr, void, env, tl)
#endif

DEF_HELPER_4(dlmzb, tl, env, tl, tl, i32)
DEF_HELPER_FLAGS_2(clcs, TCG_CALL_NO_RWG_SE, tl, env, i32)
#if !defined(CONFIG_USER_ONLY)
DEF_HELPER_2(rac, tl, env, tl)
#endif
DEF_HELPER_3(div, tl, env, tl, tl)
DEF_HELPER_3(divo, tl, env, tl, tl)
DEF_HELPER_3(divs, tl, env, tl, tl)
DEF_HELPER_3(divso, tl, env, tl, tl)

DEF_HELPER_2(load_dcr, tl, env, tl)
DEF_HELPER_3(store_dcr, void, env, tl, tl)

DEF_HELPER_2(load_dump_spr, void, env, i32)
DEF_HELPER_2(store_dump_spr, void, env, i32)
DEF_HELPER_4(fscr_facility_check, void, env, i32, i32, i32)
DEF_HELPER_4(msr_facility_check, void, env, i32, i32, i32)
DEF_HELPER_FLAGS_1(load_tbl, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_1(load_tbu, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_1(load_atbl, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_1(load_atbu, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_1(load_vtb, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_1(load_601_rtcl, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_1(load_601_rtcu, TCG_CALL_NO_RWG, tl, env)
#if !defined(CONFIG_USER_ONLY)
#if defined(TARGET_PPC64)
DEF_HELPER_FLAGS_1(load_purr, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_2(store_ptcr, void, env, tl)
#endif
DEF_HELPER_2(store_sdr1, void, env, tl)
DEF_HELPER_2(store_pidr, void, env, tl)
DEF_HELPER_2(store_lpidr, void, env, tl)
DEF_HELPER_FLAGS_2(store_tbl, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_tbu, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_atbl, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_atbu, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_601_rtcl, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_601_rtcu, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_1(load_decr, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_2(store_decr, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_1(load_hdecr, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_2(store_hdecr, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_vtb, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_2(store_hid0_601, void, env, tl)
DEF_HELPER_3(store_403_pbr, void, env, i32, tl)
DEF_HELPER_FLAGS_1(load_40x_pit, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_2(store_40x_pit, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_2(store_40x_dbcr0, void, env, tl)
DEF_HELPER_2(store_40x_sler, void, env, tl)
DEF_HELPER_FLAGS_2(store_booke_tcr, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_booke_tsr, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_3(store_ibatl, void, env, i32, tl)
DEF_HELPER_3(store_ibatu, void, env, i32, tl)
DEF_HELPER_3(store_dbatl, void, env, i32, tl)
DEF_HELPER_3(store_dbatu, void, env, i32, tl)
DEF_HELPER_3(store_601_batl, void, env, i32, tl)
DEF_HELPER_3(store_601_batu, void, env, i32, tl)
#endif

#define dh_alias_fprp ptr
#define dh_ctype_fprp ppc_fprp_t *
#define dh_is_signed_fprp dh_is_signed_ptr

DEF_HELPER_4(dadd, void, env, fprp, fprp, fprp)
DEF_HELPER_4(daddq, void, env, fprp, fprp, fprp)
DEF_HELPER_4(dsub, void, env, fprp, fprp, fprp)
DEF_HELPER_4(dsubq, void, env, fprp, fprp, fprp)
DEF_HELPER_4(dmul, void, env, fprp, fprp, fprp)
DEF_HELPER_4(dmulq, void, env, fprp, fprp, fprp)
DEF_HELPER_4(ddiv, void, env, fprp, fprp, fprp)
DEF_HELPER_4(ddivq, void, env, fprp, fprp, fprp)
DEF_HELPER_3(dcmpo, i32, env, fprp, fprp)
DEF_HELPER_3(dcmpoq, i32, env, fprp, fprp)
DEF_HELPER_3(dcmpu, i32, env, fprp, fprp)
DEF_HELPER_3(dcmpuq, i32, env, fprp, fprp)
DEF_HELPER_3(dtstdc, i32, env, fprp, i32)
DEF_HELPER_3(dtstdcq, i32, env, fprp, i32)
DEF_HELPER_3(dtstdg, i32, env, fprp, i32)
DEF_HELPER_3(dtstdgq, i32, env, fprp, i32)
DEF_HELPER_3(dtstex, i32, env, fprp, fprp)
DEF_HELPER_3(dtstexq, i32, env, fprp, fprp)
DEF_HELPER_3(dtstsf, i32, env, fprp, fprp)
DEF_HELPER_3(dtstsfq, i32, env, fprp, fprp)
DEF_HELPER_3(dtstsfi, i32, env, i32, fprp)
DEF_HELPER_3(dtstsfiq, i32, env, i32, fprp)
DEF_HELPER_5(dquai, void, env, fprp, fprp, i32, i32)
DEF_HELPER_5(dquaiq, void, env, fprp, fprp, i32, i32)
DEF_HELPER_5(dqua, void, env, fprp, fprp, fprp, i32)
DEF_HELPER_5(dquaq, void, env, fprp, fprp, fprp, i32)
DEF_HELPER_5(drrnd, void, env, fprp, fprp, fprp, i32)
DEF_HELPER_5(drrndq, void, env, fprp, fprp, fprp, i32)
DEF_HELPER_5(drintx, void, env, fprp, fprp, i32, i32)
DEF_HELPER_5(drintxq, void, env, fprp, fprp, i32, i32)
DEF_HELPER_5(drintn, void, env, fprp, fprp, i32, i32)
DEF_HELPER_5(drintnq, void, env, fprp, fprp, i32, i32)
DEF_HELPER_3(dctdp, void, env, fprp, fprp)
DEF_HELPER_3(dctqpq, void, env, fprp, fprp)
DEF_HELPER_3(drsp, void, env, fprp, fprp)
DEF_HELPER_3(drdpq, void, env, fprp, fprp)
DEF_HELPER_3(dcffix, void, env, fprp, fprp)
DEF_HELPER_3(dcffixq, void, env, fprp, fprp)
DEF_HELPER_3(dctfix, void, env, fprp, fprp)
DEF_HELPER_3(dctfixq, void, env, fprp, fprp)
DEF_HELPER_4(ddedpd, void, env, fprp, fprp, i32)
DEF_HELPER_4(ddedpdq, void, env, fprp, fprp, i32)
DEF_HELPER_4(denbcd, void, env, fprp, fprp, i32)
DEF_HELPER_4(denbcdq, void, env, fprp, fprp, i32)
DEF_HELPER_3(dxex, void, env, fprp, fprp)
DEF_HELPER_3(dxexq, void, env, fprp, fprp)
DEF_HELPER_4(diex, void, env, fprp, fprp, fprp)
DEF_HELPER_4(diexq, void, env, fprp, fprp, fprp)
DEF_HELPER_4(dscri, void, env, fprp, fprp, i32)
DEF_HELPER_4(dscriq, void, env, fprp, fprp, i32)
DEF_HELPER_4(dscli, void, env, fprp, fprp, i32)
DEF_HELPER_4(dscliq, void, env, fprp, fprp, i32)

DEF_HELPER_1(tbegin, void, env)
DEF_HELPER_FLAGS_1(fixup_thrm, TCG_CALL_NO_RWG, void, env)

#ifdef TARGET_PPC64
DEF_HELPER_FLAGS_3(lq_le_parallel, TCG_CALL_NO_WG, i64, env, tl, i32)
DEF_HELPER_FLAGS_3(lq_be_parallel, TCG_CALL_NO_WG, i64, env, tl, i32)
DEF_HELPER_FLAGS_5(stq_le_parallel, TCG_CALL_NO_WG,
                   void, env, tl, i64, i64, i32)
DEF_HELPER_FLAGS_5(stq_be_parallel, TCG_CALL_NO_WG,
                   void, env, tl, i64, i64, i32)
DEF_HELPER_5(stqcx_le_parallel, i32, env, tl, i64, i64, i32)
DEF_HELPER_5(stqcx_be_parallel, i32, env, tl, i64, i64, i32)
#endif
