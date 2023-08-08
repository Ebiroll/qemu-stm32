DEF_HELPER_FLAGS_3(raise_exception_err, TCG_CALL_NO_WG, noreturn, env, i32, i32)
DEF_HELPER_FLAGS_2(raise_exception, TCG_CALL_NO_WG, noreturn, env, i32)
DEF_HELPER_FLAGS_4(tw, TCG_CALL_NO_WG, void, env, tl, tl, i32)
#if defined(TARGET_PPC64)
DEF_HELPER_FLAGS_4(td, TCG_CALL_NO_WG, void, env, tl, tl, i32)
#endif
DEF_HELPER_4(HASHST, void, env, tl, tl, tl)
DEF_HELPER_4(HASHCHK, void, env, tl, tl, tl)
DEF_HELPER_4(HASHSTP, void, env, tl, tl, tl)
DEF_HELPER_4(HASHCHKP, void, env, tl, tl, tl)
#if !defined(CONFIG_USER_ONLY)
DEF_HELPER_2(store_msr, void, env, tl)
DEF_HELPER_1(ppc_maybe_interrupt, void, env)
DEF_HELPER_1(rfi, void, env)
DEF_HELPER_1(40x_rfci, void, env)
DEF_HELPER_1(rfci, void, env)
DEF_HELPER_1(rfdi, void, env)
DEF_HELPER_1(rfmci, void, env)
#if defined(TARGET_PPC64)
DEF_HELPER_2(scv, noreturn, env, i32)
DEF_HELPER_2(pminsn, void, env, i32)
DEF_HELPER_1(rfid, void, env)
DEF_HELPER_1(rfscv, void, env)
DEF_HELPER_1(hrfid, void, env)
DEF_HELPER_2(rfebb, void, env, tl)
DEF_HELPER_2(store_lpcr, void, env, tl)
DEF_HELPER_2(store_pcr, void, env, tl)
DEF_HELPER_2(store_mmcr0, void, env, tl)
DEF_HELPER_2(store_mmcr1, void, env, tl)
DEF_HELPER_3(store_pmc, void, env, i32, i64)
DEF_HELPER_2(read_pmc, tl, env, i32)
DEF_HELPER_2(insns_inc, void, env, i32)
DEF_HELPER_1(handle_pmc5_overflow, void, env)
#endif
DEF_HELPER_2(book3s_trace, void, env, tl)
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
DEF_HELPER_FLAGS_2(CFUGED, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(PDEPD, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(PEXTD, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_1(CDTBCD, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(CBCDTD, TCG_CALL_NO_RWG_SE, tl, tl)
#if defined(TARGET_PPC64)
DEF_HELPER_FLAGS_2(cmpeqb, TCG_CALL_NO_RWG_SE, i32, tl, tl)
DEF_HELPER_FLAGS_1(popcntw, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_2(bpermd, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_3(srad, tl, env, tl, tl)
DEF_HELPER_FLAGS_0(darn32, TCG_CALL_NO_RWG, tl)
DEF_HELPER_FLAGS_0(darn64, TCG_CALL_NO_RWG, tl)
#endif

DEF_HELPER_FLAGS_1(cntlsw32, TCG_CALL_NO_RWG_SE, i32, i32)
DEF_HELPER_FLAGS_1(cntlzw32, TCG_CALL_NO_RWG_SE, i32, i32)
DEF_HELPER_FLAGS_2(brinc, TCG_CALL_NO_RWG_SE, tl, tl, tl)

DEF_HELPER_1(float_check_status, void, env)
DEF_HELPER_1(fpscr_check_status, void, env)
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
DEF_HELPER_3(fadds, f64, env, f64, f64)
DEF_HELPER_3(fsub, f64, env, f64, f64)
DEF_HELPER_3(fsubs, f64, env, f64, f64)
DEF_HELPER_3(fmul, f64, env, f64, f64)
DEF_HELPER_3(fmuls, f64, env, f64, f64)
DEF_HELPER_3(fdiv, f64, env, f64, f64)
DEF_HELPER_3(fdivs, f64, env, f64, f64)
DEF_HELPER_4(fmadd, i64, env, i64, i64, i64)
DEF_HELPER_4(fmsub, i64, env, i64, i64, i64)
DEF_HELPER_4(fnmadd, i64, env, i64, i64, i64)
DEF_HELPER_4(fnmsub, i64, env, i64, i64, i64)
DEF_HELPER_4(fmadds, i64, env, i64, i64, i64)
DEF_HELPER_4(fmsubs, i64, env, i64, i64, i64)
DEF_HELPER_4(fnmadds, i64, env, i64, i64, i64)
DEF_HELPER_4(fnmsubs, i64, env, i64, i64, i64)
DEF_HELPER_2(FSQRT, f64, env, f64)
DEF_HELPER_2(FSQRTS, f64, env, f64)
DEF_HELPER_2(fre, i64, env, i64)
DEF_HELPER_2(fres, i64, env, i64)
DEF_HELPER_2(frsqrte, i64, env, i64)
DEF_HELPER_2(frsqrtes, i64, env, i64)
DEF_HELPER_FLAGS_3(FSEL, TCG_CALL_NO_RWG_SE, i64, i64, i64, i64)

DEF_HELPER_FLAGS_2(ftdiv, TCG_CALL_NO_RWG_SE, i32, i64, i64)
DEF_HELPER_FLAGS_1(ftsqrt, TCG_CALL_NO_RWG_SE, i32, i64)

#define dh_alias_avr ptr
#define dh_ctype_avr ppc_avr_t *
#define dh_typecode_avr dh_typecode_ptr

#define dh_alias_vsr ptr
#define dh_ctype_vsr ppc_vsr_t *
#define dh_typecode_vsr dh_typecode_ptr

#define dh_alias_acc ptr
#define dh_ctype_acc ppc_acc_t *
#define dh_typecode_acc dh_typecode_ptr

DEF_HELPER_FLAGS_4(VAVGUB, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(VAVGUH, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(VAVGUW, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(VABSDUB, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(VABSDUH, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(VABSDUW, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(VAVGSB, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(VAVGSH, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(VAVGSW, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_4(vcmpeqfp, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgefp, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtfp, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpbfp, void, env, avr, avr, avr)
DEF_HELPER_FLAGS_4(VCMPNEZB, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(VCMPNEZH, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(VCMPNEZW, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_4(vcmpeqfp_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgefp_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpgtfp_dot, void, env, avr, avr, avr)
DEF_HELPER_4(vcmpbfp_dot, void, env, avr, avr, avr)
DEF_HELPER_FLAGS_3(vmrglb, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vmrglh, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vmrglw, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vmrghb, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vmrghh, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vmrghw, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMULESB, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMULESH, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMULESW, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMULEUB, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMULEUH, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMULEUW, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMULOSB, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMULOSH, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMULOSW, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMULOUB, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMULOUH, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMULOUW, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VDIVSQ, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VDIVUQ, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VDIVESD, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VDIVEUD, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VDIVESQ, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VDIVEUQ, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMODSQ, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VMODUQ, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vslo, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vsro, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vsrv, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vslv, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VPRTYBQ, TCG_CALL_NO_RWG, void, avr, avr, i32)
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
DEF_HELPER_FLAGS_3(VADDUQM, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_4(VADDECUQ, TCG_CALL_NO_RWG, void, avr, avr, avr, avr)
DEF_HELPER_FLAGS_4(VADDEUQM, TCG_CALL_NO_RWG, void, avr, avr, avr, avr)
DEF_HELPER_FLAGS_3(VADDCUQ, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VSUBUQM, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_4(VSUBECUQ, TCG_CALL_NO_RWG, void, avr, avr, avr, avr)
DEF_HELPER_FLAGS_4(VSUBEUQM, TCG_CALL_NO_RWG, void, avr, avr, avr, avr)
DEF_HELPER_FLAGS_3(VSUBCUQ, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_4(vsldoi, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_3(vextractub, TCG_CALL_NO_RWG, void, avr, avr, i32)
DEF_HELPER_FLAGS_3(vextractuh, TCG_CALL_NO_RWG, void, avr, avr, i32)
DEF_HELPER_FLAGS_3(vextractuw, TCG_CALL_NO_RWG, void, avr, avr, i32)
DEF_HELPER_FLAGS_3(vextractd, TCG_CALL_NO_RWG, void, avr, avr, i32)
DEF_HELPER_4(VINSBLX, void, env, avr, i64, tl)
DEF_HELPER_4(VINSHLX, void, env, avr, i64, tl)
DEF_HELPER_4(VINSWLX, void, env, avr, i64, tl)
DEF_HELPER_4(VINSDLX, void, env, avr, i64, tl)
DEF_HELPER_FLAGS_2(VSTRIBL, TCG_CALL_NO_RWG, i32, avr, avr)
DEF_HELPER_FLAGS_2(VSTRIBR, TCG_CALL_NO_RWG, i32, avr, avr)
DEF_HELPER_FLAGS_2(VSTRIHL, TCG_CALL_NO_RWG, i32, avr, avr)
DEF_HELPER_FLAGS_2(VSTRIHR, TCG_CALL_NO_RWG, i32, avr, avr)
DEF_HELPER_FLAGS_2(vupkhpx, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vupklpx, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vupkhsb, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vupkhsh, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vupkhsw, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vupklsb, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vupklsh, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vupklsw, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_4(VMSUMUBM, TCG_CALL_NO_RWG, void, avr, avr, avr, avr)
DEF_HELPER_FLAGS_4(VMSUMMBM, TCG_CALL_NO_RWG, void, avr, avr, avr, avr)
DEF_HELPER_FLAGS_4(VPERM, TCG_CALL_NO_RWG, void, avr, avr, avr, avr)
DEF_HELPER_FLAGS_4(VPERMR, TCG_CALL_NO_RWG, void, avr, avr, avr, avr)
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
DEF_HELPER_FLAGS_3(vpkpx, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_5(VMHADDSHS, void, env, avr, avr, avr, avr)
DEF_HELPER_5(VMHRADDSHS, void, env, avr, avr, avr, avr)
DEF_HELPER_FLAGS_4(VMSUMUHM, TCG_CALL_NO_RWG, void, avr, avr, avr, avr)
DEF_HELPER_5(VMSUMUHS, void, env, avr, avr, avr, avr)
DEF_HELPER_FLAGS_4(VMSUMSHM, TCG_CALL_NO_RWG, void, avr, avr, avr, avr)
DEF_HELPER_5(VMSUMSHS, void, env, avr, avr, avr, avr)
DEF_HELPER_FLAGS_5(VMLADDUHM, TCG_CALL_NO_RWG, void, avr, avr, avr, avr, i32)
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
DEF_HELPER_FLAGS_4(VRLWMI, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(VRLDMI, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(VRLDNM, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(VRLWNM, TCG_CALL_NO_RWG, void, avr, avr, avr, i32)
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

DEF_HELPER_FLAGS_2(vclzb, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vclzh, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vctzb, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vctzh, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vctzw, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vctzd, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vpopcntb, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vpopcnth, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vpopcntw, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_2(vpopcntd, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_1(vclzlsbb, TCG_CALL_NO_RWG, tl, avr)
DEF_HELPER_FLAGS_1(vctzlsbb, TCG_CALL_NO_RWG, tl, avr)
DEF_HELPER_FLAGS_3(vbpermd, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vbpermq, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vpmsumb, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vpmsumh, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vpmsumw, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(VPMSUMD, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_2(vextublx, TCG_CALL_NO_RWG, tl, tl, avr)
DEF_HELPER_FLAGS_2(vextuhlx, TCG_CALL_NO_RWG, tl, tl, avr)
DEF_HELPER_FLAGS_2(vextuwlx, TCG_CALL_NO_RWG, tl, tl, avr)
DEF_HELPER_FLAGS_2(vextubrx, TCG_CALL_NO_RWG, tl, tl, avr)
DEF_HELPER_FLAGS_2(vextuhrx, TCG_CALL_NO_RWG, tl, tl, avr)
DEF_HELPER_FLAGS_2(vextuwrx, TCG_CALL_NO_RWG, tl, tl, avr)
DEF_HELPER_5(VEXTDUBVLX, void, env, avr, avr, avr, tl)
DEF_HELPER_5(VEXTDUHVLX, void, env, avr, avr, avr, tl)
DEF_HELPER_5(VEXTDUWVLX, void, env, avr, avr, avr, tl)
DEF_HELPER_5(VEXTDDVLX, void, env, avr, avr, avr, tl)

DEF_HELPER_FLAGS_2(vsbox, TCG_CALL_NO_RWG, void, avr, avr)
DEF_HELPER_FLAGS_3(vcipher, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vcipherlast, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vncipher, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vncipherlast, TCG_CALL_NO_RWG, void, avr, avr, avr)
DEF_HELPER_FLAGS_3(vshasigmaw, TCG_CALL_NO_RWG, void, avr, avr, i32)
DEF_HELPER_FLAGS_3(vshasigmad, TCG_CALL_NO_RWG, void, avr, avr, i32)
DEF_HELPER_FLAGS_4(vpermxor, TCG_CALL_NO_RWG, void, avr, avr, avr, avr)

DEF_HELPER_FLAGS_4(bcdadd, TCG_CALL_NO_RWG, i32, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(bcdsub, TCG_CALL_NO_RWG, i32, avr, avr, avr, i32)
DEF_HELPER_FLAGS_3(bcdcfn, TCG_CALL_NO_RWG, i32, avr, avr, i32)
DEF_HELPER_FLAGS_3(bcdctn, TCG_CALL_NO_RWG, i32, avr, avr, i32)
DEF_HELPER_FLAGS_3(bcdcfz, TCG_CALL_NO_RWG, i32, avr, avr, i32)
DEF_HELPER_FLAGS_3(bcdctz, TCG_CALL_NO_RWG, i32, avr, avr, i32)
DEF_HELPER_FLAGS_3(bcdcfsq, TCG_CALL_NO_RWG, i32, avr, avr, i32)
DEF_HELPER_FLAGS_3(bcdctsq, TCG_CALL_NO_RWG, i32, avr, avr, i32)
DEF_HELPER_FLAGS_4(bcdcpsgn, TCG_CALL_NO_RWG, i32, avr, avr, avr, i32)
DEF_HELPER_FLAGS_3(bcdsetsgn, TCG_CALL_NO_RWG, i32, avr, avr, i32)
DEF_HELPER_FLAGS_4(bcds, TCG_CALL_NO_RWG, i32, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(bcdus, TCG_CALL_NO_RWG, i32, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(bcdsr, TCG_CALL_NO_RWG, i32, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(bcdtrunc, TCG_CALL_NO_RWG, i32, avr, avr, avr, i32)
DEF_HELPER_FLAGS_4(bcdutrunc, TCG_CALL_NO_RWG, i32, avr, avr, avr, i32)

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
DEF_HELPER_5(XSMADDDP, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(XSMSUBDP, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(XSNMADDDP, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(XSNMSUBDP, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_4(XSCMPEQDP, void, env, vsr, vsr, vsr)
DEF_HELPER_4(XSCMPGTDP, void, env, vsr, vsr, vsr)
DEF_HELPER_4(XSCMPGEDP, void, env, vsr, vsr, vsr)
DEF_HELPER_4(XSCMPEQQP, void, env, vsr, vsr, vsr)
DEF_HELPER_4(XSCMPGTQP, void, env, vsr, vsr, vsr)
DEF_HELPER_4(XSCMPGEQP, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xscmpexpdp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscmpexpqp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscmpodp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscmpudp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscmpoqp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscmpuqp, void, env, i32, vsr, vsr)
DEF_HELPER_4(xsmaxdp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(xsmindp, void, env, vsr, vsr, vsr)
DEF_HELPER_4(XSMAXCDP, void, env, vsr, vsr, vsr)
DEF_HELPER_4(XSMINCDP, void, env, vsr, vsr, vsr)
DEF_HELPER_4(XSMAXJDP, void, env, vsr, vsr, vsr)
DEF_HELPER_4(XSMINJDP, void, env, vsr, vsr, vsr)
DEF_HELPER_4(XSMAXCQP, void, env, vsr, vsr, vsr)
DEF_HELPER_4(XSMINCQP, void, env, vsr, vsr, vsr)
DEF_HELPER_3(xscvdphp, void, env, vsr, vsr)
DEF_HELPER_4(xscvdpqp, void, env, i32, vsr, vsr)
DEF_HELPER_3(xscvdpsp, void, env, vsr, vsr)
DEF_HELPER_2(xscvdpspn, i64, env, i64)
DEF_HELPER_4(XSCVQPDP, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscvqpsdz, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscvqpswz, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscvqpudz, void, env, i32, vsr, vsr)
DEF_HELPER_4(xscvqpuwz, void, env, i32, vsr, vsr)
DEF_HELPER_3(XSCVQPUQZ, void, env, vsr, vsr)
DEF_HELPER_3(XSCVQPSQZ, void, env, vsr, vsr)
DEF_HELPER_3(XSCVUQQP, void, env, vsr, vsr)
DEF_HELPER_3(XSCVSQQP, void, env, vsr, vsr)
DEF_HELPER_3(xscvhpdp, void, env, vsr, vsr)
DEF_HELPER_4(xscvsdqp, void, env, i32, vsr, vsr)
DEF_HELPER_3(xscvspdp, void, env, vsr, vsr)
DEF_HELPER_FLAGS_1(XSCVSPDPN, TCG_CALL_NO_RWG_SE, i64, i64)
DEF_HELPER_3(xscvdpsxds, void, env, vsr, vsr)
DEF_HELPER_3(xscvdpsxws, void, env, vsr, vsr)
DEF_HELPER_3(xscvdpuxds, void, env, vsr, vsr)
DEF_HELPER_3(xscvdpuxws, void, env, vsr, vsr)
DEF_HELPER_3(xscvsxddp, void, env, vsr, vsr)
DEF_HELPER_3(xscvuxdsp, void, env, vsr, vsr)
DEF_HELPER_3(xscvsxdsp, void, env, vsr, vsr)
DEF_HELPER_4(xscvudqp, void, env, i32, vsr, vsr)
DEF_HELPER_3(xscvuxddp, void, env, vsr, vsr)
DEF_HELPER_4(XSTSTDCSP, void, env, i32, i32, vsr)
DEF_HELPER_4(XSTSTDCDP, void, env, i32, i32, vsr)
DEF_HELPER_4(XSTSTDCQP, void, env, i32, i32, vsr)
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
DEF_HELPER_5(XSMADDSP, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(XSMSUBSP, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(XSNMADDSP, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(XSNMSUBSP, void, env, vsr, vsr, vsr, vsr)

DEF_HELPER_5(XSMADDQP, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(XSMADDQPO, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(XSMSUBQP, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(XSMSUBQPO, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(XSNMADDQP, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(XSNMADDQPO, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(XSNMSUBQP, void, env, vsr, vsr, vsr, vsr)
DEF_HELPER_5(XSNMSUBQPO, void, env, vsr, vsr, vsr, vsr)

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
DEF_HELPER_3(XVCVSPBF16, void, env, vsr, vsr)
DEF_HELPER_3(xvcvspsxds, void, env, vsr, vsr)
DEF_HELPER_3(xvcvspsxws, void, env, vsr, vsr)
DEF_HELPER_3(xvcvspuxds, void, env, vsr, vsr)
DEF_HELPER_3(xvcvspuxws, void, env, vsr, vsr)
DEF_HELPER_3(xvcvsxdsp, void, env, vsr, vsr)
DEF_HELPER_3(xvcvuxdsp, void, env, vsr, vsr)
DEF_HELPER_3(xvcvsxwsp, void, env, vsr, vsr)
DEF_HELPER_3(xvcvuxwsp, void, env, vsr, vsr)
DEF_HELPER_FLAGS_4(XVTSTDCSP, TCG_CALL_NO_RWG, void, vsr, vsr, i64, i32)
DEF_HELPER_FLAGS_4(XVTSTDCDP, TCG_CALL_NO_RWG, void, vsr, vsr, i64, i32)
DEF_HELPER_3(xvrspi, void, env, vsr, vsr)
DEF_HELPER_3(xvrspic, void, env, vsr, vsr)
DEF_HELPER_3(xvrspim, void, env, vsr, vsr)
DEF_HELPER_3(xvrspip, void, env, vsr, vsr)
DEF_HELPER_3(xvrspiz, void, env, vsr, vsr)
DEF_HELPER_FLAGS_2(XXGENPCVBM_be_exp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVBM_be_comp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVBM_le_exp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVBM_le_comp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVHM_be_exp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVHM_be_comp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVHM_le_exp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVHM_le_comp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVWM_be_exp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVWM_be_comp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVWM_le_exp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVWM_le_comp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVDM_be_exp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVDM_be_comp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVDM_le_exp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_2(XXGENPCVDM_le_comp, TCG_CALL_NO_RWG, void, vsr, avr)
DEF_HELPER_FLAGS_3(XXEXTRACTUW, TCG_CALL_NO_RWG, void, vsr, vsr, i32)
DEF_HELPER_FLAGS_5(XXPERMX, TCG_CALL_NO_RWG, void, vsr, vsr, vsr, vsr, tl)
DEF_HELPER_FLAGS_3(XXINSERTW, TCG_CALL_NO_RWG, void, vsr, vsr, i32)
DEF_HELPER_FLAGS_2(XVXSIGSP, TCG_CALL_NO_RWG, void, vsr, vsr)
DEF_HELPER_FLAGS_5(XXEVAL, TCG_CALL_NO_RWG, void, vsr, vsr, vsr, vsr, i32)
DEF_HELPER_FLAGS_5(XXBLENDVB, TCG_CALL_NO_RWG, void, vsr, vsr, vsr, vsr, i32)
DEF_HELPER_FLAGS_5(XXBLENDVH, TCG_CALL_NO_RWG, void, vsr, vsr, vsr, vsr, i32)
DEF_HELPER_FLAGS_5(XXBLENDVW, TCG_CALL_NO_RWG, void, vsr, vsr, vsr, vsr, i32)
DEF_HELPER_FLAGS_5(XXBLENDVD, TCG_CALL_NO_RWG, void, vsr, vsr, vsr, vsr, i32)
DEF_HELPER_5(XVI4GER8, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVI4GER8PP, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVI8GER4, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVI8GER4PP, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVI8GER4SPP, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVI16GER2, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVI16GER2S, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVI16GER2PP, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVI16GER2SPP, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF16GER2, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF16GER2PP, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF16GER2PN, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF16GER2NP, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF16GER2NN, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVBF16GER2, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVBF16GER2PP, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVBF16GER2PN, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVBF16GER2NP, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVBF16GER2NN, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF32GER, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF32GERPP, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF32GERPN, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF32GERNP, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF32GERNN, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF64GER, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF64GERPP, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF64GERPN, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF64GERNP, void, env, vsr, vsr, acc, i32)
DEF_HELPER_5(XVF64GERNN, void, env, vsr, vsr, acc, i32)

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
DEF_HELPER_FLAGS_1(tlbia, TCG_CALL_NO_RWG, void, env)
DEF_HELPER_FLAGS_2(tlbie, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(tlbiva, TCG_CALL_NO_RWG, void, env, tl)
#if defined(TARGET_PPC64)
DEF_HELPER_FLAGS_4(tlbie_isa300, TCG_CALL_NO_WG, void, \
        env, tl, tl, i32)
DEF_HELPER_FLAGS_3(SLBMTE, TCG_CALL_NO_RWG, void, env, tl, tl)
DEF_HELPER_2(SLBMFEE, tl, env, tl)
DEF_HELPER_2(SLBMFEV, tl, env, tl)
DEF_HELPER_2(SLBFEE, tl, env, tl)
DEF_HELPER_FLAGS_2(SLBIA, TCG_CALL_NO_RWG, void, env, i32)
DEF_HELPER_FLAGS_3(SLBIAG, TCG_CALL_NO_RWG, void, env, tl, i32)
DEF_HELPER_FLAGS_2(SLBIE, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(SLBIEG, TCG_CALL_NO_RWG, void, env, tl)
#endif
DEF_HELPER_FLAGS_2(load_sr, TCG_CALL_NO_RWG, tl, env, tl)
DEF_HELPER_FLAGS_3(store_sr, TCG_CALL_NO_RWG, void, env, tl, tl)

DEF_HELPER_1(msgsnd, void, tl)
DEF_HELPER_2(msgclr, void, env, tl)
DEF_HELPER_1(book3s_msgsnd, void, tl)
DEF_HELPER_2(book3s_msgclr, void, env, tl)
#endif

DEF_HELPER_4(dlmzb, tl, env, tl, tl, i32)
#if !defined(CONFIG_USER_ONLY)
DEF_HELPER_2(rac, tl, env, tl)

DEF_HELPER_2(load_dcr, tl, env, tl)
DEF_HELPER_3(store_dcr, void, env, tl, tl)
#endif

DEF_HELPER_2(load_dump_spr, void, env, i32)
DEF_HELPER_2(store_dump_spr, void, env, i32)
DEF_HELPER_3(spr_core_write_generic, void, env, i32, tl)
DEF_HELPER_3(spr_write_CTRL, void, env, i32, tl)

DEF_HELPER_4(fscr_facility_check, void, env, i32, i32, i32)
DEF_HELPER_4(msr_facility_check, void, env, i32, i32, i32)
DEF_HELPER_FLAGS_1(load_tbl, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_1(load_tbu, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_1(load_atbl, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_1(load_atbu, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_1(load_vtb, TCG_CALL_NO_RWG, tl, env)
#if !defined(CONFIG_USER_ONLY)
#if defined(TARGET_PPC64)
DEF_HELPER_FLAGS_1(load_purr, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_2(store_purr, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_2(store_ptcr, void, env, tl)
DEF_HELPER_FLAGS_1(load_dpdes, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_2(store_dpdes, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_2(book3s_msgsndp, void, env, tl)
DEF_HELPER_2(book3s_msgclrp, void, env, tl)
DEF_HELPER_1(load_tfmr, tl, env)
DEF_HELPER_2(store_tfmr, void, env, tl)
#endif
DEF_HELPER_2(store_sdr1, void, env, tl)
DEF_HELPER_2(store_pidr, void, env, tl)
DEF_HELPER_2(store_lpidr, void, env, tl)
DEF_HELPER_FLAGS_2(store_tbl, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_tbu, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_atbl, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_atbu, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_1(load_decr, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_2(store_decr, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_1(load_hdecr, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_2(store_hdecr, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_vtb, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_tbu40, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_1(load_40x_pit, TCG_CALL_NO_RWG, tl, env)
DEF_HELPER_FLAGS_2(store_40x_pit, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_40x_tcr, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_40x_tsr, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_2(store_40x_pid, void, env, tl)
DEF_HELPER_2(store_40x_dbcr0, void, env, tl)
DEF_HELPER_2(store_40x_sler, void, env, tl)
DEF_HELPER_FLAGS_2(store_booke_tcr, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_FLAGS_2(store_booke_tsr, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_3(store_ibatl, void, env, i32, tl)
DEF_HELPER_3(store_ibatu, void, env, i32, tl)
DEF_HELPER_3(store_dbatl, void, env, i32, tl)
DEF_HELPER_3(store_dbatu, void, env, i32, tl)
#endif

#define dh_alias_fprp ptr
#define dh_ctype_fprp ppc_fprp_t *
#define dh_typecode_fprp dh_typecode_ptr

DEF_HELPER_4(DADD, void, env, fprp, fprp, fprp)
DEF_HELPER_4(DADDQ, void, env, fprp, fprp, fprp)
DEF_HELPER_4(DSUB, void, env, fprp, fprp, fprp)
DEF_HELPER_4(DSUBQ, void, env, fprp, fprp, fprp)
DEF_HELPER_4(DMUL, void, env, fprp, fprp, fprp)
DEF_HELPER_4(DMULQ, void, env, fprp, fprp, fprp)
DEF_HELPER_4(DDIV, void, env, fprp, fprp, fprp)
DEF_HELPER_4(DDIVQ, void, env, fprp, fprp, fprp)
DEF_HELPER_3(DCMPO, i32, env, fprp, fprp)
DEF_HELPER_3(DCMPOQ, i32, env, fprp, fprp)
DEF_HELPER_3(DCMPU, i32, env, fprp, fprp)
DEF_HELPER_3(DCMPUQ, i32, env, fprp, fprp)
DEF_HELPER_3(DTSTDC, i32, env, fprp, i32)
DEF_HELPER_3(DTSTDCQ, i32, env, fprp, i32)
DEF_HELPER_3(DTSTDG, i32, env, fprp, i32)
DEF_HELPER_3(DTSTDGQ, i32, env, fprp, i32)
DEF_HELPER_3(DTSTEX, i32, env, fprp, fprp)
DEF_HELPER_3(DTSTEXQ, i32, env, fprp, fprp)
DEF_HELPER_3(DTSTSF, i32, env, fprp, fprp)
DEF_HELPER_3(DTSTSFQ, i32, env, fprp, fprp)
DEF_HELPER_3(DTSTSFI, i32, env, i32, fprp)
DEF_HELPER_3(DTSTSFIQ, i32, env, i32, fprp)
DEF_HELPER_5(DQUAI, void, env, fprp, fprp, i32, i32)
DEF_HELPER_5(DQUAIQ, void, env, fprp, fprp, i32, i32)
DEF_HELPER_5(DQUA, void, env, fprp, fprp, fprp, i32)
DEF_HELPER_5(DQUAQ, void, env, fprp, fprp, fprp, i32)
DEF_HELPER_5(DRRND, void, env, fprp, fprp, fprp, i32)
DEF_HELPER_5(DRRNDQ, void, env, fprp, fprp, fprp, i32)
DEF_HELPER_5(DRINTX, void, env, fprp, fprp, i32, i32)
DEF_HELPER_5(DRINTXQ, void, env, fprp, fprp, i32, i32)
DEF_HELPER_5(DRINTN, void, env, fprp, fprp, i32, i32)
DEF_HELPER_5(DRINTNQ, void, env, fprp, fprp, i32, i32)
DEF_HELPER_3(DCTDP, void, env, fprp, fprp)
DEF_HELPER_3(DCTQPQ, void, env, fprp, fprp)
DEF_HELPER_3(DRSP, void, env, fprp, fprp)
DEF_HELPER_3(DRDPQ, void, env, fprp, fprp)
DEF_HELPER_3(DCFFIX, void, env, fprp, fprp)
DEF_HELPER_3(DCFFIXQ, void, env, fprp, fprp)
DEF_HELPER_3(DCFFIXQQ, void, env, fprp, avr)
DEF_HELPER_3(DCTFIX, void, env, fprp, fprp)
DEF_HELPER_3(DCTFIXQ, void, env, fprp, fprp)
DEF_HELPER_3(DCTFIXQQ, void, env, avr, fprp)
DEF_HELPER_4(DDEDPD, void, env, fprp, fprp, i32)
DEF_HELPER_4(DDEDPDQ, void, env, fprp, fprp, i32)
DEF_HELPER_4(DENBCD, void, env, fprp, fprp, i32)
DEF_HELPER_4(DENBCDQ, void, env, fprp, fprp, i32)
DEF_HELPER_3(DXEX, void, env, fprp, fprp)
DEF_HELPER_3(DXEXQ, void, env, fprp, fprp)
DEF_HELPER_4(DIEX, void, env, fprp, fprp, fprp)
DEF_HELPER_4(DIEXQ, void, env, fprp, fprp, fprp)
DEF_HELPER_4(DSCRI, void, env, fprp, fprp, i32)
DEF_HELPER_4(DSCRIQ, void, env, fprp, fprp, i32)
DEF_HELPER_4(DSCLI, void, env, fprp, fprp, i32)
DEF_HELPER_4(DSCLIQ, void, env, fprp, fprp, i32)

DEF_HELPER_1(tbegin, void, env)
DEF_HELPER_FLAGS_1(fixup_thrm, TCG_CALL_NO_RWG, void, env)
