#ifndef TARGET_SPARC64
DEF_HELPER_1(rett, void, env)
DEF_HELPER_2(wrpsr, void, env, tl)
DEF_HELPER_1(rdpsr, tl, env)
DEF_HELPER_1(power_down, void, env)
#else
DEF_HELPER_FLAGS_2(wrpil, TCG_CALL_NO_RWG, void, env, tl)
DEF_HELPER_2(wrgl, void, env, tl)
DEF_HELPER_2(wrpstate, void, env, tl)
DEF_HELPER_1(done, void, env)
DEF_HELPER_1(retry, void, env)
DEF_HELPER_FLAGS_1(flushw, TCG_CALL_NO_WG, void, env)
DEF_HELPER_FLAGS_1(saved, TCG_CALL_NO_RWG, void, env)
DEF_HELPER_FLAGS_1(restored, TCG_CALL_NO_RWG, void, env)
DEF_HELPER_1(rdccr, tl, env)
DEF_HELPER_2(wrccr, void, env, tl)
DEF_HELPER_1(rdcwp, tl, env)
DEF_HELPER_2(wrcwp, void, env, tl)
DEF_HELPER_FLAGS_2(array8, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(set_softint, TCG_CALL_NO_RWG, void, env, i64)
DEF_HELPER_FLAGS_2(clear_softint, TCG_CALL_NO_RWG, void, env, i64)
DEF_HELPER_FLAGS_2(write_softint, TCG_CALL_NO_RWG, void, env, i64)
DEF_HELPER_FLAGS_2(tick_set_count, TCG_CALL_NO_RWG, void, ptr, i64)
DEF_HELPER_FLAGS_3(tick_get_count, TCG_CALL_NO_WG, i64, env, ptr, int)
DEF_HELPER_FLAGS_2(tick_set_limit, TCG_CALL_NO_RWG, void, ptr, i64)
#endif
DEF_HELPER_1(debug, void, env)
DEF_HELPER_1(save, void, env)
DEF_HELPER_1(restore, void, env)
DEF_HELPER_FLAGS_3(udiv, TCG_CALL_NO_WG, i64, env, tl, tl)
DEF_HELPER_FLAGS_3(sdiv, TCG_CALL_NO_WG, i64, env, tl, tl)
DEF_HELPER_3(taddcctv, tl, env, tl, tl)
DEF_HELPER_3(tsubcctv, tl, env, tl, tl)
#if !defined(CONFIG_USER_ONLY) || defined(TARGET_SPARC64)
DEF_HELPER_FLAGS_4(ld_asi, TCG_CALL_NO_WG, i64, env, tl, int, i32)
DEF_HELPER_FLAGS_5(st_asi, TCG_CALL_NO_WG, void, env, tl, i64, int, i32)
#endif
DEF_HELPER_FLAGS_1(get_fsr, TCG_CALL_NO_WG_SE, tl, env)
DEF_HELPER_FLAGS_2(set_fsr_noftt, 0, void, env, tl)
DEF_HELPER_FLAGS_2(fsqrts, 0, f32, env, f32)
DEF_HELPER_FLAGS_2(fsqrtd, 0, f64, env, f64)
DEF_HELPER_FLAGS_2(fsqrtq, 0, i128, env, i128)
DEF_HELPER_FLAGS_3(fcmps, 0, tl, env, f32, f32)
DEF_HELPER_FLAGS_3(fcmpd, 0, tl, env, f64, f64)
DEF_HELPER_FLAGS_3(fcmpes, 0, tl, env, f32, f32)
DEF_HELPER_FLAGS_3(fcmped, 0, tl, env, f64, f64)
DEF_HELPER_FLAGS_3(fcmpq, 0, tl, env, i128, i128)
DEF_HELPER_FLAGS_3(fcmpeq, 0, tl, env, i128, i128)
#ifdef TARGET_SPARC64
DEF_HELPER_FLAGS_3(fcmps_fcc1, 0, tl, env, f32, f32)
DEF_HELPER_FLAGS_3(fcmps_fcc2, 0, tl, env, f32, f32)
DEF_HELPER_FLAGS_3(fcmps_fcc3, 0, tl, env, f32, f32)
DEF_HELPER_FLAGS_3(fcmpd_fcc1, 0, tl, env, f64, f64)
DEF_HELPER_FLAGS_3(fcmpd_fcc2, 0, tl, env, f64, f64)
DEF_HELPER_FLAGS_3(fcmpd_fcc3, 0, tl, env, f64, f64)
DEF_HELPER_FLAGS_3(fcmpes_fcc1, 0, tl, env, f32, f32)
DEF_HELPER_FLAGS_3(fcmpes_fcc2, 0, tl, env, f32, f32)
DEF_HELPER_FLAGS_3(fcmpes_fcc3, 0, tl, env, f32, f32)
DEF_HELPER_FLAGS_3(fcmped_fcc1, 0, tl, env, f64, f64)
DEF_HELPER_FLAGS_3(fcmped_fcc2, 0, tl, env, f64, f64)
DEF_HELPER_FLAGS_3(fcmped_fcc3, 0, tl, env, f64, f64)
DEF_HELPER_FLAGS_3(fcmpq_fcc1, 0, tl, env, i128, i128)
DEF_HELPER_FLAGS_3(fcmpq_fcc2, 0, tl, env, i128, i128)
DEF_HELPER_FLAGS_3(fcmpq_fcc3, 0, tl, env, i128, i128)
DEF_HELPER_FLAGS_3(fcmpeq_fcc1, 0, tl, env, i128, i128)
DEF_HELPER_FLAGS_3(fcmpeq_fcc2, 0, tl, env, i128, i128)
DEF_HELPER_FLAGS_3(fcmpeq_fcc3, 0, tl, env, i128, i128)
#endif
DEF_HELPER_2(raise_exception, noreturn, env, int)

DEF_HELPER_FLAGS_3(faddd, 0, f64, env, f64, f64)
DEF_HELPER_FLAGS_3(fsubd, 0, f64, env, f64, f64)
DEF_HELPER_FLAGS_3(fmuld, 0, f64, env, f64, f64)
DEF_HELPER_FLAGS_3(fdivd, 0, f64, env, f64, f64)

DEF_HELPER_FLAGS_3(faddq, 0, i128, env, i128, i128)
DEF_HELPER_FLAGS_3(fsubq, 0, i128, env, i128, i128)
DEF_HELPER_FLAGS_3(fmulq, 0, i128, env, i128, i128)
DEF_HELPER_FLAGS_3(fdivq, 0, i128, env, i128, i128)

DEF_HELPER_FLAGS_3(fadds, 0, f32, env, f32, f32)
DEF_HELPER_FLAGS_3(fsubs, 0, f32, env, f32, f32)
DEF_HELPER_FLAGS_3(fmuls, 0, f32, env, f32, f32)
DEF_HELPER_FLAGS_3(fdivs, 0, f32, env, f32, f32)

DEF_HELPER_FLAGS_3(fsmuld, 0, f64, env, f32, f32)
DEF_HELPER_FLAGS_3(fdmulq, 0, i128, env, f64, f64)

DEF_HELPER_FLAGS_2(fitod, 0, f64, env, s32)
DEF_HELPER_FLAGS_2(fitoq, 0, i128, env, s32)

DEF_HELPER_FLAGS_2(fitos, 0, f32, env, s32)

#ifdef TARGET_SPARC64
DEF_HELPER_FLAGS_2(fxtos, 0, f32, env, s64)
DEF_HELPER_FLAGS_2(fxtod, 0, f64, env, s64)
DEF_HELPER_FLAGS_2(fxtoq, 0, i128, env, s64)
#endif
DEF_HELPER_FLAGS_2(fdtos, 0, f32, env, f64)
DEF_HELPER_FLAGS_2(fstod, 0, f64, env, f32)
DEF_HELPER_FLAGS_2(fqtos, 0, f32, env, i128)
DEF_HELPER_FLAGS_2(fstoq, 0, i128, env, f32)
DEF_HELPER_FLAGS_2(fqtod, 0, f64, env, i128)
DEF_HELPER_FLAGS_2(fdtoq, 0, i128, env, f64)
DEF_HELPER_FLAGS_2(fstoi, 0, s32, env, f32)
DEF_HELPER_FLAGS_2(fdtoi, 0, s32, env, f64)
DEF_HELPER_FLAGS_2(fqtoi, 0, s32, env, i128)
#ifdef TARGET_SPARC64
DEF_HELPER_FLAGS_2(fstox, 0, s64, env, f32)
DEF_HELPER_FLAGS_2(fdtox, 0, s64, env, f64)
DEF_HELPER_FLAGS_2(fqtox, 0, s64, env, i128)

DEF_HELPER_FLAGS_2(fpmerge, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(fmul8x16, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(fmul8x16al, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(fmul8x16au, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(fmul8sux16, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(fmul8ulx16, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(fmuld8sux16, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(fmuld8ulx16, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(fexpand, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_3(pdist, TCG_CALL_NO_RWG_SE, i64, i64, i64, i64)
DEF_HELPER_FLAGS_2(fpack16, TCG_CALL_NO_RWG_SE, i32, i64, i64)
DEF_HELPER_FLAGS_3(fpack32, TCG_CALL_NO_RWG_SE, i64, i64, i64, i64)
DEF_HELPER_FLAGS_2(fpackfix, TCG_CALL_NO_RWG_SE, i32, i64, i64)
DEF_HELPER_FLAGS_3(bshuffle, TCG_CALL_NO_RWG_SE, i64, i64, i64, i64)
#define VIS_CMPHELPER(name)                                              \
    DEF_HELPER_FLAGS_2(f##name##16, TCG_CALL_NO_RWG_SE,      \
                       i64, i64, i64)                                    \
    DEF_HELPER_FLAGS_2(f##name##32, TCG_CALL_NO_RWG_SE,      \
                       i64, i64, i64)
VIS_CMPHELPER(cmpgt)
VIS_CMPHELPER(cmpeq)
VIS_CMPHELPER(cmple)
VIS_CMPHELPER(cmpne)
#endif
#undef VIS_HELPER
#undef VIS_CMPHELPER
