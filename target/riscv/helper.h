/* Exceptions */
DEF_HELPER_2(raise_exception, noreturn, env, i32)

/* Floating Point - rounding mode */
DEF_HELPER_FLAGS_2(set_rounding_mode, TCG_CALL_NO_WG, void, env, i32)

/* Floating Point - fused */
DEF_HELPER_FLAGS_4(fmadd_s, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(fmadd_d, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(fmsub_s, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(fmsub_d, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(fnmsub_s, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(fnmsub_d, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(fnmadd_s, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(fnmadd_d, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)

/* Floating Point - Single Precision */
DEF_HELPER_FLAGS_3(fadd_s, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fsub_s, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmul_s, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fdiv_s, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmin_s, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmax_s, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_2(fsqrt_s, TCG_CALL_NO_RWG, i64, env, i64)
DEF_HELPER_FLAGS_3(fle_s, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_3(flt_s, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_3(feq_s, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_2(fcvt_w_s, TCG_CALL_NO_RWG, tl, env, i64)
DEF_HELPER_FLAGS_2(fcvt_wu_s, TCG_CALL_NO_RWG, tl, env, i64)
#if defined(TARGET_RISCV64)
DEF_HELPER_FLAGS_2(fcvt_l_s, TCG_CALL_NO_RWG, tl, env, i64)
DEF_HELPER_FLAGS_2(fcvt_lu_s, TCG_CALL_NO_RWG, tl, env, i64)
#endif
DEF_HELPER_FLAGS_2(fcvt_s_w, TCG_CALL_NO_RWG, i64, env, tl)
DEF_HELPER_FLAGS_2(fcvt_s_wu, TCG_CALL_NO_RWG, i64, env, tl)
#if defined(TARGET_RISCV64)
DEF_HELPER_FLAGS_2(fcvt_s_l, TCG_CALL_NO_RWG, i64, env, tl)
DEF_HELPER_FLAGS_2(fcvt_s_lu, TCG_CALL_NO_RWG, i64, env, tl)
#endif
DEF_HELPER_FLAGS_1(fclass_s, TCG_CALL_NO_RWG_SE, tl, i64)

/* Floating Point - Double Precision */
DEF_HELPER_FLAGS_3(fadd_d, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fsub_d, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmul_d, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fdiv_d, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmin_d, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmax_d, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_2(fcvt_s_d, TCG_CALL_NO_RWG, i64, env, i64)
DEF_HELPER_FLAGS_2(fcvt_d_s, TCG_CALL_NO_RWG, i64, env, i64)
DEF_HELPER_FLAGS_2(fsqrt_d, TCG_CALL_NO_RWG, i64, env, i64)
DEF_HELPER_FLAGS_3(fle_d, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_3(flt_d, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_3(feq_d, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_2(fcvt_w_d, TCG_CALL_NO_RWG, tl, env, i64)
DEF_HELPER_FLAGS_2(fcvt_wu_d, TCG_CALL_NO_RWG, tl, env, i64)
#if defined(TARGET_RISCV64)
DEF_HELPER_FLAGS_2(fcvt_l_d, TCG_CALL_NO_RWG, tl, env, i64)
DEF_HELPER_FLAGS_2(fcvt_lu_d, TCG_CALL_NO_RWG, tl, env, i64)
#endif
DEF_HELPER_FLAGS_2(fcvt_d_w, TCG_CALL_NO_RWG, i64, env, tl)
DEF_HELPER_FLAGS_2(fcvt_d_wu, TCG_CALL_NO_RWG, i64, env, tl)
#if defined(TARGET_RISCV64)
DEF_HELPER_FLAGS_2(fcvt_d_l, TCG_CALL_NO_RWG, i64, env, tl)
DEF_HELPER_FLAGS_2(fcvt_d_lu, TCG_CALL_NO_RWG, i64, env, tl)
#endif
DEF_HELPER_FLAGS_1(fclass_d, TCG_CALL_NO_RWG_SE, tl, i64)

/* Special functions */
DEF_HELPER_3(csrrw, tl, env, tl, tl)
DEF_HELPER_4(csrrs, tl, env, tl, tl, tl)
DEF_HELPER_4(csrrc, tl, env, tl, tl, tl)
#ifndef CONFIG_USER_ONLY
DEF_HELPER_2(sret, tl, env, tl)
DEF_HELPER_2(mret, tl, env, tl)
DEF_HELPER_1(wfi, void, env)
DEF_HELPER_1(tlb_flush, void, env)
#endif

/* Hypervisor functions */
#ifndef CONFIG_USER_ONLY
DEF_HELPER_1(hyp_tlb_flush, void, env)
#endif

/* Vector functions */
DEF_HELPER_3(vsetvl, tl, env, tl, tl)
DEF_HELPER_5(vlb_v_b, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlb_v_b_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlb_v_h, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlb_v_h_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlb_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlb_v_w_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlb_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlb_v_d_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlh_v_h, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlh_v_h_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlh_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlh_v_w_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlh_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlh_v_d_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlw_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlw_v_w_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlw_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlw_v_d_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vle_v_b, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vle_v_b_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vle_v_h, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vle_v_h_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vle_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vle_v_w_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vle_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vle_v_d_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbu_v_b, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbu_v_b_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbu_v_h, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbu_v_h_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbu_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbu_v_w_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbu_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbu_v_d_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlhu_v_h, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlhu_v_h_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlhu_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlhu_v_w_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlhu_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlhu_v_d_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlwu_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlwu_v_w_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlwu_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlwu_v_d_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsb_v_b, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsb_v_b_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsb_v_h, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsb_v_h_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsb_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsb_v_w_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsb_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsb_v_d_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsh_v_h, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsh_v_h_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsh_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsh_v_w_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsh_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsh_v_d_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsw_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsw_v_w_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsw_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vsw_v_d_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vse_v_b, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vse_v_b_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vse_v_h, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vse_v_h_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vse_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vse_v_w_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vse_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vse_v_d_mask, void, ptr, ptr, tl, env, i32)
DEF_HELPER_6(vlsb_v_b, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlsb_v_h, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlsb_v_w, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlsb_v_d, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlsh_v_h, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlsh_v_w, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlsh_v_d, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlsw_v_w, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlsw_v_d, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlse_v_b, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlse_v_h, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlse_v_w, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlse_v_d, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlsbu_v_b, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlsbu_v_h, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlsbu_v_w, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlsbu_v_d, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlshu_v_h, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlshu_v_w, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlshu_v_d, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlswu_v_w, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlswu_v_d, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vssb_v_b, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vssb_v_h, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vssb_v_w, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vssb_v_d, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vssh_v_h, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vssh_v_w, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vssh_v_d, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vssw_v_w, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vssw_v_d, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vsse_v_b, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vsse_v_h, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vsse_v_w, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vsse_v_d, void, ptr, ptr, tl, tl, env, i32)
DEF_HELPER_6(vlxb_v_b, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxb_v_h, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxb_v_w, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxb_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxh_v_h, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxh_v_w, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxh_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxw_v_w, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxw_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxe_v_b, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxe_v_h, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxe_v_w, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxe_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxbu_v_b, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxbu_v_h, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxbu_v_w, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxbu_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxhu_v_h, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxhu_v_w, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxhu_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxwu_v_w, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vlxwu_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vsxb_v_b, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vsxb_v_h, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vsxb_v_w, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vsxb_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vsxh_v_h, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vsxh_v_w, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vsxh_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vsxw_v_w, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vsxw_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vsxe_v_b, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vsxe_v_h, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vsxe_v_w, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vsxe_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_5(vlbff_v_b, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbff_v_h, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbff_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbff_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlhff_v_h, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlhff_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlhff_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlwff_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlwff_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vleff_v_b, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vleff_v_h, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vleff_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vleff_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbuff_v_b, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbuff_v_h, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbuff_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlbuff_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlhuff_v_h, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlhuff_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlhuff_v_d, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlwuff_v_w, void, ptr, ptr, tl, env, i32)
DEF_HELPER_5(vlwuff_v_d, void, ptr, ptr, tl, env, i32)
#ifdef TARGET_RISCV64
DEF_HELPER_6(vamoswapw_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamoswapd_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamoaddw_v_d,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamoaddd_v_d,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamoxorw_v_d,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamoxord_v_d,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamoandw_v_d,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamoandd_v_d,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamoorw_v_d,   void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamoord_v_d,   void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamominw_v_d,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamomind_v_d,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamomaxw_v_d,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamomaxd_v_d,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamominuw_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamominud_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamomaxuw_v_d, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamomaxud_v_d, void, ptr, ptr, tl, ptr, env, i32)
#endif
DEF_HELPER_6(vamoswapw_v_w, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamoaddw_v_w,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamoxorw_v_w,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamoandw_v_w,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamoorw_v_w,   void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamominw_v_w,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamomaxw_v_w,  void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamominuw_v_w, void, ptr, ptr, tl, ptr, env, i32)
DEF_HELPER_6(vamomaxuw_v_w, void, ptr, ptr, tl, ptr, env, i32)
