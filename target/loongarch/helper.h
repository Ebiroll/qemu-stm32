/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) 2021 Loongson Technology Corporation Limited
 */

DEF_HELPER_2(raise_exception, noreturn, env, i32)

DEF_HELPER_FLAGS_1(bitrev_w, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(bitrev_d, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(bitswap, TCG_CALL_NO_RWG_SE, tl, tl)

DEF_HELPER_FLAGS_3(asrtle_d, TCG_CALL_NO_WG, void, env, tl, tl)
DEF_HELPER_FLAGS_3(asrtgt_d, TCG_CALL_NO_WG, void, env, tl, tl)

DEF_HELPER_FLAGS_3(crc32, TCG_CALL_NO_RWG_SE, tl, tl, tl, tl)
DEF_HELPER_FLAGS_3(crc32c, TCG_CALL_NO_RWG_SE, tl, tl, tl, tl)
DEF_HELPER_FLAGS_2(cpucfg, TCG_CALL_NO_RWG_SE, tl, env, tl)

/* Floating-point helper */
DEF_HELPER_FLAGS_3(fadd_s, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fadd_d, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fsub_s, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fsub_d, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmul_s, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmul_d, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fdiv_s, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fdiv_d, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmax_s, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmax_d, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmin_s, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmin_d, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmaxa_s, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmaxa_d, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmina_s, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmina_d, TCG_CALL_NO_WG, i64, env, i64, i64)

DEF_HELPER_FLAGS_5(fmuladd_s, TCG_CALL_NO_WG, i64, env, i64, i64, i64, i32)
DEF_HELPER_FLAGS_5(fmuladd_d, TCG_CALL_NO_WG, i64, env, i64, i64, i64, i32)

DEF_HELPER_FLAGS_3(fscaleb_s, TCG_CALL_NO_WG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fscaleb_d, TCG_CALL_NO_WG, i64, env, i64, i64)

DEF_HELPER_FLAGS_2(flogb_s, TCG_CALL_NO_WG, i64, env, i64)
DEF_HELPER_FLAGS_2(flogb_d, TCG_CALL_NO_WG, i64, env, i64)

DEF_HELPER_FLAGS_2(fsqrt_s, TCG_CALL_NO_WG, i64, env, i64)
DEF_HELPER_FLAGS_2(fsqrt_d, TCG_CALL_NO_WG, i64, env, i64)
DEF_HELPER_FLAGS_2(frsqrt_s, TCG_CALL_NO_WG, i64, env, i64)
DEF_HELPER_FLAGS_2(frsqrt_d, TCG_CALL_NO_WG, i64, env, i64)
DEF_HELPER_FLAGS_2(frecip_s, TCG_CALL_NO_WG, i64, env, i64)
DEF_HELPER_FLAGS_2(frecip_d, TCG_CALL_NO_WG, i64, env, i64)

DEF_HELPER_FLAGS_2(fclass_s, TCG_CALL_NO_RWG_SE, i64, env, i64)
DEF_HELPER_FLAGS_2(fclass_d, TCG_CALL_NO_RWG_SE, i64, env, i64)

/* fcmp.cXXX.s */
DEF_HELPER_4(fcmp_c_s, i64, env, i64, i64, i32)
/* fcmp.sXXX.s */
DEF_HELPER_4(fcmp_s_s, i64, env, i64, i64, i32)
/* fcmp.cXXX.d */
DEF_HELPER_4(fcmp_c_d, i64, env, i64, i64, i32)
/* fcmp.sXXX.d */
DEF_HELPER_4(fcmp_s_d, i64, env, i64, i64, i32)

DEF_HELPER_2(fcvt_d_s, i64, env, i64)
DEF_HELPER_2(fcvt_s_d, i64, env, i64)
DEF_HELPER_2(ffint_d_w, i64, env, i64)
DEF_HELPER_2(ffint_d_l, i64, env, i64)
DEF_HELPER_2(ffint_s_w, i64, env, i64)
DEF_HELPER_2(ffint_s_l, i64, env, i64)
DEF_HELPER_2(ftintrm_l_s, i64, env, i64)
DEF_HELPER_2(ftintrm_l_d, i64, env, i64)
DEF_HELPER_2(ftintrm_w_s, i64, env, i64)
DEF_HELPER_2(ftintrm_w_d, i64, env, i64)
DEF_HELPER_2(ftintrp_l_s, i64, env, i64)
DEF_HELPER_2(ftintrp_l_d, i64, env, i64)
DEF_HELPER_2(ftintrp_w_s, i64, env, i64)
DEF_HELPER_2(ftintrp_w_d, i64, env, i64)
DEF_HELPER_2(ftintrz_l_s, i64, env, i64)
DEF_HELPER_2(ftintrz_l_d, i64, env, i64)
DEF_HELPER_2(ftintrz_w_s, i64, env, i64)
DEF_HELPER_2(ftintrz_w_d, i64, env, i64)
DEF_HELPER_2(ftintrne_l_s, i64, env, i64)
DEF_HELPER_2(ftintrne_l_d, i64, env, i64)
DEF_HELPER_2(ftintrne_w_s, i64, env, i64)
DEF_HELPER_2(ftintrne_w_d, i64, env, i64)
DEF_HELPER_2(ftint_l_s, i64, env, i64)
DEF_HELPER_2(ftint_l_d, i64, env, i64)
DEF_HELPER_2(ftint_w_s, i64, env, i64)
DEF_HELPER_2(ftint_w_d, i64, env, i64)
DEF_HELPER_2(frint_s, i64, env, i64)
DEF_HELPER_2(frint_d, i64, env, i64)

DEF_HELPER_FLAGS_1(set_rounding_mode, TCG_CALL_NO_RWG, void, env)

DEF_HELPER_1(rdtime_d, i64, env)

#ifndef CONFIG_USER_ONLY
/* CSRs helper */
DEF_HELPER_1(csrrd_pgd, i64, env)
DEF_HELPER_1(csrrd_cpuid, i64, env)
DEF_HELPER_1(csrrd_tval, i64, env)
DEF_HELPER_2(csrwr_estat, i64, env, tl)
DEF_HELPER_2(csrwr_asid, i64, env, tl)
DEF_HELPER_2(csrwr_tcfg, i64, env, tl)
DEF_HELPER_2(csrwr_ticlr, i64, env, tl)
DEF_HELPER_2(iocsrrd_b, i64, env, tl)
DEF_HELPER_2(iocsrrd_h, i64, env, tl)
DEF_HELPER_2(iocsrrd_w, i64, env, tl)
DEF_HELPER_2(iocsrrd_d, i64, env, tl)
DEF_HELPER_3(iocsrwr_b, void, env, tl, tl)
DEF_HELPER_3(iocsrwr_h, void, env, tl, tl)
DEF_HELPER_3(iocsrwr_w, void, env, tl, tl)
DEF_HELPER_3(iocsrwr_d, void, env, tl, tl)

/* TLB helper */
DEF_HELPER_1(tlbwr, void, env)
DEF_HELPER_1(tlbfill, void, env)
DEF_HELPER_1(tlbsrch, void, env)
DEF_HELPER_1(tlbrd, void, env)
DEF_HELPER_1(tlbclr, void, env)
DEF_HELPER_1(tlbflush, void, env)
DEF_HELPER_1(invtlb_all, void, env)
DEF_HELPER_2(invtlb_all_g, void, env, i32)
DEF_HELPER_2(invtlb_all_asid, void, env, tl)
DEF_HELPER_3(invtlb_page_asid, void, env, tl, tl)
DEF_HELPER_3(invtlb_page_asid_or_g, void, env, tl, tl)

DEF_HELPER_4(lddir, tl, env, tl, tl, i32)
DEF_HELPER_4(ldpte, void, env, tl, tl, i32)
DEF_HELPER_1(ertn, void, env)
DEF_HELPER_1(idle, void, env)
#endif

/* LoongArch LSX  */
DEF_HELPER_4(vhaddw_h_b, void, env, i32, i32, i32)
DEF_HELPER_4(vhaddw_w_h, void, env, i32, i32, i32)
DEF_HELPER_4(vhaddw_d_w, void, env, i32, i32, i32)
DEF_HELPER_4(vhaddw_q_d, void, env, i32, i32, i32)
DEF_HELPER_4(vhaddw_hu_bu, void, env, i32, i32, i32)
DEF_HELPER_4(vhaddw_wu_hu, void, env, i32, i32, i32)
DEF_HELPER_4(vhaddw_du_wu, void, env, i32, i32, i32)
DEF_HELPER_4(vhaddw_qu_du, void, env, i32, i32, i32)
DEF_HELPER_4(vhsubw_h_b, void, env, i32, i32, i32)
DEF_HELPER_4(vhsubw_w_h, void, env, i32, i32, i32)
DEF_HELPER_4(vhsubw_d_w, void, env, i32, i32, i32)
DEF_HELPER_4(vhsubw_q_d, void, env, i32, i32, i32)
DEF_HELPER_4(vhsubw_hu_bu, void, env, i32, i32, i32)
DEF_HELPER_4(vhsubw_wu_hu, void, env, i32, i32, i32)
DEF_HELPER_4(vhsubw_du_wu, void, env, i32, i32, i32)
DEF_HELPER_4(vhsubw_qu_du, void, env, i32, i32, i32)

DEF_HELPER_FLAGS_4(vaddwev_h_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwev_w_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwev_d_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwev_q_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwod_h_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwod_w_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwod_d_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwod_q_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vsubwev_h_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwev_w_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwev_d_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwev_q_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwod_h_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwod_w_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwod_d_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwod_q_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vaddwev_h_bu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwev_w_hu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwev_d_wu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwev_q_du, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwod_h_bu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwod_w_hu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwod_d_wu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwod_q_du, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vsubwev_h_bu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwev_w_hu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwev_d_wu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwev_q_du, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwod_h_bu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwod_w_hu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwod_d_wu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsubwod_q_du, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vaddwev_h_bu_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwev_w_hu_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwev_d_wu_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwev_q_du_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwod_h_bu_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwod_w_hu_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwod_d_wu_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vaddwod_q_du_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vavg_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavg_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavg_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavg_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavg_bu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavg_hu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavg_wu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavg_du, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vavgr_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavgr_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavgr_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavgr_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavgr_bu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavgr_hu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavgr_wu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vavgr_du, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vabsd_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vabsd_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vabsd_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vabsd_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vabsd_bu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vabsd_hu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vabsd_wu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vabsd_du, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vadda_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vadda_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vadda_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vadda_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vmini_b, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmini_h, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmini_w, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmini_d, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmini_bu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmini_hu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmini_wu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmini_du, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)

DEF_HELPER_FLAGS_4(vmaxi_b, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmaxi_h, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmaxi_w, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmaxi_d, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmaxi_bu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmaxi_hu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmaxi_wu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vmaxi_du, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)

DEF_HELPER_FLAGS_4(vmuh_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmuh_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmuh_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmuh_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmuh_bu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmuh_hu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmuh_wu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmuh_du, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vmulwev_h_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwev_w_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwev_d_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwod_h_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwod_w_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwod_d_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vmulwev_h_bu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwev_w_hu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwev_d_wu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwod_h_bu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwod_w_hu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwod_d_wu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vmulwev_h_bu_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwev_w_hu_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwev_d_wu_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwod_h_bu_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwod_w_hu_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmulwod_d_wu_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vmadd_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmadd_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmadd_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmadd_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmsub_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmsub_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmsub_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmsub_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vmaddwev_h_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwev_w_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwev_d_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwod_h_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwod_w_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwod_d_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vmaddwev_h_bu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwev_w_hu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwev_d_wu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwod_h_bu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwod_w_hu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwod_d_wu, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_FLAGS_4(vmaddwev_h_bu_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwev_w_hu_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwev_d_wu_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwod_h_bu_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwod_w_hu_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vmaddwod_d_wu_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_4(vdiv_b, void, env, i32, i32, i32)
DEF_HELPER_4(vdiv_h, void, env, i32, i32, i32)
DEF_HELPER_4(vdiv_w, void, env, i32, i32, i32)
DEF_HELPER_4(vdiv_d, void, env, i32, i32, i32)
DEF_HELPER_4(vdiv_bu, void, env, i32, i32, i32)
DEF_HELPER_4(vdiv_hu, void, env, i32, i32, i32)
DEF_HELPER_4(vdiv_wu, void, env, i32, i32, i32)
DEF_HELPER_4(vdiv_du, void, env, i32, i32, i32)
DEF_HELPER_4(vmod_b, void, env, i32, i32, i32)
DEF_HELPER_4(vmod_h, void, env, i32, i32, i32)
DEF_HELPER_4(vmod_w, void, env, i32, i32, i32)
DEF_HELPER_4(vmod_d, void, env, i32, i32, i32)
DEF_HELPER_4(vmod_bu, void, env, i32, i32, i32)
DEF_HELPER_4(vmod_hu, void, env, i32, i32, i32)
DEF_HELPER_4(vmod_wu, void, env, i32, i32, i32)
DEF_HELPER_4(vmod_du, void, env, i32, i32, i32)

DEF_HELPER_FLAGS_4(vsat_b, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vsat_h, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vsat_w, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vsat_d, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vsat_bu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vsat_hu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vsat_wu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vsat_du, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)

DEF_HELPER_3(vexth_h_b, void, env, i32, i32)
DEF_HELPER_3(vexth_w_h, void, env, i32, i32)
DEF_HELPER_3(vexth_d_w, void, env, i32, i32)
DEF_HELPER_3(vexth_q_d, void, env, i32, i32)
DEF_HELPER_3(vexth_hu_bu, void, env, i32, i32)
DEF_HELPER_3(vexth_wu_hu, void, env, i32, i32)
DEF_HELPER_3(vexth_du_wu, void, env, i32, i32)
DEF_HELPER_3(vexth_qu_du, void, env, i32, i32)

DEF_HELPER_FLAGS_4(vsigncov_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsigncov_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsigncov_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vsigncov_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)

DEF_HELPER_3(vmskltz_b, void, env, i32, i32)
DEF_HELPER_3(vmskltz_h, void, env, i32, i32)
DEF_HELPER_3(vmskltz_w, void, env, i32, i32)
DEF_HELPER_3(vmskltz_d, void, env, i32, i32)
DEF_HELPER_3(vmskgez_b, void, env, i32, i32)
DEF_HELPER_3(vmsknz_b, void, env, i32,i32)

DEF_HELPER_FLAGS_4(vnori_b, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)

DEF_HELPER_4(vsllwil_h_b, void, env, i32, i32, i32)
DEF_HELPER_4(vsllwil_w_h, void, env, i32, i32, i32)
DEF_HELPER_4(vsllwil_d_w, void, env, i32, i32, i32)
DEF_HELPER_3(vextl_q_d, void, env, i32, i32)
DEF_HELPER_4(vsllwil_hu_bu, void, env, i32, i32, i32)
DEF_HELPER_4(vsllwil_wu_hu, void, env, i32, i32, i32)
DEF_HELPER_4(vsllwil_du_wu, void, env, i32, i32, i32)
DEF_HELPER_3(vextl_qu_du, void, env, i32, i32)

DEF_HELPER_4(vsrlr_b, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlr_h, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlr_w, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlr_d, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlri_b, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlri_h, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlri_w, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlri_d, void, env, i32, i32, i32)

DEF_HELPER_4(vsrar_b, void, env, i32, i32, i32)
DEF_HELPER_4(vsrar_h, void, env, i32, i32, i32)
DEF_HELPER_4(vsrar_w, void, env, i32, i32, i32)
DEF_HELPER_4(vsrar_d, void, env, i32, i32, i32)
DEF_HELPER_4(vsrari_b, void, env, i32, i32, i32)
DEF_HELPER_4(vsrari_h, void, env, i32, i32, i32)
DEF_HELPER_4(vsrari_w, void, env, i32, i32, i32)
DEF_HELPER_4(vsrari_d, void, env, i32, i32, i32)

DEF_HELPER_4(vsrln_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vsrln_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vsrln_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vsran_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vsran_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vsran_w_d, void, env, i32, i32, i32)

DEF_HELPER_4(vsrlni_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlni_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlni_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlni_d_q, void, env, i32, i32, i32)
DEF_HELPER_4(vsrani_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vsrani_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vsrani_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vsrani_d_q, void, env, i32, i32, i32)

DEF_HELPER_4(vsrlrn_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlrn_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlrn_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vsrarn_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vsrarn_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vsrarn_w_d, void, env, i32, i32, i32)

DEF_HELPER_4(vsrlrni_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlrni_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlrni_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vsrlrni_d_q, void, env, i32, i32, i32)
DEF_HELPER_4(vsrarni_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vsrarni_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vsrarni_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vsrarni_d_q, void, env, i32, i32, i32)

DEF_HELPER_4(vssrln_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrln_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrln_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssran_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssran_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssran_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssrln_bu_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrln_hu_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrln_wu_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssran_bu_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssran_hu_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssran_wu_d, void, env, i32, i32, i32)

DEF_HELPER_4(vssrlni_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlni_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlni_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlni_d_q, void, env, i32, i32, i32)
DEF_HELPER_4(vssrani_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrani_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrani_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssrani_d_q, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlni_bu_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlni_hu_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlni_wu_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlni_du_q, void, env, i32, i32, i32)
DEF_HELPER_4(vssrani_bu_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrani_hu_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrani_wu_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssrani_du_q, void, env, i32, i32, i32)

DEF_HELPER_4(vssrlrn_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlrn_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlrn_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarn_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarn_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarn_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlrn_bu_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlrn_hu_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlrn_wu_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarn_bu_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarn_hu_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarn_wu_d, void, env, i32, i32, i32)

DEF_HELPER_4(vssrlrni_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlrni_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlrni_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlrni_d_q, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarni_b_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarni_h_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarni_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarni_d_q, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlrni_bu_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlrni_hu_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlrni_wu_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssrlrni_du_q, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarni_bu_h, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarni_hu_w, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarni_wu_d, void, env, i32, i32, i32)
DEF_HELPER_4(vssrarni_du_q, void, env, i32, i32, i32)

DEF_HELPER_3(vclo_b, void, env, i32, i32)
DEF_HELPER_3(vclo_h, void, env, i32, i32)
DEF_HELPER_3(vclo_w, void, env, i32, i32)
DEF_HELPER_3(vclo_d, void, env, i32, i32)
DEF_HELPER_3(vclz_b, void, env, i32, i32)
DEF_HELPER_3(vclz_h, void, env, i32, i32)
DEF_HELPER_3(vclz_w, void, env, i32, i32)
DEF_HELPER_3(vclz_d, void, env, i32, i32)

DEF_HELPER_3(vpcnt_b, void, env, i32, i32)
DEF_HELPER_3(vpcnt_h, void, env, i32, i32)
DEF_HELPER_3(vpcnt_w, void, env, i32, i32)
DEF_HELPER_3(vpcnt_d, void, env, i32, i32)

DEF_HELPER_FLAGS_4(vbitclr_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vbitclr_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vbitclr_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vbitclr_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vbitclri_b, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vbitclri_h, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vbitclri_w, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vbitclri_d, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)

DEF_HELPER_FLAGS_4(vbitset_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vbitset_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vbitset_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vbitset_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vbitseti_b, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vbitseti_h, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vbitseti_w, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vbitseti_d, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)

DEF_HELPER_FLAGS_4(vbitrev_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vbitrev_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vbitrev_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vbitrev_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(vbitrevi_b, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vbitrevi_h, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vbitrevi_w, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vbitrevi_d, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)

DEF_HELPER_4(vfrstp_b, void, env, i32, i32, i32)
DEF_HELPER_4(vfrstp_h, void, env, i32, i32, i32)
DEF_HELPER_4(vfrstpi_b, void, env, i32, i32, i32)
DEF_HELPER_4(vfrstpi_h, void, env, i32, i32, i32)

DEF_HELPER_4(vfadd_s, void, env, i32, i32, i32)
DEF_HELPER_4(vfadd_d, void, env, i32, i32, i32)
DEF_HELPER_4(vfsub_s, void, env, i32, i32, i32)
DEF_HELPER_4(vfsub_d, void, env, i32, i32, i32)
DEF_HELPER_4(vfmul_s, void, env, i32, i32, i32)
DEF_HELPER_4(vfmul_d, void, env, i32, i32, i32)
DEF_HELPER_4(vfdiv_s, void, env, i32, i32, i32)
DEF_HELPER_4(vfdiv_d, void, env, i32, i32, i32)

DEF_HELPER_5(vfmadd_s, void, env, i32, i32, i32, i32)
DEF_HELPER_5(vfmadd_d, void, env, i32, i32, i32, i32)
DEF_HELPER_5(vfmsub_s, void, env, i32, i32, i32, i32)
DEF_HELPER_5(vfmsub_d, void, env, i32, i32, i32, i32)
DEF_HELPER_5(vfnmadd_s, void, env, i32, i32, i32, i32)
DEF_HELPER_5(vfnmadd_d, void, env, i32, i32, i32, i32)
DEF_HELPER_5(vfnmsub_s, void, env, i32, i32, i32, i32)
DEF_HELPER_5(vfnmsub_d, void, env, i32, i32, i32, i32)

DEF_HELPER_4(vfmax_s, void, env, i32, i32, i32)
DEF_HELPER_4(vfmax_d, void, env, i32, i32, i32)
DEF_HELPER_4(vfmin_s, void, env, i32, i32, i32)
DEF_HELPER_4(vfmin_d, void, env, i32, i32, i32)

DEF_HELPER_4(vfmaxa_s, void, env, i32, i32, i32)
DEF_HELPER_4(vfmaxa_d, void, env, i32, i32, i32)
DEF_HELPER_4(vfmina_s, void, env, i32, i32, i32)
DEF_HELPER_4(vfmina_d, void, env, i32, i32, i32)

DEF_HELPER_3(vflogb_s, void, env, i32, i32)
DEF_HELPER_3(vflogb_d, void, env, i32, i32)

DEF_HELPER_3(vfclass_s, void, env, i32, i32)
DEF_HELPER_3(vfclass_d, void, env, i32, i32)

DEF_HELPER_3(vfsqrt_s, void, env, i32, i32)
DEF_HELPER_3(vfsqrt_d, void, env, i32, i32)
DEF_HELPER_3(vfrecip_s, void, env, i32, i32)
DEF_HELPER_3(vfrecip_d, void, env, i32, i32)
DEF_HELPER_3(vfrsqrt_s, void, env, i32, i32)
DEF_HELPER_3(vfrsqrt_d, void, env, i32, i32)

DEF_HELPER_3(vfcvtl_s_h, void, env, i32, i32)
DEF_HELPER_3(vfcvth_s_h, void, env, i32, i32)
DEF_HELPER_3(vfcvtl_d_s, void, env, i32, i32)
DEF_HELPER_3(vfcvth_d_s, void, env, i32, i32)
DEF_HELPER_4(vfcvt_h_s, void, env, i32, i32, i32)
DEF_HELPER_4(vfcvt_s_d, void, env, i32, i32, i32)

DEF_HELPER_3(vfrintrne_s, void, env, i32, i32)
DEF_HELPER_3(vfrintrne_d, void, env, i32, i32)
DEF_HELPER_3(vfrintrz_s, void, env, i32, i32)
DEF_HELPER_3(vfrintrz_d, void, env, i32, i32)
DEF_HELPER_3(vfrintrp_s, void, env, i32, i32)
DEF_HELPER_3(vfrintrp_d, void, env, i32, i32)
DEF_HELPER_3(vfrintrm_s, void, env, i32, i32)
DEF_HELPER_3(vfrintrm_d, void, env, i32, i32)
DEF_HELPER_3(vfrint_s, void, env, i32, i32)
DEF_HELPER_3(vfrint_d, void, env, i32, i32)

DEF_HELPER_3(vftintrne_w_s, void, env, i32, i32)
DEF_HELPER_3(vftintrne_l_d, void, env, i32, i32)
DEF_HELPER_3(vftintrz_w_s, void, env, i32, i32)
DEF_HELPER_3(vftintrz_l_d, void, env, i32, i32)
DEF_HELPER_3(vftintrp_w_s, void, env, i32, i32)
DEF_HELPER_3(vftintrp_l_d, void, env, i32, i32)
DEF_HELPER_3(vftintrm_w_s, void, env, i32, i32)
DEF_HELPER_3(vftintrm_l_d, void, env, i32, i32)
DEF_HELPER_3(vftint_w_s, void, env, i32, i32)
DEF_HELPER_3(vftint_l_d, void, env, i32, i32)
DEF_HELPER_3(vftintrz_wu_s, void, env, i32, i32)
DEF_HELPER_3(vftintrz_lu_d, void, env, i32, i32)
DEF_HELPER_3(vftint_wu_s, void, env, i32, i32)
DEF_HELPER_3(vftint_lu_d, void, env, i32, i32)
DEF_HELPER_4(vftintrne_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vftintrz_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vftintrp_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vftintrm_w_d, void, env, i32, i32, i32)
DEF_HELPER_4(vftint_w_d, void, env, i32, i32, i32)
DEF_HELPER_3(vftintrnel_l_s, void, env, i32, i32)
DEF_HELPER_3(vftintrneh_l_s, void, env, i32, i32)
DEF_HELPER_3(vftintrzl_l_s, void, env, i32, i32)
DEF_HELPER_3(vftintrzh_l_s, void, env, i32, i32)
DEF_HELPER_3(vftintrpl_l_s, void, env, i32, i32)
DEF_HELPER_3(vftintrph_l_s, void, env, i32, i32)
DEF_HELPER_3(vftintrml_l_s, void, env, i32, i32)
DEF_HELPER_3(vftintrmh_l_s, void, env, i32, i32)
DEF_HELPER_3(vftintl_l_s, void, env, i32, i32)
DEF_HELPER_3(vftinth_l_s, void, env, i32, i32)

DEF_HELPER_3(vffint_s_w, void, env, i32, i32)
DEF_HELPER_3(vffint_d_l, void, env, i32, i32)
DEF_HELPER_3(vffint_s_wu, void, env, i32, i32)
DEF_HELPER_3(vffint_d_lu, void, env, i32, i32)
DEF_HELPER_3(vffintl_d_w, void, env, i32, i32)
DEF_HELPER_3(vffinth_d_w, void, env, i32, i32)
DEF_HELPER_4(vffint_s_l, void, env, i32, i32, i32)

DEF_HELPER_FLAGS_4(vseqi_b, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vseqi_h, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vseqi_w, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vseqi_d, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)

DEF_HELPER_FLAGS_4(vslei_b, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslei_h, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslei_w, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslei_d, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslei_bu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslei_hu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslei_wu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslei_du, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)

DEF_HELPER_FLAGS_4(vslti_b, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslti_h, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslti_w, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslti_d, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslti_bu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslti_hu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslti_wu, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)
DEF_HELPER_FLAGS_4(vslti_du, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)

DEF_HELPER_5(vfcmp_c_s, void, env, i32, i32, i32, i32)
DEF_HELPER_5(vfcmp_s_s, void, env, i32, i32, i32, i32)
DEF_HELPER_5(vfcmp_c_d, void, env, i32, i32, i32, i32)
DEF_HELPER_5(vfcmp_s_d, void, env, i32, i32, i32, i32)

DEF_HELPER_FLAGS_4(vbitseli_b, TCG_CALL_NO_RWG, void, ptr, ptr, i64, i32)

DEF_HELPER_3(vsetanyeqz_b, void, env, i32, i32)
DEF_HELPER_3(vsetanyeqz_h, void, env, i32, i32)
DEF_HELPER_3(vsetanyeqz_w, void, env, i32, i32)
DEF_HELPER_3(vsetanyeqz_d, void, env, i32, i32)
DEF_HELPER_3(vsetallnez_b, void, env, i32, i32)
DEF_HELPER_3(vsetallnez_h, void, env, i32, i32)
DEF_HELPER_3(vsetallnez_w, void, env, i32, i32)
DEF_HELPER_3(vsetallnez_d, void, env, i32, i32)

DEF_HELPER_4(vpackev_b, void, env, i32, i32, i32)
DEF_HELPER_4(vpackev_h, void, env, i32, i32, i32)
DEF_HELPER_4(vpackev_w, void, env, i32, i32, i32)
DEF_HELPER_4(vpackev_d, void, env, i32, i32, i32)
DEF_HELPER_4(vpackod_b, void, env, i32, i32, i32)
DEF_HELPER_4(vpackod_h, void, env, i32, i32, i32)
DEF_HELPER_4(vpackod_w, void, env, i32, i32, i32)
DEF_HELPER_4(vpackod_d, void, env, i32, i32, i32)

DEF_HELPER_4(vpickev_b, void, env, i32, i32, i32)
DEF_HELPER_4(vpickev_h, void, env, i32, i32, i32)
DEF_HELPER_4(vpickev_w, void, env, i32, i32, i32)
DEF_HELPER_4(vpickev_d, void, env, i32, i32, i32)
DEF_HELPER_4(vpickod_b, void, env, i32, i32, i32)
DEF_HELPER_4(vpickod_h, void, env, i32, i32, i32)
DEF_HELPER_4(vpickod_w, void, env, i32, i32, i32)
DEF_HELPER_4(vpickod_d, void, env, i32, i32, i32)

DEF_HELPER_4(vilvl_b, void, env, i32, i32, i32)
DEF_HELPER_4(vilvl_h, void, env, i32, i32, i32)
DEF_HELPER_4(vilvl_w, void, env, i32, i32, i32)
DEF_HELPER_4(vilvl_d, void, env, i32, i32, i32)
DEF_HELPER_4(vilvh_b, void, env, i32, i32, i32)
DEF_HELPER_4(vilvh_h, void, env, i32, i32, i32)
DEF_HELPER_4(vilvh_w, void, env, i32, i32, i32)
DEF_HELPER_4(vilvh_d, void, env, i32, i32, i32)

DEF_HELPER_5(vshuf_b, void, env, i32, i32, i32, i32)
DEF_HELPER_4(vshuf_h, void, env, i32, i32, i32)
DEF_HELPER_4(vshuf_w, void, env, i32, i32, i32)
DEF_HELPER_4(vshuf_d, void, env, i32, i32, i32)
DEF_HELPER_4(vshuf4i_b, void, env, i32, i32, i32)
DEF_HELPER_4(vshuf4i_h, void, env, i32, i32, i32)
DEF_HELPER_4(vshuf4i_w, void, env, i32, i32, i32)
DEF_HELPER_4(vshuf4i_d, void, env, i32, i32, i32)

DEF_HELPER_4(vpermi_w, void, env, i32, i32, i32)

DEF_HELPER_4(vextrins_b, void, env, i32, i32, i32)
DEF_HELPER_4(vextrins_h, void, env, i32, i32, i32)
DEF_HELPER_4(vextrins_w, void, env, i32, i32, i32)
DEF_HELPER_4(vextrins_d, void, env, i32, i32, i32)
