#include "def-helper.h"

DEF_HELPER_1(exception, noreturn, i32)
DEF_HELPER_2(exception_cause, noreturn, i32, i32)
DEF_HELPER_3(exception_cause_vaddr, noreturn, i32, i32, i32)
DEF_HELPER_2(debug_exception, noreturn, i32, i32)

DEF_HELPER_FLAGS_1(nsa, TCG_CALL_CONST | TCG_CALL_PURE, i32, i32)
DEF_HELPER_FLAGS_1(nsau, TCG_CALL_CONST | TCG_CALL_PURE, i32, i32)
DEF_HELPER_1(wsr_windowbase, void, i32)
DEF_HELPER_3(entry, void, i32, i32, i32)
DEF_HELPER_1(retw, i32, i32)
DEF_HELPER_1(rotw, void, i32)
DEF_HELPER_2(window_check, void, i32, i32)
DEF_HELPER_0(restore_owb, void)
DEF_HELPER_1(movsp, void, i32)
DEF_HELPER_1(wsr_lbeg, void, i32)
DEF_HELPER_1(wsr_lend, void, i32)
DEF_HELPER_1(simcall, void, env)
DEF_HELPER_0(dump_state, void)

DEF_HELPER_2(waiti, void, i32, i32)
DEF_HELPER_2(timer_irq, void, i32, i32)
DEF_HELPER_1(advance_ccount, void, i32)
DEF_HELPER_1(check_interrupts, void, env)

DEF_HELPER_1(wsr_rasid, void, i32)
DEF_HELPER_FLAGS_2(rtlb0, TCG_CALL_CONST | TCG_CALL_PURE, i32, i32, i32)
DEF_HELPER_FLAGS_2(rtlb1, TCG_CALL_CONST | TCG_CALL_PURE, i32, i32, i32)
DEF_HELPER_2(itlb, void, i32, i32)
DEF_HELPER_2(ptlb, i32, i32, i32)
DEF_HELPER_3(wtlb, void, i32, i32, i32)

DEF_HELPER_1(wsr_ibreakenable, void, i32)
DEF_HELPER_2(wsr_ibreaka, void, i32, i32)
DEF_HELPER_2(wsr_dbreaka, void, i32, i32)
DEF_HELPER_2(wsr_dbreakc, void, i32, i32)

#include "def-helper.h"
