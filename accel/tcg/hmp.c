#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "qapi/qapi-commands-machine.h"
#include "exec/exec-all.h"
#include "monitor/monitor.h"
#include "sysemu/tcg.h"

static void hmp_info_opcount(Monitor *mon, const QDict *qdict)
{
    dump_opcount_info();
}

static void hmp_tcg_register(void)
{
    monitor_register_hmp_info_hrt("jit", qmp_x_query_jit);
    monitor_register_hmp("opcount", true, hmp_info_opcount);
}

type_init(hmp_tcg_register);
