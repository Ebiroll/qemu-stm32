/*
 * QTest testcase for CPU plugging
 *
 * Copyright (c) 2015 SUSE Linux GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"

#include "qemu-common.h"
#include "libqtest.h"
#include "qapi/qmp/types.h"

struct PlugTestData {
    char *machine;
    const char *cpu_model;
    char *device_model;
    unsigned sockets;
    unsigned cores;
    unsigned threads;
    unsigned maxcpus;
};
typedef struct PlugTestData PlugTestData;

static void test_plug_with_cpu_add(gconstpointer data)
{
    const PlugTestData *s = data;
    char *args;
    QDict *response;
    unsigned int i;

    args = g_strdup_printf("-machine %s -cpu %s "
                           "-smp sockets=%u,cores=%u,threads=%u,maxcpus=%u",
                           s->machine, s->cpu_model,
                           s->sockets, s->cores, s->threads, s->maxcpus);
    qtest_start(args);

    for (i = s->sockets * s->cores * s->threads; i < s->maxcpus; i++) {
        response = qmp("{ 'execute': 'cpu-add',"
                       "  'arguments': { 'id': %d } }", i);
        g_assert(response);
        g_assert(!qdict_haskey(response, "error"));
        QDECREF(response);
    }

    qtest_end();
    g_free(args);
}

static void test_plug_without_cpu_add(gconstpointer data)
{
    const PlugTestData *s = data;
    char *args;
    QDict *response;

    args = g_strdup_printf("-machine %s -cpu %s "
                           "-smp sockets=%u,cores=%u,threads=%u,maxcpus=%u",
                           s->machine, s->cpu_model,
                           s->sockets, s->cores, s->threads, s->maxcpus);
    qtest_start(args);

    response = qmp("{ 'execute': 'cpu-add',"
                   "  'arguments': { 'id': %d } }",
                   s->sockets * s->cores * s->threads);
    g_assert(response);
    g_assert(qdict_haskey(response, "error"));
    QDECREF(response);

    qtest_end();
    g_free(args);
}

static void test_plug_with_device_add_x86(gconstpointer data)
{
    const PlugTestData *td = data;
    char *args;
    unsigned int s, c, t;

    args = g_strdup_printf("-machine %s -cpu %s "
                           "-smp sockets=%u,cores=%u,threads=%u,maxcpus=%u",
                           td->machine, td->cpu_model,
                           td->sockets, td->cores, td->threads, td->maxcpus);
    qtest_start(args);

    for (s = td->sockets; s < td->maxcpus / td->cores / td->threads; s++) {
        for (c = 0; c < td->cores; c++) {
            for (t = 0; t < td->threads; t++) {
                char *id = g_strdup_printf("id-%i-%i-%i", s, c, t);
                qtest_qmp_device_add(td->device_model, id, "'socket-id':'%i', "
                                     "'core-id':'%i', 'thread-id':'%i'",
                                     s, c, t);
                g_free(id);
            }
        }
    }

    qtest_end();
    g_free(args);
}

static void test_plug_with_device_add_coreid(gconstpointer data)
{
    const PlugTestData *td = data;
    char *args;
    unsigned int c;

    args = g_strdup_printf("-machine %s -cpu %s "
                           "-smp 1,sockets=%u,cores=%u,threads=%u,maxcpus=%u",
                           td->machine, td->cpu_model,
                           td->sockets, td->cores, td->threads, td->maxcpus);
    qtest_start(args);

    for (c = td->cores; c < td->maxcpus / td->sockets / td->threads; c++) {
        char *id = g_strdup_printf("id-%i", c);
        qtest_qmp_device_add(td->device_model, id, "'core-id':'%i'", c);
        g_free(id);
    }

    qtest_end();
    g_free(args);
}

static void test_data_free(gpointer data)
{
    PlugTestData *pc = data;

    g_free(pc->machine);
    g_free(pc->device_model);
    g_free(pc);
}

static void add_pc_test_case(const char *mname)
{
    char *path;
    PlugTestData *data;

    if (!g_str_has_prefix(mname, "pc-")) {
        return;
    }
    data = g_new(PlugTestData, 1);
    data->machine = g_strdup(mname);
    data->cpu_model = "Haswell"; /* 1.3+ theoretically */
    data->device_model = g_strdup_printf("%s-%s-cpu", data->cpu_model,
                                         qtest_get_arch());
    data->sockets = 1;
    data->cores = 3;
    data->threads = 2;
    data->maxcpus = data->sockets * data->cores * data->threads * 2;
    if (g_str_has_suffix(mname, "-1.4") ||
        (strcmp(mname, "pc-1.3") == 0) ||
        (strcmp(mname, "pc-1.2") == 0) ||
        (strcmp(mname, "pc-1.1") == 0) ||
        (strcmp(mname, "pc-1.0") == 0) ||
        (strcmp(mname, "pc-0.15") == 0) ||
        (strcmp(mname, "pc-0.14") == 0) ||
        (strcmp(mname, "pc-0.13") == 0) ||
        (strcmp(mname, "pc-0.12") == 0) ||
        (strcmp(mname, "pc-0.11") == 0) ||
        (strcmp(mname, "pc-0.10") == 0)) {
        path = g_strdup_printf("cpu-plug/%s/init/%ux%ux%u&maxcpus=%u",
                               mname, data->sockets, data->cores,
                               data->threads, data->maxcpus);
        qtest_add_data_func_full(path, data, test_plug_without_cpu_add,
                                 test_data_free);
        g_free(path);
    } else {
        PlugTestData *data2 = g_memdup(data, sizeof(PlugTestData));

        data2->machine = g_strdup(data->machine);
        data2->device_model = g_strdup(data->device_model);

        path = g_strdup_printf("cpu-plug/%s/cpu-add/%ux%ux%u&maxcpus=%u",
                               mname, data->sockets, data->cores,
                               data->threads, data->maxcpus);
        qtest_add_data_func_full(path, data, test_plug_with_cpu_add,
                                 test_data_free);
        g_free(path);
        path = g_strdup_printf("cpu-plug/%s/device-add/%ux%ux%u&maxcpus=%u",
                               mname, data2->sockets, data2->cores,
                               data2->threads, data2->maxcpus);
        qtest_add_data_func_full(path, data2, test_plug_with_device_add_x86,
                                 test_data_free);
        g_free(path);
    }
}

static void add_pseries_test_case(const char *mname)
{
    char *path;
    PlugTestData *data;

    if (!g_str_has_prefix(mname, "pseries-") ||
        (g_str_has_prefix(mname, "pseries-2.") && atoi(&mname[10]) < 7)) {
        return;
    }
    data = g_new(PlugTestData, 1);
    data->machine = g_strdup(mname);
    data->cpu_model = "power8_v2.0";
    data->device_model = g_strdup("power8_v2.0-spapr-cpu-core");
    data->sockets = 2;
    data->cores = 3;
    data->threads = 1;
    data->maxcpus = data->sockets * data->cores * data->threads * 2;

    path = g_strdup_printf("cpu-plug/%s/device-add/%ux%ux%u&maxcpus=%u",
                           mname, data->sockets, data->cores,
                           data->threads, data->maxcpus);
    qtest_add_data_func_full(path, data, test_plug_with_device_add_coreid,
                             test_data_free);
    g_free(path);
}

int main(int argc, char **argv)
{
    const char *arch = qtest_get_arch();

    g_test_init(&argc, &argv, NULL);

    if (strcmp(arch, "i386") == 0 || strcmp(arch, "x86_64") == 0) {
        qtest_cb_for_every_machine(add_pc_test_case);
    } else if (g_str_equal(arch, "ppc64")) {
        qtest_cb_for_every_machine(add_pseries_test_case);
    }

    return g_test_run();
}
