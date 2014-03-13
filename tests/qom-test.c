/*
 * QTest testcase for QOM
 *
 * Copyright (c) 2013 SUSE LINUX Products GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include <glib.h>
#include <string.h>

#include "qemu-common.h"
#include "libqtest.h"
#include "qemu/osdep.h"
#include "qapi/qmp/types.h"

static const char *blacklist_x86[] = {
    "xenfv", "xenpv", NULL
};

static const struct {
    const char *arch;
    const char **machine;
} blacklists[] = {
    { "i386", blacklist_x86 },
    { "x86_64", blacklist_x86 },
};

static bool is_blacklisted(const char *arch, const char *mach)
{
    int i;
    const char **p;

    for (i = 0; i < ARRAY_SIZE(blacklists); i++) {
        if (!strcmp(blacklists[i].arch, arch)) {
            for (p = blacklists[i].machine; *p; p++) {
                if (!strcmp(*p, mach)) {
                    return true;
                }
            }
        }
    }
    return false;
}

static void test_properties(const char *path)
{
    char *child_path;
    QDict *response, *tuple;
    QList *list;
    QListEntry *entry;

    g_test_message("Obtaining properties of %s", path);
    response = qmp("{ 'execute': 'qom-list',"
                   "  'arguments': { 'path': '%s' } }", path);
    g_assert(response);

    g_assert(qdict_haskey(response, "return"));
    list = qobject_to_qlist(qdict_get(response, "return"));
    QLIST_FOREACH_ENTRY(list, entry) {
        tuple = qobject_to_qdict(qlist_entry_obj(entry));
        if (strstart(qdict_get_str(tuple, "type"), "child<", NULL)) {
            child_path = g_strdup_printf("%s/%s",
                                         path, qdict_get_str(tuple, "name"));
            test_properties(child_path);
            g_free(child_path);
        } else {
            const char *prop = qdict_get_str(tuple, "name");
            g_test_message("Testing property %s.%s", path, prop);
            response = qmp("{ 'execute': 'qom-get',"
                           "  'arguments': { 'path': '%s',"
                           "                 'property': '%s' } }",
                           path, prop);
            /* qom-get may fail but should not, e.g., segfault. */
            g_assert(response);
        }
    }
}

static void test_machine(gconstpointer data)
{
    const char *machine = data;
    char *args;
    QDict *response;

    args = g_strdup_printf("-machine %s", machine);
    qtest_start(args);

    test_properties("/machine");

    response = qmp("{ 'execute': 'quit' }");
    g_assert(qdict_haskey(response, "return"));

    qtest_end();
    g_free(args);
}

static void add_machine_test_cases(void)
{
    const char *arch = qtest_get_arch();
    QDict *response, *minfo;
    QList *list;
    const QListEntry *p;
    QObject *qobj;
    QString *qstr;
    const char *mname, *path;

    qtest_start("-machine none");
    response = qmp("{ 'execute': 'query-machines' }");
    g_assert(response);
    list = qdict_get_qlist(response, "return");
    g_assert(list);

    for (p = qlist_first(list); p; p = qlist_next(p)) {
        minfo = qobject_to_qdict(qlist_entry_obj(p));
        g_assert(minfo);
        qobj = qdict_get(minfo, "name");
        g_assert(qobj);
        qstr = qobject_to_qstring(qobj);
        g_assert(qstr);
        mname = qstring_get_str(qstr);
        if (!is_blacklisted(arch, mname)) {
            path = g_strdup_printf("/%s/qom/%s", arch, mname);
            g_test_add_data_func(path, mname, test_machine);
        }
    }
    qtest_end();
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    add_machine_test_cases();

    return g_test_run();
}
