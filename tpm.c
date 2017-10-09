/*
 * TPM configuration
 *
 * Copyright (C) 2011-2013 IBM Corporation
 *
 * Authors:
 *  Stefan Berger    <stefanb@us.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 * Based on net.c
 */
#include "qemu/osdep.h"

#include "qapi/qmp/qerror.h"
#include "sysemu/tpm_backend.h"
#include "sysemu/tpm.h"
#include "qemu/config-file.h"
#include "qemu/error-report.h"
#include "qmp-commands.h"

static QLIST_HEAD(, TPMBackend) tpm_backends =
    QLIST_HEAD_INITIALIZER(tpm_backends);

static bool tpm_models[TPM_MODEL__MAX];

void tpm_register_model(enum TpmModel model)
{
    tpm_models[model] = true;
}

#ifdef CONFIG_TPM

static const TPMBackendClass *
tpm_be_find_by_type(enum TpmType type)
{
    ObjectClass *oc;
    char *typename = g_strdup_printf("tpm-%s", TpmType_str(type));

    oc = object_class_by_name(typename);
    g_free(typename);

    if (!object_class_dynamic_cast(oc, TYPE_TPM_BACKEND)) {
        return NULL;
    }

    return TPM_BACKEND_CLASS(oc);
}

/*
 * Walk the list of available TPM backend drivers and display them on the
 * screen.
 */
static void tpm_display_backend_drivers(void)
{
    int i;

    fprintf(stderr, "Supported TPM types (choose only one):\n");

    for (i = 0; i < TPM_TYPE__MAX; i++) {
        const TPMBackendClass *bc = tpm_be_find_by_type(i);
        if (!bc) {
            continue;
        }
        fprintf(stderr, "%12s   %s\n", TpmType_str(i), bc->desc);
    }
    fprintf(stderr, "\n");
}

/*
 * Find the TPM with the given Id
 */
TPMBackend *qemu_find_tpm(const char *id)
{
    TPMBackend *drv;

    if (id) {
        QLIST_FOREACH(drv, &tpm_backends, list) {
            if (!strcmp(drv->id, id)) {
                return drv;
            }
        }
    }

    return NULL;
}

static int tpm_init_tpmdev(void *dummy, QemuOpts *opts, Error **errp)
{
    const char *value;
    const char *id;
    const TPMBackendClass *be;
    TPMBackend *drv;
    Error *local_err = NULL;
    int i;

    if (!QLIST_EMPTY(&tpm_backends)) {
        error_report("Only one TPM is allowed.");
        return 1;
    }

    id = qemu_opts_id(opts);
    if (id == NULL) {
        error_report(QERR_MISSING_PARAMETER, "id");
        return 1;
    }

    value = qemu_opt_get(opts, "type");
    if (!value) {
        error_report(QERR_MISSING_PARAMETER, "type");
        tpm_display_backend_drivers();
        return 1;
    }

    i = qapi_enum_parse(&TpmType_lookup, value, -1, NULL);
    be = i >= 0 ? tpm_be_find_by_type(i) : NULL;
    if (be == NULL) {
        error_report(QERR_INVALID_PARAMETER_VALUE,
                     "type", "a TPM backend type");
        tpm_display_backend_drivers();
        return 1;
    }

    /* validate backend specific opts */
    qemu_opts_validate(opts, be->opts, &local_err);
    if (local_err) {
        error_report_err(local_err);
        return 1;
    }

    drv = be->create(opts, id);
    if (!drv) {
        return 1;
    }

    tpm_backend_open(drv, &local_err);
    if (local_err) {
        error_report_err(local_err);
        return 1;
    }

    QLIST_INSERT_HEAD(&tpm_backends, drv, list);

    return 0;
}

/*
 * Walk the list of TPM backend drivers that are in use and call their
 * destroy function to have them cleaned up.
 */
void tpm_cleanup(void)
{
    TPMBackend *drv, *next;

    QLIST_FOREACH_SAFE(drv, &tpm_backends, list, next) {
        QLIST_REMOVE(drv, list);
        object_unref(OBJECT(drv));
    }
}

/*
 * Initialize the TPM. Process the tpmdev command line options describing the
 * TPM backend.
 */
int tpm_init(void)
{
    if (qemu_opts_foreach(qemu_find_opts("tpmdev"),
                          tpm_init_tpmdev, NULL, NULL)) {
        return -1;
    }

    return 0;
}

/*
 * Parse the TPM configuration options.
 * To display all available TPM backends the user may use '-tpmdev help'
 */
int tpm_config_parse(QemuOptsList *opts_list, const char *optarg)
{
    QemuOpts *opts;

    if (!strcmp(optarg, "help")) {
        tpm_display_backend_drivers();
        return -1;
    }
    opts = qemu_opts_parse_noisily(opts_list, optarg, true);
    if (!opts) {
        return -1;
    }
    return 0;
}

#endif /* CONFIG_TPM */

/*
 * Walk the list of active TPM backends and collect information about them
 * following the schema description in qapi-schema.json.
 */
TPMInfoList *qmp_query_tpm(Error **errp)
{
    TPMBackend *drv;
    TPMInfoList *info, *head = NULL, *cur_item = NULL;

    QLIST_FOREACH(drv, &tpm_backends, list) {
        if (!tpm_models[drv->fe_model]) {
            continue;
        }
        info = g_new0(TPMInfoList, 1);
        info->value = tpm_backend_query_tpm(drv);

        if (!cur_item) {
            head = cur_item = info;
        } else {
            cur_item->next = info;
            cur_item = info;
        }
    }

    return head;
}

TpmTypeList *qmp_query_tpm_types(Error **errp)
{
    unsigned int i = 0;
    TpmTypeList *head = NULL, *prev = NULL, *cur_item;

    for (i = 0; i < TPM_TYPE__MAX; i++) {
        if (!tpm_be_find_by_type(i)) {
            continue;
        }
        cur_item = g_new0(TpmTypeList, 1);
        cur_item->value = i;

        if (prev) {
            prev->next = cur_item;
        }
        if (!head) {
            head = cur_item;
        }
        prev = cur_item;
    }

    return head;
}

TpmModelList *qmp_query_tpm_models(Error **errp)
{
    unsigned int i = 0;
    TpmModelList *head = NULL, *prev = NULL, *cur_item;

    for (i = 0; i < TPM_MODEL__MAX; i++) {
        if (!tpm_models[i]) {
            continue;
        }
        cur_item = g_new0(TpmModelList, 1);
        cur_item->value = i;

        if (prev) {
            prev->next = cur_item;
        }
        if (!head) {
            head = cur_item;
        }
        prev = cur_item;
    }

    return head;
}
