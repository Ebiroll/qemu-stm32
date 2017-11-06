/*
 * Public TPM functions
 *
 * Copyright (C) 2011-2013 IBM Corporation
 *
 * Authors:
 *  Stefan Berger    <stefanb@us.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */
#ifndef QEMU_TPM_H
#define QEMU_TPM_H

#include "qom/object.h"
#include "qapi-types.h"

int tpm_config_parse(QemuOptsList *opts_list, const char *optarg);
int tpm_init(void);
void tpm_cleanup(void);

typedef enum TPMVersion {
    TPM_VERSION_UNSPEC = 0,
    TPM_VERSION_1_2 = 1,
    TPM_VERSION_2_0 = 2,
} TPMVersion;

#define TYPE_TPM_IF "tpm-if"
#define TPM_IF_CLASS(klass)                                 \
    OBJECT_CLASS_CHECK(TPMIfClass, (klass), TYPE_TPM_IF)
#define TPM_IF_GET_CLASS(obj)                           \
    OBJECT_GET_CLASS(TPMIfClass, (obj), TYPE_TPM_IF)
#define TPM_IF(obj)                             \
    INTERFACE_CHECK(TPMIf, (obj), TYPE_TPM_IF)

typedef struct TPMIf {
    Object parent_obj;
} TPMIf;

typedef struct TPMIfClass {
    InterfaceClass parent_class;

    enum TpmModel model;
    void (*request_completed)(TPMIf *obj);
} TPMIfClass;

#define TYPE_TPM_TIS                "tpm-tis"

/* returns NULL unless there is exactly one TPM device */
static inline TPMIf *tpm_find(void)
{
    Object *obj = object_resolve_path_type("", TYPE_TPM_IF, NULL);

    return TPM_IF(obj);
}

TPMVersion tpm_tis_get_tpm_version(Object *obj);

static inline TPMVersion tpm_get_version(TPMIf *ti)
{
    if (!ti) {
        return TPM_VERSION_UNSPEC;
    }

    return tpm_tis_get_tpm_version(OBJECT(ti));
}

#endif /* QEMU_TPM_H */
