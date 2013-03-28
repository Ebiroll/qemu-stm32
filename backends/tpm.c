/*
 * QEMU TPM Backend
 *
 * Copyright IBM, Corp. 2013
 *
 * Authors:
 *  Stefan Berger   <stefanb@us.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 * Based on backends/rng.c by Anthony Liguori
 */

#include "backends/tpm.h"
#include "tpm/tpm_int.h"
#include "qapi/qmp/qerror.h"

enum TpmType tpm_backend_get_type(TPMBackend *s)
{
    TPMBackendClass *k = TPM_BACKEND_GET_CLASS(s);

    return k->ops->type;
}

const char *tpm_backend_get_desc(TPMBackend *s)
{
    TPMBackendClass *k = TPM_BACKEND_GET_CLASS(s);

    return k->ops->desc();
}

void tpm_backend_destroy(TPMBackend *s)
{
    TPMBackendClass *k = TPM_BACKEND_GET_CLASS(s);

    return k->ops->destroy(s);
}

int tpm_backend_init(TPMBackend *s, TPMState *state,
                     TPMRecvDataCB *datacb)
{
    TPMBackendClass *k = TPM_BACKEND_GET_CLASS(s);

    return k->ops->init(s, state, datacb);
}

int tpm_backend_startup_tpm(TPMBackend *s)
{
    TPMBackendClass *k = TPM_BACKEND_GET_CLASS(s);

    return k->ops->startup_tpm(s);
}

bool tpm_backend_had_startup_error(TPMBackend *s)
{
    TPMBackendClass *k = TPM_BACKEND_GET_CLASS(s);

    return k->ops->had_startup_error(s);
}

size_t tpm_backend_realloc_buffer(TPMBackend *s, TPMSizedBuffer *sb)
{
    TPMBackendClass *k = TPM_BACKEND_GET_CLASS(s);

    return k->ops->realloc_buffer(sb);
}

void tpm_backend_deliver_request(TPMBackend *s)
{
    TPMBackendClass *k = TPM_BACKEND_GET_CLASS(s);

    k->ops->deliver_request(s);
}

void tpm_backend_reset(TPMBackend *s)
{
    TPMBackendClass *k = TPM_BACKEND_GET_CLASS(s);

    k->ops->reset(s);
}

void tpm_backend_cancel_cmd(TPMBackend *s)
{
    TPMBackendClass *k = TPM_BACKEND_GET_CLASS(s);

    k->ops->cancel_cmd(s);
}

bool tpm_backend_get_tpm_established_flag(TPMBackend *s)
{
    TPMBackendClass *k = TPM_BACKEND_GET_CLASS(s);

    return k->ops->get_tpm_established_flag(s);
}

static bool tpm_backend_prop_get_opened(Object *obj, Error **errp)
{
    TPMBackend *s = TPM_BACKEND(obj);

    return s->opened;
}

void tpm_backend_open(TPMBackend *s, Error **errp)
{
    object_property_set_bool(OBJECT(s), true, "opened", errp);
}

static void tpm_backend_prop_set_opened(Object *obj, bool value, Error **errp)
{
    TPMBackend *s = TPM_BACKEND(obj);
    TPMBackendClass *k = TPM_BACKEND_GET_CLASS(s);

    if (value == s->opened) {
        return;
    }

    if (!value && s->opened) {
        error_set(errp, QERR_PERMISSION_DENIED);
        return;
    }

    if (k->opened) {
        k->opened(s, errp);
    }

    if (!error_is_set(errp)) {
        s->opened = value;
    }
}

static void tpm_backend_instance_init(Object *obj)
{
    object_property_add_bool(obj, "opened",
                             tpm_backend_prop_get_opened,
                             tpm_backend_prop_set_opened,
                             NULL);
}

static const TypeInfo tpm_backend_info = {
    .name = TYPE_TPM_BACKEND,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(TPMBackend),
    .instance_init = tpm_backend_instance_init,
    .class_size = sizeof(TPMBackendClass),
    .abstract = true,
};

static void register_types(void)
{
    type_register_static(&tpm_backend_info);
}

type_init(register_types);
