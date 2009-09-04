/*
 * QInt data type.
 *
 * Copyright (C) 2009 Red Hat Inc.
 *
 * Authors:
 *  Luiz Capitulino <lcapitulino@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */
#include "qint.h"
#include "qobject.h"
#include "qemu-common.h"

static void qint_destroy_obj(QObject *obj);

static const QType qint_type = {
    .code = QTYPE_QINT,
    .destroy = qint_destroy_obj,
};

/**
 * qint_from_int(): Create a new QInt from an int64_t
 *
 * Return strong reference.
 */
QInt *qint_from_int(int64_t value)
{
    QInt *qi;

    qi = qemu_malloc(sizeof(*qi));
    qi->value = value;
    QOBJECT_INIT(qi, &qint_type);

    return qi;
}

/**
 * qint_get_int(): Get the stored integer
 */
int64_t qint_get_int(const QInt *qi)
{
    return qi->value;
}

/**
 * qobject_to_qint(): Convert a QObject into a QInt
 */
QInt *qobject_to_qint(const QObject *obj)
{
    if (qobject_type(obj) != QTYPE_QINT)
        return NULL;

    return container_of(obj, QInt, base);
}

/**
 * qint_destroy_obj(): Free all memory allocated by a
 * QInt object
 */
static void qint_destroy_obj(QObject *obj)
{
    assert(obj != NULL);
    qemu_free(qobject_to_qint(obj));
}
