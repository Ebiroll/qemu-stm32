/*
 * QEMU Object Model.
 *
 * Based on ideas by Avi Kivity <avi@redhat.com>
 *
 * Copyright (C) 2009, 2015 Red Hat Inc.
 *
 * Authors:
 *  Luiz Capitulino <lcapitulino@redhat.com>
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 * QObject Reference Counts Terminology
 * ------------------------------------
 *
 *  - Returning references: A function that returns an object may
 *  return it as either a weak or a strong reference.  If the reference
 *  is strong, you are responsible for calling QDECREF() on the reference
 *  when you are done.
 *
 *  If the reference is weak, the owner of the reference may free it at
 *  any time in the future.  Before storing the reference anywhere, you
 *  should call QINCREF() to make the reference strong.
 *
 *  - Transferring ownership: when you transfer ownership of a reference
 *  by calling a function, you are no longer responsible for calling
 *  QDECREF() when the reference is no longer needed.  In other words,
 *  when the function returns you must behave as if the reference to the
 *  passed object was weak.
 */
#ifndef QOBJECT_H
#define QOBJECT_H

#include <stddef.h>
#include <assert.h>

typedef enum {
    QTYPE_NONE,    /* sentinel value, no QObject has this type code */
    QTYPE_QNULL,
    QTYPE_QINT,
    QTYPE_QSTRING,
    QTYPE_QDICT,
    QTYPE_QLIST,
    QTYPE_QFLOAT,
    QTYPE_QBOOL,
    QTYPE_MAX,
} qtype_code;

typedef struct QObject {
    qtype_code type;
    size_t refcnt;
} QObject;

/* Get the 'base' part of an object */
#define QOBJECT(obj) (&(obj)->base)

/* High-level interface for qobject_incref() */
#define QINCREF(obj)      \
    qobject_incref(QOBJECT(obj))

/* High-level interface for qobject_decref() */
#define QDECREF(obj)              \
    qobject_decref(obj ? QOBJECT(obj) : NULL)

/* Initialize an object to default values */
static inline void qobject_init(QObject *obj, qtype_code type)
{
    assert(QTYPE_NONE < type && type < QTYPE_MAX);
    obj->refcnt = 1;
    obj->type = type;
}

/**
 * qobject_incref(): Increment QObject's reference count
 */
static inline void qobject_incref(QObject *obj)
{
    if (obj)
        obj->refcnt++;
}

/**
 * qobject_destroy(): Free resources used by the object
 */
void qobject_destroy(QObject *obj);

/**
 * qobject_decref(): Decrement QObject's reference count, deallocate
 * when it reaches zero
 */
static inline void qobject_decref(QObject *obj)
{
    assert(!obj || obj->refcnt);
    if (obj && --obj->refcnt == 0) {
        qobject_destroy(obj);
    }
}

/**
 * qobject_type(): Return the QObject's type
 */
static inline qtype_code qobject_type(const QObject *obj)
{
    assert(QTYPE_NONE < obj->type && obj->type < QTYPE_MAX);
    return obj->type;
}

extern QObject qnull_;

static inline QObject *qnull(void)
{
    qobject_incref(&qnull_);
    return &qnull_;
}

#endif /* QOBJECT_H */
