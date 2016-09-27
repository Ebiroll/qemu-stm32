/*
 * COarse-grain LOck-stepping Virtual Machines for Non-stop Service (COLO)
 * (a.k.a. Fault Tolerance or Continuous Replication)
 *
 * Copyright (c) 2016 HUAWEI TECHNOLOGIES CO., LTD.
 * Copyright (c) 2016 FUJITSU LIMITED
 * Copyright (c) 2016 Intel Corporation
 *
 * Author: Zhang Chen <zhangchen.fnst@cn.fujitsu.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later.  See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu-common.h"
#include "qapi/qmp/qerror.h"
#include "qapi/error.h"
#include "net/net.h"
#include "qom/object_interfaces.h"
#include "qemu/iov.h"
#include "qom/object.h"
#include "qemu/typedefs.h"
#include "net/queue.h"
#include "sysemu/char.h"
#include "qemu/sockets.h"
#include "qapi-visit.h"

#define TYPE_COLO_COMPARE "colo-compare"
#define COLO_COMPARE(obj) \
    OBJECT_CHECK(CompareState, (obj), TYPE_COLO_COMPARE)

#define COMPARE_READ_LEN_MAX NET_BUFSIZE

typedef struct CompareState {
    Object parent;

    char *pri_indev;
    char *sec_indev;
    char *outdev;
    CharDriverState *chr_pri_in;
    CharDriverState *chr_sec_in;
    CharDriverState *chr_out;
    SocketReadState pri_rs;
    SocketReadState sec_rs;
} CompareState;

typedef struct CompareClass {
    ObjectClass parent_class;
} CompareClass;

typedef struct CompareChardevProps {
    bool is_socket;
} CompareChardevProps;

static char *compare_get_pri_indev(Object *obj, Error **errp)
{
    CompareState *s = COLO_COMPARE(obj);

    return g_strdup(s->pri_indev);
}

static void compare_set_pri_indev(Object *obj, const char *value, Error **errp)
{
    CompareState *s = COLO_COMPARE(obj);

    g_free(s->pri_indev);
    s->pri_indev = g_strdup(value);
}

static char *compare_get_sec_indev(Object *obj, Error **errp)
{
    CompareState *s = COLO_COMPARE(obj);

    return g_strdup(s->sec_indev);
}

static void compare_set_sec_indev(Object *obj, const char *value, Error **errp)
{
    CompareState *s = COLO_COMPARE(obj);

    g_free(s->sec_indev);
    s->sec_indev = g_strdup(value);
}

static char *compare_get_outdev(Object *obj, Error **errp)
{
    CompareState *s = COLO_COMPARE(obj);

    return g_strdup(s->outdev);
}

static void compare_set_outdev(Object *obj, const char *value, Error **errp)
{
    CompareState *s = COLO_COMPARE(obj);

    g_free(s->outdev);
    s->outdev = g_strdup(value);
}

static void compare_pri_rs_finalize(SocketReadState *pri_rs)
{
    /* if packet_enqueue pri pkt failed we will send unsupported packet */
}

static void compare_sec_rs_finalize(SocketReadState *sec_rs)
{
    /* if packet_enqueue sec pkt failed we will notify trace */
}

static int compare_chardev_opts(void *opaque,
                                const char *name, const char *value,
                                Error **errp)
{
    CompareChardevProps *props = opaque;

    if (strcmp(name, "backend") == 0 &&
        strcmp(value, "socket") == 0) {
        props->is_socket = true;
        return 0;
    } else if (strcmp(name, "host") == 0 ||
              (strcmp(name, "port") == 0) ||
              (strcmp(name, "server") == 0) ||
              (strcmp(name, "wait") == 0) ||
              (strcmp(name, "path") == 0)) {
        return 0;
    } else {
        error_setg(errp,
                   "COLO-compare does not support a chardev with option %s=%s",
                   name, value);
        return -1;
    }
}

/*
 * Return 0 is success.
 * Return 1 is failed.
 */
static int find_and_check_chardev(CharDriverState **chr,
                                  char *chr_name,
                                  Error **errp)
{
    CompareChardevProps props;

    *chr = qemu_chr_find(chr_name);
    if (*chr == NULL) {
        error_setg(errp, "Device '%s' not found",
                   chr_name);
        return 1;
    }

    memset(&props, 0, sizeof(props));
    if (qemu_opt_foreach((*chr)->opts, compare_chardev_opts, &props, errp)) {
        return 1;
    }

    if (!props.is_socket) {
        error_setg(errp, "chardev \"%s\" is not a tcp socket",
                   chr_name);
        return 1;
    }
    return 0;
}

/*
 * Called from the main thread on the primary
 * to setup colo-compare.
 */
static void colo_compare_complete(UserCreatable *uc, Error **errp)
{
    CompareState *s = COLO_COMPARE(uc);

    if (!s->pri_indev || !s->sec_indev || !s->outdev) {
        error_setg(errp, "colo compare needs 'primary_in' ,"
                   "'secondary_in','outdev' property set");
        return;
    } else if (!strcmp(s->pri_indev, s->outdev) ||
               !strcmp(s->sec_indev, s->outdev) ||
               !strcmp(s->pri_indev, s->sec_indev)) {
        error_setg(errp, "'indev' and 'outdev' could not be same "
                   "for compare module");
        return;
    }

    if (find_and_check_chardev(&s->chr_pri_in, s->pri_indev, errp)) {
        return;
    }

    if (find_and_check_chardev(&s->chr_sec_in, s->sec_indev, errp)) {
        return;
    }

    if (find_and_check_chardev(&s->chr_out, s->outdev, errp)) {
        return;
    }

    qemu_chr_fe_claim_no_fail(s->chr_pri_in);

    qemu_chr_fe_claim_no_fail(s->chr_sec_in);

    qemu_chr_fe_claim_no_fail(s->chr_out);

    net_socket_rs_init(&s->pri_rs, compare_pri_rs_finalize);
    net_socket_rs_init(&s->sec_rs, compare_sec_rs_finalize);

    return;
}

static void colo_compare_class_init(ObjectClass *oc, void *data)
{
    UserCreatableClass *ucc = USER_CREATABLE_CLASS(oc);

    ucc->complete = colo_compare_complete;
}

static void colo_compare_init(Object *obj)
{
    object_property_add_str(obj, "primary_in",
                            compare_get_pri_indev, compare_set_pri_indev,
                            NULL);
    object_property_add_str(obj, "secondary_in",
                            compare_get_sec_indev, compare_set_sec_indev,
                            NULL);
    object_property_add_str(obj, "outdev",
                            compare_get_outdev, compare_set_outdev,
                            NULL);
}

static void colo_compare_finalize(Object *obj)
{
    CompareState *s = COLO_COMPARE(obj);

    if (s->chr_pri_in) {
        qemu_chr_add_handlers(s->chr_pri_in, NULL, NULL, NULL, NULL);
        qemu_chr_fe_release(s->chr_pri_in);
    }
    if (s->chr_sec_in) {
        qemu_chr_add_handlers(s->chr_sec_in, NULL, NULL, NULL, NULL);
        qemu_chr_fe_release(s->chr_sec_in);
    }
    if (s->chr_out) {
        qemu_chr_fe_release(s->chr_out);
    }

    g_free(s->pri_indev);
    g_free(s->sec_indev);
    g_free(s->outdev);
}

static const TypeInfo colo_compare_info = {
    .name = TYPE_COLO_COMPARE,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(CompareState),
    .instance_init = colo_compare_init,
    .instance_finalize = colo_compare_finalize,
    .class_size = sizeof(CompareClass),
    .class_init = colo_compare_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_USER_CREATABLE },
        { }
    }
};

static void register_types(void)
{
    type_register_static(&colo_compare_info);
}

type_init(register_types);
