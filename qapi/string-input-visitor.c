/*
 * String parsing visitor
 *
 * Copyright Red Hat, Inc. 2012-2016
 *
 * Author: Paolo Bonzini <pbonzini@redhat.com>
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "qapi/string-input-visitor.h"
#include "qapi/visitor-impl.h"
#include "qapi/qmp/qerror.h"
#include "qemu/option.h"
#include "qemu/queue.h"
#include "qemu/range.h"


struct StringInputVisitor
{
    Visitor visitor;

    GList *ranges;
    GList *cur_range;
    int64_t cur;

    const char *string;
    void *list; /* Only needed for sanity checking the caller */
};

static StringInputVisitor *to_siv(Visitor *v)
{
    return container_of(v, StringInputVisitor, visitor);
}

static void free_range(void *range, void *dummy)
{
    g_free(range);
}

static int parse_str(StringInputVisitor *siv, const char *name, Error **errp)
{
    char *str = (char *) siv->string;
    long long start, end;
    Range *cur;
    char *endptr;

    if (siv->ranges) {
        return 0;
    }

    do {
        errno = 0;
        start = strtoll(str, &endptr, 0);
        if (errno == 0 && endptr > str) {
            if (*endptr == '\0') {
                cur = g_malloc0(sizeof(*cur));
                range_set_bounds(cur, start, start);
                siv->ranges = range_list_insert(siv->ranges, cur);
                cur = NULL;
                str = NULL;
            } else if (*endptr == '-') {
                str = endptr + 1;
                errno = 0;
                end = strtoll(str, &endptr, 0);
                if (errno == 0 && endptr > str && start <= end &&
                    (start > INT64_MAX - 65536 ||
                     end < start + 65536)) {
                    if (*endptr == '\0') {
                        cur = g_malloc0(sizeof(*cur));
                        range_set_bounds(cur, start, end);
                        siv->ranges = range_list_insert(siv->ranges, cur);
                        cur = NULL;
                        str = NULL;
                    } else if (*endptr == ',') {
                        str = endptr + 1;
                        cur = g_malloc0(sizeof(*cur));
                        range_set_bounds(cur, start, end);
                        siv->ranges = range_list_insert(siv->ranges, cur);
                        cur = NULL;
                    } else {
                        goto error;
                    }
                } else {
                    goto error;
                }
            } else if (*endptr == ',') {
                str = endptr + 1;
                cur = g_malloc0(sizeof(*cur));
                range_set_bounds(cur, start, start);
                siv->ranges = range_list_insert(siv->ranges, cur);
                cur = NULL;
            } else {
                goto error;
            }
        } else {
            goto error;
        }
    } while (str);

    return 0;
error:
    g_list_foreach(siv->ranges, free_range, NULL);
    g_list_free(siv->ranges);
    siv->ranges = NULL;
    error_setg(errp, QERR_INVALID_PARAMETER_VALUE, name ? name : "null",
               "an int64 value or range");
    return -1;
}

static void
start_list(Visitor *v, const char *name, GenericList **list, size_t size,
           Error **errp)
{
    StringInputVisitor *siv = to_siv(v);

    /* We don't support visits without a list */
    assert(list);
    siv->list = list;

    if (parse_str(siv, name, errp) < 0) {
        *list = NULL;
        return;
    }

    siv->cur_range = g_list_first(siv->ranges);
    if (siv->cur_range) {
        Range *r = siv->cur_range->data;
        if (r) {
            siv->cur = range_lob(r);
        }
        *list = g_malloc0(size);
    } else {
        *list = NULL;
    }
}

static GenericList *next_list(Visitor *v, GenericList *tail, size_t size)
{
    StringInputVisitor *siv = to_siv(v);
    Range *r;

    if (!siv->ranges || !siv->cur_range) {
        return NULL;
    }

    r = siv->cur_range->data;
    if (!r) {
        return NULL;
    }

    if (!range_contains(r, siv->cur)) {
        siv->cur_range = g_list_next(siv->cur_range);
        if (!siv->cur_range) {
            return NULL;
        }
        r = siv->cur_range->data;
        if (!r) {
            return NULL;
        }
        siv->cur = range_lob(r);
    }

    tail->next = g_malloc0(size);
    return tail->next;
}

static void end_list(Visitor *v, void **obj)
{
    StringInputVisitor *siv = to_siv(v);

    assert(siv->list == obj);
}

static void parse_type_int64(Visitor *v, const char *name, int64_t *obj,
                             Error **errp)
{
    StringInputVisitor *siv = to_siv(v);

    if (!siv->string) {
        error_setg(errp, QERR_INVALID_PARAMETER_TYPE, name ? name : "null",
                   "integer");
        return;
    }

    if (parse_str(siv, name, errp) < 0) {
        return;
    }

    if (!siv->ranges) {
        goto error;
    }

    if (!siv->cur_range) {
        Range *r;

        siv->cur_range = g_list_first(siv->ranges);
        if (!siv->cur_range) {
            goto error;
        }

        r = siv->cur_range->data;
        if (!r) {
            goto error;
        }

        siv->cur = range_lob(r);
    }

    *obj = siv->cur;
    siv->cur++;
    return;

error:
    error_setg(errp, QERR_INVALID_PARAMETER_VALUE, name ? name : "null",
               "an int64 value or range");
}

static void parse_type_uint64(Visitor *v, const char *name, uint64_t *obj,
                              Error **errp)
{
    /* FIXME: parse_type_int64 mishandles values over INT64_MAX */
    int64_t i;
    Error *err = NULL;
    parse_type_int64(v, name, &i, &err);
    if (err) {
        error_propagate(errp, err);
    } else {
        *obj = i;
    }
}

static void parse_type_size(Visitor *v, const char *name, uint64_t *obj,
                            Error **errp)
{
    StringInputVisitor *siv = to_siv(v);
    Error *err = NULL;
    uint64_t val;

    if (siv->string) {
        parse_option_size(name, siv->string, &val, &err);
    } else {
        error_setg(errp, QERR_INVALID_PARAMETER_TYPE, name ? name : "null",
                   "size");
        return;
    }
    if (err) {
        error_propagate(errp, err);
        return;
    }

    *obj = val;
}

static void parse_type_bool(Visitor *v, const char *name, bool *obj,
                            Error **errp)
{
    StringInputVisitor *siv = to_siv(v);

    if (siv->string) {
        if (!strcasecmp(siv->string, "on") ||
            !strcasecmp(siv->string, "yes") ||
            !strcasecmp(siv->string, "true")) {
            *obj = true;
            return;
        }
        if (!strcasecmp(siv->string, "off") ||
            !strcasecmp(siv->string, "no") ||
            !strcasecmp(siv->string, "false")) {
            *obj = false;
            return;
        }
    }

    error_setg(errp, QERR_INVALID_PARAMETER_TYPE, name ? name : "null",
               "boolean");
}

static void parse_type_str(Visitor *v, const char *name, char **obj,
                           Error **errp)
{
    StringInputVisitor *siv = to_siv(v);
    if (siv->string) {
        *obj = g_strdup(siv->string);
    } else {
        *obj = NULL;
        error_setg(errp, QERR_INVALID_PARAMETER_TYPE, name ? name : "null",
                   "string");
    }
}

static void parse_type_number(Visitor *v, const char *name, double *obj,
                              Error **errp)
{
    StringInputVisitor *siv = to_siv(v);
    char *endp = (char *) siv->string;
    double val;

    errno = 0;
    if (siv->string) {
        val = strtod(siv->string, &endp);
    }
    if (!siv->string || errno || endp == siv->string || *endp) {
        error_setg(errp, QERR_INVALID_PARAMETER_TYPE, name ? name : "null",
                   "number");
        return;
    }

    *obj = val;
}

static void parse_optional(Visitor *v, const char *name, bool *present)
{
    StringInputVisitor *siv = to_siv(v);

    if (!siv->string) {
        *present = false;
        return;
    }

    *present = true;
}

Visitor *string_input_get_visitor(StringInputVisitor *v)
{
    return &v->visitor;
}

static void string_input_free(Visitor *v)
{
    StringInputVisitor *siv = to_siv(v);

    string_input_visitor_cleanup(siv);
}

void string_input_visitor_cleanup(StringInputVisitor *v)
{
    g_list_foreach(v->ranges, free_range, NULL);
    g_list_free(v->ranges);
    g_free(v);
}

StringInputVisitor *string_input_visitor_new(const char *str)
{
    StringInputVisitor *v;

    v = g_malloc0(sizeof(*v));

    v->visitor.type = VISITOR_INPUT;
    v->visitor.type_int64 = parse_type_int64;
    v->visitor.type_uint64 = parse_type_uint64;
    v->visitor.type_size = parse_type_size;
    v->visitor.type_bool = parse_type_bool;
    v->visitor.type_str = parse_type_str;
    v->visitor.type_number = parse_type_number;
    v->visitor.start_list = start_list;
    v->visitor.next_list = next_list;
    v->visitor.end_list = end_list;
    v->visitor.optional = parse_optional;
    v->visitor.free = string_input_free;

    v->string = str;
    return v;
}
