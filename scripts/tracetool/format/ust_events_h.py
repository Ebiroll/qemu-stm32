#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
trace/generated-ust-provider.h
"""

__author__     = "Mohamad Gebai <mohamad.gebai@polymtl.ca>"
__copyright__  = "Copyright 2012, Mohamad Gebai <mohamad.gebai@polymtl.ca>"
__license__    = "GPL version 2 or (at your option) any later version"

__maintainer__ = "Stefan Hajnoczi"
__email__      = "stefanha@redhat.com"


from tracetool import out


def generate(events, backend, group):
    events = [e for e in events
              if "disabled" not in e.properties]

    if group == "all":
        include = "trace-ust-all.h"
    else:
        include = "trace-ust.h"

    out('/* This file is autogenerated by tracetool, do not edit. */',
        '',
        '#undef TRACEPOINT_PROVIDER',
        '#define TRACEPOINT_PROVIDER qemu',
        '',
        '#undef TRACEPOINT_INCLUDE_FILE',
        '#define TRACEPOINT_INCLUDE_FILE ./%s' % include,
        '',
        '#if !defined (TRACE_%s_GENERATED_UST_H) || \\'  % group.upper(),
        '     defined(TRACEPOINT_HEADER_MULTI_READ)',
        '#define TRACE_%s_GENERATED_UST_H' % group.upper(),
        '',
        '#include "qemu-common.h"',
        '#include <lttng/tracepoint.h>',
        '',
        '/*',
        ' * LTTng ust 2.0 does not allow you to use TP_ARGS(void) for tracepoints',
        ' * requiring no arguments. We define these macros introduced in more recent'
        ' * versions of LTTng ust as a workaround',
        ' */',
        '#ifndef _TP_EXPROTO1',
        '#define _TP_EXPROTO1(a)               void',
        '#endif',
        '#ifndef _TP_EXDATA_PROTO1',
        '#define _TP_EXDATA_PROTO1(a)          void *__tp_data',
        '#endif',
        '#ifndef _TP_EXDATA_VAR1',
        '#define _TP_EXDATA_VAR1(a)            __tp_data',
        '#endif',
        '#ifndef _TP_EXVAR1',
        '#define _TP_EXVAR1(a)',
        '#endif',
        '')

    for e in events:
        if len(e.args) > 0:
            out('TRACEPOINT_EVENT(',
                '   qemu,',
                '   %(name)s,',
                '   TP_ARGS(%(args)s),',
                '   TP_FIELDS(',
                name=e.name,
                args=", ".join(", ".join(i) for i in e.args))

            types = e.args.types()
            names = e.args.names()
            fmts = e.formats()
            for t,n,f in zip(types, names, fmts):
                if ('char *' in t) or ('char*' in t):
                    out('       ctf_string(' + n + ', ' + n + ')')
                elif ("%p" in f) or ("x" in f) or ("PRIx" in f):
                    out('       ctf_integer_hex('+ t + ', ' + n + ', ' + n + ')')
                elif ("ptr" in t) or ("*" in t):
                    out('       ctf_integer_hex('+ t + ', ' + n + ', ' + n + ')')
                elif ('int' in t) or ('long' in t) or ('unsigned' in t) or ('size_t' in t):
                    out('       ctf_integer(' + t + ', ' + n + ', ' + n + ')')
                elif ('double' in t) or ('float' in t):
                    out('       ctf_float(' + t + ', ' + n + ', ' + n + ')')
                elif ('void *' in t) or ('void*' in t):
                    out('       ctf_integer_hex(unsigned long, ' + n + ', ' + n + ')')

            out('   )',
                ')',
                '')

        else:
            out('TRACEPOINT_EVENT(',
                '   qemu,',
                '   %(name)s,',
                '   TP_ARGS(void),',
                '   TP_FIELDS()',
                ')',
                '',
                name=e.name)

    out('#endif /* TRACE_%s_GENERATED_UST_H */' % group.upper(),
        '',
        '/* This part must be outside ifdef protection */',
        '#include <lttng/tracepoint-event.h>')
