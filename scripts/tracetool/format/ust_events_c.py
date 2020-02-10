# -*- coding: utf-8 -*-

"""
trace/generated-ust.c
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

    out('/* This file is autogenerated by tracetool, do not edit. */',
        '',
        '#include "qemu/osdep.h"',
        '',
        '#define TRACEPOINT_DEFINE',
        '#define TRACEPOINT_CREATE_PROBES',
        '',
        '/* If gcc version 4.7 or older is used, LTTng ust gives a warning when compiling with',
        '   -Wredundant-decls.',
        ' */',
        '#pragma GCC diagnostic ignored "-Wredundant-decls"',
        '',
        '#include "trace-ust-all.h"')
