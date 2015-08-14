#!/usr/bin/python

# GDB debugging support
#
# Copyright 2012 Red Hat, Inc. and/or its affiliates
#
# Authors:
#  Avi Kivity <avi@redhat.com>
#
# This work is licensed under the terms of the GNU GPL, version 2.  See
# the COPYING file in the top-level directory.
#
# Contributions after 2012-01-13 are licensed under the terms of the
# GNU GPL, version 2 or (at your option) any later version.


import gdb

import os, sys

# Annoyingly, gdb doesn't put the directory of scripts onto the
# module search path. Do it manually.

sys.path.append(os.path.dirname(__file__))

from qemugdb import mtree, coroutine

class QemuCommand(gdb.Command):
    '''Prefix for QEMU debug support commands'''
    def __init__(self):
        gdb.Command.__init__(self, 'qemu', gdb.COMMAND_DATA,
                             gdb.COMPLETE_NONE, True)

QemuCommand()
coroutine.CoroutineCommand()
mtree.MtreeCommand()
