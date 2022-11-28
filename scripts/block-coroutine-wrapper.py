#! /usr/bin/env python3
"""Generate coroutine wrappers for block subsystem.

The program parses one or several concatenated c files from stdin,
searches for functions with the 'co_wrapper' specifier
and generates corresponding wrappers on stdout.

Usage: block-coroutine-wrapper.py generated-file.c FILE.[ch]...

Copyright (c) 2020 Virtuozzo International GmbH.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import sys
import re
from typing import Iterator


def gen_header():
    copyright = re.sub('^.*Copyright', 'Copyright', __doc__, flags=re.DOTALL)
    copyright = re.sub('^(?=.)', ' * ', copyright.strip(), flags=re.MULTILINE)
    copyright = re.sub('^$', ' *', copyright, flags=re.MULTILINE)
    return f"""\
/*
 * File is generated by scripts/block-coroutine-wrapper.py
 *
{copyright}
 */

#include "qemu/osdep.h"
#include "block/coroutines.h"
#include "block/block-gen.h"
#include "block/block_int.h"\
"""


class ParamDecl:
    param_re = re.compile(r'(?P<decl>'
                          r'(?P<type>.*[ *])'
                          r'(?P<name>[a-z][a-z0-9_]*)'
                          r')')

    def __init__(self, param_decl: str) -> None:
        m = self.param_re.match(param_decl.strip())
        if m is None:
            raise ValueError(f'Wrong parameter declaration: "{param_decl}"')
        self.decl = m.group('decl')
        self.type = m.group('type')
        self.name = m.group('name')


class FuncDecl:
    def __init__(self, return_type: str, name: str, args: str,
                 variant: str) -> None:
        self.return_type = return_type.strip()
        self.name = name.strip()
        self.struct_name = snake_to_camel(self.name)
        self.args = [ParamDecl(arg.strip()) for arg in args.split(',')]
        self.create_only_co = 'mixed' not in variant

        subsystem, subname = self.name.split('_', 1)
        self.co_name = f'{subsystem}_co_{subname}'

        t = self.args[0].type
        if t == 'BlockDriverState *':
            ctx = 'bdrv_get_aio_context(bs)'
        elif t == 'BdrvChild *':
            ctx = 'bdrv_get_aio_context(child->bs)'
        elif t == 'BlockBackend *':
            ctx = 'blk_get_aio_context(blk)'
        else:
            ctx = 'qemu_get_aio_context()'
        self.ctx = ctx

    def gen_list(self, format: str) -> str:
        return ', '.join(format.format_map(arg.__dict__) for arg in self.args)

    def gen_block(self, format: str) -> str:
        return '\n'.join(format.format_map(arg.__dict__) for arg in self.args)


# Match wrappers declared with a co_wrapper mark
func_decl_re = re.compile(r'^int\s*co_wrapper'
                          r'(?P<variant>(_[a-z][a-z0-9_]*)?)\s*'
                          r'(?P<wrapper_name>[a-z][a-z0-9_]*)'
                          r'\((?P<args>[^)]*)\);$', re.MULTILINE)


def func_decl_iter(text: str) -> Iterator:
    for m in func_decl_re.finditer(text):
        yield FuncDecl(return_type='int',
                       name=m.group('wrapper_name'),
                       args=m.group('args'),
                       variant=m.group('variant'))


def snake_to_camel(func_name: str) -> str:
    """
    Convert underscore names like 'some_function_name' to camel-case like
    'SomeFunctionName'
    """
    words = func_name.split('_')
    words = [w[0].upper() + w[1:] for w in words]
    return ''.join(words)


def create_mixed_wrapper(func: FuncDecl) -> str:
    """
    Checks if we are already in coroutine
    """
    name = func.co_name
    struct_name = func.struct_name
    return f"""\
int {func.name}({ func.gen_list('{decl}') })
{{
    if (qemu_in_coroutine()) {{
        return {name}({ func.gen_list('{name}') });
    }} else {{
        {struct_name} s = {{
            .poll_state.ctx = {func.ctx},
            .poll_state.in_progress = true,

{ func.gen_block('            .{name} = {name},') }
        }};

        s.poll_state.co = qemu_coroutine_create({name}_entry, &s);

        return bdrv_poll_co(&s.poll_state);
    }}
}}"""


def create_co_wrapper(func: FuncDecl) -> str:
    """
    Assumes we are not in coroutine, and creates one
    """
    name = func.co_name
    struct_name = func.struct_name
    return f"""\
int {func.name}({ func.gen_list('{decl}') })
{{
    {struct_name} s = {{
        .poll_state.ctx = {func.ctx},
        .poll_state.in_progress = true,

{ func.gen_block('        .{name} = {name},') }
    }};
    assert(!qemu_in_coroutine());

    s.poll_state.co = qemu_coroutine_create({name}_entry, &s);

    return bdrv_poll_co(&s.poll_state);
}}"""


def gen_wrapper(func: FuncDecl) -> str:
    assert not '_co_' in func.name
    assert func.return_type == 'int'

    name = func.co_name
    struct_name = func.struct_name

    creation_function = create_mixed_wrapper
    if func.create_only_co:
        creation_function = create_co_wrapper

    return f"""\
/*
 * Wrappers for {name}
 */

typedef struct {struct_name} {{
    BdrvPollCo poll_state;
{ func.gen_block('    {decl};') }
}} {struct_name};

static void coroutine_fn {name}_entry(void *opaque)
{{
    {struct_name} *s = opaque;

    s->poll_state.ret = {name}({ func.gen_list('s->{name}') });
    s->poll_state.in_progress = false;

    aio_wait_kick();
}}

{creation_function(func)}"""


def gen_wrappers(input_code: str) -> str:
    res = ''
    for func in func_decl_iter(input_code):
        res += '\n\n\n'
        res += gen_wrapper(func)

    return res


if __name__ == '__main__':
    if len(sys.argv) < 3:
        exit(f'Usage: {sys.argv[0]} OUT_FILE.c IN_FILE.[ch]...')

    with open(sys.argv[1], 'w', encoding='utf-8') as f_out:
        f_out.write(gen_header())
        for fname in sys.argv[2:]:
            with open(fname, encoding='utf-8') as f_in:
                f_out.write(gen_wrappers(f_in.read()))
                f_out.write('\n')
