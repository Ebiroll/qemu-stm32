"""
QAPI introspection generator

Copyright (C) 2015-2018 Red Hat, Inc.

Authors:
 Markus Armbruster <armbru@redhat.com>

This work is licensed under the terms of the GNU GPL, version 2.
See the COPYING file in the top-level directory.
"""

from typing import Optional

from .common import (
    c_name,
    gen_endif,
    gen_if,
    mcgen,
)
from .gen import QAPISchemaMonolithicCVisitor
from .schema import (
    QAPISchemaArrayType,
    QAPISchemaBuiltinType,
    QAPISchemaType,
)


def _make_tree(obj, ifcond, comment=None):
    extra = {
        'if': ifcond,
        'comment': comment
    }
    return (obj, extra)


def _tree_to_qlit(obj, level=0, dict_value=False):

    def indent(level):
        return level * 4 * ' '

    if isinstance(obj, tuple):
        ifobj, extra = obj
        ifcond = extra.get('if')
        comment = extra.get('comment')

        # NB: _tree_to_qlit is called recursively on the values of a
        # key:value pair; those values can't be decorated with
        # comments or conditionals.
        msg = "dict values cannot have attached comments or if-conditionals."
        assert not dict_value, msg

        ret = ''
        if comment:
            ret += indent(level) + '/* %s */\n' % comment
        if ifcond:
            ret += gen_if(ifcond)
        ret += _tree_to_qlit(ifobj, level)
        if ifcond:
            ret += '\n' + gen_endif(ifcond)
        return ret

    ret = ''
    if not dict_value:
        ret += indent(level)
    if obj is None:
        ret += 'QLIT_QNULL'
    elif isinstance(obj, str):
        ret += 'QLIT_QSTR(' + to_c_string(obj) + ')'
    elif isinstance(obj, list):
        elts = [_tree_to_qlit(elt, level + 1).strip('\n')
                for elt in obj]
        elts.append(indent(level + 1) + "{}")
        ret += 'QLIT_QLIST(((QLitObject[]) {\n'
        ret += '\n'.join(elts) + '\n'
        ret += indent(level) + '}))'
    elif isinstance(obj, dict):
        elts = []
        for key, value in sorted(obj.items()):
            elts.append(indent(level + 1) + '{ %s, %s }' %
                        (to_c_string(key),
                         _tree_to_qlit(value, level + 1, True)))
        elts.append(indent(level + 1) + '{}')
        ret += 'QLIT_QDICT(((QLitDictEntry[]) {\n'
        ret += ',\n'.join(elts) + '\n'
        ret += indent(level) + '}))'
    elif isinstance(obj, bool):
        ret += 'QLIT_QBOOL(%s)' % ('true' if obj else 'false')
    else:
        assert False                # not implemented
    if level > 0:
        ret += ','
    return ret


def to_c_string(string):
    return '"' + string.replace('\\', r'\\').replace('"', r'\"') + '"'


class QAPISchemaGenIntrospectVisitor(QAPISchemaMonolithicCVisitor):

    def __init__(self, prefix, unmask):
        super().__init__(
            prefix, 'qapi-introspect',
            ' * QAPI/QMP schema introspection', __doc__)
        self._unmask = unmask
        self._schema = None
        self._trees = []
        self._used_types = []
        self._name_map = {}
        self._genc.add(mcgen('''
#include "qemu/osdep.h"
#include "%(prefix)sqapi-introspect.h"

''',
                             prefix=prefix))

    def visit_begin(self, schema):
        self._schema = schema

    def visit_end(self):
        # visit the types that are actually used
        for typ in self._used_types:
            typ.visit(self)
        # generate C
        name = c_name(self._prefix, protect=False) + 'qmp_schema_qlit'
        self._genh.add(mcgen('''
#include "qapi/qmp/qlit.h"

extern const QLitObject %(c_name)s;
''',
                             c_name=c_name(name)))
        self._genc.add(mcgen('''
const QLitObject %(c_name)s = %(c_string)s;
''',
                             c_name=c_name(name),
                             c_string=_tree_to_qlit(self._trees)))
        self._schema = None
        self._trees = []
        self._used_types = []
        self._name_map = {}

    def visit_needed(self, entity):
        # Ignore types on first pass; visit_end() will pick up used types
        return not isinstance(entity, QAPISchemaType)

    def _name(self, name):
        if self._unmask:
            return name
        if name not in self._name_map:
            self._name_map[name] = '%d' % len(self._name_map)
        return self._name_map[name]

    def _use_type(self, typ):
        assert self._schema is not None

        # Map the various integer types to plain int
        if typ.json_type() == 'int':
            typ = self._schema.lookup_type('int')
        elif (isinstance(typ, QAPISchemaArrayType) and
              typ.element_type.json_type() == 'int'):
            typ = self._schema.lookup_type('intList')
        # Add type to work queue if new
        if typ not in self._used_types:
            self._used_types.append(typ)
        # Clients should examine commands and events, not types.  Hide
        # type names as integers to reduce the temptation.  Also, it
        # saves a few characters on the wire.
        if isinstance(typ, QAPISchemaBuiltinType):
            return typ.name
        if isinstance(typ, QAPISchemaArrayType):
            return '[' + self._use_type(typ.element_type) + ']'
        return self._name(typ.name)

    @staticmethod
    def _gen_features(features):
        return [_make_tree(f.name, f.ifcond) for f in features]

    def _gen_tree(self, name, mtype, obj, ifcond, features):
        comment: Optional[str] = None
        if mtype not in ('command', 'event', 'builtin', 'array'):
            if not self._unmask:
                # Output a comment to make it easy to map masked names
                # back to the source when reading the generated output.
                comment = f'"{self._name(name)}" = {name}'
            name = self._name(name)
        obj['name'] = name
        obj['meta-type'] = mtype
        if features:
            obj['features'] = self._gen_features(features)
        self._trees.append(_make_tree(obj, ifcond, comment))

    def _gen_member(self, member):
        obj = {'name': member.name, 'type': self._use_type(member.type)}
        if member.optional:
            obj['default'] = None
        if member.features:
            obj['features'] = self._gen_features(member.features)
        return _make_tree(obj, member.ifcond)

    def _gen_variants(self, tag_name, variants):
        return {'tag': tag_name,
                'variants': [self._gen_variant(v) for v in variants]}

    def _gen_variant(self, variant):
        obj = {'case': variant.name, 'type': self._use_type(variant.type)}
        return _make_tree(obj, variant.ifcond)

    def visit_builtin_type(self, name, info, json_type):
        self._gen_tree(name, 'builtin', {'json-type': json_type}, [], None)

    def visit_enum_type(self, name, info, ifcond, features, members, prefix):
        self._gen_tree(name, 'enum',
                       {'values': [_make_tree(m.name, m.ifcond, None)
                                   for m in members]},
                       ifcond, features)

    def visit_array_type(self, name, info, ifcond, element_type):
        element = self._use_type(element_type)
        self._gen_tree('[' + element + ']', 'array', {'element-type': element},
                       ifcond, None)

    def visit_object_type_flat(self, name, info, ifcond, features,
                               members, variants):
        obj = {'members': [self._gen_member(m) for m in members]}
        if variants:
            obj.update(self._gen_variants(variants.tag_member.name,
                                          variants.variants))

        self._gen_tree(name, 'object', obj, ifcond, features)

    def visit_alternate_type(self, name, info, ifcond, features, variants):
        self._gen_tree(name, 'alternate',
                       {'members': [
                           _make_tree({'type': self._use_type(m.type)},
                                      m.ifcond, None)
                           for m in variants.variants]},
                       ifcond, features)

    def visit_command(self, name, info, ifcond, features,
                      arg_type, ret_type, gen, success_response, boxed,
                      allow_oob, allow_preconfig, coroutine):
        assert self._schema is not None

        arg_type = arg_type or self._schema.the_empty_object_type
        ret_type = ret_type or self._schema.the_empty_object_type
        obj = {'arg-type': self._use_type(arg_type),
               'ret-type': self._use_type(ret_type)}
        if allow_oob:
            obj['allow-oob'] = allow_oob
        self._gen_tree(name, 'command', obj, ifcond, features)

    def visit_event(self, name, info, ifcond, features, arg_type, boxed):
        assert self._schema is not None
        arg_type = arg_type or self._schema.the_empty_object_type
        self._gen_tree(name, 'event', {'arg-type': self._use_type(arg_type)},
                       ifcond, features)


def gen_introspect(schema, output_dir, prefix, opt_unmask):
    vis = QAPISchemaGenIntrospectVisitor(prefix, opt_unmask)
    schema.visit(vis)
    vis.write(output_dir)
