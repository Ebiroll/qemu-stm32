/*
 *  Test code for qdev global-properties handling
 *
 *  Copyright (c) 2012 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <glib.h>
#include <stdint.h>

#include "hw/qdev.h"
#include "qom/object.h"
#include "qapi/visitor.h"


#define TYPE_STATIC_PROPS "static_prop_type"
#define STATIC_TYPE(obj) \
    OBJECT_CHECK(MyType, (obj), TYPE_STATIC_PROPS)

#define PROP_DEFAULT 100

typedef struct MyType {
    DeviceState parent_obj;

    uint32_t prop1;
    uint32_t prop2;
} MyType;

static Property static_props[] = {
    DEFINE_PROP_UINT32("prop1", MyType, prop1, PROP_DEFAULT),
    DEFINE_PROP_UINT32("prop2", MyType, prop2, PROP_DEFAULT),
    DEFINE_PROP_END_OF_LIST()
};

static void static_prop_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = NULL;
    dc->props = static_props;
}

static const TypeInfo static_prop_type = {
    .name = TYPE_STATIC_PROPS,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(MyType),
    .class_init = static_prop_class_init,
};

/* Test simple static property setting to default value */
static void test_static_prop_subprocess(void)
{
    MyType *mt;

    mt = STATIC_TYPE(object_new(TYPE_STATIC_PROPS));
    qdev_init_nofail(DEVICE(mt));

    g_assert_cmpuint(mt->prop1, ==, PROP_DEFAULT);
}

static void test_static_prop(void)
{
    g_test_trap_subprocess("/qdev/properties/static/default/subprocess", 0, 0);
    g_test_trap_assert_passed();
    g_test_trap_assert_stderr("");
    g_test_trap_assert_stdout("");
}

/* Test setting of static property using global properties */
static void test_static_globalprop_subprocess(void)
{
    MyType *mt;
    static GlobalProperty props[] = {
        { TYPE_STATIC_PROPS, "prop1", "200" },
        {}
    };

    qdev_prop_register_global_list(props);

    mt = STATIC_TYPE(object_new(TYPE_STATIC_PROPS));
    qdev_init_nofail(DEVICE(mt));

    g_assert_cmpuint(mt->prop1, ==, 200);
    g_assert_cmpuint(mt->prop2, ==, PROP_DEFAULT);
}

static void test_static_globalprop(void)
{
    g_test_trap_subprocess("/qdev/properties/static/global/subprocess", 0, 0);
    g_test_trap_assert_passed();
    g_test_trap_assert_stderr("");
    g_test_trap_assert_stdout("");
}

#define TYPE_DYNAMIC_PROPS "dynamic-prop-type"
#define DYNAMIC_TYPE(obj) \
    OBJECT_CHECK(MyType, (obj), TYPE_DYNAMIC_PROPS)

static void prop1_accessor(Object *obj,
                           Visitor *v,
                           void *opaque,
                           const char *name,
                           Error **errp)
{
    MyType *mt = DYNAMIC_TYPE(obj);

    visit_type_uint32(v, &mt->prop1, name, errp);
}

static void prop2_accessor(Object *obj,
                           Visitor *v,
                           void *opaque,
                           const char *name,
                           Error **errp)
{
    MyType *mt = DYNAMIC_TYPE(obj);

    visit_type_uint32(v, &mt->prop2, name, errp);
}

static void dynamic_instance_init(Object *obj)
{
    object_property_add(obj, "prop1", "uint32", prop1_accessor, prop1_accessor,
                        NULL, NULL, NULL);
    object_property_add(obj, "prop2", "uint32", prop2_accessor, prop2_accessor,
                        NULL, NULL, NULL);
}

static void dynamic_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = NULL;
}


static const TypeInfo dynamic_prop_type = {
    .name = TYPE_DYNAMIC_PROPS,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(MyType),
    .instance_init = dynamic_instance_init,
    .class_init = dynamic_class_init,
};

/* Test setting of dynamic properties using global properties */
static void test_dynamic_globalprop_subprocess(void)
{
    MyType *mt;
    static GlobalProperty props[] = {
        { TYPE_DYNAMIC_PROPS, "prop1", "101" },
        { TYPE_DYNAMIC_PROPS, "prop2", "102" },
        { TYPE_DYNAMIC_PROPS"-bad", "prop3", "103", true },
        {}
    };
    int all_used;

    qdev_prop_register_global_list(props);

    mt = DYNAMIC_TYPE(object_new(TYPE_DYNAMIC_PROPS));
    qdev_init_nofail(DEVICE(mt));

    g_assert_cmpuint(mt->prop1, ==, 101);
    g_assert_cmpuint(mt->prop2, ==, 102);
    all_used = qdev_prop_check_global();
    g_assert_cmpuint(all_used, ==, 1);
}

static void test_dynamic_globalprop(void)
{
    g_test_trap_subprocess("/qdev/properties/dynamic/global/subprocess", 0, 0);
    g_test_trap_assert_passed();
    g_test_trap_assert_stderr_unmatched("*prop1*");
    g_test_trap_assert_stderr_unmatched("*prop2*");
    g_test_trap_assert_stderr("*Warning: \"-global dynamic-prop-type-bad.prop3=103\" not used\n*");
    g_test_trap_assert_stdout("");
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    module_call_init(MODULE_INIT_QOM);
    type_register_static(&static_prop_type);
    type_register_static(&dynamic_prop_type);

    g_test_add_func("/qdev/properties/static/default/subprocess",
                    test_static_prop_subprocess);
    g_test_add_func("/qdev/properties/static/default",
                    test_static_prop);

    g_test_add_func("/qdev/properties/static/global/subprocess",
                    test_static_globalprop_subprocess);
    g_test_add_func("/qdev/properties/static/global",
                    test_static_globalprop);

    g_test_add_func("/qdev/properties/dynamic/global/subprocess",
                    test_dynamic_globalprop_subprocess);
    g_test_add_func("/qdev/properties/dynamic/global",
                    test_dynamic_globalprop);

    g_test_run();

    return 0;
}
