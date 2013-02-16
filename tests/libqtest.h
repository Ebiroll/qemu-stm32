/*
 * QTest
 *
 * Copyright IBM, Corp. 2012
 * Copyright Red Hat, Inc. 2012
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *  Paolo Bonzini     <pbonzini@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */
#ifndef LIBQTEST_H
#define LIBQTEST_H

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <sys/types.h>

typedef struct QTestState QTestState;

extern QTestState *global_qtest;

/**
 * qtest_init:
 * @extra_args: other arguments to pass to QEMU.
 *
 * Returns: #QTestState instance.
 */
QTestState *qtest_init(const char *extra_args);

/**
 * qtest_quit:
 * @s: #QTestState instance to operate on.
 *
 * Shut down the QEMU process associated to @s.
 */
void qtest_quit(QTestState *s);

/**
 * qtest_qmp:
 * @s: #QTestState instance to operate on.
 * @fmt...: QMP message to send to qemu
 *
 * Sends a QMP message to QEMU
 */
void qtest_qmp(QTestState *s, const char *fmt, ...);

/**
 * qtest_qmpv:
 * @s: #QTestState instance to operate on.
 * @fmt: QMP message to send to QEMU
 * @ap: QMP message arguments
 *
 * Sends a QMP message to QEMU.
 */
void qtest_qmpv(QTestState *s, const char *fmt, va_list ap);

/**
 * qtest_get_irq:
 * @s: #QTestState instance to operate on.
 * @num: Interrupt to observe.
 *
 * Returns: The level of the @num interrupt.
 */
bool qtest_get_irq(QTestState *s, int num);

/**
 * qtest_irq_intercept_in:
 * @s: #QTestState instance to operate on.
 * @string: QOM path of a device.
 *
 * Associate qtest irqs with the GPIO-in pins of the device
 * whose path is specified by @string.
 */
void qtest_irq_intercept_in(QTestState *s, const char *string);

/**
 * qtest_irq_intercept_out:
 * @s: #QTestState instance to operate on.
 * @string: QOM path of a device.
 *
 * Associate qtest irqs with the GPIO-out pins of the device
 * whose path is specified by @string.
 */
void qtest_irq_intercept_out(QTestState *s, const char *string);

/**
 * qtest_outb:
 * @s: #QTestState instance to operate on.
 * @addr: I/O port to write to.
 * @value: Value being written.
 *
 * Write an 8-bit value to an I/O port.
 */
void qtest_outb(QTestState *s, uint16_t addr, uint8_t value);

/**
 * qtest_outw:
 * @s: #QTestState instance to operate on.
 * @addr: I/O port to write to.
 * @value: Value being written.
 *
 * Write a 16-bit value to an I/O port.
 */
void qtest_outw(QTestState *s, uint16_t addr, uint16_t value);

/**
 * qtest_outl:
 * @s: #QTestState instance to operate on.
 * @addr: I/O port to write to.
 * @value: Value being written.
 *
 * Write a 32-bit value to an I/O port.
 */
void qtest_outl(QTestState *s, uint16_t addr, uint32_t value);

/**
 * qtest_inb:
 * @s: #QTestState instance to operate on.
 * @addr: I/O port to read from.
 *
 * Returns an 8-bit value from an I/O port.
 */
uint8_t qtest_inb(QTestState *s, uint16_t addr);

/**
 * qtest_inw:
 * @s: #QTestState instance to operate on.
 * @addr: I/O port to read from.
 *
 * Returns a 16-bit value from an I/O port.
 */
uint16_t qtest_inw(QTestState *s, uint16_t addr);

/**
 * qtest_inl:
 * @s: #QTestState instance to operate on.
 * @addr: I/O port to read from.
 *
 * Returns a 32-bit value from an I/O port.
 */
uint32_t qtest_inl(QTestState *s, uint16_t addr);

/**
 * qtest_memread:
 * @s: #QTestState instance to operate on.
 * @addr: Guest address to read from.
 * @data: Pointer to where memory contents will be stored.
 * @size: Number of bytes to read.
 *
 * Read guest memory into a buffer.
 */
void qtest_memread(QTestState *s, uint64_t addr, void *data, size_t size);

/**
 * qtest_memwrite:
 * @s: #QTestState instance to operate on.
 * @addr: Guest address to write to.
 * @data: Pointer to the bytes that will be written to guest memory.
 * @size: Number of bytes to write.
 *
 * Write a buffer to guest memory.
 */
void qtest_memwrite(QTestState *s, uint64_t addr, const void *data, size_t size);

/**
 * qtest_clock_step_next:
 * @s: #QTestState instance to operate on.
 *
 * Advance the vm_clock to the next deadline.
 *
 * Returns: The current value of the vm_clock in nanoseconds.
 */
int64_t qtest_clock_step_next(QTestState *s);

/**
 * qtest_clock_step:
 * @s: QTestState instance to operate on.
 * @step: Number of nanoseconds to advance the clock by.
 *
 * Advance the vm_clock by @step nanoseconds.
 *
 * Returns: The current value of the vm_clock in nanoseconds.
 */
int64_t qtest_clock_step(QTestState *s, int64_t step);

/**
 * qtest_clock_set:
 * @s: QTestState instance to operate on.
 * @val: Nanoseconds value to advance the clock to.
 *
 * Advance the vm_clock to @val nanoseconds since the VM was launched.
 *
 * Returns: The current value of the vm_clock in nanoseconds.
 */
int64_t qtest_clock_set(QTestState *s, int64_t val);

/**
 * qtest_get_arch:
 *
 * Returns: The architecture for the QEMU executable under test.
 */
const char *qtest_get_arch(void);

/**
 * qtest_add_func:
 * @str: Test case path.
 * @fn: Test case function
 *
 * Add a GTester testcase with the given name and function.
 * The path is prefixed with the architecture under test, as
 * returned by qtest_get_arch().
 */
void qtest_add_func(const char *str, void (*fn));

/**
 * qtest_start:
 * @args: other arguments to pass to QEMU
 *
 * Start QEMU and assign the resulting #QTestState to a global variable.
 * The global variable is used by "shortcut" functions documented below.
 *
 * Returns: #QTestState instance.
 */
static inline QTestState *qtest_start(const char *args)
{
    global_qtest = qtest_init(args);
    return global_qtest;
}

/**
 * qmp:
 * @fmt...: QMP message to send to qemu
 *
 * Sends a QMP message to QEMU
 */
static inline void qmp(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    qtest_qmpv(global_qtest, fmt, ap);
    va_end(ap);
}

/**
 * get_irq:
 * @num: Interrupt to observe.
 *
 * Returns: The level of the @num interrupt.
 */
static inline bool get_irq(int num)
{
    return qtest_get_irq(global_qtest, num);
}

/**
 * irq_intercept_in:
 * @string: QOM path of a device.
 *
 * Associate qtest irqs with the GPIO-in pins of the device
 * whose path is specified by @string.
 */
static inline void irq_intercept_in(const char *string)
{
    qtest_irq_intercept_in(global_qtest, string);
}

/**
 * qtest_irq_intercept_out:
 * @string: QOM path of a device.
 *
 * Associate qtest irqs with the GPIO-out pins of the device
 * whose path is specified by @string.
 */
static inline void irq_intercept_out(const char *string)
{
    qtest_irq_intercept_out(global_qtest, string);
}

/**
 * outb:
 * @addr: I/O port to write to.
 * @value: Value being written.
 *
 * Write an 8-bit value to an I/O port.
 */
static inline void outb(uint16_t addr, uint8_t value)
{
    qtest_outb(global_qtest, addr, value);
}

/**
 * outw:
 * @addr: I/O port to write to.
 * @value: Value being written.
 *
 * Write a 16-bit value to an I/O port.
 */
static inline void outw(uint16_t addr, uint16_t value)
{
    qtest_outw(global_qtest, addr, value);
}

/**
 * outl:
 * @addr: I/O port to write to.
 * @value: Value being written.
 *
 * Write a 32-bit value to an I/O port.
 */
static inline void outl(uint16_t addr, uint32_t value)
{
    qtest_outl(global_qtest, addr, value);
}

/**
 * inb:
 * @addr: I/O port to read from.
 *
 * Reads an 8-bit value from an I/O port.
 *
 * Returns: Value read.
 */
static inline uint8_t inb(uint16_t addr)
{
    return qtest_inb(global_qtest, addr);
}

/**
 * inw:
 * @addr: I/O port to read from.
 *
 * Reads a 16-bit value from an I/O port.
 *
 * Returns: Value read.
 */
static inline uint16_t inw(uint16_t addr)
{
    return qtest_inw(global_qtest, addr);
}

/**
 * inl:
 * @addr: I/O port to read from.
 *
 * Reads a 32-bit value from an I/O port.
 *
 * Returns: Value read.
 */
static inline uint32_t inl(uint16_t addr)
{
    return qtest_inl(global_qtest, addr);
}

/**
 * memread:
 * @addr: Guest address to read from.
 * @data: Pointer to where memory contents will be stored.
 * @size: Number of bytes to read.
 *
 * Read guest memory into a buffer.
 */
static inline void memread(uint64_t addr, void *data, size_t size)
{
    qtest_memread(global_qtest, addr, data, size);
}

/**
 * memwrite:
 * @addr: Guest address to write to.
 * @data: Pointer to the bytes that will be written to guest memory.
 * @size: Number of bytes to write.
 *
 * Write a buffer to guest memory.
 */
static inline void memwrite(uint64_t addr, const void *data, size_t size)
{
    qtest_memwrite(global_qtest, addr, data, size);
}

/**
 * clock_step_next:
 *
 * Advance the vm_clock to the next deadline.
 *
 * Returns: The current value of the vm_clock in nanoseconds.
 */
static inline int64_t clock_step_next(void)
{
    return qtest_clock_step_next(global_qtest);
}

/**
 * clock_step:
 * @step: Number of nanoseconds to advance the clock by.
 *
 * Advance the vm_clock by @step nanoseconds.
 *
 * Returns: The current value of the vm_clock in nanoseconds.
 */
static inline int64_t clock_step(int64_t step)
{
    return qtest_clock_step(global_qtest, step);
}

/**
 * clock_set:
 * @val: Nanoseconds value to advance the clock to.
 *
 * Advance the vm_clock to @val nanoseconds since the VM was launched.
 *
 * Returns: The current value of the vm_clock in nanoseconds.
 */
static inline int64_t clock_set(int64_t val)
{
    return qtest_clock_set(global_qtest, val);
}

#endif
