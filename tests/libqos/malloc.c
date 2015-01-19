/*
 * libqos malloc support
 *
 * Copyright (c) 2014
 *
 * Author:
 *  John Snow <jsnow@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "libqos/malloc.h"
#include "qemu-common.h"
#include <stdio.h>
#include <inttypes.h>
#include <glib.h>

static void mlist_delete(MemList *list, MemBlock *node)
{
    g_assert(list && node);
    QTAILQ_REMOVE(list, node, MLIST_ENTNAME);
    g_free(node);
}

static MemBlock *mlist_find_key(MemList *head, uint64_t addr)
{
    MemBlock *node;
    QTAILQ_FOREACH(node, head, MLIST_ENTNAME) {
        if (node->addr == addr) {
            return node;
        }
    }
    return NULL;
}

static MemBlock *mlist_find_space(MemList *head, uint64_t size)
{
    MemBlock *node;

    QTAILQ_FOREACH(node, head, MLIST_ENTNAME) {
        if (node->size >= size) {
            return node;
        }
    }
    return NULL;
}

static MemBlock *mlist_sort_insert(MemList *head, MemBlock *insr)
{
    MemBlock *node;
    g_assert(head && insr);

    QTAILQ_FOREACH(node, head, MLIST_ENTNAME) {
        if (insr->addr < node->addr) {
            QTAILQ_INSERT_BEFORE(node, insr, MLIST_ENTNAME);
            return insr;
        }
    }

    QTAILQ_INSERT_TAIL(head, insr, MLIST_ENTNAME);
    return insr;
}

static inline uint64_t mlist_boundary(MemBlock *node)
{
    return node->size + node->addr;
}

static MemBlock *mlist_join(MemList *head, MemBlock *left, MemBlock *right)
{
    g_assert(head && left && right);

    left->size += right->size;
    mlist_delete(head, right);
    return left;
}

static void mlist_coalesce(MemList *head, MemBlock *node)
{
    g_assert(node);
    MemBlock *left;
    MemBlock *right;
    char merge;

    do {
        merge = 0;
        left = QTAILQ_PREV(node, MemList, MLIST_ENTNAME);
        right = QTAILQ_NEXT(node, MLIST_ENTNAME);

        /* clowns to the left of me */
        if (left && mlist_boundary(left) == node->addr) {
            node = mlist_join(head, left, node);
            merge = 1;
        }

        /* jokers to the right */
        if (right && mlist_boundary(node) == right->addr) {
            node = mlist_join(head, node, right);
            merge = 1;
        }

    } while (merge);
}

static uint64_t mlist_fulfill(QGuestAllocator *s, MemBlock *freenode,
                                                                uint64_t size)
{
    uint64_t addr;
    MemBlock *usednode;

    g_assert(freenode);
    g_assert_cmpint(freenode->size, >=, size);

    addr = freenode->addr;
    if (freenode->size == size) {
        /* re-use this freenode as our used node */
        QTAILQ_REMOVE(&s->free, freenode, MLIST_ENTNAME);
        usednode = freenode;
    } else {
        /* adjust the free node and create a new used node */
        freenode->addr += size;
        freenode->size -= size;
        usednode = mlist_new(addr, size);
    }

    mlist_sort_insert(&s->used, usednode);
    return addr;
}

/* To assert the correctness of the list.
 * Used only if ALLOC_PARANOID is set. */
static void mlist_check(QGuestAllocator *s)
{
    MemBlock *node;
    uint64_t addr = s->start > 0 ? s->start - 1 : 0;
    uint64_t next = s->start;

    QTAILQ_FOREACH(node, &s->free, MLIST_ENTNAME) {
        g_assert_cmpint(node->addr, >, addr);
        g_assert_cmpint(node->addr, >=, next);
        addr = node->addr;
        next = node->addr + node->size;
    }

    addr = s->start > 0 ? s->start - 1 : 0;
    next = s->start;
    QTAILQ_FOREACH(node, &s->used, MLIST_ENTNAME) {
        g_assert_cmpint(node->addr, >, addr);
        g_assert_cmpint(node->addr, >=, next);
        addr = node->addr;
        next = node->addr + node->size;
    }
}

static uint64_t mlist_alloc(QGuestAllocator *s, uint64_t size)
{
    MemBlock *node;

    node = mlist_find_space(&s->free, size);
    if (!node) {
        fprintf(stderr, "Out of guest memory.\n");
        g_assert_not_reached();
    }
    return mlist_fulfill(s, node, size);
}

static void mlist_free(QGuestAllocator *s, uint64_t addr)
{
    MemBlock *node;

    if (addr == 0) {
        return;
    }

    node = mlist_find_key(&s->used, addr);
    if (!node) {
        fprintf(stderr, "Error: no record found for an allocation at "
                "0x%016" PRIx64 ".\n",
                addr);
        g_assert_not_reached();
    }

    /* Rip it out of the used list and re-insert back into the free list. */
    QTAILQ_REMOVE(&s->used, node, MLIST_ENTNAME);
    mlist_sort_insert(&s->free, node);
    mlist_coalesce(&s->free, node);
}

MemBlock *mlist_new(uint64_t addr, uint64_t size)
{
    MemBlock *block;

    if (!size) {
        return NULL;
    }
    block = g_malloc0(sizeof(MemBlock));

    block->addr = addr;
    block->size = size;

    return block;
}

/*
 * Mostly for valgrind happiness, but it does offer
 * a chokepoint for debugging guest memory leaks, too.
 */
void alloc_uninit(QGuestAllocator *allocator)
{
    MemBlock *node;
    MemBlock *tmp;
    QAllocOpts mask;

    /* Check for guest leaks, and destroy the list. */
    QTAILQ_FOREACH_SAFE(node, &allocator->used, MLIST_ENTNAME, tmp) {
        if (allocator->opts & (ALLOC_LEAK_WARN | ALLOC_LEAK_ASSERT)) {
            fprintf(stderr, "guest malloc leak @ 0x%016" PRIx64 "; "
                    "size 0x%016" PRIx64 ".\n",
                    node->addr, node->size);
        }
        if (allocator->opts & (ALLOC_LEAK_ASSERT)) {
            g_assert_not_reached();
        }
        g_free(node);
    }

    /* If we have previously asserted that there are no leaks, then there
     * should be only one node here with a specific address and size. */
    mask = ALLOC_LEAK_ASSERT | ALLOC_PARANOID;
    QTAILQ_FOREACH_SAFE(node, &allocator->free, MLIST_ENTNAME, tmp) {
        if ((allocator->opts & mask) == mask) {
            if ((node->addr != allocator->start) ||
                (node->size != allocator->end - allocator->start)) {
                fprintf(stderr, "Free list is corrupted.\n");
                g_assert_not_reached();
            }
        }

        g_free(node);
    }

    g_free(allocator);
}

uint64_t guest_alloc(QGuestAllocator *allocator, size_t size)
{
    uint64_t rsize = size;
    uint64_t naddr;

    rsize += (allocator->page_size - 1);
    rsize &= -allocator->page_size;
    g_assert_cmpint((allocator->start + rsize), <=, allocator->end);
    g_assert_cmpint(rsize, >=, size);

    naddr = mlist_alloc(allocator, rsize);
    if (allocator->opts & ALLOC_PARANOID) {
        mlist_check(allocator);
    }

    return naddr;
}

void guest_free(QGuestAllocator *allocator, uint64_t addr)
{
    mlist_free(allocator, addr);
    if (allocator->opts & ALLOC_PARANOID) {
        mlist_check(allocator);
    }
}

QGuestAllocator *alloc_init(uint64_t start, uint64_t end)
{
    QGuestAllocator *s = g_malloc0(sizeof(*s));
    MemBlock *node;

    s->start = start;
    s->end = end;

    QTAILQ_INIT(&s->used);
    QTAILQ_INIT(&s->free);

    node = mlist_new(s->start, s->end - s->start);
    QTAILQ_INSERT_HEAD(&s->free, node, MLIST_ENTNAME);

    return s;
}
