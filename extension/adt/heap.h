#ifndef HEAP_H
#define HEAP_H

#include "dynarr.h"


typedef int (*comp_func_t)(const void *, const void *);

/**
 * Heap data structure
 *
 * The max element as defined by `comp_func` is the root, so by default this is
 * a maxheap. For a minheap, negate the result of `comp_func`.
 */
struct heap_t {
    struct dynarr_t arr;
    comp_func_t comp_func;
};

/* Creates a new heap */
bool heap_make(
    struct heap_t *heap,
    size_t initial_capacity,
    size_t elem_size,
    comp_func_t comp_func
);

/* Inserts an element into the heap */
bool heap_insert(struct heap_t *heap, const void *data);

/* Gets the root (maximal) element of the heap */
void *heap_get_root(const struct heap_t *heap);

/* Removes the root (maximal) element of the heap */
void heap_pop_root(struct heap_t *heap);

/* Clears all elements from the heap */
void heap_clear(struct heap_t *heap);

/* Frees the heap */
void heap_free(struct heap_t *heap);


#endif
