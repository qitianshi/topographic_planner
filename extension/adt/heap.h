#ifndef HEAP_H
#define HEAP_H

#include "dynarr.h"


typedef int (*comp_func_t)(const void* b, const void* a, void* state);
typedef void (*str_idx_t)(const void* node, size_t index, void* state);

/**
 * Heap data structure
 *
 * The max element as defined by `comp_func` is the root, so by default this is
 * a maxheap. For a minheap, negate the result of `comp_func`.
 */
struct heap_t {
    struct dynarr_t arr;
    comp_func_t comp_func;
    str_idx_t str_idx_func;
    void *state;
};

//need to map an element void* to an size_t index


/* Creates a new heap */
bool heap_make(
    struct heap_t *heap,
    size_t initial_capacity,
    size_t elem_size,
    comp_func_t comp_func,
    str_idx_t str_idx_func,
    void* state
);

bool heap_is_empty(struct heap_t *heap);

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

void *heap_get(struct heap_t *heap, size_t item_idx);

void heap_dec_key(struct heap_t *heap, size_t item_idx);

#endif
