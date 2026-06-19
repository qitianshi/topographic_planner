#include <assert.h>
#include <stdlib.h>

#include "heap.h"

#define HEAP_PARENT(x) (((x) - 1) / 2)
#define HEAP_LEFT_LEAF(x) (2 * (x) + 1)
#define HEAP_RIGHT_LEAF(x) (2 * (x) + 2)


static void heap_swap(struct heap_t *heap, size_t index_a, size_t index_b) {
    assert(heap);
    if (index_a == index_b) return;

    char *a = (char *)heap_get(heap, index_a);
    char *b = (char *)heap_get(heap, index_b);
    if (!a || !b) return;

    // Swap byte by byte to avoid malloc.
    size_t elem_size = heap->arr.elem_size;
    for (size_t i = 0; i < elem_size; i++) {
        char tmp = a[i];
        a[i] = b[i];
        b[i] = tmp;
    }
    if (heap->str_idx_func){
        heap->str_idx_func(b, index_b, heap->state);
        heap->str_idx_func(a, index_a, heap->state);
    }
}

static void heap_fix(struct heap_t *heap, size_t index);

static bool heap_is_larger(
    struct heap_t *heap,
    const void* left_val_idx,
    const void* right_val_idx
){
    return heap->comp_func(left_val_idx, right_val_idx, heap->state) > 0;
}

static void heap_fix(struct heap_t *heap, size_t index) {
    assert(heap);

    size_t largest_idx = index;         // Initial val
    size_t left_idx = HEAP_LEFT_LEAF(index);
    size_t right_idx = HEAP_RIGHT_LEAF(index);
    size_t length = heap->arr.length;

    // Fetches elements safely, NULL if out of bounds
    const void *largest_val = heap_get(heap, largest_idx);     // Initial val
    const void *left_val =
        (left_idx < length) ? heap_get(heap, left_idx) : NULL;
    const void *right_val =
        (right_idx < length) ? heap_get(heap, right_idx) : NULL;

    // Compares with left leaf
    if (left_val && heap_is_larger(heap, left_val, largest_val)) {
        largest_idx = left_idx;
        largest_val = left_val;
    }

    // Compares with right leaf
    if (right_val && heap_is_larger(heap, right_val, largest_val)) {
        largest_idx = right_idx;
    }

    if (largest_idx != index) {
        heap_swap(heap, index, largest_idx);
        heap_fix(heap, largest_idx);
    }
}

/*
static void heap_build(struct heap_t *heap) {
    assert(heap);

    // No building necessary
    if (heap->arr.length <= 1) return;

    // Build heap from the bottom-up.
    // Starting from the last non-leaf node, up to the root.
    for (size_t i = heap->arr.length / 2; i > 0; i--) {
        heap_fix(heap, i - 1);
    }
}
*/

bool heap_is_empty(struct heap_t *heap){
    return heap->arr.length == 0;
}

bool heap_make(
    struct heap_t *heap,
    size_t initial_capacity,
    size_t elem_size,
    comp_func_t comp_func,
    str_idx_t str_loc_func,
    void* state
) {
    assert(heap);
    assert(comp_func);
    assert(elem_size > 0);

    bool dynarr_result = dynarr_make(&heap->arr, initial_capacity, elem_size);
    if (!dynarr_result) {
        return false;
    }
    heap->str_idx_func = str_loc_func;
    heap->state = state;
    heap->comp_func = comp_func;
    return true;
}

static void heap_percolate_up(struct heap_t *heap, size_t index) {
    // Percolate up the new element
    while (index > 0) {
        size_t parent_idx = HEAP_PARENT(index);

        void *curr_val = heap_get(heap, index);
        void *parent_val = heap_get(heap, parent_idx);
        assert(curr_val);
        assert(parent_val);

        // If current is larger than parent, swap them
        if (heap_is_larger(heap, curr_val, parent_val)) {
            heap_swap(heap, index, parent_idx);
            index = parent_idx;
        } else {
            break; // Element is in correct position
        }
    }
}

bool heap_insert(struct heap_t *heap, const void *data) {
    assert(heap);
    assert(data);

    // Append to the end
    if (!dynarr_append(&heap->arr, data)) return false;
    if(heap->str_idx_func){
        heap->str_idx_func(data, heap->arr.length - 1, heap->state);
    }
    heap_percolate_up(heap, heap->arr.length - 1);
    return true;
}

void *heap_get_root(const struct heap_t *heap) {
    assert(heap);
    return dynarr_get(&heap->arr, 0);
}

void heap_pop_root(struct heap_t *heap) {
    assert(heap);
    if (heap->arr.length == 0) return;

    size_t last_idx = heap->arr.length - 1;

    if (last_idx > 0) {
        heap_swap(heap, 0, last_idx);
    }

    dynarr_pop(&heap->arr, 1);

    if (heap->arr.length > 0) {
        heap_fix(heap, 0);
    }
}

void heap_clear(struct heap_t *heap) {
    dynarr_clear(&heap->arr);
}

void heap_free(struct heap_t *heap) {
    dynarr_free(&heap->arr);
}

void* heap_get(struct heap_t *heap, size_t item_idx){
    assert(item_idx < heap->arr.length);
    return dynarr_get(&heap->arr, item_idx);
}

void heap_dec_key(struct heap_t *heap, size_t item_idx) {
    heap_percolate_up(heap, item_idx);
}
