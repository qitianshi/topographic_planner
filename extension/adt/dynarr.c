#include "dynarr.h"

#include <stdbool.h>
#include <stdlib.h>

bool dynarr_make(
    struct dynarr_t *arr,
    size_t initial_capacity,
    size_t elem_size
) {
    arr->data = malloc(initial_capacity * elem_size);

    if (!arr->data)
        return false;   // Failed to allocate memory
    arr->size = 0;
    arr->capacity = initial_capacity * elem_size;

    return true;        // Success
}

size_t dynarr_length(struct dynarr_t *arr, size_t elem_size) {
    return arr->size / elem_size;
}

bool dynarr_append(struct dynarr_t *arr, const void *data, size_t elem_size) {
    void* back = dyn_array_skip(arr, elem_size);

    if (!back) {
        return false;
    } 
    memcpy((char*)arr->data + arr->size - elem_size, data, elem_size);

    return back;    // Success
}

void dynarr_pop(struct dynarr_t *arr, size_t amount, size_t elem_size) {
    if ((size_t)amount * elem_size > arr->size) {
        arr->size = 0;
    } else {
        arr->size -= (size_t)amount * elem_size;
    }
}

void *dynarr_get(const struct dynarr_t *arr, size_t index, size_t elem_size) {
    assert(index < arr->size / elem_size && index >= 0);
    return (char*)arr->data + index * elem_size;
}

void dynarr_clear(struct dynarr_t *arr) {
    arr->size = 0;
}

void dynarr_free(struct dynarr_t *arr) {
    free(arr->data);
    arr->data = NULL;
    arr->size = 0;
    arr->capacity = 0;
}
