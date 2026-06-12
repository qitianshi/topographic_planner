#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>

#include "dynarr.h"


bool dynarr_make(
    struct dynarr_t *arr,
    size_t initial_capacity,
    size_t elem_size
) {
    assert(arr);
    assert(elem_size > 0);

    if (initial_capacity > 0) {
        arr->data = malloc(initial_capacity * elem_size);
        if (!arr->data) {
            return false;
        }
    } else {
        arr->data = NULL;
    }

    arr->length = 0;
    arr->capacity = initial_capacity;
    arr->elem_size = elem_size;

    return true;
}

bool dynarr_append(struct dynarr_t *arr, const void *data) {
    assert(arr);
    assert(data);

    if (arr->length >= arr->capacity) {
        // Double the capacity, or set to 1 if it was 0
        size_t new_capacity = (arr->capacity > 0) ? arr->capacity * 2 : 1;

        char *new_data = realloc(arr->data, new_capacity * arr->elem_size);
        if (!new_data) {
            return false;   // Memory reallocation failed
        }

        arr->data = new_data;
        arr->capacity = new_capacity;
    }

    // Calculate the destination address using length and elem_size
    char *dest = arr->data + (arr->length * arr->elem_size);
    memcpy(dest, data, arr->elem_size);

    arr->length++;

    return true;
}

void dynarr_pop(struct dynarr_t *arr, size_t n_elems) {
    assert(arr);

    // Prevent underflow by capping the pop amount
    if (n_elems > arr->length) {
        arr->length = 0;
    } else {
        arr->length -= n_elems;
    }
}

void *dynarr_get(const struct dynarr_t *arr, size_t index) {
    assert(arr);
    assert(arr->data);

    if (index >= arr->length) {
        return NULL;
    }

    return arr->data + (index * arr->elem_size);
}

void dynarr_clear(struct dynarr_t *arr) {
    assert(arr);
    arr->length = 0;
}

void dynarr_free(struct dynarr_t *arr) {
    assert(arr);
    free(arr->data);
}
