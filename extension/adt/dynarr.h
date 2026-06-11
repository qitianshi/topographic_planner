#ifndef DYNARR_H
#define DYNARR_H

#include <stdbool.h>
#include <stdlib.h>


/* Dynamically resizing array */
struct dynarr_t {
    char *data;
    size_t size;            // Number of bytes
    size_t capacity;        // Number of bytes
};

/* Creates a new dynarray */
bool dynarr_make(
    struct dynarr_t *arr,
    size_t initial_capacity,
    size_t elem_size
);

/* Returns the length (number of elements) of the array */
size_t dynarr_length(struct dynarr_t *arr, size_t elem_size);

/* Appends an element to end of the array */
bool dynarr_append(
    struct dynarr_t *arr,
    const void *data,
    size_t elem_size
);

/* Pops an element from the end of the array */
void dynarr_pop(
    struct dynarr_t *arr,
    size_t amount,
    size_t elem_size
);

/* Gets an element at an index in the array */
void *dynarr_get(
    const struct dynarr_t *arr,
    size_t index,
    size_t elem_size
);

/* Clears all elements from the array */
void dynarr_clear(struct dynarr_t *arr);

/* Frees the array */
void dynarr_free(struct dynarr_t *arr);

#endif
