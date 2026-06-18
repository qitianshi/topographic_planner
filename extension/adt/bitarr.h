#ifndef BITARR_H
#define BITARR_H
#include <stdbool.h>
#include <stddef.h>

typedef unsigned char bitarr_t;

bitarr_t *bitarr_make(size_t num_bits);

void bitarr_free(bitarr_t* arr);

bool bitarr_get(const bitarr_t *arr, size_t bit_idx);

void bitarr_set(bitarr_t *arr, size_t bit_idx);
#endif
