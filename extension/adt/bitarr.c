#include "bitarr.h"
#include <limits.h>
#include <stdlib.h>

bitarr_t *bitarr_make(size_t num_bits) {
    const size_t num_bytes = (num_bits + CHAR_BIT - 1) / CHAR_BIT;
    return calloc(num_bytes, 1);
}

void bitarr_free(bitarr_t* arr){
    free(arr);
}

bool bitarr_get(const bitarr_t *arr, size_t bit_idx) {
    const size_t byte_idx = bit_idx / CHAR_BIT;
    const size_t bit_pos = bit_idx % CHAR_BIT;
    return arr[byte_idx] & (1 << bit_pos);
}

void bitarr_set(bitarr_t *arr, size_t bit_idx) {
    const size_t byte_idx = bit_idx / CHAR_BIT;
    const size_t bit_pos = bit_idx % CHAR_BIT;
    arr[byte_idx] |= (1 << bit_pos);
}
