#ifndef RNG_H
#define RNG_H

#include <stdbool.h>
#include <stdint.h>

// Uses the hardware random number generator (RNG) to get 32 bits of entropy.
// Returned bool indicates whether the process succeeded or failed.
bool RNG_GetRand32(uint32_t *return_value);

#endif
