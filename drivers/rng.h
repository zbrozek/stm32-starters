#ifndef RNG_H
#define RNG_H

#include <stdbool.h>
#include <stdint.h>

// Initialize the hardware random number generator.
bool RNG_Init();

// Uses the hardware random number generator (RNG) to get 32 bits of entropy.
// Returned bool indicates whether the process succeeded or failed. The user
// must call RNG_Init() before using this function.
bool RNG_GetRand32(uint32_t* return_value);

#endif
