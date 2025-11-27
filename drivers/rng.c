#include "rng.h"

#include "stm32.h"

// Uses the hardware random number generator (RNG) to get 32 bits of entropy.
// Returned bool indicates whether the process succeeded or failed.
bool RNG_GetRand32(uint32_t* return_value) {
  static uint32_t previous_value = 0;
  uint32_t current_value = 0;

  // Initialize the hardware if it isn't already.
  if (!(RNG->CR & RNG_CR_RNGEN)) {
    RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;  // Enable RNG clock in AHB2.
    RNG->CR |= RNG_CR_RNGEN;            // Enable the RNG peripheral.

    // We're instructed to throw away the first read of the RNG.
    if (!RNG_GetRand32(&current_value)) {
      return false;
    }
    previous_value = current_value;
  }

  // Wait for the hardware to become ready.
  while (!(RNG->SR & RNG_SR_DRDY));

  // We're instructed to verify that successive calls produce new values.
  current_value = RNG->DR;
  if (current_value != previous_value) {
    previous_value = current_value;
    *return_value = current_value;
    return true;
  }

  return false;
}