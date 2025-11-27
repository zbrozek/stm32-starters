#include "rng.h"

#include "stm32.h"

// Initialize the hardware random number generator.
bool RNG_Init() {
  // Disable interrupts around shared peripheral use to avoid race conditions.
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  // Enable the RNG clock and enable the RNG.
  RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;  // Enable RNG clock in AHB2.
  RNG->CR |= RNG_CR_RNGEN;            // Enable the RNG peripheral.

  // Wait for the hardware to become ready, with timeout.
  uint32_t loop_count = 0;
  uint32_t loop_until = 10000;
  while (!(RNG->SR & RNG_SR_DRDY)) {
    if (loop_count > loop_until) {
      // Re-enable interrupts and bail out unsuccessfully.
      __set_PRIMASK(primask);
      return false;
    }
    ++loop_count;
  }

  // Throw away the first read of the RNG.
  uint32_t rng_throwaway = RNG->DR;

  // Re-enable interrupts.
  __set_PRIMASK(primask);

  return true;
}

// Uses the hardware random number generator (RNG) to get 32 bits of entropy.
// Returned bool indicates whether the process succeeded or failed. The user
// must call RNG_Init() before using this function.
bool RNG_GetRand32(uint32_t* return_value) {
  // Disable interrupts around shared peripheral use to avoid race conditions.
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  static uint32_t previous_value = 0;
  uint32_t current_value = 0;

  // Wait for the hardware to become ready, with timeout.
  uint32_t loop_count = 0;
  uint32_t loop_until = 10000;
  while (!(RNG->SR & RNG_SR_DRDY)) {
    if (loop_count > loop_until) {
      // Re-enable interrupts and bail out unsuccessfully.
      __set_PRIMASK(primask);
      return false;
    }
    ++loop_count;
  }

  // We're instructed by the reference manual to verify that successive calls
  // produce new values. I don't love doing this, but we're doing what we're
  // told.
  current_value = RNG->DR;
  if (current_value != previous_value) {
    previous_value = current_value;
    *return_value = current_value;
    // Re-enable interrupts.
    __set_PRIMASK(primask);
    return true;
  }

  // Re-enable interrupts and bail out unsuccessfully.
  __set_PRIMASK(primask);
  return false;
}