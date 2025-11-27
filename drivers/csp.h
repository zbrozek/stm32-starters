#ifndef CSP_H
#define CSP_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// Call more often than 2^32 cycles to maintain a 64-bit cycle count.
void CSP_UpdateGrossCycleCount(void);

// Compute the total number of elapsed clock cycles since boot.
int64_t CSP_TotalClockCycles(void);

// Compute the number of milliseconds elapsed since boot.
int64_t CSP_TimeMillis(void);

// Return flash size in 32-bit words.
uint32_t CSP_GetFlashSize(void);

// Return flash start address for this processor.
uint32_t CSP_GetFlashStartAddr(void);

// Reads 96-bit unique ID into caller-provided array.
void CSP_GetUniqueIdentifier(uint32_t* id);

// Convenience function to compare two unique identififers. Returns true if the
// identifiers are the same.
bool CSP_CompareUniqueIdentifier(uint32_t* idA, uint32_t* idB);

// Reset the CPU.
void CSP_Reboot(void);

// Jump to the built-in system bootloader from anywhere. Useful for triggering
// the bootloader without needing to open boxes and press buttons.
void CSP_JumpToBootloader(void);

// Delay for a specified duration; useful when timers are unavailable. The user
// must call CSP_EnableCycleCounter() before using this function.
void CSP_DelayMicros(uint32_t waitTimeMicros, uint32_t cyclesPerSecond);

// Enable the debug core cycle counter.
void CSP_EnableCycleCounter(void);

// Compute CRC32 over an input buffer.
uint32_t CSP_ComputeCrc32(uint32_t* input, uint32_t words);

// Print out some clock information.
void CSP_PrintStartupInfo();

#endif
