#ifndef CSP_H
#define CSP_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// Call more often than 2^32 cycles to maintain a 64-bit cycle count
void CSP_UpdateGrossCycleCount(void);

// Compute the total number of elapsed clock cycles since boot
int64_t CSP_TotalClockCycles(void);

// Compute the number of milliseconds elapsed since boot
int64_t CSP_TimeMillis(void);

// Return flash size in 32-bit words
int32_t CSP_GetFlashSize(void);

// Return flash start address for this processor
int32_t CSP_GetFlashStartAddr(void);

// Reset the CPU
void CSP_Reboot(void);

// Core configuration to turn on cycle counting since boot
void CSP_EnableCycleCounter(void);

// Print out some clock information
void CSP_PrintStartupInfo();

#endif