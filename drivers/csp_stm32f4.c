#include "limits.h"
#include "csp.h"

#include "stm32.h"
#include "rcc.h"

// Variables local to this source file
volatile static int64_t CSP_grossCycleCount = 0;

// Registers and flags related to the Cortex-M4F cycle counter
#define CYCCNT (*(volatile const uint32_t*)0xE0001004)
#define DWT_CTRL (*(volatile uint32_t*)0xE0001000)
#define DWT_CTRL_CYCEN 0x00000001
#define SCB_DEMCR (*(volatile uint32_t*)0xE000EDFC)
#define SCB_DEMCR_TRCEN 0x01000000

// Call more often than 2^32 cycles to maintain a 64-bit cycle count
void CSP_UpdateGrossCycleCount(void) {
    static uint32_t lastCycleCount = 0;
    uint32_t currentCycleCount = CYCCNT;

    if(currentCycleCount < lastCycleCount){
        CSP_grossCycleCount += ULONG_MAX;
    }

    lastCycleCount = currentCycleCount;
}

// Compute the total number of elapsed clock cycles since boot
int64_t CSP_TotalClockCycles(void) {
    // Define the order of volatile accesses by creating a temporary variable
    int64_t bigPart = CSP_grossCycleCount;
    return bigPart + CYCCNT;
}

// Compute the number of milliseconds elapsed since boot
int64_t CSP_TimeMillis(void) {
    return CSP_TotalClockCycles() * 1000 / SystemCoreClock;
}

// Return flash size in 32-bit words
int32_t CSP_GetFlashSize(void) {
    return ((*(volatile uint32_t*)0x1FFF7A22) & 0x0000FFFF) << 8;
}

// Return flash start address for this processor
int32_t CSP_GetFlashStartAddr(void) {
    return 0x08000000;
}

// Reset the CPU
void CSP_Reboot(void) {
    NVIC_SystemReset();
}

// M3/M4 core configuration to turn on cycle counting since boot
void CSP_EnableCycleCounter() {
  // Trace enable, which in turn enables the DWT_CTRL register
  SCB_DEMCR |= SCB_DEMCR_TRCEN;
  // Start the CPU cycle counter (read with CYCCNT)
  DWT_CTRL |= DWT_CTRL_CYCEN;
}

// Print out some clock information
void CSP_PrintStartupInfo() {
  Rcc rcc;
  RCC_ReadClocks(&rcc);

  printf("STM32F4 with %u KiB flash.\r\n", CSP_GetFlashSize() >> 10);
  printf("SYSCLK at %d MHz.\r\n", rcc.sys / 1000000);
  printf("HCLK (AHB) at %d MHz.\r\n", rcc.ahb / 1000000);
  printf("PCLK1 (APB1) at %d MHz.\r\n", rcc.apb1 / 1000000);
  printf("PCLK2 (APB2) at %d MHz.\r\n\r\n", rcc.apb2 / 1000000);
}