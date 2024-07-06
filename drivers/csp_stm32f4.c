#include "limits.h"
#include "csp.h"

#include "stm32.h"
#include "rcc.h"

// Memory addresses in this file come from ST's RM0090 document, the reference
// manual for the STM32F42x. Available at time of writing from ST at:
//   https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

// Variables local to this source file
volatile static int64_t CSP_grossCycleCount = 0;
__no_init volatile uint32_t UniqueIdentifier[3] @ 0x1FFF7A10;  // RM0090 39.1

// Call more often than 2^32 cycles to maintain a 64-bit cycle count.
void CSP_UpdateGrossCycleCount(void) {
  static uint32_t lastCycleCount = 0;
  uint32_t currentCycleCount = DWT->CYCCNT;

  if(currentCycleCount < lastCycleCount) {
    CSP_grossCycleCount += ULONG_MAX;
  }

  lastCycleCount = currentCycleCount;
}

// Compute the total number of elapsed clock cycles since boot.
int64_t CSP_TotalClockCycles(void) {
  // Define the order of volatile accesses by creating a temporary variable
  int64_t bigPart = CSP_grossCycleCount;
  return bigPart + DWT->CYCCNT;
}

// Compute the number of milliseconds elapsed since boot.
int64_t CSP_TimeMillis(void) {
  return CSP_TotalClockCycles() * 1000 / SystemCoreClock;
}

// Return flash size in 32-bit words.
int32_t CSP_GetFlashSize(void) {
  // RM0090 39.2 "Flash size"
  return ((*(volatile uint32_t*)0x1FFF7A22) & 0x0000FFFF) << 10;
}

// Return flash start address for this processor.
int32_t CSP_GetFlashStartAddr(void) {
  // Appears to be the same for all STM32 processors, regardless of series.
  return 0x08000000;
}

// Reads 96-bit unique ID into caller-provided array.
void CSP_GetUniqueIdentifier(uint32_t* Id) {
  for(uint8_t i = 0; i < 3; i++) {
    Id[i] = UniqueIdentifier[i];
  }
}

// Convenience function to compare two unique identififers. Returns true if the
// identifiers are the same.
bool CSP_CompareUniqueIdentifier(uint32_t* IdA, uint32_t* IdB) {
  // STM32F4 unique identifiers are 96 bits long. We will iterate over three
  // uint32_t elements and do simple integer comparison and return false on
  // any failure.
  for(uint8_t i = 0; i < 3; i++) {
    if(IdA[i] != IdB[i]) {
      return false;
    }
  }

  return true;
}

// Reset the CPU.
void CSP_Reboot(void) {
  NVIC_SystemReset();
}

// Enable the debug core cycle counter.
void CSP_EnableCycleCounter() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Print out some clock information.
void CSP_PrintStartupInfo() {
  Rcc rcc;
  RCC_ReadClocks(&rcc);

  printf("STM32F4 with %u KiB flash.\r\n", CSP_GetFlashSize() >> 10);
  printf("SYSCLK at %d MHz.\r\n", rcc.sys / 1000000);
  printf("HCLK (AHB) at %d MHz.\r\n", rcc.ahb / 1000000);
  printf("PCLK1 (APB1) at %d MHz.\r\n", rcc.apb1 / 1000000);
  printf("PCLK2 (APB2) at %d MHz.\r\n\r\n", rcc.apb2 / 1000000);
}
