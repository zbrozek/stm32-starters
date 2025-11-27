#include "csp.h"
#include "limits.h"
#include "rcc.h"
#include "stm32.h"

// Memory addresses in this file come from ST's RM0410 document, the reference
// manual for the STM32F76x. Available at time of writing from ST at:
//   https://www.st.com/resource/en/reference_manual/rm0410-stm32f76xxx-and-stm32f77xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

// Variables local to this source file
volatile static int64_t CSP_grossCycleCount = 0;
__no_init volatile uint32_t UniqueIdentifier[3] @0x1FF0F420;  // RM0410 45.1

// Call more often than 2^32 cycles to maintain a 64-bit cycle count.
void CSP_UpdateGrossCycleCount(void) {
  static uint32_t lastCycleCount = 0;
  uint32_t currentCycleCount = DWT->CYCCNT;

  if (currentCycleCount < lastCycleCount) {
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
uint32_t CSP_GetFlashSize(void) {
  // RM0410 45.2 "Flash size"
  return ((*(volatile uint32_t*)0x1FF0F442) & 0x0000FFFF) << 10;
}

// Return flash start address for this processor.
uint32_t CSP_GetFlashStartAddr(void) {
  // Appears to be the same for all STM32 processors, regardless of series.
  return 0x08000000;
}

// Reads 96-bit unique ID into caller-provided array.
void CSP_GetUniqueIdentifier(uint32_t* id) {
  for (uint8_t i = 0; i < 3; i++) {
    id[i] = UniqueIdentifier[i];
  }
}

// Convenience function to compare two unique identififers. Returns true if the
// identifiers are the same.
bool CSP_CompareUniqueIdentifier(uint32_t* idA, uint32_t* idB) {
  // STM32F7 unique identifiers are 96 bits long. We will iterate over three
  // uint32_t elements and do simple integer comparison and return false on
  // any failure.
  for (uint8_t i = 0; i < 3; i++) {
    if (idA[i] != idB[i]) {
      return false;
    }
  }

  return true;
}

// Reset the CPU.
void CSP_Reboot(void) {
  NVIC_SystemReset();
}

// Jump to the built-in system bootloader from anywhere. Useful for triggering
// the bootloader without needing to open boxes and press buttons.
void CSP_JumpToBootloader(void) {
  Rcc rcc;
  RCC_ReadClocks(&rcc);

  // See AN2606, "Bootloader configuration" tables for further information:
  //   https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf
  const uint32_t SysBootAddr = 0x1FF00000;

  // This code is going to fundamentally alter MCU configuration and code
  // execution. We need to disable interrupts so that some other code that isn't
  // prepared for these changes can't accidentally take over execution.
  __disable_irq();

  // The system bootloader assumes that the chip is in its reset configuration,
  // and that means running on its "high speed internal" RC oscillator. It's not
  // clear that we actually need to turn off the other clocks, but it seemed
  // good to tidy up a bit after ourselves.
  rcc.src = eRccSrcHsi;
  rcc.hse_bypass = false;
  rcc.pll.src = eRccSrcHsi;
  SystemCoreClock = RCC_ClockConfig(&rcc, 16000000);
  RCC_DisableHse();
  RCC_DisablePll();

  // Clear RCC interrupt flags. Not exactly clear why (or even if) we need to do
  // this, but it was in ST example code. It's believable that if there are any
  // leftover interrupts post-jump that the bootloader would erroneously jump
  // to a handler.
  RCC->CIR |= (RCC_CIR_CSSC | RCC_CIR_PLLSAIRDYC | RCC_CIR_PLLI2SRDYC |
               RCC_CIR_PLLRDYC | RCC_CIR_HSERDYC | RCC_CIR_HSIRDYC |
               RCC_CIR_LSERDYC | RCC_CIR_LSIRDYC | RCC_CIR_HSERDYC);

  // Set AHB and APB clocks to reset state. The system bootloader configures
  // clocks and peripherals and we don't want any bits to be unexpectedly
  // different from what it expects at startup.
  RCC->AHB1ENR = 0x00100000;
  RCC->AHB2ENR = 0x00000000;
  RCC->AHB3ENR = 0x00000000;
  RCC->APB1ENR = 0x00000000;
  RCC->APB2ENR = 0x00000000;

  // Disable SysTick and return it to default values. ST code routinely uses the
  // SysTick for timeouts and configures it at start-up. We don't want to
  // accidentally fire the SysTick before the system bootloader setup code is
  // ready to handle them.
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  // Not actually sure that we need to do either of these.
  RCC->CR &= ~RCC_CR_CSSON;  // Disable clock security switching.
  RCC->CSR |= RCC_CSR_RMVF;  // Clear reset flags.

  // Clear interrupt enable and pending registers. We don't want to accidentally
  // trigger not-yet-ready interrupt handlers post-jump. There are 8 NVIC ICER
  // and ICPR registers, per PM0253 section 4.2, table 40:
  //   https://www.st.com/resource/en/programming_manual/pm0253-stm32f7-series-and-stm32h7-series-cortexm7-processor-programming-manual-stmicroelectronics.pdf
  for (uint8_t i = 0; i < 8; i++) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  // See RM0410 section 7.2.1; we are setting up the memory map as if we had
  // booted with BOOT0 strapped low, which goes to the system bootloader.
  SYSCFG->MEMRMP = SYSCFG_MEMRMP_MEM_BOOT;

  // IRQs are enabled at reset, and the system bootloader does not enable them
  // internally. If they aren't re-enabled then USB enumeration is started and
  // times out eventually.
  __enable_irq();

  // With the execution context set up, now we can jump to the bootloader.
  void (*SysMemBootJump)(void);
  SysMemBootJump = (void (*)(void))(*((uint32_t*)((SysBootAddr + 4))));
  __set_MSP(*(uint32_t*)SysBootAddr);
  SysMemBootJump();  // Perform the jump.
}

// Delay for a specified duration; useful when timers are unavailable. The user
// must call CSP_EnableCycleCounter() before using this function.
void CSP_DelayMicros(uint32_t waitTimeMicros, uint32_t cyclesPerSecond) {
  uint32_t currentCycleCount = DWT->CYCCNT;
  uint32_t cyclesToWait = cyclesPerSecond / (1000 * 1000) * waitTimeMicros;
  uint32_t waitUntilCycleCount = currentCycleCount + cyclesToWait;

  while (DWT->CYCCNT < waitUntilCycleCount);
}

// Enable the debug core cycle counter.
void CSP_EnableCycleCounter() {
  // Access codes comes from the ARM Architecture Reference Manual debug
  // supplement, available at time of writing from:
  //   https://developer.arm.com/documentation/ddi0379/a/Debug-Register-Reference/Management-registers/Lock-Access-Register--LAR-
  DWT->LAR = 0xC5ACCE55;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Compute CRC32 over an input buffer.
uint32_t CSP_ComputeCrc32(uint32_t* input, uint32_t words) {
  // This function is not thread-safe because it interacts directly with the
  // CRC peripheral. We disable interrupts to prevent race conditions.
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  // Enable and reset the state of the CRC peripheral
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
  CRC->CR |= CRC_CR_RESET;

  // Feeds the input data into the CRC peripheral.
  for (uint32_t i = 0; i < words; i++) {
    CRC->DR = input[i];
  }

  // Save the CRC before we disable the CRC peripheral.
  uint32_t crc = CRC->DR;

  // Disable the CRC peripheral.
  RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;

  // Re-enable interrupts.
  __set_PRIMASK(primask);

  return crc;
}

// Print out some clock information.
void CSP_PrintStartupInfo() {
  Rcc rcc;
  RCC_ReadClocks(&rcc);

  printf("STM32F7 with %u KiB flash.\r\n", CSP_GetFlashSize() >> 10);
  printf("SYSCLK at %d MHz.\r\n", rcc.sys / 1000000);
  printf("HCLK (AHB) at %d MHz.\r\n", rcc.ahb / 1000000);
  printf("PCLK1 (APB1) at %d MHz.\r\n", rcc.apb1 / 1000000);
  printf("PCLK2 (APB2) at %d MHz.\r\n\r\n", rcc.apb2 / 1000000);
}
