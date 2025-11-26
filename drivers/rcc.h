#ifndef RCC_H
#define RCC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum RccSrcT {
  eRccSrcHsi = 0x00,
  eRccSrcHse = 0x01,
  eRccSrcPll = 0x02
} RccSrc;

typedef struct PllCfgT {
  RccSrc src;
  uint32_t in;
  uint32_t out;
  uint32_t m;
  uint32_t n;
  uint32_t p;
  uint32_t q;
  uint32_t r;
} PllCfg;

typedef struct RccT {
  RccSrc src;
  uint32_t sys;
  uint32_t ahb;
  uint32_t apb1;
  uint32_t apb2;
  PllCfg pll;
  bool hse_bypass;
} Rcc;

// Enables the high-speed external (HSE) clock.
// When bypass is true, the inverter oscillator driver is disabled. This is
// used with external oscillators. Set this to false when using a crystal.
void RCC_EnableHse(bool bypass);

// Disables the high-speed external (HSE) clock.
void RCC_DisableHse();

// Enables the high-speed internal (HSI) RC oscillator clock.
void RCC_EnableHsi();

// Disables the high-speed internal (HSI) RC oscillator clock.
void RCC_DisableHsi();

// Disables the phase-locked loop (PLL) clock.
void RCC_DisablePll();

// Populates *rcc with system clock information computed by reading registers.
void RCC_ReadClocks(Rcc* rcc);

// Computes values for and sets up the STM32 clock tree, as well as setting
// appropriate flash wait states (assuming 3.3 volts VCC).
//    *rcc: May be NULL. If not, is populated with clock frequencies.
//    target: May be NULL if PLL is unused. Set to the target SYSCLK frequency.
uint32_t RCC_ClockConfig(Rcc* rcc, uint32_t target);

#endif
