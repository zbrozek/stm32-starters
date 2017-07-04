#ifndef SPI_H
#define SPI_H

#include "stm32.h"
#include "stdbool.h"

// Configure the SPI clock and peripheral. Pin configuration is up to the
// application software.
bool SPI_Init(SPI_TypeDef *spi, uint32_t clock, uint32_t cpol, uint32_t cpha);

// Read or write an 8-bit register. Returns the peripheral response.
//   data - what's sent after the address; may be null.
uint8_t SPI_Reg8(SPI_TypeDef *spi, uint8_t addr, uint8_t data);

// Transfer a block.
void SPI_ReadMulti8(SPI_TypeDef *spi, uint8_t addr, uint8_t *data,
    uint32_t len);

#endif
