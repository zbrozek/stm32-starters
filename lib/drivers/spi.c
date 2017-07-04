#include "spi.h"
#include "rcc.h"
#include "stdlib.h"

// TODO(zbrozek): Use SPI_WaitBsy for a SPI_Deinit.
// TODO(zbrozek): Maybe support 16-bit transfers.

// Compute the clock divider.
uint32_t SPI_GetDividerBits(uint32_t pclk, uint32_t spiclk) {
  // Compute the minimum legal clock divider
  uint32_t div_bits;
  for(div_bits = 0; div_bits <= 7; div_bits++) {
    if((pclk >> (1 + div_bits)) <= spiclk) {
      break;
    }
  }
  return div_bits;
}

// Wait for SPI "transmit empty"
static inline void SPI_WaitTxe(SPI_TypeDef *spi) {
    while(!(spi->SR & SPI_SR_TXE));
}

// Wait for SPI "receive not empty"
static inline void SPI_WaitRxne(SPI_TypeDef *spi) {
    while(!(spi->SR & SPI_SR_RXNE));
}

#if 0
// Wait for SPI "not busy"
static inline void SPI_WaitBsy(SPI_TypeDef* spi) {
    while(!(spi->SR & SPI_SR_BSY));
}
#endif

// Configure the SPI clock and peripheral. Pin configuration is up to the
// application software.
bool SPI_Init(SPI_TypeDef *spi, uint32_t clock, uint32_t cpol,
    uint32_t cpha) {

  // Prepare to compute and enable the SPI clocks.
  Rcc rcc;
  RCC_ReadClocks(&rcc);
  uint32_t pclk = rcc.apb2;
  volatile uint32_t *rcc_reg = &RCC->APB2ENR;
  uint32_t rcc_bit;

  switch((uint32_t)spi) {
#ifdef SPI1
    case (uint32_t)SPI1:
    rcc_bit = RCC_APB2ENR_SPI1EN;
    break;
#endif
#ifdef SPI2
    case (uint32_t)SPI2:
    pclk = rcc.apb1;
    rcc_reg = &RCC->APB1ENR;
    rcc_bit = RCC_APB1ENR_SPI2EN;
    break;
#endif
#ifdef SPI3
    case (uint32_t)SPI3:
    pclk = rcc.apb1;
    rcc_reg = &RCC->APB1ENR;
    rcc_bit = RCC_APB1ENR_SPI3EN;
    break;
#endif
#ifdef SPI4
    case (uint32_t)SPI4:
    rcc_bit = RCC_APB2ENR_SPI4EN;
    break;
#endif
#ifdef SPI5
    case (uint32_t)SPI5:
    rcc_bit = RCC_APB2ENR_SPI5EN;
    break;
#endif
#ifdef SPI6
    case (uint32_t)SPI6:
    rcc_bit = RCC_APB2ENR_SPI6EN;
    break;
#endif
    default:
    return false;
  }

  // Enable the SPI peripheral clock.
  *rcc_reg |= rcc_bit;

  // Configure the peripheral
  spi->CR1 =
      SPI_CR1_SSM |  // Software chip-select control.
      SPI_CR1_SSI |  // Tie the hardware slave select high.
      SPI_CR1_MSTR |  // Master mode control.
      (SPI_GetDividerBits(pclk, clock) << SPI_CR1_BR_Pos) |  // Clock divider.
      (cpol << SPI_CR1_CPOL_Pos) |  // Clock polarity.
      (cpha << SPI_CR1_CPHA_Pos);  // Clock phase.

  // Disable the I2S module; select I2C mode.
  spi->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;

  // Enable the SPI peripheral in master mode.
  spi->CR1 |= SPI_CR1_SPE;

  return true;
}

// Read or write an 8-bit register. Returns the peripheral response.
//   data - what's sent after the address; may be null.
uint8_t SPI_Reg8(SPI_TypeDef *spi, uint8_t addr, uint8_t data) {
  SPI_WaitTxe(spi);
  spi->DR = addr;
  SPI_WaitTxe(spi);
  spi->DR = data;
  SPI_WaitTxe(spi);
  SPI_WaitRxne(spi);
  uint8_t dummy = spi->DR;
  SPI_WaitRxne(spi);
  return spi->DR;
}

// Read a multi-register block. Note that len must be >1.
void SPI_ReadMulti8(SPI_TypeDef *spi, uint8_t addr, uint8_t *data,
    uint32_t len) {
  uint8_t dummy = spi->DR;  // Clear erroneously available data.
  SPI_WaitTxe(spi);
  spi->DR = addr;  // Send the block start address.
  SPI_WaitRxne(spi);
  dummy = spi->DR;  // Throw away the data transferred from the start address.
  spi->DR = 0;  // Start clocking the next byte.

  // Clock out and capture the rest of the block.
  for(uint32_t count = 0; count < len; count++) {
    if(count < len - 1) {
      SPI_WaitTxe(spi);
      spi->DR = 0;
    }
    SPI_WaitRxne(spi);
    data[count] = spi->DR;
  }
}
