#ifndef ETHERNET_CONFIG_H
#define ETHERNET_CONFIG_H

#include "ethernet.h"

// SMI/MDIO address of the PHY. There are standardized registers that the
// Ethernet driver will use to reset and release the PHY to avoid start-up bus
// corruption.
static const uint8_t kPhySmiAddress = 0;

// Define the network addressing.  These parameters will be used if either
// ipconfigUSE_DHCP is 0 or if ipconfigUSE_DHCP is 1 but DHCP auto configuration
// fails.
static const uint8_t ucIPAddress[4] = {192, 168, 0, 100};
static const uint8_t ucNetMask[4] = {255, 255, 255, 0};
static const uint8_t ucGatewayAddress[4] = { 192, 168, 0, 1};

// DNS server to use. May be local or on the open internet.
static const uint8_t ucDNSServerAddress[4] = {8, 8, 8, 8};  // Google.

// SMI / MDIO bus pins. Technically a per-board configuration, but in practice
// unlikely to differ. Most packages don't have multiple pins multiplexed to 
// this peripheral.
static Smi smi = {
  .pin_mdc = {GPIOC, 1},
  .pin_mdio = {GPIOA, 2},
  .used = true,
};

// MII pins. Only one of RMII or MII is required.
static Mii mii = {
  .pin_crs = NULL,  // Optional
  .pin_col = NULL,  // Optional
  .pin_rxdv = NULL,
  .pin_rxclk = NULL,
  .pin_rxd0 = NULL,
  .pin_rxd1 = NULL,
  .pin_rxd2 = NULL,
  .pin_rxd3 = NULL,
  .pin_rxer = NULL,  // Optional
  .pin_txen = NULL,
  .pin_txclk = NULL,
  .pin_txd0 = NULL,
  .pin_txd1 = NULL,
  .pin_txd2 = NULL,
  .pin_txd3 = NULL,
  .used = false,
};

// RMII pins. Only one of RMII or MII is required; RMII is fewer pins.
static Rmii rmii = {
  .pin_crsdv = {GPIOA, 7},
  .pin_refclk = {GPIOA, 1},
  .pin_rxd0 = {GPIOC, 4},
  .pin_rxd1 = {GPIOC, 5},
  .pin_rxer = {GPIOG, 2},  // Not always used.
  .pin_txen = {GPIOG, 11},
  .pin_txd0 = {GPIOG, 13},
  .pin_txd1 = {GPIOG, 14},
  .used = true,
};

#endif
