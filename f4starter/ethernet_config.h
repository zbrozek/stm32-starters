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

// The MAC address array is not declared const as the MAC address will
// normally be read from an EEPROM and not hard coded (in real deployed
// applications).
static uint8_t ucMACAddress[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

// SMI / MDIO bus pins. Technically a per-board configuration, but in practice
// unlikely to differ. Most packages don't have multiple pins multiplexed to 
// this peripheral.
static Smi smi = {
  .pin_mdc = {GPIOA, 2},
  .pin_mdio = {GPIOC, 1}
};

// RMII pins. Only one of RMII or MII is required; RMII is fewer pins.
static Rmii rmii = {
  .pin_crsdv = {GPIOA, 7},
  .pin_refclk = {GPIOA, 1},
  .pin_rxd0 = {GPIOC, 4},
  .pin_rxd1 = {GPIOC, 5},
  .pin_rxer = {GPIOG, 2},
  .pin_txen = {GPIOB, 11},
  .pin_txd0 = {GPIOB, 12},
  .pin_txd1 = {GPIOB, 13}
};

#endif