#ifndef ETHERNET_H
#define ETHERNET_H

#include "stm32.h"
#include "pin.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "FreeRTOS_UDP_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"

typedef  __packed struct SmiT {
  Pin pin_mdc;
  Pin pin_mdio;
} Smi;

typedef  __packed struct MiiT {
  Pin pin_crs;
  Pin pin_col;
  Pin pin_rxdv;
  Pin pin_rxclk;
  Pin pin_rxd0;
  Pin pin_rxd1;
  Pin pin_rxd2;
  Pin pin_rxd3;
  Pin pin_rxer;
  Pin pin_txen;
  Pin pin_txclk;
  Pin pin_txd0;
  Pin pin_txd1;
  Pin pin_txd2;
  Pin pin_txd3;
} Mii;

typedef  __packed struct RmiiT {
  Pin pin_crsdv;
  Pin pin_refclk;
  Pin pin_rxd0;
  Pin pin_rxd1;
  Pin pin_rxer;
  Pin pin_txen;
  Pin pin_txd0;
  Pin pin_txd1;
} Rmii;

// The peripheral has some silly timing requirements around register writes.
void ETH_WriteReg(volatile uint32_t *reg, uint32_t value);

// PHY handling functions.
bool ETH_SmiTransfer(uint16_t addr, uint16_t reg, uint16_t *data, bool write);
extern bool PHY_Init(void);

// Deferred processing tasks
static void rxTask(void *pvParameters);
static void txTask(void *pvParameters);

// FreeRTOS Ethernet driver hooks.
BaseType_t xNetworkInterfaceInitialise(void);
BaseType_t xNetworkInterfaceOutput(
    NetworkBufferDescriptor_t * const pxDescriptor,
    BaseType_t xReleaseAfterSend );

#endif