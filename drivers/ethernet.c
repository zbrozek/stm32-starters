// This file implements most of the glue between the Ethernet hardware and
// FreeRTOS+UDP. Check out the documentation at:
// http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/Embedded_Ethernet_Porting.html

// ************************************
// #### DO NOT TURN ON CACHES YET! ####
// ************************************

// TODO(zbrozek): Add nice constant definitions for standard SMI registers.
// TODO(zbrozek): Make the DMA descriptors a bitfield or a union.
// TODO(zbrozek): Make sure that all handoffs to/from DMA are cache-coherent.
//   SCB_InvalidateDCache_by_Addr
//   SCB_CleanDCache_by_Addr

#include "stm32.h"
#include "rcc.h"
#include "ethernet.h"
#include "ethernet_config.h"
#include <stdbool.h>
#include <stdio.h>

// Pad the Ethernet frame up to a 4-byte boundary.
#define FRAME_PADDED ((ipTOTAL_ETHERNET_FRAME_SIZE + 3) & ~0x03U)

typedef struct ETH_DmaDescT {
  uint32_t Status;
  uint32_t ControlBufferSize;
  uint8_t *Buffer1Addr;
  void *Buffer2NextDescAddr;
  uint32_t ExtendedStatus;
  uint32_t Reserved1;
  uint32_t TimeStampLow;
  uint32_t TimeStampHigh;
} ETH_DmaDesc;

// Constants
const uint32_t kEthAf = 11;
const uint32_t kMacmiiarCrMask = ((uint32_t)0xFFFFFFE3U);
const uint32_t kMaccrClearMask = ((uint32_t)0xFF20810FU);
const uint32_t kMacfcrClearMask = ((uint32_t)0x0000FF41U);
const uint32_t kDmaomrClearMask = ((uint32_t)0xF8DE3F23U);

// Globals
TaskHandle_t rx_task_handle, tx_task_handle;

#pragma data_alignment=4
ETH_DmaDesc tx_dma_desc[ipconfigNUM_TX_DESCRIPTORS];
ETH_DmaDesc rx_dma_desc[ipconfigNUM_RX_DESCRIPTORS];

// The peripheral has some silly timing requirements around register writes.
void ETH_WriteReg(volatile uint32_t *reg, uint32_t value) {
  *reg = value;
  value = *reg;
  vTaskDelay(1);
  *reg = value;
}

bool ETH_InitSmiGpio(Smi *smi) {
  if(smi->used == false) { return false; }

  Pin_ConfigGpioPin(&(smi->pin_mdio), ePinModeAlt, ePinOutputOpenDrain,
      ePinSpeedMax, ePinPullUp, kEthAf);
  Pin_ConfigGpioPin(&(smi->pin_mdc), ePinModeAlt, ePinOutputPushPull,
      ePinSpeedMax, ePinPullNone, kEthAf);

  // Pick the SMI clock divisor.
  Rcc rcc;
  RCC_ReadClocks(&rcc);
  uint32_t clk_div_bits = 0;
  if(rcc.ahb >= (150 * 1000 * 1000)) {
    clk_div_bits = ETH_MACMIIAR_CR_Div102;
  } else if (rcc.ahb >= (100 * 1000 * 1000)) {
    clk_div_bits = ETH_MACMIIAR_CR_Div62;
  } else if (rcc.ahb >= (60 * 1000 * 1000)) {
    clk_div_bits = ETH_MACMIIAR_CR_Div42;
  } else if (rcc.ahb >= (35 * 1000 * 1000)) {
    clk_div_bits = ETH_MACMIIAR_CR_Div26;
  } else {
    clk_div_bits = ETH_MACMIIAR_CR_Div16;
  }

  // Clear and set the clock bits.
  ETH->MACMIIAR &= ETH_MACMIIAR_CR;
  ETH->MACMIIAR |= clk_div_bits;

  return true;
}

bool ETH_InitMiiGpio(Mii *mii) {
  if(mii->used == false) { return false; }

  Pin *pins = (Pin*)mii;

  for(int a = 0; a < 15; a++) {
    if(&pins[a]) {
      Pin_ConfigGpioPin(&pins[a], ePinModeAlt, ePinOutputPushPull,
          ePinSpeedMax, ePinPullNone, kEthAf);
    }
  }

  return true;
}

bool ETH_InitRmiiGpio(Rmii *rmii) {
  if(rmii->used == false) { return false; }

  Pin *pins = (Pin*)rmii;

  for(int a = 0; a < 8; a++) {
    if(&pins[a]) {
      Pin_ConfigGpioPin(&pins[a], ePinModeAlt, ePinOutputPushPull,
          ePinSpeedMax, ePinPullNone, kEthAf);
    }
  }

  return true;
}

// Transfers one register from an SMI-connected slave.
bool ETH_SmiTransfer(uint16_t addr, uint16_t reg, uint16_t *data, bool write) {
  // Bail if the SMI bus isn't idle.
  if(ETH->MACMIIAR & ETH_MACMIIAR_MB) {
    return false;
  }

  // Start a new transaction.
  ETH->MACMIIDR = *data;
  ETH->MACMIIAR &= ~(ETH_MACMIIAR_PA | ETH_MACMIIAR_MR | ETH_MACMIIAR_MW);
  ETH->MACMIIAR |=
    (addr & 0x1F) << ETH_MACMIIAR_PA_Pos |  // PHY address.
    (reg & 0x1F) << ETH_MACMIIAR_MR_Pos |  // PHY register.
    (write ? ETH_MACMIIAR_MW : 0) |  // Read/write bit.
    ETH_MACMIIAR_MB;  // Tell the peripheral to start the transaction.

  // Busy wait until the transaction completes.
  while(ETH->MACMIIAR & ETH_MACMIIAR_MB);

  // Send back the data to the caller.
  if(!write) {
    *data = ETH->MACMIIDR;
  }

  return true;
}

// Configure the GPIOs to the PHY.
//   Only one of mii or rmii must be non-null.
//   smi may be null.
void ETH_InitBus(Smi *smi, Mii *mii, Rmii *rmii) {
  // Put the Ethernet peripheral into reset.
  RCC->AHB1RSTR |= RCC_AHB1RSTR_ETHMACRST;

  // Enable the SYSCFG clock.
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // Choose MII or RMII, must be done while MAC is in reset and clock disabled.
  SYSCFG->PMC &= ~(SYSCFG_PMC_MII_RMII_SEL);
  SYSCFG->PMC |= rmii->used ? SYSCFG_PMC_MII_RMII_SEL : 0;

  // Enable Ethernet GPIOs.
  ETH_InitMiiGpio(mii);
  ETH_InitRmiiGpio(rmii);

  // Enable Ethernet clocks.
  RCC->AHB1ENR |=
      RCC_AHB1ENR_ETHMACEN |
      RCC_AHB1ENR_ETHMACTXEN |
      RCC_AHB1ENR_ETHMACRXEN;

  // Take the Ethernet peripheral out of reset.
  RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_ETHMACRST);

  // Enable the Ethernet interrupt.
  NVIC_SetPriority(ETH_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(ETH_IRQn);

  // Issue Ethernet DMA peripheral reset.
  ETH->DMABMR |= ETH_DMABMR_SR;
  while(ETH->DMABMR & ETH_DMABMR_SR);  // Apparently it takes a while to reset.

  // Configure SMI GPIOs.
  ETH_InitSmiGpio(smi);
};

// Initialize the MAC with sane defaults.
void ETH_MacInit(void) {
  uint32_t maccr = ETH->MACCR & kMaccrClearMask;
  maccr |=
      ETH_MACCR_FES |  // MII is 100 mbps.
      ETH_MACCR_DM |  // Enable full duplex TX/RX.
      ETH_MACCR_IPCO |  // Automatic RX IP checksumming.
      ETH_MACCR_APCS;  // Automatic pad/checksum stripping on received frames.
  ETH_WriteReg(&ETH->MACCR, maccr);
}

// Initialize the MAC DMA peripheral.
void ETH_DmaInit(void) {
  uint32_t dmaomr = ETH->DMAOMR;
  dmaomr &= kDmaomrClearMask;
  dmaomr |=
      ETH_DMAOMR_RSF |  // Enable RX store-and-forward.
      ETH_DMAOMR_TSF |  // Enable TX store-and-forward.
      ETH_DMAOMR_FTF |  // Flush the TX FIFO.
      ETH_DMAOMR_OSF;  // Start next frame before receiving prev frame status.
  ETH_WriteReg(&ETH->DMAOMR, dmaomr);

  uint32_t dmabmr =
      ETH_DMABMR_AAB |  // Enable address-aligned beats.
      ETH_DMABMR_USP |  // Use both RX and TX burst length numbers.
      ETH_DMABMR_RDP_32Beat | // 32 beats in a single RX transaction.
      ETH_DMABMR_FB |  // Enable fixed burst transfers.
      ETH_DMABMR_PBL_32Beat |  // 32 beats in a single TX transaction.
      ETH_DMABMR_EDE;  // Enable enhanced 32-byte descriptors.
  ETH_WriteReg(&ETH->DMABMR, dmabmr);

  uint32_t dmaier =
      ETH_DMAIER_NISE |  // Enable normal interrupt summary.
      ETH_DMAIER_RIE |  // Enable receive interrupts.
      ETH_DMAIER_TIE;  // Enable transmit interrupts.
  ETH_WriteReg(&ETH->DMAIER, dmaier);
}

// Take the (defined in ethernet_config.h) MAC address and inform the hardware.
void ETH_SetMacAddr(uint8_t ucMACAddress[6]) {
  uint32_t maca0hr = ETH->MACA0HR & 0xFFFF0000;
  maca0hr |= (ucMACAddress[5] << 8) | (ucMACAddress[4]);
  ETH_WriteReg(&ETH->MACA0HR, maca0hr);
  uint32_t maca0lr =
      (ucMACAddress[3] << 24) |
      (ucMACAddress[2] << 16) |
      (ucMACAddress[1] << 8) |
      (ucMACAddress[0]);
  ETH_WriteReg(&ETH->MACA0LR, maca0lr);
}

// Find the next CPU-owned descriptor. Doesn't care about RX vs TX.
//   head - points to the first descriptor
ETH_DmaDesc *ETH_GetNextDesc(ETH_DmaDesc *head,
    int *start, int len, bool check_owned) {
  int end = *start + len - 1; // Only cycle through once - don't return start.
  int current;
  ETH_DmaDesc *desc;

  // Iterate over descriptors. If desired, return only those owned by the CPU.
  for(int a = *start; a < end; a++) {
    current = a % len;
    desc = &head[current];
    if(!check_owned || !(desc->Status & (1U << 31))) {
      *start = (current + 1) % len;
      return desc;
    }
  }
  return NULL;  // No descriptors are owned by the CPU.
}

// Wire up the TX DMA descriptors, but we don't actually point to any buffers.
// Those are allocated at transmission.
void ETH_DmaDescTxInit(ETH_DmaDesc *head, int len) {
  // Inform the hardware where to look for descriptors.
  ETH->DMATDLAR = (uint32_t)head;  // Start of descriptor list register.

  // Clear the memory used by the descriptors.
  memset(head, 0, len * sizeof(ETH_DmaDesc));

  // Chain all of the descriptors.
  int next_idx = 0;
  ETH_DmaDesc *current = ETH_GetNextDesc(head, &next_idx, len, false);
  ETH_DmaDesc *next;
  do {
    next = ETH_GetNextDesc(head, &next_idx, len, false);
    current->Status = (1U << 20);  // Second address chained.
    current->Buffer2NextDescAddr = next;
    current = next;
  } while(next != head);
}

// Wire up the RX descriptors. Except we actually allocate buffers.
void ETH_DmaDescRxInit(ETH_DmaDesc *head, int len) {
  static size_t frame_len = FRAME_PADDED;

  // Inform the hardware where to look for descriptors.
  ETH->DMARDLAR = (uint32_t)rx_dma_desc;  // Start of descriptor list register.

  // Clear the memory used by the descriptors.
  memset(head, 0, len * sizeof(ETH_DmaDesc));

  // Chain all of the descriptors.
  int next_idx = 0;
  ETH_DmaDesc *current = ETH_GetNextDesc(head, &next_idx, len, false);
  ETH_DmaDesc *next;
  do {
    next = ETH_GetNextDesc(head, &next_idx, len, false);
    current->ControlBufferSize = (1U << 14) |  // Second address chained.
        (frame_len & 0x1FFF);  // Buffer is the maximum packet length.
    current->Buffer1Addr = pucGetNetworkBuffer(&frame_len);
    current->Buffer2NextDescAddr = next;
    current->Status = (1U << 31);  // Set ownership to DMA peripheral.
    current = next;
  } while(next != head);
}

void ETH_Start() {
  // Start MAC TX and RX.
  ETH_WriteReg(&ETH->MACCR, ETH->MACCR | ETH_MACCR_TE);
  ETH_WriteReg(&ETH->MACCR, ETH->MACCR | ETH_MACCR_RE);

  // Flush the DMA TX FIFO.
  ETH_WriteReg(&ETH->DMAOMR, ETH->DMAOMR | ETH_DMAOMR_FTF);

  // Start DMA TX and RX.
  ETH_WriteReg(&ETH->DMAOMR, ETH->DMAOMR | ETH_DMAOMR_ST);
  ETH_WriteReg(&ETH->DMAOMR, ETH->DMAOMR | ETH_DMAOMR_SR);
}

// FreeRTOS calls this from a thread (so it's OK if this blocks) to set up the
// the Ethernet peripheral.
BaseType_t xNetworkInterfaceInitialise( void ) {
  // Enable the various digital buses to the PHY.
  ETH_InitBus(&smi, &mii, &rmii);
  PHY_Init();  // Link should be up when this function returns.

  // Configure the Ethernet and DMA peripherals.
  ETH_MacInit();
  ETH_DmaInit();
  ETH_SetMacAddr(ucMACAddress);
  ETH_DmaDescTxInit(tx_dma_desc, ipconfigNUM_TX_DESCRIPTORS);
  ETH_DmaDescRxInit(rx_dma_desc, ipconfigNUM_RX_DESCRIPTORS);

  // Fire up a task which will handle packet reception.
  xTaskCreate(rxTask,
      "rxTask",
      configMINIMAL_STACK_SIZE,
      NULL,
      configMAX_PRIORITIES - 1,
      &rx_task_handle);

  // Fire up a task which will release TX buffers.
  xTaskCreate(txTask,
      "txTask",
      configMINIMAL_STACK_SIZE,
      NULL,
      configMAX_PRIORITIES - 2,
      &tx_task_handle);

  // Enable data reception and transmission at the DMA peripheral. Now we can
  // expect to start moving packets.
  ETH_Start();

  return pdPASS;
}

// This finds a CPU-owned TX descriptor, points it to the Ethernet buffer, and
// sets some flags to hand it back to the DMA for transmission. Note that the
// pucEthernetBuffer gets released back to the IP stack in the TX ISR.
BaseType_t xNetworkInterfaceOutput(
    NetworkBufferDescriptor_t * const pxDescriptor,
    BaseType_t xReleaseAfterSend ) {
  static int next_tx = 0;

  // Fetch the next CPU-owned TX descriptor.
  ETH_DmaDesc *desc = ETH_GetNextDesc(tx_dma_desc,
      &next_tx, ipconfigNUM_TX_DESCRIPTORS, true);

  if(!desc) {return pdFALSE;}  // No available TX descriptors.

  // FreeRTOS+TCP doesn't zero the ICMP checksum field, but the STM32 MAC
  // requires zeros for hardware checksumming. This is a hack workaround.
  ProtocolPacket_t *pxPacket;
  pxPacket = (ProtocolPacket_t*)(pxDescriptor->pucEthernetBuffer);
  if(pxPacket->xICMPPacket.xIPHeader.ucProtocol == ipPROTOCOL_ICMP) {
    pxPacket->xICMPPacket.xICMPHeader.usChecksum = (uint16_t)0u;
  }

  // Populate the fresh descriptor.
  desc->Buffer1Addr = pxDescriptor->pucEthernetBuffer;
  desc->ControlBufferSize = (pxDescriptor->xDataLength & 0x1FFF);
  desc->Status |=
      (1U << 31) |  // Hand the descriptor back to the DMA.
      (1U << 30) |  // Trigger an interrupt on completion.
      (1U << 29) |  // Descriptor contains the first segment of a packet.
      (1U << 28) |  // Descriptor contains the last segment of a packet.
      (3U << 22) |  // Enable full hardware checksumming.
      (1U << 20);  // Second address chained.

  // Check to see if the peripheral is suspended; kick it if necessary.
  if((ETH->DMASR & ETH_DMASR_TPS) == ETH_DMASR_TPS_Suspended) {
    ETH->DMACHTDR = (uint32_t)desc;  // Poll this transmit descriptor next.
    //ETH->DMASR |= (1U << 2);  // Clear the underflow status.
    ETH->DMATPDR = 0;  // Issue poll request to get to a running state.
  }

  iptraceNETWORK_INTERFACE_TRANSMIT();  // Call the trace macro for debugging.

  // Free the descriptor, but not the buffer, and send it back to the IP stack.
  pxDescriptor->pucEthernetBuffer = NULL;
  vReleaseNetworkBufferAndDescriptor(pxDescriptor);

  return pdTRUE;
}

// This task gets notifications from the receive interrupt. Note that multiple
// RX interrupts are not nested. If a packet is received while servicing the
// Ethernet interrupt, no followup interrupt will fire. It is therefore
// important to process all available packets each time this task receives a
// notification.
static void rxTask(void *pvParameters) {
  ETH_DmaDesc *rx_desc;
  int next_idx = 0;
  uint8_t *pucTemp;
  NetworkBufferDescriptor_t *pxDescriptor;
  IPStackEvent_t xRxEvent;

  for(;;) {
    // Block on the availability of a reception notification.
    ulTaskNotifyTake( pdFALSE, portMAX_DELAY );

    // Iterate over all CPU-owned RX descriptors.
    while(rx_desc = ETH_GetNextDesc(rx_dma_desc,
        &next_idx, ipconfigNUM_RX_DESCRIPTORS, true)) {
      // Get a new buffer that's big enough to receive another packet. It'll be
      // released when it's processed by the IP stack.
      pxDescriptor = pxGetNetworkBufferWithDescriptor(FRAME_PADDED, 0);

      // Make sure we're not trying to write to a buffer we didn't get. Hitting
      // this drops the packet and releases the buffer.
      if(!pxDescriptor) {
        vReleaseNetworkBuffer(rx_desc->Buffer1Addr); // Release the buffer.
        continue;
      }

      // Pointer swap for the Ethernet buffers.
      pucTemp = pxDescriptor->pucEthernetBuffer;
      pxDescriptor->pucEthernetBuffer = rx_desc->Buffer1Addr;
      pxDescriptor->xDataLength = ((rx_desc->Status >> 16) & 0x1FFF);
      rx_desc->Buffer1Addr = pucTemp;

      // ipBUFFER_PADDING bytes before the buffer is a pointer to the IP stack's
      // descriptor associated with that buffer. The example code swaps them,
      // and therefore we do too, but only the first statement seems strictly
      // necessary.
      *((NetworkBufferDescriptor_t**)
          (pxDescriptor->pucEthernetBuffer - ipBUFFER_PADDING)) = pxDescriptor;
      *((NetworkBufferDescriptor_t**)
          (rx_desc->Buffer1Addr - ipBUFFER_PADDING)) = (NetworkBufferDescriptor_t*)rx_desc;

      // Release the DMA descriptor back to DMA.
      rx_desc->Status |= (1u << 31);

      // Try to notify the IP stack of what's just happened.
      xRxEvent.eEventType = eNetworkRxEvent;  // We've received a packet.
      xRxEvent.pvData = (void*) pxDescriptor;  // This is the descriptor.
      if( xSendEventStructToIPTask(&xRxEvent, 0) == pdFALSE) {
        // We couldn't send the packet to the IP stack.
        vReleaseNetworkBufferAndDescriptor( pxDescriptor );
        iptraceETHERNET_RX_EVENT_LOST();
      } else {
        // Message successfully handed off; call the trace macros to log it.
        iptraceNETWORK_INTERFACE_RECEIVE();
      }
    }
  }
}

// This task gets notifications from the transmit interrupt. It releases network
// buffers associated with the now-free descriptors.
static void txTask(void *pvParameters) {
  static int next_tx = 0;
  ETH_DmaDesc *tx_desc;

  for(;;) {
    // Block on transmission notifications.
    ulTaskNotifyTake( pdFALSE, portMAX_DELAY );

    // Find the descriptor of the just-transmitted packet.
    tx_desc = ETH_GetNextDesc(tx_dma_desc,
        &next_tx, ipconfigNUM_TX_DESCRIPTORS, true);

    // Don't need to do a null check here; vReleaseNetworkBuffer() does that.
    vReleaseNetworkBuffer(tx_desc->Buffer1Addr);
    tx_desc->Buffer1Addr = NULL;
  }
}

// We've configured our peripheral to fire on "normal" Ethernet interrupts. The
// handler checks for passed-to-CPU descriptors, hands control of the associated
// buffers back to the IP stack.
void ETH_IRQHandler(void) {
  BaseType_t task_woken;

  // Notify the RX task that a packet is available.
  if(ETH->DMASR & (1U << 6)) {
    vTaskNotifyGiveFromISR(rx_task_handle, &task_woken);
  }

  // Notify the TX task that a packet has been transmitted.
  if(ETH->DMASR & 1U) {
    vTaskNotifyGiveFromISR(tx_task_handle, &task_woken);
  }

  ETH->DMASR |= (1U << 16);  // Clear normal interrupts.
  portYIELD_FROM_ISR(task_woken);  // Jump back to the deferred processing task.
}
