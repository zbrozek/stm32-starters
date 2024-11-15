// FreeRTOS headers
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_sockets.h"

// STM32 headers
#include "stm32f7xx.h"
#include "core_cm7.h"
#include "csp.h"
#include "pin.h"
#include "rcc.h"
#include "rng.h"
#include "ethernet.h"
#include "ethernet_config.h"

// Library headers
#include <stdio.h>
#include <stdbool.h>
#include <yfuns.h>

// Global variable declarations.
uint32_t SystemCoreClock = 8000000;  // Internal RC oscillator is 8 MHz.

// Don't actually need to do anything here, this is mostly just an example.
bool PHY_Init(void)
{
  printf("Waiting for Ethernet link-up.\n");
  // Wait until link is up
  uint16_t resp = 0;
  for(;;) {
    ETH_SmiTransfer(0, 1, &resp, false);
    if(resp & (1U << 2)) {break;}
    vTaskDelay(100);
  }
  printf("Link up!\n");
  return true;
}

// Demo task to run with the scheduler.
static void exampleTask(void *pvParameters)
{
  // Hack to silence compiler warnings about unused pvParameters.
  ( void ) pvParameters;

  // Initialise xNextWakeTime - this only needs to be done once.
  TickType_t xNextWakeTime = xTaskGetTickCount();

  // Tasks must run forever without return.
  for(uint32_t count = 0; true; count++) {
    printf("%d: derp\n", count);
    // Pauses this task until 1000 ms after its last wakeup.
    vTaskDelayUntil(&xNextWakeTime, 1000);
  }
}

// Main function. Expected to set up clocks, create tasks, and launch the
// preemptive scheduler. Should never return.
int main( void )
{
  // Enable various processor speed-ups - dangerous.
  /*FLASH->ACR |= FLASH_ACR_ARTEN;
  SCB_EnableICache();
  SCB_EnableDCache();*/

  Rcc rcc;
  rcc.src = eRccSrcPll;
  rcc.hse_bypass = false;
  rcc.pll.src = eRccSrcHse;
  SystemCoreClock = RCC_ClockConfig(&rcc, 168000000);

  CSP_EnableCycleCounter();  // Enable CM7 debug core for nice timestamps.
  CSP_PrintStartupInfo();  // Cute boot message to assist with debugging.

  // Initialize our demo task.
  xTaskCreate( exampleTask,      // The function that implements the task.
      "example",                 // The text name assigned to the task - for debug only as it is not used by the kernel.
      configMINIMAL_STACK_SIZE,  // The size of the stack to allocate to the task.
      NULL,                      // The parameter passed to the task - not used in this case.
      tskIDLE_PRIORITY,          // The priority assigned to the task.
      NULL );                    // The task handle is not required, so NULL is passed.

  // Initialize the IP stack. PHY_Init() called from ethernet.c
  // Board IP: 192.168.0.30
  uint8_t MacAddress[6];
  ETH_GetMacAddress(MacAddress);
  BaseType_t stack_initialized = FreeRTOS_IPInit( ucIPAddress,
      ucNetMask,
      ucGatewayAddress,
      ucDNSServerAddress,
      MacAddress );

  // Print out the MAC address.
  printf("MAC address is ");
  for(uint8_t i = 0; i < 5; i++) {
    printf("%02X:", MacAddress[i]);
  }
  printf("%02X\n", MacAddress[5]);

  // Print out the IP address.
  printf("IP address is ");
  for(uint8_t i = 0; i < 3; i++) {
    printf("%d.", ucIPAddress[i]);
  }
  printf("%d\n", ucIPAddress[3]);

  // Set interrupt group priority to 4 (argument is 3). Getting this wrong will
  // cause hard faults at context switches.
  NVIC_SetPriorityGrouping(3);

  // Start the scheduler. We should never return from here.
  vTaskStartScheduler();

  // Should never get here.
  return 0;
}

// Device host name used by FreeRTOS+TCP, e.g., during DHCP requests.
const char *pcApplicationHostnameHook(void)
{
  const char* hostname = "f7starter";
  return hostname;
}

// This is a good place to turn on a sadness LED.
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
  while(1);
}

// You could add some simple power-saving code here.
void vApplicationIdleHook(void)
{
}

// This is called by SysTick - it happens every millisecond.
void vApplicationTickHook(void)
{
  CSP_UpdateGrossCycleCount();
}

// Called by FreeRTOS+TCP when getting a new sequence number. Uses the hardware
// RNG to improve randomness.
extern uint32_t ulApplicationGetNextSequenceNumber
(
  uint32_t ulSourceAddress,
  uint16_t usSourcePort,
  uint32_t ulDestinationAddress,
  uint16_t usDestinationPort
)
{
  uint32_t random_value;
  RNG_GetRand32(&random_value);
  return random_value;
}

// Supply a random number to FreeRTOS+TCP stack.
BaseType_t xApplicationGetRandomNumber(uint32_t *pulNumber)
{
    return RNG_GetRand32(pulNumber);
}

// Workaround to avoid programs with printf from failing without a debugger.
int (putchar)(int c) {
  if (c != EOF) {
    unsigned char uc = c;
    if(CoreDebug->DHCSR & 1U) {  // Debugger is connected, so use semihosting.
      if (__write(_LLIO_STDOUT, &uc, 1) == 1) {
        return c;
      }
    } else {  // Debugger isn't connected, so prints go to the void.
      return c;
    }
  }
  return EOF;
}

void SystemInit(void)
{
  // Enable floating point unit.
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));

  // Set the vector table start address to the beginning of flash. Note that
  // this location may change if the application ends up launched by a
  // bootloader rather than the system loader.
  SCB->VTOR = FLASH_BASE;
}
