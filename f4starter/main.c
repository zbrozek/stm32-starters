// FreeRTOS headers
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_sockets.h"

// STM32 headers
#include "stm32.h"
#include "csp.h"
#include "pin.h"
#include "rcc.h"
#include "spi.h"
#include "ethernet.h"
#include "ethernet_config.h"

// Library headers
#include <stdio.h>
#include <yfuns.h>

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

// Don't actually need to do anything here, this is mostly just an example.
bool PHY_Init(void) {
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

static void exampleTask(void *pvParameters) {
  // Hack to silence compiler warnings about unused pvParameters.
  ( void ) pvParameters;

  // Initialise xNextWakeTime - this only needs to be done once.
  TickType_t xNextWakeTime = xTaskGetTickCount();

  for( ;; ) {
    printf("derp\n");
    // Pauses this task until 1000 ms after its last wakeup.
    vTaskDelayUntil(&xNextWakeTime, 1000);
  }
}

int main()
{
  Rcc rcc;
  rcc.src = eRccSrcPll;
  rcc.hse_bypass = false;
  rcc.pll.src = eRccSrcHse;
  SystemCoreClock = RCC_ClockConfig(&rcc, 168000000);
  
  CSP_EnableCycleCounter();  // Enable debug core for nice timestamps.
  CSP_PrintStartupInfo();  // Cute boot message to assist with debugging.

  xTaskCreate( exampleTask,                     // The function that implements the task.
      "example",                                // The text name assigned to the task - for debug only as it is not used by the kernel.
      configMINIMAL_STACK_SIZE,                 // The size of the stack to allocate to the task.
      NULL,                                     // The parameter passed to the task - not used in this case.
      configLIBRARY_LOWEST_INTERRUPT_PRIORITY,  // The priority assigned to the task.
      NULL );                                   // The task handle is not required, so NULL is passed.

  BaseType_t stack_initialized = FreeRTOS_IPInit( ucIPAddress,
      ucNetMask,
      ucGatewayAddress,
      ucDNSServerAddress,
      ucMACAddress );

  // Set interrupt group priority to 4. Getting this
  // wrong will cause hard faults at context switches.
  NVIC_SetPriorityGrouping(0x300);

  // Start the scheduler. We should never return from here.
  vTaskStartScheduler();

  // Should never get here.
  return 0;
}

const char *pcApplicationHostnameHook(void) {
  const char* hostname = "f4starter";
  return hostname;
}

// This is a good place to turn on a sadness LED.
void vApplicationStackOverflowHook(void){while(1);}

// You could add some simple power-saving code here.
void vApplicationIdleHook(void){}

// This is SysTick - it happens every millisecond.
void vApplicationTickHook(void){
  CSP_UpdateGrossCycleCount();
}
