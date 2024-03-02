// Function definitions for interrupt handlers. There are weak definitions for
// dummy handlers in the startup_*.s file which these functions override.

// Note that interrupt functions can be defined anywhere within the project and
// that it can become ambiguous which is the dominant defintion. Take care to
// avoid multiple overrides. Notably, when using FreeRTOS, do not implement your
// own definitions of SVC_Handler(), PendSV_Handler(), or SysTick_Handler().

void NMI_Handler(void);

void HardFault_Handler(void){while (1);}

void MemManage_Handler(void){while (1);}

void BusFault_Handler(void){while (1);}

void UsageFault_Handler(void){while(1);}

void DebugMon_Handler(void);
