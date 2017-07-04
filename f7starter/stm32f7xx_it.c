#include "stm32f7xx_it.h"

void NMI_Handler(void){}

void HardFault_Handler(void){while (1);}

void MemManage_Handler(void){while (1);}

void BusFault_Handler(void){while (1);}

void UsageFault_Handler(void){while (1);}

__weak void SVC_Handler(void){}

void DebugMon_Handler(void){}

__weak void PendSV_Handler(void){}

__weak void SysTick_Handler(void){}
