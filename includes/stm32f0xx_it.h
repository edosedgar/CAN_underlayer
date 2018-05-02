#ifndef __STM32F0XX_IT_H
#define __STM32F0XX_IT_H

void NMI_Handler(void);

void HardFault_Handler(void);

void SVC_Handler(void);

void PendSV_Handler(void);

void SysTick_Handler(void);

void EXTI_0_1_Handler(void);

#endif
