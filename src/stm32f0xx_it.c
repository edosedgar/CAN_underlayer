#include "stm32f0xx_it.h"
#include <stm32f0xx_ll_gpio.h>

void
NMI_Handler(void) {
}

void
HardFault_Handler(void) {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
    while (1);
}

void
SVC_Handler(void) {
}

void
PendSV_Handler(void) {
}

void
SysTick_Handler(void) {
}

void
EXTI_0_1_Handler(void) {

}
