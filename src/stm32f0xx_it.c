#include "stm32f0xx_it.h"
#include "can_api.h"
#include <stm32f0xx_ll_gpio.h>
#include <stm32f0xx_ll_exti.h>

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
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0))
    {
        char response[MAX_JSON_LENGTH] = {0};
        (void)can_get(response);

        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    }
}
