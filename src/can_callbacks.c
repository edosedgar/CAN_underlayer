#include "can_callbacks.h"

#include <stm32f0xx_ll_gpio.h>
#include <stm32f0xx_ll_bus.h>
#include <stm32f0xx_ll_rcc.h>
#include <stm32f0xx_ll_system.h>

/**
  This file is expected to be edited
  by project-makers. Put here definitions
  of all you callback-functions
  */

void
my_setup(void) {
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
        LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
        return;
}

void
my_loop(void) {
        while (1);
        return;
}

int
get_PC8(uint32_t* buf) {
        buf[0] = LL_GPIO_IsOutputPinSet(GPIOC, LL_GPIO_PIN_8);
        LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);
        return 1;
}
