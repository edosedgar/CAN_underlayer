#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_usart.h"

#include "can_api.h"
#include "can_callbacks.h"

#include "xprintf.h"

char
usart_getc(void) {
        uint8_t byte;

        while (!LL_USART_IsActiveFlag_RXNE(USART1))
        byte = LL_USART_ReceiveData8(USART1);

        return byte;
}

void
usart_putc(char symbol) {
        LL_USART_TransmitData8(USART1, symbol);
        while (!LL_USART_IsActiveFlag_TC(USART1));
}

void
config_USART() {
        /*
         * GPIO Init
         */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        //USART1_TX
        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_1);
        LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
        //USART1_RX
        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_1);
        LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
        /*
         * UART Setting
         */
        //USART Set clock source
        LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
        LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
        //USART Setting
        LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
        LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);
        LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
        LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);
        LL_USART_SetTransferBitOrder(USART1, LL_USART_BITORDER_LSBFIRST);
        LL_USART_SetBaudRate(USART1, SystemCoreClock,
                             LL_USART_OVERSAMPLING_16, 115200);
        LL_USART_Enable(USART1);
        while (!(LL_USART_IsActiveFlag_TEACK(USART1) &&
                 LL_USART_IsActiveFlag_REACK(USART1)));
        /*
         * xprintf setting
         */
        xdev_out(usart_putc);
        xdev_in(usart_getc);
        return;
}

/**
  * System Clock Configuration
  * The system Clock is configured as follow :
  *    System Clock source            = PLL (HSI/2)
  *    SYSCLK(Hz)                     = 48000000
  *    HCLK(Hz)                       = 48000000
  *    AHB Prescaler                  = 1
  *    APB1 Prescaler                 = 1
  *    HSI Frequency(Hz)              = 8000000
  *    PLLMUL                         = 12
  *    Flash Latency(WS)              = 1
  */

void
config_RCC() {
        /* Set FLASH latency */
        LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

        /* Enable HSI and wait for activation*/
        LL_RCC_HSI_Enable();
        while (LL_RCC_HSI_IsReady() != 1);

        /* Main PLL configuration and activation */
        LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2,
                                    LL_RCC_PLL_MUL_12);

        LL_RCC_PLL_Enable();
        while (LL_RCC_PLL_IsReady() != 1);

        /* Sysclk activation on the main PLL */
        LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
        LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
        while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

        /* Set APB1 prescaler */
        LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

        /* Set systick to 1ms */
        SysTick_Config(48000000/1000);

        /* Update CMSIS variable (which can be updated also
         * through SystemCoreClockUpdate function) */
        SystemCoreClock = 48000000;
}

int
main(void) {
        config_RCC();
        config_USART();

        (void)can_do_setup(my_setup);
        //(void)can_add_get("PC8", get_PC8);
        //(void)can_do_loop(my_loop);

        while (1);
        return 0;
}
