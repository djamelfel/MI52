#include <stm32f4xx.h>
#include "uart.h"

#define LED_PIN         5
#define LED_ON()        GPIOA->BSRR = (1 << LED_PIN)
#define LED_OFF()       GPIOA->BSRR = ((1 << LED_PIN)<<16)

void set_clk(void);

int main(void) {
    /* set clock on HSE and max speed for core and buses */
    set_clk();
    set_USART2();

    /* Configure GPIOA pin 5 as output */
    GPIOA->MODER |= (1 << (LED_PIN << 1));
    LED_OFF();

    USART_TypeDef* uart = USART2;

#ifdef POLLING
    for(;;) {
        int len = UART_Receive(uart, data, BUFFER_SIZE, 0xffffff);
        UART_Transmit(uart, data, len);
    }
#endif

#ifdef INTERUPT
    for(;;) {
        UART_Receive_IT(uart, data, BUFFER_SIZE);
        while(uart_Device.state);
        UART_Transmit_IT(uart, data, BUFFER_SIZE);
    }
#endif

#ifdef DMA
    for(;;) {
        UART_Receive_DMA(uart, data, BUFFER_SIZE);
        while(uart_Device.state);
        UART_Transmit_DMA(uart, data, BUFFER_SIZE);
    }
#endif
    return 1;
}

/* set clock for nucleo board :
 * HSE (8MHz)
 * Fvco = 336 MHZ
 * Fcore = 84MHz
 * FAHB1 = 84MHz
 * FAPB1 = 42MHz
 * FAPB2 = 84MHz
 */

void set_clk(void) {
    /* define port instance */
    RCC_TypeDef * rcc = RCC;
    FLASH_TypeDef * flash = FLASH;

    /* enable clock using HSE instead of HSI*/
    rcc->CR |= RCC_CR_HSEON ;	/* 1<<16 */
    /* wait for HSE to be stable */
    while ((rcc->CR & RCC_CR_HSERDY) == 0) ;

    /* PLL configuration */
    rcc->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLQ );
    rcc->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE
                        | (RCC_PLLCFGR_PLLM & (8<<0))
                        | (RCC_PLLCFGR_PLLN & (336<<6))
                        | (RCC_PLLCFGR_PLLP & (1 << 16))
                        | (RCC_PLLCFGR_PLLQ & (7<<24));
    /* PLL enable */
    rcc->CR |= RCC_CR_PLLON;
    /* wait for the PLL to be stable */
    while ((rcc->CR & RCC_CR_PLLRDY) == 0) ;

    /* increase the flash WS first */
    flash->ACR = (flash->ACR & (~FLASH_ACR_LATENCY))|FLASH_ACR_LATENCY_2WS ;

    /* modify CPU clock sources */
    rcc->CFGR = (rcc->CFGR & (~RCC_CFGR_SW))| RCC_CFGR_SW_PLL ;
    /* modify AHB divisor (no DIV)*/
    rcc->CFGR = (rcc->CFGR & (~RCC_CFGR_HPRE))| RCC_CFGR_HPRE_DIV1 ;
    /* Modify APB1 (low speed) divisor DIV2 */
    rcc->CFGR = (rcc->CFGR & (~RCC_CFGR_PPRE1))| RCC_CFGR_PPRE1_DIV2 ;
     /* Modify APB2 (high speed) no div */
    rcc->CFGR = (rcc->CFGR & (~RCC_CFGR_PPRE2))| RCC_CFGR_PPRE2_DIV1 ;

    SystemCoreClockUpdate(); //SystemCoreClock = 84000000; ((HSE_VALUE / PLLM) / PLLP) * PLLN / HPRE_DIVN

}
