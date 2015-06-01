#include <stm32f4xx.h>
#include "uart.h"

void set_USART2(void) {
     /*-------------- clock enable --------------- */
     /* Enable GPIOA clock */
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
     /* Enable GPIOA clock */
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
     /* Enable USARTx clock */
     RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

#ifdef DMA
     /* enable DMA1 clock */
     RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; 
#endif

     /*-------------- Pin muxing configuration --------------- */
     /* Configure alternate function for UART2 RX and TX */
     GPIOA->MODER &= ~(0xf << 4);
     GPIOA->MODER |= (2 << 2*USART2_TX_PIN);
     GPIOA->MODER |= (2 << 2*USART2_RX_PIN);

     /* set up alternate function 7 for PA2 (TX)  */
     GPIOA->AFR[0] &= ~(0xf << 4*USART2_TX_PIN);
     GPIOA->AFR[0] |= (7 << 4*USART2_TX_PIN);

     /* set up alternate function 7 for PA3 (RX) */
     GPIOA->AFR[0] &= ~(0xf << (4*USART2_RX_PIN));
     GPIOA->AFR[0] |= (7 << (4*USART2_RX_PIN));

     /*-------------- UART parameters configuration ---------- */
     /* Configure USART CR1 (TX, RX and overslampling) */
     USART2->CR1 |= USART_CR1_RE;  // RX
     USART2->CR1 |= USART_CR1_TE;  // TX
     USART2->CR1 |= USART_CR1_OVER8; // overslamping

     /* Configure USART CR2 (STOP bits,) */
     USART2->CR2 &= ~USART_CR2_STOP; // STOP bits

     /* Configure USART CR3 */
     USART2->CR3 &= ~USART_CR3_RTSE; // No flow control

     /* USART BRR Configuration */
     USART2->BRR = 0x2D5;     // Defined freq at 42MHz

     /* enable USART (CR1) */
     USART2->CR1 |= USART_CR1_UE;

#ifdef DMA
     /*-------------- DMA parameters configuration for USART2 --------------- */
     /* set up the DMA chanel 4 stream 5 for USART2 RX */
     DMA1_Stream5->CR &= ~DMA_SxCR_EN;
     DMA1_Stream5->CR |= DMA_SxCR_CHSEL_2;
     DMA1_Stream5->CR &= ~DMA_SxCR_MSIZE;      // 8 bits transfer
     DMA1_Stream5->CR &= ~DMA_SxCR_PSIZE;      // 8 bits transfer
     DMA1_Stream5->CR |= DMA_SxCR_PL_0;        // priority medium
     DMA1_Stream5->CR |= DMA_SxCR_MINC;        // memory increment
     DMA1_Stream5->CR &= ~DMA_SxCR_PINC;       // no memory increment
     DMA1_Stream5->CR |= DMA_SxCR_TCIE;        // interrupt on transfer complete
     DMA1_Stream5->CR |= DMA_SxFCR_DMDIS;      // direct mode

     /* set up the DMA chanel 4 stream 6 for USART2 TX */
     DMA1_Stream6->CR &= ~DMA_SxCR_EN;
     DMA1_Stream6->CR |= DMA_SxCR_CHSEL_2;
     DMA1_Stream6->CR &= ~DMA_SxCR_MSIZE;     // 8 bits transfer
     DMA1_Stream6->CR &= ~DMA_SxCR_PSIZE;      // 8 bits transfer
     DMA1_Stream6->CR |= DMA_SxCR_PL_0;        // priority medium
     DMA1_Stream6->CR |= DMA_SxCR_MINC;        // memory increment
     DMA1_Stream6->CR &= ~DMA_SxCR_PINC;       // no memory increment
     DMA1_Stream6->CR |= DMA_SxCR_TCIE;        // interrupt on transfer complete
     DMA1_Stream6->CR |= DMA_SxFCR_DMDIS;      // direct mode
#endif

#ifdef INTERUPT
     /*-------------- interrupt --------------- */
     /* ------------- Desactivate interruption ------------------*/
     USART2->CR1 &= ~(USART_CR1_TXEIE);
     USART2->CR1 &= ~(USART_CR1_TCIE);
     USART2->CR1 &= ~(USART_CR1_RXNEIE);

     /* set up priority and autorize interrupt */
     NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
     NVIC_SetPriority(USART2_IRQn,0x30);
     NVIC_EnableIRQ(USART2_IRQn);

     __enable_irq();
#endif

#ifdef DMA
     NVIC_SetPriority(DMA1_Stream5_IRQn,0x30);
     NVIC_EnableIRQ(DMA1_Stream5_IRQn);
     NVIC_SetPriority(DMA1_Stream6_IRQn,0x30);
     NVIC_EnableIRQ(DMA1_Stream6_IRQn);

     __enable_irq();
#endif
}

/* user functions using polling method */
void UART_Transmit(USART_TypeDef * uart, uint8_t * data, uint32_t len) {
     for(int i=0; i<len; i++) {
          while( (uart->SR & USART_SR_TXE) == 0);
          uart->DR = data[i];
     }
}

uint32_t UART_Receive(USART_TypeDef * uart, uint8_t * data, uint32_t len, uint32_t time_out) {
     int len_count = 0;
     while(time_out != 0 && len_count < len) {
          while(!(uart->SR & USART_SR_RXNE)) {
               time_out--;
          }
          if(time_out != 0) {
               data[len_count++] = uart->DR;
          }
     }
     return len_count;
}

/* user functions using interrupt driven method */
uint32_t UART_Transmit_IT(USART_TypeDef * uart, uint8_t * data, uint32_t len) {
     uart_Device.instance = uart;
     uart_Device.TxBuffer = data;
     uart_Device.TxSize = len;
     uart_Device.TxCount = 0;

     // enable interruption on transmision
     uart_Device.instance->CR1 |= USART_CR1_TXEIE;
}

uint32_t UART_Receive_IT(USART_TypeDef * uart, uint8_t * data, uint32_t len) {
     uart_Device.instance = uart;
     uart_Device.RxBuffer = data;
     uart_Device.RxSize = len;
     uart_Device.RxCount = 0;
     uart_Device.state = 1;

     uart_Device.instance->CR1 |= USART_CR1_RXNEIE;
}

/* USART2 ISR */
void USART2_IRQHandler(void){
     // data is transmit to shift register
     if (((uart_Device.instance->SR & USART_SR_TXE) == USART_SR_TXE) && ((uart_Device.instance->CR1 & USART_CR1_TXEIE) == USART_CR1_TXEIE)) {
          // stay data to send
          if(uart_Device.TxSize > uart_Device.TxCount) {
               uart_Device.instance->DR = uart_Device.TxBuffer[uart_Device.TxCount++];
          }
          else {
               // deactivate interrupt generated by data send to shift register
               uart_Device.instance->CR1 &= ~(USART_CR1_TXEIE);
               // an interrupt will be generated whenever TC=1 (transfer complete)
               uart_Device.instance->CR1 |= USART_CR1_TCIE;
          }
     }
     // last data is send from shift register
     else if ((uart_Device.instance->SR & USART_SR_RXNE) == USART_SR_RXNE && (uart_Device.instance->CR1 & USART_CR1_RXNEIE) == USART_CR1_RXNEIE) {
          if(uart_Device.RxSize > uart_Device.RxCount) {
               uart_Device.RxBuffer[uart_Device.RxCount++] = uart_Device.instance->DR;
          }
          else {
               uart_Device.instance->CR1 &= ~(USART_CR1_RXNEIE);
               uart_Device.state = 0;
          }
     }
     else if (((uart_Device.instance->SR & USART_SR_TC) == USART_SR_TC) && (uart_Device.instance->CR1 & USART_CR1_TCIE) == USART_CR1_TCIE) {
          // deactivate interrupt generated when transfer are completed
          uart_Device.instance->CR1 &= ~(USART_CR1_TCIE);
     }
}

/* user functions using DMA and interrupt method */
uint32_t UART_Transmit_DMA(USART_TypeDef * uart, uint8_t * data, uint32_t len) {
     DMA1_Stream6->M0AR = (uint32_t)data;       // copy data adress to DMA control register
     DMA1_Stream6->PAR = (uint32_t)&(uart->DR); // copy data address to
     DMA1_Stream6->CR |= DMA_SxCR_DIR_0;
     DMA1_Stream6->NDTR = len;
     uart_Device.instance = uart;
     uart_Device.instance->CR3 |= USART_CR3_DMAT;
     uart_Device.instance->SR &= ~USART_SR_TC;

     DMA1_Stream6->CR |= DMA_SxCR_EN;
}

uint32_t UART_Receive_DMA(USART_TypeDef * uart, uint8_t * data, uint32_t len) {
     DMA1_Stream5->M0AR = (uint32_t)data;
     DMA1_Stream5->PAR = (uint32_t)&(uart->DR);
     DMA1_Stream5->CR &= ~DMA_SxCR_DIR;
     DMA1_Stream5->NDTR = len;
     uart_Device.instance = uart;
     uart_Device.instance->CR3 |= USART_CR3_DMAR;
     uart_Device.instance->SR &= ~USART_SR_TC;
     uart_Device.state = 1;

     DMA1_Stream5->CR |= DMA_SxCR_EN;
}

/* DMA1 Stream 5 ISR */
void DMA1_Stream5_IRQHandler(void) {
     DMA1->HIFCR = DMA_HIFCR_CTCIF5;
     DMA1_Stream6->CR &= ~DMA_SxCR_TCIE;
     uart_Device.instance->CR3 &= ~USART_CR3_DMAR;
     uart_Device.state = 0;
}

/* DMA1 Stream 6 ISR */
void DMA1_Stream6_IRQHandler(void) {
     DMA1->HIFCR = DMA_HIFCR_CTCIF6;
     DMA1_Stream6->CR &= ~DMA_SxCR_TCIE;
     uart_Device.instance->CR3 &= ~USART_CR3_DMAT;
}
