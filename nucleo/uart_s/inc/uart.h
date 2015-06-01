#ifndef UART_H
#define UART_H
#define INTERUPT

#define BUFFER_SIZE     70
/* Definition for USARTx Pins */
#define USART2_TX_PIN   2
#define USART2_TX_AF    7
#define USART2_RX_PIN   3
#define USART2_RX_AF    7

/* define the priority grouping for the cortex-M implemented in STM32 */
#define NVIC_PRIORITYGROUP_4            ((uint32_t)0x00000003)

/* function definition */
void set_USART2(void);
void UART_Transmit(USART_TypeDef * uart, uint8_t * data, uint32_t len);
uint32_t UART_Receive(USART_TypeDef * uart, uint8_t * data, uint32_t len, uint32_t time_out);
uint32_t UART_Transmit_IT(USART_TypeDef * uart, uint8_t * data, uint32_t len);
uint32_t UART_Receive_IT(USART_TypeDef * uart, uint8_t * data, uint32_t len);
void USART2_IRQHandler(void);
uint32_t UART_Transmit_DMA(USART_TypeDef * uart, uint8_t * data, uint32_t len);
uint32_t UART_Receive_DMA(USART_TypeDef * uart, uint8_t * data, uint32_t len);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);

uint8_t data[BUFFER_SIZE];

typedef struct{
    USART_TypeDef  * instance; /* address of the USART registers */
    uint32_t state; 	   /* can be used as a lock for transmission and reception, error, ... */
    uint8_t * TxBuffer;	   /* application buffer for transmission */
    uint8_t * RxBuffer; 	   /* application buffer for reception */
    uint32_t TxSize;	   /* number of data to transmit */
    uint32_t RxSize;	   /* number of data to receive */
    uint32_t TxCount; 	   /* number of data already transmitted */
    uint32_t RxCount; 	   /* number of data already received */
} UART_Device;

UART_Device uart_Device;
#endif /*UART_H*/
