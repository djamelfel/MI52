/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UART_H
#define UART_H

#include <stm32f4xx_hal.h>

#ifdef __cplusplus
 extern "C" {
#endif

/* UART handler declaration */
UART_HandleTypeDef UartHandle;

void set_USART2(void);


#ifdef __cplusplus
}
#endif

#endif /* UART_H */

