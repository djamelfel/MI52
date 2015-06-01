/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LED_H
#define LED_H

#include <stm32f4xx.h>

#ifdef __cplusplus
 extern "C" {
#endif

#define LED_ON() GPIOA->BSRR = (1 << 5)
#define LED_OFF() GPIOA->BSRR = ((1 << 5)<<16)


#ifdef __cplusplus
}
#endif

#endif /* LED_H */

