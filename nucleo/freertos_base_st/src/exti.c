#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"

/* B1 (blue button) on PC13 */
void set_EXTI (void)
{
	SYSCFG_TypeDef *syscfg = SYSCFG;
	EXTI_TypeDef *exti = EXTI;
	GPIO_InitTypeDef   GPIO_InitStructure;

	/* Enable GPIOC clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	/* enable SYSCFGEN clock */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	/* Configure GPIOC pin 13 as input */
	GPIOC->MODER &= ~(3 << (13 << 1));

/* here select PC13 as EXTI interrupt in the SYSCFG registrers and 
 * configure to generate an interrupt on the falling edge of the pin
 */




  /* Enable and set EXTI Interrupt  priority */
	NVIC_SetPriority(EXTI15_10_IRQn, 9);
//	NVIC_EnableIRQ(EXTI15_10_IRQn);

}


void EXTI15_10_IRQHandler(void)
{
}

/* Hal callback function for interrupt if HAL_GPIO_EXTI_IRQHandler is used */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}
