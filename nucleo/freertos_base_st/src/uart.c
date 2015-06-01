#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include "uart.h"
/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static void Error_Handler(void);

/* static initialisation of the init structure */
static DMA_HandleTypeDef USART2_TX_DMA_Handle = {
	.Instance = DMA1_Stream6,
	.Init = {
			.Channel = DMA_CHANNEL_4,   
			.Direction = DMA_MEMORY_TO_PERIPH,  
			.PeriphInc = DMA_PINC_DISABLE,
			.MemInc = DMA_MINC_ENABLE,
			.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
			.MemDataAlignment = DMA_MDATAALIGN_BYTE,
			.Mode = DMA_NORMAL,
			.Priority = DMA_PRIORITY_MEDIUM,
			.FIFOMode = DMA_FIFOMODE_DISABLE, //ENABLE,
			.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL, 	//DMA_FIFO_THRESHOLD_HALFFULL, 
			.MemBurst = DMA_MBURST_SINGLE, 							//with fifo full only otherwise  DMA_MBURST_SINGLE,
			.PeriphBurst = DMA_PBURST_SINGLE,	
		},
};

static DMA_HandleTypeDef USART2_RX_DMA_Handle = {
	.Instance = DMA1_Stream5,
	.Init = {
			.Channel = DMA_CHANNEL_4,   
			.Direction = DMA_PERIPH_TO_MEMORY,  
			.PeriphInc = DMA_PINC_DISABLE,
			.MemInc = DMA_MINC_ENABLE,
			.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
			.MemDataAlignment = DMA_MDATAALIGN_BYTE,
			.Mode = DMA_NORMAL,
			.Priority = DMA_PRIORITY_MEDIUM,
			.FIFOMode = DMA_FIFOMODE_ENABLE,
			.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL, //DMA_FIFO_THRESHOLD_HALFFULL,
			.MemBurst = DMA_MBURST_INC4, // DMA_MBURST_SINGLE,
			.PeriphBurst = DMA_PBURST_SINGLE,	
		},
};


void set_USART2(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*------------ Enable peripherals and GPIO Clocks --------------*/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
  /*---------- reset the USART ------------------ */
  __HAL_RCC_USART2_FORCE_RESET();
	for (uint32_t i = 0; i<100;i++);
  __HAL_RCC_USART2_RELEASE_RESET();

  /*---------Configure peripheral GPIO -----------*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = GPIO_PIN_2;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);   
  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin 			= GPIO_PIN_3;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;    
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*-----------------Configure the UART peripheral ---------------------*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART2 configured as follow:
		- Word Length = 8 Bits
		- Stop Bit = One Stop bit
		- Parity = NONE
		- BaudRate = 115200 baud
		- Hardware flow control disabled (RTS and CTS signals) */

	/* setting of the initialisation structure for UART2 (cf hal_usart.h) */
	UartHandle.Instance          = USART2;   
	UartHandle.Init.BaudRate     = 115200;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode         = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	if(HAL_UART_Init(&UartHandle) != HAL_OK){
		/* Initialization Error */
		Error_Handler(); 
	}

	/* init DMA */
	HAL_DMA_Init(&USART2_TX_DMA_Handle); 
	HAL_DMA_Init(&USART2_RX_DMA_Handle);  
    
  /* Associate the initialized DMA handle to the the UART handle */
	__HAL_LINKDMA(&UartHandle, hdmatx, USART2_TX_DMA_Handle);
  __HAL_LINKDMA(&UartHandle, hdmarx, USART2_RX_DMA_Handle);
//	UartHandle.hdmatx = &USART2_TX_DMA_Handle;	//not sufficient...
//	UartHandle.hdmarx = &USART2_RX_DMA_Handle;


	/*-------------- interrupt --------------- */
	/* autorize interrupt */
	NVIC_SetPriority(USART2_IRQn,8);
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(DMA1_Stream6_IRQn,7);
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	NVIC_SetPriority(DMA1_Stream5_IRQn,7);
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);
//	HAL_NVIC_SetPriority(USART2_IRQn, 8, 1);
//  HAL_NVIC_EnableIRQ(USART2_IRQn);

}


/* callback function called at the end of transmission */
void HAL_UART_TxCpltCallback ( UART_HandleTypeDef *huart)
{
}

void HAL_UART_RxCpltCallback ( UART_HandleTypeDef *huart)
{
}

void HAL_UART_ErrorCallback ( UART_HandleTypeDef *huart)
{
}

/**
  * @brief  This function handles DMA RX interrupt request.  
  * @param  None
  * @retval None    
  */
void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmarx);
}

/**
  * @brief  This function handles DMA TX interrupt request.
  * @param  None
  * @retval None  
  */
void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmatx);
}


/**
  * @brief  This function handles UART interrupt request.  
  * @param  None
  * @retval None  
  */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(& UartHandle);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while(1)
  {
  }
}




