/**
  ******************************************************************************
  * @file    src/main.c
  * @author
  * @version
  * @date    20-april-2015
  * @brief   free RTOS example
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
/* libc include */
#include <stdlib.h>
#include <stdio.h>
/* CMSIS and STM32 definition */
#include <stm32f4xx.h>
/* for HAL functions */
#include <stm32f4xx_hal.h>
/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* application specific */
#include "led.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char stdio_buffer[BUFSIZ];    /** BUFSIZ defined in newlib at 1024 bytes */
/* Private function prototypes -----------------------------------------------*/
/* The task functions. */
extern void vTask1( void *pvParameters );
extern void vTask2( void *pvParameters );

static void SystemClock_Config(void);
static void Error_Handler(void);

extern void set_EXTI (void);
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void ) {
    int status;
    /* GPIO structure */
    static GPIO_InitTypeDef  GPIO_InitStruct;

/* ------------------------ Harware Initialisation ----------------------------*/
    #define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003)
    NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );
    NVIC_SetPriority(SysTick_IRQn,0xF);    /* lowest Priority */
    /* STM32F4xx HAL library initialization:
        - Configure the Flash prefetch, instruction and Data caches
        - Configure the Systick to generate an interrupt each 1 msec
        - Set NVIC Group Priority to 4
        - Global MSP (MCU Support Package) initialization
    */
    HAL_Init();
    /* Configure the system clock to 84 MHz */
    SystemClock_Config();

    /* configure GPIOA PIN5 to drive the led  */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin =     GPIO_PIN_5;
    GPIO_InitStruct.Mode =     GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull =     GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* activate the USART2 for console */
    set_USART2();
    set_EXTI();
    /* -------------------------for stdlib ------------------------------------------*/
    srand(13); /* init seed for rand() */
    /* set buffering on stdio */
    /*_IOFBF     full buffering, _IOLBF     line buffering, _IONBF     no buffering */
    setvbuf( stdout, stdio_buffer, _IOFBF, (size_t) BUFSIZ);
    fflush (stdout);

    /* -------------------------Create Tasks ------------------------------------------*/

    /* Create one of the two tasks. Note that a real application should
    check the return value of the xTaskCreate() call to ensure the task was
    created successfully. */
    xTaskCreate(
        vTask1, /* Pointer to the function that implements the task. */
        (const signed char *) "Task 1",/* Text name for the task. This is to facilitate
                debugging only. */
        configMINIMAL_STACK_SIZE,/* Stack depth  */
        NULL,/* We are not using the task parameter. */
        2,/* This task will run at priority 2 */
        NULL  /* We are not going to use the task handle. */
        );
    /* Create an other task in exactly the same way and at the same
    priority. */
    xTaskCreate( vTask1, (const signed char *)"Task 2", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
    /* Create the other task in exactly the same way and a higher priority */
    xTaskCreate( vTask2, (const signed char *)"Task 3",configMINIMAL_STACK_SIZE, NULL, 3, NULL );

    /* Start the scheduler so the tasks start executing. */
    vTaskStartScheduler();
    /* If all is well then main() will never reach here as the scheduler
    will now be running the tasks. If main() does reach here then it is
    likely that there was insufficient heap memory available for the idle
    task to be created. */
    for( ;; );
}



void vApplicationTickHook( void ) {
    HAL_IncTick();
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void ) {
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void ) {
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName ) {
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/



/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void) {
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
        clocked below the maximum system frequency, to update the voltage scaling value
        regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* Enable HSI Oscillator and activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
    RCC_OscInitStruct.HSICalibrationValue = 0x10;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
          Error_Handler();
    }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
          Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void) {
  while(1) {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1) {
  }
}
#endif

