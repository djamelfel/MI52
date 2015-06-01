#include <stm32f4xx.h>
/* for HAL functions */
#include <stm32f4xx_hal.h>
#include <stdio.h>
//#include <stdlib.h>

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "led.h"

void vTask1( void *pvParameters ) {
    /* Variables can be declared just as per a normal function. Each instance
    of a task created using this function will have its own copy of the
    iVariableExample variable. This would not be true if the variable was
    declared static – in which case only one copy of the variable would exist
    and this copy would be shared by each created instance of the task. */
    char * TaskName;
    uint32_t delay;
    TickType_t count, count_prev;

    /* get taskname */
    TaskName = pcTaskGetTaskName( NULL );

    /* A task will normally be implemented as an infinite loop. */
    for( ;; ) {
        /* The code to implement the task functionality will go here. */
        count = xTaskGetTickCount();
        printf("%s, Tick=%d\r\n",TaskName,count); // '\n' force flush if line buffering
        fflush (stdout);
    }
    /* Should the task implementation ever break out of the above loop
    then the task must be deleted before reaching the end of this function.
    The NULL parameter passed to the vTaskDelete() function indicates that
    the task to be deleted is the calling (this) task. */
    vTaskDelete( NULL );
}

void vTask2( void *pvParameters ) {
    uint32_t i;
    for( ;; ) {
        LED_ON();
        for (i=0;i<5000000;i++);
        LED_OFF();
        for (i=0;i<5000000;i++);
    }
    vTaskDelete( NULL );
}


