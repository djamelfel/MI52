/* Support files for GNU libc.  Files in the system namespace go here.
   Files in the C namespace (ie those that do not start with an
   underscore) go in .c.  */

#include <_ansi.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include <errno.h>
#include <reent.h>
#include <unistd.h>
#include <sys/wait.h>

#include <stm32f4xx.h>
#include "stm32f4xx_hal.h"
#include "uart.h"

#undef errno
extern int errno;

#define FreeRTOS

#ifdef FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"
#define MAX_STACK_SIZE configTOTAL_HEAP_SIZE
#else
#define MAX_STACK_SIZE 32*0x400	//32ko
#endif

extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));

#ifndef FreeRTOS
  register char * stack_ptr asm("sp");
#endif


extern UART_HandleTypeDef UartHandle;
extern SemaphoreHandle_t uart_tx_complete;
extern SemaphoreHandle_t uart_rx_complete;

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

/* _sbrk called to allocate new bloc of memory to be used by malloc and friends */
caddr_t _sbrk(int incr)
{
	extern char end asm("end");
	extern char _Min_Stack_Size;
	static char *heap_end;
	static char *max_stack;
	char *prev_heap_end,*min_stack_ptr;

	max_stack = &_Min_Stack_Size;		/* from ld script */
	if (heap_end == 0)							/* define first adress for heap */
		heap_end = &end;							/* from ld script */

	prev_heap_end = heap_end;

	/* Use the NVIC offset register to locate the main stack pointer. */
	min_stack_ptr = (char*)(*(unsigned int *)*(unsigned int *)0xE000ED08); /*VTOR*/
	/* Locate the STACK bottom address */
	min_stack_ptr -= MAX_STACK_SIZE;

	if (heap_end + incr > min_stack_ptr)
//	if (heap_end + incr > stack_ptr)
	{
//		write(1, "Heap and stack collision\n", 25);
//		abort();
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}

/*
 * _gettimeofday primitive (Stub function)
 * */
int _gettimeofday (struct timeval * tp, struct timezone * tzp)
{
  /* Return fixed data for the timezone.  */
  if (tzp)
    {
      tzp->tz_minuteswest = 0;
      tzp->tz_dsttime = 0;
    }

  return 0;
}
void initialise_monitor_handles()
{
}

int _getpid(void)
{
	return 1;
}

int _kill(int pid, int sig)
{
	errno = EINVAL;
	return -1;
}
/*
void _exit (int status)
{
	_kill(status, -1);
	while (1) {}
}
*/

/* _write called for standard output */
int _write(int file, char *ptr, int len)
{
//	HAL_UART_Transmit(&UartHandle, ptr, len ,HAL_MAX_DELAY); 
		while (len-->0){
			while (!(USART2->SR & USART_SR_TXE));
				USART2->DR = *ptr++;
	}	
	return len;
}

int _close(int file)
{
	return -1;
}

int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
	return 0;
}

/* _read called for standard input */
int _read(int file, char *ptr, int len)
{
   return len;
}

int _open(char *path, int flags, ...)
{
	/* Pretend like we always fail */
	return -1;
}

int _wait(int *status)
{
	errno = ECHILD;
	return -1;
}

int _unlink(char *name)
{
	errno = ENOENT;
	return -1;
}

int _times(struct tms *buf)
{
	return -1;
}

int _stat(char *file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _link(char *old, char *new)
{
	errno = EMLINK;
	return -1;
}

int _fork(void)
{
	errno = EAGAIN;
	return -1;
}

int _execve(char *name, char **argv, char **env)
{
	errno = ENOMEM;
	return -1;
}

