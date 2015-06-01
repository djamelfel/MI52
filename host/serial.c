#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>

#define BUFF_SIZE 256

int main(void) {
  int i;
  int fd, fd2 ;
  struct termios options;
  char buff[BUFF_SIZE] ;
  ssize_t read_res,wres,total_len;

  /* opnen the serial port */
  fd = open(
    // the name of the serial port
    // as a c-string (char *)
    // eg. /dev/ttys0
    "/dev/tty.usbmodem643",
    // configuration options
    // O_RDWR - we need read
    //     and write access
    // O_CTTY - prevent other
    //     input (like keyboard)
    //     from affecting what we read
    // O_NDELAY - do not block
    O_RDWR //  | O_NOCTTY | O_NDELAY
  );
  if(fd == -1) {
    printf("cant open /dev/ttyusbmodem26443\n");
    return -1;
  }


  /* Serial line configuration */

  // get the current settings of the
  // serial port
  tcgetattr(fd, &options);

  // set the read and write speed to
  // 115200 BAUD
  // All speeds can be prefixed with B
  // as a settings.
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  // now to set the other settings
  // here we will have two examples.
  // The first will be no parity,
  // the second will be odd parity.
  // Both will assume 8-bit words

  // PARENB is enable parity bit
  // so this disables the parity bit
  options.c_cflag &= ~PARENB;

  // CSTOPB means 2 stop bits
  // otherwise (in this case)
  // only one stop bit
  options.c_cflag &= ~CSTOPB;

  // CSIZE is a mask for all the
  // data size bits, so anding
  // with the negation clears out
  // the current data size setting
  options.c_cflag &= ~CSIZE;

  // CS8 means 8-bits per work
  options.c_cflag |= CS8;


  /* raw output */
  options.c_oflag &= ~OPOST;

  // apply the settings to the serial port
  // TCSANOW means apply the changes now
  // other valid options include:
  //    TCSADRAIN - wait until every
  //        thing has been transmitted
  //    TCSAFLUSH - flush buffers
  //        and apply changes
  if(tcsetattr(fd, TCSANOW, &options)!= 0) {
    printf("error code goes here");
  }

  // Flush the buffer one more time.
  tcflush(fd, TCIOFLUSH);

  /* open the file  */

  fd2 = open("test.txt",O_RDWR );
  if(fd2 == -1) {
    printf("cant open test.txt\n");
    close(fd);
    close(fd2);
    return -1 ;
  }


  /* read BUFF_SIZE byte from the file */
  read_res = read(fd2, buff, BUFF_SIZE);
  total_len = 0;

  if (read_res == -1) {
    /* error */
    printf("error reading file image.bin\n");
    close(fd);
    close(fd2);
    return -1 ;
  }
  else {
    /* send data on the serial lines */
	wres=write(fd,buff,read_res);

    /* test the write result */
    if (wres==-1){
      /* if opened with O_NDELAY (O_NONBLOCK) */
      /* try until the device not busy */
      if (errno == EAGAIN){
	printf("debug");
	while ((wres==-1)&&(errno == EAGAIN))
	wres=write(fd,buff,read_res);
      }
      else {
	printf("write error N° %d \n",errno);
      }
    }
    total_len+=wres;
    printf("write total_len = %d \n",(int)total_len);
  }

  /* close the file */
  close(fd2);
  /* close the serial port */
  if(close(fd) == -1) {
    printf("error code goes here");
  }
}
