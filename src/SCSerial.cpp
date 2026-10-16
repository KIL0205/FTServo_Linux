/*
 * SCSerial.cpp
 * FT serial servo hardware interface implementation.
 * Date: 2022.3.29
 * Author:
 */

#include "SCSerial.h"
// HLS_WORKBOOK_MACOS_COMPAT_PATCH
#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
#endif

SCSerial::SCSerial()
{
	IOTimeOut = 100;
	fd = -1;
	txBufLen = 0;
}

SCSerial::SCSerial(u8 End):SCS(End)
{
	IOTimeOut = 100;
	fd = -1;
	txBufLen = 0;
}

SCSerial::SCSerial(u8 End, u8 Level):SCS(End, Level)
{
	IOTimeOut = 100;
	fd = -1;
	txBufLen = 0;
}

bool SCSerial::begin(int baudRate, const char* serialPort)
{
	if(fd != -1){
		close(fd);
		fd = -1;
	}
	//printf("servo port:%s\n", serialPort);
    if(serialPort == NULL)
		return false;
    fd = open(serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd == -1){
		perror("open:");
        return false;
	}
    fcntl(fd, F_SETFL, FNDELAY);
    tcgetattr(fd, &orgopt);
    tcgetattr(fd, &curopt);
    speed_t CR_BAUDRATE;
    switch(baudRate){
    case 9600:
        CR_BAUDRATE = B9600;
        break;
    case 19200:
        CR_BAUDRATE = B19200;
        break;
    case 38400:
        CR_BAUDRATE = B38400;
        break;
    case 57600:
        CR_BAUDRATE = B57600;
        break;
    case 115200:
        CR_BAUDRATE = B115200;
        break;
    #ifndef __APPLE__
case 500000:
        CR_BAUDRATE = B500000;
        break;
    case 1000000:
        CR_BAUDRATE = B1000000;
        break;
#endif
    default:
		CR_BAUDRATE = B115200;
        break;
    }
    cfsetispeed(&curopt, CR_BAUDRATE);
    cfsetospeed(&curopt, CR_BAUDRATE);

	printf("serial speed %d\n", baudRate);
    //Mostly 8N1
    curopt.c_cflag &= ~PARENB;
    curopt.c_cflag &= ~CSTOPB;
    curopt.c_cflag &= ~CSIZE;
    curopt.c_cflag |= CS8;
    curopt.c_cflag |= CREAD;
    curopt.c_cflag |= CLOCAL;//disable modem statuc check
    cfmakeraw(&curopt);//make raw mode
    curopt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    if(tcsetattr(fd, TCSANOW, &curopt) == 0){
        #ifdef __APPLE__
        if (baudRate == 500000 || baudRate == 1000000) {
            speed_t requestedSpeed = static_cast<speed_t>(baudRate);
            if (ioctl(fd, IOSSIOSPEED, &requestedSpeed) == -1) {
                perror("IOSSIOSPEED (adapter may not support this rate)");
                close(fd);
                fd = -1;
                return false;
            }
        }
#endif
        return true;
    }else{
		perror("tcsetattr:");
		return false;
	}
}

int SCSerial::setBaudRate(int baudRate)
{ 
    if(fd==-1){
		return -1;
	}
    tcgetattr(fd, &orgopt);
    tcgetattr(fd, &curopt);
    speed_t CR_BAUDRATE;
    switch(baudRate){
    case 9600:
        CR_BAUDRATE = B9600;
        break;
    case 19200:
        CR_BAUDRATE = B19200;
        break;
    case 38400:
        CR_BAUDRATE = B38400;
        break;
    case 57600:
        CR_BAUDRATE = B57600;
        break;
    case 115200:
        CR_BAUDRATE = B115200;
        break;
    case 230400:
        CR_BAUDRATE = B230400;
	break;
    #ifndef __APPLE__
case 500000:
        CR_BAUDRATE = B500000;
        break;
#endif
    default:
        break;
    }
    cfsetispeed(&curopt, CR_BAUDRATE);
    cfsetospeed(&curopt, CR_BAUDRATE);
    return 1;
}

int SCSerial::readSCS(unsigned char *nDat, int nLen, unsigned long TimeOut)
{	
    int fs_sel;
    fd_set fs_read;
	int rvLen = 0;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);

    time.tv_sec = 0;
    time.tv_usec = TimeOut*1000;

    //使用select实现串口的多路通信
	while(1){
		fs_sel = select(fd+1, &fs_read, NULL, NULL, &time);
		if(fs_sel > 0){
			const int n = read(fd, nDat+rvLen, nLen-rvLen);
			if(n <= 0){
				// EOF or read error (e.g. adapter unplugged): stop instead of spinning forever.
				txError = txError ? txError : 3;
				return rvLen;
			}
			rvLen += n;
			//printf("nLen = %d rvLen = %d\n", nLen, rvLen);
			if(rvLen<nLen){
				continue;
			}else{
				return rvLen;
			}
		}else{
			//printf("serial read fd read return 0\n");
			return rvLen;
		}
	}
}

int SCSerial::readSCS(unsigned char *nDat, int nLen)
{	
    int fs_sel;
    fd_set fs_read;
	int rvLen = 0;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);

    time.tv_sec = 0;
    time.tv_usec = IOTimeOut*1000;

    //使用select实现串口的多路通信
	while(1){
		fs_sel = select(fd+1, &fs_read, NULL, NULL, &time);
		if(fs_sel > 0){
			const int n = read(fd, nDat+rvLen, nLen-rvLen);
			if(n <= 0){
				// EOF or read error (e.g. adapter unplugged): stop instead of spinning forever.
				txError = txError ? txError : 3;
				return rvLen;
			}
			rvLen += n;
			//printf("nLen = %d rvLen = %d\n", nLen, rvLen);
			if(rvLen<nLen){
				continue;
			}else{
				return rvLen;
			}
		}else{
			//printf("serial read fd read return 0\n");
			return rvLen;
		}
	}
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
	while(nLen--){
		txBuf[txBufLen++] = *nDat++;
	}
	return txBufLen;
}

int SCSerial::writeSCS(unsigned char bDat)
{
	txBuf[txBufLen++] = bDat;
	return txBufLen;
}

void SCSerial::rFlushSCS()
{
	tcflush(fd, TCIFLUSH);
}

void SCSerial::wFlushSCS()
{
	if(txBufLen){
		const int expected = txBufLen;
		const ssize_t written = write(fd, txBuf, txBufLen);
		// Host-side detection only: negative = write error, short = incomplete frame. Cleared by ClearTxError().
		if(written < 0){
			txError = 1;
		}else if(written != expected){
			txError = 2;
		}
		txBufLen = 0;
	}
}

void SCSerial::end()
{
	fd = -1;
	close(fd);
}