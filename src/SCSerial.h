/*
 * SCSerial.h
 * FT serial servo hardware interface layer.
 * Date: 2022.3.29
 * Author:
 */

#ifndef _SCSERIAL_H
#define _SCSERIAL_H

#include "SCS.h"
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/select.h>

/**
 * @brief POSIX serial-port wrapper for FT servo communication.
 *
 * This class owns the native file descriptor and handles low-level buffering,
 * baud-rate setup, flushes, and blocking or timeout-based reads/writes used by
 * the higher-level protocol classes.
 */
class SCSerial : public SCS
{
public:
	/**
	 * @brief Constructs an uninitialized serial driver with default timeout settings.
	 */
	SCSerial();
	/**
	 * @brief Constructs a serial driver with the specified byte order.
	 *
	 * @param End Endianness flag used by the servo packet encoding layer.
	 */
	SCSerial(u8 End);
	/**
	 * @brief Constructs a serial driver with explicit byte order and acknowledgement level.
	 *
	 * @param End Endianness flag used by the protocol conversion helpers.
	 * @param Level Servo acknowledgement level for command replies.
	 */
	SCSerial(u8 End, u8 Level);

protected:
	/**
	 * @brief Queues a byte array for transmission over the serial port.
	 *
	 * @param nDat Buffer to enqueue.
	 * @param nLen Number of bytes to enqueue.
	 * @return Total buffered byte count after enqueuing.
	 */
	int writeSCS(unsigned char *nDat, int nLen);
	/**
	 * @brief Reads exactly nLen bytes from the serial port or until timeout.
	 *
	 * @param nDat Destination buffer.
	 * @param nLen Number of bytes to read.
	 * @return Number of bytes actually received.
	 */
	int readSCS(unsigned char *nDat, int nLen);
	/**
	 * @brief Reads exactly nLen bytes with a caller-provided timeout.
	 *
	 * @param nDat Destination buffer.
	 * @param nLen Number of bytes to read.
	 * @param TimeOut Timeout in milliseconds.
	 * @return Number of bytes actually received.
	 */
	int readSCS(unsigned char *nDat, int nLen, unsigned long TimeOut);
	/**
	 * @brief Queues a single byte for transmission.
	 *
	 * @param bDat Byte to enqueue.
	 * @return Total buffered byte count after enqueuing.
	 */
	int writeSCS(unsigned char bDat);
	/**
	 * @brief Clears pending serial input.
	 */
	void rFlushSCS();
	/**
	 * @brief Flushes the transmit buffer to the serial device.
	 */
	void wFlushSCS();
public:
	/** Last transmit error since ClearTxError(): 0 none, 1 write() failed, 2 short write, 3 read error/EOF. */
	int LastTxError() const { return txError; }
	void ClearTxError() { txError = 0; }
	/**
	 * @brief Timeout used by read/write operations waiting for servo replies.
	 */
	unsigned long int IOTimeOut;
public:
	/**
	 * @brief Reconfigures the serial port to a different baud rate.
	 *
	 * @param baudRate Desired baud rate in bits per second.
	 * @return 1 on success, -1 when the port is not open.
	 */
	int setBaudRate(int baudRate);
	/**
	 * @brief Opens a hardware serial port and configures the serial parameters.
	 *
	 * @param baudRate Desired communication speed for the servo bus.
	 * @param serialPort Device path such as /dev/ttyUSB0.
	 * @return true when the port opened and configured successfully.
	 */
	bool begin(int baudRate, const char* serialPort);
	/**
	 * @brief Closes the serial port and releases the device handle.
	 */
	void end();
protected:
    /**
     * @brief Open file descriptor for the current serial port.
     */
    int fd;
    /**
     * @brief Original terminal settings captured before configuration.
     */
    struct termios orgopt;
	/**
	 * @brief Current terminal settings used for the active interface.
	 */
	struct termios curopt;
	/**
	 * @brief Buffer for queued serial bytes before flush.
	 */
	unsigned char txBuf[255];
	/**
	 * @brief Number of bytes currently queued in the transmit buffer.
	 */
	int txBufLen;
	int txError = 0;
};

#endif