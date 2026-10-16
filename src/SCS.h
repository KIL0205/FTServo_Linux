/*
 * SCS.h
 * FT serial servo protocol layer.
 * Date: 2025.9.27
 * Author:
 */

#ifndef _SCS_H
#define _SCS_H

#include "INST.h"

/**
 * @brief Common packet framing, checksum, and register-access logic for FT servos.
 *
 * This class implements the transport-level protocol used by FT servo families:
 * packet assembly, CRC validation, register reads/writes, ping/ack handling, and
 * synchronous multi-servo reads.
 */
class SCS{
public:
	/**
	 * @brief Constructs a protocol object with the default acknowledgement model.
	 */
	SCS();
	/**
	 * @brief Constructs a protocol object with a fixed endianness setting.
	 *
	 * @param End Byte order used when packing 16-bit values into packet payloads.
	 */
	SCS(u8 End);
	/**
	 * @brief Constructs a protocol object with explicit endianness and level settings.
	 *
	 * @param End Byte order used for 16-bit register values.
	 * @param Level Response level expected from the servo.
	 */
	SCS(u8 End, u8 Level);
	/**
	 * @brief Writes a block of data to a register address with ordinary command framing.
	 *
	 * @param ID Target servo identifier.
	 * @param MemAddr Register address to write.
	 * @param nDat Data buffer to send.
	 * @param nLen Number of data bytes.
	 * @return 1 when the write is acknowledged, otherwise 0.
	 */
	int genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);
	/**
	 * @brief Sends a buffered register write that is executed later by RegWriteAction.
	 *
	 * @param ID Target servo identifier.
	 * @param MemAddr Register address to update.
	 * @param nDat Data buffer to queue.
	 * @param nLen Number of data bytes.
	 * @return 1 when the command is accepted, otherwise 0.
	 */
	int regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);
	/**
	 * @brief Executes all queued deferred register writes for a target servo.
	 *
	 * @param ID Servo identifier; broadcast 0xfe is valid for all attached devices.
	 * @return 1 when the command is acknowledged, otherwise 0.
	 */
	int RegWriteAction(u8 ID = 0xfe);
	/**
	 * @brief Broadcasts one register update to multiple servos in a single packet.
	 *
	 * @param ID Array of target servo IDs.
	 * @param IDN Number of IDs in the array.
	 * @param MemAddr Register address to write.
	 * @param nDat Packed register payload shared by all target devices.
	 * @param nLen Bytes per servo payload.
	 */
	void syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen);
	/**
	 * @brief Writes a single byte to a register.
	 *
	 * @param ID Target servo identifier.
	 * @param MemAddr Register address.
	 * @param bDat Byte value to write.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int writeByte(u8 ID, u8 MemAddr, u8 bDat);
	/**
	 * @brief Writes a 16-bit value to a register.
	 *
	 * @param ID Target servo identifier.
	 * @param MemAddr Register address.
	 * @param wDat Two-byte value to write.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int writeWord(u8 ID, u8 MemAddr, u16 wDat);
	/**
	 * @brief Reads a block of register data from a servo.
	 *
	 * @param ID Target servo identifier.
	 * @param MemAddr Starting register address.
	 * @param nData Output buffer for the read payload.
	 * @param nLen Number of bytes to request.
	 * @return Number of bytes received on success, 0 on a protocol error.
	 */
	int Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen);
	/**
	 * @brief Reads a single byte from a servo register.
	 *
	 * @param ID Target servo identifier.
	 * @param MemAddr Register address.
	 * @return The byte value or -1 when the transaction fails.
	 */
	int readByte(u8 ID, u8 MemAddr);
	/**
	 * @brief Reads a 16-bit value from a servo register.
	 *
	 * @param ID Target servo identifier.
	 * @param MemAddr Starting register address.
	 * @return The combined 16-bit value or -1 when the read fails.
	 */
	int readWord(u8 ID, u8 MemAddr);
	/**
	 * @brief Pings a servo to confirm that it is responding on the bus.
	 *
	 * @param ID Target servo identifier.
	 * @return The servo ID when the ping succeeds, otherwise -1.
	 */
	int Ping(u8 ID);
	/**
	 * @brief Queues a synchronous read request for multiple servos.
	 *
	 * @param ID Array of target servo IDs.
	 * @param IDN Number of IDs in the array.
	 * @param MemAddr Starting register address for each target.
	 * @param nLen Number of bytes to read from each servo.
	 * @return Number of bytes fetched into the receive buffer.
	 */
	int syncReadPacketTx(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen);
	/**
	 * @brief Decodes one servo's response payload from a sync-read buffer.
	 *
	 * @param ID Servo identifier whose response should be decoded.
	 * @param nDat Output buffer for the decoded register bytes.
	 * @return Number of data bytes returned on success, otherwise 0.
	 */
	int syncReadPacketRx(u8 ID, u8 *nDat);
	/**
	 * @brief Extracts the next byte from the current sync-read packet.
	 *
	 * @return The next byte value or -1 on buffer exhaustion.
	 */
	int syncReadRxPacketToByte();
	/**
	 * @brief Extracts the next 16-bit value from the current sync-read packet.
	 *
	 * @param negBit Bit index used to reconstruct a signed value. Use 0 for unsigned data.
	 * @return The decoded 16-bit or signed value, or -1 on buffer exhaustion.
	 */
	int syncReadRxPacketToWrod(u8 negBit=0);
	/**
	 * @brief Initializes the sync-read receive buffer and timeout.
	 *
	 * @param IDN Number of servos expected to respond.
	 * @param rxLen Number of bytes expected per servo response.
	 * @param TimeOut Timeout in milliseconds for the read window.
	 */
	void syncReadBegin(u8 IDN, u8 rxLen, u32 TimeOut);
	/**
	 * @brief Releases the sync-read receive buffer.
	 */
	void syncReadEnd();
	/**
	 * @brief Resets the target servo state.
	 *
	 * @param ID Servo identifier.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int Reset(u8 ID);
	/**
	 * @brief Re-centers the servo using its calibration routine.
	 *
	 * @param ID Servo identifier.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int Recal(u8 ID);
	u8 getState() { return u8Status; }
	u8 getLastError() { return u8Error; }
public:
	u8 Level;
	u8 End;
	u8 u8Status;
	u8 u8Error;
	u8 syncReadRxPacketIndex;
	u8 syncReadRxPacketLen;
	u8 *syncReadRxPacket;
	u8 *syncReadRxBuff;
	u16 syncReadRxBuffLen;
	u16 syncReadRxBuffMax;
	u32 syncTimeOut;
protected:
	virtual int writeSCS(unsigned char *nDat, int nLen) = 0;
	virtual int readSCS(unsigned char *nDat, int nLen) = 0;
	virtual int readSCS(unsigned char *nDat, int nLen, unsigned long TimeOut) = 0;
	virtual int writeSCS(unsigned char bDat) = 0;
	virtual void rFlushSCS() = 0;
	virtual void wFlushSCS() = 0;
protected:
	/**
	 * @brief Packs a command frame with the target ID, register address, and payload.
	 *
	 * @param ID Target servo identifier.
	 * @param MemAddr Register address to access.
	 * @param nDat Payload data bytes.
	 * @param nLen Number of payload bytes.
	 * @param Fun Instruction ID for the packet.
	 */
	void writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun);
	/**
	 * @brief Packs a 16-bit value into low/high bytes according to the configured endianness.
	 *
	 * @param DataL Pointer to the low byte location.
	 * @param DataH Pointer to the high byte location.
	 * @param Data Source 16-bit value.
	 */
	void Host2SCS(u8 *DataL, u8* DataH, u16 Data);
	/**
	 * @brief Recombines a low/high byte pair into a 16-bit host-value.
	 *
	 * @param DataL Low-order byte.
	 * @param DataH High-order byte.
	 * @return Reconstructed 16-bit value.
	 */
	u16	SCS2Host(u8 DataL, u8 DataH);
	/**
	 * @brief Waits for and validates a servo acknowledgement packet.
	 *
	 * @param ID Target servo identifier.
	 * @return 1 when the acknowledge frame is valid, otherwise 0.
	 */
	int	Ack(u8 ID);
	/**
	 * @brief Reads bytes until a valid packet header is found.
	 *
	 * @return 1 when a valid header begins the frame, otherwise 0.
	 */
	int checkHead();
};
#endif