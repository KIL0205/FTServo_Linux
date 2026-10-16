/*
 * SCSCL.h
 * FT SCSCL-series serial servo application layer.
 * Date: 2025.9.27
 * Author:
 */

#ifndef _SCSCL_H
#define _SCSCL_H

// Register map definition
//-------EPROM (read-only)--------
#define SCSCL_VERSION_L 3
#define SCSCL_VERSION_H 4

//-------EPROM (read/write)--------
#define SCSCL_ID 5
#define SCSCL_BAUD_RATE 6
#define SCSCL_MIN_ANGLE_LIMIT_L 9
#define SCSCL_MIN_ANGLE_LIMIT_H 10
#define SCSCL_MAX_ANGLE_LIMIT_L 11
#define SCSCL_MAX_ANGLE_LIMIT_H 12
#define SCSCL_CW_DEAD 26
#define SCSCL_CCW_DEAD 27

//-------SRAM (read/write)--------
#define SCSCL_TORQUE_ENABLE 40
#define SCSCL_GOAL_POSITION_L 42
#define SCSCL_GOAL_POSITION_H 43
#define SCSCL_GOAL_TIME_L 44
#define SCSCL_GOAL_TIME_H 45
#define SCSCL_GOAL_SPEED_L 46
#define SCSCL_GOAL_SPEED_H 47
#define SCSCL_LOCK 48

//-------SRAM (read-only)--------
#define SCSCL_PRESENT_POSITION_L 56
#define SCSCL_PRESENT_POSITION_H 57
#define SCSCL_PRESENT_SPEED_L 58
#define SCSCL_PRESENT_SPEED_H 59
#define SCSCL_PRESENT_LOAD_L 60
#define SCSCL_PRESENT_LOAD_H 61
#define SCSCL_PRESENT_VOLTAGE 62
#define SCSCL_PRESENT_TEMPERATURE 63
#define SCSCL_MOVING 66
#define SCSCL_PRESENT_CURRENT_L 69
#define SCSCL_PRESENT_CURRENT_H 70

#include "SCSerial.h"

/**
 * @brief Application-layer driver for FT's SCSCL-series servo family.
 *
 * This class wraps the common serial protocol and exposes the registers and
 * motion commands typically used for SCSCL devices, including position writes,
 * mode configuration, torque enablement, and feedback parsing.
 */
class SCSCL : public SCSerial
{
public:
	/**
	 * @brief Creates an SCSCL driver with defaults.
	 */
	SCSCL();
	/**
	 * @brief Creates an SCSCL driver for a specific endianness.
	 *
	 * @param End Byte order used for 16-bit values in packet payloads.
	 */
	SCSCL(u8 End);
	/**
	 * @brief Creates an SCSCL driver with explicit endianness and acknowledgement mode.
	 *
	 * @param End Byte order used for 16-bit values.
	 * @param Level Desired acknowledgement level from the servo.
	 */
	SCSCL(u8 End, u8 Level);
	/**
	 * @brief Sends a single-servo position command in normal mode.
	 *
	 * @param ID Servo identifier.
	 * @param Position Target position in device counts.
	 * @param Time Motion time parameter for the target move.
	 * @param Speed Optional speed limit in device counts.
	 * @return 1 when the command is acknowledged, otherwise 0.
	 */
	int WritePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0);
	/**
	 * @brief Queues a position command for delayed execution.
	 *
	 * @param ID Servo identifier.
	 * @param Position Target position in device counts.
	 * @param Time Motion time parameter.
	 * @param Speed Optional speed limit in device counts.
	 * @return 1 when the command is accepted, otherwise 0.
	 */
	int RegWritePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0);
	/**
	 * @brief Broadcasts a position write to multiple servos in one transaction.
	 *
	 * @param ID Array of target servo IDs.
	 * @param IDN Number of IDs in the array.
	 * @param Position Position payload for each servo.
	 * @param Time Time payload for each servo.
	 * @param Speed Speed payload for each servo.
	 */
	void SyncWritePos(u8 ID[], u8 IDN, u16 Position[], u16 Time[], u16 Speed[]);
	/**
	 * @brief Switches the target servo into PWM output mode.
	 *
	 * @param ID Servo identifier.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int PWMMode(u8 ID);
	/**
	 * @brief Sends a PWM command in PWM output mode.
	 *
	 * @param ID Servo identifier.
	 * @param pwmOut Pulse-width output value.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int WritePWM(u8 ID, s16 pwmOut);
	/**
	 * @brief Enables or disables torque for a servo.
	 *
	 * @param ID Servo identifier.
	 * @param Enable 1 to enable torque, 0 to disable it.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int EnableTorque(u8 ID, u8 Enable);
	/**
	 * @brief Unlocks the EEPROM so that configuration registers can be modified.
	 *
	 * @param ID Servo identifier.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int unLockEprom(u8 ID);
	/**
	 * @brief Locks the EEPROM after configuration updates.
	 *
	 * @param ID Servo identifier.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int LockEprom(u8 ID);
	/**
	 * @brief Requests a full feedback packet from the target servo.
	 *
	 * @param ID Servo identifier, or -1 to use the last buffered packet.
	 * @return Number of bytes received on success, -1 on failure.
	 */
	int FeedBack(int ID);
	/**
	 * @brief Reads the current position register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return Position encoded in device counts.
	 */
	int ReadPos(int ID);
	/**
	 * @brief Reads the current speed register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return Speed encoded in native device counts.
	 */
	int ReadSpeed(int ID);
	/**
	 * @brief Reads the present load register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return Load value in the device's encoded units.
	 */
	int ReadLoad(int ID);
	/**
	 * @brief Reads the supply voltage register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return Voltage in tenths of a volt, or device-specific value from the register.
	 */
	int ReadVoltage(int ID);
	/**
	 * @brief Reads the temperature register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return Temperature in degrees Celsius.
	 */
	int ReadTemper(int ID);
	/**
	 * @brief Reads the movement status register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return 0 or 1 movement flag from the target device.
	 */
	int ReadMove(int ID);
	/**
	 * @brief Reads the current draw register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return Current in the servo's native current units.
	 */
	int ReadCurrent(int ID);
private:
	u8 Mem[SCSCL_PRESENT_CURRENT_H-SCSCL_PRESENT_POSITION_L+1];
};

#endif