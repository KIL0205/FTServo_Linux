/*
 * SMS_STS.h
 * FT SMS/STS-series serial servo application layer.
 * Date: 2025.9.27
 * Author:
 */

#ifndef _SMS_STS_H
#define _SMS_STS_H

// Register map definition
//-------EPROM (read-only)--------
#define SMS_STS_MODEL_L 3
#define SMS_STS_MODEL_H 4

//-------EPROM (read/write)--------
#define SMS_STS_ID 5
#define SMS_STS_BAUD_RATE 6
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_CW_DEAD 26
#define SMS_STS_CCW_DEAD 27
#define SMS_STS_OFS_L 31
#define SMS_STS_OFS_H 32
#define SMS_STS_MODE 33

//-------SRAM (read/write)--------
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_GOAL_SPEED_H 47
#define SMS_STS_TORQUE_LIMIT_L 48
#define SMS_STS_TORQUE_LIMIT_H 49
#define SMS_STS_LOCK 55

//-------SRAM (read-only)--------
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70

#include "SCSerial.h"

/**
 * @brief Application-layer driver for FT's SMS/STS serial servo family.
 *
 * This class exposes a subset of the standard FT servo motion/control API for
 * SMS/STS devices, including position commands, mode changes, torque control,
 * and feedback access.
 */
class SMS_STS : public SCSerial
{
public:
	/**
	 * @brief Constructs an SMS_STS driver with default settings.
	 */
	SMS_STS();
	/**
	 * @brief Constructs an SMS_STS driver for a specific endianness.
	 *
	 * @param End Byte order used during register packing.
	 */
	SMS_STS(u8 End);
	/**
	 * @brief Constructs an SMS_STS driver with explicit endianness and acknowledgement level.
	 *
	 * @param End Byte order used during register packing.
	 * @param Level Servo reply level expected by the protocol layer.
	 */
	SMS_STS(u8 End, u8 Level);
	/**
	 * @brief Sends a single-servo position command in normal mode.
	 *
	 * @param ID Target servo identifier.
	 * @param Position Goal position in device counts.
	 * @param Speed Desired motion speed.
	 * @param ACC Acceleration value for the move.
	 * @return 1 when the command is acknowledged, otherwise 0.
	 */
	int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);
	/**
	 * @brief Queues a position command for execution by RegWriteAction.
	 *
	 * @param ID Target servo identifier.
	 * @param Position Goal position in device counts.
	 * @param Speed Desired motion speed.
	 * @param ACC Acceleration value for the move.
	 * @return 1 when the command is accepted, otherwise 0.
	 */
	int RegWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);
	/**
	 * @brief Broadcasts a position write to multiple servos.
	 *
	 * @param ID Array of target servo IDs.
	 * @param IDN Number of IDs in the array.
	 * @param Position Position payload per servo.
	 * @param Speed Speed payload per servo.
	 * @param ACC Acceleration payload per servo.
	 */
	void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);
	/**
	 * @brief Broadcasts a speed-control command to multiple servos.
	 *
	 * @param ID Array of target servo IDs.
	 * @param IDN Number of IDs in the array.
	 * @param Speed Speed payload per servo.
	 * @param ACC Acceleration payload per servo.
	 */
	void SyncWriteSpe(u8 ID[], u8 IDN, s16 Speed[], u8 ACC[]);
	/**
	 * @brief Switches the servo into servo-position mode.
	 *
	 * @param ID Servo identifier.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int ServoMode(u8 ID);
	/**
	 * @brief Switches the servo into wheel-speed mode.
	 *
	 * @param ID Servo identifier.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int WheelMode(u8 ID);
	/**
	 * @brief Sends a speed command while in wheel mode.
	 *
	 * @param ID Servo identifier.
	 * @param Speed Desired rotation speed.
	 * @param ACC Acceleration value for the move.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int WriteSpe(u8 ID, s16 Speed, u8 ACC = 0);
	/**
	 * @brief Enables or disables torque for the target servo.
	 *
	 * @param ID Servo identifier.
	 * @param Enable 1 to enable torque, 0 to disable it.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int EnableTorque(u8 ID, u8 Enable);
	/**
	 * @brief Unlocks the EEPROM so settings can be rewritten.
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
	 * @brief Re-centers the servo using its offset calibration routine.
	 *
	 * @param ID Servo identifier.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int CalibrationOfs(u8 ID);
	/**
	 * @brief Reads the current feedback block into the cached buffer.
	 *
	 * @param ID Servo identifier to query; use -1 to read from the cached feedback block.
	 * @return Number of bytes received on success, -1 on failure.
	 */
	int FeedBack(int ID);
	/**
	 * @brief Reads the present position register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback block.
	 * @return Position in the device's native count units.
	 */
	int ReadPos(int ID);
	/**
	 * @brief Reads the present speed register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback block.
	 * @return Speed in the device's native count units.
	 */
	int ReadSpeed(int ID);
	/**
	 * @brief Reads the present load register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback block.
	 * @return Load value in the device's native unit encoding.
	 */
	int ReadLoad(int ID);
	/**
	 * @brief Reads the supply voltage register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback block.
	 * @return Voltage register value.
	 */
	int ReadVoltage(int ID);
	/**
	 * @brief Reads the temperature register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback block.
	 * @return Temperature register value.
	 */
	int ReadTemper(int ID);
	/**
	 * @brief Reads the motion status register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback block.
	 * @return Movement flag from the servo.
	 */
	int ReadMove(int ID);
	/**
	 * @brief Reads the present current register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback block.
	 * @return Current in the device's native count units.
	 */
	int ReadCurrent(int ID);
private:
	u8 Mem[SMS_STS_PRESENT_CURRENT_H-SMS_STS_PRESENT_POSITION_L+1];
};

#endif