/*
 * HLSCL.h
 * FT HLS-series serial servo application layer.
 * Date: 2025.9.27
 * Author:
 */

#ifndef _HLSCL_H
#define _HLSCL_H

// Register map definition
//-------EPROM (read-only)--------
#define HLSCL_MODEL_L 3
#define HLSCL_MODEL_H 4

//-------EPROM (read/write)--------
#define HLSCL_ID 5
#define HLSCL_BAUD_RATE 6
#define HLSCL_SECOND_ID 7
#define HLSCL_MIN_ANGLE_LIMIT_L 9
#define HLSCL_MIN_ANGLE_LIMIT_H 10
#define HLSCL_MAX_ANGLE_LIMIT_L 11
#define HLSCL_MAX_ANGLE_LIMIT_H 12
#define HLSCL_CW_DEAD 26
#define HLSCL_CCW_DEAD 27
#define HLSCL_OFS_L 31
#define HLSCL_OFS_H 32
#define HLSCL_MODE 33

//-------SRAM (read/write)--------
#define HLSCL_TORQUE_ENABLE 40
#define HLSCL_ACC 41
#define HLSCL_GOAL_POSITION_L 42
#define HLSCL_GOAL_POSITION_H 43
#define HLSCL_GOAL_TORQUE_L 44
#define HLSCL_GOAL_TORQUE_H 45
#define HLSCL_GOAL_SPEED_L 46
#define HLSCL_GOAL_SPEED_H 47
#define HLSCL_TORQUE_LIMIT_L 48
#define HLSCL_TORQUE_LIMIT_H 49
#define HLSCL_LOCK 55

//-------SRAM (read-only)--------
#define HLSCL_PRESENT_POSITION_L 56
#define HLSCL_PRESENT_POSITION_H 57
#define HLSCL_PRESENT_SPEED_L 58
#define HLSCL_PRESENT_SPEED_H 59
#define HLSCL_PRESENT_LOAD_L 60
#define HLSCL_PRESENT_LOAD_H 61
#define HLSCL_PRESENT_VOLTAGE 62
#define HLSCL_PRESENT_TEMPERATURE 63
#define HLSCL_MOVING 66
#define HLSCL_PRESENT_CURRENT_L 69
#define HLSCL_PRESENT_CURRENT_H 70

#include "SCSerial.h"
#include <fstream>
#include <string.h>

/**
 * @brief Application-layer driver for FT's HLS-series serial servo family.
 *
 * This class exposes the HLS-specific register map and motion commands used for
 * position, speed, torque, and configuration control on single-servo and
 * multi-servo HLS networks.
 */
class HLSCL : public SCSerial
{
public:
	/**
	 * @brief Constructs an HLSCL driver with default settings.
	 */
	HLSCL();
	/**
	 * @brief Constructs an HLSCL driver for a specific endianness.
	 *
	 * @param End Byte order used during register packing.
	 */
	HLSCL(u8 End);
	/**
	 * @brief Constructs an HLSCL driver with a configured endianness and reply level.
	 *
	 * @param End Byte order used during register packing.
	 * @param Level Servo reply level expected by the protocol layer.
	 */
	HLSCL(u8 End, u8 Level);
	/**
	 * @brief Sends a single-servo position command with acceleration and torque settings.
	 *
	 * @param ID Target servo identifier.
	 * @param Position Goal position in device counts.
	 * @param Speed Target motion speed.
	 * @param ACC Acceleration value for the move.
	 * @param Torque Optional torque limit for the command.
	 * @return 1 when the command is acknowledged, otherwise 0.
	 */
	int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0, u16 Torque = 0);
	/**
	 * @brief Queues a position command for later execution using RegWriteAction.
	 *
	 * @param ID Target servo identifier.
	 * @param Position Goal position in device counts.
	 * @param Speed Target motion speed.
	 * @param ACC Acceleration value for the move.
	 * @param Torque Optional torque limit for the command.
	 * @return 1 when the command is accepted, otherwise 0.
	 */
	int RegWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0, u16 Torque = 0);
	/**
	 * @brief Broadcasts a position write to multiple HLS servos.
	 *
	 * @param ID Array of target servo IDs.
	 * @param IDN Number of IDs in the array.
	 * @param Position Position payload for each servo.
	 * @param Speed Speed payload for each servo.
	 * @param ACC Acceleration payload for each servo.
	 * @param Torque Torque payload for each servo.
	 */
	void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[], u16 Torque[]);
	/**
	 * @brief Broadcasts a speed-control command to multiple HLS servos.
	 *
	 * @param ID Array of target servo IDs.
	 * @param IDN Number of IDs in the array.
	 * @param Speed Speed payload per servo.
	 * @param ACC Acceleration payload per servo.
	 * @param Torque Torque payload per servo.
	 */
	void SyncWriteSpe(u8 ID[], u8 IDN, s16 Speed[], u8 ACC[], u16 Torque[]);
	/**
	 * @brief Switches the servo to normal position-servo mode.
	 *
	 * @param ID Servo identifier.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int ServoMode(u8 ID);
	/**
	 * @brief Switches the servo to wheel-speed mode.
	 *
	 * @param ID Servo identifier.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int WheelMode(u8 ID);
	/**
	 * @brief Switches the servo to electronic-torque limit mode.
	 *
	 * @param ID Servo identifier.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int EleMode(u8 ID);
	/**
	 * @brief Sends a speed command while the servo is in wheel mode.
	 *
	 * @param ID Servo identifier.
	 * @param Speed Desired rotation speed.
	 * @param ACC Acceleration setting.
	 * @param Torque Optional torque limit for the command.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int WriteSpe(u8 ID, s16 Speed, u8 ACC = 0, u16 Torque = 0);
	/**
	 * @brief Sends a torque command while the servo is in electronic-torque mode.
	 *
	 * @param ID Servo identifier.
	 * @param Torque Goal torque value.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int WriteEle(u8 ID, s16 Torque);
	/**
	 * @brief Enables or disables the servo's output torque.
	 *
	 * @param ID Servo identifier.
	 * @param Enable 1 to enable torque, 0 to disable it.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int EnableTorque(u8 ID, u8 Enable);
	/**
	 * @brief Unlocks the EEPROM so configuration registers can be written.
	 *
	 * @param ID Servo identifier.
	 * @return 1 when acknowledged, otherwise 0.
	 */
	int unLockEprom(u8 ID);
	/**
	 * @brief Locks the EEPROM after configuration changes.
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
	 * @param ID Servo identifier to query; use -1 to reuse the current cache.
	 * @return Number of bytes received on success, -1 on failure.
	 */
	int FeedBack(int ID);
	/**
	 * @brief Reads the present position register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return Position in the device's native count units.
	 */
	int ReadPos(int ID);
	/**
	 * @brief Reads the present speed register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return Speed in the device's native count units.
	 */
	int ReadSpeed(int ID);
	/**
	 * @brief Reads the present load register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return Load in the device's native count units.
	 */
	int ReadLoad(int ID);
	/**
	 * @brief Reads the supply voltage register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return Voltage register contents.
	 */
	int ReadVoltage(int ID);
	/**
	 * @brief Reads the servo temperature register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return Temperature register value.
	 */
	int ReadTemper(int ID);
	/**
	 * @brief Reads the motion status register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return Movement flag value from the servo.
	 */
	int ReadMove(int ID);
	/**
	 * @brief Reads the present current register.
	 *
	 * @param ID Servo identifier, or -1 to read from the cached feedback buffer.
	 * @return Current in the device's native count units.
	 */
	int ReadCurrent(int ID);

	// /**
	//  * @brief save the EPROM configuration
	//  * 
	//  * @param ID Servo identifier to read from .
	//  * @param fileName File to save the config to
	//  * @return 1 on success, -1 on failure
	//  */
	// int saveConfig(int ID, std::string fileName);

	// /**
	//  * @brief save the EPROM configuration
	//  * 
	//  * @param ID Servo identifier to write to.
	//  * @param fileName File to Load the config from
	//  * @return 1 on success, -1 on failure
	//  */
	// void loadConfig(int ID, std::string fileName);

private:
	u8 Mem[HLSCL_PRESENT_CURRENT_H-HLSCL_PRESENT_POSITION_L+1];
};

#endif