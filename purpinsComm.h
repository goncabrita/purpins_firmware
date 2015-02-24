/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, ISR University of Coimbra.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the ISR University of Coimbra nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Goncalo Cabrita, Bruno Antunes and Bruno Gouveia on 13/08/2012
 *********************************************************************/

#ifndef __PURPINSCOMM
#define __PURPINSCOMM

#include <stdint.h>

class SerialAbstract;

/** @file */

/**
 * @defgroup COMM_GROUP Communication
 *
 * @{
 */

#define PP_START_BYTE 0x40
#define SERIAL_BUFFER_SIZE  259

/** @} */

/**
 * purpinsAction enum with all the possible actions for the Purpins robot
 */
enum purpinsAction
{
	// Get version
	PP_ACTION_GET_VERSION = 1,
	// Actuate the robot
	PP_ACTION_DRIVE = 2,
	PP_ACTION_DRIVE_MOTORS = 3,
	PP_ACTION_DRIVE_PWM = 4,
	// Get robot sensors
	PP_ACTION_GET_ODOMETRY = 5,
	PP_ACTION_GET_MOTOR_SPEEDS = 6,
	PP_ACTION_GET_ENCODER_PULSES = 7,
	PP_ACTION_GET_IMU = 8,
	PP_ACTION_GET_IR_SENSORS = 9,
	PP_ACTION_GET_GAS_SENSOR = 10,
	// Sensor streaming
	PP_ACTION_SET_SENSORS_PACK = 20,
	PP_ACTION_GET_SENSORS_PACK = 21,
	PP_ACTION_SET_SENSOR_STREAMING = 22,
	// Localisation
	PP_ACTION_SET_GLOBAL_POSE = 23,
	PP_ACTION_SET_NEIGHBORS_POSES = 24,
	// Configuration
	PP_ACTION_SET_PID_GAINS = 25,
	PP_ACTION_GET_PID_GAINS = 26,
	PP_ACTION_SET_ODOMETRY_CALIBRATION = 27,
	PP_ACTION_GET_ODOMETRY_CALIBRATION = 28,
	PP_ACTION_ERROR = 29,
	PP_ACTION_COUNT = 30
};

enum purpinsError
{
	PP_ERROR_UNKNOWN_ACTION = 1,
	PP_ERROR_CHECK_SUM = 2,
	PP_ERROR_BUFFER_SIZE = 3,
	PP_ERROR_SENSOR_NOT_AVAILABLE = 4
};

enum
{
	AWATING_START_BYTE = 0,
	AWATING_ACTION_BYTE = 1,
	AWATING_SIZE_BYTE = 2,
	GETTING_DATA = 3
};

/**
 * Communication Class with the communication protocol
 *
 * \date Sep 16, 2014
 */
class purpinsComm
{

public:

	/**
	 * Constructor.
	 *
	 * @param _serial Object with one of the SerialAbastact implementations
	 *
	 */
	purpinsComm(SerialAbstract & _serial);

	uint8_t getMsg(uint8_t * data, size_t & data_size);

	void sendMsg(uint8_t action, void * ptr, size_t size);

	void sendAck(uint8_t action);

	void error(uint8_t error_type);

private:
	/// Buffer for serial input
	uint8_t serial_buffer[SERIAL_BUFFER_SIZE];
	/// The current size of serial input buffer
	size_t serial_buffer_size;
	/// Status of the serial port
	uint8_t serial_port_status;
	/// Cyclic Redundancy Check
	uint8_t crc_;

	SerialAbstract & serial;
};

#endif //__PURPINSCOMM
// EOF

