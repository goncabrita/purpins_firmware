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

#define PP_STR              '@'
#define PP_SEP              ','
#define PP_END              'e'

#define SERIAL_BUFFER_SIZE  64

#define MAX_IN_ARGS  6
#define MAX_OUT_ARGS 7

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
	PP_ACTION_DRIVE_DIRECT = 3,
	// Get robot sensors
	PP_ACTION_GET_ODOMETRY = 4,
	PP_ACTION_GET_ENCODER_PULSES = 5,
	PP_ACTION_GET_WHEEL_VELOCITIES = 6,
	// Debug mode
	PP_ACTION_GET_DEBUG = 7,
	PP_ACTION_SET_DEBUG = 8,
	// Configuration
	PP_ACTION_SET_PID_GAINS = 9,
	PP_ACTION_GET_PID_GAINS = 10,
	PP_ACTION_SET_ODOMETRY_CALIBRATION = 11,
	PP_ACTION_GET_ODOMETRY_CALIBRATION = 12,
	PP_ACTION_SET_ID = 13,
	PP_ACTION_GET_ID = 14,
	PP_ACTION_COUNT = 15
};

extern int MQ_ACTION_PARAM_COUNT[];

enum
{
	AWATING_DATA = 0,
	GETTING_DATA = 1
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

	int getMsg(int * argv);

	void reply(int action, int * argv, int argc);

	/**
	 * Set the robot's ID
	 *
	 * @param new_id id number
	 *
	 */
	void setID(uint8_t new_id);

	/**
	 * Get the robot's ID
	 *
	 * @return id number
	 *
	 */
	uint8_t getID();

	/**
	 * Set the robot's mode
	 *
	 * @param new_mode one of the available modes
	 *
	 */
	void setMode(uint8_t new_mode);

	/**
	 * Get the robot's current mode
	 *
	 * @return current mode
	 *
	 */
	uint8_t getMode();

	void sendDebugMsg();
	int getDebug();
	void setDebug(int d);

	/// Debug message buffer
	char debug_msg[20];
	/// Debug ON (1) or OFF (0)
	uint8_t debug;

private:
	/// Buffer for serial input
	char serial_buffer[SERIAL_BUFFER_SIZE];
	/// The current size of serial input buffer
	int serial_buffer_size;
	/// Status of the serial port, AWATING_DATA or GETTING_DATA
	uint8_t serial_port_status;

	/// Used by the function getValue to store the index of the next separator uint8_t
	int next_separator;

	int getValue(int start_index);

	/// The robot ID
	uint8_t id;
	/// The robot mode of operation (Serial or XBee)
	uint8_t mode;

	unsigned long start_time;

	SerialAbstract & serial;
};

#endif //__PURPINSCOMM
// EOF

