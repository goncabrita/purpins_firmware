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
* Author: Gonçalo Cabrita, Bruno Antunes and Bruno Gouveia on 13/08/2012
*********************************************************************/

extern unsigned long millis ();
class SerialAbstract;

// Modes of operation
#define PP_MODE_SERIAL      0
#define PP_MODE_XBEE_API    1

#define PP_STR              '@'
#define PP_SEP              ','
#define PP_END              'e'

#define SERIAL_BUFFER_SIZE  64

#define MAX_IN_ARGS  6
#define MAX_OUT_ARGS 7

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
    PP_ACTION_GET_GAS_SENSOR = 7,
    PP_ACTION_GET_IR_BUMPER = 8,
    PP_ACTION_GET_LINE_SENSOR = 9,
    PP_ACTION_GET_BATTERY = 10,
    // Debug mode
    PP_ACTION_GET_DEBUG = 11,
    PP_ACTION_SET_DEBUG = 12,
    // Configuration
    PP_ACTION_SET_PID_GAINS = 13,
    PP_ACTION_GET_PID_GAINS = 14,
    PP_ACTION_SET_ODOMETRY_CALIBRATION = 15,
    PP_ACTION_GET_ODOMETRY_CALIBRATION = 16,
    PP_ACTION_SET_ID = 17,
    PP_ACTION_GET_ID = 18,
    PP_ACTION_SET_MODE = 19,
    PP_ACTION_GET_MODE = 20,
    PP_ACTION_SET_GAS_CALIBRATION = 21,
    PP_ACTION_GET_GAS_CALIBRATION = 22,
    PP_ACTION_SET_BATTERY_TYPE = 23,
    PP_ACTION_GET_BATTERY_TYPE = 24,
    PP_ACTION_SET_TIMEOUT = 25,
    PP_ACTION_GET_TIMEOUT = 26,
    // Group messages
    // Group 1 - Odometry, gas sensor
    PP_ACTION_GET_GROUP_1 = 27,
    // Group 2 - Odometry, gas sensor, IR bumper
    PP_ACTION_GET_GROUP_2 = 28,
    PP_ACTION_COUNT = 29
};

extern int MQ_ACTION_PARAM_COUNT[];

enum
{
    AWATING_DATA = 0,
    GETTING_DATA = 1
};

class purpinsComm
{
    public:
    purpinsComm(SerialAbstract * _serial);
    
    int getMsg(int * argv);
    
    void reply(int action, int * argv, int argc);
    
    void setID(uint8_t new_id);
    uint8_t getID();
    
    void setMode(uint8_t new_mode);
    uint8_t getMode();
    
    void sendDebugMsg();
    int getDebug();
    void setDebug(int d);
    
    // Debug message buffer
    char debug_msg[20];
    // Debug ON (1) or OFF (0)
    uint8_t debug;
    
    private:
    // Buffer for serial input
    char serial_buffer[SERIAL_BUFFER_SIZE];
    // The current size of serial input buffer
    int serial_buffer_size;
    // Status of the serial port, AWATING_DATA or GETTING_DATA
    uint8_t serial_port_status;
    
    // Used by the function getValue to store the index of the next separator uint8_t
    int next_separator;
    
    int getValue(int start_index);
    
    // The robot ID
    uint8_t id;
    // The robot mode of operation (Serial or XBee)
    uint8_t mode;
    
    unsigned long start_time;
    
    //TODO: change for serialcomabstract
    // XBee object
    SerialAbstract * serial;
};

// EOF

