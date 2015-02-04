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
#include <cstring>
#include "purpinsComm.h"
#include "SerialAbstract.h"

extern "C"{
#include "utils/ustdlib.h"
extern  unsigned long millis(void);
}



int PP_ACTION_PARAM_COUNT[] = {0, 2, 2, 0, 0, 0, 0, 1, 3, 0, 2, 0, 1, 0};

purpinsComm::purpinsComm(SerialAbstract & _serial):serial(_serial)
{
	serial_port_status = AWATING_DATA;
	debug = 0;  // By default debug is OFF
	id=-1;
    serial_buffer_size=SERIAL_BUFFER_SIZE;
}

int purpinsComm::getMsg(int * argv)
{
	// If data is available...
	if(serial.available())
	{
		// Read a uint8_t from the serial port
		uint8_t newByte = serial.read();

		// Got a start uint8_t!!!
		if(newByte == PP_STR)
		{
			start_time = millis();

			// If we are waiting for data start a new message
			if(serial_port_status == AWATING_DATA)
			{
				serial_port_status = GETTING_DATA;

				memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
				serial_buffer[0] = newByte;
				serial_buffer_size = 1;
			}
			// Otherwise this is an error!
			else
			{
				if(debug) serial.println("[ERROR] Got start uint8_t in the middle of a message!");
				memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
				serial_port_status = AWATING_DATA;
			}
		}
		// Got an end uint8_t and a message is being constructed
		else if(newByte == PP_END && serial_port_status == GETTING_DATA)
		{
			if(serial_buffer_size < SERIAL_BUFFER_SIZE-1)
			{
				serial_buffer[serial_buffer_size] = newByte;
				serial_buffer_size++;

				serial_buffer[serial_buffer_size] = '\0';
				serial_buffer_size++;

				if(debug)
				{
					usprintf(debug_msg,"[INFO] Got message: %s %d ms", serial_buffer, millis()-start_time);
					serial.println(debug_msg);
				}

				serial_port_status = AWATING_DATA;

				// Next parse incoming data...
				int msg_id = getValue(0);
				if(msg_id != id && msg_id != 0)
				{
					usprintf(debug_msg,"[INFO] The message id %d does not match this Purpins id %d", msg_id, id);
					serial.println(debug_msg);
					return 0;
				}

				int action = getValue(next_separator+1);

				if(action == -1 && debug) serial.println("[ERROR] Could not find an action in the message!");

				if(action > 0 && action < PP_ACTION_COUNT)
				{
					for(int i=0 ; i<PP_ACTION_PARAM_COUNT[action-1] ; i++)
					{
						argv[i] = getValue(next_separator+1);

						if(argv[i] == -1)
						{
							if(debug) serial.println("[ERROR] Insufficient number of parameters for this action!");
							return 0;
						}
					}
					return action;
				}
				else
				{
					if(debug){
						usprintf(debug_msg,"[ERROR] Unknown action: %d",action);
						serial.println(debug_msg);
					}
				}
			}
			else
			{
				if(debug) serial.println("[ERROR] Buffer size exceeded!");
				memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
				serial_port_status = AWATING_DATA;
			}
		}
		// Got a uint8_t and a message is being constructed
		else if(serial_port_status == GETTING_DATA)
		{
			if(serial_buffer_size < SERIAL_BUFFER_SIZE)
			{
				serial_buffer[serial_buffer_size] = newByte;
				serial_buffer_size++;
			}
			else
			{
				if(debug) serial.println("[ERROR] Buffer size exceeded!");
				memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
				serial_port_status = AWATING_DATA;
			}
		}
	}

	return 0;
}

void purpinsComm::reply(int action, int * argv, int argc)
{

	char reply_string[20];
	usprintf(reply_string, "%c%d%c%d", PP_STR, id, PP_SEP, action);

	unsigned int len;

	for(int i=0; i<argc ; i++)
	{
		char temp[20];
		len = ustrlen(reply_string);
		ustrncpy(temp, reply_string, len);
		temp[len] = 0;
		usprintf(reply_string, "%s%c%d", temp, PP_SEP, argv[i]);
	}

	len = ustrlen(reply_string);
	reply_string[len] = PP_END;
	reply_string[len+1] = 0;

	serial.println(reply_string);
}

// Helper function for retrieving int values from the serial input buffer
int purpinsComm::getValue(int start_index)
{
	if(serial_buffer[start_index-1] == PP_END) return -1;

	char data[16];
	int i = 0;

	while(serial_buffer[start_index+i] != PP_SEP && serial_buffer[start_index+i] != PP_END && start_index+i < SERIAL_BUFFER_SIZE && i < 16)
	{
		data[i] = serial_buffer[start_index+i];
		i++;
	}
	data[i] = '\0';
	next_separator = start_index+i;

	if(i == 0) return -1;

	return (int)ustrtoul(data, 0, 0);
}

void purpinsComm::setID(uint8_t new_id)
{
	id = new_id;
}

uint8_t purpinsComm::getID()
{
	return id;
}

void purpinsComm::setMode(uint8_t new_mode)
{
	mode = new_mode;
}

uint8_t purpinsComm::getMode()
{
	return mode;
}

void purpinsComm::sendDebugMsg()
{
	if(debug) serial.println(debug_msg);
}

int purpinsComm::getDebug()
{
	return debug;
}

void purpinsComm::setDebug(int d)
{
	debug = d;

	if(debug) serial.println("[INFO] Debug mode is ON!");
	else serial.println("[INFO] Debug mode is OFF...");
}

// EOF
