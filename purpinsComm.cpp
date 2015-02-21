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
#include <cstdio>
#include <string.h>
#include "purpinsComm.h"
#include "SerialAbstract.h"

extern "C"
{
	#include "utils/ustdlib.h"
	extern  unsigned long millis(void);
}

size_t PP_ACTION_DATA_SIZE[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

purpinsComm::purpinsComm(SerialAbstract & _serial):serial(_serial)
{
	serial_port_status = AWATING_START_BYTE;
    serial_buffer_size = 0;
}

uint8_t purpinsComm::getMsg(uint8_t * data)
{
	// If data is available...
	if(serial.available())
	{
		// Read a uint8_t from the serial port
		uint8_t new_byte = serial.read();

		// Start byte
		if(serial_port_status == AWATING_START_BYTE && new_byte == PP_START_BYTE)
		{
			memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
			serial_buffer_size = 1;
			serial_buffer[0] = new_byte;
			serial_port_status = AWATING_ACTION_BYTE;
		}
		// Action byte
		else if(serial_port_status == AWATING_ACTION_BYTE)
		{
			if(new_byte <= 0 || new_byte >= PP_ACTION_COUNT)
			{
				error(PP_ERROR_UNKNOWN_ACTION);
				memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
				serial_port_status = AWATING_START_BYTE;
				return 0;
			}

			serial_buffer_size = 2;
			serial_buffer[1] = new_byte;
			serial_port_status = GETTING_DATA;
		}
		// Data
		else if(serial_port_status == GETTING_DATA)
		{
			if(serial_buffer_size < SERIAL_BUFFER_SIZE)
			{
				serial_buffer[serial_buffer_size] = new_byte;
				serial_buffer_size++;

				// Check sum
				if(serial_buffer_size == (3 + PP_ACTION_DATA_SIZE[serial_buffer[1]-1]))
				{
					// Check sum
					uint8_t check_sum = 0;
					for(unsigned int i=0 ; i<serial_buffer_size ; i++)
					{
						check_sum += serial_buffer[i];
					}
					if(check_sum != 0)
					{
						error(PP_ERROR_CHECK_SUM);
						memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
						serial_port_status = AWATING_START_BYTE;
						return 0;
					}

					if(PP_ACTION_DATA_SIZE[serial_buffer[1]-1] > 0) memcpy(data, serial_buffer+2, PP_ACTION_DATA_SIZE[serial_buffer[1]-1]);
					serial_port_status = AWATING_START_BYTE;
					return serial_buffer[1];
				}
			}
			else
			{
				error(PP_ERROR_BUFFER_SIZE);
				memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
				serial_port_status = AWATING_START_BYTE;
				return 0;
			}
		}
	}
	return 0;
}

void purpinsComm::sendMsg(uint8_t action, void * ptr, size_t size)
{
	uint8_t msg[size+3];

	msg[0] = PP_START_BYTE;
	msg[1] = action;

	memcpy(msg+2, (uint8_t*)ptr, size);

	uint8_t check_sum = 0;
	for(unsigned int i=0 ; i<size+2 ; i++)
	{
		check_sum += msg[i];
	}
	// TODO: Invert bits in checksum and add 1
	msg[size+2] = check_sum;

	serial.write((char*)msg, size+3);
}

void purpinsComm::parse(uint8_t action, uint8_t * data, void * ptr)
{
	memcpy(ptr, data, PP_ACTION_DATA_SIZE[serial_buffer[1]-1]);
}

void purpinsComm::error(uint8_t error_type)
{
	sendMsg(PP_ACTION_ERROR, (void*)(&error_type), 1);
}

// EOF
