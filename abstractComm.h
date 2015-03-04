/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, ISR University of Coimbra.
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
 * Author: Bruno Gouveia on 02/07/2014
 *********************************************************************/

#ifndef ABSTRACTCOMM_H_
#define ABSTRACTCOMM_H_

#define PP_COMM_TYPE_USB		0
#define PP_COMM_TYPE_CC3000		1
#define PP_COMM_TYPE_ESP8266	2
#define PP_COMM_TYPE_XBEE		3

/**
 * Abstract Class to communicate with the robot with the necessary interface to use different communication protocols
 *
 * \author Bruno Duarte Gouveia
 * \date Sep 16, 2014
 */
class AbstractComm {
public:

	/**
	 * read a single byte
	 * \return char from the communication device
	 */
	virtual unsigned int read(char * buffer, unsigned int max_length) = 0;

	virtual void write(const char * buffer, unsigned int length) = 0;

	virtual unsigned int type() = 0;

	virtual ~AbstractComm();

protected:
	AbstractComm(){

	}
};


#endif /* ABSTRACTCOMM_H_ */

// EOF
