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
* Author: Gonalo Cabrita on 16/08/2012
*********************************************************************/

#include "pid.h"

void reset(pid_ * pid)
{
	pid->last_error = 0.0;
	pid->accumulated_error = 0.0;
}

int32_t run(pid_ * pid, float reference, float sensor)
{
	double kp, ki_dt, kd_dt;

	kp = pid->kp;
	ki_dt = pid->ki * pid->sample_time;
	kd_dt = pid->kd / pid->sample_time;

	// Calculate error
	float error = reference - sensor;
	// Calculate accumulated error
	pid->accumulated_error += error;

	// Proportional term
	float p_term = kp * error;
	if(p_term > MAX_OUTPUT) p_term = MAX_OUTPUT;
	else if(p_term < MIN_OUTPUT) p_term = MIN_OUTPUT;

	// Integral term
	float i_term = ki_dt * pid->accumulated_error;
	if(i_term > MAX_OUTPUT) i_term = MAX_OUTPUT;
	else if(i_term < MIN_OUTPUT) i_term = MIN_OUTPUT;

	// Derivative term
	float d_term = kd_dt * (error - pid->last_error);
	if(d_term > MAX_OUTPUT) d_term = MAX_OUTPUT;
	else if(d_term < MIN_OUTPUT) d_term = MIN_OUTPUT;

	pid->last_error = error;

	// Output
	int32_t output = (int)(p_term + i_term + d_term);

	if(output > MAX_OUTPUT) output = MAX_OUTPUT;
	else if(output < MIN_OUTPUT) output = MIN_OUTPUT;

	return output;
}

// EOF

