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
 * Author: Gon�alo Cabrita on 20/02/2015
 *********************************************************************/

#ifndef __PURPINSDATATYPES
#define __PURPINSDATATYPES

typedef struct _robot_speed
{
	float linear;
	float angular;

} RobotSpeed;

typedef struct _motor_speeds
{
	float left;
	float right;

} MotorSpeeds;

typedef struct _motor_pwms
{
	int32_t left;
	int32_t right;

} MotorPWMs;

typedef struct _pose
{
	float x;
	float y;
	float yaw;

} Pose;

typedef struct _encoder_pulses
{
	int32_t left;
	int32_t right;

} EncoderPulses;

typedef struct _imu
{
	float orientation_x;
	float orientation_y;
	float orientation_z;
	float orientation_w;

	float angular_velocity_x;
	float angular_velocity_y;
	float angular_velocity_z;

	float linear_acceleration_x;
	float linear_acceleration_y;
	float linear_acceleration_z;

} IMU;

typedef struct _ir_sensors
{
	float left;
	float left_center;
	float center;
	float right_center;
	float right;

} IR;

typedef struct _gas_sensor
{
	float concentration;

} Gas;

typedef struct _pid_gains
{
	float kp;
	float ki;
	float kd;

} PIDGains;

typedef struct _server
{
	uint8_t ip[4];
	uint32_t port;

} Server;

typedef struct _network
{
	char ssid[32];
	char key[32];
	uint32_t security;

} Network;

#endif /* PURPINSDATATYPES_H_ */

// EOF
