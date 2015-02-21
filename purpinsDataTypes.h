/*
 * purpinsDataTypes.h
 *
 *  Created on: Feb 20, 2015
 *      Author: cabrita
 */

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

#endif /* PURPINSDATATYPES_H_ */
