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
* Author: Gonçalo Cabrita and Bruno Gouveia on 16/08/2012
*********************************************************************/

#ifndef __PURPINSROBOT
#define __PURPINSROBOT

#include <stdint.h>
#include "pid.h"
#include "purpinsDataTypes.h"

#define PWM_FREQUENCY (20000)           	/**< Motor PWM frequency in Hz */
#define QEI_LOOP_FREQUENCY (40)           	/**< QEI loop frequency in Hz */
#define QEI_RATE (1.0/QEI_LOOP_FREQUENCY)	/**< QEI loop interval in seconds */

#define PURPINS_AXLE_LENGTH (0.10)
#define PURPINS_WHEEL_DIAMETER (0.042)
#define PURPINS_WHEEL_RADIUS (PURPINS_WHEEL_DIAMETER/2.0)

#define PURPINS_TICKS_PER_TURN (2000)
#define PURPINS_TICKS_TO_RAD ((2.0*M_PI)/PURPINS_TICKS_PER_TURN)
#define PURPINS_TICKS_TO_M ((PURPINS_WHEEL_DIAMETER*M_PI)/PURPINS_TICKS_PER_TURN)

/**
 * PurpinsMotor structure used for convenience with all the motor variables
 */
typedef struct _motor
{
	bool closed_loop; // Flag for controlling the PID

	float speed; // Measured speed in rad/s
	float target_speed; // Reference speed in rad/s
	int32_t pwm; // Corresponding PWM value

	int32_t ticks; // Ticks counted by the QEI
	int32_t last_ticks; // Last ticks counted by the QEI

	PID pid; // Motor speed PID

} PurpinsMotor;


class purpinsRobot
{

public:
    /**
     * Constructor.
     */
	purpinsRobot();

	/**
	 * Set the linear and angular goal speeds for the robot
	 *
	 * @param speed Robot goal speed, linear in m/s, angular in rad/s
	 */
	void setSpeed(RobotSpeed * speed);

	/**
	 * Get the current linear and angular speeds for the robot
	 *
	 *  @param speed Robot speed, linear in m/s, angular in rad/s
	 */
	void getSpeed(RobotSpeed * speed);

    /**
     * Set the goal speeds for the two motors
     *
     * @param motor_speeds Left and right goal motor speeds in rad/s
     */
	void setMotorSpeeds(MotorSpeeds * motor_speeds);

	/**
     * Get the current motor speeds in rad/s
     *
     *  @param motor_speeds Left and right motor speeds in rad/s
     */
	void getMotorSpeeds(MotorSpeeds * motor_speeds);
	/**
	 * Get the left motor speed in rad/s
	 *
	 * @return Left motor speed in rad/s
	 */
	float getLeftMotorSpeed() {return left_motor_.speed;};
	/**
	 * Get the right motor speed in rad/s
	 *
	 * @return Right motor speed in rad/s
	 */
	float getRightMotorSpeed() {return right_motor_.speed;};

	/**
	 * Set the PWM for the two motors
	 *
	 * @param pwm Motors PWM
	 */
	void setPWM(MotorPWMs * pwm);

	/**
	 * Calculates the odometry of the robot
	 */
	void calculateOdometry();

	/**
	 * Get the odometry of the robot
	 *
	 * @param odometry Robot odometry
	 */
	void getOdometry(Pose * odometry);
	/**
	 * Get the robot odometry in x
	 *
	 * @return X in m
	 */
	float getX() {return odometry_.x;};
	/**
	 * Get the robot odometry in y
	 *
	 * @return Y in m
	 */
	float getY() {return odometry_.y;};
	/**
	 * Get the robot odometry in yaw
	 *
	 * @return Yaw in rad
	 */
	float getYaw() {return odometry_.yaw;};

	/**
	 * Reset the odometry of the robot
	 */
	void resetOdometry();

	/**
	 * Set the global pose of the robot
	 *
	 * @param pose Robot global pose
	 */
	void setGlobalPose(Pose * pose);

	/**
     * Get the QEI ticks for the two motors
     *
     * @param encoder_pulses Left and right encoder ticks
     */
	void getEncoderTicks(EncoderPulses * encoder_pulses);
	/**
	 * Get the left encoder ticks
	 *
	 * @return Left encoder ticks
	 */
	int32_t getLeftTicks() {return left_motor_.ticks;};
	/**
	 * Get the right encoder ticks
	 *
	 * @return Right encoder ticks
	 */
	int32_t getRightTicks() {return right_motor_.ticks;};

	PIDGains * leftPIDGains() {return &left_motor_.pid.gains;};
	PIDGains * rightPIDGains() {return &right_motor_.pid.gains;};

private:
	PurpinsMotor left_motor_;
	PurpinsMotor right_motor_;

	Pose odometry_;
	Pose global_pose_;

	/**
     * Configure the QEI
     *
     */
	void configureQEI();

	/**
     * Configure the motor PWM
     *
     */
	void configurePWM();

};

extern "C"
{
	void leftQEIHandler();
	void rightQEIHandler();
}

#endif

// EOF
