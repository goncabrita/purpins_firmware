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
* Author: Gonçalo Cabrita and Bruno Gouveia on 16/08/2012
*********************************************************************/

#include <stdint.h>
#include "pid.h"

#ifndef PURPINS_ROBOT_H_
#define PURPINS_ROBOT_H_

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

	pid_t pid; // Motor speed PID

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
	 * @param linear_speed Linear goal speed in m/s
	 * @param angular_speed Angular goal speed in rad/s
	 */
	void setSpeed(float linear_speed, float angular_speed);

	/**
	 * Get the current linear and angular speeds for the robot
	 *
	 *  @param linear_speed Linear robot speed in m/s
	 *  @param angular_speed Angular robot speed in rad/s
	 */
	void getSpeed(float & linear_speed,float & angular_speed);

    /**
     * Set the goal speeds for the two motors
     *
     * @param left_speed Left goal motor speed in rad/s
     * @param right_speed Right goal motor speed in rad/s
     */
	void setMotorSpeeds(float left_speed, float right_speed);

	/**
     * Get the current motor speeds in rad/s
     *
     *  @param left_speed Left motor speed in rad/s
     *  @param right_speed Right motor speed in rad/s
     */
	void getMotorSpeeds(float & left_speed,float & right_speed);
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
	 * @param left_pwm Left goal motor PWM
	 * @param right_pwm Right goal motor PWM
	 */
	void setPWM(int left_pwm, int right_pwm);

	/**
	 * Calculates the odometry of the robot
	 */
	void calculateOdometry();

	/**
	 * Get the odometry of the robot
	 *
	 * @param left_ticks Left encoder ticks
	 * @param right_ticks Right encoder ticks
	 */
	void getOdometry(float & x, float & y, float & yaw);
	/**
	 * Get the robot odometry in x
	 *
	 * @return X in m
	 */
	float getX() {return x_;};
	/**
	 * Get the robot odometry in y
	 *
	 * @return Y in m
	 */
	float getY() {return y_;};
	/**
	 * Get the robot odometry in yaw
	 *
	 * @return Yaw in rad
	 */
	float getYaw() {return yaw_;};

	/**
	 * Reset the odometry of the robot
	 */
	void resetOdometry();

	/**
     * Get the QEI ticks for the two motors
     *
     * @param left_ticks Left encoder ticks
     * @param right_ticks Right encoder ticks
     */
	void getEncoderTicks(int32_t & left_ticks, int32_t & right_ticks);
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

private:
	PurpinsMotor left_motor_;
	PurpinsMotor right_motor_;

	float x_;
	float y_;
	float yaw_;

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
