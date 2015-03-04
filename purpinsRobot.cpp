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

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <cstring>
#include <math.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/qei.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"

#include "purpinsRobot.h"

static PurpinsMotor * left_motor_ptr;
static PurpinsMotor * right_motor_ptr;
static unsigned long pwm_period;

purpinsRobot::purpinsRobot()
{
	left_motor_.closed_loop = false;
	left_motor_.speed = 0.0;
	left_motor_.target_speed = 0.0;
	left_motor_.ticks = 0;
	left_motor_.last_ticks = 0;
	left_motor_.pid.gains.kp = 100.0;
	left_motor_.pid.gains.ki = 10.0;
	left_motor_.pid.gains.kd = 0.1;
	left_motor_.pid.sample_time = QEI_RATE;

	right_motor_.closed_loop = false;
	right_motor_.speed = 0.0;
	right_motor_.target_speed = 0.0;
	right_motor_.ticks = 0;
	right_motor_.last_ticks = 0;
	right_motor_.pid.gains.kp = 100.0;
	right_motor_.pid.gains.ki = 10.0;
	right_motor_.pid.gains.kd = 0.1;
	right_motor_.pid.sample_time = QEI_RATE;

	left_motor_ptr = &left_motor_;
	right_motor_ptr = &right_motor_;

	reset(&left_motor_.pid);
	reset(&right_motor_.pid);

	resetOdometry();

	configureQEI();
	configurePWM();
}

void purpinsRobot::setSpeed(RobotSpeed * speed)
{
	MotorSpeeds motor_speeds;

	motor_speeds.left = speed->linear - PURPINS_AXLE_LENGTH * speed->angular / 2.0;
	motor_speeds.right = speed->linear + PURPINS_AXLE_LENGTH * speed->angular / 2.0;

	setMotorSpeeds(&motor_speeds);
}

void purpinsRobot::getSpeed(RobotSpeed * speed)
{
	speed->linear = (right_motor_.speed + left_motor_.speed) / 2.0;
	speed->angular = (right_motor_.speed - left_motor_.speed) / PURPINS_AXLE_LENGTH;
}

void purpinsRobot::setMotorSpeeds(MotorSpeeds * motor_speeds)
{
	left_motor_.closed_loop = true;
	left_motor_.target_speed = motor_speeds->left;
	right_motor_.closed_loop = false;
	right_motor_.target_speed = motor_speeds->right;
}

void purpinsRobot::getMotorSpeeds(MotorSpeeds * motor_speeds)
{
	motor_speeds->left = left_motor_.speed;
	motor_speeds->right = right_motor_.speed;
}

void purpinsRobot::setPWM(MotorPWMs * pwm)
{
	left_motor_.closed_loop = false;
	left_motor_.pwm = pwm->left;
	right_motor_.closed_loop = false;
	right_motor_.pwm = pwm->right;
}

void purpinsRobot::calculateOdometry()
{
	int32_t left_ticks = left_motor_.ticks - left_motor_.last_ticks;
	int32_t right_ticks = right_motor_.ticks - right_motor_.last_ticks;

	double distance = (right_ticks + left_ticks) * PURPINS_TICKS_TO_M / 2.0;
	double angle = (right_ticks - left_ticks) * PURPINS_TICKS_TO_M / PURPINS_AXLE_LENGTH;

	odometry_.yaw += angle;
	odometry_.x += distance*cos(odometry_.yaw);
	odometry_.y += distance*sin(odometry_.yaw);

	left_motor_.last_ticks = left_motor_.ticks;
	right_motor_.last_ticks = right_motor_.ticks;
}

void purpinsRobot::getOdometry(Pose * odometry)
{
	odometry->x = odometry_.x;
	odometry->y = odometry_.y;
	odometry->yaw = odometry_.yaw;
}

void purpinsRobot::resetOdometry()
{
	odometry_.x = 0.0;
	odometry_.y = 0.0;
	odometry_.yaw = 0.0;
}

void purpinsRobot::setGlobalPose(Pose * pose)
{
	global_pose_.x = pose->x;
	global_pose_.y = pose->y;
	global_pose_.yaw = pose->yaw;
}

void purpinsRobot::getEncoderTicks(EncoderPulses * encoder_pulses)
{
	encoder_pulses->left = left_motor_.ticks;
	encoder_pulses->right = right_motor_.ticks;
}

void purpinsRobot::configureQEI()
{
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	//
	// Set GPIO C5 and C6 as QEI pins.
	//
	MAP_GPIOPinConfigure(GPIO_PC5_PHA1);
	MAP_GPIOPinConfigure(GPIO_PC6_PHB1);
	MAP_GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);

	//
	// Set GPIO D6 and D7 as QEI pins.
	//
	MAP_GPIOPinConfigure(GPIO_PD6_PHA0);
	MAP_GPIOPinConfigure(GPIO_PD7_PHB0);
	MAP_GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	MAP_QEIConfigure(QEI0_BASE,
			(QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP
					| QEI_CONFIG_NO_RESET), PURPINS_TICKS_PER_TURN-1);
	MAP_QEIConfigure(QEI1_BASE,
			(QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP
					| QEI_CONFIG_NO_RESET), PURPINS_TICKS_PER_TURN-1);

	MAP_QEIEnable(QEI0_BASE);
	MAP_QEIEnable(QEI1_BASE);

	MAP_QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1,
			MAP_SysCtlClockGet() / QEI_LOOP_FREQUENCY);
	MAP_QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1,
			MAP_SysCtlClockGet() / QEI_LOOP_FREQUENCY);

	QEIIntRegister(QEI1_BASE, rightQEIHandler);
	QEIIntRegister(QEI0_BASE, leftQEIHandler);

	MAP_IntEnable(INT_QEI0);
	MAP_IntEnable(INT_QEI1);

	MAP_QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
	MAP_QEIIntEnable(QEI1_BASE, QEI_INTTIMER);
	MAP_QEIVelocityEnable(QEI0_BASE);
	MAP_QEIVelocityEnable(QEI1_BASE);
}

void purpinsRobot::configurePWM()
{
	// Configure PWM Clock
	MAP_SysCtlPWMClockSet(SYSCTL_PWMDIV_2);

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

	pwm_period = MAP_SysCtlClockGet() / 2 / PWM_FREQUENCY; //PWM frequency

	MAP_GPIOPinConfigure(GPIO_PF1_M1PWM5);
	MAP_GPIOPinConfigure(GPIO_PF2_M1PWM6);
	MAP_GPIOPinConfigure(GPIO_PF3_M1PWM7);
	MAP_GPIOPinConfigure(GPIO_PC4_M0PWM6);

	MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	MAP_GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);

	// gen 3 for m0pwm6
	MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	// gen 3 for m1pwm6 and m1pwm7
	MAP_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	// gen 2 for m1pwm5
	MAP_PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

	// Set the Period (expressed in clock ticks)
	MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, pwm_period);
	MAP_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, pwm_period);
	MAP_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, pwm_period);

	// Set PWM duty-0%
	MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5 , 0);
	MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6 , 0);
	MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7 , 0);
	MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6 , 0);

	// Enable the PWM generators
	MAP_PWMGenEnable(PWM1_BASE, PWM_GEN_2);
	MAP_PWMGenEnable(PWM1_BASE, PWM_GEN_3);
	MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_3);

	// Turn on the Output pins
	MAP_PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
	MAP_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
	MAP_PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
	MAP_PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
}

void leftQEIHandler()
{
	MAP_QEIIntClear(QEI0_BASE,QEI_INTTIMER);

	left_motor_ptr->ticks = MAP_QEIPositionGet(QEI0_BASE);
	left_motor_ptr->speed = (float)(MAP_QEIVelocityGet(QEI0_BASE))*MAP_QEIDirectionGet(QEI0_BASE)*QEI_LOOP_FREQUENCY*PURPINS_TICKS_TO_RAD;

	// Run PID
	if(left_motor_ptr->closed_loop)
		left_motor_ptr->pwm = run(&left_motor_ptr->pid, left_motor_ptr->target_speed, left_motor_ptr->speed);

	if(left_motor_ptr->pwm == 0)
	{
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 0);
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 0);
	}
	else if(left_motor_ptr->pwm < 0)
	{
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 0);
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, -left_motor_ptr->pwm);
	}
	else
	{
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, left_motor_ptr->pwm);
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 0);
	}
}

void rightQEIHandler()
{
	MAP_QEIIntClear(QEI1_BASE,QEI_INTTIMER);

	right_motor_ptr->ticks = MAP_QEIPositionGet(QEI1_BASE);
	right_motor_ptr->speed = (float)(MAP_QEIVelocityGet(QEI1_BASE))*MAP_QEIDirectionGet(QEI1_BASE)*QEI_LOOP_FREQUENCY*PURPINS_TICKS_TO_RAD;

	// Run PID
	if(right_motor_ptr->closed_loop)
		right_motor_ptr->pwm = run(&right_motor_ptr->pid, right_motor_ptr->target_speed, right_motor_ptr->speed);

	if(right_motor_ptr->pwm == 0)
	{
		MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 0);
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 0);
	}
	else if(right_motor_ptr->pwm < 0)
	{
		MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 0);
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, -right_motor_ptr->pwm);
	}
	else
	{
		MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, right_motor_ptr->pwm);
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 0);
	}
}

// EOF
