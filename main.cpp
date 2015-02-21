/*
 * main.cpp
 *
 *  Created on: Apr 10, 2014
 *      Author: bgouveia
 */


/*! \mainpage Purpins
 *
 * \image html purpins.jpg
 *
 * \section intro_sec Introduction
 *
 * The aim of this project is to develop a cheap mobile robot for being used at the LSE for mobile robot swarming experiments.
 * The main goal is to allow for
 * swarming experimentation at a small cost without sacrificing robot quality. It might however also be used for other types of research such as multi-robot experimentation or even as a single robot
 *
 * \section install_sec Description
 *
 * \subsection arch Architecture
 *
 * \image html architecture.svg
 *
 * \image latex architecture.eps "Purpins Architecture" width=\textwidth
 *
 * \subsection power Power Supply
 *
 * \image html power.svg
 * \image latex power.eps "Purpins Power Supply" width=\textwidth
 *
 * \subsection schem Schematic
 *
 * \image html purpins_schematic.svg
 * \image latex purpins_schematic.eps "Purpins Schematic" width=\textwidth
 *
 *
 * \subsection in Launchpad Modifications
 *
 * To separate the PD0 pin from PB6 and PD1 from PB7 (pins connected to maintain compatibility with some MSP430 Boosterpacks) you need to unsolder the R9 and and R10 resistors from the Tiva-C EK-TM4C123GXL Board.
 * \image html tiva_closeup.jpg
 * \image latex tiva_closeup.eps "R9 and R10 Resistors"
 *
 *
 * \subsection cn Compilation Notes
 *
 * Depending on the IMU available on the board (MPU6050 or MPU9150) you need to enable the "MPU6050" or "MPU9150" and "AK8975_SECONDARY" preprocessor symbols respectively.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include <driverlib/debug.h>
#include <driverlib/fpu.h>
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/gpio.h>
#include <cstring>
#include "SerialUARTImpl.h"
#include "purpinsComm.h"
#include "purpinsRobot.h"
#include "libs/linux-mpu9150/mpu9150/mpu9150.h"

#define VERSION 11

#define SYSTICKS_PER_SECOND 1000

unsigned long milliSec = 0;
unsigned long ulClockMS = 0;

extern "C"
{
	int decimalOf(float val)
	{
		int retval = (val-(int)val)*100;
		return (retval>0)?retval:-retval;
	}

	void delayMSec(unsigned long msec)
	{
		MAP_SysCtlDelay(ulClockMS*msec);
	}

	void delayuSec(unsigned long usec)
	{
		MAP_SysCtlDelay((ulClockMS/1000)*usec);
	}

	void SysTickHandler(void)
	{
		milliSec++;
	}

	unsigned long millis(void)
	{
		return milliSec;
	}

}

int main()
{
	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	MAP_FPUEnable();
	MAP_FPULazyStackingEnable();

	//
	// Set the clocking to run from the PLL at 50MHZ, change to SYSCTL_SYSDIV_2_5 for 80MHz if neeeded
	//
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	MAP_IntMasterDisable();

	SerialAbstract * serial = new SerialUARTImpl();
	purpinsComm communication(*serial);

	purpinsRobot purpins;

	//
	// MPU6050 configuration
	//
	mpudata_t mpu;
	unsigned long sample_rate = 10 ;
	bool mpu_available = mpu9150_init(0, sample_rate, 0);
	memset(&mpu, 0, sizeof(mpudata_t));

	//
	// Get the current processor clock frequency.
	//
	ulClockMS = MAP_SysCtlClockGet() / (3 * 1000);

	unsigned long loop_delay = (1000 / sample_rate) - 2;

	//
	// Configure SysTick to occur 1000 times per second
	//
	MAP_SysTickPeriodSet(MAP_SysCtlClockGet() / SYSTICKS_PER_SECOND);
	MAP_SysTickIntEnable();
	MAP_SysTickEnable();

	MAP_IntMasterEnable();

	uint8_t data[SERIAL_BUFFER_SIZE];
	uint8_t action;

	while(1)
	{
		action = communication.getMsg(data);

		// If we got an action...
		if(action > 0)
		{
			// Process it!!!
			if(action == PP_ACTION_GET_VERSION)
			{
				uint8_t version = VERSION;
				communication.sendMsg(PP_ACTION_GET_VERSION, (void*)(&version), 1);
			}
			else if(action == PP_ACTION_DRIVE)
			{
				RobotSpeed speed;
				communication.parse(action, data, (void*)(&speed));
				purpins.setSpeed(&speed);
			}
			else if(action == PP_ACTION_DRIVE_MOTORS)
			{
				MotorSpeeds motor_speeds;
				communication.parse(action, data, (void*)(&motor_speeds));
				purpins.setMotorSpeeds(&motor_speeds);
			}
			else if(action == PP_ACTION_DRIVE_PWM)
			{
				MotorPWMs motor_pwms;
				communication.parse(action, data, (void*)(&motor_pwms));
				purpins.setPWM(&motor_pwms);
			}
			else if(action == PP_ACTION_GET_ODOMETRY)
			{
				Pose odometry;
				purpins.getOdometry(&odometry);
				communication.sendMsg(PP_ACTION_GET_ODOMETRY, (void*)(&odometry), sizeof(odometry));
			}
			else if(action == PP_ACTION_GET_MOTOR_SPEEDS)
			{
				MotorSpeeds speed;
				purpins.getMotorSpeeds(&speed);
				communication.sendMsg(PP_ACTION_GET_MOTOR_SPEEDS, (void*)(&speed), sizeof(speed));
			}
			else if(action == PP_ACTION_GET_ENCODER_PULSES)
			{
				EncoderPulses encoder_pulses;
				purpins.getEncoderTicks(&encoder_pulses);
				communication.sendMsg(PP_ACTION_GET_ENCODER_PULSES, (void*)(&encoder_pulses), sizeof(encoder_pulses));
			}
			else if(action == PP_ACTION_GET_IMU)
			{
				if(mpu_available)
				{
					IMU imu_data;
					imu_data.orientation_x = mpu.fusedQuat[0];
					imu_data.orientation_y = mpu.fusedQuat[1];
					imu_data.orientation_z = mpu.fusedQuat[2];
					imu_data.orientation_w = mpu.fusedQuat[4];
					imu_data.linear_acceleration_x = mpu.calibratedAccel[0];
					imu_data.linear_acceleration_y = mpu.calibratedAccel[1];
					imu_data.linear_acceleration_z = mpu.calibratedAccel[2];
					imu_data.angular_velocity_x = mpu.calibratedMag[0];
					imu_data.angular_velocity_y = mpu.calibratedMag[1];
					imu_data.angular_velocity_z = mpu.calibratedMag[2];
					communication.sendMsg(PP_ACTION_GET_IMU, (void*)(&imu_data), sizeof(imu_data));
				}
				else
				{
					communication.error(PP_ERROR_SENSOR_NOT_AVAILABLE);
				}
			}
			else if(action == PP_ACTION_GET_IR_SENSORS)
			{
				// TODO: Add the IR sensors
			}
			else if(action == PP_ACTION_GET_GAS_SENSOR)
			{
				// TODO: Add the gas sensor
			}
			else if(action == PP_ACTION_SET_SENSORS_PACK)
			{

			}
			else if(action == PP_ACTION_GET_SENSORS_PACK)
			{

			}
			else if(action == PP_ACTION_SET_SENSOR_STREAMING)
			{

			}
			else if(action == PP_ACTION_SET_GLOBAL_POSE)
			{

			}
			else if(action == PP_ACTION_SET_NEIGHBORS_POSES)
			{

			}
			else if(action == PP_ACTION_SET_PID_GAINS)
			{
				// TODO: Finish this
			}
			else if(action == PP_ACTION_GET_PID_GAINS)
			{
				// TODO: Finish this
			}
			else if(action == PP_ACTION_SET_ODOMETRY_CALIBRATION)
			{
				// TODO: Finish this
			}
			else if(action == PP_ACTION_GET_ODOMETRY_CALIBRATION)
			{
				// TODO: Finish this
			}

		} // if(action > 0)
	}
	return 0;
}
