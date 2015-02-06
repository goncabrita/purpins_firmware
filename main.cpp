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

#define SYSTICKS_PER_SECOND 1000

#define VERSION 11

// Conversions
#define FLOAT_TO_INT 1000
#define INT_TO_FLOAT 0.001

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
	communication.setID(1);
	communication.setDebug(1);

	purpinsRobot purpins;

	//
	// Get the current processor clock frequency.
	//
	ulClockMS = MAP_SysCtlClockGet() / (3 * 1000);

	mpudata_t mpu;

	unsigned long sample_rate = 10 ;

	//mpu9150_set_debug(1)
	serial->println("Initializing MPU_6050...");

	if(mpu9150_init(0,sample_rate, 0))
	{
		serial->println("MPU6050 - MPU6050 connection failed");
	}
	memset(&mpu, 0, sizeof(mpudata_t));

	unsigned long loop_delay = (1000 / sample_rate) - 2;

	//
	// Configure SysTick to occur 1000 times per second
	//
	MAP_SysTickPeriodSet(MAP_SysCtlClockGet() / SYSTICKS_PER_SECOND);
	MAP_SysTickIntEnable();
	MAP_SysTickEnable();

	MAP_IntMasterEnable();

	int arg[MAX_IN_ARGS];
	int reply_arg[MAX_OUT_ARGS];
	int action = 0;

	while(1)
	{
		action = communication.getMsg(arg);

		// If we got an action...
		if(action > 0)
		{
			// Process it!!!
			switch(action)
			{
				case PP_ACTION_GET_VERSION:
					reply_arg[0] = VERSION;
					communication.reply(PP_ACTION_GET_VERSION, reply_arg, 1);
					break;

				case PP_ACTION_DRIVE:
					purpins.setMotorSpeeds((float)(arg[0]*INT_TO_FLOAT), (float)(arg[1]*INT_TO_FLOAT));
					break;

				case PP_ACTION_DRIVE_DIRECT:
					purpins.setPWM(arg[0], arg[1]);
					break;

				case PP_ACTION_GET_ODOMETRY:
					reply_arg[0] = (int)(purpins.getX()*FLOAT_TO_INT);
					reply_arg[1] = (int)(purpins.getY()*FLOAT_TO_INT);
					reply_arg[2] = (int)(purpins.getYaw()*FLOAT_TO_INT);
					communication.reply(PP_ACTION_GET_ODOMETRY, reply_arg, 3);
					break;

				case PP_ACTION_GET_ENCODER_PULSES:
					reply_arg[0] = (int)(purpins.getLeftTicks());
					reply_arg[1] = (int)(purpins.getRightTicks());
					communication.reply(PP_ACTION_GET_ENCODER_PULSES, reply_arg, 2);
					break;

				case PP_ACTION_GET_WHEEL_VELOCITIES:
					reply_arg[0] = (int)(purpins.getLeftMotorSpeed()*FLOAT_TO_INT);
					reply_arg[1] = (int)(purpins.getRightMotorSpeed()*FLOAT_TO_INT);
					communication.reply(PP_ACTION_GET_WHEEL_VELOCITIES, reply_arg, 2);
					break;

				case PP_ACTION_GET_DEBUG:
					reply_arg[0] = communication.getDebug();
					communication.reply(PP_ACTION_GET_DEBUG, reply_arg, 1);
					break;

				case PP_ACTION_SET_DEBUG:
					communication.setDebug(arg[0]);
					break;

				case PP_ACTION_SET_PID_GAINS:
					break;

				case PP_ACTION_GET_PID_GAINS:
					break;

				case PP_ACTION_SET_ODOMETRY_CALIBRATION:
					break;

				case PP_ACTION_GET_ODOMETRY_CALIBRATION:
					break;

				case PP_ACTION_SET_ID:
					communication.setID(arg[0]);
					break;

				case PP_ACTION_GET_ID:
					reply_arg[0] = communication.getID();
					communication.reply(PP_ACTION_GET_ID, reply_arg, 1);
					break;

			} // switch

		} // if(action > 0)
	}
	return 0;
}


