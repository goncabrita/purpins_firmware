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
 * This is the introduction.
 *
 * \section install_sec Description
 *
 * \subsection arch Architecture
 *
 * \image html architecture.svg
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
#include <driverlib/i2c.h>
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/gpio.h>
#include <cstring>



#include "SerialUARTImpl.h"
#include "purpinsMotors.h"
#include "purpinsComm.h"

#include "utils/uartstdio.h"

#include "libs/linux-mpu9150/mpu9150/mpu9150.h"

#define SYSTICKS_PER_SECOND     1000


unsigned long milliSec = 0;
unsigned long ulClockMS=0;

extern "C" {

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

void print_fused_euler_angles(mpudata_t *mpu)
{
	//UARTprintf("acc scale: %d.%02d,%d.%02d,%d.%02d\n", (int)x, decimalOf(x)

    float x,y,z;

	x=mpu->fusedEuler[VEC3_X] * RAD_TO_DEGREE;
	y=mpu->fusedEuler[VEC3_Y] * RAD_TO_DEGREE;
	z=mpu->fusedEuler[VEC3_Z] * RAD_TO_DEGREE;

	UARTprintf("\rX: %d.%02d Y: %d.%02d Z: %d.%02d\n",(int)x, decimalOf(z),(int)y, decimalOf(y),(int)z, decimalOf(z));
}

int main(){

	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	MAP_FPUEnable();
	MAP_FPULazyStackingEnable();

	//
	// Set the clocking to run from the PLL.
	//
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ);


	MAP_IntMasterDisable();

	SerialAbstract * serial = new SerialUARTImpl();
	purpinsMotors motors();

	// Get the current processor clock frequency.
	ulClockMS = MAP_SysCtlClockGet() / (3 * 1000);

	//purpinsComm communication(*serial);

	mpudata_t mpu;

	unsigned long sample_rate = 10 ;

	//mpu9150_set_debug(1);
	serial->println("Initializing MPU_6050...");

	if (mpu9150_init(0,sample_rate, 0)){
		serial->println("MPU6050 - MPU6050 connection failed");
	}
	memset(&mpu, 0, sizeof(mpudata_t));


	//
	// Configure SysTick to occur 1000 times per second
	//
	MAP_SysTickPeriodSet(MAP_SysCtlClockGet() / SYSTICKS_PER_SECOND);
	MAP_SysTickIntEnable();
	MAP_SysTickEnable();

	MAP_IntMasterEnable();

	unsigned long loop_delay = (1000 / sample_rate) - 2;

	while(1){


		if (mpu9150_read(&mpu) == 0) {
			print_fused_euler_angles(&mpu);
		}

		delayMSec(loop_delay);

	}
	return 0;
}
