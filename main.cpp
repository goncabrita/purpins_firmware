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
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>


#include "SerialUARTImpl.h"
#include "purpinsMotors.h"
#include "purpinsComm.h"

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
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ);


	MAP_IntMasterDisable();

	SerialAbstract * serial = new SerialUARTImpl();
	purpinsMotors motors();

	// Get the current processor clock frequency.
	ulClockMS = MAP_SysCtlClockGet() / (3 * 1000);

	purpinsComm communication(*serial);


	//
	// Configure SysTick to occur 1000 times per second
	//
	MAP_SysTickPeriodSet(MAP_SysCtlClockGet() / SYSTICKS_PER_SECOND);
	MAP_SysTickIntEnable();
	MAP_SysTickEnable();


	MAP_IntMasterEnable();




	while(1){

	}
	return 0;
}


