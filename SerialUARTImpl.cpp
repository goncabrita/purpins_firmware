/*
 * SerialUARTImpl.cpp
 *
 *  Created on: Sep 16, 2014
 *      Author: bgouveia
 */


#include "SerialUARTImpl.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
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
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"



SerialUARTImpl::SerialUARTImpl() {
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Set GPIO A0 and A1 as UART pins.
	//
	MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
	MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0,115200,MAP_SysCtlClockGet());
}


SerialUARTImpl::~SerialUARTImpl(){

}

unsigned int SerialUARTImpl::available(){
	return UARTRxBytesAvail();
}

char SerialUARTImpl::read(){
	return UARTgetc();
}

void SerialUARTImpl::println(char * string){
	UARTprintf("%s\n",string);
}
