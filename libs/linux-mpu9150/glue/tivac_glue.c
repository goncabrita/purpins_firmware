////////////////////////////////////////////////////////////////////////////
//
//  This file is part of linux-mpu9150
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_i2c.h>
#include <inc/hw_ints.h>
#include <driverlib/interrupt.h>
#include <driverlib/i2c.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/gpio.h>
#include <driverlib/eeprom.h>
#include <driverlib/fpu.h>
#include <driverlib/uart.h>
#include <driverlib/pin_map.h>
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include <stdint.h>

#include "tivac_glue.h"

extern void delayMSec(unsigned long msec);
extern unsigned long millis(void);


void __no_operation(void) { }

//PB2/PB3 i2c 0

void linux_set_i2c_bus(int bus)
{
	//
	// For this example I2C0 is used with PortB[3:2].  The actual port and
	// pins used may be different on your part, consult the data sheet for
	// more information.  GPIO port B needs to be enabled so these pins can
	// be used.
	//

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//
	// The I2C0 peripheral must be enabled before use.
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

	//
	// Configure the pin muxing for I2C0 functions on port B2 and B3.
	// This step is not necessary if your part does not support pin muxing.
	//

	//
	// Set GPIO B2 and B3 as I2C pins.
	//
	MAP_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	MAP_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	MAP_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);

	MAP_I2CMasterInitExpClk(I2C0_BASE,MAP_SysCtlClockGet(),true);  //false = 100khz , true = 400khz
}


int linux_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
		unsigned char length, unsigned char const *data)
{

	unsigned char num = length;


	MAP_I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);  // false = write.  true = read.
	MAP_I2CMasterDataPut(I2C0_BASE, reg_addr);

	if (length == 0) {

		MAP_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
		// Wait until done transmitting
		while(MAP_I2CMasterBusy(I2C0_BASE));

		return 1; // all done
	}else{

		MAP_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		// Wait until done transmitting
		while(MAP_I2CMasterBusy(I2C0_BASE));

		num--;

		//
		// Continue sending consecutive data
		//
		while(num > 1)
		{
			MAP_I2CMasterDataPut(I2C0_BASE, *data++);
			MAP_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
			while(MAP_I2CMasterBusy(I2C0_BASE));
			num--;
		}

		//
		// Send last piece of data and a STOP
		//
		MAP_I2CMasterDataPut(I2C0_BASE, *data);
		MAP_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
		while(MAP_I2CMasterBusy(I2C0_BASE));

		return length;

	}

	return 0;
}

int linux_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
		unsigned char length, unsigned char *data)
{
	unsigned char num = length;

	linux_i2c_write(slave_addr, reg_addr, 0, NULL);


	MAP_I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);  // false = write.  true = read.

	// Send single piece of data if it's the only piece to send
	if (length == 1){
		MAP_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
		// Wait until done transmitting
		while(MAP_I2CMasterBusy(I2C0_BASE));
		*data++ = MAP_I2CMasterDataGet(I2C0_BASE);
		num--;
		return 1; // all done
	}

	// We have multiple data to send
	//
	// Start sending the first byte of the burst (already loaded with I2CMasterDataPut)
	//
	MAP_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	// Wait until done transmitting
	while(MAP_I2CMasterBusy(I2C0_BASE));

	*data++ = MAP_I2CMasterDataGet(I2C0_BASE);
	num--;

	//
	// Continue sending consecutive data
	//
	while(num > 1)
	{
		MAP_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
		while(MAP_I2CMasterBusy(I2C0_BASE));
		*data++ = MAP_I2CMasterDataGet(I2C0_BASE);
		num--;
	}

	//
	// Send last piece of data and a STOP
	//
	MAP_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while(MAP_I2CMasterBusy(I2C0_BASE));
	*data++ = MAP_I2CMasterDataGet(I2C0_BASE);
	num--;

	return length;
}

int linux_delay_ms(unsigned long num_ms)
{
	delayMSec(num_ms);

	return 0;
}

int linux_get_ms(unsigned long *count)
{
	*count=millis();
	return 0;
}

