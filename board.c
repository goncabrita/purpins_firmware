//*****************************************************************************
//
// board.c -  Board functions for EK-TM4C123GXL Tiva C Series LaunchPad.
//
// Copyright (c) 2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "driverlib/timer.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"
#include "driverlib/fpu.h"
#include "driverlib/debug.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "wlan.h"
#include "evnt_handler.h"
#include "nvmem.h"
#include "socket.h"
#include "cc3000_common.h"
#include "netapp.h"
#include "spi.h"
#include "hci.h"
#include "dispatcher.h"
#include "spi_version.h"
#include "board.h"

//*****************************************************************************
//
//! pio_init
//!
//! \param  none
//!
//! \return none
//!
//! \brief  Initialize the board's I/O
//
//*****************************************************************************
void pio_init()
{
    //
    // Configure the system peripheral bus that IRQ & EN pin are mapped to.
    //
    MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_IRQ_PORT);
    MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_SW_EN_PORT);

    //
    // Disable all the interrupts before configuring the lines.
    //
    MAP_GPIOIntDisable(SPI_GPIO_IRQ_BASE, 0xFF);

    //
    // Configure the WLAN_IRQ pin as an input.
    //
    MAP_GPIOPinTypeGPIOInput(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN);
    GPIOPadConfigSet(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN, GPIO_STRENGTH_2MA,
                        GPIO_PIN_TYPE_STD_WPU);

    //
    // Setup the GPIO interrupt for this pin
    //
    MAP_GPIOIntTypeSet(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN, GPIO_FALLING_EDGE);

    //
    // Configure the pins for the enable signal to the CC3000.
    //
    MAP_GPIOPinTypeGPIOOutput(SPI_GPIO_SW_EN_BASE, SPI_EN_PIN);
    MAP_GPIODirModeSet( SPI_GPIO_SW_EN_BASE, SPI_EN_PIN, GPIO_DIR_MODE_OUT );
    MAP_GPIOPadConfigSet( SPI_GPIO_SW_EN_BASE, SPI_EN_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD );

    MAP_GPIOPinWrite(SPI_GPIO_SW_EN_BASE, SPI_EN_PIN, PIN_LOW);
    SysCtlDelay(600000);
    SysCtlDelay(600000);
    SysCtlDelay(600000);

    //
    // Disable WLAN CS with pull up Resistor
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SPI_PORT);
    MAP_GPIOPinTypeGPIOOutput(SPI_CS_PORT, SPI_CS_PIN);
    GPIOPadConfigSet(SPI_CS_PORT, SPI_CS_PIN, GPIO_STRENGTH_2MA,
                                GPIO_PIN_TYPE_STD_WPU);
    MAP_GPIOPinWrite(SPI_CS_PORT, SPI_CS_PIN, PIN_HIGH);

    //
    // Enable interrupt for WLAN_IRQ pin
    //
    MAP_GPIOIntEnable(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN);

    //
    // Clear interrupt status
    //
    SpiCleanGPIOISR();

    MAP_IntEnable(INT_GPIO_SPI);
}

//*****************************************************************************
//
//! ReadWlanInterruptPin
//!
//! \param  none
//!
//! \return none
//!
//! \brief  return wlan interrup pin
//
//*****************************************************************************
long ReadWlanInterruptPin(void)
{
    return MAP_GPIOPinRead(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN);
}

//*****************************************************************************
//
//! Enable waln IrQ pin
//!
//! \param  none
//!
//! \return none
//!
//! \brief  Nonr
//
//*****************************************************************************
void WlanInterruptEnable()
{
    MAP_GPIOIntEnable(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN);
}

//*****************************************************************************
//
//! Disable waln IrQ pin
//!
//! \param  none
//!
//! \return none
//!
//! \brief  Nonr
//
//*****************************************************************************
void WlanInterruptDisable()
{
    MAP_GPIOIntDisable(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN);
}

//*****************************************************************************
//
//! WriteWlanPin
//!
//! \param  new val
//!
//! \return none
//!
//! \brief  This functions enables and disables the CC3000 Radio
//
//*****************************************************************************
void WriteWlanPin( unsigned char val )
{
    if(val)
    {
        MAP_GPIOPinWrite(SPI_GPIO_SW_EN_BASE, SPI_EN_PIN,PIN_HIGH);
    }
    else
    {
        MAP_GPIOPinWrite(SPI_GPIO_SW_EN_BASE, SPI_EN_PIN, PIN_LOW);
    }

}

