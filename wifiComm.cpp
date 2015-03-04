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
 * Author: Goncalo Cabrita on 01/03/2015
 *********************************************************************/

//#include "netapp.h"
//#include "security.h"
#include "wlan.h"
#include "evnt_handler.h"
#include "nvmem.h"
#include "cc3000_common.h"
#include "spi.h"
#include "hci.h"
#include "spi_version.h"
#include "board.h"
#include "host_driver_version.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "wifiComm.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

//*****************************************************************************
//
//  Global variables used by this program for event tracking.
//
//*****************************************************************************
volatile uint32_t g_ui32SmartConfigFinished;
volatile uint32_t g_ui32CC3000Connected;
volatile uint32_t g_ui32CC3000DHCP;
volatile uint32_t g_ui32OkToDoShutDown;
volatile uint32_t g_ui32CC3000DHCP_configured;
volatile uint8_t g_ui8StopSmartConfig;

char *g_ppcSecurity[] = {"Open", "WEP", "WPA", "WPA2"};

WiFiComm::WiFiComm()
{
	socket_ = SENTINEL_EMPTY;
	initCC3000();
}

WiFiComm::~WiFiComm()
{
	if(socket_ != SENTINEL_EMPTY) closeSocket();
}

unsigned int WiFiComm::read(char * buffer, unsigned int max_length)
{
	if((g_ui32CC3000DHCP == 0) || (g_ui32CC3000Connected == 0)) wlan_connect(network_.security, network_.ssid, ustrlen(network_.ssid), NULL, (unsigned char*)network_.key, ustrlen(network_.key));
	if(socket_ == SENTINEL_EMPTY) openSocket();

	return recv(socket_, buffer, max_length, 0);
}

void WiFiComm::write(const char * buffer, unsigned int length)
{
	if((g_ui32CC3000DHCP == 0) || (g_ui32CC3000Connected == 0)) wlan_connect(network_.security, network_.ssid, ustrlen(network_.ssid), NULL, (unsigned char*)network_.key, ustrlen(network_.key));
	if(socket_ == SENTINEL_EMPTY) openSocket();

	send(socket_, buffer, length, 0);
}

uint32_t WiFiComm::initCC3000()
{
	//
	// Set the system clock and initialize GPIOs used in this configuration.
	//
	pio_init();

	//
	// Initialize the SPI and IRQ lines connecting the CC3000.  We run the SPI
	// interface at 1MHz.
	//
	init_spi(1000000, MAP_SysCtlClockGet());

	UARTprintf("[CC3000] Initializing CC3000\n");
	//
	// Tell the WiFi driver which application- and board-specific functions
	// to call in response to various events and when interrupt and pin
	// control is required.
	//
	wlan_init(CC3000AsyncCallback, sendWLFWPatch, sendDriverPatch,
			  sendBootLoaderPatch, ReadWlanInterruptPin,
			  WlanInterruptEnable, WlanInterruptDisable,
			  WriteWlanPin);

	UARTprintf("[CC3000]Starting WiFi stack\n");
	//
	// Start the WiFi stack.
	//
	wlan_start(0);

	//
	// Mask out all non-required events from CC3000.
	//
	wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE | HCI_EVNT_WLAN_UNSOL_INIT |
						HCI_EVNT_WLAN_ASYNC_PING_REPORT);

	MAP_SysCtlDelay(1000000);

	// Print version string.
	UARTprintf("[CC3000] SPI/Driver version %d.%d\n", SPI_VERSION_NUMBER, DRIVER_VERSION_NUMBER );

	//
	// Set flag to stop smart config if running.
	//
	g_ui8StopSmartConfig = 0;

	return(0);
}

//*****************************************************************************
//
// Open socket.
//
//*****************************************************************************
uint32_t WiFiComm::openSocket()
{
	int32_t i32Check = 0;
	sockaddr tSocketAddr;

	UARTprintf("[CC3000] Opening socket: Waiting for DHCP process to finish\n");
	//
	// Wait for DHCP process to finish. If you are using a static IP address
	// please delete the wait for DHCP event - ulCC3000DHCP
	//
	while((g_ui32CC3000DHCP == 0) || (g_ui32CC3000Connected == 0))
	{
		hci_unsolicited_event_handler();
		ROM_SysCtlDelay(1000);
	}

	UARTprintf("[CC3000] Socket open!\n");
	//
	// Open socket.
	//
	i32Check = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	//
	// Error checking.
	//
	if(i32Check >= 0)
	{
		socket_ = i32Check;
	}
	else
	{
		UARTprintf("[CC3000] Socket Function returned an error."
							"Socket not opened. Error code %d.\n",i32Check);
		return(0);
	}

	//
	// The family is always AF_INET on CC3000.
	//
	tSocketAddr.sa_family = AF_INET;

	//
	// The destination port.
	//
	uint32_t ui32Port = server_.port;
	tSocketAddr.sa_data[0] = (ui32Port & 0xFF00) >> 8;
	tSocketAddr.sa_data[1] = (ui32Port & 0x00FF) >> 0;

	//
	// The destination IP address - http://cool-farm.com
	//
	tSocketAddr.sa_data[2] = server_.ip[0];
	tSocketAddr.sa_data[3] = server_.ip[1];
	tSocketAddr.sa_data[4] = server_.ip[2];
	tSocketAddr.sa_data[5] = server_.ip[3];

	//
	// Connect to TCP Socket on Server (if not already conneted)
	//
	i32Check = connect(socket_, &tSocketAddr, sizeof(sockaddr));
	if(i32Check != 0)
	{
		UARTprintf("[CC3000] Socket connect failed with error code '%d'\n", i32Check);
		UARTprintf("Please make sure there is a server with the "
				   "specified socket open to connect to\n");

		return(0);
	}

	unsigned long timeout = 10; //milliseconds
	if(setsockopt(socket_, SOL_SOCKET, SOCKOPT_RECV_TIMEOUT, &timeout, sizeof(timeout)))
	{
		return(0);
	}

	return(socket_);
}

//*****************************************************************************
//
// Open socket.
//
//*****************************************************************************
uint8_t WiFiComm::closeSocket()
{
	int32_t i32Check = 0;
	//
	// Close the socket.
	//
	i32Check = closesocket(socket_);

	//
	// Error checking.
	//
	if(i32Check == 0)
	{
		socket_ = SENTINEL_EMPTY;
		return(1);
	}
	return(0);
}

uint8_t WiFiComm::resetCC3000()
{
    //
    // Stop the CC3000. No return value provided.
    //
    wlan_stop();

    //
    // Wait a bit.
    //
    ROM_SysCtlDelay(100000);

    //
    // Restart the CC3000.
    //
    wlan_start(0);

    return(0);
}

//*****************************************************************************
//
// The function, used during CC3000 initialization, returns a pointer to the
// driver patch.  Since there is no patch provided at runtime (the patches
// are taken from the EEPROM and not from the host), it returns 0.
//
//*****************************************************************************
char* sendDriverPatch(unsigned long *length)
{
    *length = 0;
    return((char *)0);
}

//*****************************************************************************
//
// The function, used during CC3000 initialization, returns a pointer to the
// bootloader patch.  Since there is no patch provided at runtime (the patches
// are taken from the EEPROM and not from the host), it returns 0.
//
//*****************************************************************************
char* sendBootLoaderPatch(unsigned long *length)
{
    *length = 0;
    return((char *)0);
}

//*****************************************************************************
//
// The function, used during CC3000 initialization, returns a pointer to the
// firmware patch.  Since there is no patch provided at runtime (the patches
// are taken from the EEPROM and not from the host), it returns NULL.
//
//*****************************************************************************
char* sendWLFWPatch(unsigned long *length)
{
    *length = 0;
    return((char *)0);
}

//*****************************************************************************
//
// Handle asynchronous events from the CC3000.
//
//*****************************************************************************
void CC3000AsyncCallback(long lEventType, char *pcData, unsigned char ucLength)
{
	netapp_pingreport_args_t *psPingData;

	//
	// Handle completion of simple config callback
	//
	if(lEventType == HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE)
	{
		UARTprintf("[CC3000] Received Asynchronous Simple Config Done Event "
								 "from CC3000\n>",0x08);

		g_ui32SmartConfigFinished = 1;
		g_ui8StopSmartConfig = 1;
	}

	//
	// Handle unsolicited connect callback.
	//
	if(lEventType == HCI_EVNT_WLAN_UNSOL_CONNECT)
	{
		//
		// Set global variable to indicate connection.
		//
		g_ui32CC3000Connected = 1;
	}

	//
	// Handle unsolicited disconnect callback.
	//
	if(lEventType == HCI_EVNT_WLAN_UNSOL_DISCONNECT)
	{
		UARTprintf("[CC3000] Received Unsolicited Disconnect from CC3000\n>");

		g_ui32CC3000Connected = 0;
		g_ui32CC3000DHCP = 0;
		g_ui32CC3000DHCP_configured = 0;
	}

	//
	// Handle DHCP connection callback.
	//
	if(lEventType == HCI_EVNT_WLAN_UNSOL_DHCP)
	{
		//
		// Notes:
		// 1) IP config parameters are received swapped
		// 2) IP config parameters are valid only if status is OK,
		//      i.e. g_ui32CC3000DHCP becomes 1
		//

		//
		// Only if status is OK, the flag is set to 1 and the addresses are
		// valid
		//
		if( *(pcData + NETAPP_IPCONFIG_MAC_OFFSET) == 0)
		{
			UARTprintf("[CC3000] DHCP Connected. IP: %d.%d.%d.%d\n", (unsigned char)pcData[3],(unsigned char)pcData[2],(unsigned char)pcData[1],(unsigned char)pcData[0]);

			//
			// DHCP success, set global accordingly.
			//
			g_ui32CC3000DHCP = 1;
		}
		else
		{
			//
			// DHCP failed, set global accordingly.
			//
			g_ui32CC3000DHCP = 0;
		}
	}

	//
	// Ping event handler
	//
	if(lEventType == HCI_EVNT_WLAN_ASYNC_PING_REPORT)
	{
		//
		// Ping data received, print to screen
		//
		psPingData = (netapp_pingreport_args_t *)pcData;

		UARTprintf("[CC3000] Data Received='\n");
		for(lEventType = 0; lEventType < ucLength; lEventType++)
		{
			UARTprintf("%d,",pcData[lEventType]);
		}
		UARTprintf("'\n");

		//
		// Test for ping failure
		//
		if(psPingData->min_round_time == -1)
		{
			UARTprintf("[CC3000] Ping Failed. Please check address and try again.\n>");
		}
		else
		{
			UARTprintf("[CC3000] Ping Results:\n"
					   "    sent: %d, received: %d, min time: %dms,"
					   " max time: %dms, avg time: %dms\n>",
					   psPingData->packets_sent, psPingData->packets_received,
					   psPingData->min_round_time, psPingData->max_round_time,
					   psPingData->avg_round_time);
		}
	}

	//
	// Handle shutdown callback.
	//
	if(lEventType == HCI_EVENT_CC3000_CAN_SHUT_DOWN)
	{
		//
		// Set global variable to indicate the device can be shutdown.
		//
		g_ui32OkToDoShutDown = 1;
	}
}
