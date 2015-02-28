/*
 * wifiComm.c
 *
 *  Created on: Jun 3, 2014
 *      Author: cabrita
 */

#ifndef WIFICOMM_C_
#define WIFICOMM_C_

#include "wifiComm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

char g_pui8Data[512];
char g_pui8SensorData[256];

char *g_ppcSecurity[] = {"Open", "WEP", "WPA", "WPA2"};

//*****************************************************************************
//
//  Global variables used by this program for event tracking.
//
//*****************************************************************************
volatile uint32_t g_ui32SmartConfigFinished, g_ui32CC3000Connected,
                  g_ui32CC3000DHCP,g_ui32OkToDoShutDown,
                  g_ui32CC3000DHCP_configured;

//*****************************************************************************
//
//  Smart Config flag variable. Used to stop Smart Config.
//
//*****************************************************************************
volatile uint8_t g_ui8StopSmartConfig;

//*****************************************************************************
//
// The function, used during CC3000 initialization, returns a pointer to the
// driver patch.  Since there is no patch provided at runtime (the patches
// are taken from the EEPROM and not from the host), it returns 0.
//
//*****************************************************************************
char *sendDriverPatch(unsigned long *Length)
{
    *Length = 0;
    return((char *)0);
}

//*****************************************************************************
//
// The function, used during CC3000 initialization, returns a pointer to the
// bootloader patch.  Since there is no patch provided at runtime (the patches
// are taken from the EEPROM and not from the host), it returns 0.
//
//*****************************************************************************
char *sendBootLoaderPatch(unsigned long *Length)
{
    *Length = 0;
    return((char *)0);
}

//*****************************************************************************
//
// The function, used during CC3000 initialization, returns a pointer to the
// firmware patch.  Since there is no patch provided at runtime (the patches
// are taken from the EEPROM and not from the host), it returns NULL.
//
//*****************************************************************************
char *sendWLFWPatch(unsigned long *Length)
{
    *Length = 0;
    return((char *)0);
}

//*****************************************************************************
//
// Handle asynchronous events from the CC3000.
//
//*****************************************************************************
void CC3000AsyncCallback(long lEventType, unsigned char *pcData, unsigned char ucLength)
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
	// Handle unsolicited disconnect callback. Turn LED Green -> Red.
	//
	if(lEventType == HCI_EVNT_WLAN_UNSOL_DISCONNECT)
	{
		UARTprintf("[CC3000] Received Unsolicited Disconnect from CC3000\n>");
		g_ui32CC3000Connected = 0;
		g_ui32CC3000DHCP = 0;
		g_ui32CC3000DHCP_configured = 0;

		// TODO: Recover from this state
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
			UARTprintf("[CC3000] DHCP Connected. IP: %d.%d.%d.%d\n", pcData[3],pcData[2],pcData[1],pcData[0]);

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
//#ifdef DEBUG
		UARTprintf("[CC3000] Data Received='\n");
		for(lEventType = 0; lEventType < ucLength; lEventType++)
		{
			UARTprintf("%d,",pcData[lEventType]);
		}
		UARTprintf("'\n");
//#endif

		//
		// Test for ping failure
		//
		if(psPingData->min_round_time == -1)
		{
			UARTprintf("[CC3000] Ping Failed. Please check address and try "
					   "again.\n>");
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

//*****************************************************************************
//
// Open socket.
//
//*****************************************************************************
uint32_t CC3000OpenSocket()
{
	int32_t i32Check = 0;
	sockaddr tSocketAddr;
	uint32_t ui32Socket;

	//UARTprintf("\n[Step1] Waiting for DHCP process to finish\n");
	//
	// Wait for DHCP process to finish. If you are using a static IP address
	// please delete the wait for DHCP event - ulCC3000DHCP
	//
	while((g_ui32CC3000DHCP == 0) || (g_ui32CC3000Connected == 0))
	{
		hci_unsolicited_event_handler();

		ROM_SysCtlDelay(1000);
	}

	//UARTprintf("[Step2] Open socket\n");
	//
	// Open socket.
	//
	i32Check = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	//
	// Error checking.
	//
	if(i32Check >= 0)
	{
		ui32Socket = i32Check;
	}
	else
	{
		UARTprintf("\n[CC3000] Socket Function returned an error."
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
	uint32_t ui32Port = 80;
	tSocketAddr.sa_data[0] = (ui32Port & 0xFF00) >> 8;
	tSocketAddr.sa_data[1] = (ui32Port & 0x00FF) >> 0;

	//
	// The destination IP address - http://cool-farm.com
	//
	tSocketAddr.sa_data[2] = 54;
	tSocketAddr.sa_data[3] = 81;
	tSocketAddr.sa_data[4] = 30;
	tSocketAddr.sa_data[5] = 183;

	//UARTprintf("[Step3] Connect\n>");
	//
	// Connect to TCP Socket on Server (if not already conneted)
	//
	i32Check = connect(ui32Socket, &tSocketAddr, sizeof(sockaddr));
	if(i32Check != 0)
	{
		UARTprintf("\n[CC3000] Connect failed with error code '%d'\n", i32Check);
		UARTprintf("Please make sure there is a server with the "
				   "specified socket open to connect to\n");
		return(0);
	}
	return(ui32Socket);
}

//*****************************************************************************
//
// Sends sensor data to the Coolfarm server.
//
//*****************************************************************************
uint8_t CC3000UpdateSensorData(uint32_t ui32Socket)
{
	int32_t i32Check = 0;
	uint32_t ui32DataLength;

	//
	// Send TCP Packet.
	// http://api.cool-farm.com/update?coolfarm_id=testcool&coolfarm_key=testpassword&light=295&water_level=11.34
	//
	usprintf(g_pui8Data,
			 "GET /update%s HTTP/1.0\r\n"
			 "Host: api-coolfarm.rhcloud.com\r\n"
			 "Accept-Charset: ISO-8859-1,UTF-8;q=0.7,*;q=0.7\r\n"
		 	 "Connection: keep-alive\r\n"
			 "Cache-Control: no-cache\r\n"
			 "Accept-Language: de,en;q=0.7,en-us;q=0.3\r\n"
			 "\r\n", g_pui8SensorData);

	//UARTprintf("[Step4] Send data\n");
	ui32DataLength = ustrlen(g_pui8Data);
	//UARTprintf("\n[CC3000] Sending data... ");
	i32Check = send(ui32Socket, g_pui8Data, ui32DataLength, 0);
	//UARTprintf("off you go!\n", i32Check);

	//
	// Validate completion of send.
	//
	if(i32Check == -1)
	{
		UARTprintf("\n[CC3000] Send Data Failed with code '%d'\n", i32Check);
		return(0);
	}
	return(1);
}

//*****************************************************************************
//
// Open socket.
//
//*****************************************************************************
uint8_t CC3000CloseSocket(uint32_t ui32Socket)
{
	int32_t i32Check = 0;
	//UARTprintf("[Step5] Close socket\n");
	//
	// Close the socket.
	//
	i32Check = closesocket(ui32Socket);

	//
	// Error checking.
	//
	if(i32Check == 0)
	{
		ui32Socket = SENTINEL_EMPTY;
		return(1);
	}
	else
	{
		UARTprintf("[CC3000] Socket close Failed.\n");
	}
	return(0);
}

uint8_t CC3000Reset()
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


#endif /* WIFICOMM_C_ */
