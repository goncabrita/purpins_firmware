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

#ifndef WIFICOMM_H_
#define WIFICOMM_H_

#include <stdint.h>
#include "abstractComm.h"
#include "purpinsDataTypes.h"

//*****************************************************************************
//
// CC3000 definitions.
//
//*****************************************************************************
#define PLATFORM_VERSION                        5
#define CC3000_APP_BUFFER_SIZE                  5
#define CC3000_RX_BUFFER_OVERHEAD_SIZE          20
#define SL_VERSION_LENGTH                       11
#define NETAPP_IPCONFIG_MAC_OFFSET              20

#define NUM_CHANNELS                            13

#define SENTINEL_EMPTY 0xFFFFFFFF

//*****************************************************************************
//
// Return codes from wlan_ioctl_statusget() which appear to be missing from
// the CC3000 SDK.
//
//*****************************************************************************
#define WLAN_STATUS_DISCONNECTED                0
#define WLAN_STATUS_SCANNING                    1
#define WLAN_STATUS_CONNECTING                  2
#define WLAN_STATUS_CONNECTED                   3

//*****************************************************************************
//
// Status values found in the tScanResult ui32Status field.
//
//*****************************************************************************
#define SCAN_AGED_RESULT                            0
#define SCAN_RESULT_VALID                           1
#define SCAN_NO_RESULT                              2

//*****************************************************************************
//
// Masks related to values found in the tScanResult ui8ValidRSSI field.
//
//*****************************************************************************
#define SCAN_IS_VALID                               0x01
#define SCAN_RSSI_MASK                              0xFE

//*****************************************************************************
//
// Masks and labels related to values found in the tScanResult
// ui8SecuritySSIDLen field.
//
//*****************************************************************************
#define SCAN_SEC_MASK                               0x03
#define SCAN_SEC_SHIFT                                 0
#define SCAN_SEC_OPEN                               0x00
#define SCAN_SEC_WEP                                0x40
#define SCAN_SEC_WPA                                0x80
#define SCAN_SEC_WPA2                               0xC0
#define SCAN_SEC_INDEX(x) (((x) & SCAN_SEC_MASK) >> SCAN_SEC_SHIFT)

//*****************************************************************************
//
// Mask, shift and macro to extract the SSID length from the ui8SecuritySSIDLen
// field.
//
//*****************************************************************************
#define SCAN_SSID_LEN_MASK                          0xFC
#define SCAN_SSID_LEN_SHIFT                            2
#define SCAN_SSID_LEN(x) (((x) & SCAN_SSID_LEN_MASK) >> SCAN_SSID_LEN_SHIFT)

//*****************************************************************************
//
// The structure returned by a call to wlan_ioctl_get_scan_results.
//
//*****************************************************************************
typedef struct
{
    uint32_t ui32NumNetworks;
    uint32_t ui32Status;
    uint8_t  ui8ValidRSSI;
    uint8_t  ui8SecuritySSIDLen;
    uint16_t ui16Time;
    char     pcSSID[32];
    char     pcBSSID[6];
}
tScanResult;

extern char *g_ppcSecurity[];

class WiFiComm: public AbstractComm {
public:
	WiFiComm();
	virtual ~WiFiComm();

	unsigned int read(char * buffer, unsigned int max_length);
	void write(const char * buffer, unsigned int length);

	unsigned int type(){return PP_COMM_TYPE_CC3000;};

	Network * network(){return &network_;};
	Server * server(){return &server_;};

private:
	Network network_;
	Server server_;

	uint32_t socket_;

	uint32_t initCC3000();
	uint32_t openSocket();
	uint8_t closeSocket();
	uint8_t resetCC3000();
};

extern "C"
{
	char* sendDriverPatch(unsigned long *length);
	char* sendBootLoaderPatch(unsigned long *length);
	char* sendWLFWPatch(unsigned long *length);
	void CC3000AsyncCallback(long lEventType, char *pcData, unsigned char ucLength);
}

#endif /* WIFICOMM_H_ */

// EOF
