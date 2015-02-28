/*
 * wifiComm.h
 *
 *  Created on: Jun 3, 2014
 *      Author: cabrita
 */

#ifndef WIFICOMM_H_
#define WIFICOMM_H_

#include "wlan.h"
#include "evnt_handler.h"
#include "nvmem.h"
#include "cc3000_common.h"
#include "netapp.h"
#include "spi.h"
#include "hci.h"
#include "spi_version.h"
#include "board.h"
#include "host_driver_version.h"
#include "security.h"

extern char g_pui8Data[512];
extern char g_pui8SensorData[256];

//*****************************************************************************
//
//  Global variables used by this program for event tracking.
//
//*****************************************************************************
extern volatile uint32_t g_ui32SmartConfigFinished, g_ui32CC3000Connected,
                  g_ui32CC3000DHCP,g_ui32OkToDoShutDown,
                  g_ui32CC3000DHCP_configured;

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

extern char *g_ppcSecurity[];

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
// Prototypes.
//
//*****************************************************************************

extern char *sendDriverPatch(unsigned long *Length);

extern char *sendBootLoaderPatch(unsigned long *Length);

extern char *sendWLFWPatch(unsigned long *Length);

extern void CC3000AsyncCallback(long lEventType, unsigned char *pcData, unsigned char ucLength);

extern uint32_t CC3000OpenSocket();

extern uint8_t CC3000UpdateSensorData(uint32_t ui32Socket);

extern uint8_t CC3000CloseSocket(uint32_t ui32Socket);

extern uint8_t CC3000Reset();

#endif /* WIFICOMM_H_ */
