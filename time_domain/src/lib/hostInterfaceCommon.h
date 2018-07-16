///////////////////////////////////////
//
// hostInterface.h
//
// Definitions for the interface between a host computer and the embedded RCM.
//
// Copyright (c) 2010 Time Domain
//

#ifndef __rcrmHostInterfaceCommon_h
#define __rcrmHostInterfaceCommon_h

// Portability

// The section "#ifdef _MSC_VER" was copied from http://msinttypes.googlecode.com/svn/trunk/stdint.h,
// an implementation of stdint.h for Microsoft Visual C/C++ versions earlier than Visual Studio 2010.
#ifdef _MSC_VER

// Visual Studio 6 and Embedded Visual C++ 4 doesn't
// realize that, e.g. char has the same size as __int8
// so we give up on __intX for them.
#if (_MSC_VER < 1300)
typedef signed char       rcrm_int8_t;
typedef signed short      rcrm_int16_t;
typedef signed int        rcrm_int32_t;
typedef unsigned char     rcrm_uint8_t;
typedef unsigned short    rcrm_uint16_t;
typedef unsigned int      rcrm_uint32_t;
#else
typedef signed __int8     rcrm_int8_t;
typedef signed __int16    rcrm_int16_t;
typedef signed __int32    rcrm_int32_t;
typedef unsigned __int8   rcrm_uint8_t;
typedef unsigned __int16  rcrm_uint16_t;
typedef unsigned __int32  rcrm_uint32_t;
#endif

typedef signed __int64       rcrm_int64_t;
typedef unsigned __int64     rcrm_uint64_t;

#else

typedef	__signed char			rcrm_int8_t;
typedef	unsigned char			rcrm_uint8_t;
typedef	short					rcrm_int16_t;
typedef	unsigned short			rcrm_uint16_t;
typedef	int						rcrm_int32_t;
typedef	unsigned int			rcrm_uint32_t;
typedef	long long				rcrm_int64_t;
typedef	unsigned long long		rcrm_uint64_t;

#endif


// Socket defines
#define RCRM_SOCKET_PORT_NUM  21210


// Internal modes of operation - not all are supported
#define RCRM_OPMODE_RCM		0
#define RCRM_OPMODE_RN		4
#define RCRM_OPMODE_DEFAULT	RCRM_OPMODE_RCM


// P400 sleep modes
#define RCRM_SLEEP_MODE_ACTIVE          0
#define RCRM_SLEEP_MODE_IDLE            1
#define RCRM_SLEEP_MODE_STANDBY_ETH     2 // wakeup via ethernet or serial
#define RCRM_SLEEP_MODE_STANDBY_SER     3 // wakeup via serial only
#define RCRM_SLEEP_MODE_SLEEP           4 // wakeup via GPIO only

///////////////////////////////////////
//
// Message types
//

// REQUEST messages are sent by the host to the embedded applicaion.
// CONFIRM messages are sent by the embedded application to the host in response to REQUEST messages.
// INFO messages are sent automatically by the embedded application to the host when various events occur.
#define RCRM_MSG_TYPE_REQUEST			(0xF000)
#define RCRM_MSG_TYPE_CONFIRM			(0xF100)
#define RCRM_MSG_TYPE_INFO				(0xF200)


///////////////////////////////////////
//
// Host <-> Embedded conversation messages
//

// Get version and temperature info
#define RCRM_GET_STATUS_INFO_REQUEST	(RCRM_MSG_TYPE_REQUEST + 1)
#define RCRM_GET_STATUS_INFO_CONFIRM	(RCRM_MSG_TYPE_CONFIRM + 1)


// Reboot P400
#define RCRM_REBOOT_REQUEST				(RCRM_MSG_TYPE_REQUEST + 2)
#define RCRM_REBOOT_CONFIRM				(RCRM_MSG_TYPE_CONFIRM + 2)

// Set opmode
#define RCRM_SET_OPMODE_REQUEST			(RCRM_MSG_TYPE_REQUEST + 3)
#define RCRM_SET_OPMODE_CONFIRM			(RCRM_MSG_TYPE_CONFIRM + 3)

// Get opmode
#define RCRM_GET_OPMODE_REQUEST			(RCRM_MSG_TYPE_REQUEST + 4)
#define RCRM_GET_OPMODE_CONFIRM			(RCRM_MSG_TYPE_CONFIRM + 4)

// Set sleep mode
#define RCRM_SET_SLEEP_MODE_REQUEST		(RCRM_MSG_TYPE_REQUEST + 5)
#define RCRM_SET_SLEEP_MODE_CONFIRM		(RCRM_MSG_TYPE_CONFIRM + 5)

// Get sleep mode
#define RCRM_GET_SLEEP_MODE_REQUEST		(RCRM_MSG_TYPE_REQUEST + 6)
#define RCRM_GET_SLEEP_MODE_CONFIRM		(RCRM_MSG_TYPE_CONFIRM + 6)

// Get ambient samples
#define RCRM_GET_AMBIENT_SAMPLES_REQUEST	(RCRM_MSG_TYPE_REQUEST + 7)
#define RCRM_GET_AMBIENT_SAMPLES_CONFIRM	(RCRM_MSG_TYPE_CONFIRM + 7)

// Execute Built-In Test
#define RCRM_BIT_REQUEST	            (RCRM_MSG_TYPE_REQUEST + 8)
#define RCRM_BIT_CONFIRM	            (RCRM_MSG_TYPE_CONFIRM + 8)

// Get cause of last boot
#define RCRM_GET_LAST_BOOT_CAUSE_REQUEST	(RCRM_MSG_TYPE_REQUEST + 9)
#define RCRM_GET_LAST_BOOT_CAUSE_CONFIRM	(RCRM_MSG_TYPE_CONFIRM + 9)

///////////////////////////////////////
//
// Common INFO messages to the host
//

// Sent when a waveform scan is completed.
// If the complete scan doesn't fit in a single packet, multiple packets are sent which can be combined to form the complete scan.
#define RCRM_FULL_SCAN_INFO				(RCRM_MSG_TYPE_INFO + 1)

// Sent when the radio is ready to receive commands.
// Sent after a power up, reboot, or wakeup from sleep event is complete.
#define RCRM_READY_INFO				    (RCRM_MSG_TYPE_INFO + 2)



///////////////////////////////////////
//
// Constants and flags
//


// *_CONFIRM message status codes
#define RCRM_CONFIRM_MSG_STATUS_SUCCESS				0
#define RCRM_CONFIRM_MSG_STATUS_GENERICFAILURE		1
#define RCRM_CONFIRM_MSG_STATUS_WRONGOPMODE			2
#define RCRM_CONFIRM_MSG_STATUS_UNSUPPORTEDVALUE	3
#define RCRM_CONFIRM_MSG_STATUS_INVALIDDURINGSLEEP	4

// Definitions of the antennaMode field in various messages

// Send RCM_ANTENNAMODE_DEFAULT in a message requiring antennaMode to use the default configured antenna
#define RCRM_ANTENNAMODE_DEFAULT		0xff

#define RCRM_ANTENNAMODE_TXA_RXA		0
#define RCRM_ANTENNAMODE_TXB_RXB		1
#define RCRM_ANTENNAMODE_TXA_RXB		2
#define RCRM_ANTENNAMODE_TXB_RXA		3

#define RCRM_ANTENNAMODE_TOGGLE_FLAG	(0x80)


#define RCRM_MAX_SCAN_SAMPLES			(350)

// TX gain levels (inverses of the actual attenuation levels)
#define RCRM_TXGAIN_MIN		0
#define RCRM_TXGAIN_MAX		63


// Scan filtering
#define RCRM_SCAN_FILTERING_RAW				(1 << 0)
#define RCRM_SCAN_FILTERING_FASTTIME		(1 << 1)
#define RCRM_SCAN_FILTERING_MOTION			(1 << 2)

// Setting/saving configuration Persist flags.

// Update the active configuration with this new configuration info.
// The active configuration is not written to flash.
#define RCRM_PERSIST_NONE				(0)

// Update the active configuration with this new configuration info.
// Write entire active configuration to flash so that it persists across reboots,
// including any not-written-to-flash changes that may have occurred (i.e. earlier set-config messages with RCRM_PERSIST_NONE).
#define RCRM_PERSIST_ALL				(1)

// Update the active configuration with this new configuration info.
// Write configuration to flash with the new changes.
// Earlier set config messsages with RCRM_PERSIST_NONE will NOT be written to flash when this flag is set.
#define RCRM_PERSIST_THIS_MSG			(2)

// (Experimental/Debug at this point)
// Update the active configuration with this new configuration info.
// Do not write the configuration to flash at this point,
// but any later set-config message with RCRM_PERSIST_ALL or RCRM_PERSIST_THIS_MSG will write these changes to flash.
// Intended when a series of set-config messages will be sent, but only one actual flash write is needed.
#define RCRM_PERSIST_THIS_MSG_NO_WRITE	(3)

///////////////////////////////////////
//
// Message struct definitions
//



typedef struct
{
	// set to RCRM_GET_STATUS_INFO_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rcrmMsg_GetStatusInfoRequest;

typedef struct
{
	// set to RCRM_GET_STATUS_INFO_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rcrm_uint8_t appVersionMajor;
	rcrm_uint8_t appVersionMinor;
	rcrm_uint16_t appVersionBuild;

	rcrm_uint8_t uwbKernelVersionMajor;
	rcrm_uint8_t uwbKernelVersionMinor;
	rcrm_uint16_t uwbKernelVersionBuild;

	rcrm_uint8_t firmwareVersion;
	rcrm_uint8_t firmwareYear;
	rcrm_uint8_t firmwareMonth;
	rcrm_uint8_t firmwareDay;

	rcrm_uint32_t serialNum;
	
	rcrm_uint8_t boardRev;
	rcrm_uint8_t bitResults;
	rcrm_uint8_t model;
	rcrm_uint8_t pulserConfig;

	// Divide this by 4 to get temperature in degrees C.
	rcrm_int32_t temperature;

	char packageVersionStr[32];

	// status code
	rcrm_uint32_t status;
} rcrmMsg_GetStatusInfoConfirm;


typedef struct
{
	// set to RCRM_REBOOT_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rcrmMsg_RebootRequest;

typedef struct
{
	// set to RCRM_REBOOT_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rcrmMsg_RebootConfirm;

typedef struct
{
	// set to RCRM_SET_OPMODE_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	// Requested operational mode of the P400.
	rcrm_uint32_t opMode;
} rcrmMsg_SetOpmodeRequest;

typedef struct
{
	// set to RCRM_SET_OPMODE_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	// Opmode of the radio
	rcrm_uint32_t opMode;

	rcrm_uint32_t status;
} rcrmMsg_SetOpmodeConfirm;

typedef struct
{
	// set to RCRM_GET_OPMODE_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rcrmMsg_GetOpmodeRequest;

typedef struct
{
	// set to RCRM_GET_OPMODE_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	// Current operational mode of the P400.
	rcrm_uint32_t opMode;
} rcrmMsg_GetOpmodeConfirm;

typedef struct
{
	// set to RCRM_SET_SLEEP_MODE_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	// Requested sleep mode of the P400.
	rcrm_uint32_t sleepMode;
} rcrmMsg_SetSleepModeRequest;

typedef struct
{
	// set to RCRM_SET_SLEEP_MODE_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	rcrm_uint32_t status;
} rcrmMsg_SetSleepModeConfirm;

typedef struct
{
	// set to RCRM_GET_SLEEP_MODE_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rcrmMsg_GetSleepModeRequest;

typedef struct
{
	// set to RCRM_GET_SLEEP_MODE_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	// Current sleep mode of the P400.
	rcrm_uint32_t sleepMode;
} rcrmMsg_GetSleepModeConfirm;

typedef struct
{
	// set to RCRM_GET_AMBIENT_SAMPLES_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

    // PII of samples - generally, the currently selected PII should be used
    // The radio can collect true "samples" (as opposed to integrated ramp
    // values) by setting this to 0.
	rcrm_uint16_t integrationIndex;

} rcrmMsg_GetAmbientSamplesRequest;

typedef struct
{
	// set to RCRM_GET_AMBIENT_SAMPLES_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
    // the ambient samples
	rcrm_int32_t samples[RCRM_MAX_SCAN_SAMPLES];
	
	rcrm_uint32_t status;
} rcrmMsg_GetAmbientSamplesConfirm;

typedef struct
{
	// set to RCRM_BIT_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rcrmMsg_BitRequest;

typedef struct
{
	// set to RCRM_BIT_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
    // BIT status - 0 is OK, anything else is an error
	rcrm_uint32_t status;
} rcrmMsg_BitConfirm;

typedef struct
{
	// set to RCRM_GET_LAST_BOOT_CAUSE_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rcrmMsg_GetLastBootCauseRequest;

typedef struct
{
	// set to RCRM_GET_LAST_BOOT_CAUSE_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	// Cause of last boot
	// 0 - Normal powerup
	// 1 - Watchdog reset
	rcrm_uint32_t lastBootCause;
} rcrmMsg_GetLastBootCauseConfirm;


typedef struct
{
	// set to RCRM_READY_INFO
	rcrm_uint16_t msgType;
	// identifier to correlate requests with info messages
	rcrm_uint16_t msgId;
} rcrmMsg_ReadyInfo;

typedef struct
{
	// set to RCRM_FULL_SCAN_INFO
	rcrm_uint16_t msgType;
	// identifier to correlate range requests with info messages
	rcrm_uint16_t msgId;

	// ID of the transmitting radio
	rcrm_uint32_t sourceId;

	// Milliseconds since radio boot at the time the scan was completed
	rcrm_uint32_t timestamp;

	// noise
	rcrm_uint16_t noise;
	// Vpeak
	rcrm_uint16_t vPeak;
	
	rcrm_uint32_t reserved1;	// inserted with RCM 2.0 to avoid breaking MRM 1.1

	// These are indices within the assembled scan.
	rcrm_int32_t ledIndex;
	rcrm_int32_t lockspotOffset;
	
	rcrm_int32_t scanStartPs;
	rcrm_int32_t scanStopPs;
	
	rcrm_uint16_t scanStepBins;
	
	// Raw, fast time, motion, etc.
	rcrm_uint8_t scanFiltering;
	
	rcrm_uint8_t reserved2;	// alignment

	// Antenna the scan was received on
	rcrm_uint8_t antennaId;
	
	// The type of operation behind this scan (ranging, BSR, MSR)
	rcrm_uint8_t operationMode;
	
	// Number of scan samples in this message
	rcrm_uint16_t numSamplesInMessage;

	// Number of samples in the entire scan
	rcrm_uint32_t numSamplesTotal;

	// Index of the message in the scan
	rcrm_uint16_t messageIndex;

	// Total number of RCRM_FULL_SCAN_INFO messages to expect for this particular scan.
	rcrm_uint16_t numMessagesTotal;

	// Scan samples.
	// Note that, unlike RCRM_SCAN_INFO, this is NOT a variable-sized packet.
	rcrm_int32_t scan[RCRM_MAX_SCAN_SAMPLES];
} rcrmMsg_FullScanInfo;

#endif	// #ifdef __rcrmHostInterfaceCommon_h
