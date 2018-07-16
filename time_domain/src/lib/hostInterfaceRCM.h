#ifndef __rcrmHostInterfaceRCM_h
#define __rcrmHostInterfaceRCM_h

#include "hostInterfaceCommon.h"

// Maximum size of user data
#define RCM_USER_DATA_LENGTH	(1000)

///////////////////////////////////////
//
// Message types
//

// REQUEST messages are sent by the host to the embedded applicaion.
// CONFIRM messages are sent by the embedded application to the host in response to REQUEST messages.
// INFO messages are sent automatically by the embedded application to the host when various events occur.
#define RCM_MSG_TYPE_REQUEST			(0x0000)
#define RCM_MSG_TYPE_CONFIRM			(0x0100)
#define RCM_MSG_TYPE_INFO				(0x0200)


///////////////////////////////////////
//
// Host <-> Embedded conversation messages
//


// Set configuration
#define RCM_SET_CONFIG_REQUEST			(RCM_MSG_TYPE_REQUEST + 1)
#define RCM_SET_CONFIG_CONFIRM			(RCM_MSG_TYPE_CONFIRM + 1)


// Get configuration
#define RCM_GET_CONFIG_REQUEST			(RCM_MSG_TYPE_REQUEST + 2)
#define RCM_GET_CONFIG_CONFIRM			(RCM_MSG_TYPE_CONFIRM + 2)


// Initiate range request
#define RCM_SEND_RANGE_REQUEST			(RCM_MSG_TYPE_REQUEST + 3)
#define RCM_SEND_RANGE_REQUEST_CONFIRM	(RCM_MSG_TYPE_CONFIRM + 3)


// Send data to another radio
#define RCM_SEND_DATA_REQUEST			(RCM_MSG_TYPE_REQUEST + 4)
#define RCM_SEND_DATA_CONFIRM			(RCM_MSG_TYPE_CONFIRM + 4)


// Set the data the responder will use when responding to a range request.
#define RCM_SET_RESPONSE_DATA_REQUEST	(RCM_MSG_TYPE_REQUEST + 5)
#define RCM_SET_RESPONSE_DATA_CONFIRM	(RCM_MSG_TYPE_CONFIRM + 5)

// Send range request with a code channel selection
#define RCM_SEND_CHANNELIZED_RANGE_REQUEST			(RCM_MSG_TYPE_REQUEST + 6)
#define RCM_SEND_CHANNELIZED_RANGE_REQUEST_CONFIRM	(RCM_MSG_TYPE_CONFIRM + 6)

///////////////////////////////////////
//
// Message struct definitions
//

typedef struct
{
	// ID of the radio in the network
	rcrm_uint32_t nodeId;

	rcrm_uint16_t integrationIndex;
	rcrm_uint8_t antennaMode;
	rcrm_uint8_t codeChannel;

	// Two separate electrical delays in picoseconds - one for each antenna
	rcrm_int32_t electricalDelayPsA;
	rcrm_int32_t electricalDelayPsB;

	rcrm_uint16_t flags;

	// 0 = lowest power, 63 = highest power
	rcrm_uint8_t txGain;

	// set to non-zero to indicate settings should persist across radio reboots
	rcrm_uint8_t persistFlag;
} rcmConfiguration;

///////////////////////////////////////
//
// RCM INFO messages to the host
//

// Results of an attempted range conversation
#define RCM_FULL_RANGE_INFO				(RCM_MSG_TYPE_INFO + 1)

// Sent when data was received over the air
#define RCM_DATA_INFO					(RCM_MSG_TYPE_INFO + 2)

// Sent when a waveform scan is completed, if the RCM_SEND_SCANINFO_PKTS flag is set.
// If the complete scan doesn't fit in a single packet, only the portion of the scan centered around the leading edge is included.
#define RCM_SCAN_INFO					(RCM_MSG_TYPE_INFO + 3)

// Overheard echoed precision range measurement.
// Only sent if the RCM_FLAG_ENABLE_ECHO_LAST_RANGE flag is set in the RCM configuration.
#define RCM_ECHOED_RANGE_INFO			(RCM_MSG_TYPE_INFO + 4)

// Smaller version of RANGE_INFO for bandwidth-limited situations.
#define RCM_SMALL_RANGE_INFO			(RCM_MSG_TYPE_INFO + 5)



// Definitions of the rcmConfiguration::flags bitfield

// RCM_SEND_SCANINFO_PKTS - when set, SCAN_INFO packets are sent whenever a scan is received.
// Default is disabled/not set.
#define RCM_SEND_SCANINFO_PKTS				(1 << 0)

// RCM_SEND_FULL_SCANINFO_PKTS - when set, FULL_SCAN_INFO packets are sent whenever a scan is received.
// Default is disabled/not set.
#define RCM_SEND_FULL_SCANINFO_PKTS			(1 << 1)

// RCM_DISABLE_FAN - (for Hammerhead Rev B) when set, turns OFF the fan.
// Default is not set/fan enabled.
#define RCM_DISABLE_FAN						(1 << 2)

// RCM_SEND_SCAN_WITH_DATA - when set, data packets cause RX to scan
// Default is not set - data packets do not cause scans.
#define RCM_SEND_SCAN_WITH_DATA				(1 << 3)

// RCM_DISABLE_CRE_RANGES - when set, non-precision (CRE + Filtered) range messages are NOT sent to the host.
// Default is not set/send CRE messages.
#define RCM_DISABLE_CRE_RANGES				(1 << 4)

// RCM_USE_SINGLE_SIDED_CCI_CHECK - when set, statistics are collected for each UWB node RCM hears and analyzed to flag cochannel interference.
// Default is not set/don't use single-sided cochannel interference check.
#define RCM_USE_SINGLE_SIDED_CCI_CHECK		(1 << 5)

// RCM_FLAG_LOS_NLOS_MISMATCH_AS_CCI - when set, a LOS/NLOS mismatch in a range conversation gets flagged as cochannel interference.
// Default is not set/use ranges even when they are LOS/NLOS mismatched.
#define RCM_FLAG_LOS_NLOS_MISMATCH_AS_CCI	(1 << 6)

// RCM_FLAG_ENABLE_ECHO_LAST_RANGE - when set, RCM will echo its last successful precision range as part of the user data in its range request flag.
// This flag also enables sending to the host all overheard echoed ranges.
#define RCM_FLAG_ENABLE_ECHO_LAST_RANGE		(1 << 7)

// RCM_FLAG_SEND_SMALL_RANGE_INFOS - when set, RCM will send RCM_SMALL_RANGE_INFOs instead of RCM_RANGE_INFOs.
#define RCM_FLAG_SEND_SMALL_RANGE_INFOS		(1 << 8)

// Range status bitmask/codes

// No bits set indicates no failures, i.e. success
#define RCM_RANGE_STATUS_SUCCESS			0

// Timeout is set by itself (since you can't have/know about any other failures in this case)
#define RCM_RANGE_STATUS_TIMEOUT			1

// Virtual Carrier Sense is also sent by itself since it's exclusive to any other failures.
#define RCM_RANGE_STATUS_VCS				3

// Weak Signal flags are sent by themselves when there is a signal strength mismatch (> 10 dB delta between computed SNRs),
// but the range doesn't have any other error flags raised

// The requester sees a weak signal from the responder (possibly responder is transmitting a weak signal)
#define RCM_RANGE_STATUS_REQ_RX_WEAK_SIGNAL		5

// The responder sees a weak signal from the requester (possibly requester is transmitting a weak signal)
#define RCM_RANGE_STATUS_RESP_RX_WEAK_SIGNAL	7

// Possible co-channel interference was detected
#define RCM_RANGE_STATUS_COCHANNEL_INTERFERENCE	9

// Line-of-sight/Non-line-of-sight mismatch on the two legs of the range conversation
#define RCM_RANGE_STATUS_LOS_NLOS_MISMATCH		11

// A ranging conversation failure (besides timeout) may be any combination of these.
#define RCM_RANGE_STATUS_REQUESTERLEDFAIL	(1 << 1)
#define RCM_RANGE_STATUS_RESPONDERLEDFAIL	(1 << 2)
#define RCM_RANGE_STATUS_RANGETOOLONG	    (1 << 5)
#define RCM_RANGE_STATUS_COARSERANGEUPDATE	(1 << 6)

#define RCM_RANGE_TYPE_PRECISION			(1 << 0)
#define RCM_RANGE_TYPE_COARSE				(1 << 1)
#define RCM_RANGE_TYPE_FILTERED				(1 << 2)

// Definitions of the antennaId field in various RCM_***_INFO messages
#define RCM_ANTENNAID_A		RCRM_ANTENNAMODE_TXA_RXA
#define RCM_ANTENNAID_B		RCRM_ANTENNAMODE_TXB_RXB

///////////////////////////////////////
//
// Host <-> RCM conversation messages
//

typedef struct
{
	// set to RCM_SET_CONFIG_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rcmConfiguration config;
} rcmMsg_SetConfigRequest;

typedef struct
{
	// set to RCM_SET_CONFIG_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// status code
	rcrm_uint32_t status;
} rcmMsg_SetConfigConfirm;


typedef struct
{
	// set to RCM_GET_CONFIG_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rcmMsg_GetConfigRequest;

typedef struct
{
	// set to RCM_GET_CONFIG_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rcmConfiguration config;

	// milliseconds since radio boot
	rcrm_uint32_t timestamp;

	// status code
	rcrm_uint32_t status;
} rcmMsg_GetConfigConfirm;


typedef struct
{
	// set to RCM_SEND_RANGE_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// ID of the responder radio to send this range request to
	rcrm_uint32_t responderId;

	// which antenna(s) to use
	rcrm_uint8_t antennaMode;

	rcrm_uint8_t reserved;	// alignment

	// number of bytes of data to send (up to 1000)
	rcrm_uint16_t dataSize;

	// Data for range request payload.
	// Only the first dataSize bytes need to actually be in the UDP packet.
	rcrm_uint8_t data[RCM_USER_DATA_LENGTH];
} rcmMsg_SendRangeRequest;

typedef struct
{
	// set to RCM_SEND_RANGE_REQUEST_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// status code
	rcrm_uint32_t status;
} rcmMsg_SendRangeRequestConfirm;


typedef struct
{
	// set to RCM_SEND_DATA_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// which antenna(s) to use
	rcrm_uint8_t antennaMode;

	rcrm_uint8_t reserved;	// alignment

	// number of bytes of data to send (up to 1000)
	rcrm_uint16_t dataSize;

	// Data for payload.
	// Only the first dataSize bytes need to actually be in the UDP packet.
	rcrm_uint8_t data[RCM_USER_DATA_LENGTH];
} rcmMsg_SendDataRequest;

typedef struct
{
	// set to RCM_SEND_DATA_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// status code
	rcrm_uint32_t status;
} rcmMsg_SendDataConfirm;


typedef struct
{
	// set to RCM_SET_RESPONSE_DATA_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rcrm_uint16_t reserved;	// alignment

	// number of bytes of data to send to a requester (up to 1000)
	rcrm_uint16_t dataSize;

	// Data for range response payload.
	// Only the first dataSize bytes need to actually be in the UDP packet.
	rcrm_uint8_t data[RCM_USER_DATA_LENGTH];
} rcmMsg_SetResponseDataRequest;

typedef struct
{
	// set to RCM_SET_RESPONSE_DATA_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// status code
	rcrm_uint32_t status;
} rcmMsg_SetResponseDataConfirm;

typedef struct
{
	// set to RCM_SEND_CHANNELIZED_RANGE_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// ID of the responder radio to send this range request to
	rcrm_uint32_t responderId;

	// which antenna(s) to use
	rcrm_uint8_t antennaMode;

	// which code channel to switch to for this range request/response
	rcrm_uint8_t codeChannel;

	// number of bytes of data to send (up to 1000)
	rcrm_uint16_t dataSize;

	// Data for range request payload.
	// Only the first dataSize bytes need to actually be in the UDP packet.
	rcrm_uint8_t data[RCM_USER_DATA_LENGTH];
} rcmMsg_SendChannelizedRangeRequest;

typedef struct
{
	// set to RCM_SEND_CHANNELIZED_RANGE_REQUEST_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// status code
	rcrm_uint32_t status;
} rcmMsg_SendChannelizedRangeRequestConfirm;

///////////////////////////////////////
//
// INFO messages to the host
//

typedef struct
{
	// set to RCM_FULL_RANGE_INFO
	rcrm_uint16_t msgType;
	// identifier to correlate range requests with info messages
	rcrm_uint16_t msgId;

	// ID of the responding radio
	rcrm_uint32_t responderId;

	// status/error codes for the range conversation
	rcrm_uint8_t rangeStatus;
	// Antenna(s) on this radio used in the range conversation
	rcrm_uint8_t antennaMode;
	// how long the range conversation took, in milliseconds
	rcrm_uint16_t stopwatchTime;

	rcrm_uint32_t precisionRangeMm;
	rcrm_uint32_t coarseRangeMm;
	rcrm_uint32_t filteredRangeMm;
	
	rcrm_uint16_t precisionRangeErrEst;
	rcrm_uint16_t coarseRangeErrEst;
	rcrm_uint16_t filteredRangeErrEst;

	rcrm_int16_t filteredRangeVel;
	rcrm_uint16_t filteredRangeVelErrEst;

	rcrm_uint8_t rangeMeasurementType;
	rcrm_uint8_t reserved;
	
	rcrm_uint16_t reqLEDFlags;
	rcrm_uint16_t respLEDFlags;

	rcrm_uint16_t noise;
	rcrm_uint16_t vPeak;
	
	rcrm_int32_t coarseTOFInBins;

	// milliseconds since radio boot at the time of the range conversation
	rcrm_uint32_t timestamp;
} rcmMsg_FullRangeInfo;



typedef struct
{
	// set to RCM_DATA_INFO
	rcrm_uint16_t msgType;
	// identifier to correlate range requests with info messages
	rcrm_uint16_t msgId;

	// ID of the sending radio
	rcrm_uint32_t sourceId;

	// noise
	rcrm_uint16_t noise;
	// Peak voltage, normalized.
	rcrm_uint16_t vPeak;
	// milliseconds since radio boot at the time the data packet was received
	rcrm_uint32_t timestamp;

	// Antenna the UWB packet was received on
	rcrm_uint8_t antennaId;

	rcrm_uint8_t reserved;	// alignment

	// number of bytes of data received, up to 1000
	rcrm_uint16_t dataSize;

	// Data from packet payload
	// Only the first dataSize bytes are actually in the UDP packet.
	rcrm_uint8_t data[RCM_USER_DATA_LENGTH];
} rcmMsg_DataInfo;

#define RCM_MAX_SCAN_SAMPLES	350

typedef struct
{
	// set to RCM_SCAN_INFO
	rcrm_uint16_t msgType;
	// identifier to correlate range requests with info messages
	rcrm_uint16_t msgId;

	// ID of the transmitting radio
	rcrm_uint32_t sourceId;

	// Antenna the UWB packet was received on
	rcrm_uint8_t antennaId;

	rcrm_uint8_t reserved;	// alignment
	
	rcrm_uint16_t LEDflags;

	// channel quality (LED rise time; lower (faster rise) is better)
	rcrm_uint16_t noise;
	// Peak voltage, normalized.
	rcrm_uint16_t vPeak;

	// milliseconds since radio boot at the time the data packet was received
	rcrm_uint32_t timestamp;

	rcrm_int32_t ledIndex;
	rcrm_int32_t lockspotOffset;

	// Number of samples provided in the scan
	rcrm_uint32_t numSamples;
	// Scan samples. Note that only the first numSamples samples are sent in the UDP packet.
	rcrm_int32_t samples[RCM_MAX_SCAN_SAMPLES];
} rcrmMsg_ScanInfo;


typedef struct
{
	// set to RCM_ECHOED_RANGE_INFO
	rcrm_uint16_t msgType;
	// identifier to correlate range requests with info messages
	rcrm_uint16_t msgId;

	// ID of the requesting radio
	rcrm_uint32_t requesterId;

	// ID of the responding radio
	rcrm_uint32_t responderId;

	// Reported range in millimeters
	rcrm_uint32_t precisionRangeMm;

	// Reported estimated range error in millimeters
	rcrm_uint16_t precisionRangeErrEst;

	// Requester and responder LED flags, or'd together.
	rcrm_uint16_t ledFlags;

	// Milliseconds since the requesting radio boot at the time of the range conversation.
	// Used to identify duplicate ELR reports.
	rcrm_uint32_t timestamp;
} rcmMsg_EchoedRangeInfo;


#pragma pack(1)

typedef struct
{
	// set to RCM_SMALL_RANGE_INFO
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	rcrm_uint32_t responderId;
	
	rcrm_uint16_t rangeCm;
	rcrm_uint8_t rangeErrEst;
	
	rcrm_uint8_t rangeType;
	
	rcrm_uint8_t rangeStatus;
	
	rcrm_uint8_t flags;
} rcmMsg_SmallRangeInfo;

#pragma pack()


#endif	// #ifdef __rcrmHostInterfaceRCM_h
