#ifndef __rcrmHostInterfaceRN_h
#define __rcrmHostInterfaceRN_h

#include "hostInterfaceCommon.h"

// Maximum size of user data
#define RN_USER_DATA_LENGTH			(1000)

// Maximum number of excluded range targets
#define RN_MAX_EXCLUDED_TARGETS		(256)

// Maximum number of entries in the full neighbor database message
#define RN_FULL_NEIGHBOR_DATABASE_MAX_ENTRY_COUNT	32

// Maximum number of entries in the small neighbor database message
#define RN_SMALL_NEIGHBOR_DATABASE_MAX_ENTRY_COUNT	80

// Maximum number of slots in the TDMA slotmap
#define RN_MAX_TDMA_SLOTS			(32)

///////////////////////////////////////
//
// Message types
//

// REQUEST messages are sent by the host to the embedded application.
// CONFIRM messages are sent by the embedded application to the host in response to REQUEST messages.
// INFO messages are sent automatically by the embedded application to the host when various events occur.
#define RN_MSG_TYPE_REQUEST			(0x3000)
#define RN_MSG_TYPE_CONFIRM			(0x3100)
#define RN_MSG_TYPE_INFO			(0x3200)


///////////////////////////////////////
//
// Host <-> Embedded conversation messages
//

// Set configuration
#define RN_SET_CONFIG_REQUEST					(RN_MSG_TYPE_REQUEST + 1)
#define RN_SET_CONFIG_CONFIRM					(RN_MSG_TYPE_CONFIRM + 1)

// Get configuration
#define RN_GET_CONFIG_REQUEST					(RN_MSG_TYPE_REQUEST + 2)
#define RN_GET_CONFIG_CONFIRM					(RN_MSG_TYPE_CONFIRM + 2)

// Set user data sent out with every range request
#define RN_SET_REQUEST_USER_DATA_REQUEST		(RN_MSG_TYPE_REQUEST + 3)
#define RN_SET_REQUEST_USER_DATA_CONFIRM		(RN_MSG_TYPE_CONFIRM + 3)

// Set user data sent out with every range response
#define RN_SET_RESPONSE_USER_DATA_REQUEST		(RN_MSG_TYPE_REQUEST + 4)
#define RN_SET_RESPONSE_USER_DATA_CONFIRM		(RN_MSG_TYPE_CONFIRM + 4)

// Get neighbor database
#define RN_GET_FULL_NEIGHBOR_DATABASE_REQUEST	(RN_MSG_TYPE_REQUEST + 5)
#define RN_GET_FULL_NEIGHBOR_DATABASE_CONFIRM	(RN_MSG_TYPE_CONFIRM + 5)
#define RN_GET_SMALL_NEIGHBOR_DATABASE_REQUEST	(RN_MSG_TYPE_REQUEST + 6)
#define RN_GET_SMALL_NEIGHBOR_DATABASE_CONFIRM	(RN_MSG_TYPE_CONFIRM + 6)

// Set excluded targets list
#define RN_SET_EXCLUDED_REQUEST					(RN_MSG_TYPE_REQUEST + 7)
#define RN_SET_EXCLUDED_CONFIRM					(RN_MSG_TYPE_CONFIRM + 7)

// Get excluded targets list
#define RN_GET_EXCLUDED_REQUEST					(RN_MSG_TYPE_REQUEST + 8)
#define RN_GET_EXCLUDED_CONFIRM					(RN_MSG_TYPE_CONFIRM + 8)

// Get health monitoring statistics
#define RN_GET_HEALTH_STATUS_REQUEST			(RN_MSG_TYPE_REQUEST + 9)
#define RN_GET_HEALTH_STATUS_CONFIRM			(RN_MSG_TYPE_CONFIRM + 9)

// Reset neighbor database, health statistics, etc. for all nodes or an individual node
#define RN_RESET_DATABASE_AND_STATS_REQUEST		(RN_MSG_TYPE_REQUEST + 10)
#define RN_RESET_DATABASE_AND_STATS_CONFIRM		(RN_MSG_TYPE_CONFIRM + 10)

// Get a copy of the user data sent out with every range request
#define RN_GET_REQUEST_USER_DATA_REQUEST		(RN_MSG_TYPE_REQUEST + 11)
#define RN_GET_REQUEST_USER_DATA_CONFIRM		(RN_MSG_TYPE_CONFIRM + 11)

// Get a copy of the user data sent out with every range response
#define RN_GET_RESPONSE_USER_DATA_REQUEST		(RN_MSG_TYPE_REQUEST + 12)
#define RN_GET_RESPONSE_USER_DATA_CONFIRM		(RN_MSG_TYPE_CONFIRM + 12)

// Set ALOHA configuration
#define RN_SET_ALOHA_CONFIG_REQUEST				(RN_MSG_TYPE_REQUEST + 13)
#define RN_SET_ALOHA_CONFIG_CONFIRM				(RN_MSG_TYPE_CONFIRM + 13)

// Get ALOHA configuration
#define RN_GET_ALOHA_CONFIG_REQUEST				(RN_MSG_TYPE_REQUEST + 14)
#define RN_GET_ALOHA_CONFIG_CONFIRM				(RN_MSG_TYPE_CONFIRM + 14)

// Get durations of OTA packets and range conversations
#define RN_GET_PACKET_DURATIONS_REQUEST			(RN_MSG_TYPE_REQUEST + 15)
#define RN_GET_PACKET_DURATIONS_CONFIRM			(RN_MSG_TYPE_CONFIRM + 15)

// Set TDMA slotmap or individual slot(s)
#define RN_SET_TDMA_SLOTMAP_REQUEST				(RN_MSG_TYPE_REQUEST + 16)
#define RN_SET_TDMA_SLOTMAP_CONFIRM				(RN_MSG_TYPE_CONFIRM + 16)

// Get entire TDMA slotmap
#define RN_GET_TDMA_SLOTMAP_REQUEST				(RN_MSG_TYPE_REQUEST + 17)
#define RN_GET_TDMA_SLOTMAP_CONFIRM				(RN_MSG_TYPE_CONFIRM + 17)

// Get single TDMA slot info
#define RN_GET_TDMA_SLOT_REQUEST				(RN_MSG_TYPE_REQUEST + 18)
#define RN_GET_TDMA_SLOT_CONFIRM				(RN_MSG_TYPE_CONFIRM + 18)

// Set TDMA configuration
#define RN_SET_TDMA_CONFIG_REQUEST				(RN_MSG_TYPE_REQUEST + 19)
#define RN_SET_TDMA_CONFIG_CONFIRM				(RN_MSG_TYPE_CONFIRM + 19)

// Get TDMA configuration
#define RN_GET_TDMA_CONFIG_REQUEST				(RN_MSG_TYPE_REQUEST + 20)
#define RN_GET_TDMA_CONFIG_CONFIRM				(RN_MSG_TYPE_CONFIRM + 20)


///////////////////////////////////////
//
// Embedded -> Host info messages
//

#define RN_TIMING_INFO							(RN_MSG_TYPE_INFO + 2)
#define RN_FULL_NEIGHBOR_DATABASE_INFO			(RN_MSG_TYPE_INFO + 3)
#define RN_SMALL_NEIGHBOR_DATABASE_INFO			(RN_MSG_TYPE_INFO + 4)

///////////////////////////////////////
//
// Flags, etc.
//

// Network synchronization modes
#define RN_NETWORK_SYNC_MODE_ALOHA				(0)
#define RN_NETWORK_SYNC_MODE_TDMA				(1)

// Neighbor database sorting flags

#define RN_NEIGHBOR_DATABASE_SORT_NODEID		(0)
#define RN_NEIGHBOR_DATABASE_SORT_RANGE			(1)
#define RN_NEIGHBOR_DATABASE_SORT_AGE			(2)

// Auto-send range info flags

#define RN_AUTOSEND_RANGEINFO_FLAGS_MASK		(3 << 0)
// Send no range infos
#define RN_AUTOSEND_RANGEINFO_NONE				(0)
// Send full range infos for successful ranges
#define RN_AUTOSEND_RANGEINFO_SUCCESSFUL		(1 << 0)
// Send full range infos for all ranges
#define RN_AUTOSEND_RANGEINFO_ALL				(2 << 0)
// Send small range infos for successful ranges
#define RN_AUTOSEND_RANGEINFO_SMALL				(3 << 0)

// Auto-send neighbor database flags

#define RN_AUTOSEND_NEIGHBOR_DATABASE_FLAGS_MASK	(3 << 2)
// Do not auto-send neighbor database
#define RN_AUTOSEND_NEIGHBOR_DATABASE_NONE			(0 << 2)
// Auto-send full neighbor database
#define RN_AUTOSEND_NEIGHBOR_DATABASE_FULL			(1 << 2)
// Auto-send small neighbor database
#define RN_AUTOSEND_NEIGHBOR_DATABASE_SMALL			(2 << 2)

// Default interface selections

// Wait for host interaction before sending messages; this is the default selection.
#define RN_INTERFACE_NONE			(0)

// Send messages using the Ethernet interface.
// Only supported on radios with Ethernet interfaces.
// address1 is the IPv4 address of the destination.
// address2 is the 16-bit port number.
#define RN_INTERFACE_IP				(1)

// Send messages using the USB interface.
#define RN_INTERFACE_USB			(2)

// Send messages using the serial interface.
#define RN_INTERFACE_SERIAL			(3)

// Send messages using the CAN bus interface.
// Only supported on radios with a CAN interface.
#define RN_INTERFACE_CAN			(4)


// Neighbor database entry flags which indicate special characteristics of the neighbor or neighbor database entry

// Indicates that this neighbor is in beacon mode
#define RN_NEIGHBOR_DATABASE_ENTRY_FLAG_IS_BEACON_NODE		(1 << 0)
// Indicates that this neighbor has set its do-not-range-to-me flag
#define RN_NEIGHBOR_DATABASE_ENTRY_FLAG_DO_NOT_RANGE_TO_ME	(1 << 1)
// Indicates that this neighbor is in the local excluded targets list
#define RN_NEIGHBOR_DATABASE_ENTRY_FLAG_EXCLUDED			(1 << 2)
// Indicates that this neighbor has been heard, but is uncalibrated and thus at the top of the list for next range target selection
#define RN_NEIGHBOR_DATABASE_ENTRY_FLAG_UNCALIBRATED		(1 << 3)

// RangeNet config flags

// Sets a flag that tells neighbors to NOT range to this node
#define RN_CONFIG_FLAG_DO_NOT_RANGE_TO_ME					(1 << 1)
// Selects precision vs. filtered ranges
#define RN_CONFIG_FLAG_NEIGHBOR_DATABASE_FILTERED_RANGE		(1 << 2)

// (1 << 3) is for a deprecated flag that was never used. It is available for the next flag to be added.

// Enables sending of RN_TIMING_INFO messages to the host
#define RN_CONFIG_FLAG_ENABLE_TIMING_INFOS					(1 << 4)


// ALOHA config flags

// Set this node in beacon mode. In beacon mode, only beacon packets are automatically transmitted (i.e. no automatic ranging to neighbors).
#define RN_ALOHA_CONFIG_FLAG_BEACON							(1 << 0)

// Use Slotted ALOHA. In Slotted ALOHA, transmit timing is rounded up to slot boundaries. Not currently supported.
#define RN_ALOHA_CONFIG_FLAG_SLOTTED_ALOHA					(1 << 1)

// Use Automatic Congestion Control (ACC). When ACC is enabled, the min and max TX intervals are automatically determined based on the number of neighboring RangeNet units.
#define RN_ALOHA_CONFIG_FLAG_USE_ACC						(1 << 2)


// TDMA slot flags

// Sleep during this slot if this node is neither the requester or potential responder.
#define RN_TDMA_SLOT_FLAG_SLEEP				(1 << 0)
// Allocate time in this slot for sending requester user data.
#define RN_TDMA_SLOT_FLAG_REQUESTER_DATA	(1 << 1)
// Allocate time in this slot for sending responder user data.
#define RN_TDMA_SLOT_FLAG_RESPONDER_DATA	(1 << 2)


// RN_SET_TDMA_SLOTMAP_REQUEST flags

// Modify the slotmap with the message instead of overwriting it.
#define RN_TDMA_SLOTMAP_FLAG_MODIFY		(1 << 0)


// Database and stats reset selection flags
// Used in RN_RESET_DATABASE_AND_STATS_REQUEST
// May be combined

// Clears all RangeNet neighbor database entries, or, if a nodeId is specified, removes that neighbor.
// Note that the neighbor database will still automatically repopulate as long as RangeNet is running normally.
// By implication, includes the effect of RN_RESET_FLAG_NEIGHBOR_STATS.
#define RN_RESET_FLAG_DATABASE			(1 << 0)

// Clears all RangeNet health stats. nodeId is irrelevant for this flag.
#define RN_RESET_FLAG_HEALTH_STATS		(1 << 1)

// Clears individual neighbor statistics (but doesn't remove from the database) for all nodes or a single specified node.
#define RN_RESET_FLAG_NEIGHBOR_STATS	(1 << 2)

// RN_TIMING_INFO UWB packet types
#define RN_TIMING_INFO_PKT_TYPE_RCM_REQUEST		(1)
#define RN_TIMING_INFO_PKT_TYPE_RCM_RESPONSE	(2)
#define RN_TIMING_INFO_PKT_TYPE_RCM_DATA		(3)
#define RN_TIMING_INFO_PKT_TYPE_RN_REQUEST		(4)
#define RN_TIMING_INFO_PKT_TYPE_RN_RESPONSE		(5)
#define RN_TIMING_INFO_PKT_TYPE_RN_BEACON		(6)
#define RN_TIMING_INFO_PKT_TYPE_RN_DATA			(9)

///////////////////////////////////////
//
// Message struct definitions
//

typedef struct
{
	// Maximum age of entries in the neighbor database.
	// Not hearing from a neighbor for this amount of time will cause it to be removed from the database.
	rcrm_uint32_t maxNeighborAgeMs;

	// Sets the update interval when autosending the neighbor database.
	rcrm_uint16_t autosendNeighborDbUpdateIntervalMs;

	// RangeNet behavior, etc. flags
	rcrm_uint16_t rnFlags;

	// RN_NETWORK_SYNC_MODE_ALOHA or RN_NETWORK_SYNC_MODE_TDMA
	rcrm_uint8_t networkSyncMode;

	// Specifies automatically sending RANGE_INFO, FULL_ or SMALL_NEIGHBOR_DATABASE, etc.
	// Bits 0-1 are RANGE_INFO flags.
	// Bits 2-3 are NEIGHBOR_DATABASE flags.
	// Bits 4-5 are neighbor database sort flags (RN_NEIGHBOR_DATABASE_SORT_RANGE, etc.)
	// Can be a combination of any one of RANGE_INFO, any one of NEIGHBOR_DB, and any one of the neighbor database sort flags.
	// Default value is RN_AUTOSEND_RANGEINFO_NONE | RN_AUTOSEND_NEIGHBOR_DATABASE_FULL | (RN_NEIGHBOR_DATABASE_SORT_NODEID << 4)
	rcrm_uint8_t autosendType;
	
	rcrm_uint8_t reserved;
	
	// Any one of the RN_INTERFACE_* interface selections.
	rcrm_uint8_t defaultIf;
	
	// Destination address fields, used by some interfaces.
	rcrm_uint32_t defaultIfAddr1;
	rcrm_uint32_t defaultIfAddr2;
} rnConfiguration;

typedef struct
{
	// Minimum and maximum time from transmit to transmit.
	// In slotted aloha, these times will be rounded up to make the next transmission start on a slot boundary.
	rcrm_uint16_t minTimeBetweenTxMs;
	rcrm_uint16_t maxTimeBetweenTxMs;

	// Maximum number of bytes the host application is going to use in the user data section of range requests and responses.
	// Used to calculate slot times, since sending user data will, in some cases significantly, decrease network update rate due to longer packets.
	// Valid values are 0 bytes to RN_USER_DATA_LENGTH bytes.
	// Note that these are the same values as used by TDMA, i.e. writing one writes the other.
	rcrm_uint16_t maxRequestDataSize;
	rcrm_uint16_t maxResponseDataSize;

	// Beacon mode, etc.
	rcrm_uint16_t flags;
	
	// Reserved; set to 0
	rcrm_uint8_t accKfactor;
	
	// Reserved; set to 0
	rcrm_uint8_t reserved;
} rnALOHAConfiguration;

typedef struct
{
	// Maximum number of bytes the host application is going to use in the user data section of range requests and responses.
	// Used to calculate slot times, since sending user data will, in some cases significantly, decrease network update rate due to longer packets.
	// Valid values are 0 bytes to RN_USER_DATA_LENGTH bytes.
	// Note that these are the same values as used by ALOHA, i.e. writing one writes the other.
	rcrm_uint16_t maxRequestDataSize;
	rcrm_uint16_t maxResponseDataSize;

	// Reserved
	rcrm_uint16_t reserved1;
	
	// Reserved
	rcrm_uint16_t reserved2;
} rnTDMAConfiguration;

typedef enum
{
	// Used to indicate an unconfigured/invalid slot,
	// such as the slot marking the end of the slotmap if not all RN_MAX_TDMA_SLOTS are filled.
	rnTDMA_SlotType_None = 0,

	// In this type of slot, the owner sends out a range request.
	rnTDMA_SlotType_Range,
	
	// In this type of slot, the owner sends out a data packet (no scan data or range response)
	rnTDMA_SlotType_Data,
} rnTDMA_SlotType;

typedef struct
{
	// rnTDMA_SlotType type of slot.
	// Declared as rcrm_uint8_t to explicitly make it 1 byte.
	rcrm_uint8_t slotType;
	
	// Index of this slot in the slotmap.
	rcrm_uint8_t slotNumber;

	// RN_TDMA_SLOT_FLAG_SLEEP, etc.
	rcrm_uint16_t flags;

	// Integration, antenna, and channel use the same values as RCM's equivalent configuration fields.
	rcrm_uint8_t integrationIndex;
	// Antenna mode: lower nibble is owner antenna mode, and upper nibble is potential responders' antenna mode.
	rcrm_uint8_t antennaMode;
	rcrm_uint8_t codeChannel;

	// Reserved for alignment and future growth.
	rcrm_uint8_t reserved1;

	// UWB node ID of the slot's owner.
	rcrm_uint32_t requesterId;
	
	// UWB node ID of the node to which the slot's owner should transmit or range.
	// Ignored unless slotType == rnTDMA_SlotType_Range.
	// If responderId == 0xFFFFFFFF (the ranging "broadcast" address), the round-robin method of choosing a target is used.
	// If responderId == 0 (none), then a RangeNet beacon packet is sent instead. However, this is not likely to be a useful configuration.
	rcrm_uint32_t responderId;

	// Requested slot duration, in microseconds.
	// If the slot as configured won't fit in this duration, an error is returned in RN_SET_TDMA_SLOTMAP_CONFIRM.
	// 0 (default) means use value computed by radio.
	rcrm_uint32_t requestedDurationMicroseconds;
} rnTDMASlotDefinition;

// Slot defintion along with computed duration.
// Used in messages containing slot info sent from the radio to the host app.
typedef struct
{
	// The slot definition.
	rnTDMASlotDefinition slot;
	
	// Minimum slot duration in microseconds, computed by the radio.
	rcrm_uint32_t minimumDurationMicroseconds;
} rnTDMASlotWithComputedDuration;

///////////////////////////////////////
//
// Host <-> RN conversation messages
//

// RangeNet general config messages

typedef struct
{
	// set to RN_SET_CONFIG_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rnConfiguration config;

	// Set to non-zero to indicate settings should persist across radio reboots
	// Not used in get config response.
	rcrm_uint8_t persistFlag;
	
	rcrm_uint8_t reserved[3];
} rnMsg_SetConfigRequest;

typedef struct
{
	// set to RN_SET_CONFIG_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// status code
	rcrm_uint32_t status;
} rnMsg_SetConfigConfirm;


typedef struct
{
	// set to RN_GET_CONFIG_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rnMsg_GetConfigRequest;

typedef struct
{
	// set to RN_GET_CONFIG_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rnConfiguration config;

	// milliseconds since radio boot
	rcrm_uint32_t timestamp;

	// status code
	rcrm_uint32_t status;
} rnMsg_GetConfigConfirm;


// ALOHA config messages

typedef struct
{
	// set to RN_SET_ALOHA_CONFIG_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rnALOHAConfiguration config;

	// Set to non-zero to indicate settings should persist across radio reboots
	// Not used in get config response.
	rcrm_uint8_t persistFlag;
	
	rcrm_uint8_t reserved[3];
} rnMsg_SetALOHAConfigRequest;

typedef struct
{
	// set to RN_SET_ALOHA_CONFIG_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// status code
	rcrm_uint32_t status;
} rnMsg_SetALOHAConfigConfirm;


typedef struct
{
	// set to RN_GET_ALOHA_CONFIG_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rnMsg_GetALOHAConfigRequest;

typedef struct
{
	// set to RN_GET_ALOHA_CONFIG_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rnALOHAConfiguration config;

	// status code
	rcrm_uint32_t status;
} rnMsg_GetALOHAConfigConfirm;



// TDMA config messages

typedef struct
{
	// set to RN_SET_TDMA_CONFIG_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rnTDMAConfiguration config;

	// Set to non-zero to indicate settings should persist across radio reboots
	// Not used in get config response.
	rcrm_uint8_t persistFlag;
	
	rcrm_uint8_t reserved[3];
} rnMsg_SetTDMAConfigRequest;

typedef struct
{
	// set to RN_SET_TDMA_CONFIG_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// status code
	rcrm_uint32_t status;
} rnMsg_SetTDMAConfigConfirm;


typedef struct
{
	// set to RN_GET_TDMA_CONFIG_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rnMsg_GetTDMAConfigRequest;

typedef struct
{
	// set to RN_GET_TDMA_CONFIG_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rnTDMAConfiguration config;

	// status code
	rcrm_uint32_t status;
} rnMsg_GetTDMAConfigConfirm;


typedef struct
{
	// set to RN_SET_TDMA_SLOTMAP_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	// Number of slots in this variable-length message
	rcrm_uint8_t numSlots;

	// RN_TDMA_SLOTMAP_FLAG_MODIFY, etc.
	rcrm_uint8_t slotmapFlags;

	// Reserved for aligment and future growth
	rcrm_uint8_t reserved;

	// Set to non-zero to indicate settings should persist across radio reboots
	rcrm_uint8_t persistFlag;
	
	// The slot definitions.
	// Only the first numSlots need be included in this variable-length message.
	rnTDMASlotDefinition slots[RN_MAX_TDMA_SLOTS];
} rnMsg_SetTDMASlotmapRequest;

typedef struct
{
	// set to RN_SET_TDMA_SLOTMAP_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// status code
	rcrm_uint32_t status;
} rnMsg_SetTDMASlotmapConfirm;


typedef struct
{
	// set to RN_GET_TDMA_SLOTMAP_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rnMsg_GetTDMASlotmapRequest;

typedef struct
{
	// set to RN_GET_TDMA_SLOTMAP_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	// Number of slots in this variable-length packet
	rcrm_uint8_t numSlots;
	
	rcrm_uint8_t reserved[3];

	// status code
	rcrm_uint32_t status;

	// The slot definitions, plus durations computed by the radio.
	// Only the first numSlots are included in this variable-length message.
	rnTDMASlotWithComputedDuration slots[RN_MAX_TDMA_SLOTS];
} rnMsg_GetTDMASlotmapConfirm;

typedef struct
{
	// set to RN_GET_TDMA_SLOT_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	// Zero-based index of the slot whose info is to be returned.
	rcrm_uint8_t slotNumber;

	// Reserved for alignment and future growth; set to zero.
	rcrm_uint8_t reserved[3];
} rnMsg_GetTDMASlotRequest;

typedef struct
{
	// set to RN_GET_TDMA_SLOT_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	// status code
	rcrm_uint32_t status;

	// The slot definition, plus duration computed by the radio.
	rnTDMASlotWithComputedDuration slot;
} rnMsg_GetTDMASlotConfirm;

typedef struct
{
	// set to RN_SET_REQUEST_USER_DATA_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rcrm_uint16_t reserved;	// alignment

	// Number of bytes of data to send in range request.
	// Can be up to RN_USER_DATA_LENGTH or maxRequestDataSize in the configuration, whichever is smaller.
	rcrm_uint16_t dataSize;

	// User data for payload.
	// Only the first dataSize bytes need to actually be in the UDP packet.
	rcrm_uint8_t data[RN_USER_DATA_LENGTH];
} rnMsg_SetRequestDataRequest;

typedef struct
{
	// set to RN_SET_REQUEST_USER_DATA_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// status code
	rcrm_uint32_t status;
} rnMsg_SetRequestDataConfirm;

typedef struct
{
	// set to RN_SET_RESPONSE_USER_DATA_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rcrm_uint16_t reserved;	// alignment

	// Number of bytes of data to send in range response.
	// Can be up to RN_USER_DATA_LENGTH or maxResponseDataSize in the configuration, whichever is smaller.
	rcrm_uint16_t dataSize;

	// User data for payload.
	// Only the first dataSize bytes need to actually be in the UDP packet.
	rcrm_uint8_t data[RN_USER_DATA_LENGTH];
} rnMsg_SetResponseDataRequest;

typedef struct
{
	// set to RN_SET_RESPONSE_USER_DATA_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// status code
	rcrm_uint32_t status;
} rnMsg_SetResponseDataConfirm;

typedef struct
{
	// set to RN_GET_FULL_NEIGHBOR_DATABASE_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	rcrm_uint8_t maxNeighborEntries;
	rcrm_uint8_t sortType;
	
	rcrm_uint16_t reserved;
} rnMsg_GetFullNeighborDatabaseRequest;

typedef struct
{
	rcrm_uint32_t nodeId;
	
	rcrm_uint8_t rangeStatus;
	
	rcrm_uint8_t antennaMode;
	
	rcrm_uint16_t stopwatchTime;
	
	rcrm_uint32_t rangeMm;
	rcrm_uint16_t rangeErrorEstimate;
	
	rcrm_int16_t rangeVelocity;
	
	rcrm_uint8_t rangeMeasurementType;
	
	rcrm_uint8_t flags;
	
	rcrm_uint16_t ledFlags;
	
	rcrm_uint16_t noise;
	rcrm_uint16_t vPeak;
	
	rcrm_uint16_t statsNumRangeAttempts;
	rcrm_uint16_t statsNumRangeSuccesses;
	
	rcrm_uint32_t statsAgeMs;
	
	rcrm_uint32_t rangeUpdateTimestampMs;
	rcrm_uint32_t lastHeardTimestampMs;
	rcrm_uint32_t addedToNDBTimestampMs;
} rnFullNeighborDatabaseEntry_t;

typedef struct
{
	// set to RN_GET_FULL_NEIGHBOR_DATABASE_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// Number of neighbors in this database message
	rcrm_uint8_t numNeighborEntries;

	// Sort type used
	rcrm_uint8_t sortType;

	rcrm_uint16_t reserved;

	rcrm_uint32_t timestamp;

	rcrm_uint32_t status;

	rnFullNeighborDatabaseEntry_t neighbors[RN_FULL_NEIGHBOR_DATABASE_MAX_ENTRY_COUNT];
} rnMsg_GetFullNeighborDatabaseConfirm;

typedef struct
{
	// set to RN_GET_SMALL_NEIGHBOR_DATABASE_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rcrm_uint8_t maxNeighborEntries;
	rcrm_uint8_t sortType;
	
	rcrm_uint16_t reserved;
} rnMsg_GetSmallNeighborDatabaseRequest;


typedef struct
{
	rcrm_uint32_t nodeId;

	rcrm_uint16_t rangeCm;
	rcrm_uint8_t rangeErrEst;
	
	rcrm_uint8_t reserved;

	rcrm_uint16_t ageMs;
	
	rcrm_uint8_t rangeMeasurementType;
	
	rcrm_uint8_t flags;
} rnSmallNeighborDatabaseEntry_t;

typedef struct
{
	// set to RN_GET_SMALL_NEIGHBOR_DATABASE_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rcrm_uint8_t numNeighborEntries;
	
	rcrm_uint8_t sortType;
	
	rcrm_uint16_t reserved;

	rnSmallNeighborDatabaseEntry_t neighbors[RN_SMALL_NEIGHBOR_DATABASE_MAX_ENTRY_COUNT];
} rnMsg_GetSmallNeighborDatabaseConfirm;


typedef struct
{
	// set to RN_SET_EXCLUDED_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rcrm_uint8_t numTargets;
	
	rcrm_uint8_t reserved[3];
	
	// This is variable length, with a max length of RN_MAX_EXCLUDED_TARGETS of course.
	// Only the first numTargets need to be in the packet.
	rcrm_uint32_t excludedTargets[RN_MAX_EXCLUDED_TARGETS];
} rnMsg_SetExcludedRequest;

typedef struct
{
	// set to RN_SET_EXCLUDED_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	rcrm_uint32_t status;
} rnMsg_SetExcludedConfirm;

typedef struct
{
	// set to RN_GET_EXCLUDED_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rnMsg_GetExcludedRequest;

typedef struct
{
	// set to RN_GET_EXCLUDED_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rcrm_uint8_t numTargets;
	
	rcrm_uint8_t reserved[3];
	
	// This is variable length, with a max length of RN_MAX_EXCLUDED_TARGETS of course.
	// Only the first numTargets need to be in the packet.
	rcrm_uint32_t excludedTargets[RN_MAX_EXCLUDED_TARGETS];
} rnMsg_GetExcludedConfirm;

typedef struct
{
	// set to RN_GET_HEALTH_STATUS_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rnMsg_GetHealthStatusRequest;

typedef struct
{
	// set to RN_GET_HEALTH_STATUS_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// Divide this by 4 to get temperature in degrees C.
	rcrm_int32_t temperature;

	// Number of neighbors in the neighbor database.
	rcrm_uint32_t numNeighbors;

	// Time over which the following counters were accumulated
	rcrm_uint32_t statisticsTimeMs;
	
	rcrm_uint32_t numRangeAttempts;
	rcrm_uint32_t numPrecisionRangeSuccesses;
	rcrm_uint32_t numCoarseRangeEstimates;
	rcrm_uint32_t numRangeTimeouts;
	rcrm_uint32_t numRangeVCS;
	rcrm_uint32_t numRangeLEDFailure;
	rcrm_uint32_t numRangeCCIFailure;
} rnMsg_GetHealthStatusConfirm;

typedef struct
{
	// set to RN_RESET_DATABASE_AND_STATS_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	// Flags indicating which items to reset (entire neighbor database info, just the statistics, etc.)
	rcrm_uint32_t resetFlags;

	// Node ID of the neighbor to reset, or 0 to reset for all neighbors/globally
	rcrm_uint32_t nodeId;
} rnMsg_ResetDatabaseAndStatsRequest;

typedef struct
{
	// set to RN_RESET_DATABASE_AND_STATS_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	rcrm_uint32_t status;
} rnMsg_ResetDatabaseAndStatsConfirm;

typedef struct
{
	// set to RN_GET_REQUEST_USER_DATA_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rnMsg_GetRequestDataRequest;

typedef struct
{
	// set to RN_GET_REQUEST_USER_DATA_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rcrm_uint16_t reserved;	// alignment

	// Number of bytes of data to send in range request.
	// Can be up to RN_USER_DATA_LENGTH or maxRequestDataSize in the configuration, whichever is smaller.
	rcrm_uint16_t dataSize;

	// User data for payload.
	// Only the first dataSize bytes need to actually be in the UDP packet.
	rcrm_uint8_t data[RN_USER_DATA_LENGTH];
} rnMsg_GetRequestDataConfirm;

typedef struct
{
	// set to RN_GET_RESPONSE_USER_DATA_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rnMsg_GetResponseDataRequest;

typedef struct
{
	// set to RN_GET_RESPONSE_USER_DATA_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rcrm_uint16_t reserved;	// alignment

	// Number of bytes of data to send in range response.
	// Can be up to RN_USER_DATA_LENGTH or maxResponseDataSize in the configuration, whichever is smaller.
	rcrm_uint16_t dataSize;

	// User data for payload.
	// Only the first dataSize bytes need to actually be in the UDP packet.
	rcrm_uint8_t data[RN_USER_DATA_LENGTH];
} rnMsg_GetResponseDataConfirm;


typedef struct
{
	// set to RN_GET_PACKET_DURATIONS_REQUEST
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
} rnMsg_GetPacketDurationsRequest;

typedef struct
{
	// set to RN_GET_PACKET_DURATIONS_CONFIRM
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;

	rcrm_uint32_t rangeReqNoDataMicroseconds;
	rcrm_uint32_t rangeRespNoDataMicroseconds;

	rcrm_uint32_t rangeReqMaxDataMicroseconds;
	rcrm_uint32_t rangeRespMaxDataMicroseconds;

	rcrm_uint32_t rangeConversationNoDataMicroseconds;
	rcrm_uint32_t rangeConversationMaxDataMicroseconds;

	rcrm_uint32_t dataNoDataMicroseconds;
	rcrm_uint32_t dataMaxDataMicroseconds;
} rnMsg_GetPacketDurationsConfirm;

typedef struct
{
	// set to RN_TIMING_INFO
	rcrm_uint16_t msgType;
	// identifier to correlate requests with confirms
	rcrm_uint16_t msgId;
	
	// Transmitter node ID, if applicable.
	rcrm_uint32_t srcNodeId;
	
	// Responder/Destination node ID, if applicable.
	rcrm_uint32_t destNodeId;
	
	// Milliseconds since radio boot time
	rcrm_uint32_t millisecondTimestamp;
	
	// Microsecond counter maintained by high-accuracy clock.
	rcrm_uint32_t microsecondTimestamp;
	
	// Network clock in microseconds
	rcrm_uint32_t microsecondNetworkClock;

	// Reserved for alignment future growth
	rcrm_uint8_t reserved[2];
	
	// RCRM_UWBPKTTYPE_RCMRANGEREQ, etc.
	rcrm_uint8_t uwbPktType;
	
	// Zero-based index of the slot in the superframe
	rcrm_uint8_t slotIndex;
	
	// Superframe count
	rcrm_uint32_t superframeNumber;
} rnMsg_TimingInfo;

#endif	// #ifdef __rcrmHostInterfaceRN_h
 
