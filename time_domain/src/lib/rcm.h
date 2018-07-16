//_____________________________________________________________________________
//
// Copyright 2011-2 Time Domain Corporation
//
//
// rcm.h
//
//   Declarations for RCM communications functions.
//
//_____________________________________________________________________________

#ifndef __rcm_h
#define __rcm_h

#ifdef __cplusplus
    extern "C" {
#endif

//_____________________________________________________________________________
//
// #includes
//_____________________________________________________________________________

// pull in message structure declarations
#include "hostInterfaceRCM.h"
#include "hostInterfaceRN.h"


//_____________________________________________________________________________
//
// #defines
//_____________________________________________________________________________

#ifndef OK
#define OK 0
#define ERR (-1)
#endif

//_____________________________________________________________________________
//
// typedefs
//_____________________________________________________________________________



//_____________________________________________________________________________
//
//  Function prototypes
//_____________________________________________________________________________


//
//  rcmBit
//
//  Parameters:  int *status - returned BIT status
//  Return:      OK or ERR
//
//  Sends RCM_BIT_REQUEST to radio and waits for RCM_BIT_CONFIRM.
//  If confirm message is received and BIT passes, returns OK.
//  If confirm message is not received or BIT fails, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rcmBit(int *status);


//
//  rcmConfigGet
//
//  Parameters:  rcmConfiguration *config - pointer to structure to hold
//                      configuration
//  Return:      OK or ERR
//
//  Sends RCM_GET_CONFIG_REQUEST to radio and waits for RCM_GET_CONFIG_CONFIRM.
//  If confirm message is received, fills in config and returns OK.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rcmConfigGet(rcmConfiguration *config);


//
//  rcmConfigSet
//
//  Parameters:  rcmConfiguration *config - pointer to structure containing
//                      new configuration
//  Return:      OK or ERR
//
//  Sends RCM_SET_CONFIG_REQUEST to radio and waits for RCM_SET_CONFIG_CONFIRM.
//  If confirm message is received, returns OK.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rcmConfigSet(rcmConfiguration *config);


//
//  rcmOpModeSet
//
//  Parameters:  int opMode - set to RCM_OPMODE_RCM
//  Return:      OK or ERR
//
//  Sends RCM_SET_OPMODE_REQUEST to radio and waits for RCM_SET_OPMODE_CONFIRM.
//  If confirm message is received, returns OK.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rcmOpModeSet(int opMode);


//
//  rcmOpModeGet
//
//  Parameters:  int opMode - set to RCM_OPMODE_RCM
//  Return:      OK or ERR
//
//  Sends RCM_SET_OPMODE_REQUEST to radio and waits for RCM_SET_OPMODE_CONFIRM.
//  If confirm message is received, returns OK.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rcmOpModeGet(int *opMode);


//
//  rcmSleepModeSet
//
//  Parameters:  int sleepMode - desired sleep mode
//  Return:      OK or ERR
//
//  Sends RCM_SET_SLEEP_MODE_REQUEST to radio and waits for
//  RCM_SET_SLEEP_MODE_CONFIRM.
//  If confirm message is received, returns OK.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rcmSleepModeSet(int sleepMode);


//
//  rcmStatusInfoGet
//
//  Parameters:  rcmMsg_GetStatusInfoConfirm *statusInfo - pointer to structure
//                      to receive status info message
//  Return:      OK or ERR
//
//  Sends RCRM_GET_STATUS_INFO_REQUEST to radio and waits for
//  RCRM_GET_STATUS_INFO_CONFIRM. If confirm message is received, fills in
//  statusInfo and returns OK. If confirm message is not received, returns
//  ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rcmStatusInfoGet(rcrmMsg_GetStatusInfoConfirm *statusInfo);


//
//  rcmRangeTo
//
//  Parameters:  int destNodeId - node ID of radio to range to
//               int antennaMode - antenna mode to use (see API for details)
//               int dataSize - number of bytes of data to send in range request
//               char *data - data to send in range request
//               rcmMsg_RangeInfo *rangeInfo - pointer to struct to hold range info
//               rcmMsg_DataInfo *dataInfo - pointer to struct to hold data info
//               rcmMsg_ScanInfo *scanInfo - pointer to struct to hold scan info
//               rcmMsg_FullScanInfo *fullScanInfo - pointer to struct to hold full scan info
//  Return:      OK or ERR
//
//  Sends RCM_SEND_RANGE_REQUEST to radio and waits for RCM_SEND_RANGE_CONFIRM.
//  If confirm message is received, receives up to 4 info messages and stores
//  them in rangeInfo, dataInfo, scanInfo, and fullScanInfo. The rangeStatus field in the
//  rangeInfo structure will indicate whether the range was successfully
//  calculated. If data was received in the range response, it will be in the
//  dataInfo structure and the dataSize field will be non-zero. If the radio is
//  configured to send scans (SEND_SCAN bit is set in config flags), the
//  scan samples will be in the scanInfo structure and the numSamples field
//  will be non-zero. if the radio is configured to send full scans (SEND_FULL_SCAN
//  bit is set in config flags, the scan samples will be in the fullScanInfo structure
//  and the numSamples field will be non-zero.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rcmInfoGet(rcmMsg_FullRangeInfo *rangeInfo, rnMsg_GetFullNeighborDatabaseConfirm *ndbInfo);
int rcmInfoGetFull(rcmMsg_FullRangeInfo *rangeInfo,
               rnMsg_GetFullNeighborDatabaseConfirm *ndbInfo,
               rcmMsg_DataInfo *dataInfo,
               rcmMsg_EchoedRangeInfo *echoedInfo);
//
//  rcmDataSend
//
//  Parameters:  int antennaMode - antenna mode to use (see API for details)
//               int dataSize - number of bytes of data to send
//               char *data - data to send
//  Return:      OK or ERR
//
//  Sends RCM_SEND_DATA_REQUEST to radio and waits for RCM_SEND_DATA_CONFIRM.
//  If confirm message is received, returns OK.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rcmDataSend(int antennaMode, int dataSize, unsigned char *data);

#ifdef __cplusplus
    }
#endif


#endif
