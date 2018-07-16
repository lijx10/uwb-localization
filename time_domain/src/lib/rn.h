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

#ifndef __rn_h
#define __rn_h

#ifdef __cplusplus
    extern "C" {
#endif

//_____________________________________________________________________________
//
// #includes
//_____________________________________________________________________________

// pull in message structure declarations
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
int rnConfigGet(rnConfiguration *config);


//
//  rnConfigSet
//
//  Parameters:  rnConfiguration *config - pointer to structure to hold
//                      configuration
//  Return:      OK or ERR
//
//  Sends RN_SET_CONFIG_REQUEST to radio and waits for RN_SET_CONFIG_CONFIRM.
//  If confirm message is received, returns OK.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rnConfigSet(rnConfiguration *config);


//
//  rnAlohaConfigGet
//
//  Parameters:  rnALOHAConfiguration *config - pointer to structure containing
//                      new configuration
//  Return:      OK or ERR
//
//  Sends RCM_GET_CONFIG_REQUEST to radio and waits for RCM_GET_CONFIG_CONFIRM.
//  If confirm message is received, fills in config and returns OK.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rnAlohaConfigGet(rnALOHAConfiguration *config);


//
//  rnAlohaConfigSet
//
//  Parameters:  rcmConfiguration *config - pointer to structure to hold
//						configuration
//  Return:      OK or ERR
//
//  Sends RCM_SET_CONFIG_REQUEST to radio and waits for RCM_SET_CONFIG_CONFIRM.
//  If confirm message is received, returns OK.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rnAlohaConfigSet(rnALOHAConfiguration *config);

//
//  rnTdmaConfigGet
//
//  Parameters:  rnTDMAConfiguration *config - pointer to structure containing
//                      new configuration
//  Return:      OK or ERR
//
//  Sends RN_GET_TDMA_CONFIG_REQUEST to radio and waits for RN_GET_TDMA_CONFIG_CONFIRM.
//  If confirm message is received, returns OK.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rnTdmaConfigGet(rnTDMAConfiguration *config);


//
//  rnTdmaConfigSet
//
//  Parameters:  rnTDMAConfiguration *config - pointer to structure to hold
//                      configuration
//  Return:      OK or ERR
//
//  Sends RN_SET_TDMA_CONFIG_REQUEST to radio and waits for RN_SET_TDMA_CONFIG_CONFIRM.
//  If confirm message is received, returns OK.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rnTdmaConfigSet(rnTDMAConfiguration *config);


//
//  rnTdmaSlotMapGet
//
//  Parameters:  rnMsg_GetTDMASlotmapConfirm *config - pointer to structure containing
//                      new configuration
//  Return:      OK or ERR
//
//  Sends RN_GET_TDMA_SLOTMAP_REQUEST to radio and waits for RN_GET_TDMA_SLOTMAP_CONFIRM.
//  If confirm message is received, returns OK.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rnTdmaSlotMapGet(rnMsg_GetTDMASlotmapConfirm *slotMap);


//
//  rnTdmaSlotMapSet
//
//  Parameters:  rnMsg_SetTDMASlotmapRequest *config - pointer to structure to hold
//                      configuration
//  Return:      OK or ERR
//
//  Sends RN_SET_TDMA_SLOTMAP_REQUEST to radio and waits for RN_SET_TDMA_SLOTMAP_CONFIRM.
//  If confirm message is received, returns OK.
//  If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rnTdmaSlotMapSet(rnMsg_SetTDMASlotmapRequest *slotMap);


//
//  rnResetNDBStats
//
//  Parameters:  void
//  Return:      OK or ERR
//
//  Sends RN_RESET_DATABASE_AND_STATS_REQUEST to radio and waits for
//  RN_RESET_DATABASE_AND_STATS_CONFIRM. If confirm message is received,
//  returns OK. If confirm message is not received, returns ERR.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rnResetNDBStats(void);

#ifdef __cplusplus
    }
#endif


#endif
