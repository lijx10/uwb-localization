//_____________________________________________________________________________
//
// Copyright 2011-2 Time Domain Corporation
//
//
// rnSampleApp.h
//
//   Declarations for RCM communications functions.
//
//_____________________________________________________________________________

#ifndef __rnSampleApp_h
#define __rnSampleApp_h

#ifdef __cplusplus
    extern "C" {
#endif

//_____________________________________________________________________________
//
// #includes
//_____________________________________________________________________________
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// pull in message structure declarations
#include "hostInterfaceCommon.h"
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
//  printRcmConfig
//
//  Parameters:  rcmConfiguration *rcmConfig
//  Return:      void
//
//  Prints out the RCM configuration to the console. Only the parameters
//  relevant to the sample app are printed.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void printRcmConfig(rcmConfiguration *rcmConfig);


//
//  printRnConfig
//
//  Parameters:  rnConfiguration *rnConfig
//  Return:      void
//
//  Prints out the RangeNet configuration to the console. Only the parameters
//  relevant to the sample app are printed.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void printRnConfig(rnConfiguration *rnConfig);


//
//  printRnAlohaConfig
//
//  Parameters:  rnALOHAConfiguration *rnAlohaConfig
//  Return:      void
//
//  Prints out the ALOHA configuration to the console. Only the parameters
//  relevant to the sample app are printed.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void printRnAlohaConfig(rnALOHAConfiguration *rnAlohaConfig);


//
//  printRnSlotmap
//
//  Parameters:  rnMsg_SetTDMASlotmapRequest *rnSlotmap
//  Return:      void
//
//  Prints out the TDMA slotmap to the console.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void printRnSlotmapConfirm(rnMsg_GetTDMASlotmapConfirm *rnSlotmap);
void printRnSlotmapRequest(rnMsg_SetTDMASlotmapRequest *rnSlotmap);


void printRnTdmaConfig(rnTDMAConfiguration *rnTdmaConfig);

#ifdef __cplusplus
    }
#endif


#endif
