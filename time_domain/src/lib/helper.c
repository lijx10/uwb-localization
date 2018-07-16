//__________________________________________________________________________________________________
//
// Copyright 2013-4 Time Domain Corporation
//
//
// rnSampleApp.c
//
//   Sample code showing how to interface to P400 RCM module embedded RangeNet software.
//
//   This code uses the functions in rcm.c and rn.c to:
//      - make sure the RCM is awake and in the correct mode
//      - get the base, RangeNet, and ALOHA configurations from the RCM and print them
//      - get the status/info from the RCM and print it
//		- modify the base, RangeNet, and ALOHA configurations for sample app use (and print them)
//		- allow the user to choose between receiving Range Info or Neighbor Database messages
//		- display received messages until either a key is hit (Windows) or 10 sec elapses (linux)
//      - restore original configuration upon exit
//
// This sample can communicate with the RCM over Ethernet, the 3.3V serial port,
// or the USB interface (which acts like a serial port).
//
//__________________________________________________________________________________________________


//_____________________________________________________________________________
//
// #includes 
//_____________________________________________________________________________

#include "helper.h"


//_____________________________________________________________________________
//
// local function prototypes
//_____________________________________________________________________________

//_____________________________________________________________________________
//
// usage - display command line parameters when none are provided
//_____________________________________________________________________________
void usage(void)
{
#ifdef WIN32
	printf("usage: rnSampleApp -i <IP address> | -s <COM port> | -u <USB COM port>\n");
#else
	printf("usage: rnSampleApp -i <IP address> | -s <COM port> | -u <Node ID>\n");
#endif
    printf("\nTo connect to radio at IP address 192.168.1.100 via Ethernet:\n");
    printf("\trnSampleApp -i 192.168.1.100\n");
    printf("\nTo connect to radio's serial port using USB-to-TTL serial converter at COM3:\n");
    printf("\trnSampleApp -s COM3\n");
#ifdef WIN32
    printf("\nTo connect to radio's USB port at COM10:\n");
    printf("\trnSampleApp -u COM10\n");
#else
    printf("\nTo connect to Node ID via USB:\n");
    printf("\trnSampleApp -u 100\n");
#endif
	exit(0);
}

//_____________________________________________________________________________
//
// printRcmConfig - prints base configuration using tags from API doc
//_____________________________________________________________________________
void printRcmConfig(rcmConfiguration *rcmConfig)
{
    printf("\tnodeId: %d\n", rcmConfig->nodeId);
    printf("\tintegrationIndex: %d\n", rcmConfig->integrationIndex);
    printf("\tantennaMode: %d\n", rcmConfig->antennaMode);
    printf("\tcodeChannel: %d\n", rcmConfig->codeChannel);
    printf("\telectricalDelayPsA: %d\n", rcmConfig->electricalDelayPsA);
    printf("\telectricalDelayPsB: %d\n", rcmConfig->electricalDelayPsB);
    printf("\tflags: 0x%X\n", rcmConfig->flags);
	printf("\ttxGain: %d\n", rcmConfig->txGain);
}

//_____________________________________________________________________________
//
// printRnConfig - prints RangeNet configuration using tags from API doc
//_____________________________________________________________________________
void printRnConfig(rnConfiguration *rnConfig)
{
    printf("\tmaxNeighborAgeMs: %d\n", rnConfig->maxNeighborAgeMs);
    printf("\tautosendNeighborDbUpdateIntervalMs: %d\n", rnConfig->autosendNeighborDbUpdateIntervalMs);
    printf("\trnFlags: 0x%X\n", rnConfig->rnFlags);
    printf("\tnetworkSyncMode: %d\n", rnConfig->networkSyncMode);
    printf("\tautosendType: 0x%X\n", rnConfig->autosendType);
    printf("\tdefaultIf: %d\n", rnConfig->defaultIf);
    printf("\tdefaultIfAddr1: %d\n", rnConfig->defaultIfAddr1);
    printf("\tdefaultIfAddr2: %d\n", rnConfig->defaultIfAddr2);
}

//_____________________________________________________________________________
//
// printRnAlohaConfig - prints ALOHA configuration using tags from API doc
//_____________________________________________________________________________
void printRnAlohaConfig(rnALOHAConfiguration *rnAlohaConfig)
{
    printf("\tminTimeBetweenTxMs: %d\n", rnAlohaConfig->minTimeBetweenTxMs);
    printf("\tmaxTimeBetweenTxMs: %d\n", rnAlohaConfig->maxTimeBetweenTxMs);
    printf("\tmaxRequestDataSize: %d\n", rnAlohaConfig->maxRequestDataSize);
    printf("\tmaxResponseDataSize: %d\n", rnAlohaConfig->maxResponseDataSize);
    printf("\tFlags: 0x%X\n", rnAlohaConfig->flags);
    printf("\taccKfactor: %d\n", rnAlohaConfig->accKfactor);
}

//_____________________________________________________________________________
//
// printRnSlotmap - prints a TDMA Slotmap
//_____________________________________________________________________________

void printRnSlotmapConfirm(rnMsg_GetTDMASlotmapConfirm *rnSlotmap)
{
    int i;

    for (i = 0; i < rnSlotmap->numSlots; i++)
    {
        printf("SLOT %02d: ", rnSlotmap->slots[i].slot.slotNumber);
        printf("Req ID:%u   ", rnSlotmap->slots[i].slot.requesterId);
        printf("Rsp ID:%u   ", rnSlotmap->slots[i].slot.responderId);
        printf("Pii:%u   ", rnSlotmap->slots[i].slot.integrationIndex);
        printf("Channel:%u   ", rnSlotmap->slots[i].slot.codeChannel);
        printf("Antenna:%u   ", rnSlotmap->slots[i].slot.antennaMode);
        printf("Flags:0x%X   ", rnSlotmap->slots[i].slot.flags);
        printf("Type:%u   ", rnSlotmap->slots[i].slot.slotType);
        printf("Man Time:%u   ", rnSlotmap->slots[i].slot.requestedDurationMicroseconds);
        printf("Auto Min Time:%u\n", rnSlotmap->slots[i].minimumDurationMicroseconds);
    }
}



void printRnSlotmapRequest(rnMsg_SetTDMASlotmapRequest *rnSlotmap)
{
    int i;

    for (i = 0; i < rnSlotmap->numSlots; i++)
    {
        printf("SLOT %02d: ", rnSlotmap->slots[i].slotNumber);
        printf("Req ID:%u   ", rnSlotmap->slots[i].requesterId);
        printf("Rsp ID:%u   ", rnSlotmap->slots[i].responderId);
        printf("Pii:%u   ", rnSlotmap->slots[i].integrationIndex);
        printf("Channel:%u   ", rnSlotmap->slots[i].codeChannel);
        printf("Antenna:%u   ", rnSlotmap->slots[i].antennaMode);
        printf("Flags:0x%X   ", rnSlotmap->slots[i].flags);
        printf("Type:%u   ", rnSlotmap->slots[i].slotType);
        printf("Man Time:%u\n", rnSlotmap->slots[i].requestedDurationMicroseconds);
    }
}


void printRnTdmaConfig(rnTDMAConfiguration *rnTdmaConfig){
    printf("\tmaxRequestDataSize: %d\n", rnTdmaConfig->maxRequestDataSize);
    printf("\tmaxResponseDataSize: %d\n", rnTdmaConfig->maxResponseDataSize);
}
