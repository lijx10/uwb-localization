//_____________________________________________________________________________
//
// Copyright 2011-2015 Time Domain Corporation
//
//
// rcm.c
//
//   A collection of functions to communicate with an RCM.
//
//
//_____________________________________________________________________________


//_____________________________________________________________________________
//
// #includes 
//_____________________________________________________________________________

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef WIN32
#include "winsock2.h"
#else // linux
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "rcmIf.h"
#include "rcm.h"
#include "rn.h"


//_____________________________________________________________________________
//
// #defines 
//_____________________________________________________________________________


//_____________________________________________________________________________
//
// typedefs
//_____________________________________________________________________________

typedef union
{
    rcmMsg_FullRangeInfo rangeInfo;
	rnMsg_GetFullNeighborDatabaseConfirm ndbInfo;
    rcmMsg_DataInfo dataInfo;
    rcmMsg_EchoedRangeInfo echoedInfo;
} infoMsgs_t;


//_____________________________________________________________________________
//
// static data
//_____________________________________________________________________________

static int msgIdCount;


//_____________________________________________________________________________
//
// Private function prototypes 
//_____________________________________________________________________________



//_____________________________________________________________________________
//
// rcmBit - execute Built-In Test
//_____________________________________________________________________________

int rcmBit(int *status)
{
    rcrmMsg_BitRequest request;
    rcrmMsg_BitConfirm confirm;
    int retVal = ERR, numBytes, waitCount;

    // create request message
	request.msgType = htons(RCRM_BIT_REQUEST);
	request.msgId = htons(msgIdCount++);

    // make sure no pending messages
    rcmIfFlush();

    // send message to RCM
	rcmIfSendPacket(&request, sizeof(request));

    // wait for response
    
    // P440s have a longer timeout for BIT, so be prepared to wait longer.
    
    for(waitCount=0; waitCount<6; waitCount++)
    {
        numBytes = rcmIfGetPacket(&confirm, sizeof(confirm));

        // did we get a response from the RCM?
        if (numBytes == sizeof(confirm))
        {
            // Handle byte ordering
            confirm.msgType = ntohs(confirm.msgType);
            confirm.status = ntohl(confirm.status);

            // is this the correct message type and is status good?
            if (confirm.msgType == RCRM_BIT_CONFIRM)
            {
                *status = confirm.status;
                retVal = OK;
            }

          break;
        }
    }

    return retVal;
}


//_____________________________________________________________________________
//
// rcmConfigGet - get rcm configuration from radio
//_____________________________________________________________________________

int rcmConfigGet(rcmConfiguration *config)
{
    rcmMsg_GetConfigRequest request;
    rcmMsg_GetConfigConfirm confirm;
    int retVal = ERR, numBytes;

    // create request message
	request.msgType = htons(RCM_GET_CONFIG_REQUEST);
	request.msgId = htons(msgIdCount++);

    // make sure no pending messages
    rcmIfFlush();

    // send message to RCM
	rcmIfSendPacket(&request, sizeof(request));

    // wait for response
    numBytes = rcmIfGetPacket(&confirm, sizeof(rcmMsg_GetConfigConfirm));

    // did we get a response from the RCM?
    if (numBytes == sizeof(rcmMsg_GetConfigConfirm))
    {
        // Handle byte ordering
        confirm.msgType = ntohs(confirm.msgType);
        confirm.msgId = ntohs(confirm.msgId);

        // is this the correct message type?
        if (confirm.msgType == RCM_GET_CONFIG_CONFIRM)
        {
            // copy config from message to caller's structure
            memcpy(config, &confirm.config, sizeof(*config));
            // Handle byte ordering
            config->nodeId = ntohl(config->nodeId);
            config->integrationIndex = ntohs(config->integrationIndex);
            config->electricalDelayPsA = ntohl(config->electricalDelayPsA);
            config->electricalDelayPsB = ntohl(config->electricalDelayPsB);
            config->flags = ntohs(config->flags);

            // milliseconds since radio boot
            confirm.timestamp = ntohl(confirm.timestamp);

            // status code
            confirm.status = ntohl(confirm.status);
            // only return OK if status is OK
            if (confirm.status == OK)
                retVal = OK;
        }
    }
    return retVal;
}


//_____________________________________________________________________________
//
// rcmConfigSet - set RCM configuration in radio
//_____________________________________________________________________________

int rcmConfigSet(rcmConfiguration *config)
{
    rcmMsg_SetConfigRequest request;
    rcmMsg_SetConfigConfirm confirm;
    int retVal = ERR, numBytes;

    // create request message
	request.msgType = htons(RCM_SET_CONFIG_REQUEST);
	request.msgId = htons(msgIdCount++);
    memcpy(&request.config, config, sizeof(*config));

    // Handle byte ordering in config struct
    request.config.nodeId = htonl(config->nodeId);
    request.config.integrationIndex = htons(config->integrationIndex);
    request.config.electricalDelayPsA = htonl(config->electricalDelayPsA);
    request.config.electricalDelayPsB = htonl(config->electricalDelayPsB);
    request.config.flags = htons(config->flags);

    // make sure no pending messages
    rcmIfFlush();

    // send message to RCM
	rcmIfSendPacket(&request, sizeof(request));

    // wait for response
    numBytes = rcmIfGetPacket(&confirm, sizeof(confirm));

    // did we get a response from the RCM?
    if (numBytes == sizeof(confirm))
    {
        // Handle byte ordering
        confirm.msgType = ntohs(confirm.msgType);
        confirm.status = ntohl(confirm.status);

        // is this the correct message type and is status good?
        if (confirm.msgType == RCM_SET_CONFIG_CONFIRM &&
                confirm.status == OK)
            retVal = OK;
    }
    return retVal;
}


//_____________________________________________________________________________
//
// rcmOpModeSet - set RCM operational mode
//_____________________________________________________________________________

int rcmOpModeSet(int opMode)
{
    rcrmMsg_SetOpmodeRequest request;
    rcrmMsg_SetOpmodeConfirm confirm;
    int retVal = ERR, numBytes;

    // create request message
	request.msgType = htons(RCRM_SET_OPMODE_REQUEST);
	request.msgId = htons(msgIdCount++);
    request.opMode = htonl(opMode);

    // make sure no pending messages
    rcmIfFlush();

    // send message to RCM
	rcmIfSendPacket(&request, sizeof(request));

    // wait for response
    numBytes = rcmIfGetPacket(&confirm, sizeof(confirm));

    // did we get a response from the RCM?
    if (numBytes == sizeof(confirm))
    {
        // Handle byte ordering
        confirm.msgType = ntohs(confirm.msgType);
        confirm.status = ntohl(confirm.status);

        // is this the correct message type and is status good?
        if (confirm.msgType == RCRM_SET_OPMODE_CONFIRM &&
                confirm.status == OK)
            retVal = OK;
    }
    return retVal;
}


//_____________________________________________________________________________
//
// rcmOpModeGet - set RCM operational mode
//_____________________________________________________________________________

int rcmOpModeGet(int *opMode)
{
    rcrmMsg_GetOpmodeRequest request;
    rcrmMsg_GetOpmodeConfirm confirm;
    int retVal = ERR, numBytes;

    // create request message
	request.msgType = htons(RCRM_GET_OPMODE_REQUEST);
	request.msgId = htons(msgIdCount++);

    // make sure no pending messages
    rcmIfFlush();

    // send message to RCM
	rcmIfSendPacket(&request, sizeof(request));

    // wait for response
    numBytes = rcmIfGetPacket(&confirm, sizeof(confirm));

    // did we get a response from the RCM?
    if (numBytes == sizeof(confirm))
    {
        // Handle byte ordering
        confirm.msgType = ntohs(confirm.msgType);

		// is this the correct message type and is status good?
        if (confirm.msgType == RCRM_GET_OPMODE_CONFIRM) {
			*opMode = ntohl(confirm.opMode);
            retVal = OK;
		}
    }
    return retVal;
}


//_____________________________________________________________________________
//
// rcmSleepModeSet - set RCM sleep mode
//_____________________________________________________________________________

int rcmSleepModeSet(int sleepMode)
{
    rcrmMsg_SetSleepModeRequest request;
    rcrmMsg_SetSleepModeConfirm confirm;
    int retVal = ERR, numBytes;

    // create request message
	request.msgType = htons(RCRM_SET_SLEEP_MODE_REQUEST);
	request.msgId = htons(msgIdCount++);
    request.sleepMode = htonl(sleepMode);

    // make sure no pending messages
    rcmIfFlush();

    // send message to RCM
	rcmIfSendPacket(&request, sizeof(request));

    // wait for response
    numBytes = rcmIfGetPacket(&confirm, sizeof(confirm));

    // did we get a response from the RCM?
    if (numBytes == sizeof(confirm))
    {
        // Handle byte ordering
        confirm.msgType = ntohs(confirm.msgType);
        confirm.status = ntohl(confirm.status);

        // is this the correct message type and is status good?
        if (confirm.msgType == RCRM_SET_SLEEP_MODE_CONFIRM &&
                confirm.status == OK)
            retVal = OK;
    }
    return retVal;
}


//_____________________________________________________________________________
//
// rcmStatusInfoGet - retrieve RCM status from radio
//_____________________________________________________________________________

int rcmStatusInfoGet(rcrmMsg_GetStatusInfoConfirm *confirm)
{
    rcrmMsg_GetStatusInfoRequest request;
    int retVal = ERR, numBytes;

    // create request message
	request.msgType = htons(RCRM_GET_STATUS_INFO_REQUEST);
	request.msgId = htons(msgIdCount++);

    // make sure no pending messages
    rcmIfFlush();

    // send message to RCM
	rcmIfSendPacket(&request, sizeof(request));

    // wait for response
    numBytes = rcmIfGetPacket(confirm, sizeof(rcrmMsg_GetStatusInfoConfirm));

    // did we get a response from the RCM?
    if (numBytes == sizeof(rcrmMsg_GetStatusInfoConfirm))
    {
        // Handle byte ordering
        confirm->msgType = ntohs(confirm->msgType);
        confirm->msgId = ntohs(confirm->msgId);

        // is this the correct message type?
        if (confirm->msgType == RCRM_GET_STATUS_INFO_CONFIRM)
        {
            // Handle byte ordering
            confirm->appVersionBuild = ntohs(confirm->appVersionBuild);
            confirm->uwbKernelVersionBuild = ntohs(confirm->uwbKernelVersionBuild);
            confirm->serialNum = ntohl(confirm->serialNum);
            confirm->temperature = ntohl(confirm->temperature);

            // status code
            confirm->status = ntohl(confirm->status);
            // only return OK if status is OK
            if (confirm->status == OK)
                retVal = OK;
        }
    }
    return retVal;
}


//_____________________________________________________________________________
//
// rcmInfoGet - range to another RCM module
//_____________________________________________________________________________

// Added by Li Jiaxin
int rcmInfoGetFull(rcmMsg_FullRangeInfo *rangeInfo,
               rnMsg_GetFullNeighborDatabaseConfirm *ndbInfo,
               rcmMsg_DataInfo *dataInfo,
               rcmMsg_EchoedRangeInfo *echoedInfo)
{
    infoMsgs_t infoMsgs;
    int retVal = ERR, numBytes;
    int i;

    // clear out caller's info structs
    memset(rangeInfo, 0, sizeof(*rangeInfo));
    memset(ndbInfo, 0, sizeof(*ndbInfo));
    memset(dataInfo, 0, sizeof(*dataInfo));
    memset(echoedInfo, 0, sizeof(*echoedInfo));
    rangeInfo->rangeStatus = RCM_RANGE_STATUS_TIMEOUT;

    // Collect the info messages
    while ((numBytes = rcmIfGetPacket(&infoMsgs, sizeof(infoMsgs))) > 0)
    {
        // Based on the configuration setup, radio should only be able to receive
        //      DATA_INFO messages as well as the RANGE_INFO or NDB_INFO message chosen
        switch(ntohs(infoMsgs.rangeInfo.msgType))
        {
            case RCM_FULL_RANGE_INFO:
                // copy message to caller's struct
                memcpy(rangeInfo, &infoMsgs.rangeInfo, sizeof(*rangeInfo));
                // handle byte ordering
                rangeInfo->msgType = ntohs(rangeInfo->msgType);
                rangeInfo->msgId = ntohs(rangeInfo->msgId);
                rangeInfo->responderId = ntohl(rangeInfo->responderId);
                rangeInfo->stopwatchTime = ntohs(rangeInfo->stopwatchTime);
                rangeInfo->precisionRangeMm = ntohl(rangeInfo->precisionRangeMm);
                rangeInfo->coarseRangeMm = ntohl(rangeInfo->coarseRangeMm);
                rangeInfo->filteredRangeMm = ntohl(rangeInfo->filteredRangeMm);
                rangeInfo->precisionRangeErrEst = ntohs(rangeInfo->precisionRangeErrEst);
                rangeInfo->coarseRangeErrEst = ntohs(rangeInfo->coarseRangeErrEst);
                rangeInfo->filteredRangeErrEst = ntohs(rangeInfo->filteredRangeErrEst);
                rangeInfo->filteredRangeVel = ntohs(rangeInfo->filteredRangeVel);
                rangeInfo->filteredRangeVelErrEst = ntohs(rangeInfo->filteredRangeVelErrEst);
                rangeInfo->reqLEDFlags = ntohs(rangeInfo->reqLEDFlags);
                rangeInfo->respLEDFlags = ntohs(rangeInfo->respLEDFlags);
                rangeInfo->noise = ntohs(rangeInfo->noise);
                rangeInfo->vPeak = ntohs(rangeInfo->vPeak);
                rangeInfo->coarseTOFInBins = ntohl(rangeInfo->coarseTOFInBins);
                rangeInfo->timestamp = ntohl(rangeInfo->timestamp);

                retVal = RANGEINFO;
                break;
            case RN_FULL_NEIGHBOR_DATABASE_INFO:
            case RN_GET_FULL_NEIGHBOR_DATABASE_CONFIRM:
                // copy message to caller's struct
                memcpy(ndbInfo, &infoMsgs.ndbInfo, sizeof(*ndbInfo));
                // handle byte ordering
                ndbInfo->msgType = ntohs(ndbInfo->msgType);
                ndbInfo->msgId = ntohs(ndbInfo->msgId);
                ndbInfo->timestamp = ntohl(ndbInfo->timestamp);
                ndbInfo->status = ntohl(ndbInfo->sortType);
                for (i = 0; i < ndbInfo->numNeighborEntries; i++)
                {
                    ndbInfo->neighbors[i].nodeId = ntohl(ndbInfo->neighbors[i].nodeId);
                    ndbInfo->neighbors[i].stopwatchTime = ntohs(ndbInfo->neighbors[i].stopwatchTime);
                    ndbInfo->neighbors[i].rangeMm = ntohl(ndbInfo->neighbors[i].rangeMm);
                    ndbInfo->neighbors[i].rangeErrorEstimate = ntohs(ndbInfo->neighbors[i].rangeErrorEstimate);
                    ndbInfo->neighbors[i].rangeVelocity = ntohs(ndbInfo->neighbors[i].rangeVelocity);
                    ndbInfo->neighbors[i].ledFlags = ntohs(ndbInfo->neighbors[i].ledFlags);
                    ndbInfo->neighbors[i].noise = ntohs(ndbInfo->neighbors[i].noise);
                    ndbInfo->neighbors[i].vPeak = ntohs(ndbInfo->neighbors[i].vPeak);
                    ndbInfo->neighbors[i].statsNumRangeAttempts = ntohs(ndbInfo->neighbors[i].statsNumRangeAttempts);
                    ndbInfo->neighbors[i].statsNumRangeSuccesses = ntohs(ndbInfo->neighbors[i].statsNumRangeSuccesses);
                    ndbInfo->neighbors[i].statsAgeMs = ntohl(ndbInfo->neighbors[i].statsAgeMs);
                    ndbInfo->neighbors[i].rangeUpdateTimestampMs = ntohl(ndbInfo->neighbors[i].rangeUpdateTimestampMs);
                    ndbInfo->neighbors[i].lastHeardTimestampMs = ntohl(ndbInfo->neighbors[i].lastHeardTimestampMs);
                    ndbInfo->neighbors[i].addedToNDBTimestampMs = ntohl(ndbInfo->neighbors[i].addedToNDBTimestampMs);
                }
                retVal = FULLNDB;
                break;
            case RCM_DATA_INFO:
                // Ignore data info messages
                // copy message to caller's struct
                memcpy(dataInfo, &infoMsgs.dataInfo, sizeof(*dataInfo));
                // handle byte ordering
                dataInfo->msgType = ntohs(dataInfo->msgType);
                dataInfo->msgId = ntohs(dataInfo->msgId);
                dataInfo->sourceId = ntohl(dataInfo->sourceId);
                dataInfo->noise = ntohs(dataInfo->noise);
                dataInfo->vPeak = ntohs(dataInfo->vPeak);
                dataInfo->timestamp = ntohl(dataInfo->timestamp);
                dataInfo->dataSize = ntohs(dataInfo->dataSize);

                retVal = DATAINFO;
                break;
            case RCM_ECHOED_RANGE_INFO:
                // Ignore data info messages
                // copy message to caller's struct
                memcpy(echoedInfo, &infoMsgs.echoedInfo, sizeof(*echoedInfo));
                // handle byte ordering
                echoedInfo->msgType = ntohs(echoedInfo->msgType);
                echoedInfo->msgId = ntohs(echoedInfo->msgId);
                echoedInfo->requesterId = ntohl(echoedInfo->requesterId);
                echoedInfo->responderId = ntohl(echoedInfo->responderId);
                echoedInfo->precisionRangeMm = ntohl(echoedInfo->precisionRangeMm);
                echoedInfo->precisionRangeErrEst = ntohs(echoedInfo->precisionRangeErrEst);
                echoedInfo->ledFlags = ntohs(echoedInfo->ledFlags);
                echoedInfo->timestamp = ntohl(echoedInfo->timestamp);

                retVal = ECHOEDINFO;
                break;
            case RCM_SEND_DATA_CONFIRM:
                break;
            default:
                printf("\nReceived Unknown Message Type: 0x%X\n", ntohs(infoMsgs.rangeInfo.msgType));
                break;
        }
        // Keep processing unless we got desired message
        if (retVal > 0)
            break;
    }
    return retVal;
}


int rcmInfoGet(rcmMsg_FullRangeInfo *rangeInfo, rnMsg_GetFullNeighborDatabaseConfirm *ndbInfo)
{
    infoMsgs_t infoMsgs;
    int retVal = ERR, numBytes;
	int i;

    // clear out caller's info structs
    memset(rangeInfo, 0, sizeof(*rangeInfo));
    memset(ndbInfo, 0, sizeof(*ndbInfo));
    rangeInfo->rangeStatus = RCM_RANGE_STATUS_TIMEOUT;

	// Collect the info messages
    while ((numBytes = rcmIfGetPacket(&infoMsgs, sizeof(infoMsgs))) > 0)
    {
		// Based on the configuration setup, radio should only be able to receive
		//      DATA_INFO messages as well as the RANGE_INFO or NDB_INFO message chosen
        switch(ntohs(infoMsgs.rangeInfo.msgType))
        {
            case RCM_FULL_RANGE_INFO:
                // copy message to caller's struct
                memcpy(rangeInfo, &infoMsgs.rangeInfo, sizeof(*rangeInfo));
                // handle byte ordering
                rangeInfo->msgType = ntohs(rangeInfo->msgType);
                rangeInfo->msgId = ntohs(rangeInfo->msgId);
                rangeInfo->responderId = ntohl(rangeInfo->responderId);
                rangeInfo->stopwatchTime = ntohs(rangeInfo->stopwatchTime);
                rangeInfo->precisionRangeMm = ntohl(rangeInfo->precisionRangeMm);
                rangeInfo->coarseRangeMm = ntohl(rangeInfo->coarseRangeMm);
                rangeInfo->filteredRangeMm = ntohl(rangeInfo->filteredRangeMm);
                rangeInfo->precisionRangeErrEst = ntohs(rangeInfo->precisionRangeErrEst);
                rangeInfo->coarseRangeErrEst = ntohs(rangeInfo->coarseRangeErrEst);
                rangeInfo->filteredRangeErrEst = ntohs(rangeInfo->filteredRangeErrEst);
                rangeInfo->filteredRangeVel = ntohs(rangeInfo->filteredRangeVel);
                rangeInfo->filteredRangeVelErrEst = ntohs(rangeInfo->filteredRangeVelErrEst);
                rangeInfo->reqLEDFlags = ntohs(rangeInfo->reqLEDFlags);
                rangeInfo->respLEDFlags = ntohs(rangeInfo->respLEDFlags);
                rangeInfo->noise = ntohs(rangeInfo->noise);
                rangeInfo->vPeak = ntohs(rangeInfo->vPeak);
                rangeInfo->coarseTOFInBins = ntohl(rangeInfo->coarseTOFInBins);
                rangeInfo->timestamp = ntohl(rangeInfo->timestamp);

				retVal = RANGEINFO;
                break;
            case RCM_DATA_INFO:
                // Ignore data info messages
				printf("\nReceived RCM_DATA_INFO. Ignoring...\n");
                break;
			case RN_FULL_NEIGHBOR_DATABASE_INFO:
			case RN_GET_FULL_NEIGHBOR_DATABASE_CONFIRM:
                // copy message to caller's struct
                memcpy(ndbInfo, &infoMsgs.ndbInfo, sizeof(*ndbInfo));
                // handle byte ordering
				ndbInfo->msgType = ntohs(ndbInfo->msgType);
				ndbInfo->msgId = ntohs(ndbInfo->msgId);
				ndbInfo->timestamp = ntohl(ndbInfo->timestamp);
				ndbInfo->status = ntohl(ndbInfo->sortType);
				for (i = 0; i < ndbInfo->numNeighborEntries; i++)
				{
					ndbInfo->neighbors[i].nodeId = ntohl(ndbInfo->neighbors[i].nodeId);
					ndbInfo->neighbors[i].stopwatchTime = ntohs(ndbInfo->neighbors[i].stopwatchTime);
					ndbInfo->neighbors[i].rangeMm = ntohl(ndbInfo->neighbors[i].rangeMm);
					ndbInfo->neighbors[i].rangeErrorEstimate = ntohs(ndbInfo->neighbors[i].rangeErrorEstimate);
					ndbInfo->neighbors[i].rangeVelocity = ntohs(ndbInfo->neighbors[i].rangeVelocity);
					ndbInfo->neighbors[i].ledFlags = ntohs(ndbInfo->neighbors[i].ledFlags);
					ndbInfo->neighbors[i].noise = ntohs(ndbInfo->neighbors[i].noise);
					ndbInfo->neighbors[i].vPeak = ntohs(ndbInfo->neighbors[i].vPeak);
					ndbInfo->neighbors[i].statsNumRangeAttempts = ntohs(ndbInfo->neighbors[i].statsNumRangeAttempts);
					ndbInfo->neighbors[i].statsNumRangeSuccesses = ntohs(ndbInfo->neighbors[i].statsNumRangeSuccesses);
					ndbInfo->neighbors[i].statsAgeMs = ntohl(ndbInfo->neighbors[i].statsAgeMs);
					ndbInfo->neighbors[i].rangeUpdateTimestampMs = ntohl(ndbInfo->neighbors[i].rangeUpdateTimestampMs);
					ndbInfo->neighbors[i].lastHeardTimestampMs = ntohl(ndbInfo->neighbors[i].lastHeardTimestampMs);
					ndbInfo->neighbors[i].addedToNDBTimestampMs = ntohl(ndbInfo->neighbors[i].addedToNDBTimestampMs);
				}
				retVal = FULLNDB;
				break;
			default:
				printf("\nReceived Unknown Message Type: 0x%X\n", ntohs(infoMsgs.rangeInfo.msgType));
				break;
        }	
		// Keep processing unless we got desired message
		if (retVal > 0)
			break;
    }
    return retVal;
}


//_____________________________________________________________________________
//
// rcmDataSend - broadcast a data-only packet
//_____________________________________________________________________________

int rcmDataSend(int antennaMode, int dataSize, unsigned char *data)
{
    rcmMsg_SendDataRequest request;
    rcmMsg_SendDataConfirm confirm;
    int retVal = ERR, numBytes;

    // create request message
	request.msgType = htons(RCM_SEND_DATA_REQUEST);
	request.msgId = htons(msgIdCount++);
    request.antennaMode = antennaMode;
    request.dataSize = htons(dataSize);
    // make sure there isn't too much data
    if (dataSize > RCM_USER_DATA_LENGTH)
        dataSize = RCM_USER_DATA_LENGTH;
    // copy data into message
    memcpy(request.data, data, dataSize);

    // make sure no pending messages
    // this will cause dropout, Li Jiaxin
//    rcmIfFlush();

    // send message to RCM
    numBytes = sizeof(request) - RCM_USER_DATA_LENGTH + dataSize;
	rcmIfSendPacket(&request, sizeof(request));

    // by Li Jiaxin, don't wait for response, otherwise it will block the ranging process.
    // the ranging process, i.e. rcmInfoGetFull, relies on the same rcmIfGetPacket.

    /*
    // wait for response
    numBytes = rcmIfGetPacket(&confirm, sizeof(confirm));

    // did we get a response from the RCM?
    if (numBytes == sizeof(confirm))
    {
        // Handle byte ordering
        confirm.msgType = ntohs(confirm.msgType);
        confirm.msgId = ntohs(confirm.msgId);

        // is this the correct message type?
        if (confirm.msgType == RCM_SEND_DATA_CONFIRM)
        {
            // status code
            confirm.status = ntohl(confirm.status);
            // only return OK if status is OK
            if (confirm.status == OK)
            {
                retVal = OK;
            }
        }
    }
    return retVal;
    */

    return OK;

}
