//_____________________________________________________________________________
//
// Copyright 2011-2 Time Domain Corporation
//
//
// rcmIf.h
//
//   Declarations for RCM interface functions.
//
//_____________________________________________________________________________

#ifndef __rcmIf_h
#define __rcmIf_h

#ifdef __cplusplus
    extern "C" {
#endif

//_____________________________________________________________________________
//
// #includes
//_____________________________________________________________________________



//_____________________________________________________________________________
//
// #defines
//_____________________________________________________________________________

#ifndef OK
#define OK 0
#define ERR (-1)
#endif

#ifndef RANGEINFO
#define RANGEINFO 1
#endif

#ifndef FULLNDB
#define FULLNDB 2
#endif

#ifndef DATAINFO
#define DATAINFO 3
#endif

#ifndef ECHOEDINFO
#define ECHOEDINFO 4
#endif

//_____________________________________________________________________________
//
// typedefs
//_____________________________________________________________________________


typedef enum {rcmIfIp, rcmIfSerial, rcmIfUsb} rcmIfType;

//_____________________________________________________________________________
//
//  Function prototypes
//_____________________________________________________________________________


//
//  rcmIfInit
//
//  Parameters:  rcmIfType ifType - type of connection to RCM
//               char *destAddr - IP address or serial/USB port name
//  Return:      OK or ERR
//
//  Performs initialization necessary for particular type of interface.
//  Returns ERR on failure.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rcmIfInit(rcmIfType ifType, const char *destAddr);


//
//  rcmIfClose
//
//  Parameters:  void
//  Return:      void
//
//  Closes socket or port to radio.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void rcmIfClose(void);


//
//  rcmIfGetPacket
//
//  Parameters:  void *pkt - pointer to location to receive packet
//               unsigned maxSize - max size of packet to receive
//  Return:      OK or ERR
//
//  Reads from RCM interface until up to maxSize bytes have been received.
//  Returns ERR on read error, otherwise OK.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rcmIfGetPacket(void *pkt, unsigned maxSize);


//
//  rcmIfSendPacket
//
//  Parameters:  void *pkt - pointer to packet to send
//               unsigned size - size of packet to send
//  Return:      OK or ERR
//
//  Sends packet to RCM interface.
//  Returns ERR on write error, otherwise OK.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int rcmIfSendPacket(void *pkt, unsigned size);


//
//  rcmIfFlush
//
//  Parameters:  void
//
//  Return:      void
//
//  Flushes any unread packets.
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void rcmIfFlush(void);

#ifdef __cplusplus
    }
#endif


#endif
