//_____________________________________________________________________________
//
// Copyright 2011-2015 Time Domain Corporation
//
//
// rcmIf.c
//
//   Functions to communicate over Ethernet, serial, or USB to an RCM
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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#endif

#include "hostInterfaceCommon.h"
#include "rcmIf.h"


//_____________________________________________________________________________
//
// #defines 
//_____________________________________________________________________________

#define DEFAULT_TIMEOUT_MS  500

//_____________________________________________________________________________
//
// static data
//_____________________________________________________________________________

#ifdef WIN32
static HANDLE hComm;
#endif
static int                  radioFd;
static struct sockaddr_in   radioAddr;
static rcmIfType            rcmIf;
static int timeoutMs = DEFAULT_TIMEOUT_MS;

//_____________________________________________________________________________
//
// Private function prototypes 
//_____________________________________________________________________________

static int rcmIfGetPacketIp(void *pkt, unsigned maxSize);
static int rcmIfGetPacketSerial(void *pkt, unsigned maxSize);
static int rcmIfSendPacketIp(void *pkt, unsigned size);
static int rcmIfSendPacketSerial(void *pkt, unsigned size);
static unsigned short crc16(void *buf, int len);


//_____________________________________________________________________________
//
// rcmIfInit - perform initialization
//_____________________________________________________________________________

int rcmIfInit(rcmIfType ifType, const char *destAddr)
{
    unsigned radioIpAddr;

    switch (ifType)
    {
        case rcmIfIp:
#ifdef WIN32
            {
                // Initialize Windows sockets
                WSADATA wsad;
                memset(&wsad, 0, sizeof(wsad));
                WSAStartup(MAKEWORD(2, 2), &wsad);
            }
#endif

            // convert from string to binary
            radioIpAddr = inet_addr(destAddr);

            // make sure IP address is valid
            if (radioIpAddr == INADDR_NONE)
            {
                printf("Invalid IP address.\n");
                return ERR;
            }

            // create UDP socket
            radioFd = (int)socket(AF_INET, SOCK_DGRAM, 0);
            if (radioFd == -1)
            {
                printf("Unable to open socket");
                return ERR;
            }

            // initialize radio address structure
            memset(&radioAddr, 0, sizeof(radioAddr));
            radioAddr.sin_family = AF_INET;
            radioAddr.sin_port = htons(RCRM_SOCKET_PORT_NUM);
            radioAddr.sin_addr.s_addr = radioIpAddr;
            break;

        case rcmIfSerial:
        case rcmIfUsb:
#ifdef WIN32
            {
                DCB dcbSerialParams = {0};
                wchar_t wcs[100];
                char comPortStr[100];

                // Windows requirement for COM ports above 9
                // but works for all
                sprintf(comPortStr, "\\\\.\\%s", destAddr);
                mbstowcs(wcs, comPortStr, sizeof(wcs));
                hComm = CreateFile(wcs,
                        GENERIC_READ | GENERIC_WRITE,
                        0,
                        0,
                        OPEN_EXISTING,
                        FILE_ATTRIBUTE_NORMAL,
                        0);
                if (hComm == INVALID_HANDLE_VALUE)
                {
                    printf("Can't open serial port\n");
                    return ERR;
                }

                dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
                GetCommState(hComm, &dcbSerialParams);
                dcbSerialParams.BaudRate=CBR_115200;
                dcbSerialParams.ByteSize=8;
                dcbSerialParams.StopBits=ONESTOPBIT;
                dcbSerialParams.Parity=NOPARITY;
                SetCommState(hComm, &dcbSerialParams);
            }
#else
            {
                char portStr[100];
                struct termios term;

                if(ifType == rcmIfUsb)
		{
		    // P400s use this naming scheme
                    //sprintf(portStr, "/dev/serial/by-id/usb-Time_Domain_P400_Radio_%s-if00", destAddr);
		  
		    // P440s use this naming scheme
                    sprintf(portStr, "/dev/serial/by-id/usb-1027_Time_Domain_P440_%s-if00", destAddr);
                    
		    radioFd = open(portStr, O_RDWR|O_NOCTTY|O_NONBLOCK);
                }
                else
                {
                    strcpy(portStr, destAddr);
                }

                radioFd = open(portStr, O_RDWR|O_NOCTTY|O_NONBLOCK);
                if (radioFd < 0)
                {
                    printf("Can't open serial port: %s\n", portStr);
                    return ERR;
                }
                tcgetattr(radioFd, &term);

                memset(&term, 0, sizeof(term));
                cfmakeraw(&term);
                term.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
                term.c_iflag = IGNPAR;
                                                
                tcflush(radioFd, TCIFLUSH);

                tcsetattr(radioFd, TCSANOW, &term);
            }
#endif
            break;

        default:
            printf("Unknown interface type.\n");
            exit(-1);
            break;
    }
    rcmIf = ifType;

    return OK;
}


//_____________________________________________________________________________
//
// rcmIfClose - perform cleanup
//_____________________________________________________________________________

void rcmIfClose(void)
{
    switch (rcmIf)
    {
        case rcmIfIp:
#ifdef WIN32
            // windows cleanup code
            closesocket(radioFd);
	        WSACleanup();
#else
            // Linux cleanup code
            close(radioFd);
#endif
            break;

        case rcmIfSerial:
        case rcmIfUsb:
#ifdef WIN32
            CloseHandle(hComm);
#else
            close(radioFd);
#endif
            break;

        default:
            printf("Unknown interface type.\n");
            exit(-1);
            break;
    }
}


//_____________________________________________________________________________
//
// rcmIfGetPacket - return RCM packet
//
// Returns number of bytes received, ERR if timeout
//_____________________________________________________________________________

int rcmIfGetPacket(void *pkt, unsigned maxSize)
{
    if (rcmIf == rcmIfIp)
        return rcmIfGetPacketIp(pkt, maxSize);
    else
        return rcmIfGetPacketSerial(pkt, maxSize);
}


//_____________________________________________________________________________
//
// rcmIfSendPacket - send packet out RCM interface
//
// Number of bytes sent, ERR on error.
//_____________________________________________________________________________

int rcmIfSendPacket(void *pkt, unsigned size)
{
    if (rcmIf == rcmIfIp)
        return rcmIfSendPacketIp(pkt, size);
    else
        return rcmIfSendPacketSerial(pkt, size);
}


//_____________________________________________________________________________
//
// rcmIfFlush - flush any pending messages
//
//_____________________________________________________________________________

void rcmIfFlush(void)
{
    int tmp, i = timeoutMs;

    timeoutMs = 0;
    while (rcmIfGetPacket(&tmp, sizeof(tmp)) > 0)
        ;
    timeoutMs = i;
}


//_____________________________________________________________________________
//
// Private functions
//_____________________________________________________________________________


//_____________________________________________________________________________
//
// rcmIfGetPacketIp - read a packet from RCM using UDP
//
//_____________________________________________________________________________

int rcmIfGetPacketIp(void *pkt, unsigned maxSize)
{
	fd_set fds;
	struct timeval tv;

    // basic select call setup
	FD_ZERO(&fds);
	FD_SET(radioFd, &fds);

	// Set up timeout
	tv.tv_sec = timeoutMs / 1000;
	tv.tv_usec = (timeoutMs * 1000) % 1000000;

	if (select(radioFd + 1, &fds, NULL, NULL, &tv) > 0)
	{
		// copy packet into buffer
		return recvfrom(radioFd, (char *)pkt, maxSize, 0, NULL, NULL);
	}

	// Timeout
	return ERR;
}


//_____________________________________________________________________________
//
// rcmIfSendPacketIp - send a packet to the RCM using UDP
//
//_____________________________________________________________________________

int rcmIfSendPacketIp(void *pkt, unsigned size)
{
	return sendto(radioFd, (const char *)pkt, size, 0,
            (struct sockaddr *)&radioAddr, sizeof(radioAddr));
}


//_____________________________________________________________________________
//
// serTimedRead - read from serial port with timeout
//
//_____________________________________________________________________________

int serTimedRead(void *buf, int cnt)
{
#ifdef WIN32
    DWORD dwBytes = 0;
    COMMTIMEOUTS timeouts={0};

    if (timeoutMs == 0)
    {
        // windows way to specify zero timeout
        timeouts.ReadIntervalTimeout = MAXDWORD;
        timeouts.ReadTotalTimeoutConstant = 0;
        timeouts.ReadTotalTimeoutMultiplier = 0;
    }
    else
    {
        timeouts.ReadIntervalTimeout = timeoutMs;
        timeouts.ReadTotalTimeoutConstant = timeoutMs;
        timeouts.ReadTotalTimeoutMultiplier = 0;
    }
    timeouts.WriteTotalTimeoutConstant = timeoutMs;
    timeouts.WriteTotalTimeoutMultiplier = 0;

    SetCommTimeouts(hComm, &timeouts);

    if (!ReadFile(hComm, buf, cnt, &dwBytes, NULL))
        return ERR;

    return dwBytes;
#else
	fd_set fds;
	struct timeval tv;
    int total = 0, i;
    char *ptr = (char *)buf;

    while (cnt)
    {
        // basic select call setup
	    FD_ZERO(&fds);
	    FD_SET(radioFd, &fds);

        // Set up timeout
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs * 1000) % 1000000;

        if (select(radioFd + 1, &fds, NULL, NULL, &tv) > 0)
        {
            i = read(radioFd, ptr, cnt);
            if (i > 0)
            {
                cnt -= i;
                ptr += i;
                total += i;
            }
        }
        else
        {
            total = -1;
            break;
        }
    }

	return total;
#endif
}


//_____________________________________________________________________________
//
// serWrite - write data to serial port
//
//_____________________________________________________________________________

int serWrite(void *buf, int cnt)
{
#ifdef WIN32
    DWORD dwBytes = 0;

    if (!WriteFile(hComm, buf, cnt, &dwBytes, NULL))
        return ERR;

    return dwBytes;
//error occurred. Report to user.
#else
    return write(radioFd, buf, cnt);
#endif
}


//_____________________________________________________________________________
//
// rcmIfGetPacketSerial - read packet from serial port
//
//_____________________________________________________________________________

int rcmIfGetPacketSerial(void *pkt, unsigned maxSize)
{
    int c=0, i, crc;
    unsigned short val;

    while (1)
    {
        // read first sync byte
        if (serTimedRead(&c, 1) <= 0)
            return ERR;
        if (c != 0xa5)
            continue;

        // read second sync byte
        if (serTimedRead(&c, 1) <= 0)
            return ERR;
        if (c != 0xa5)
            continue;

        break;
    }

    // read size
    if (serTimedRead(&val, sizeof(val)) <= 0)
        return ERR;
    val = ntohs(val);

    // read packet
    if (val > maxSize)
        val = maxSize;
    i = serTimedRead((char *)pkt, val);

    // read crc for serial port
    if (rcmIf == rcmIfSerial)
    {
        if (serTimedRead(&crc, 2) != 2)
            return ERR;
        crc = ntohs(crc);
        if (crc != crc16(pkt, val))
        {
            printf("CRC error\n");
            return ERR;
        }
    }
    return i;
}


//_____________________________________________________________________________
//
// rcmIfSendPacketSerial - write packet to serial port
//
//_____________________________________________________________________________

int rcmIfSendPacketSerial(void *pkt, unsigned size)
{
    int i;
    unsigned short val;

    // send sync bytes
    i = serWrite("\xa5\xa5", 2);
    if (i < 0)
        return ERR;

    // send size bytes
    val = htons(size);
    i = serWrite(&val, sizeof(val));
    if (i < 0)
        return ERR;

    // send packet
    i = serWrite(pkt, size);
    if (i < 0)
        return ERR;

    // send CRC for serial port
    if (rcmIf == rcmIfSerial)
    {
        val = crc16(pkt, size);
        val = htons(val);
        if (serWrite(&val, sizeof(val)) < 0)
            return ERR;
    }
    return OK;
}



// Table of CRC constants - implements x^16+x^12+x^5+1
static const unsigned short crc16_tab[] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0, 
};


//_____________________________________________________________________________
//
// crc16 - calculate 16-bit CRC on buffer
//
//_____________________________________________________________________________

unsigned short crc16(void *ptr, int len)
{
    int i;
    unsigned short cksum;
    unsigned char *buf = (unsigned char *)ptr;

    cksum = 0;
    for (i = 0;  i < len;  i++) {
        cksum = crc16_tab[((cksum>>8) ^ *buf++) & 0xFF] ^ (cksum << 8);
    }
    return cksum;
}

