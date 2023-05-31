/*
*  TTldiar System
*  Driver Interface
*
*  Copyright 2021 TT Team
*  All rights reserved.
*
*	Author: Titans  2021-2-21
*
*/
#pragma once

#include "serial.h"
#include  "winnt.h"

namespace ttlidar {

class raw_serial : public ttlidar::Serial
{
public:
    enum{
        SERIAL_RX_BUFFER_SIZE = 512,
        SERIAL_TX_BUFFER_SIZE = 128,
        SERIAL_RX_TIMEOUT     = 2000,
        SERIAL_TX_TIMEOUT     = 2000,
    };

    raw_serial();
    virtual ~raw_serial();
    virtual bool bind(const char * portname, _u32 baudrate, _u32 flags = 0);
    virtual bool open();
    virtual void close();
	virtual void flush(_u32 flags);
    
    virtual int waitfordata(size_t data_count,_u32 timeout = -1, size_t * returned_size = NULL);

    virtual int senddata(const unsigned char * data, size_t size);
    virtual int recvdata(unsigned char * data, size_t size);

    virtual int waitforsent(_u32 timeout = -1, size_t * returned_size = NULL);
    virtual int waitforrecv(_u32 timeout = -1, size_t * returned_size = NULL);

    virtual size_t rxqueue_count();


protected:
    bool open(const char * portname, _u32 baudrate, _u32 flags);
    void _init();

    char _portName[20];
    uint32_t _baudrate;
    uint32_t _flags;

    OVERLAPPED _ro, _wo;
    OVERLAPPED _wait_o;
    volatile HANDLE _serial_handle;
    DCB _dcb;
    COMMTIMEOUTS _co;
};

}
