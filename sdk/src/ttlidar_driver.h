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


#include "tttypes.h"
#include "ttlidar_protocol.h"
#include "ttlidar.h"

namespace ttlidar {

typedef struct _lidar_scan_point {
	_u8			signal;	
	float		angle;			
	float		distance;	
	float		motorspeed;
	float		angleoffset;
} __attribute__((packed)) LIDAR_SCAN_INFO_T;


class  TTlidarDriver
{
  

public:

	enum {
        DEFAULT_TIMEOUT = 300, 
    };

	RUNNING_STATE RunningStates;

    TTlidarDriver();
    virtual ~TTlidarDriver();

public:
    virtual result_t connect(const char * port_path, _u32 baudrate);
    virtual void 	 disconnect();
    virtual bool     isConnected();
    virtual result_t resetlidar(_u32 timeout = DEFAULT_TIMEOUT);
	
	virtual result_t startScan(_u8 scanmode = SCAN_MODE,_u32 timeout = DEFAULT_TIMEOUT);
    virtual result_t stopScan(_u32 timeout = DEFAULT_TIMEOUT);
	virtual result_t getDeviceInfo(LIDAR_DEV_INFO_T *info, _u32 timeout = DEFAULT_TIMEOUT);
	virtual result_t setMotorRpm(_u16 speed, _u32 timeout = DEFAULT_TIMEOUT);

	virtual result_t grabScanData(LIDAR_SCAN_INFO_T * scanBuff, size_t & count, _u32 timeout = DEFAULT_TIMEOUT);


protected:
	
	result_t _recvPacket(_u8 * node, _u32 timeout);
	result_t _parsePacket(_u8 *recvBuff, LIDAR_SCAN_INFO_T *scanBuff ,size_t & count);
	result_t _recvScanData(LIDAR_SCAN_INFO_T * scanBuff, size_t & count, _u32 timeout = DEFAULT_TIMEOUT);
	result_t _recvScanDataThreadCallback();
	

    result_t _sendCommand(_u8 cmd, const void * payload = NULL, size_t payloadsize = 0);
	result_t _waitforResponse(FRAME_T * header, _u8 Cmmd,const void * payload, size_t payloadsize, _u32 timeout);

	
	uint16_t _checksum(uint8_t *startByte, uint16_t numBytes);
	uint16_t _cyclechecksum(uint8_t *startByte, _u32 pos, uint16_t numBytes, _u32 buffersize);

    void     stopGrabingScanData();


private:

    bool     					_is_connected;
	bool     					_is_scanning;
	bool     					_is_getdata;


	Locker         				_lock;
    Event          				_event;
	Serial        				*_serial;

	_u8							_flag_scan_buffer;
	LIDAR_SCAN_INFO_T			_scan_buff_left[2048];
	LIDAR_SCAN_INFO_T			_scan_buff_right[2048];
    size_t						_scan_buff_count;
	Thread 						_scanthread;

	_u8 						_resp_buff[1024];

};


}
