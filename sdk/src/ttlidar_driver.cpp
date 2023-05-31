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


#include "ttlidar_driver.h"

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

namespace ttlidar {


TTlidarDriver::TTlidarDriver(): _is_connected(false), _is_scanning(false)
{
    _serial = Serial::Create();
	_flag_scan_buffer = FLAG_BUFFER_LEFT;
	memset(_scan_buff_left, 0, sizeof(_scan_buff_left));
	memset(_scan_buff_right, 0, sizeof(_scan_buff_right));
    _scan_buff_count = 0;
	RunningStates = PORT_RDY;
}

TTlidarDriver::~TTlidarDriver()
{
    disconnect();
	Serial::Release(_serial);
}

result_t TTlidarDriver::connect(const char * port_path, _u32 baudrate)
{
     AutoLocker l(_lock);
    if (isConnected()) return RESULT_ALREADY_DONE;

    if (!_serial) return RESULT_INSUFFICIENT_MEMORY;

    // establish the serial connection...
    if (!_serial->bind(port_path, baudrate)  ||  !_serial->open()) {
        return RESULT_INVALID_DATA;
    }

    _serial->flush(0);
    _is_connected = true;

	_is_getdata = true;
    return RESULT_OK;
}

void TTlidarDriver::disconnect()
{
    if (!_is_connected) return ;
	stopScan();
    _serial->close();
}

bool TTlidarDriver::isConnected()
{
    return _is_connected;
}



result_t TTlidarDriver::resetlidar(_u32 timeout)
{
	_u8 param = 0x08;
    result_t result;
	FRAME_T *response = reinterpret_cast<FRAME_T *>(_resp_buff);

	if (!isConnected()) return RESULT_OPERATION_FAIL;

	stopGrabingScanData();

	{
		 AutoLocker l(_lock);
		param = RESET_MODE;
		if (IS_FAIL(result = _sendCommand(CMD_SET_WORK_MODE, &param, 1))) {
			return result;
		}

		if (IS_FAIL(result = _waitforResponse(response, CMD_SET_WORK_MODE, NULL,1, timeout))) {
			return result;
		}

		if ((response->command&FRAME_ERROR) > 0)
		{
			return *(response->paramBuff);
		}
	}

	return RESULT_OK;
}

result_t TTlidarDriver::setMotorRpm(_u16 speed, _u32 timeout)
{
	_u8 Data[] = { 0x23,0x01,0x67,0x45,0xAB,0x89,0xEF,0xCD,0x00,0x00};
	result_t result;
	FRAME_T *response = reinterpret_cast<FRAME_T *>(_resp_buff);

	Data[8] = speed;
	Data[9] = speed >> 8;

	if (!isConnected()) return RESULT_OPERATION_FAIL;

	//stopGrabingScanData();

	{
		// AutoLocker l(_lock);

		if (IS_FAIL(result = _sendCommand(CMD_SET_MOTOR_SPEED, &Data, 10))) {
			return result;
		}

		if (IS_FAIL(result = _waitforResponse(response, CMD_SET_MOTOR_SPEED, NULL,1, timeout))) {
			return result;
		}

		if ((response->command&FRAME_ERROR) > 0)
		{
			return (*(response->paramBuff));
		}
	}

	return RESULT_OK;
}

result_t TTlidarDriver::getDeviceInfo(LIDAR_DEV_INFO_T * info, _u32 timeout)
{
	result_t  result;
	FRAME_T *response = reinterpret_cast<FRAME_T *>(_resp_buff);

	if (!isConnected()) return RESULT_OPERATION_FAIL;

	stopGrabingScanData();

	{
		 AutoLocker l(_lock);
		if (IS_FAIL(result = _sendCommand(CMD_READ_DEV_VER, NULL, 0))) {
			return result;
		}

		if (IS_FAIL(result = _waitforResponse(response, CMD_READ_DEV_VER, NULL,45, timeout))) {
			return result;
		}

		if ((response->command&FRAME_ERROR) > 0)
		{
			return (*(response->paramBuff));
		}

		memcpy(info, response->paramBuff, response->paramLen);
	}
	return RESULT_OK;
}

result_t TTlidarDriver::startScan(_u8 scanmode,_u32 timeout)
{
	_u8 param = scanmode;
	result_t result;
	FRAME_T *response = reinterpret_cast<FRAME_T *>(_resp_buff);

	if (!isConnected()) return RESULT_OPERATION_FAIL;
	if (_is_scanning) return RESULT_ALREADY_DONE;

	{
		 AutoLocker l(_lock);
		_serial->flush(0);
		if (IS_FAIL(result = _sendCommand(CMD_SET_WORK_MODE, &param, 1))) {
			return result;
		}
		
		/*if (IS_FAIL(result = _waitforResponse(response, CMD_WORK_MODE, NULL, 1, timeout))) {
			return result;
		}
	
		if ((response->command&FRAME_ERROR) > 0)
		{
			return (*(response->paramBuff));
		}
		*/
		_is_scanning = true;
		_scanthread = CLASS_THREAD(TTlidarDriver, _recvScanDataThreadCallback);
	}

	return RESULT_OK;
}


result_t TTlidarDriver::stopScan(_u32 timeout)
{
	_u8 param = 0x00;
	result_t result;
	FRAME_T *response = reinterpret_cast<FRAME_T *>(_resp_buff);

	if (!isConnected()) return RESULT_OPERATION_FAIL;

	stopGrabingScanData();

	{
		 AutoLocker l(_lock);

		if (IS_FAIL(result = _sendCommand(CMD_SET_WORK_MODE,&param,1))) {
			return result;
		}

		if (IS_FAIL(result = _waitforResponse(response, CMD_SET_WORK_MODE, NULL,1, timeout))) {
			return result;
		}

		if ((response->command&FRAME_ERROR) > 0)
		{
			return (*(response->paramBuff));
		}
	}

	return RESULT_OK;
}

result_t TTlidarDriver::_recvScanDataThreadCallback()
{
	LIDAR_SCAN_INFO_T				*scanBuff = NULL;
	size_t                          count = 0;
	size_t                          scanCount = 0;
	result_t                        result;

	_flag_scan_buffer = FLAG_BUFFER_LEFT;
	scanBuff = _scan_buff_right;

	while (_is_scanning)
	{
		if (IS_FAIL(result = _recvScanData(scanBuff, count))) {
			if (!((result == RESULT_OPERATION_TIMEOUT) || (result == RESULT_RECEIVE_NODE_ERROR))) {
				_is_scanning = false;
				return RESULT_OPERATION_FAIL;
			}
		}

		_lock.lock();
		_flag_scan_buffer = _flag_scan_buffer ^ 0x01;
		if (FLAG_BUFFER_LEFT == _flag_scan_buffer)
		{
			scanBuff = _scan_buff_right;
		}
		else if (FLAG_BUFFER_RIGHT == _flag_scan_buffer)
		{
			scanBuff = _scan_buff_left;
		}
		else{}
		_scan_buff_count = count;
		_event.set();
		_lock.unlock();

	}
	_is_scanning = false;
	return RESULT_OK;
}

result_t TTlidarDriver::_recvScanData(LIDAR_SCAN_INFO_T * scanBuff, size_t & count, _u32 timeout)
{
	_u8		 	recvBuff[1024];
	_u32     	startTime = getms();
	_u32     	waitTime;
	result_t 	result;
	
	if (!_is_connected) 
		return RESULT_OPERATION_FAIL;
	
	while ((waitTime = getms() - startTime) <= timeout ) 
	{
		if (IS_FAIL(result = _recvPacket(recvBuff, timeout - waitTime))) 
			return result;
		
		result = _parsePacket(recvBuff,scanBuff,count);
		if(result == RESULT_OK)
			return RESULT_OK;
					
		if(result == RESULT_CONTINUE)
			continue;
	}

	count = 0;
	printf("_recvScanData() timeout!!!!\n");
	return RESULT_OPERATION_TIMEOUT;
}

result_t TTlidarDriver::_parsePacket(_u8 *recvBuff, LIDAR_SCAN_INFO_T *scanBuff ,size_t & count)
{
	_u16	paramLen = 0;
	float	startAngle = 0;
	float	endAngle = 0;
	float	angleStep = 0;
	static  float lastStartAngle = 0;
	float   speed;
	float   angleoffset = 0;
	bool	newPacketFlag = false;
	_u16	crcCheckNum = 0,calcchecksum;
	_u16	scanNum = 0;
	static size_t scanCount =0 ,packetCount = 0;
	

	paramLen = (recvBuff[3] << 8) | recvBuff[2];
	crcCheckNum = (recvBuff[5] << 8) | recvBuff[4];
	calcchecksum = _checksum(recvBuff, paramLen);
	
	if (crcCheckNum == calcchecksum)
	{		
		newPacketFlag = true;	

		scanNum = (paramLen - 7) / 2; //20230426
		speed = static_cast<float>((recvBuff[9] << 8) | recvBuff[8]) * 0.01;
		startAngle = static_cast<float>((recvBuff[11] << 8) | recvBuff[10]) * 0.01;
		endAngle = static_cast<float>((recvBuff[13] << 8) | recvBuff[12]) * 0.01;
		angleStep = (endAngle - startAngle) / (scanNum - 1);
		

		if (startAngle >= 0 && endAngle <= 24)
		{
			//printf("if (startAngle >= 0 && endAngle <= 24)");
			newPacketFlag = true;
			packetCount = 0;
			scanCount = 0;			
		}

		if (startAngle < 24)
		{
			if (lastStartAngle < 336)
			{

				//printf("if (lastStartAngle < 336)\n");
				newPacketFlag = false;
				packetCount = 0;
				scanCount = 0;
			}
		}
		else
		{
			if (startAngle < lastStartAngle)
			{
				//printf("if (startAngle < lastStartAngle)\n");
				newPacketFlag = false;
				packetCount = 0;
				scanCount = 0;
			}
			else
			{
				if ((startAngle - lastStartAngle) > 40)
				{
					//printf("if ((startAngle - lastStartAngle) > 40)\n");
					newPacketFlag = false;
					packetCount = 0;
					scanCount = 0;
				}
			}
			
		}
		
		lastStartAngle = startAngle;

		if (newPacketFlag)
		{	
			
			angleoffset = 0;
			for (int i = 0; i < scanNum; i++)
			{
				(scanBuff + scanCount)->motorspeed = speed;

				float angle = startAngle + i * angleStep;			
				if (angle >= 165)
					angle -= 165;
				else
					angle = angle + 360 - 165;
				(scanBuff + scanCount)->angle = angle;
				
				(scanBuff + scanCount)->angleoffset = angleoffset;
				(scanBuff + scanCount)->distance = ((recvBuff[16 + i * 2] << 8) | recvBuff[15 + i * 2])*0.001; //20230426
				scanCount++;
			}
			packetCount++;
		}			
	}
	else
	{
		//printf("checksum fail.................................\n");
		RunningStates = CHKSUM_ERROR;
		return RESULT_CONTINUE;
	}

	//printf("packetCount=%d,sampleNumberCount=%d................\n",packetCount,scanCount);

	if (packetCount == MAX_TEETH_NUM)
	{
		count = scanCount;
		return RESULT_OK;
	}
}


result_t TTlidarDriver::_recvPacket(_u8 * scanBuff, _u32 timeout)
{
	int		recvPos = 0;
	_u32	starttime = getms();
	_u8		recvBuff[512];
	_u32	waitTime;
	_u16	paramLen = 0;
	size_t  recvSize;
	size_t  remainSize;
	size_t  pos;
	bool    frameDataRcvFlag = false;
	
	
	char tmp_message[100];
	memset(tmp_message, 0, 100);

	while ((waitTime = getms() - starttime) <= timeout) {

		if (false == frameDataRcvFlag)
		{
			//receive head info
			remainSize = sizeof(FRAME_T) - recvPos;
		}
		else
		{
			//receive data
			remainSize = paramLen + 8 - recvPos;
		}
		recvSize = 0;
		
		int result = _serial->waitfordata(remainSize, timeout, &recvSize);

		if (result == Serial::ANS_DEV_ERR)
		{
			return RESULT_OPERATION_FAIL;
		}

		if ((result == Serial::ANS_TIMEOUT) && (0 == recvSize))
		{			
			printf("[Eror]: Receive data timeout!\r\n");
			RunningStates = RX_DATA_TIMOUT;
			continue;

		}

		if (recvSize > remainSize) 
			recvSize = remainSize;

		_serial->recvdata(recvBuff, recvSize);
		for (pos = 0; pos < recvSize; ++pos)
		{
			_u8 currentByte = recvBuff[pos];

			if (0 == recvPos)
			{
				if (FRAME_HEAD1_FLAG != currentByte)
				{
					frameDataRcvFlag = false;
					continue;
				}
			}
			else if (1 == recvPos)
			{
				if (FRAME_HEAD2_FLAG != currentByte)
				{
					recvPos = 0;
					frameDataRcvFlag = false;
					continue;
				}
			}
			else if (3 == recvPos)
			{
				paramLen = (currentByte << 8) | scanBuff[2];				
				if (paramLen > 300)
				{
					recvPos = 0;
					paramLen = 0;
					frameDataRcvFlag = false;
					continue;
				}				
			}
			else if (6 == recvPos)//
			{
				if (CMD_REPORT_DIST != currentByte)
				{
					recvPos = 0;
					paramLen = 0;
					frameDataRcvFlag = false;
					continue;
				}
				else
					frameDataRcvFlag = true;					
			}

			scanBuff[recvPos++] = currentByte;

			if ((recvPos >= (paramLen + 8))&&(recvPos > sizeof(FRAME_T)))
			{
				RunningStates = WORK_WELL;
				return RESULT_OK;
			}
		}
	}
	return RESULT_OPERATION_TIMEOUT;
}

result_t TTlidarDriver::grabScanData(LIDAR_SCAN_INFO_T * scanBuff, size_t & count, _u32 timeout)
{
    switch (_event.wait(timeout))
    {
		case  Event::EVENT_TIMEOUT:
			count = 0;
			return RESULT_OPERATION_TIMEOUT;
			break;

		case  Event::EVENT_OK:
			{
				 AutoLocker l(_lock);

				size_t size_to_copy = min(count, _scan_buff_count);

				if (FLAG_BUFFER_LEFT == _flag_scan_buffer)
				{
					memcpy(scanBuff, _scan_buff_left, size_to_copy*sizeof(LIDAR_SCAN_INFO_T));
				}
				else if (FLAG_BUFFER_RIGHT == _flag_scan_buffer)
				{
					memcpy(scanBuff, _scan_buff_right, size_to_copy*sizeof(LIDAR_SCAN_INFO_T));
				}

				count = size_to_copy;
				_scan_buff_count = 0;
			}
			return RESULT_OK;
			break;

		default:
			count = 0;
			return RESULT_OPERATION_FAIL;
			break;
    }

}

result_t TTlidarDriver::_sendCommand(_u8 cmd, const void * payload, size_t payloadsize)
{
	_u16 checksum = 0;
	_u8 commBuf[1024] = {0};
	FRAME_T *commFrameTmp = reinterpret_cast<FRAME_T *>(commBuf);
	const _u8 *pPayload = reinterpret_cast<const _u8 *>(payload);

	if (!_is_connected) return RESULT_OPERATION_FAIL;

	commBuf[0] = 0xA5;
	commBuf[1] = 0x5A;
	commBuf[2] = payloadsize;
	commBuf[3] = payloadsize >> 8;
	commBuf[6] = cmd;
	commBuf[7] = cmd >> 8;
	
	if (payloadsize > 0)
	{			
		for(int i = 0; i < payloadsize; i++)
			*(commFrameTmp->paramBuff + i) = *(pPayload + i);
	}

	checksum = _checksum(commBuf, sizeof(FRAME_T) + payloadsize);
	commBuf[4] = checksum;
	commBuf[5] = checksum >> 8;
	
	for (int i = 0; i < (sizeof(FRAME_T)+payloadsize);i++)
		printf("%02x ", commBuf[i]);
	printf("\n");
	_serial->senddata(commBuf, sizeof(FRAME_T)+payloadsize );
	return RESULT_OK;
}


result_t TTlidarDriver::_waitforResponse(FRAME_T * header, _u8 Cmmd,const void * payload, size_t payloadsize, _u32 timeout)
{
	_u8 Data[512];
	int  recvPos = 0,readPos = 0,i;
	_u32 starttime = getms();
	_u8  *headerBuffer = reinterpret_cast<_u8 *>(header);
	_u32 waitTime;
	_u16 framelen,checksumCac, checksumRcv;
	size_t remainSize = sizeof(FRAME_T) + payloadsize + sizeof(_u16);
	size_t recvSize;

	while ((waitTime = getms() - starttime) <= timeout) 
	{
		int result = _serial->waitfordata(remainSize, timeout - waitTime, &recvSize);
		if (result == Serial::ANS_DEV_ERR)
		{
			return RESULT_OPERATION_FAIL;
		}
		else if (result == Serial::ANS_TIMEOUT)
		{
			return RESULT_OPERATION_TIMEOUT;
		}

		if (recvSize > (sizeof(Data) - (recvPos & 0x1FF)))
		{
			_serial->recvdata(&Data[recvPos & 0x1FF], sizeof(Data) - (recvPos & 0x1FF));
			_serial->recvdata(Data,recvSize - sizeof(Data) + (recvPos & 0x1FF));
		}
		else
			_serial->recvdata(&Data[recvPos & 0x1FF], recvSize);

		recvPos += recvSize;
		if ((readPos + remainSize) > recvPos)
			continue;
		else
		{
			for (; readPos < recvPos; readPos++)
			{
				if (Data[readPos & 0x1FF] != FRAME_HEAD1_FLAG)
					continue;
				else
				{
					framelen = (Data[(readPos + 2) & 0x1FF] << 8) | Data[(readPos + 1) & 0x1FF];
					if (framelen > 100)
						continue;

					if ((framelen + 2) > (recvPos - readPos))
						break;

					if ((Data[(readPos + 4) & 0x1FF]&COMM_MASK) != Cmmd)
						continue;

					checksumCac = _cyclechecksum(Data,readPos,framelen,sizeof(Data));
					checksumRcv = (Data[(readPos + framelen + 1) & 0x1FF] << 8) | Data[(readPos + framelen) & 0x1FF];
					if (checksumCac == checksumRcv)
					{
						for (i = 0; i < (payloadsize + sizeof(FRAME_T)); i++)
						{
							*headerBuffer++ = Data[(readPos + i) & 0x1FF];
						}
						return RESULT_OK;
					}
					else
						return RESULT_INVALID_DATA;
				}
			}
		}
	}
	return RESULT_OPERATION_TIMEOUT;
}

uint16_t TTlidarDriver::_cyclechecksum(uint8_t *startByte, _u32 pos,uint16_t numBytes,_u32 buffersize)
{
	uint16_t checksum = 0;                           
	while (numBytes--)
	{
		checksum += *(startByte + (pos%buffersize));     
		pos++;
	}
	return checksum;
}
//
//uint16_t TTlidarDriver::_checksum(uint8_t *startByte, uint16_t numBytes)
//{
//	uint16_t checksum = 0;                           
//	while (numBytes--)
//	{
//		checksum += *startByte++;      
//	}
//	return checksum;
//}


uint16_t TTlidarDriver::_checksum(uint8_t* startByte, uint16_t numBytes)
{
	uint16_t checksum = 0;            
	checksum += *startByte;
	checksum += *(startByte + 1);
	checksum += *(startByte + 2);
	checksum += *(startByte + 3);

	checksum += *(startByte + 6);
	checksum += *(startByte + 7);
	int i=1;
	while (i <= numBytes)
	{	
		checksum += *(startByte + 7 + i);
		i++;
	}
	return checksum;
}


void TTlidarDriver::stopGrabingScanData()
{
    _is_scanning = false;
    _scanthread.join();
	_serial->flush(0);
}

}

