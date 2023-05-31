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

// TT-Lidar Input Packets

#if defined(_WIN32)
#pragma pack(1)
#endif

#include "tttypes.h"

//----------------------------------------
// protocol macro
//-----------------------------------------
#define FRAME_HEAD1_FLAG				0xA5	//Frame header
#define FRAME_HEAD2_FLAG				0x5A	//Frame header
#define FRAME_PC_TO_DEV					0x00	//attribute frame
#define FRAME_DEV_TO_PC					0x40	//command farme
#define FRAME_ERROR						0x80    //error frame


//-----------------------------------------
#define MAX_TEETH_NUM					15
#define FLAG_BUFFER_LEFT				0x00
#define FLAG_BUFFER_RIGHT				0x01
#define	COMM_MASK						0x3F

//----------------------------------------
// command word  enum
//-----------------------------------------
typedef enum _cmd_code
{
	CMD_SET_WORK_MODE = 0x01,					//
	CMD_SET_MOTOR_SPEED = 0x04,					//
	CMD_READ_DEV_VER = 0x0D,					//read version
	CMD_REPORT_DIST = 0x09,						//report lidar scan info
	CMD_REPORT_DEV_ERROR = 0x16,				//report device error
}CMD_CODE;

//----------------------------------------
// work  mode    enum
//-----------------------------------------
typedef enum _work_mode_
{
	IDEL_MODE = 0x00,
	SCAN_MODE = 0x01,
	RESET_MODE = 0x07,
}WORK_MODE;

//----------------------------------------
// ERROR CODE    enum
//-----------------------------------------
typedef enum _running_state
{
	PORT_RDY = 0,
	WORK_WELL,
	RX_DATA_TIMOUT,
	CHKSUM_ERROR,
	MOTOR_STALL,
}RUNNING_STATE;

//----------------------------------------
// protocol structure
//-----------------------------------------

typedef struct _frame_structure
{
	_u16	framehead;			//frame head
	_u16	paramLen;			//param length
	_u16	checksum;			//
	_u16	command;
	_u8		paramBuff[0];		//param buffer
} __attribute__((packed)) FRAME_T;

//----------------------------------------
// device_info   structure
//-----------------------------------------
typedef struct _lidar_devive_info_t {
	_u8		productsn[26];			
	_u8		softwarever[15];		
} __attribute__((packed)) LIDAR_DEV_INFO_T;



#if defined(_WIN32)
#pragma pack()
#endif
