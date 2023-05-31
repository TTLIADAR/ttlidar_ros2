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


#ifdef _WIN32

//fake stdint.h for VC only

typedef signed   char     int8_t;
typedef unsigned char     uint8_t;

typedef __int16           int16_t;
typedef unsigned __int16  uint16_t;

typedef __int32           int32_t;
typedef unsigned __int32  uint32_t;

typedef __int64           int64_t;
typedef unsigned __int64  uint64_t;

#else

#include <stdint.h>

#endif

typedef int8_t         _s8;
typedef uint8_t        _u8;

typedef int16_t        _s16;
typedef uint16_t       _u16;

typedef int32_t        _s32;
typedef uint32_t       _u32;

typedef int64_t        _s64;
typedef uint64_t       _u64;

#define __small_endian

#ifndef __GNUC__
#define __attribute__(x)
#endif


#ifdef _AVR_
typedef _u8            word_size_t;
#define THREAD_PROC    
#elif defined (WIN64)
typedef _u64           word_size_t;
#define THREAD_PROC    __stdcall
#elif defined (WIN32)
typedef _u32           word_size_t;
#define THREAD_PROC    __stdcall
#elif defined (__GNUC__)
typedef unsigned long  word_size_t;
#define THREAD_PROC   
#elif defined (__ICCARM__)
typedef _u32            word_size_t;
#define THREAD_PROC  
#endif


typedef uint32_t result_t;

#define RESULT_OK						0
#define RESULT_CONTINUE					1	
#define RESULT_ALREADY_DONE				0x20
#define RESULT_FAIL_FLAG				0x80008000
#define RESULT_INVALID_DATA				(0x0 | RESULT_FAIL_FLAG)
#define RESULT_OPERATION_FAIL			(0x1 | RESULT_FAIL_FLAG)
#define RESULT_OPERATION_TIMEOUT		(0x2 | RESULT_FAIL_FLAG)
#define RESULT_OPERATION_STOP			(0x3 | RESULT_FAIL_FLAG)
#define RESULT_OPERATION_NOT_SUPPORT    (0x4 | RESULT_FAIL_FLAG)
#define RESULT_FORMAT_NOT_SUPPORT		(0x5 | RESULT_FAIL_FLAG)
#define RESULT_INSUFFICIENT_MEMORY		(0x6 | RESULT_FAIL_FLAG)
#define RESULT_RECEIVE_NODE_ERROR		(0x7 | RESULT_FAIL_FLAG)

#define IS_OK(x)						(((x) & RESULT_FAIL_FLAG) == 0 )
#define IS_FAIL(x)						(((x) & RESULT_FAIL_FLAG))

typedef word_size_t (THREAD_PROC * thread_proc_t ) ( void * );


