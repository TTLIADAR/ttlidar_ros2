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

#include "arch/win32/arch_win32.h"
#include <process.h>

namespace ttlidar {


Thread Thread::create(thread_proc_t proc, void * data)
{
    Thread newborn(proc, data);

    newborn._handle = (word_size_t)( 
        _beginthreadex(NULL, 0, (unsigned int (_stdcall * )( void * ))proc,
                        data, 0, NULL));
    return newborn;
}

result_t Thread::terminate()
{
    if (!this->_handle) return RESULT_OK;
    if (TerminateThread( reinterpret_cast<HANDLE>(this->_handle), -1))
    {
        CloseHandle(reinterpret_cast<HANDLE>(this->_handle));
        this->_handle = NULL;
        return RESULT_OK;
    }else
    {
        return RESULT_OPERATION_FAIL;
    }
}

result_t Thread::setPriority( priority_level_t p)
{
	if (!this->_handle) return RESULT_OPERATION_FAIL;

	int win_priority =  THREAD_PRIORITY_NORMAL;
	switch(p)
	{
	case PRIORITY_REALTIME:
		win_priority = THREAD_PRIORITY_TIME_CRITICAL;
		break;
	case PRIORITY_HIGH:
		win_priority = THREAD_PRIORITY_HIGHEST;
		break;
	case PRIORITY_NORMAL:
		win_priority = THREAD_PRIORITY_NORMAL;
		break;
	case PRIORITY_LOW:
		win_priority = THREAD_PRIORITY_LOWEST;
		break;
	case PRIORITY_IDLE:
		win_priority = THREAD_PRIORITY_IDLE;
		break;
	}

	if (SetThreadPriority(reinterpret_cast<HANDLE>(this->_handle), win_priority))
	{
		return RESULT_OK;
	}
	return RESULT_OPERATION_FAIL;
}

Thread::priority_level_t Thread::getPriority()
{
	if (!this->_handle) return PRIORITY_NORMAL;
	int win_priority =  ::GetThreadPriority(reinterpret_cast<HANDLE>(this->_handle));
	
	if (win_priority == THREAD_PRIORITY_ERROR_RETURN)
	{
		return PRIORITY_NORMAL;
	}

	if (win_priority >= THREAD_PRIORITY_TIME_CRITICAL )
	{
		return PRIORITY_REALTIME;
	}
	else if (win_priority<THREAD_PRIORITY_TIME_CRITICAL && win_priority>=THREAD_PRIORITY_ABOVE_NORMAL)
	{	
		return PRIORITY_HIGH;
	}
	else if (win_priority<THREAD_PRIORITY_ABOVE_NORMAL && win_priority>THREAD_PRIORITY_BELOW_NORMAL)
	{
		return PRIORITY_NORMAL;
	}else if (win_priority<=THREAD_PRIORITY_BELOW_NORMAL && win_priority>THREAD_PRIORITY_IDLE)
	{
		return PRIORITY_LOW;
	}else if (win_priority<=THREAD_PRIORITY_IDLE)
	{
		return PRIORITY_IDLE;
	}
	return PRIORITY_NORMAL;
}

result_t Thread::join(unsigned long timeout)
{
    if (!this->_handle) return RESULT_OK;
    switch ( WaitForSingleObject(reinterpret_cast<HANDLE>(this->_handle), timeout))
    {
    case WAIT_OBJECT_0:
        CloseHandle(reinterpret_cast<HANDLE>(this->_handle));
        this->_handle = NULL;
        return RESULT_OK;
    case WAIT_ABANDONED:
        return RESULT_OPERATION_FAIL;
    case WAIT_TIMEOUT:
        return RESULT_OPERATION_TIMEOUT;
    }

    return RESULT_OK;
}

}
