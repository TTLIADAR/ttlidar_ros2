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

#define CLASS_THREAD(c , x ) 	ttlidar::Thread::create_member<c, &c::x>(this )
	
namespace ttlidar{ 

class Thread
{
public:
    enum priority_level_t
	{
		PRIORITY_REALTIME = 0,
		PRIORITY_HIGH     = 1,
		PRIORITY_NORMAL   = 2,
		PRIORITY_LOW      = 3,
		PRIORITY_IDLE     = 4,
	};

    template <class T, result_t (T::*PROC)(void)>
    static Thread create_member(T * pthis)
    {
		return create(_thread_thunk<T,PROC>, pthis);
	}

	template <class T, result_t (T::*PROC)(void) >
	static word_size_t THREAD_PROC _thread_thunk(void * data)
	{
		return (static_cast<T *>(data)->*PROC)();
	}
	static Thread create(thread_proc_t proc, void * data = NULL );

public:
    ~Thread() { }
    Thread():  _data(NULL),_func(NULL),_handle(0)  {}
    word_size_t getHandle(){ return _handle;}
    result_t terminate();
    void *getData() { return _data;}
    result_t join(unsigned long timeout = -1);
	result_t setPriority( priority_level_t p);
	priority_level_t getPriority();

    bool operator== ( const Thread & right) { return this->_handle == right._handle; }
protected:
    Thread( thread_proc_t proc, void * data ): _data(data),_func(proc), _handle(0)  {}
    void * _data;
    thread_proc_t _func;
    word_size_t _handle;
};

}

