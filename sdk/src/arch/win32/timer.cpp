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

#include "ttlidar.h"
#include "timer.h"
#include <mmsystem.h>
#pragma comment(lib, "Winmm.lib")


namespace ttlidar {


#define BEGIN_STATIC_CODE( _blockname_ ) \
    static class _static_code_##_blockname_ {   \
    public:     \
        _static_code_##_blockname_ () 


#define END_STATIC_CODE( _blockname_ ) \
    }   _instance_##_blockname_;



static LARGE_INTEGER _current_freq;

void HPtimer_reset()
{
    BOOL ans=QueryPerformanceFrequency(&_current_freq);
    _current_freq.QuadPart/=1000;
}

_u32 getHDTimer()
{
    LARGE_INTEGER current;
    QueryPerformanceCounter(&current);

    return (_u32)(current.QuadPart/_current_freq.QuadPart);
}

BEGIN_STATIC_CODE(timer_cailb)
{
    HPtimer_reset();
}END_STATIC_CODE(timer_cailb)

}
