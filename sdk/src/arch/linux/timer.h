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
#include <unistd.h>

static inline void delay(word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

namespace ttlidar {

_u64 tt_getus();
_u32 tt_getms();

}

#define getms() ttlidar::tt_getms()
