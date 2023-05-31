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

#define delay(x)   ::Sleep(x)

namespace ttlidar {
    void HPtimer_reset();
    _u32 getHDTimer();
}

#define getms()   ttlidar::getHDTimer()

