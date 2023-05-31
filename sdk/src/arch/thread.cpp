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

#if defined(_WIN32)
#include "arch/win32/winthread.hpp"
#elif defined(_MACOS)
#include "arch/macOS/thread.hpp"
#elif defined(__GNUC__)
#include "arch/linux/linuxthread.hpp"
#else
#error no threading implemention found for this platform.
#endif

