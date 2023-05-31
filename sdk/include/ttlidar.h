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


#if defined(_WIN32)
#include "arch\win32\arch_win32.h"
#elif defined(_MACOS)
#include "arch/macOS/arch_macOS.h"
#elif defined(__GNUC__)
#include "arch/linux/arch_linux.h"
#else
#error "unsupported target"
#endif

#include <time.h>
#include "serial.h"
#include "thread.h"
#include "locker.h"
#include "event.h"

