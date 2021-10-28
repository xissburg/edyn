/**
 * Copy-paste-adapt from SDL (Simple DirectMedia Layer).
 */

/*
  Simple DirectMedia Layer
  Copyright (C) 1997-2018 Sam Lantinga <slouken@libsdl.org>

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#include "edyn/time/time.hpp"
#include "edyn/config/config.h"
#include <sys/time.h>
#include <cerrno>

#define HAVE_NANOSLEEP 1
#define HAVE_CLOCK_GETTIME 1

namespace edyn {

#if HAVE_NANOSLEEP || HAVE_CLOCK_GETTIME
#include <time.h>
#endif
#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

/* Use CLOCK_MONOTONIC_RAW, if available, which is not subject to adjustment by NTP */
#if HAVE_CLOCK_GETTIME
#ifdef CLOCK_MONOTONIC_RAW
#define EDYN_MONOTONIC_CLOCK CLOCK_MONOTONIC_RAW
#else
#define EDYN_MONOTONIC_CLOCK CLOCK_MONOTONIC
#endif
#endif

struct time_start_info {
    #if defined(__APPLE__)
    uint64_t start_mach;
    mach_timebase_info_data_t mach_base_info;
    #else
    timespec start_ts;
    #endif
    bool has_monotonic_time {false};
    timeval start_tv;

    time_start_info() {
        /* Set first ticks value */
    #if defined(__APPLE__)
        kern_return_t ret = mach_timebase_info(&mach_base_info);
        if (ret == 0) {
            has_monotonic_time = true;
            start_mach = mach_absolute_time();
        } else
    #elif HAVE_CLOCK_GETTIME
        if (clock_gettime(EDYN_MONOTONIC_CLOCK, &start_ts) == 0) {
            has_monotonic_time = true;
        } else
    #endif
        {
            gettimeofday(&start_tv, nullptr);
        }
    }
};

static const time_start_info info;

uint32_t ticks()
{
    uint32_t ticks;

    if (info.has_monotonic_time) {
#if defined(__APPLE__)
        uint64_t now = mach_absolute_time();
        ticks = (uint32_t)((((now - info.start_mach) * info.mach_base_info.numer) / info.mach_base_info.denom) / 1e6);
#elif HAVE_CLOCK_GETTIME
        struct timespec now;
        clock_gettime(EDYN_MONOTONIC_CLOCK, &now);
        ticks = (now.tv_sec - info.start_ts.tv_sec) * 1e3 + (now.tv_nsec -
                                                 info.start_ts.tv_nsec) / 1e6;
#else
        EDYN_ASSERT(false);
        ticks = 0;
#endif
    } else {
        struct timeval now;

        gettimeofday(&now, nullptr);
        ticks = (uint32_t)((now.tv_sec - info.start_tv.tv_sec) * 1e3 + (now.tv_usec - info.start_tv.tv_usec) / 1e3);
    }
    return (ticks);
}

uint64_t performance_counter() {
    uint64_t ticks;

    if (info.has_monotonic_time) {
#if HAVE_CLOCK_GETTIME
        struct timespec now;

        clock_gettime(EDYN_MONOTONIC_CLOCK, &now);
        ticks = now.tv_sec;
        ticks *= 1e9;
        ticks += now.tv_nsec;
#elif defined(__APPLE__)
        ticks = mach_absolute_time();
#else
        EDYN_ASSERT(false);
        ticks = 0;
#endif
    } else {
        struct timeval now;

        gettimeofday(&now, nullptr);
        ticks = now.tv_sec;
        ticks *= 1e6;
        ticks += now.tv_usec;
    }
    return (ticks);
}

uint64_t performance_frequency()
{
    if (info.has_monotonic_time) {
#if HAVE_CLOCK_GETTIME
        return 1e9;
#elif defined(__APPLE__)
        uint64_t freq = mach_base_info.denom;
        freq *= 1e9;
        freq /= mach_base_info.numer;
        return freq;
#endif
    }

    return 1e6;
}

void delay(uint32_t ms) {
    int was_error;

#if HAVE_NANOSLEEP
    struct timespec elapsed, tv;
#else
    struct timeval tv;
    uint32_t then, now, elapsed;
#endif

    /* Set the timeout interval */
#if HAVE_NANOSLEEP
    elapsed.tv_sec = ms / 1e3;
    elapsed.tv_nsec = (ms % 1000) * 1e6;
#else
    then = ticks();
#endif
    do {
        errno = 0;

#if HAVE_NANOSLEEP
        tv.tv_sec = elapsed.tv_sec;
        tv.tv_nsec = elapsed.tv_nsec;
        was_error = nanosleep(&tv, &elapsed);
#else
        /* Calculate the time interval left (in case of interrupt) */
        now = ticks();
        elapsed = (now - then);
        then = now;
        if (elapsed >= ms) {
            break;
        }
        ms -= elapsed;
        tv.tv_sec = ms / 1000;
        tv.tv_usec = (ms % 1000) * 1000;

        was_error = select(0, NULL, NULL, NULL, &tv);
#endif /* HAVE_NANOSLEEP */
    } while (was_error && (errno == EINTR));
}

}
